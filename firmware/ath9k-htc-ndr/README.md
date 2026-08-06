# ath9k-htc-ndr — the named-data-radio MAC in Wi-Fi firmware (AR9271)

Patches to [`qca/open-ath9k-htc-firmware`](https://github.com/qca/open-ath9k-htc-firmware) that put
Tier-0 name filtering and (next) hardware-scheduled TX **on the dongle**, where the design says they
belong.

## Why this part, and not any other radio we own

`named-filter-mac-redesign.md` names two things as unreachable on commodity Wi-Fi. The AR9271 is the
first Wi-Fi part in this stack where both are reachable, because we get to write its firmware.

**§8.2 — "on commodity Wi-Fi we get the CPU win but *not* the wakeup win."** In monitor mode the NIC
delivers everything; the USB transfer and the host wakeup have already happened before any filter of
ours can run. On the AR9271, `ath_tgt_rx_tasklet()` (`target_firmware/wlan/if_ath.c:478`) runs on the
dongle's Xtensa core, and `HTC_SendMsg(..., RX_ENDPOINT_ID, skb)` at line 514 is the exact USB
handoff. A frame rejected before that line never crosses USB. This is NDN-NIC's actual result, on
Wi-Fi.

**§8.5 item 1 — "no hardware-scheduled TX. This is the biggest loss — larger than any filter
improvement."** The AR9271's MAC has the primitives, and the open firmware exposes them:

| register | address | what it gives us |
|---|---|---|
| `AR_TSF_L32` / `AR_TSF_U32` | `0x804c` / `0x8050` | hardware TSF; already read by `ar5416GetTsf64()` |
| `AR_QUIET1` / `AR_QUIET2` | `0x80fc` / `0x8100` | next-quiet TSF, period, duration — a **hardware TX gate** |
| `AR_D_GBL_IFS_MISC` bit 27 | `0x10f0` | `FORCE_XMIT_SLOT_BOUND` — force TX on slot boundaries |
| `AR_D_GBL_IFS_MISC` bit 28 | `0x10f0` | `IGNORE_BACKOFF` — already used by `ar5416AbortTxDma()`, so the access path is proven in-tree |
| `AR_D_GBL_IFS_SLOT` | `0x1070` | programmable slot duration |
| `AR_NAV` | `0x8084` | current NAV — makes §7 validation item 4 directly *readable* on the receiver rather than inferred from behaviour |

That replaces "a host-side sleep plus EDCCA-off", which is why §8.5 says guard bands must be
milliseconds. The `AR_QUIET1/2` field definitions are present but `#if 0`'d in `ar5416reg.h` — the
open firmware never used them.

Caveats, stated up front: AR9271 is **2.4 GHz only, 1×1, HT20, USB 2.0**. It is a substrate for the
MAC, not a throughput radio.

## Why the firmware is C and not Rust

`rustc` 1.96 does have Xtensa targets — but all three are Espressif core configs
(`xtensa-esp32{,s2,s3}-none-elf`), and the backend ships with no precompiled `core` (needs
`-Zbuild-std`). Xtensa is a **configurable** ISA: the core config *is* the instruction set and ABI.
MAGPIE (the AR9271's core) is a different configuration — from the toolchain's own
`local/patches/gcc.patch`: windowed registers, `XSHAL_USE_ABSOLUTE_LITERALS=1`, `MUL32` **and**
`MUL32_HIGH`, **no `DIV32`**, no MAC16, and **no I-cache or D-cache at all**. There is no `magpie`
CPU in LLVM's Xtensa backend.

On top of that, the RAM firmware is a **ROM patch**: it links against routines at fixed addresses in
on-chip ROM (`rom-addrs-magpie.ld`, `magpie_fw_dev/target/rompatch/`) under the windowed ABI, so it
could not be pure Rust regardless.

A miscompiling toolchain is the worst possible foundation for a project whose method is
[measurement over reasoning](../../../ndn-ext). So: **C on the dongle, kept small and surgical; Rust
on the host.** The firmware needs only the AND-mask test and the TSF/QUIET arming — everything above
that (mask computation, lease scheduling, cognition) belongs in the host driver.

## Layout

```
src/ndr_tier0.h/.c    Tier-0 prefix-set Bloom filter — C port of lr2021-nrf54l15-rs/src/tier0.rs
src/ndr_filter.h/.c   receive policy: registered masks, enable flag, counters
tools/gen_vectors.rs  emits reference vectors from the Rust (normative) implementation
tools/ndr_tier0_selftest.c   cross-checks the C against those vectors
```

### The cross-check is not optional

The firmware and the nRF54L15 testbed must agree bit-for-bit on the wire. A divergence raises no
error anywhere — a frame simply stops matching. Run it after touching either implementation:

```sh
cd tools
rustc -O --edition 2021 gen_vectors.rs -o /tmp/gen_vectors && /tmp/gen_vectors > /tmp/vectors.h
cc -DNDR_HOST_TEST -I../src -I/tmp -O2 -Wall ndr_tier0_selftest.c ../src/ndr_tier0.c -o /tmp/selftest
/tmp/selftest        # PASS: 32 vectors, 0 failures
```

It has already earned its keep — see below.

### ★ It found a real false-negative bug in the design

`for_each_prefix` stops at `MAX_DEPTH = 8`, so a sender transmitting `/a/b/c/d/e/f/g/h/i` inserts at
deepest `/a/b/c/d/e/f/g` — **seven** components. A receiver registered on `/a/b/c/d/e/f/g/h` built
its mask over bits the sender never set, and **dropped a frame that genuinely was under its
prefix**. That is a true false negative, the one failure the design forbids.

The measured "zero false negatives at every depth" could not see it: the check only ever queried
prefixes that had been *inserted*, which makes the property tautological.

Fix (`ndr_clamp_prefix`, mirrored into `tier0.rs::clamp_prefix`): a registration deeper than the cap
is clamped to its 7-component ancestor. That costs extra false positives and no false negatives, and
Tier 1/2 does the exact match — which is what "deeper matching is left to the software tier" was
always supposed to mean, and it only works if the frame survives Tier 0 to reach that tier.

## Build

Needs an `xtensa-elf` cross toolchain, built from source by the upstream `Makefile`
(gmp/mpfr/mpc/binutils-2.35/gcc-10.2). **nixpkgs 25.05 does not package this firmware** — checked.

Two NixOS-specific fixes are required and are not upstream's fault:

1. **MPFR's test suite fails on `tsprintf`** (182/183 pass; it is glibc-version-sensitive and
   unrelated to the arithmetic GCC uses). Drop the checks: `sed -i 's/$(MAKE) check && //g' Makefile`.
2. **GCC 10.2 will not build under the NixOS gcc wrapper's hardening** — `-Werror=format-security`
   trips on `libcpp/macro.c:183`. Export `NIX_HARDENING_ENABLE=""` before building.

See `../../../../scratchpad/build-ath9k-fw.sh` for the exact working invocation.

## Loading it — the AR9271 is unbrickable

The firmware lives in **RAM and is downloaded over USB at every probe**. There is no flash to
corrupt: a bad build means a failed probe, and recovery is `unbind`/`bind` or a replug. Compared to
the MM6108 (repaging faulted the chip) and the GD32 (a one-shot SWD window), this is an unusually
forgiving target — iterate freely.

The loader path is already overridden on o5p-1 for the Morse work:

The name the kernel actually requests is **`ath9k_htc/htc_9271-1.4.0.fw`** — a subdirectory and a
version, *not* `htc_9271.fw`. Mainline builds the versioned name and only falls back to the legacy
one; `modinfo ath9k_htc` is authoritative.

```sh
cat /sys/module/firmware_class/parameters/path     # -> /tmp/fwpatch
mkdir -p /tmp/fwpatch/ath9k_htc                    # additive; does not disturb /tmp/fwpatch/morse
cp target_firmware/htc_9271.fw /tmp/fwpatch/ath9k_htc/htc_9271-1.4.0.fw
echo -n 2-1.2:1.0 | sudo tee /sys/bus/usb/drivers/ath9k_htc/unbind
echo -n 2-1.2:1.0 | sudo tee /sys/bus/usb/drivers/ath9k_htc/bind
```

No NixOS rebuild needed. To revert, delete the file and rebind — the loader falls back to the stock
image in the nix store. Confirm which image actually loaded by md5, not by assuming: the driver logs
`Transferred FW: ..., size: N`, and stock and ours differ.

## Milestones

- **M0 — DONE (2026-08-06).** Toolchain built; the patched firmware builds for both targets with
  `ndr_tier0.c`/`ndr_filter.c` linked in, and **runs on the AR9271 on o5p-1**: `Transferred FW:
  ath9k_htc/htc_9271-1.4.0.fw, size: 51008`, `HTC initialized with 33 credits`, `FW Version: 1.4`,
  `Atheros AR9271 Rev:1`. Loaded-file md5 matches our build and differs from stock, so this is not
  a stock-image false positive. `ndr_cfg.enabled` is 0, so behaviour is stock — which is the point.
  *This must come first: `ndr_cfg.enabled` defaults to 0 so the patched build is behaviourally
  identical to stock. If the filtered and unfiltered builds differ in any way other than the filter,
  the measurement is worthless.*
- **M1** — Tier-0 filter on-chip. Measure frames-to-host, USB bytes, host CPU with the filter off vs
  on. Closes §7 validation item 2 and the Wi-Fi half of open task #43.
- **M2 — DONE (2026-08-06).** `WMI_ACCESS_MEMORY_CMDID` implemented (`src/ndr_mem.c`) — upstream
  declared the struct but its WLAN-build handler was `adf_os_assert(0)`, so the command asserted
  rather than worked. Host side is `ndn-radio-drivers/src/ath9k_htc.rs` +
  `examples/ath9k_ndr_stats.rs`. Verified on hardware: `GET_FW_VERSION -> 1.4`, a `0xdeadbeef`
  sentinel written to `ndr_stats.short_frame` and read back byte-exact, and **three consecutive
  runs with no replug**. Addresses come from the linker (`xtensa-elf-nm build/k2/fw.elf | grep
  ndr_`), never hardcoded.

- **M3** — hardware-scheduled TX: arm `AR_QUIET1/2` off the TSF. Compare the achievable guard band
  against the ~40–60 µs measured on the nRF54L15 testbed and the milliseconds we need today.
- **M4** — NAV. Write the lease into Duration/ID and read `AR_NAV` on a second radio. This is §7
  item 4, it is a one-afternoon measurement, and it gates a design choice — do it early.
