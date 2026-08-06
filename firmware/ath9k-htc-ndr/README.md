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

## ★ What caps Tier-0, and why ACK/CTS survive it

The frames that get through Tier-0 are the **neighbours'** 802.11 handshake, not ours — in a
capture our injector's address appears zero times and the RAs are other (MAC-randomised) clients.
Every unicast data frame is ACKed, and RTS/CTS + CTS-to-self is 11g/n protection.

They survive **structurally**: an ACK/CTS is 14 bytes carrying exactly one address (`RA:`), while
Tier-0 lives in `addr1‖addr2` and needs 16. So they hit the `len < 16` rule, which passes on
purpose — dropping what cannot be evaluated is how a false-positive-only filter acquires false
negatives.

That puts a ceiling on the headline. M1's channel was management-heavy (2262 frames: 1306 beacons,
507 probe responses, 232 control), all long and filterable. On a channel carrying real **data**,
every unaggregated data frame is followed by an ACK, so control frames approach ~50% of all frames
and Tier-0's **frame** rejection falls toward ~50%. A-MPDU/Block-Ack softens this (one BA per
aggregate). **Byte rejection stays ~99% either way**, since an ACK is 14 B against data frames up
to 1500 B.

So report both: the bandwidth/USB-byte win is robust, while the **per-frame wakeup win — the one
§8.2 actually prizes — is capped by the ambient control-frame fraction**. It is also a concrete
argument for the custom-PHY/802.11ba direction in §8.5, where our MAC emits no ACKs at all.

## M3b — on-air boundary-placement error, measured with a second radio

Two AR9271s: o5p-1 transmits saturated with the quiet schedule armed (period 100 TU, duration
50 TU, FW 1.50 verified), minidronesys-05 receives in monitor mode and timestamps every frame with
`radiotap.mactime` (hardware TSF, ~1 µs).

The **trailing** edge of each quiet window is the observable: the TX queue is already saturated, so
the MAC transmits the instant it is allowed. The leading edge is not usable — it is quantised by the
injector's ~1078 µs inter-frame spacing, and the AR9271 caps monitor injection at ~950 frames/s
regardless of PHY rate (54 Mbit/s is no faster than 1 Mbit/s) or host parallelism (8 processes are
no faster than 1), so that spacing cannot be reduced.

| | ch1 (168 ambient f/s) | **ch13 (9 ambient f/s)** |
|---|---|---|
| boundaries observed | 297 | 297, **all exactly one period apart** |
| mean interval | 102401.0 µs | **102399.2 µs** (configured 102400) |
| **mean period error** | +1.0 µs | **−0.8 µs** |
| per-boundary jitter (sd) | 453 µs | **173 µs** |
| median deviation | 60.0 µs | **51.8 µs** |
| p90 / max | 843 / 3396 µs | **225 / 1288 µs** |

**The hardware honours the TSF schedule to under a microsecond** — the mean period error is −0.8 µs
across a 102.4 ms period, over 297 consecutive windows with none missed or slipped.

**Median boundary error is 52 µs**, which puts this in the same class as the nRF54L15 testbed's
~40–60 µs scheduled-TX guard, and 20–100× better than the millisecond-scale host-sleep guard §8.5
describes. The design's "guard bands must be milliseconds" is a property of the *host-side*
approximation, not of this hardware.

### With backoff removed: ~1 µs

Disabling the random backoff (`AR_D_GBL_IFS_MISC.IGNORE_BACKOFF`, build flag `NDR_NO_BACKOFF`)
removes the CSMA component and exposes the hardware:

| | with backoff (FW 1.50) | **no backoff (FW 1.55)** |
|---|---|---|
| mean period error | −0.8 µs | **+0.1 µs** |
| **median deviation** | 51.8 µs | **1.1 µs** |
| p90 | 224.8 µs | **3.1 µs** |
| p99 / max | 1218 / 1288 µs | 1001 / 1002 µs |
| duration sd | 0.510 ms | **0.192 ms** |

**The AR9271 places its transmit boundary within ~1 µs of the TSF schedule.** `radiotap.mactime`
has 1 µs granularity, so at a 1.1 µs median this measurement is now **resolution-limited, not
hardware-limited** — the true figure may be sub-microsecond and this method cannot say.

The residual ~1 ms p99 is the remaining ambient traffic: `IGNORE_BACKOFF` removes the random
backoff but **not carrier sense**, so a neighbour's frame already on air at the boundary still
defers us. At 9 ambient frames/s that hits ~1% of boundaries.

**What this means for the design.** §8.5 treats sub-millisecond base slots as unreachable on
commodity Wi-Fi. They are not, on this part: slot *boundaries* place to the microsecond, and only
slot *lengths* quantise to 1 TU (the duration field). A named airtime lease with millisecond-scale
base slots is implementable on hardware costing about £15 — the "guards must be milliseconds"
constraint was always the host-side sleep, never the radio.

⚠ `IGNORE_BACKOFF` is a **measurement mode**, not a deployment default — a node ignoring backoff is
antisocial on a shared channel. Though note the lease design makes it less reckless than it sounds:
*inside* a granted lease the node is meant to own the medium, so backoff is precisely the thing the
lease replaces.

⚠ **52 µs (with backoff) is a ceiling, not the hardware's floor.** The measurement still includes CSMA acquisition:
after a quiet window ends the MAC must still do DIFS plus a random backoff before its first frame.
That is also what the ch1→ch13 comparison isolates — the *tail* is contention (p90 843→225 µs) while
the *median* barely moves (60→52 µs). To measure below this, disable backoff with
`AR_D_GBL_IFS_MISC.IGNORE_BACKOFF`, which this part already exposes.

Quiet duration reads 52.82 ms against 51.20 configured; the ~1.6 ms excess is the leading edge being
known only to the last frame before the window (~1078 µs) plus trailing-edge contention.

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

- **M3 — DONE (2026-08-06).** Hardware-scheduled TX, measured: `noquiet 879 f/s → quiet armed
  20 f/s → noquiet 898 f/s`. **97.7% of transmit opportunity removed by the MAC itself**, no host in
  the loop, fully restored when disarmed. §8.5's "biggest loss" answered on real hardware.

  The mechanism is the **generic-timer block, not `AR_QUIET1/2`** (which are inert here — bit 16 of
  `AR_QUIET1` never reads back, and their field defines are `#if 0`'d upstream, which was the hint):

  ```
  duration -> AR_QUIET2            period -> AR_QUIET_PERIOD      (0x8238)
  start    -> AR_NEXT_QUIET_TIMER (0x8218)
  enable   -> AR_TIMER_MODE (0x8240) bit AR_QUIET_TIMER_EN (0x40)
  ```

  These are full 32-bit TSF/µs registers rather than 16-bit TU fields — which is also what makes
  sub-millisecond base slots expressible at all (1 TU = 1024 µs is not).

  **Calibrated (2026-08-06).** Period fixed at 100 TU, duration swept, injection rate measured:

  | duration | frames/s | rate / baseline | ideal `1 − d/100` |
  |---|---|---|---|
  | 0 (control) | 809 / 868 | 1.000 | 1.00 |
  | 10 TU | 771 | 0.919 | 0.90 |
  | 25 TU | 662 | 0.789 | 0.75 |
  | 50 TU | 436 | 0.520 | 0.50 |
  | 75 TU | 239 | 0.285 | 0.25 |
  | 90 TU | 94 | 0.112 | 0.10 |

  Linear, slope **−8.27 f/s per TU** against an ideal −8.39: **one configured TU of quiet removes
  0.986 TU of airtime.** So the **duration field is in TU (1024 µs), exactly as `ar5416reg.h`
  documents** — the earlier "~98% when 50% was asked for" was entirely a software bug of mine
  (re-arming from the receive path restarted the window ~100×/s), not a hardware property.

  **Mixed units, and the asymmetry matters.** `AR_NEXT_QUIET_TIMER` and `AR_QUIET_PERIOD` are 32-bit
  and **microsecond**-granular (writing `period_TU × 1024` there produces a 102.4 ms period);
  `AR_QUIET2`'s duration is **TU**-granular. So a lease *boundary* can be placed to the microsecond
  while its *length* quantises to ~1 ms. For the design that means start times can be slot-accurate
  today, and only the lease length is coarse.

  The residual (measured duty runs 1–3.5 points under configured) is consistent with a frame already
  in flight completing across the quiet boundary — at ~1.2 ms/frame that is ≤1.2% of the period —
  and is within the baseline's own ±3.5% spread. **Not resolvable below ~1 TU with this method**,
  since a 1 TU duty is ~1% against that noise.

- **M4** — NAV. Write the lease into Duration/ID and read `AR_NAV` on a second radio. This is §7
  item 4, it is a one-afternoon measurement, and it gates a design choice — do it early.
