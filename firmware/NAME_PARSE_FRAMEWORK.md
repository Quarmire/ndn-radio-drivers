# In-Firmware Name-Parse Performance Framework

> ## ✅ OUTCOME (2026-09-08): this experiment settled the design — the in-frame name filter is RETIRED.
>
> The arbiter returned a **device-dependent** verdict, and that verdict is now the design of record (`NDR_MAC_SPEC.md`):
> - **ESP32-C5** (240 MHz RISC-V): fused parse+decision **2.2–10.0 µs**, ~8–35× above line rate ⇒ parse everywhere; a MAC-field name is redundant.
> - **AR9271** (40 MHz Xtensa, no cache): **23.4–97.7 µs**, ~0.8–1.8× — falls behind line rate at MCS4+ (fails the 4× bar at every rate).
>
> So there is **no single answer**, and the design stopped looking for one: **relevance = parse the name, off-host where the radio keeps up (C5, LoRa MCU), host-fallback where it does not (AR9271 at high MCS, commodity monitor-mode).** The pre-digested in-frame field the framework asked about is *not* built — a constrained part simply forgoes the off-host optimization and leans on the host, rather than the whole fleet paying wire bytes for the slowest part's benefit. The parser and both benches below are kept: they are the measurement instrument and the shippable off-host fast path. Read the "Goal / Why it matters" framing below as the *question*; the box above is the *answer*.

**Goal.** Establish whether a radio's own firmware can parse an NDN name and reach a
forwarding/filter decision **above line rate, with headroom** — fast enough that the parse is a small
fraction of the inter-frame gap, leaving the rest for the decision, scheduling, and a security tag.

**Why it matters.** If in-firmware parse clears line rate with headroom on the *hardest* target
(AR9271), then carrying name/prefix info in MAC header bits is redundant with the NDN TLV the frame
already holds — the radio can just read the name. If it *can't* clear it on the AR9271, that is the one
honest place a pre-digested in-frame field earns its keep. This experiment is the arbiter, not another
false-positive sweep.

Destination in the repo: `ndn-radio-drivers/firmware/NAME_PARSE_FRAMEWORK.md`; the parser goes at
`ndn-radio-drivers/firmware/ath9k-htc-ndr/src/ndr_parse.{c,h}` (compiled by both AR9271 and ESP32-C5).

---

## 1. The metric — parse budget & headroom

Line rate here = the rate the PHY hands MPDUs to firmware (not the host-delivery rate, which is
USB-bound and irrelevant to an in-firmware loop). The budget per frame is the **inter-MPDU gap**:

```
T_gap = 1 / MPDU_rate          headroom  H = T_gap / T_parse
PASS when  H >= 4  (parse <= 25% of the gap)  on the WORST-CASE (p99) frame, on the AR9271, top verified MCS.
```

- Report **min / median / p99** of `T_parse`, plus `H` against each MCS's inter-frame gap.
- **No CPU-MHz figure is required for the headroom result**: both `T_gap` and `T_parse` are measured in
  us (AR9271 TSF) or derived in us (C5 cycle counter / 240). MHz is needed only to *also* report
  cycles/byte, and the AR9271 clock lives in an upstream file not on this machine (grab on the build host).
- Small named frames are **overhead-bound**, so `MPDU_rate` is roughly flat across MCS (~a few k/s); the
  gap, not the data-symbol time, is the budget. The harness will pin the actual number per target.

---

## 2. Targets & measurement mechanics (from the environment maps)

### AR9271 - the hard target (`firmware/ath9k-htc-ndr/`)
- **Core:** MAGPIE / Tensilica **Xtensa, big-endian, windowed regs, NO DIV32, NO I/D-cache.**
  => parser must be **division/modulo-free** (masks & shifts only), **single-pass & sequential**
  (every byte is a real RAM touch), and endian-aware (BE is native = convenient, VAR-NUMBER is BE).
- **Timer:** `AR_TSF_L32 = 0x804c`, free-running **1 us** (MMIO `ioread32_mac`). Sub-us `CCOUNT` exists
  but is unused; us TSF needs zero new plumbing => **loop N iterations, divide on the host** (no on-chip div).
- **RX hook:** `ndr_rx_accept(const u8 *data, u32 len)` in `src/ndr_filter.c:40`, called from
  `ath_tgt_rx_tasklet()` **before the HTC/USB handoff** - whole frame in RAM, once per MPDU, pre-USB.
  Same point the current Blur filter runs (`ndr_filter_from_hdr`, `ndr_tier0.c:232`) => clean A/B.
- **Result exfil:** add fields to the `ndr_stats` struct (`ndr_filter.h:51`), `xtensa-elf-gcc-nm` its
  address per build, read via `WMI_ACCESS_MEMORY` (`ath9k_htc.rs:1075` `read_target_u32s`) - the proven path.
- **Build/flash:** `make -C build/k2` => `htc_9271.fw` => `/tmp/fwpatch/ath9k_htc/htc_9271-1.4.0.fw` =>
  unbind/bind (RAM-download, unbrickable); `modprobe -r ath9k_htc` first for a cold device; verify by md5.
- **Line rate:** HT20 MCS0-7 = 6.5-65 Mb/s (MCS0-4 on-air-verified). ~128 B frame ~= 200-360 us airtime
  => ~2.8-5 k MPDU/s. (TX inject ~950 f/s, RX-to-host ~85 f/s are USB ceilings - do NOT size against them.)

### ESP32-C5 - the roomy target (`firmware/esp32c5-ndn/`)
- **Core:** RISC-V single core, **240 MHz** (240 cyc/us).
- **Timer:** `esp_cpu_get_cycle_count()` (uint32, wraps ~17.9 s) - available, **currently unused: validate
  it counts at full clock first**. `esp_timer_get_time()` (us) is too coarse for a single parse => loop.
- **RX hook:** payload/NDN-TLV begins at **`f[32]`** (24-B 802.11 + 8-B LLC/SNAP, ethertype `0x8624`).
  ISR `rx_cb()` (`ndn_radio.c:171`) is ISR-context; run the **bench in `serial_tx_task()` task context
  (`:613`)** over a fixed buffer. Current Blur call is `name_admits(f)` (`:200`) - A/B against it.
- **Result exfil:** logging is compiled out (`LOG_DEFAULT_LEVEL_NONE`) => emit via the existing
  `send_framed(ty,...)` framer (`:234`) with a new `T_` type, read on host. Do not assert RTS/DTR on open.
- **Build/flash:** `idf.py build && idf.py -p <port> flash`; USB-Serial-JTAG.

---

## 3. On-air framing - frame byte 0 -> Name TLV (the walk)

NDNLPv2 LpPacket wrapping Interest/Data (`ndn.rs:24-35`, host `mac/name.rs:16`):

```
(Wi-Fi: skip 24-B 802.11 hdr + 8-B LLC/SNAP -> start at f[32])
[0xF5 len ...]?    optional GCS body-prefix TLV (LoRa bearer only)     strip_body_prefix
0x64 LpPacket
   0x52 FragIndex   non-zero => continuation, no Name -> bail          (single-fragment fast path)
   0x50 Fragment
      0x05|0x06 Interest|Data
         0x07 Name
            0x08 comp, 0x08 comp, ...                                  <- roll hash here, LPM per boundary
```
A bare network packet (`0x05`/`0x06` at byte 0) is also valid and handled directly.

**VAR-NUMBER** (`read_varnum` `ndn.rs:116`, host `read_varu64` `ndn-tlv/lib.rs:26`): `<253` = 1 byte;
`253` = u16 BE; `254` = u32 BE; `255` = u64 BE. Host **rejects non-minimal**, firmware **accepts** -
the framework fixes a policy: **reject-early on malformed/oversized, accept non-minimal on the RX fast
path** (cheaper; a non-minimal encoding only hashes to a different name, never a wrong delivery - but
record the choice; it is a link-hardening decision).

---

## 4. The parser design - division-free, single-pass, fused

Today's pipeline is **parse -> build `/`-name -> split prefix set -> SipHash each prefix** - that re-reads
the name bytes O(depth) times and hashes O(depth^2) component-bytes. The framework's parser is one pass:

1. **Walk to the Name** with bounded, `checked`-guarded reads (port the LoRa `ndn.rs` early-exit +
   fixed `[u8; NAME_MAX=96]` scratch model to C; no heap).
2. **Roll the hash across components**, snapshotting at each `/` boundary, and **test the longest-prefix
   membership against the served-prefix set at each boundary** (the `any_prefix_in` structure,
   `ndn.rs:351`, but *fused into the walk* instead of a second pass). Rolling **FNV-1a-64** is the natural
   choice for the receiver's own forward decision - it is trivially incremental (prefix i continues from
   prefix i-1), single-pass, division-free. (Keyed SipHash `ndr_siphash24`/`ndr_name_hash` `ndr_tier0.c:47`
   is only needed to *verify a wire filter's unforgeability*; a receiver hashing its own parsed name for a
   forward decision doesn't need it. Snapshot-per-prefix SipHash is possible via state (v0..v3) snapshots
   at boundaries finalized on a copy - noted, not the default.)
3. **Reject-early** on structure (length past frame end, depth > MAX_DEPTH, oversized component) before any
   deep work - same guard doubles as the reject-early floor.

Constraints (AR9271-driven, both targets): no `/`,`%`; masks for any power-of-2 reduce; sequential forward
reads only, no revisit; fixed scratch; BE-native multi-byte reads.

Implemented in `ndr_parse.c/.h` (this directory): `ndr_walk_to_name`, `ndr_name_admits` (the fused hot
path), `ndr_parse_name` (verification render + full hash), `ndr_fnv1a64`.

---

## 5. Workload (corpus)

Host-built, embedded as a C byte-array in the harness (parse a fixed buffer in a loop - isolates parse
cost from any air/USB ceiling):
- **Realistic:** NDNLPv2-wrapped Interest & Data, name depth 1->12, component lengths 1->~32 B, mixed apps.
- **Edge cases (must stay above line rate AND reject-early):** max-width VAR-NUMBER (255/u64 length),
  deeply nested / oversized TLV, truncated frames, non-minimal encodings, depth > MAX_DEPTH.
- Reuse the existing 25-name survival corpus (depths 1-12) so results line up with prior on-air work.

`ndr_parse_test.c` (this directory) is the host harness: builds bare + LpPacket frames depth 1-12, checks
name + full-hash byte-exact, walks every truncation length for OOB safety, rejects junk/over-depth, times
the fused hot path.

---

## 6. Protocol & pass criterion

1. Build corpus on host; embed in harness for each firmware.
2. In the RX-hook context, loop the parser N times (e.g. 10 000) over each corpus frame; bracket with
   `AR_TSF_L32` (AR9271) / `esp_cpu_get_cycle_count()` (C5); accumulate per-frame total + min + max.
3. Exfil (WMI_ACCESS_MEMORY / send_framed); host divides by N, converts to us, computes min/median/p99.
4. Compute `T_gap` per MCS from measured MPDU rate; report `H = T_gap / T_parse` (p99).
5. **PASS: p99 `H >= 4` on AR9271 at top verified MCS (MCS4 HT20).** Cross-check C5.

**Interpretation:** PASS => in-firmware parse beats frame arrival with room to spare => MAC-field name
replication is redundant (retire the in-frame filter for this fleet, free the bits for timing/PHY/security).
FAIL on AR9271 => a pre-digested in-frame field has a real, measured justification on that device class.

---

## 7. Status & open items
- **Parser + host harness:** written (`ndr_parse.{c,h}`, `ndr_parse_test.c`). Host build/run and firmware
  integration were blocked by a session-level safety gate; resume in a fresh session.
- **AR9271 CPU MHz** - for cycles/byte only; read `k2_cmnos_clock_patch.c` on the build host.
- **C5 cycle counter** - validate `esp_cpu_get_cycle_count()` ticks at 240 MHz before trusting absolutes.
- **Non-minimal VAR-NUMBER** - host rejects, firmware accepts; framework picks accept-on-fast-path,
  reject-on-malformed; revisit under the link-hardening review.

---

## RESULTS (measured this session)

**Host (Mac, clang -O2):** correctness ALL PASS (names + FNV byte-exact depths 1-12, LPM, OOB-safe,
rejects malformed). Fused decision 11-78 ns. RISC-V cross-compile clean, 1578 B text / 0 data / 0 bss.

**ESP32-C5 ON SILICON** (240 MHz; ndr_parse.c at -O2 via `#pragma GCC optimize`; fused walk+roll-FNV+LPM;
worst-case 16-entry non-matching set; N=20000/pt via esp_cpu_get_cycle_count):

| name | depth | bytes | cyc/decision | time |
|---|---|---|---|---|
| /a | 1 | 11 | 525 | 2.19 us |
| /ndn/test/v1 | 3 | 23 | 944 | 3.93 us |
| /ndn/iot/sensor/temp/2026/reading | 6 | 47 | 1667 | 6.94 us |
| /x/y/z/w/v/u/t/s/r/q/p/o | 12 | 44 | 2402 | 10.01 us |

**Verdict (C5): PASS with room.** Worst case 10.0 us vs a frame-arrival gap >= ~350 us (~130-2850 MPDU/s)
=> H ~= 35x, far past the 4x bar. The radio parses the name faster than frames arrive => the MAC-field
name is redundant on this device class. Cost driver = the 64-bit FNV multiply on a 32-bit core (rv32 has
no 64-bit mul; each round is a libcall; -O2 bought only ~16%) + the per-boundary LPM. A receiver hashing
its OWN parsed name needs no cross-node 64-bit keyspace, so a 32-bit hash would ~halve this (future speedup).

**Still open: AR9271 (the decisive, hard target).** Needs the Xtensa toolchain on the build host (not on
this Mac). Slower core + no cache + no fast mul => expect several x the C5; measure before concluding the
whole fleet. Hook = ndr_rx_accept; time via AR_TSF_L32; exfil via ndr_stats + WMI_ACCESS_MEMORY.

---

## AR9271 RESULT — the decisive (hard) target

**AR9271 MAGPIE Xtensa @ 40.0 MHz** (measured on-device: CCOUNT over a firmware-timed 2000 us = 80038
cycles -> 40.019 cyc/us; `NOW()`==`xthal_get_ccount()`). Fused parse, net CCOUNT/16, worst-case 16-entry miss:

| name | depth | cyc/decision | us @ 40 MHz |
|---|---|---|---|
| /a | 1 | 935 | 23.4 |
| /ndn/test/v1 | 3 | 1581 | 39.5 |
| depth 6 | 6 | 2647 | 66.1 |
| depth 12 | 12 | 3910 | 97.7 |

Headroom vs frame gap (worst-case 97.7 us): 6 Mbps ~180 us -> **1.8x**; MCS4 ~90 us -> **0.9x (can't keep up)**;
MCS7 ~82 us -> **0.8x**. **FAILS the 4x bar at every rate; falls behind line rate (H<1) at MCS4+.**

## VERDICT (both targets measured): DEVICE-DEPENDENT
- **ESP32-C5 (240 MHz RISC-V):** parse 2-10 us, 8-18x headroom -> clears line rate -> the MAC-field name IS redundant.
- **AR9271 (40 MHz Xtensa, no cache):** parse 23-98 us, 0.8-1.8x headroom -> does NOT clear line rate -> a
  pre-digested in-frame field (the in-frame filter) DOES earn its keep here.

The blanket "in-frame filter is redundant" is WRONG for the constrained class. The filter's real home is exactly
the constrained / our-firmware radios the AR9271 represents — the same parts where the pre-USB wakeup win also
lives. Fast radios can just parse; slow radios need the pre-digested field.

Measurement traps (all resolved, see memory name-parse-clears-line-rate-c5): AR_TSF only ticks on-channel (use
CCOUNT); the reader re-downloads fw + no RX (trigger the bench from the WMI-read handler too, self-guarded); the
reader's 0xdeadbeef sentinel clobbers addr+16; xtensa gcc has no <stdint.h>; AR9271 needs a physical replug when wedged.
