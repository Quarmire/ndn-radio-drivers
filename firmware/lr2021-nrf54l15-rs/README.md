# lr2021-nrf54l15-rs — the named-radio MAC testbed

Firmware for the **Seeed XIAO nRF54L15 + Semtech LR2021** (LoRa Gen 4 / "LoRa Plus").

This is not just another LoRa node. It is the only hardware in the rig that can test the
**custom-hardware** MAC design rather than the commodity-Wi-Fi degradation of it — see
`ndn-ext/crates/faces/ndn-face-monitor-wifi/docs/named-filter-mac-redesign.md` §8.5. Task #102.

## What it unlocks

| Wi-Fi limitation | here |
|---|---|
| No hardware-scheduled TX — a slot is faked with a host sleep, so guards must be **ms** | DPPI + TIMER fires TX at an exact tick, no CPU in the loop |
| No hardware RX timestamp we own end-to-end | GPIOTE→DPPI→`TIMER.CC.CAPTURE` latches the DIO edge in silicon |
| Filter confined to 94 bits of 802.11 address field | full frame control — measure the real sizing curve |
| 32 B/frame of 802.11 header + LLC/SNAP | none |
| — | FLRC to **2.6 Mbit/s**, so slot structure is testable (pure LoRa SF7/125k is ~5.5 kbit/s: a 256 B frame is ~370 ms) |
| — | FLPR RISC-V coprocessor = NDN-NIC's "constrained NIC microcontroller", in real silicon |

## Status

**★ 7E-A5 v3 — modulation is a KNOB, not an identity (2026-08-28).** `SetPacketType` is a runtime
command with 14 modes and this firmware called it once at bring-up, then encoded that one-time choice
as identity: `EVT_CAP.radio_kind = 2` meant "LR2021-FLRC" and `3` was reserved for "LR2021-LoRa", as
though a node that changed modulation became a different part. That is undone. `CMD_SET_PHY` now
moves the node between **LoRa**, **FLRC** and **LR-FHSS** at runtime and replies with a whole new
`EVT_CAP`, because `max_payload`, `sf_min`/`sf_max`, the airtime model and `sched_gran_ns` are all
per-PHY. Also new: `CMD_SET_HOP` (intra-packet frequency hopping, table written by the host) and
`CMD_TX_AT_ABS` (schedule against an instant on the node's own clock). See
"7E-A5 v3" below.

**★ HFXO in every binary (2026-08-28) — and every M3–M5 timing number predates it.** `embassy-nrf`'s
`Config::default()` selects the **internal RC** as the high-frequency source, and 21 of the 22
binaries that bring up peripherals used it; only `m6_bridge` called `hw::init_peripherals()`
(`m1_bare` initialises nothing by design). Measured against a precise
host cadence the RC source runs **+2002 ppm**; on the crystal the same node measures **+16.7 ppm**.
Every binary now boots through `hw::init_peripherals()`.

**What that does and does not invalidate.** The M4 62.5 ns floor and the M5 58.9 µs guard band are
**resolution** figures — a ruler's ticks do not move when the oscillator driving it drifts — and they
stand. They are **not accuracy** figures, and were never taken on a disciplined clock, so nothing
absolute, drift-derived or common-view may be quoted from them: **re-take M4 and M5 through the
crystal before citing either as accuracy.** The +16.7 ppm figure is what a two-node common view has
to work against.

**7E-A5 parity closed (2026-08-28): `CMD_TX_AT` is real, and six more opcodes landed.** Scheduled TX
is CPU-mediated rather than DPPI — the trigger pin is the RX stamp's capture source and the stamp
wins — and `EVT_CAP.sched_gran_ns` says **50 µs** rather than pretending to the timer's 62.5 ns. Plus
`EVT_TX_STARTED` on every transmit path, `SET_BEACON`, `SET_CAD_CFG`, `SET_PREAMBLE`, `SET_DEBUG`,
`SET_RX_GAIN`. See "M6 result" below.

**M7a done — the Tier-0 prefix-set filter is built and its false-positive curve is MEASURED on
hardware, correcting #91's sizing: k=4, not 6.** Zero false negatives everywhere. See "M7a" below.

**#104/#105 done — and both are negative results worth having.** Stamping on the chip's SYNC event
gains nothing (the receiver is not the jitter source), the RF-switch pins make no measurable
difference, and the guard-band figure is **less precise than M5 implied**: see "#104/#105" below.

**M6 complete — the node speaks the rig's 7E-A5 host protocol on `/dev/ttyACM0`.** All five
sub-GHz/2.4 GHz nodes are now one fleet. See "M6 result" below.

**M5 complete — hardware-scheduled TX, 100% of armed slots transmit, and the guard band is
measured: 58.9 µs vs 100.9 µs software.** See "M5 result" below.

**M4 complete — the hardware RX timestamp works, and its resolution floor is below what the
instrument can measure: 62.5 ns.** See "M4 result" below.

**M3 complete — on-air FLRC link, 125/126 frames delivered (~99.2%).**
o5p-0 → o5p-1 at 2477 MHz, 2.6 Mbit/s, 0 dBm, one 200 ms beacon carrying a sequence number, over
~25 s. Exactly **one** frame lost after the first, sustained. See "M3 result" below.

**M2 complete — the LR2021 answers on both boards.** `get_version()` returns **firmware 1.24**
(`major=0x01 minor=0x18`), status OK, BUSY idle, on o5p-0 (probe `DDCBDC3E`) and o5p-1
(`8369B83C`). A plausible value, not the all-`0x00`/all-`0xff` signature of a mis-wired bus.
Embassy is up, the GRTC time driver ticks at exactly 1000 ms, and RTT is readable over SWD.

```
$ cargo build --release
   Finished `release` profile [optimized + debuginfo]
```

Verified from the linked image, not asserted: `.vector_table` @ `0x0`, `.text` @ `0x478`,
`.data` LMA in flash / VMA `0x20000000`, initial SP `0x20040000` (= top of the 256 KB RAM), reset
handler `0x479`. The memory map in `memory.x` is therefore consistent with the part.

| | milestone | needs board | proves |
|---|---|---|---|
| **M0** | ✅ builds for `thumbv8m.main-none-eabihf` | no | embassy-nrf `nrf54l15-app-s` + the `lr2021` driver resolve and link |
| **M1** | ✅ RTT on both boards | yes | flash + run + debug I/O, and the GRTC time driver |
| **M2** | ✅ SPI up, `get_version()` = fw 1.24 | yes | the `board` pin map, SPI mode and wiring |
| **M3** | ✅ FLRC link, 125/126 delivered | yes | an on-air link at a usable rate |
| **M4** | ✅ DPPI+TIMER RX capture, jitter measured | yes | the RX-timestamp floor: **≤62.5 ns**, below instrument resolution (⚠ taken on the RC clock — resolution, not accuracy) |
| **M5** | ✅ scheduled TX, 1100/1100 slots | yes | guard-band floor **58.9 µs** (vs 100.9 µs software) ⇒ sub-ms base slots for #93 (⚠ same caveat) |
| **M6** | ✅ 7E-A5 bridge on `/dev/ttyACM0` | yes | parity with the Waveshare/Heltec nodes — all five are one fleet; v2 gaps closed 2026-08-28 |
| **M7a** | ✅ Tier-0 filter built + FP curve measured | yes | #91's filter works; **k=4 measured, not the predicted 6–7** |
| **M7b** | run it on the FLPR RISC-V coprocessor | yes | NDN-NIC's "NIC microcontroller" in real silicon — feasible, see below |

## Toolchain findings (the de-risking, recorded)

- **`embassy-nrf` 0.11.0 supports this part**: features `nrf54l15-app-s` / `-ns` (also `nrf54l05`,
  `nrf54l10`, `nrf54lm20`). DPPI, GPIOTE `InputChannel`, and `timer::Cc::{task_capture,
  event_compare}` are all present — every primitive M4/M5 need.
- **A Rust LR2021 driver exists**: [`lr2021`](https://github.com/TheClams/lr2021) v0.13.1
  (Dec 2025) — async, `no_std`, `embedded-hal-async`. **Community-maintained, not official
  Semtech**, 12 commits: treat it as a starting point, not a dependency to trust blindly.
  **Vendored** at `vendor/lr2021` — see below.
  - **FLRC is "tested"** and `FlrcBitrate::Br2600` confirms 2.6 Mbit/s. `FlrcCr::{Cr12,Cr34,Cr23}`.
  - **LoRa is only "partial"** — matters for interop with the SX1262/SX1276 nodes (M3+).
  - CAD *is* present (`set_cad_params`/`set_cad`), and `set_rx_duty_cycle(listen, cycle,
    use_lora_cad, dram_ret)` gives the duty-cycled wake path task #90 needs.
  - **No packet timestamping and no timed TX** — and this does *not* matter, because neither
    belongs to the radio. Both are MCU-side, via DPPI (see `src/timing.rs`).
- `cortex-m` needs `critical-section-single-core`, or the link fails on
  `_critical_section_1_0_acquire`.

## Vendored LR2021 driver

`vendor/lr2021` is crates.io 0.13.1 with **one** local change. Upstream's manifest says:

```toml
embassy-time = { version = "0.5.0", features = ["defmt", "defmt-timestamp-uptime", "tick-hz-32_768"] }
```

A driver crate must not pin the *application's* tick rate. `tick-hz-32_768` collides with the 1 MHz
GRTC time driver that **every nRF54L board uses**, so upstream as published cannot build against the
HAL it is most likely to be paired with. The vendored copy leaves `embassy-time` bare and lets the
application choose. Worth reporting upstream.

Vendoring is the right call here anyway: the driver is young, and M4/M5 will need hooks it does not
have.

## Flashing — resolved

The XIAO enumerates as `2886:0066 Seeed Studio XIAO nrf54 **CMSIS-DAP**`, i.e. an onboard SWD debug
probe — *not* a UF2 bootloader. So `memory.x`'s `FLASH ORIGIN = 0x0` is correct, and `probe-rs`
drives it directly. `/dev/ttyACM0` is the probe's virtual COM port (useful later for M6).

```sh
# on the OPi (NixOS; probe-rs is not installed system-wide)
P=$(nix build --impure --no-link --print-out-paths nixpkgs#probe-rs-tools)/bin/probe-rs
sudo -n $P run --chip nRF54L15 /tmp/lr2021-m1.elf     # flash + attach RTT
sudo -n $P download --chip nRF54L15 <elf>             # flash only
sudo -n $P verify   --chip nRF54L15 <elf>
```

Target confirmed live over SWD: Cortex-M33, ARMv8-M, Nordic VLSI ASA, DPv2.

## The pin map — resolved from two devicetrees

| signal | XIAO | nRF54L15 | polarity / pull |
|---|---|---|---|
| `DIO8` (IRQ) | D0 | **P1.04** | active high, pull-down |
| `BUSY` | D1 | **P1.05** | active high, pull-up |
| `NRESET` | D2 | **P1.06** | active **low** |
| `NSS` | D3 | **P1.07** | active **low** |
| `SCK` | D8 | **P2.01** | `SPIM00`, ≤16 MHz |
| `MOSI` | D10 | **P2.02** | |
| `MISO` | D9 | **P2.04** | |

Sources: Semtech's shield overlay `boards/shields/semtech_wio_lr20xx/semtech_wio_lr20xx_common.dtsi`
in [Lora-net/usp_zephyr](https://github.com/Lora-net/usp_zephyr) (control lines, polarities, pulls,
SPI ceiling) and Zephyr's `boards/seeed/xiao_nrf54l15/seeed_xiao_connector.dtsi` (D*n* → port/pin).

**The earlier placeholder was wrong on every single line** — different port, different pins,
different order. That is exactly why `main()` was left stopping short of SPI until the map was
sourced: a guessed pinout fails as "the radio never answers", indistinguishable from a dead part.

The SPI pins are on **P2.x**, which forces the instance to `SERIAL00`/`SPIM00` (the high-speed one
off the 128/64 MHz PLL domain), not a `SERIAL2x`. `embassy-nrf` implements `SPIM00` only under `_s`,
which `nrf54l15-app-s` provides.

Further shield facts for later milestones: `reg-mode = DCDC`, `lf-clk = RC`, `tcxo = 1.8 V` with
wakeup 0, `rx-boost-cfg = 7`, `tx-power-offset = 0`, calibration at 470 MHz / 897.5 MHz / 2441 MHz,
and per-dBm PA tables for both LF and HF paths. Two SMAs: **LF** (150–960 MHz) and **HF** (2.4 GHz +
S-band).

## M7a — the Tier-0 filter, and a correction to its sizing

`src/tier0.rs` implements #91's in-frame prefix-set Bloom filter: 94 usable bits (96 minus the I/G
and U/L bits, which must keep their locally-administered/group meaning), every prefix of the name
inserted, depth capped at 8, keyed so a private group's filter is unlinkable. Hashing is the
project's existing FNV-1a-64 name hash expanded by double hashing, keeping **one name-hash keyspace**
shared with the FIB and data plane (#44) rather than adding a second family.

`m7_filter_test` measures it on device, 20 000 trials per point:

```
M=94 bits, K=4, depth cap 8
  depth 2: bits_set 12/94 | FP  19/20000 = 0.095% | false negatives 0
  depth 4: bits_set 19/94 | FP  48/20000 = 0.24%  | false negatives 0
  depth 6: bits_set 27/94 | FP 160/20000 = 0.80%  | false negatives 0
  depth 8: bits_set 29/94 | FP 187/20000 = 0.94%  | false negatives 0
```

**Zero false negatives at every depth** — the property the whole design rests on, checked on every
iteration rather than sampled. Worst case at the depth cap is **0.94% ⇒ 99.06% zero-parse
rejection**, against NDN-NIC's 96.30% (which needed 16 KB of receiver table; this is 12 bytes in the
frame — different jobs, see §8.1 of the design note).

### The correction: k=4, not 6

#91 chose **k=6** from `(M/n)·ln2`, which gives ~7 for these sizes. Measured at the depth cap:

| k | bits set | FP at depth 8 |
|---|---|---|
| 3 | 25/94 | 1.99% |
| **4** | **29/94** | **0.94%** ← measured optimum |
| 5 | 35/94 | 0.98% |
| 6 | 42/94 | 1.09% |
| 8 | 53/94 | 1.50% |

The formula assumes a query's k positions are **independent**. With only 94 bits they are not: k=6
positions collide *with each other* ~15% of the time, and a query whose 6 positions collapse to 3
distinct bits has the false-positive rate of k=3. That effect is invisible to the formula and grows
with k, so the true optimum sits *below* the predicted one. **Small-m Bloom filters are their own
regime — do not size this one from the asymptotic formula.** Lower k is also cheaper: 4 hash
positions per prefix instead of 6.

Ruled out along the way: deriving `h1`/`h2` by splitting one FNV output was suspected of correlating
the positions (FNV's high bits are its weak half). Switching to two **independent** keyed hashes
measured no better — so the deviation is the small-m effect above, not hash quality. The independent
pair is kept anyway: it costs one extra FNV over a short prefix and removes a theoretical weakness
that simply did not bite at these sizes.

### M7b — the FLPR coprocessor is reachable

Not attempted yet, but the path is confirmed rather than assumed: `nrf-pac` ships an
**`nrf54l15-flpr`** chip, `embassy-nrf` has a **`vpr`** module with a full loader
(`Vpr::new / load / init / start / stop`, plus `make_secure`) and an `FlprReset` boot option, and
both **`riscv32imac-`/`riscv32imc-unknown-none-elf`** targets are already installed. `tier0.rs` is
dependency-free integer code, so it compiles for RISC-V unchanged. What remains is a second crate
for the FLPR image, a linker script placing it in shared RAM, and a mailbox protocol — that is
NDN-NIC's "constrained NIC microcontroller" in real silicon, which the paper simulated and never
built.

## #104 / #105 — two negative results, and a correction to the M5 number

### #105 — the RF-switch pins make no measurable difference

Zephyr's dts asserts `rfsw_ctl` (P2.05, active low) and `rfsw_pwr` (P2.03, active high) at boot;
our firmware never did. Driven vs floating, back to back at the same geometry:

| arm | delivery | crc_err | rssi_raw |
|---|---|---|---|
| floating | 125/126 | 21 | 190 / 199 / 198 |
| driven | 125/126 | 18 | 187 / 192 / 199 |

Identical delivery, overlapping RSSI. **They are driven by default now anyway** — it matches what the
board's own devicetree does and costs nothing — but the unknown is closed by measurement rather than
by assumption. Build with `--features no-rf-switch` to reproduce the floating arm.

### #104 — the receiver is *not* the jitter source

`SetTimestampSource(TS0, SYNC)` + `GetTimestampValue` let the chip stamp **syncword detection** — a
fixed, early point in the frame — instead of packet-done, and report the delay to the SPI NSS edge in
32 MHz ticks. Reconstructed onto our 16 MHz timer and compared against the DIO edge **on the same
frames** (n≈2000):

```
DIO  (packet-done) min=320835 mean=321163 max=321526 p2p=691 ticks
SYNC (chip stamp)  min=320834 mean=321163 max=321526 p2p=692 ticks   ts_fail 0
```

**Identical to within one tick, means exactly equal.** The receiving radio's demodulate-to-DIO path
is already as deterministic as its own SYNC stamp, so the packet-done latency M5 flagged as a
suspect contributes nothing. That *localizes* the residual: not the receiver, and not oscillator
drift either (two crystals at ±20 ppm over a 20 ms period is ~13 ticks, against a spread of ~600).
What remains is the transmitter's internal trigger→on-air path.

`RampTime` is now **`Ramp2u` instead of `Ramp16u`** regardless — the PA ramp sits between the trigger
and the first on-air symbol, so a shorter ramp is strictly less offset. Whether it reduced *jitter*
is unproven; see below.

### The correction: 58.9 µs was over-precise

Three runs of nominally the same measurement:

| run | config | p2p |
|---|---|---|
| M5 | Ramp16u, RF switch floating | 942 ticks = 58.9 µs |
| #104 a | Ramp16u, RF switch driven | 691 ticks = 43.2 µs |
| #104 b | Ramp2u, RF switch driven | 596–620 ticks = 37.3–38.8 µs |

The trend is downward, but run 1→2 changed nothing that should affect transmit timing and still moved
by 250 ticks. **Run-to-run variation of this statistic is large, so quoting "58.9 µs" to three
significant figures was wrong of me.** The defensible statement is **~40–60 µs end to end**, and a
guard band should be sized on the worst of several runs, not one run's peak-to-peak.

This does not change the conclusion for #93 — at ~785 µs of airtime a 40–60 µs guard is still
5–8% overhead, and still sub-millisecond base slots — but it changes how the number should be quoted
and re-measured.

## M6 result — the host bridge

Speaks the same 7E-A5 protocol as the Waveshare and Heltec nodes, on **UART20** — the UART the
XIAO's onboard CMSIS-DAP probe bridges to `/dev/ttyACM0` (Zephyr's dts: `zephyr,console = &uart20`;
the header UART `uart21` on D6/D7 goes to the pin header instead and the host could not reach it).
No new host transport needed.

```
EVT_INFO         payload=00 01 18 93a40540 00   → status ok, fw 1.24, 2477000000 Hz, 0 dBm
EVT_RSSI         payload=00c7
EVT_UNSUPPORTED  payload=9901                   → cmd 0x99 rejected, not ignored
```

Why parity matters: the rig's five nodes (2 LR2021 + 2 Waveshare + 1 Heltec) become one addressable
fleet, which is what the N≥3 MAC experiments need — the claimable-slot and hidden-terminal tests
(#94/#95) cannot run on a two-node link.

### Closing the v2 gaps (2026-08-28)

`cmd_bitmap` went from `0x0C9F_CB5E` to **`0x1DBF_DFDE`** and `sched_gran_ns` from 0 to **50 000**.
(v3 took it to `0xFDBF_DFDE` — see "7E-A5 v3" above.)

| opcode | what it does here | why not the obvious thing |
|---|---|---|
| `0x18 TX_AT` | wait on the **same free-running counter** the RX stamp uses, then `SetTx` over SPI | the DPPI trigger needs DIO8 as an *input*, and DIO8 is the RX stamp's capture source. The stamp is the fleet's only hardware timestamp, so the schedule moved to the CPU rather than the pin changing hands. **v3 note:** its delay is counted from the firmware's arm, so the serial round trip is inside the placement — `CMD_TX_AT_ABS` is the one that reaches `sched_gran_ns` |
| `0x88 TX_STARTED` | emitted on `TX`, `TX_LBT` **and** `TX_AT`, between staging and key-up | airtime is computed from the live link (`LinkState::airtime_us`), not hardcoded — the rung ladder spans 10×. Rounded **up**, floor 1 ms: an FLRC frame is sub-millisecond and the fleet's field cannot say so |
| `0x07 SET_BEACON` | periodic self-transmit, **default OFF** | a node that beacons on power-up transmits into a neighbour's measurement; this bench runs several in one band |
| `0x0A SET_CAD_CFG` | `sym` scales the CCA window 1/2/4/8/16×; `det_peak`/`det_min` **must be 0** | reinterpreting a correlator ratio as a dBm threshold would read a saturated channel as idle. **v3 gap:** in LoRa mode the part *does* have a real correlator (`SetLoraCadParams`), so those two fields have a genuine analogue there; wiring it needs the host to key `CMD_CAD`'s meaning on `phy_current` first, and refusing is the safe direction until then |
| `0x0C SET_PREAMBLE` | value taken as FLRC preamble **bits**, rounded up to the 4-bit step, 4..32 only | FLRC has no symbols. Outside the register's reach it is refused, not clamped — LoRa's 8 *symbols* and 8 *bits* are different requests. **In v3's LoRa mode the field finally means symbols**, and needs no reinterpretation at all; in LR-FHSS there is no preamble knob and it is refused |
| `0x15 SET_DEBUG` | toggles `EVT_LOG`, off by default | `EVT_LOG` was declared and never emitted; this gives it a purpose without letting diagnostics compete with `EVT_RX` for a 115200 link |
| `0x1C SET_RX_GAIN` | `0` → AGC auto, `1` → max manual gain | the LR2021 has a 0..13 ladder, but the fleet's byte is two-valued; exposing the ladder would make `1` mean "boosted" on one node and the **lowest** gain on another |

Still refused, all `REASON_NO_HARDWARE` — understood, and unreachable from firmware:
**`0x05 SET_SYNC`** (the wire carries one byte, FLRC's syncword is 32 bits — and two nodes that
disagree about a syncword do not error, they go silent, which is indistinguishable from a dead
radio), **`0x0D SF_SCAN`** (no spreading factor to scan for in FLRC, and no multi-SF scan implemented
in LoRa), **`0x16 ENTER_BOOTLOADER`** (the XIAO reflashes over its own CMSIS-DAP probe; there is no
ROM loader to jump to).

The 50 µs is **derived, not measured**: `SCHED_TICK_NS` 63 + `SCHED_SPI_NS` 5 000 + `SCHED_RAMP_NS`
2 000 + `SCHED_MCU_NS` 30 900 (M4's measured executor-wake path) = 37 963 ns, declared as 50 000 with
the balance as margin. Over-stating a granularity is safe; under-stating it tells a slot scheduler it
can pack slots this node cannot hit. Measuring it properly is an M5-shaped run: a `CMD_TX_AT` train at
a fixed period, read as a consecutive-pair spread off the receiver's hardware stamp.

Two deliberate differences from the Waveshare node, both documented in `src/serial.rs`:

- **`EVT_RX.ts` carries the M4 hardware capture** (DPPI-latched at the DIO edge, 62.5 ns), not a
  software millisecond counter. Same field, far better number — stamp precision is a per-node
  property the host should read rather than assume.
- **Commands this PHY has no analogue for are answered with `EVT_UNSUPPORTED`, not ignored.** A host
  that assumes a spreading-factor knob in FLRC gets an error instead of silence: a diagnosable bug
  rather than a mystery. In v3 the honest answer to several of those is now "switch PHY first".

The on-device NDN data plane is **shared by path** with `waveshare-lora-rs` (`#[path]` to its
`ndn.rs`), not copied. Filter/dedup/relay semantics must be byte-identical across nodes that
interoperate; two implementations agreeing today would drift, and the failure mode — one node
silently dropping traffic its neighbour forwards — is indistinguishable from a link problem.

## 7E-A5 v3 — modulation as a knob (2026-08-28)

`cmd_bitmap` went from `0x1DBF_DFDE` to **`0xFDBF_DFDE`**, `EVT_CAP` from 29 bytes to **34**, and
`proto_ver` from 2 to **3**.

⚠ **`0xFDBF_DFDE` is now a full field.** `CMD_GET_HOPTRACE` is opcode **0x20 = 32**, one past the end
of the four-byte `cmd_bitmap`, so it is the first command in this protocol that cannot be advertised
and has to be **probed** for (send it; get `EVT_HOPTRACE` or `EVT_UNSUPPORTED`). Widening the field
would break four firmwares and two host crates at once to advertise one diagnostic opcode, so it is
not widened — `serial::CMD_BITMAP_FULL` carries the bit for firmware-side reasoning and the wire
field is its low 32 bits **by construction**, with a test pinning the boundary rather than leaving it
to be rediscovered when a 33rd opcode silently vanishes.

`cmd_bitmap` is now **per-PHY**, like everything else in `EVT_CAP`: the figure above is the surface
in LoRa and LR-FHSS, and in **FLRC it is `0xBDBF_DFDE`** — `CMD_SET_HOP` (bit 30) drops, because
this part has no FLRC hopping command and `check_hop` refuses every call there. The bitmap means
"implemented and *will act*", and the host builds a whole `HopCapability` out of that bit.

### The design error this fixes

FLRC ran because `flrc_link::configure()` called `set_packet_type(PacketType::Flrc)` **once**. That
one-time choice then travelled on the wire as the node's *kind*. It is not a kind: `SetPacketType`
(datasheet Table 8-1) is a runtime command with fourteen modes — `0x0` LoRa, `0x2` FSK, `0x3` BLE,
`0x4` RTToF, `0x5` FLRC, `0x6` BPSK, `0x7` LR-FHSS, `0x8` WM-Bus, `0x9` Wi-SUN, `0xA` OOK, `0xC`
Z-Wave, `0xD` O-QPSK — and the vendored crate exposes every one. **Modulation is a knob cognition
actuates, exactly like MCS or spreading factor.** It is fleet-wide too: the SX1262 does LoRa + GFSK,
the SX1276 does LoRa + FSK + OOK.

### What changed on the wire

| | |
|---|---|
| `CMD_SET_PHY 0x1D` | `[packet_type u8]` → **the full new `EVT_CAP`** |
| `CMD_SET_HOP 0x1E` | `[hop_ctrl][period u16][n][freq_hz u32]*n`, n ≤ 40 → `EVT_INFO` |
| `CMD_TX_AT_ABS 0x1F` | `[target_ticks u64][frame]` → `EVT_TXDONE` |
| `CMD_GET_HOPTRACE 0x20` | `[]` → `EVT_HOPTRACE`. ⚠ **opcode 32 — past the end of the 32-bit `cmd_bitmap`**, so it is probed for, not advertised |
| `EVT_PHY_ERR 0x8D` | `[requested_phy, chip_status]` — an advertised PHY the **chip** refused |
| `EVT_HOPTRACE 0x8E` | `[stamp_hz u32][n u8][(idx u8, t_ticks u32)]*n`, n ≤ 32 — this node's own hop timeline |
| `EVT_CAP[1]` | `radio_kind` now names the **part**: 0 SX1262, 1 SX1276, 2 LR2021. v2's `3` is retired |
| `EVT_CAP[29..33]` | `phy_bitmap u32` — bit N set ⇒ `SetPacketType` value N is usable here |
| `EVT_CAP[33]` | `phy_current u8` |

`phy_bitmap` on the default LF build is **`0x0000_00A1`** = LoRa | FLRC | LR-FHSS. On an HF
(`PHY_HF=1`) build LR-FHSS is dropped: its grids (25.39 / 3.91 kHz) and bandwidths (up to 1523.4 kHz)
are the LoRa Alliance's sub-GHz channel plans. A set bit is a promise, so an untested mode is simply
not advertised.

**`EVT_CAP` describes the CURRENT PHY**, which is why `CMD_SET_PHY` replies with a whole body and the
host must replace its profile rather than patch fields:

| | LoRa | FLRC | LR-FHSS |
|---|---|---|---|
| `max_payload` | 247 | 47 | 247 |
| `sf_min`/`sf_max` | 7 / 12 | 0 / 0 | 0 / 0 |
| airtime, 48 B | ~100 ms @ SF7/125k | 230 µs @ 2.6 Mbit/s | ~3.4 s @ CR 1/3 |
| framing | explicit header, payload as-is | fixed PDU, in-frame length byte, software whitening | `LrFhssBuildFrame` |
| `CMD_SET_MOD` | `[sf, bw, cr]` — the fleet's own meaning | `[rung, 0, FlrcCr]` | `[0, LrfhssBw, LrfhssCr]` |
| intra-packet hopping | yes | **no** (refused, never ignored) | yes |
| `sched_gran_ns` | 50 000 | 50 000 | 50 000 |

The granularity is the same on all three **by derivation**: its four terms (timer tick, `SetTx` over
SPI, PA ramp, MCU reaction) are PHY-independent, because the frame is *staged* before the deadline
and only one five-byte `SetTx` happens after it. It is still exposed as a function of the PHY so a
future mode with a different fire path cannot silently inherit the wrong number.

LoRa reports **SF7..SF12**, not the part's SF5..SF12: SF5 does not exist on an SX127x at all, and SF6
needs `comp_sx127x_sf6_sw` plus implicit-header framing — a different packet format, not a different
number. That is also exactly the span the Waveshare node advertises, which is what makes a fleet-wide
`CMD_SET_MOD` sweep mean the same thing everywhere.

### `CMD_TX_AT_ABS` — and why the relative opcode could never hit 50 µs

Scheduled TX works; the **relative** opcode caps its placement. An absolute-boundary slot train
measured on this node:

```
  45/45 fired
  accuracy   mean gap 2,399,818 ticks vs 2,400,000 nominal   ->  11 µs over 44 slots
  jitter     sd 553 µs, p2p 1875 µs                          ->  vs 50 µs declared
  the same node's CMD_GET_INFO round trip: p2p 550 µs
```

Those two 550 µs figures are the same number. `CMD_TX_AT`'s `delay_us` is counted from when the
**firmware** processes the arm, so the host→device serial latency lands inside the placement — as
exercised it is *worse* than the software path (sd 553 µs vs 155 µs) because it pays an extra round
trip for nothing.

`CMD_TX_AT_ABS` names an instant on the same free-running TIMER20 that `CMD_READ_CLOCK` reports and
`EVT_RX.ts` is latched from, so that latency is spent *before* the deadline where it is free. The
declared `sched_gran_ns` = **50 µs** describes this path and only this path:

```
  timer tick, ceil(1e9 / 16 MHz)     63 ns
  SetTx over SPI, 5 B @ 8 MHz     5 000 ns
  PA ramp, RampTime::Ramp2u       2 000 ns
  MCU reaction (M4-measured)     30 900 ns
                                 --------
                                 37 963 ns  ->  declared 50 000
```

The 32/64-bit arithmetic is explicit: the counter is 32 bits and wraps every ~268 s, the target is
64-bit on the firmware-extended clock, a target already past **fires immediately** (never a wait for
the wrap), and a target more than 1 s ahead is `OUT_OF_RANGE`. Inside that bound the low 32 bits are
unambiguous, which is what makes the cast safe — `m5_tx` measured the alternative: 74 transmits and
then silence until the counter came round. `CMD_TX_AT` stays, because it is still the right primitive
for a delay the *firmware* computes, where no serial hop exists.

### Intra-packet hopping (`CMD_SET_HOP`)

The carrier moves **inside one frame**, on a table the host writes in Hz — up to 40 entries, the chip
depth for both mechanisms. Not to be confused with `CMD_DATAPLANE`'s name-keyed inter-frame hopping,
which still has no channel-index convention on this bearer and is still refused.

⚠ **§9.8: the LR20xx and SX127x hop INCOMPATIBLY by default** — "the internal timing, frequency
switching mechanisms, and control logic evolved between the chip generations, making them unable to
properly synchronize their hopping sequences". There is an explicit SX1276 compatibility mode (one
bit of `lora_modem_main_tx_cfg1` at `0xF30A24`, wrapped as `comp_sx127x_hopping`) and it is enabled
**whenever hopping is on**, because the Heltec SX1276 is the intended peer and a hop sequence only
one end can follow fails mid-frame — which reads as a marginal channel rather than as a
misconfiguration.

Asking for hopping in FLRC is refused with a reason. There is no FLRC hopping command on this part,
and silently accepting the table would leave a host believing its frames were spread when they were
sitting on one carrier.

★ **Two vendor-crate defects found and worked around** (in our code, not by forking the driver):

- `set_lora_hopping` has an **off-by-one**: it fills `buffer[0..4 + 4n]` and transmits `3 + 4n`,
  truncating the last frequency's low byte. With `n = 0` the length is right by accident, so the
  disable path would have looked fine.
- `set_lrfhss_hopping` is **uncallable from outside the crate**: it takes `&[LrfhssHop]`, and
  `LrfhssHop`'s two fields are private with no constructor.

### `CMD_GET_HOPTRACE` — each node timestamps its OWN hop events

**The question.** An LR2021 and a Heltec (SX1276) both do LoRa intra-packet hopping, each
interoperates with its own kind, and they cannot hop with each other. Everything cheap is measured:

| TX (hopping) | RX | RX hop | result |
|---|---|---|---|
| Heltec n=1 | Waveshare (no hop support) | off | **4/4** |
| LR2021 n=1 | Waveshare | off | **4/4** |
| Heltec n=1 | LR2021, hop **off** | off | **4/4** |
| Heltec n=1 | LR2021, hop **on** | on | **0/4** |
| LR2021 n=1 | Heltec, hop **on** | on | **1/20** |
| LR2021 n=1 | Heltec, hop **off** | off | **20/20** |
| LR2021 n=4 | LR2021, hop on | on | 4/4 |

`n = 1` is a **one-entry hop list**: the machinery runs and the carrier can never move. So it is not
the frequency sequence or its phase, not the frame format (a plain receiver decodes both parts'
hopping transmissions perfectly), and not structural (1/20 is not 0/20 — a format mismatch would be
absolute). What is left is that the two disagree about **when** a hop boundary falls, which is what
§9.8 says in as many words. A period sweep is already ruled out: with the Heltec's RX period fixed at
8 symbols, sweeping the LR2021's TX period over 2/4/8/16 gave 0–1 of 10 at **every** setting, so no
search over the exposed knobs will find it.

★ **The measurement needs no cross-vendor decode**, which is what makes it possible at all: each node
stamps its **own** hop events on its **own** clock and the host compares two timelines. At SF7/BW125
one symbol is 2^7/125000 = **1.024 ms**, so a nominal 8-symbol period is **8.192 ms** = 131,072 ticks
here. Whatever differs — the interval, the phase of the first hop relative to the start of a frame,
or whether hops continue between frames — is the answer.

**Does this part signal each hop at all? Yes — two interrupts, from source.** `vendor/lr2021/src/status.rs`:

| bit | constant | accessor | the crate's own wording |
|---|---|---|---|
| `0x0000_1000` | `IRQ_MASK_LORA_TX_RX_HOP` (:203) | `Intr::lora_tx_rx_hop()` (:361) | "IRq for LoRa intra-packet hopping" |
| `0x0200_0000` | `IRQ_MASK_FHSS` (:231) | `Intr::fhss()` (:413) | "IRQ after each ramp-up for intra-packet hopping" |

Both are enabled and neither is presumed: which one the silicon actually raises — or whether both
fire, at two different instants — is a measurement, not a reading of two doc strings.
`hoptrace::HopTrace` folds them into **one event per IRQ poll**, so enabling both cannot double-count.

They reach the MCU on **DIO8**, the only LR2021 DIO the shield routes to the XIAO (→ D0 → P1.04).
`set_dio_irq` takes a per-DIO mask, so the hop bits are added to the line **only while a hop plan is
live**.

**Where the stamp is taken.** `TIMER20.CC[0]`, latched by **DPPI in silicon** at the DIO8 rising
edge — the same capture register, the same free-running 16 MHz counter and the same GPIOTE/DPPI route
`RxCapture` already uses for `EVT_RX.ts`. Sharing the timebase is the point: a hop instant and a
frame arrival on one node subtract directly.

Between the RF hop boundary and that tick:

| term | size |
|---|---|
| carrier transition → IRQ assertion inside the LR2021 | **NOT MEASURED.** `IRQ_MASK_FHSS` fires "after each ramp-up", i.e. deliberately after the PLL/PA settle — a positive offset of unknown size. `IRQ_MASK_LORA_TX_RX_HOP` states no phase at all |
| DIO8 pad + shield trace → P1.04 | ns, below one 62.5 ns tick |
| GPIOTE edge detect → DPPI → `CC[0].CAPTURE` | a few 16 MHz cycles, fixed — the same silicon path M4 measured |
| CPU wake, SPI status read, `CC[0]` read | lands **after** the latch and cannot contaminate it |

The first term is **not folded in and not guessed at**. It does not have to be: the quantities being
compared (a hop *interval*, and the *phase* of the first hop against a frame) are differences, and a
constant offset cancels in both.

**The one DIO8 cost, stated rather than hidden.** DIO8 carries every enabled interrupt and stays high
until the status is cleared over SPI, so a second event while it is already high raises no new edge —
`CC[0]` always holds the **first** edge since the last clear. Consequence: if a hop and an `RxDone`
fall in one poll window they share the one capture, and `CC[0]` is **whichever came first — not
necessarily the hop.** Inside a packet the hop does come first; but after an `RxDone` the modem stays
in RX and keeps hopping, so a hop landing in the ≤1 ms before the next poll is stamped with the
*frame's* instant. At a ~1 ms poll and an 8.192 ms period that is ~12 % of receptions — **derived from
those two numbers, not measured.** Such an entry is *ambiguous*, and a host must **drop** it rather
than correct it. It is never silent: the case is **counted** (`EVT_LOG` under `CMD_SET_DEBUG`) and
the offending `EVT_RX.ts` appears verbatim as a `t_ticks` in the ring. ★ The clean avoidance is a run
discipline, not firmware — **take the timeline on a node that is hopping but not receiving.**

It is also gated: with hopping off, which is every measurement taken on this board to date, the DIO8
mask and the RX stamp path are bit-identical to before. The capture path is **not** taken away from
`RxCapture`, which keeps `GPIOTE20_CH0`, `PPI20_CH0` and `CC[0]`; a separate line is simply not
available, because the shield routes exactly one radio DIO to the MCU.

**⚠ A second sharing cost: the poll rate bounds which hops are SEPARATED.** The main loop reads the
status roughly every 1 ms (the UART read timeout plus the SPI work), which is ~8× finer than an
8.192 ms period — but only roughly, and `wait_tx_done` polls at a firm 200 µs while the node is
transmitting. Anything that keeps the main loop away for longer than a hop period collapses every hop
in that gap into **one** entry stamped at the first of them; a 170-byte `EVT_HOPTRACE` reply on the
115200 UART is ~15 ms and is itself such a gap, so **do not poll the trace during the window you are
measuring**. The failure is not silent at the host: the surviving intervals come out as integer
MULTIPLES of the true period, which is the first thing a reader should check.

**`idx` is a firmware counter, not a chip readback.** The SX1276 reports the live hop index in
`RegHopChannel`; this part exposes no equivalent — `SetLoraHopping` (opcode 556) is commented out of
the vendor command spec, there is no `GetLoraHopStatus`, and no documented address in
`lr2021::constants` reads one back. So the index counts hop interrupts modulo the table depth, and it
is deliberately **not** reset per frame — whether the chip restarts its sequence at entry 0 for each
packet is one of the things the trace exists to find out. At `n = 1`, which is the configuration every
row of the table above uses, every index is 0 either way.

☠ **`idx` is therefore NOT the same quantity as the Heltec's `idx`**, though the byte, the mask and
the flag bit are identical. There it is `RegHopChannel` verbatim — the *chip* counting — so it jumps
when a hop is swallowed and would show a per-packet reset; here it advances once per *recorded* event
and can show neither. ★ **Compare `t_ticks` across the two nodes; compare `idx` only within one.** A
difference in the first `idx` value between the parts is a fact about this firmware's counter, not
about the silicon.

**Arming clears the ring; disarming does not.** `CMD_SET_HOP 1` starts a fresh timeline, so one plan's
events can never be served under another. `CMD_SET_HOP 0` and `CMD_SET_PHY` stop the recording and
**keep** what was recorded — the same rule `heltec-lora-rs` follows. The agreement matters: if one
node cleared on disable and the other did not, a harness that stopped the hopping before pulling the
timeline would get a full trace from one part and `n = 0` from the other, and at the host `n = 0` from
a part that hops is indistinguishable from "this part does not signal its hops at all" — a conclusion
the measurement protocol explicitly draws. So `n = 0` means exactly *nothing recorded since the last
plan was armed*, on both nodes.

**Where a transmit started.** Bit 7 of `idx` (`hoptrace::IDX_TX_KEYED`) marks a **TX key-up** entry
rather than a hop; the table is 40 deep, so that bit cannot occur on a real index and the wire layout
is unchanged. It costs **no SPI on the transmit hot path** — the index is the firmware counter and
the instant is one MCU timer-register read, both taken in the gap before `SetTx` leaves the MCU. ⚠ It
is the one entry whose stamp is a *software* read: between it and the RF key-up sit the `SetTx`
transaction (≈5 µs at 8 MHz) and the chip's PLL/PA ramp, **neither measured**. There is no hardware
key-up event to capture instead — `IRQ_MASK_TX_TIMESTAMP` marks the *end* of a transmitted packet.

**⚠ Nothing here is measured on air.** This agent does not touch hardware; the instrument is built
and the timeline comes from a node on the bench.

### LR-FHSS — reachable, and the chip gets to settle the contradiction

The datasheet disagrees with itself:

- **§17.1**: "In the LR20xx, LR-FHSS is implemented as a transmit-only mode."
- **§17.2.2**: `LrFhssSetSyncword` "configures the synchronization word utilized for LR-FHSS
  **detection on the receiver side**, and for building the Tx frame on the transmitter side."

There *is* a plausible mechanism for a real transmit-only limit: LR-FHSS modulates at **488.28125
bit/s** (exactly 2048 µs/bit) and Table 11-2 gives the generic (G)FSK modem a bitrate minimum of
**500 bps** — 2.4% above it.

That is a hypothesis and the firmware does not act on it. `CMD_SET_PHY 0x07` brings the mode up,
issues a real `SetRxContinuous`, and forwards **the chip's literal status byte** as `EVT_PHY_ERR
[0x07, chip_status]` if it is refused. Crucially, a refusal to *arm RX* does **not** revert the PHY:
LR-FHSS transmits perfectly well, and reverting would make it unreachable — the opposite of the
point. The host gets `EVT_PHY_ERR` *and* the new `EVT_CAP`.

⚠ **Not measured here.** This agent does not touch hardware. The mechanism to answer the question is
in place; the answer comes from a node on the bench. And **arming is not receiving** — a `CMD_OK`
from `SetRxContinuous` would not be evidence that LR-FHSS demodulates anything.

`ReadLrFhssHoppingTable` (0x58) and `WriteLrFhssHoppingTable` (0x59) are both exposed, and a
`CMD_SET_HOP` in LR-FHSS with `CMD_SET_DEBUG` on performs a write/read-back round trip and reports
the first echoed frequency as `EVT_LOG`. That is what turns "the hop sequence is ours to write" from
a claim into something checkable without a reflash. The `pkt_length` and `nb_hopping_blocks`
arguments are **inferred** from the frame structure, not sourced — which is precisely why the
read-back exists.

The LR-FHSS airtime model's frame-structure constants (114-bit sync header, 48-bit fragments, 2-bit
block preambles) come from Semtech's `lr_fhss_mac.c` and are **not verified against this chip**. They
are used anyway because the alternative is worse: the TxDone watchdog was a flat 20 ms, which is
*shorter than a single LR-FHSS frame at any setting*, so every transmit would have been reported as
failed after airing perfectly. The watchdog is now `2 × airtime + 20 ms` on every PHY, so a model
wrong by up to 2× degrades into a longer wait rather than a truncated transmit.

### Where the code lives now

| file | what |
|---|---|
| `src/phy.rs` | pure logic, **host-tested**: the `Phy` enum, per-PHY payload/SF/granularity, the band and PA range, hop validation, the LoRa bandwidth code spaces, `PHY_BITMAP` |
| `src/phy_link.rs` | the shared front-end sequence, `PhyState`, the dispatch, per-PHY TX staging and RX read |
| `src/flrc_link.rs` | FLRC's parameters, modem block and framing — and re-exports of what moved, so twenty binaries keep compiling |
| `src/lora_link.rs` | LoRa's parameters, modem block, hopping |
| `src/hoptrace.rs` | pure logic, **host-tested**: the `EVT_HOPTRACE` ring and encoding, the DIO8-sharing rule, the symbol/tick arithmetic the trace is read against |
| `src/lrfhss_link.rs` | LR-FHSS's parameters, frame build, hopping table read/write, the RX arm |

The generalisation is of `flrc_link`'s `LinkState`/`apply` split, **not a copy of it per PHY** — the
one copy of the Semtech-ordered sequence has a hole in the middle where the modulation goes. Three
copies would drift, which is exactly what had already happened once between `configure` and `retune`.

### The bug this reproduced, and it was already in the tracker

The first version used single-byte `Uarte::read` in the same loop that polls the radio over SPI, and
**dropped host commands**: `CMD_GET_INFO` vanished while later commands got through. At 115200 a byte
is ~87 µs and an SPI poll easily exceeds that, so bytes arriving mid-poll were simply lost.

That is the *identical* defect that cost the Waveshare firmware ~50% of its commands — **task #17,
"interrupt/DMA-driven USART RX"**. An interrupt/DMA-backed ring (`BufferedUarte`) is the fix there
and here. **Any host-facing serial loop that also drives SPI needs one**; the unbuffered version
fails intermittently and looks like a host or cabling problem.

## M5 result — hardware-scheduled TX and the guard band

**Mechanism.** The LR2021 supports *DIO TX/RX triggers* natively, so:

```
TIMER20.CC[2] == target ──event──▶ DPPI ──task──▶ GPIOTE OUT sets P1.04
                                                       │
                                     LR2021 DIO8 = DioFunc::TxTrigger → transmit starts
```

No SPI and no CPU between the timer and the transmission. The shield exposes exactly one DIO, so the
TX node trades its IRQ pin for the trigger — acceptable, because completion timing cannot move the
transmit *instant*, and the receiver keeps its IRQ pin and does the timestamping.

**Reliability: 1100 armed slots, 1100 transmits, 0 errors.**

> ⚠ **Taken on the internal RC clock** (see Status). The spread below is a resolution/precision
> figure and stands; it is not an accuracy figure, and the two nodes' relative drift term inside it
> was measured against a source since shown to be ~2000 ppm off. Re-take it through
> `hw::init_peripherals()` before quoting it as anything but a guard-band bound.

**The number**, measured end to end by the M4 receiver over consecutive-slot pairs:

| transmit scheduling | consecutive-pair spread | delivery |
|---|---|---|
| **software** — `Timer::after` + `set_tx()` over SPI (what the Wi-Fi face does today) | **1615 ticks = 100.9 µs** (n=1698) | — |
| **hardware-triggered** — timer compare → DPPI → DIO | **942 ticks = 58.9 µs** (n=1998) | 1 gap in ~2000 |

**What the 58.9 µs is, honestly.** This is a *two-node, end-to-end* figure: it contains the
transmitter's scheduling error, the two nodes' relative oscillator drift, **and the receiving
radio's internal demodulate-to-DIO variability** — which M4 could not isolate, because M4 measured
only the MCU-side path *after* the DIO edge. So 58.9 µs is an upper bound on transmit jitter and the
correct number for sizing a guard band, but it is **not** proof that the transmitter alone is that
loose. Separating the terms needs a wired trigger-to-trigger measurement between the two boards.

**What it means for the MAC (#93).** At FLRC 2.6 Mbit/s a 255-byte frame is ~785 µs of airtime, so a
base slot of `airtime + guard` costs **~7% guard overhead at 58.9 µs**, against ~13% with software
scheduling — and against the Wi-Fi path, where the guard is *milliseconds* and dominates any slot
short enough to be useful. The lease MAC gets **sub-millisecond base slots** on this hardware. The
slot length is set by airtime, not by scheduling error, which is exactly the regime a slot MAC wants.

## Root-causing the dropped slots (#103) — three bugs, none of them the radio

The first attempt transmitted ~68% of armed slots. Reading the chip's own `GetErrors` per slot
settled it immediately: **`chip_busy` never asserted once in 1292 slots.** The prime suspect — the
documented "DIO trigger could not be executed because chip was busy changing mode" — was wrong, and
only the error read disproved it. All three real causes were mine:

1. **Re-arming an already-fired compare.** A 15 ms sleep against a 20 ms period meant roughly one
   iteration in four came round while `target` was still in the future and re-armed a compare that
   had already fired. Writing a CC value the counter has passed produces no event. (`armed`/slots =
   1292/1000 = 1.29, exactly the 20/15 ratio.) Fixed by arming each target exactly once.
2. **Never clearing the TX FIFO.** `wr_tx_fifo_from` *appends*, so any slot that did not transmit
   left its frame behind and the FIFO filled monotonically. This one is worth remembering because it
   presents as a *scheduling* failure while being a buffer-management one: 7% → 72% delivery.
3. **Polling `tx_done` before the transmit had happened.** The fixed sleep frequently woke *before*
   the scheduled instant, read a not-yet-set `tx_done`, and cleared the IRQ — so a perfectly good
   slot was recorded as silent. About a quarter of the apparent failures were the instrument, not
   the radio. Fixed by sleeping until past the target, computed from the hardware clock: 72% → 100%.

### The bug worth carrying into the MAC work

The very first version accumulated the next transmit instant (`target += PERIOD`). It transmitted
**74 times and then stopped for good**: one slow iteration pushed the target into the past, and a
compare armed in the past never matches until the 32-bit counter wraps (~4.5 min at 16 MHz).

That is not a quirk of this binary — **it is exactly the defect a slot scheduler has if it advances
its slot pointer by addition instead of recomputing the next boundary from the common-view clock.**
The fix (re-derive `target` from the clock, and make the sequence number *be* the slot index
`target / PERIOD`) is the same discipline #84/#85 need. Deriving the sequence from the slot also
makes "consecutive sequence numbers" and "consecutive slots" the same statement by construction, so
a skipped slot can never masquerade as transmit jitter.

## M4 result — the RX-timestamp floor

> ⚠ **Taken on the internal RC clock** (see Status). 62.5 ns is the *ruler's* smallest division and
> stays true on any oscillator; what does not survive is any absolute-time or common-view claim built
> on it. Re-take through `hw::init_peripherals()` before quoting accuracy.

Every frame is stamped twice from **one** timer: `CC[0]` by DPPI at the DIO8 edge (no CPU), `CC[1]`
by the CPU when the async task wakes. `CC[1] − CC[0]` is therefore the whole software path, and one
timer means no inter-oscillator drift contaminates it.

At 1 MHz (1 µs/tick):

```
SW-PATH LATENCY  min=30 mean=30 max=31 p2p=1 us        (n=700)
```

`p2p = 1 µs` is *exactly one tick* — the result sat on the quantization floor and could not tell
"1 µs of jitter" from "less than the ruler can see". So the clock was raised 16× and it was re-run:

```
SW-PATH LATENCY  min=494 mean=494 max=494 p2p=0 ticks @62.5 ns   (n=500)
hw inter-arrival min=323190 ticks = 20.2 ms  (matches the 20 ms TX period)
```

**Peak-to-peak zero.** 494 ticks = **30.9 µs, constant to within one 62.5 ns tick over 500 frames**.
16 MHz is the nRF timer's maximum, so 62.5 ns is the instrument's floor, not the signal's.

### What this establishes, and what it does not

- **The DPPI capture path is live and correct.** Two independent checks: the software–hardware delta
  stays *exactly* constant (a stale `CC[0]` would make it grow without bound), and the hardware
  inter-arrival reproduces the 20 ms transmit period (a stale register would give 0).
- **RX-timestamp resolution ≤ 62.5 ns.** For scale, the Wi-Fi path measured ~0.4 µs with the Realtek
  RXTSFL hardware stamp and ~55 µs in software.
- **The honest caveat: the software path was *also* perfect here — 0 ticks of jitter.** That is a
  property of *this workload*, not of software timestamping: one task, an idle 128 MHz M33, an
  identical instruction sequence every time. Under a real MAC — SPI in flight, several tasks, other
  interrupts — the software number will degrade and the hardware number will not. **The value of
  DPPI is not the 30.9 µs offset (a constant offset calibrates out); it is that the hardware figure
  cannot get worse under load.** Re-measure the software path under load before quoting it anywhere.
- Still open, as flagged before the run: the DIO edge marks packet-done *inside* the radio, not the
  first on-air symbol. That offset is constant-looking here but has not been separated.

## M3 result — the on-air FLRC link

```
INFO  m3_rx: FLRC 2477000000 Hz, 2.6 Mbit/s, syncword 0x86244e44 — listening
INFO  m3_rx: FIRST FRAME, seq 82
INFO  m3_rx: got  25 / expected  26 (lost 1), crc_err  4, last seq 107, rssi_raw 190
INFO  m3_rx: got  50 / expected  51 (lost 1), crc_err 12, last seq 132, rssi_raw 179
INFO  m3_rx: got 125 / expected 126 (lost 1), crc_err 31, last seq 207, rssi_raw 196
```

`m3_tx` beacons a 4-byte big-endian sequence number plus a `NDN-M3` tag every 200 ms; `m3_rx` tracks
the sequence, so the result is a **delivery ratio, not a liveness blink** — the loss count stays flat
at 1 across the whole run, i.e. nothing is lost after the initial frame.

**Unexplained, and deliberately not explained away:** `crc_err` climbs steadily (~31 over 126 good
frames) while the *lost payload* count stays at 1. Frames arriving corrupted would show up as
losses, and they do not — so these are most likely **false syncword matches on ambient 2.4 GHz
noise**, not damaged packets. That is a hypothesis, not a measurement; confirm it (e.g. by counting
CRC errors with the transmitter off) before relying on it.

### Band choice — reasoned, not yet measured

2.4 GHz (**HF** port) rather than sub-GHz, because 902–928 MHz on this bench already carries **LoRa
*and* HaLow**, which have been measured interfering there. This board exists to measure microsecond
timing; siting it in the one band with known self-interference would pollute exactly the numbers it
is here to produce. 2477 MHz is the quiet corner — above US Wi-Fi ch11 (~2473; ch12–14 are not
permitted in the US) and below the BLE advertising channel at 2480, so a ~2.4 MHz-wide signal clears
both. **Verify with a spectrum look before trusting timing results taken here.**

## Four traps hit during bring-up, all now pinned in code or config

0. **The LR2021 does not drive its IRQ pin until told to.** M3 polled the IRQ over SPI and never
   needed DIO8, so it sat undriven. M4 then armed a DPPI capture on its edge and measured **nothing**
   — a silent zero-sample result that reads as "hardware timestamping does not work" rather than
   "the interrupt was never routed to the pin". Fixed by `set_dio_irq(Dio8, ...)` inside the shared
   `flrc_link::configure`, where no binary can forget it.
1. **`time-driver-rtc1` is wrong for this part** — nRF54L uses **`time-driver-grtc`** (1 MHz, which
   is conveniently the 1 µs resolution `src/timing.rs` wants). The wrong one still *builds*.
2. **`TICK_HZ` defined twice.** `embassy-time-driver`'s default feature selects `tick-hz-32_768`
   while GRTC selects `tick-hz-1_000_000`. Fixed by depending on `embassy-time-driver` directly with
   `default-features = false`.
3. **`DEFMT_LOG` defaults to `error`** — so `info!`/`debug!` compile to *nothing*, the RTT write
   offset never advances, and the board presents exactly like a dead target: flash verifies, the
   reset handler demonstrably runs, and no output ever appears. Pinned as `DEFMT_LOG = "trace"` in
   `.cargo/config.toml [env]`.

**The bisect that found it is kept as `src/bin/m1_bare.rs`** — bare `cortex-m-rt`, no Embassy, no
peripherals. Start there for any future "the board is dead" scare. The decisive measurement was
reading the SEGGER RTT control block directly over SWD: `WrOff` pinned at 0 while the "SEGGER RTT"
magic was present in RAM, then wiping the magic and resetting and watching it *return* — which
proved the reset handler ran and the core executed, so only the logging could be at fault.

## Build

```sh
cargo build --release
# raw image, if a UF2/DFU path is used instead of probe-rs:
llvm-objcopy -O binary target/thumbv8m.main-none-eabihf/release/lr2021-nrf54l15-rs fw.bin
```

The target is pinned in `.cargo/config.toml` (`thumbv8m.main-none-eabihf` — Cortex-M33 + FPU).
Standalone crate with its own `[workspace]`, like the other firmware here; not in the host workspace.
