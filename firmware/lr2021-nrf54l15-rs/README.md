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
| **M4** | ✅ DPPI+TIMER RX capture, jitter measured | yes | the RX-timestamp floor: **≤62.5 ns**, below instrument resolution |
| **M5** | ✅ scheduled TX, 1100/1100 slots | yes | guard-band floor **58.9 µs** (vs 100.9 µs software) ⇒ sub-ms base slots for #93 |
| **M6** | ✅ 7E-A5 bridge on `/dev/ttyACM0` | yes | parity with the Waveshare/Heltec nodes — all five are one fleet |
| **M7** | Tier-0 prefix-set filter on the FLPR coprocessor | yes | the NDN-NIC architecture in real silicon (#91) |

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

Two deliberate differences from the Waveshare node, both documented in `src/serial.rs`:

- **`EVT_RX.ts_us` carries the M4 hardware capture** (DPPI-latched at the DIO edge, 62.5 ns), not a
  software millisecond counter. Same field, far better number — stamp precision is a per-node
  property the host should read rather than assume.
- **LoRa-only commands are answered with `EVT_UNSUPPORTED`, not ignored.** A host that assumes a
  spreading-factor knob gets an error instead of silence: a diagnosable bug rather than a mystery.

The on-device NDN data plane is **shared by path** with `waveshare-lora-rs` (`#[path]` to its
`ndn.rs`), not copied. Filter/dedup/relay semantics must be byte-identical across nodes that
interoperate; two implementations agreeing today would drift, and the failure mode — one node
silently dropping traffic its neighbour forwards — is indistinguishable from a link problem.

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
