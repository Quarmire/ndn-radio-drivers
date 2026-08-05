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
| **M4** | DPPI+TIMER RX capture, **jitter measured** | yes | the RX-timestamp floor — *the number this board exists for* |
| **M5** | DPPI+TIMER scheduled TX, **error measured** | yes | the guard-band floor ⇒ µs or ms base slots for the lease MAC (#93) |
| **M6** | 7E-A5 serial bridge + `ndn-embedded` data plane | yes | parity with the Waveshare/Heltec nodes so all five interoperate |
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

## Three traps hit during bring-up, all now pinned in config

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
