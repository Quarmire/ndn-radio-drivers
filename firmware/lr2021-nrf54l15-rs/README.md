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

**M1 complete — running on both boards** (o5p-0 probe `DDCBDC3E`, o5p-1 probe `8369B83C`).
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
| **M2** | SPI up, `get_version()` answers | yes | the `board` pin map — **most likely first failure** |
| **M3** | FLRC TX↔RX between the two kits | yes | an on-air link at a usable rate |
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

## Before M2 — one unknown left, cheap to resolve and expensive to guess

1. **The pin map in `src/board.rs` is a guess.** It is the XIAO form-factor default, unverified
   against the kit. On this rig a wrong pinout presents as *"the radio never answers"*, which is
   indistinguishable from a dead part and has cost days before. Check continuity or the kit
   schematic and correct the file. `get_version()` (M2) is the test.

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
