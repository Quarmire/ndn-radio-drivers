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

**M0 complete — builds and links.** Nothing below M2 has been near hardware.

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
| **M1** | blink / RTT | yes | the flash + debug path |
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
  - **FLRC is "tested"** and `FlrcBitrate::Br2600` confirms 2.6 Mbit/s. `FlrcCr::{Cr12,Cr34,Cr23}`.
  - **LoRa is only "partial"** — matters for interop with the SX1262/SX1276 nodes (M3+).
  - CAD *is* present (`set_cad_params`/`set_cad`), and `set_rx_duty_cycle(listen, cycle,
    use_lora_cad, dram_ret)` gives the duty-cycled wake path task #90 needs.
  - **No packet timestamping and no timed TX** — and this does *not* matter, because neither
    belongs to the radio. Both are MCU-side, via DPPI (see `src/timing.rs`).
- `cortex-m` needs `critical-section-single-core`, or the link fails on
  `_critical_section_1_0_acquire`.

## Before the first flash — two unknowns, both cheap to resolve, both expensive to guess

1. **The pin map in `src/board.rs` is a guess.** It is the XIAO form-factor default, unverified
   against the kit. On this rig a wrong pinout presents as *"the radio never answers"*, which is
   indistinguishable from a dead part and has cost days before. Check continuity or the kit
   schematic and correct the file. `get_version()` (M2) is the test.
2. **The flash path is unknown.** If the XIAO ships a UF2/MBR bootloader, `memory.x`'s
   `FLASH ORIGIN` must move past it and `LENGTH` shrink to match; if it is bare SWD, the current
   map is right. Read the board's bootloader map — do not assume.

Tooling not yet installed on this host: `probe-rs` (for SWD flash + RTT). `cargo install probe-rs-tools`.

## Build

```sh
cargo build --release
# raw image, if a UF2/DFU path is used instead of probe-rs:
llvm-objcopy -O binary target/thumbv8m.main-none-eabihf/release/lr2021-nrf54l15-rs fw.bin
```

The target is pinned in `.cargo/config.toml` (`thumbv8m.main-none-eabihf` — Cortex-M33 + FPU).
Standalone crate with its own `[workspace]`, like the other firmware here; not in the host workspace.
