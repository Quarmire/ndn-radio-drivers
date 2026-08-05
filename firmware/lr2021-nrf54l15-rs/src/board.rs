//! Board pin map — **Seeed XIAO nRF54L15 ↔ Semtech LR2021 LoRa Plus expansion board**.
//!
//! **Source of truth, not a guess.** Composed from two authoritative devicetrees:
//!
//! - Semtech's own shield overlay, `boards/shields/semtech_wio_lr20xx/semtech_wio_lr20xx_common.dtsi`
//!   in [`Lora-net/usp_zephyr`](https://github.com/Lora-net/usp_zephyr) — gives the LR2021 control
//!   lines in XIAO `D<n>` terms, their polarities and pulls, and the SPI ceiling.
//! - Zephyr's XIAO board definition, `boards/seeed/xiao_nrf54l15/seeed_xiao_connector.dtsi` and
//!   `xiao_nrf54l15-pinctrl.dtsi` — maps `D<n>` onto nRF54L15 port/pin and fixes the SPI instance.
//!
//! Recorded because the earlier placeholder here was the XIAO form-factor *default*, and it was
//! wrong on **every single line** — different port, different pins, different order. On this rig a
//! wrong pinout presents as "the radio never answers", which is indistinguishable from a dead part.
//!
//! | signal | XIAO | nRF54L15 | polarity / pull | notes |
//! |---|---|---|---|---|
//! | `DIO8` (IRQ) | D0 | **P1.04** | active high, pull-down | LR2021 DIO8 is the IRQ line, `IRQ_ALL_MASK`. **Also the hardware-timestamp reference** — see [`crate::timing`] |
//! | `BUSY` | D1 | **P1.05** | active high, pull-up | every command waits on this |
//! | `NRESET` | D2 | **P1.06** | active **low** | |
//! | `NSS` | D3 | **P1.07** | active **low** | driven by us as a GPIO, not by the SPIM, so a whole command holds CS low |
//! | `SCK` | D8 | **P2.01** | — | `SPIM00` |
//! | `MOSI` | D10 | **P2.02** | — | |
//! | `MISO` | D9 | **P2.04** | — | |
//!
//! The SPI pins sit on **P2.x**, which is why the instance must be **`SERIAL00`/`SPIM00`** (the
//! high-speed one, clocked from the 128/64 MHz PLL domain) rather than one of the `SERIAL2x`
//! peripherals. `embassy-nrf` implements `SPIM00` only under its `_s` feature, which
//! `nrf54l15-app-s` provides.
//!
//! Other facts from the shield devicetree, for later milestones: `reg-mode = DCDC`,
//! `lf-clk = RC`, `tcxo-voltage = 1.8 V` with `tcxo-wakeup-time = 0`, `rx-boost-cfg = 7`,
//! `tx-power-offset = 0`, and calibration frequencies 470 MHz / 897.5 MHz / 2441 MHz. The board has
//! two SMA ports: **LF** (sub-GHz, 150–960 MHz) and **HF** (2.4 GHz ISM + S-band).

/// SPI clock for bring-up. Semtech's devicetree sets `spi-max-frequency = 16 MHz`; start at half
/// that and raise it only once `get_version()` is stable — a marginal SPI reads as a flaky radio.
pub const SPI_FREQ_HZ: u32 = 8_000_000;

/// The SPI ceiling the shield devicetree declares. Do not exceed.
pub const SPI_FREQ_MAX_HZ: u32 = 16_000_000;

// ── RF switch control ────────────────────────────────────────────────────────────────────────────

/// **RF-switch control pins the board expects firmware to drive.**
///
/// Zephyr's board dts (`boards/seeed/xiao_nrf54l15/xiao_nrf54l15_nrf54l15_cpuapp.dts`) declares
/// both as `regulator-fixed` with `regulator-boot-on`, i.e. Zephyr asserts them at startup:
///
/// ```text
///   rfsw_ctl: enable-gpios = <&gpio2 5 GPIO_ACTIVE_LOW>;   // P2.05 — assert = drive LOW
///   rfsw_pwr: enable-gpios = <&gpio2 3 GPIO_ACTIVE_HIGH>;  // P2.03 — assert = drive HIGH
/// ```
///
/// M0–M6 ran without driving either and still linked at ~99%, so the pins evidently settle
/// somewhere usable — but "usable" was an inference, not a measurement, and the switch sits in the
/// path every RSSI and TX-power number is read through. Driven by default now; build with
/// `--features no-rf-switch` to leave them floating for the A/B.
/// Assert = drive LOW.
pub const PIN_RFSW_CTL: (u8, u8) = (2, 5);
/// Assert = drive HIGH.
pub const PIN_RFSW_PWR: (u8, u8) = (2, 3);
