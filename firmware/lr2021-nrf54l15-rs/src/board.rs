//! Board pin map — **Seeed XIAO nRF54L15 ↔ Semtech LR2021**.
//!
//! ## ⚠ Every constant here is UNVERIFIED until the board is on the bench
//!
//! This map is the XIAO form-factor default (the standard XIAO SPI trio on D8/D9/D10 plus three
//! control lines). It has **not** been checked against the actual kit, and on this rig guessing a
//! pinout has a near-zero hit rate. **Before the first flash: read continuity or the kit schematic
//! and correct this file.** A wrong map fails as "the radio never answers" — which looks exactly
//! like a dead chip, and has cost this project days before.
//!
//! The LR2021 needs four lines beyond power (per the driver's `Lr2021::new`):
//!
//! | signal  | direction | purpose |
//! |---------|-----------|---------|
//! | `SCK` / `MOSI` / `MISO` | SPI | command + FIFO transport |
//! | `NSS`   | MCU → radio | SPI chip select (driven by us, not the SPIM, so a whole command
//! |         |             | sequence holds CS low) |
//! | `RESET` | MCU → radio | active-low reset |
//! | `BUSY`  | radio → MCU | radio is processing; every command waits for it to fall |
//! | `DIO`   | radio → MCU | IRQ line — **and the timing reference, see [`crate::timing`]** |
//!
//! `DIO_IRQ` is the load-bearing one for this whole testbed: its edge is the on-air event we
//! capture in hardware to get a sub-µs RX timestamp, so it must land on a pin that GPIOTE can
//! observe and DPPI can route.

/// SPI clock. The LR2021 tolerates a fast SPI; start slow and raise it only after the version
/// read is stable — a marginal SPI reads as a flaky radio.
pub const SPI_FREQ_HZ: u32 = 8_000_000;

/// Which of the three SPIM instances to use (SPIM20/21/22 exist on this part). SPIM21 is chosen
/// only because it is unlikely to clash with the XIAO's other default peripherals; re-check when
/// the pin map is confirmed.
pub const SPIM_INSTANCE: &str = "SPIM21";

// ── Pin numbers, as (port, pin) so they can be checked against the schematic at a glance ────────
// P<port>_<pin>. Fill these in from the board, then wire them in `main.rs`.

/// SPI clock — XIAO D8.
pub const PIN_SCK: (u8, u8) = (1, 11);
/// SPI MOSI — XIAO D10.
pub const PIN_MOSI: (u8, u8) = (1, 12);
/// SPI MISO — XIAO D9.
pub const PIN_MISO: (u8, u8) = (1, 13);
/// SPI chip select, driven as a plain GPIO output (active low).
pub const PIN_NSS: (u8, u8) = (1, 14);
/// LR2021 reset, active low.
pub const PIN_RESET: (u8, u8) = (1, 10);
/// LR2021 BUSY, input.
pub const PIN_BUSY: (u8, u8) = (1, 9);
/// LR2021 DIO used as the IRQ + hardware-timestamp reference. See [`crate::timing`].
pub const PIN_DIO_IRQ: (u8, u8) = (1, 8);
