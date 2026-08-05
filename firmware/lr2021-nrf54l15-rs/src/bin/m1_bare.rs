//! **M1a — the bisect binary.** Bare `cortex-m-rt`, no Embassy, no HAL, no peripherals touched.
//!
//! Kept permanently rather than deleted after use: it is the reference answer to *"is the target
//! running and is RTT readable?"*, and every future "the board is dead" scare should start here
//! before anything else is suspected.
//!
//! It exists because the first M1 attempt flashed cleanly and printed nothing. Reading the RTT
//! control block directly showed `WrOff == 0` and not advancing — so the reset handler HAD run (the
//! `.data` copy put the "SEGGER RTT" magic in RAM) but execution never reached the first log call.
//! That isolates the fault to `embassy_nrf::init()` or later, and this binary proves the half of the
//! system that is *not* at fault.
//!
//! Run: `probe-rs run --chip nRF54L15 target/thumbv8m.main-none-eabihf/release/m1_bare`

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt_rtt as _;
// Linked but never called: `cortex-m-rt` runs with the `device` feature (nrf-pac enables it), so it
// expects the device crate to supply `__INTERRUPTS`. Importing the HAL provides that vector table
// without executing a single line of it — which is the whole point of this binary.
use embassy_nrf as _;
use panic_probe as _;

#[entry]
fn main() -> ! {
    // First thing, before anything can fault: if this line appears, the core runs and RTT works.
    defmt::info!("m1_bare: core alive, RTT up, no HAL initialised");

    let mut n: u32 = 0;
    loop {
        // Pure busy-wait — no timer peripheral, no clock configuration, nothing that could trap.
        cortex_m::asm::delay(64_000_000);
        n += 1;
        defmt::info!("m1_bare tick {}", n);
    }
}
