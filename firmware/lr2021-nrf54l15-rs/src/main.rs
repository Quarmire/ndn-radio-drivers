//! Firmware for the **Seeed XIAO nRF54L15 + Semtech LR2021** — the named-radio MAC testbed.
//!
//! Why this board rather than just another LoRa node: it removes both constraints commodity Wi-Fi
//! imposes on the MAC design (`ndn-face-monitor-wifi/docs/named-filter-mac-redesign.md` §8.5).
//!
//! 1. **Hardware-scheduled TX and hardware RX timestamping** via nRF54L15 DPPI + TIMER, no CPU in
//!    the loop — see [`timing`]. On Wi-Fi a slot is faked with a host-side sleep, which is exactly
//!    why guard bands there must be milliseconds wide.
//! 2. **FLRC at up to 2.6 Mbit/s** (`FlrcBitrate::Br2600`). Pure LoRa at SF7/125 kHz is ~5.5 kbit/s,
//!    so a 256 B frame is ~370 ms and slot structure is not testable at all. At FLRC the same frame
//!    is well under a millisecond.
//! 3. **Full frame control** — no 802.11 header, so the Tier-0 prefix-set name filter is not pinned
//!    at the 94 bits an 802.11 address field allows and its sizing curve can be measured properly.
//! 4. The **FLPR RISC-V coprocessor** is literally NDN-NIC's "constrained NIC microcontroller",
//!    which that paper simulated and never built. Running the name filter there is a later milestone.
//!
//! ## Bring-up milestones — each gated on the previous one *measuring*, not compiling
//!
//! | | milestone | needs board | proves |
//! |---|---|---|---|
//! | **M0** | builds for `thumbv8m.main-none-eabihf` | no | toolchain: embassy-nrf `nrf54l15-app-s` + the `lr2021` driver resolve and link |
//! | **M1** | blink / RTT on the XIAO | yes | the flash + debug path (UF2 bootloader? SWD? — unknown, see `memory.x`) |
//! | **M2** | SPI up, `get_version()` answers | yes | the [`board`] pin map is right — the step most likely to fail first |
//! | **M3** | FLRC TX↔RX between the two kits | yes | an on-air link at a usable rate |
//! | **M4** | DPPI+TIMER RX capture, jitter measured | yes | **the number this board exists for**: the RX-timestamp floor |
//! | **M5** | DPPI+TIMER scheduled TX, error measured | yes | the guard-band floor ⇒ whether the lease MAC (#93) gets µs or ms base slots |
//! | **M6** | 7E-A5 serial bridge + `ndn-embedded` data plane | yes | parity with the Waveshare/Heltec nodes so all five interoperate |
//! | **M7** | Tier-0 prefix-set filter on the FLPR coprocessor | yes | the NDN-NIC architecture in real silicon |
//!
//! **Current state: M0.** Nothing below M2 has been near hardware, and the [`board`] pin map is a
//! guess that must be checked against the kit before the first flash.

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_nrf::gpio::{Input, Level, Output, OutputDrive, Pull};
use embassy_nrf::{bind_interrupts, peripherals, spim};
use embassy_time::{Duration, Instant, Timer};

use defmt_rtt as _;
use panic_probe as _;

pub mod board;
pub mod timing;

bind_interrupts!(struct Irqs {
    SERIAL00 => spim::InterruptHandler<peripherals::SERIAL00>;
});

/// **M1** — prove the flash → run → debug-I/O path end to end.
///
/// Deliberately RTT rather than an LED blink: RTT needs no knowledge of the board's pin map, and the
/// pin map is exactly the thing still unverified ([`board`]). This isolates "can we build, flash and
/// observe the target" from "is the wiring right", so an M2 failure has only one possible cause.
///
/// It also proves the embassy time driver is running: the printed uptime must advance ~1 s per line.
/// A frozen counter means the clock/time-driver feature is wrong, not the radio.
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());

    defmt::info!("lr2021-nrf54l15-rs M1: target alive, RTT up, embassy time driver running");

    // ── M2: SPI to the LR2021 ────────────────────────────────────────────────────────────────
    // Pin map is from Semtech's + Zephyr's devicetrees, not guessed — see [`board`].
    let mut cfg = spim::Config::default();
    cfg.frequency = spim::Frequency::M8; // board::SPI_FREQ_HZ; the shield's ceiling is 16 MHz
    cfg.mode = spim::MODE_0; // LR2021: CPOL=0, CPHA=0
    // SPIM00 (SERIAL00) because the shield's SPI lands on P2.x — see [`board`].
    let spi = spim::Spim::new(
        p.SERIAL00, Irqs, p.P2_01, /* SCK, D8 */
        p.P2_04, /* MISO, D9 */ p.P2_02, /* MOSI, D10 */
        cfg,
    );

    // NSS and NRESET are both active-low, driven by us rather than by the SPIM so that a whole
    // command sequence can hold CS low. Start NSS high (deselected) and NRESET high (not in reset).
    let nss = Output::new(p.P1_07, Level::High, OutputDrive::Standard);
    let nreset = Output::new(p.P1_06, Level::High, OutputDrive::Standard);
    // Pulls follow the shield devicetree exactly: BUSY pulls up, DIO8/IRQ pulls down.
    let busy = Input::new(p.P1_05, Pull::Up);
    let _dio_irq = Input::new(p.P1_04, Pull::Down); // M4 will route this edge through DPPI

    let mut radio = lr2021::Lr2021::new(nreset, busy, spi, nss);

    match radio.reset().await {
        Ok(()) => defmt::info!("M2: LR2021 reset asserted"),
        Err(e) => defmt::error!("M2: reset FAILED: {}", defmt::Debug2Format(&e)),
    }
    Timer::after(Duration::from_millis(50)).await;

    // The M2 pass/fail: a chip that answers on SPI returns its version. A wrong pin map, a wrong
    // SPI mode, or a part held in reset all show up here as an error or an implausible value.
    match radio.get_version().await {
        // Printed field by field rather than as an opaque blob: an all-zero or all-0xff version is
        // the classic signature of a mis-wired or unpowered SPI, and it must not look like a pass.
        Ok(mut v) => defmt::info!(
            "M2 PASS: LR2021 firmware {}.{} (raw major={=u8:#04x} minor={=u8:#04x}) status_ok={}",
            v.major(),
            v.minor(),
            v.major(),
            v.minor(),
            v.status().is_ok()
        ),
        Err(e) => defmt::error!("M2 FAIL: get_version: {}", defmt::Debug2Format(&e)),
    }

    let mut tick = 0u32;
    loop {
        Timer::after(Duration::from_secs(1)).await;
        tick += 1;
        defmt::info!(
            "tick {} uptime {} ms busy={}",
            tick,
            Instant::now().as_millis(),
            radio.is_busy()
        );
    }
}
