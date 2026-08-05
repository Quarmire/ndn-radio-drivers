//! Board bring-up: turn the nRF54L15 peripherals into a ready-to-talk [`Radio`].
//!
//! Lives in the lib rather than in each binary so the pin map is applied in exactly one place. Three
//! milestone binaries wiring SPI independently is precisely how two nodes end up disagreeing about a
//! pin and failing as "the radio never answers".

use embassy_nrf::gpio::{Input, Level, Output, OutputDrive, Pull};
use embassy_nrf::{Peripherals, bind_interrupts, peripherals, spim};

use lr2021::{BusyAsync, Lr2021};

use crate::board;

bind_interrupts!(pub struct Irqs {
    SERIAL00 => spim::InterruptHandler<peripherals::SERIAL00>;
});

/// The concrete driver type on this board: GPIO-driven NSS/NRESET, SPIM00, and an async BUSY pin
/// (so a command waits on an edge rather than spinning on a poll).
pub type Radio = Lr2021<Output<'static>, spim::Spim<'static>, BusyAsync<Input<'static>>>;

/// Claim the pins named in [`crate::board`] and build the radio. Also returns the DIO8/IRQ input:
/// it is the radio's interrupt line *and* the hardware-timestamp reference [`crate::timing`] will
/// route through DPPI at M4, so it is handed back rather than consumed here.
pub fn init(p: Peripherals) -> (Radio, Input<'static>) {
    let mut cfg = spim::Config::default();
    cfg.frequency = spim::Frequency::M8; // board::SPI_FREQ_HZ; shield ceiling is 16 MHz
    cfg.mode = spim::MODE_0; // LR2021: CPOL=0, CPHA=0
    let _ = board::SPI_FREQ_MAX_HZ;

    // SPIM00 (SERIAL00) because the shield's SPI lands on P2.x — see [`crate::board`].
    let spi = spim::Spim::new(
        p.SERIAL00, Irqs, p.P2_01, /* SCK,  D8  */
        p.P2_04, /* MISO, D9  */ p.P2_02, /* MOSI, D10 */
        cfg,
    );

    // NSS and NRESET are active-low and driven by us, not by the SPIM, so a whole command sequence
    // can hold CS low. Both start high: deselected, and not held in reset.
    let nss = Output::new(p.P1_07, Level::High, OutputDrive::Standard);
    let nreset = Output::new(p.P1_06, Level::High, OutputDrive::Standard);
    // Pulls follow the shield devicetree exactly: BUSY pulls up, DIO8/IRQ pulls down.
    let busy = Input::new(p.P1_05, Pull::Up);
    let dio_irq = Input::new(p.P1_04, Pull::Down);

    (Lr2021::new(nreset, busy, spi, nss), dio_irq)
}
