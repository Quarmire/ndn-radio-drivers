//! Board bring-up: turn the nRF54L15 peripherals into a ready-to-talk [`Radio`].
//!
//! Lives in the lib rather than in each binary so the pin map is applied in exactly one place. Three
//! milestone binaries wiring SPI independently is precisely how two nodes end up disagreeing about a
//! pin and failing as "the radio never answers".

use embassy_nrf::gpio::{Level, Output, OutputDrive, Pull};
use embassy_nrf::gpio::Input;
use embassy_nrf::{Peri, Peripherals, bind_interrupts, peripherals, spim};

use lr2021::{BusyAsync, Lr2021};

use crate::board;

bind_interrupts!(pub struct Irqs {
    SERIAL00 => spim::InterruptHandler<peripherals::SERIAL00>;
    SERIAL20 => embassy_nrf::buffered_uarte::InterruptHandler<peripherals::SERIAL20>;
});

/// The concrete driver type on this board: GPIO-driven NSS/NRESET, SPIM00, and an async BUSY pin
/// (so a command waits on an edge rather than spinning on a poll).
pub type Radio = Lr2021<Output<'static>, spim::Spim<'static>, BusyAsync<Input<'static>>>;

/// The peripherals the hardware-timestamp path needs, handed out rather than consumed here.
///
/// All four must live in the **same DPPI domain**, and that is not a style preference — a DPPI
/// channel cannot connect peripherals across domains. `P1_04` belongs to `GPIOTE20`, which forces
/// `TIMER20` and a `PPI20_*` channel. Bundling them makes the constraint impossible to violate by
/// picking a peripheral at a call site.
pub struct TimingParts {
    /// LR2021 DIO8 — the radio IRQ line, and the event whose edge is captured in hardware.
    pub dio: Peri<'static, peripherals::P1_04>,
    /// The free-running MAC clock (domain 20).
    pub timer: Peri<'static, peripherals::TIMER20>,
    /// GPIOTE channel that turns the DIO edge into an event (domain 20).
    pub gpiote: Peri<'static, peripherals::GPIOTE20_CH0>,
    /// DPPI channel wiring that event to the timer's capture task (domain 20).
    pub ppi: Peri<'static, peripherals::PPI20_CH0>,
}

/// The host-bridge UART. **UART20 specifically**, because that is the one the XIAO's onboard
/// CMSIS-DAP debug probe bridges to `/dev/ttyACM0` (Zephyr's board dts names it
/// `zephyr,console = &uart20`). The header UART — `uart21` on D6/D7 — goes to the pin header
/// instead, and using it would leave the host with nothing to talk to.
pub struct UartParts {
    pub uart: Peri<'static, peripherals::SERIAL20>,
    /// TX, P1.09.
    pub tx: Peri<'static, peripherals::P1_09>,
    /// RX, P1.08 (the board pulls it up).
    pub rx: Peri<'static, peripherals::P1_08>,
}

/// Claim the pins named in [`crate::board`] and build the radio, handing back the peripherals the
/// timestamp path needs. `dio` is deliberately *not* turned into an `Input` here: at M4 it becomes a
/// GPIOTE `InputChannel` instead, and that constructor wants the raw pin.
pub fn init(p: Peripherals) -> (Radio, TimingParts, UartParts) {
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

    // Assert the board's RF-switch controls, as Zephyr's `regulator-boot-on` nodes do (see
    // `board::PIN_RFSW_CTL` / `PIN_RFSW_PWR`). Leaked deliberately: these must stay asserted for the
    // life of the program, and dropping the `Output` would release the pin mid-experiment.
    #[cfg(not(feature = "no-rf-switch"))]
    {
        core::mem::forget(Output::new(p.P2_05, Level::Low, OutputDrive::Standard)); // rfsw_ctl, active low
        core::mem::forget(Output::new(p.P2_03, Level::High, OutputDrive::Standard)); // rfsw_pwr, active high
    }

    let timing = TimingParts { dio: p.P1_04, timer: p.TIMER20, gpiote: p.GPIOTE20_CH0, ppi: p.PPI20_CH0 };
    let uart = UartParts { uart: p.SERIAL20, tx: p.P1_09, rx: p.P1_08 };
    (Lr2021::new(nreset, busy, spi, nss), timing, uart)
}
