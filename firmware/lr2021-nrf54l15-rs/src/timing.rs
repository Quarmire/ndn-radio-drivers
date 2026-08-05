//! **The reason this board exists**: hardware RX timestamping and hardware-scheduled TX, with no
//! CPU in the loop.
//!
//! These are the two primitives commodity Wi-Fi denies us (see
//! `ndn-face-monitor-wifi/docs/named-filter-mac-redesign.md` §8.5, item 1 — the single largest cost
//! of building the named-radio MAC on commodity 802.11). On Wi-Fi we approximate a slot with a
//! host-side sleep plus EDCCA-off, which is *why* guard bands have to be milliseconds wide. Here the
//! nRF54L15's **DPPI** (Distributed Programmable Peripheral Interconnect) wires a peripheral *event*
//! straight to a peripheral *task*, so both operations happen in silicon at a fixed, jitter-free
//! latency — the CPU only reads the result afterwards.
//!
//! Note what this means for the `lr2021` driver's feature gaps: it exposes no packet timestamping
//! and no timed TX, and that turns out **not to matter**, because neither belongs to the radio. Both
//! are MCU-side, and the MCU has dedicated hardware for exactly this.
//!
//! ## RX timestamp — capture the on-air event
//!
//! ```text
//!   LR2021 DIO (rising edge on packet-done)
//!        │
//!   GPIOTE InputChannel  ──event──▶ DPPI channel ──task──▶ TIMER.CC[n].CAPTURE
//!        │                                                      │
//!        └─ also raises the IRQ the async task awaits            └─ CC[n] now holds the
//!                                                                   free-running tick count
//!                                                                   at the instant of the edge
//! ```
//!
//! The captured value is latched by hardware at the edge, so interrupt latency, scheduler jitter
//! and SPI readback time all land *after* the measurement and cannot contaminate it. This is the
//! same idea as the Realtek RXTSFL hardware stamp that gave us ~0.4 µs common-view on Wi-Fi versus
//! ~55 µs in software — but here we own both ends of it.
//!
//! **Honest caveat, to be measured and not assumed:** the DIO edge marks *packet-done inside the
//! radio*, not the first on-air symbol. Between the two sit the radio's demodulation and packet
//! processing. That offset is only a problem if it is *variable*; a constant offset cancels in a
//! two-way exchange and calibrates out in a one-way one. **Measuring the DIO-edge jitter is
//! milestone M4 and is the number this entire board is here to produce.**
//!
//! ## Scheduled TX — the guard band's true floor
//!
//! ```text
//!   TIMER.CC[m] == target tick ──event──▶ DPPI channel ──task──▶ GPIOTE OutputChannel (or an
//!                                                                 SPIM start task) that keys TX
//! ```
//!
//! A slot MAC's guard band has to cover the worst-case error between "when the node intended to
//! transmit" and "when it actually did." Under a host-side sleep that error is the OS scheduler's,
//! i.e. milliseconds. Under a DPPI compare it is the timer's, i.e. one tick. **How much the guard
//! can shrink is milestone M5** — and it decides whether the named airtime lease (#93) can use
//! microsecond base slots or is stuck with millisecond ones.
//!
//! Caveat of the same shape: DPPI can fire a task at an exact tick, but the LR2021 still needs its
//! own TX-start latency after that. The plan is to pre-load the FIFO and pre-configure the packet,
//! so the scheduled event triggers only the final `SetTx`, making the residual latency as close to
//! constant as the part allows. Whether it *is* constant is, again, a measurement.
//!
//! ## Clock choice
//!
//! A 1 MHz timer gives 1 µs resolution and wraps a 32-bit counter every ~71.6 minutes; a 16 MHz
//! timer gives 62.5 ns and wraps every ~4.5 minutes. Start at 1 MHz — it matches the µs common-view
//! vocabulary the rest of the stack already speaks (`ndn-time`, `LinkStamp`, the scheduler epoch) —
//! and only move to 16 MHz if M4 shows the jitter floor is well under a microsecond. Wrap handling
//! is the caller's; the MAC's epochs are far shorter than either wrap period.

use embassy_nrf::timer::Frequency;

/// Timer frequency for the MAC clock: 1 MHz ⇒ 1 tick = 1 µs, matching the µs vocabulary used by
/// `ndn-time` and the face scheduler. See the module note on when to raise it.
/// **16 MHz (62.5 ns/tick), raised from 1 MHz after the first M4 run.**
///
/// At 1 MHz the software-path spread measured `p2p = 1 µs` — which is *exactly one tick*, i.e. the
/// result was at the quantization floor and could not distinguish "1 µs of real jitter" from "less
/// jitter than the ruler can see". Raising the clock 16× is the cheap way to find out which. The
/// cost is wrap period: ~4.5 minutes instead of ~71.6, which is far longer than any MAC epoch.
pub const MAC_CLOCK: Frequency = Frequency::F16MHz;

/// Ticks per microsecond at [`MAC_CLOCK`] — the conversion the MAC layer reasons in.
pub const TICKS_PER_US: u32 = 16;

/// Nanoseconds per tick at [`MAC_CLOCK`] — the true resolution of every [`HwStamp`].
pub const TICK_NS: u32 = 1000 / TICKS_PER_US;

/// A hardware-captured instant on the MAC clock: the raw tick count latched by
/// `TIMER.CC[n].CAPTURE` at a DPPI-routed event edge.
///
/// Deliberately a distinct type from a software timestamp. On the Wi-Fi path the stack learned the
/// hard way that mixing a hardware stamp with a host stamp silently degrades a ~0.4 µs measurement
/// to a ~55 µs one; the type keeps them apart so that cannot happen quietly here.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct HwStamp {
    /// Raw timer ticks, free-running, wraps at 2^32.
    pub ticks: u32,
}

impl HwStamp {
    /// Wrap this raw capture value.
    pub const fn from_ticks(ticks: u32) -> Self {
        Self { ticks }
    }

    /// Microseconds since the timer started, modulo the 32-bit wrap.
    pub const fn micros(self) -> u32 {
        self.ticks / TICKS_PER_US
    }

    /// Ticks elapsed from `earlier` to `self`, correct across a single wrap.
    pub const fn since(self, earlier: HwStamp) -> u32 {
        self.ticks.wrapping_sub(earlier.ticks)
    }
}

// ── The hardware capture path ────────────────────────────────────────────────────────────────────

use embassy_nrf::gpio::Pull;
use embassy_nrf::gpiote::{InputChannel, InputChannelPolarity};
use embassy_nrf::ppi::Ppi;
use embassy_nrf::timer::Timer;

use crate::hw::TimingParts;

/// The RX-timestamp path: a DIO edge captured into a timer register **by hardware**, with the CPU
/// nowhere in the loop.
///
/// Holds the DPPI channel and the timer for their whole lifetime on purpose — dropping either
/// silently disconnects the route, and a silently-disconnected capture returns a stale register
/// value rather than an error. The result would be plausible-looking timestamps that are simply
/// wrong, which is the worst failure mode a measurement instrument can have.
pub struct RxCapture {
    dio: InputChannel<'static>,
    timer: Timer<'static>,
    /// Kept alive to hold the DIO-event → capture-task route open.
    _route: Ppi<'static, embassy_nrf::peripherals::PPI20_CH0, 1, 1>,
}

/// CC register holding the **hardware** capture (written by DPPI at the edge).
const CC_HW: usize = 0;
/// CC register used for the **software** capture (written by the CPU when the task wakes).
const CC_SW: usize = 1;

impl RxCapture {
    /// Wire DIO8's rising edge to `TIMER20.CC[0].CAPTURE` over DPPI and start the clock.
    ///
    /// Rising edge because the LR2021 asserts DIO8 high on an interrupt; the line falls again only
    /// when the IRQ is cleared over SPI, long after the event we care about.
    pub fn new(parts: TimingParts) -> Self {
        let dio = InputChannel::new(parts.gpiote, parts.dio, Pull::Down, InputChannelPolarity::LoToHi);

        let timer = Timer::new(parts.timer);
        timer.set_frequency(MAC_CLOCK); // 1 MHz ⇒ 1 tick = 1 µs
        timer.start();

        // The whole point: peripheral event → peripheral task, no CPU, no interrupt latency.
        let mut route = Ppi::new_one_to_one(parts.ppi, dio.event_in(), timer.cc(CC_HW).task_capture());
        route.enable();

        Self { dio, timer, _route: route }
    }

    /// Await the next DIO edge. The hardware capture has *already* happened by the time this
    /// returns — that is the entire idea. Anything slow that follows cannot corrupt the stamp.
    pub async fn wait_edge(&mut self) {
        self.dio.wait().await;
    }

    /// The value DPPI latched at the edge.
    pub fn hw_stamp(&self) -> HwStamp {
        HwStamp::from_ticks(self.timer.cc(CC_HW).read())
    }

    /// Capture *now*, in software, from the **same timer**.
    ///
    /// Same clock is what makes the comparison honest: reading a second, independent clock would mix
    /// the software path's latency with the relative drift of two oscillators, and at these
    /// magnitudes the drift term is not negligible. One timer, two CC registers, no drift.
    pub fn sw_stamp(&self) -> HwStamp {
        HwStamp::from_ticks(self.timer.cc(CC_SW).capture())
    }

    /// Read the free-running counter without disturbing either capture register.
    pub fn now(&self) -> HwStamp {
        HwStamp::from_ticks(self.timer.cc(5).capture())
    }
}

/// Running min/max/mean of a µs quantity — enough to report a spread without floating point or
/// storing samples on a device with no allocator.
#[derive(Clone, Copy)]
pub struct Spread {
    pub n: u32,
    pub min: u32,
    pub max: u32,
    sum: u64,
}

impl Default for Spread {
    fn default() -> Self {
        Self { n: 0, min: u32::MAX, max: 0, sum: 0 }
    }
}

impl Spread {
    /// Fold in one sample.
    pub fn push(&mut self, v: u32) {
        self.n += 1;
        self.min = self.min.min(v);
        self.max = self.max.max(v);
        self.sum += v as u64;
    }

    /// Mean, truncated. Zero before the first sample.
    pub fn mean(&self) -> u32 {
        if self.n == 0 { 0 } else { (self.sum / self.n as u64) as u32 }
    }

    /// Peak-to-peak — **the number that matters for a MAC guard band**, since a guard must cover
    /// the worst case, not the average.
    pub fn peak_to_peak(&self) -> u32 {
        if self.n == 0 { 0 } else { self.max - self.min }
    }
}
