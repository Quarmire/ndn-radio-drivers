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
pub const MAC_CLOCK: Frequency = Frequency::F1MHz;

/// Ticks per microsecond at [`MAC_CLOCK`] — the conversion the MAC layer reasons in.
pub const TICKS_PER_US: u32 = 1;

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
