//! **The RX-timestamp contract, without the registers.** Wrap extension, edge attribution, and the
//! `stamp_kind` byte EVT_CAP puts on the wire.
//!
//! Everything here is pure arithmetic and pure decision. That is deliberate: the register poking
//! lives in `rxstamp` (device-only, untestable off-target), and the three things that are actually
//! easy to get *wrong* — extending a 16-bit counter, reconstructing an instant backwards from the
//! ISR, and deciding whether a captured edge may be attributed to the frame being reported — live
//! here where `cargo test --target <host> --lib` can run them. The split is the same one the LR2021
//! firmware makes between `airtime`/`serial` (host-testable wire contracts) and `timing`/`hw`
//! (device-only), and for the same reason.
//!
//! ## What the hardware path buys, and what it does not
//!
//! Until now `EVT_RX.ts` was `micros()` read in the poll loop *after* `poll_rx` had finished an SPI
//! `GetIrqStatus`, a `ClearIrqStatus`, a `GetRxBufferStatus`, the whole buffer readback and a
//! `GetPacketStatus`. At SCK = 1 MHz that is `(19 + n) × 8 µs` of SPI — ~280 µs for a 16-byte frame,
//! ~2.1 ms for a 247-byte one — **before** the poll-loop phase and any blocking command handler are
//! counted. Note the shape of that: it is not jitter around a constant, it is a bias that GROWS WITH
//! FRAME LENGTH, and a length-coupled bias is exactly the term that cannot cancel in a two-way
//! exchange. (Those figures are arithmetic from `poll_rx`'s transaction sizes, not measurements;
//! they are a floor, since real per-byte HAL overhead only adds.)
//!
//! The capture removes that term by taking the number in silicon at the DIO1 edge. It does **not**
//! improve the resolution: one tick is 1 µs before and after, because [`STAMP_HZ`] is the unit the
//! rest of the stack already speaks. The gain is entirely accuracy.
//!
//! ## ☠ What is NOT measured between the frame arriving in the air and this tick
//!
//! Stated as terms, with nothing folded into the number. Anyone differencing two nodes' stamps is
//! differencing all of these too.
//!
//! | term | status |
//! |---|---|
//! | propagation TX antenna → RX antenna | 3.34 ns/m; below one tick at bench range |
//! | antenna → RF switch → LNA → mixer → IF filter group delay | **NOT MEASURED.** Positive, and not constant across bandwidth — IF group delay scales roughly as 1/BW, so it MOVES on any `CMD_SET_MOD` that changes BW. Any calibration would be per-(SF,BW) and void after a mode change |
//! | demodulation | `RxDone` marks the END of the packet — after the last symbol and the CRC check — not the first on-air symbol. Recovering a start-of-frame instant needs the time-on-air subtracted; `airtime_ms_for` computes one, in whole ms and from the reported length. That is a DERIVATION and is deliberately not folded in here |
//! | packet-done → DIO1 assertion inside the SX1262 | **NOT MEASURED**, and not specified by Semtech. Believed sub-symbol, bounded by nothing |
//! | DIO1 pad → PB0 trace | ns; below one tick |
//! | GPIO synchroniser + the timer's `IC3F` input filter | 125–250 ns at 8 MHz, fixed by construction; a bias, not jitter |
//! | quantisation | one tick = 1 µs |
//! | **tick-rate accuracy** | ★ the MCU runs on the **8 MHz HSI RC oscillator** — `rcc.cfgr.freeze()` is called with no HSE — and it clocks both SysTick and the capture timer. Every figure here is in NOMINAL microseconds; the rate itself is untrimmed, ~1%. The SX1262's 32 MHz TCXO clocks the RADIO, not this counter. For a cross-node common view this term dominates all the others put together, and it is **NOT MEASURED** |
//!
//! If the demodulate-and-flag offset is constant it cancels in a two-way exchange and calibrates out
//! in a one-way one; if it varies it is a floor this timer cannot lift. Measuring it is a separate
//! job from building the capture, and this module reports the capture *only*.

/// Capture-clock rate: 1 MHz ⇒ one tick is one microsecond.
///
/// Deliberately equal to the `STAMP_HZ` on the wire and to TIM2's scheduler tick, so `EVT_RX.ts`,
/// `EVT_CLOCK`, `CMD_TX_AT_ABS` and the deadline compare are all the same unit on the same counter
/// and no conversion exists to get wrong. Raising it would rescale three wire fields and leave
/// `CMD_TX_AT`'s microseconds behind — see the note in `main.rs`.
pub const STAMP_HZ: u32 = 1_000_000;

/// One full turn of the 16-bit capture timer: 65 536 ticks = 65.536 ms at [`STAMP_HZ`].
pub const WRAP_TICKS: u64 = 1 << 16;

/// `stamp_kind` — the fleet's four-value vocabulary for *how good `EVT_RX.ts` actually is*.
///
/// Mirrored from `firmware/lr2021-nrf54l15-rs/src/serial.rs`, which is where the fleet writes it
/// down, and decoded on the host by `ndn-radio-drivers`' `StampKind::from_code`. Named here rather
/// than written as a literal at the emitter, because this byte is the ONE thing that makes the host
/// publish `LatchPoint::RadioCapture` and set `can_common_view` — a stray `3` claims a measurement
/// this node might not be making.
pub mod stamp_kind {
    /// No per-frame stamp at all; `stamp_hz` is 0.
    pub const NONE: u8 = 0;
    /// Latched when the serial line delivered the frame to the host.
    pub const HOST_RECV: u8 = 1;
    /// A counter incremented by firmware — carries poll/interrupt latency.
    pub const SOFTWARE_COUNTER: u8 = 2;
    /// Latched in silicon at the radio's own event edge, free-running counter.
    pub const HARDWARE_FREE_RUNNING: u8 = 3;
}

/// EVT_CAP field offsets this module owns. The record is fixed-width and the host slices it by
/// offset, so the two fields that describe the timestamp are named once and written by
/// [`encode_stamp_fields`] rather than indexed by hand at the emitter.
pub const CAP_STAMP_HZ_OFF: usize = 12;
/// See [`CAP_STAMP_HZ_OFF`].
pub const CAP_STAMP_KIND_OFF: usize = 16;

/// **Extend the 16-bit counter to the free-running tick count.**
///
/// `wraps` is the overflow count the timer ISR maintains, `cnt` the counter read, and `wrap_pending`
/// the timer's own overflow flag *as read after `cnt`*. That third argument is the whole reason this
/// is a function rather than a shift-and-or.
///
/// Two distinct races exist, and the classic "read hi, read lo, read hi again" closes only the first:
///
/// 1. *The wrap ISR runs during the read.* The caller catches that by re-reading `wraps` and
///    retrying. Bounded — a wrap happens 15.26 times a second.
/// 2. *A wrap has happened and its ISR has not run yet*, because it is pending behind another
///    interrupt at the same priority. Then `wraps` is **stale and consistent on both sides**, so (1)
///    does not see it. `wrap_pending` is read for exactly this case, and `cnt < WRAP/2` decides the
///    epoch: a small count with a wrap pending belongs to the NEW epoch, while a large count with a
///    wrap pending means the wrap landed after `cnt` was sampled and must not be added.
///
/// **Validity condition, stated in the quantity it actually depends on.** Exact so long as the
/// TIM3 *update ISR* is serviced within half a wrap (32.768 ms) of the overflow it reports. The
/// READER's own duration does not enter: the epoch decision is made from `cnt` and `wrap_pending`,
/// both sampled by the reader, and it is `wraps` being stale by more than one epoch — i.e. an
/// overflow that has been pending, unserviced, for longer than 32.768 ms — that makes the
/// `cnt < WRAP/2` rule pick the wrong epoch and return a whole wrap low. (An earlier version of
/// this comment said "so long as the caller's read is short... it is about ten instructions",
/// which invites the wrong conclusion — that the clock is safe BECAUSE the read is quick.)
///
/// What actually supports the claim is the same property [`LAT_SANE_US`] leans on: this firmware
/// contains no `interrupt::free` and no critical section anywhere, both ISRs are a handful of
/// instructions, and nothing sets an NVIC priority — so the TIM3 update is serviced in
/// microseconds, four orders of magnitude inside the bound.
pub const fn extend(wraps: u32, cnt: u16, wrap_pending: bool) -> u64 {
    let hi = if wrap_pending && (cnt as u64) < WRAP_TICKS / 2 {
        wraps.wrapping_add(1)
    } else {
        wraps
    };
    ((hi as u64) << 16) | cnt as u64
}

/// **Reconstruct the capture instant, backwards from the clock read inside the ISR.**
///
/// Returns `(stamp, latency)`: the extended tick count of the edge, and how many ticks elapsed
/// between the edge and the ISR reading the clock.
///
/// The hard case in a naive implementation is a capture at `CNT = 0xFFF0` immediately followed by a
/// wrap: both flags are set in one read and `(wraps << 16) | ccr` is a whole 65.536 ms too late.
/// This does not try to order the flags. `now` already has the wrap folded in, `now as u16` *is*
/// `CNT`, so the low-16 difference is exactly the elapsed ticks and subtracting it from `now` is
/// automatically correct across a wrap boundary with no epoch bookkeeping at all.
///
/// The latency is also a free and useful measurement: it is **how wrong a software stamp taken in
/// the ISR would have been**, which is the number that says whether this path was worth building.
pub const fn reconstruct(now: u64, cc: u16) -> (u64, u32) {
    let lat = (now as u16).wrapping_sub(cc) as u32;
    (now.wrapping_sub(lat as u64), lat)
}

/// Ceiling on the ISR-entry latency [`reconstruct`] may report before the value is thrown away.
///
/// **Half a wrap, and that is a boundary rather than a taste.** The low-16 subtraction is exact for
/// any true latency below one full wrap (65 536 µs) and silently aliases above it — a true 70 000 µs
/// would read as 4 464 and be indistinguishable from a real one, so no threshold can *detect* that
/// case. What half a wrap does mark is where the value stops being self-consistent: past it, "the
/// edge was just before now" and "the edge was nearly a whole turn ago" are the same number.
///
/// The reason the undetectable case does not happen is a property of the firmware, not of this
/// constant: nothing here masks interrupts (there is no `interrupt::free` and no critical section
/// anywhere in the crate) and both ISRs are a handful of instructions, so entry latency is
/// microseconds. The watermark is reported in EVT_STATS so that claim is checkable on air rather
/// than merely asserted here.
pub const LAT_SANE_US: u32 = (WRAP_TICKS / 2) as u32;

/// Why a frame's timestamp fell back to the software read. Wire values — they travel in
/// `EVT_RX_STAMP` byte 1 — so the numbering is a contract and 0 is reserved for "not degraded".
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum Degrade {
    /// No DIO1 edge was captured in this window at all, yet the chip reported `RxDone`. Either the
    /// capture channel missed the edge or some path cleared the chip IRQ without opening a window.
    NoEdge = 1,
    /// The timer's overcapture flag was set: a second edge landed while the first was unread, so the
    /// **earlier value is gone**. That means an edge was LOST, which is categorically worse than a
    /// stamp being imprecise, and it is reported rather than papered over.
    Overcapture = 2,
    /// More than one edge was captured since the window opened. See the ☠ note on [`classify`]: the
    /// register holds the LATEST edge, so the value may belong to a later frame than the one being
    /// reported. A plausible timestamp on the wrong frame is the worst failure a measurement
    /// instrument has, so it is discarded.
    MultipleEdges = 3,
    /// The ISR-entry latency exceeded [`LAT_SANE_US`], so the backward reconstruction cannot be
    /// trusted to have stayed inside one wrap.
    IsrLatency = 4,
    /// The boot self-test did not pass, so this node is not claiming a hardware capture at all and
    /// `stamp_kind` stays [`stamp_kind::SOFTWARE_COUNTER`]. Present so that a node with a failed
    /// self-test still reports *per frame* why its stamps are software, instead of going quiet.
    NoCapturePath = 5,
    /// ★ **The chip completed a number of packets in this window other than exactly one**, so the
    /// captured edge and the payload about to be reported cannot be shown to describe the same
    /// reception. See [`attribute`] — this is the ONE failure the edge counters are structurally
    /// blind to, because a frame arriving while DIO1 is already high raises no edge at all while
    /// the chip's buffer, length and packet status all advance to describe it.
    Coalesced = 6,
}

impl Degrade {
    /// The wire byte.
    pub const fn code(self) -> u8 {
        self as u8
    }
}

/// The outcome of asking "may this capture be attributed to the frame I am about to report?".
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum StampVerdict {
    /// A capture that satisfies every rule, on the free-running tick timebase.
    Hardware(u64),
    /// No usable capture. The caller reports the software read instead, **and says so on the wire**.
    Software(Degrade),
}

impl StampVerdict {
    /// The captured ticks, or `None` when the frame must fall back.
    pub const fn ticks(self) -> Option<u64> {
        match self {
            StampVerdict::Hardware(t) => Some(t),
            StampVerdict::Software(_) => None,
        }
    }

    /// Why it fell back, or `None` when it did not.
    pub const fn degrade(self) -> Option<Degrade> {
        match self {
            StampVerdict::Hardware(_) => None,
            StampVerdict::Software(d) => Some(d),
        }
    }

    /// The `stamp_kind` that is true **of this frame** — which is not always the one EVT_CAP
    /// advertises for the node. This is the value `EVT_RX_STAMP` carries.
    pub const fn frame_stamp_kind(self) -> u8 {
        match self {
            StampVerdict::Hardware(_) => stamp_kind::HARDWARE_FREE_RUNNING,
            StampVerdict::Software(_) => stamp_kind::SOFTWARE_COUNTER,
        }
    }
}

/// **The attribution rule, in one place.**
///
/// > The capture supplies the INSTANT. The chip's IRQ status word supplies the REASON. Neither is
/// > valid without the other, and they describe the same event only if they were read in the same
/// > IRQ-clear cycle.
///
/// The "same cycle" half is structural and is enforced by the *call site*, not by this function:
/// `Sx1262::poll_rx` reads the capture between `GetIrqStatus` and `ClearIrqStatus`, in the window
/// where DIO1 is provably still high and therefore no second rising edge can physically exist; and
/// `Sx1262::clear_irq` opens a fresh window at every one of the eleven sites that close one (ten
/// `self.clear_irq(...)` calls plus `poll_rx`'s `clear_irq_from`), so the timer's window and the
/// chip's IRQ latch are the same window by construction rather than by discipline. What is left for
/// this function is everything countable about that window.
///
/// ☠ **This function alone is NOT the attribution rule, and believing it was is how a frame's
/// payload got published with an earlier frame's edge.** DIO1 is level-latched: a second frame
/// arriving while the line is still high raises **no edge**, so the counts this function sees are
/// exactly `edges == 1, over == 0` — indistinguishable from a clean single reception — while the
/// chip's buffer pointer, `payloadLengthRx` and `GetPacketStatus` have all moved on to describe the
/// LATER frame. Nothing in the timer can see that. [`attribute`] closes it with the chip's own
/// completed-packet count, and every caller must apply both.
///
/// ☠ **`CCR3` holds the LATEST edge — the opposite of the LR2021's `CC[0]`.** On an STM32/GD32
/// input capture an overcapture *overwrites* the register and raises `CCxOF`; the LR2021's DPPI
/// capture keeps the FIRST edge because DIO8 stays high. The two firmwares agree in effect only
/// because both DIO lines latch until cleared, so the LR2021's doc wording must not be carried
/// across. Here the hazard is a stamp that is too FRESH — it can belong to a later frame.
///
/// Arguments: `edges` and `overcaptures` are counted since the window opened, `lat_us` is the
/// latency [`reconstruct`] reported for the newest capture, and `ticks` is that capture.
pub const fn classify(edges: u32, overcaptures: u32, lat_us: u32, ticks: u64) -> StampVerdict {
    // Order is by severity of what the value would mean if it were believed, not by cheapness of
    // the test. "An edge was lost" outranks "there were two", which outranks "the arithmetic is
    // out of its validity range".
    if edges == 0 {
        return StampVerdict::Software(Degrade::NoEdge);
    }
    if overcaptures != 0 {
        return StampVerdict::Software(Degrade::Overcapture);
    }
    if edges > 1 {
        return StampVerdict::Software(Degrade::MultipleEdges);
    }
    if lat_us > LAT_SANE_US {
        return StampVerdict::Software(Degrade::IsrLatency);
    }
    StampVerdict::Hardware(ticks)
}

/// **The chip-side half of the attribution rule: exactly ONE packet may have completed in the
/// window that supplied the edge.**
///
/// ★ This is the fix for the failure the module exists to prevent, and it is worth stating why the
/// timer cannot do it alone. DIO1 stays high from an `RxDone` until the `ClearIrqStatus` goes out
/// over SPI. So during the ~22 ms an `EVT_RX` push occupies the USART, a frame **B** raises the
/// line and is captured; a frame **C** arriving while it is still high raises nothing at all, and
/// the chip's buffer, length and packet status advance to describe C. The next `poll_rx` then sees
/// one edge, no overcapture and a small latency — [`classify`] returns `Hardware(t_B)` — and reads
/// C's payload. That is a *plausible* timestamp on the *wrong* frame, wrong by up to a whole poll
/// gap (22 ms here; ~78 ms after any `SET_*` TCXO restart; seconds behind an SF12 relay), published
/// under a `stamp_kind` byte that claims 1 µs.
///
/// `pkt_delta` is the number of receptions the CHIP counted between the instant this window opened
/// and the instant it closed, read from its own `GetStats` — the one counter on this part that
/// advances for a coalesced frame. One packet is the only attributable case:
///
/// * `== 1` — the single edge and the single packet are the same event. Attributable.
/// * `>= 2` — coalesced: the edge is the first frame's, the payload is the last frame's. Discard.
/// * `== 0` — the reported packet was already counted when the window opened, i.e. the reception
///   straddles the window boundary. Not reachable in normal operation (the baseline is snapshotted
///   *before* the `ClearIrqStatus` that opens the window, so a packet that sets `RxDone` afterwards
///   is always new), and treated as ambiguous rather than assumed benign.
///
/// **Which way this errs is deliberate.** The baseline is read a few tens of microseconds *before*
/// the clear, so a frame landing in that gap is counted in the NEXT window's delta rather than
/// this one's: the count can therefore run one HIGH and never one low. High costs a good stamp;
/// low would admit a wrong one. (Reading the baseline after the clear inverts exactly that, which
/// is why it is not done.)
///
/// An already-degraded verdict keeps its original reason — it is falling back either way, and the
/// timer's reason is the more specific one.
pub const fn attribute(v: StampVerdict, pkt_delta: u16) -> StampVerdict {
    if pkt_delta == 1 {
        return v;
    }
    match v {
        StampVerdict::Hardware(_) => StampVerdict::Software(Degrade::Coalesced),
        already_degraded => already_degraded,
    }
}

/// **Does this frame need an `EVT_RX_STAMP` note?**
///
/// Only when the frame's own `stamp_kind` differs from the one EVT_CAP advertises for the node. The
/// event's whole meaning is "this frame is not what the capability record says", so on a node whose
/// boot self-test failed — which advertises `SOFTWARE_COUNTER` and means it — a note on every frame
/// would repeat the capability rather than qualify it, and would put a second frame on the wire for
/// every reception on the one node least able to afford it.
pub const fn note_needed(v: StampVerdict, advertised_kind: u8) -> bool {
    v.frame_stamp_kind() != advertised_kind
}

/// The `stamp_kind` byte this node may honestly advertise.
///
/// `hw_capture_live` is the boot self-test's verdict, not "the capture module compiled". A `3` makes
/// the host publish `LatchPoint::RadioCapture`, a 1 µs `stamp_precision_ns` instead of the 1 ms
/// host-receive floor, and `can_common_view = true` — a 1000× tightening of the number the
/// timekeeper believes. It is not a claim to make on the strength of the code building.
pub const fn stamp_kind_byte(hw_capture_live: bool) -> u8 {
    if hw_capture_live {
        stamp_kind::HARDWARE_FREE_RUNNING
    } else {
        stamp_kind::SOFTWARE_COUNTER
    }
}

/// Write `stamp_hz` (u32 BE) and `stamp_kind` into an EVT_CAP record.
///
/// Trivial, and it exists anyway: these two fields must agree — a `stamp_kind` of 0 with a non-zero
/// rate, or a hardware kind with a rate of 0, is a record the host reads as a contradiction — and
/// keeping the pair in one function is what lets a host test pin them together.
///
/// ★ **What `stamp_hz` is a claim about**: the counter `micros64()` reads, whichever
/// [`ClockSource`] [`choose_timebase`] selected — never "the rate TIM3 was programmed for". Those
/// two came apart exactly once, and the result was a detected 2 MHz timer still shipping
/// 1 000 000 on the wire. They cannot come apart now: the capture timer is only ever the clock when
/// its rate was MEASURED at [`STAMP_HZ`] (or when nothing could measure it, in which case the kind
/// drops to [`stamp_kind::SOFTWARE_COUNTER`] and nothing common-views the field), and the SysTick
/// fallback counts microseconds by construction.
pub fn encode_stamp_fields(cap: &mut [u8], stamp_hz: u32, kind: u8) {
    cap[CAP_STAMP_HZ_OFF..CAP_STAMP_HZ_OFF + 4].copy_from_slice(&stamp_hz.to_be_bytes());
    cap[CAP_STAMP_KIND_OFF] = kind;
}

// A kind that claims a per-frame stamp must come with a rate to interpret it in. Pinned at build
// time because the host's `tick_ns()` is `Some` only when `stamp_hz > 0`, and its common-view
// predicate is `HardwareFreeRun && tick_ns().is_some()` — a hardware kind with a zero rate would
// advertise a capability and then fail the predicate silently.
const _: () = assert!(STAMP_HZ > 0);
const _: () = assert!(stamp_kind_byte(true) == stamp_kind::HARDWARE_FREE_RUNNING);
const _: () = assert!(stamp_kind_byte(false) == stamp_kind::SOFTWARE_COUNTER);
// The reason codes are a wire contract; 0 stays reserved for "not degraded".
const _: () = assert!(Degrade::NoEdge.code() == 1);
const _: () = assert!(Degrade::NoCapturePath.code() == 5);
const _: () = assert!(Degrade::Coalesced.code() == 6);

// =====================================================================================================
// Which counter is allowed to be the node's microsecond clock
// =====================================================================================================

/// Ticks the capture timer must count in one SysTick millisecond, nominal — [`STAMP_HZ`] / 1000.
pub const TICKS_PER_MS_NOMINAL: u16 = (STAMP_HZ / 1_000) as u16;
/// Accepted band for the measured rate. ±2 % — tight, because the reference is SysTick rather than
/// a cycle-counting delay loop and the only slack needed is the handful of instructions between
/// noticing the tick and reading `CNT`. What it must catch is a prescaler or clock-tree error,
/// which is a factor of **2 or 8**.
pub const TICKS_PER_MS_MIN: u16 = 980;
/// See [`TICKS_PER_MS_MIN`].
pub const TICKS_PER_MS_MAX: u16 = 1_020;

/// Which counter [`STAMP_HZ`] describes — i.e. which one the node's `micros64()` actually reads.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ClockSource {
    /// TIM3, whose rate was **measured** in band at boot. The capture register and the clock are
    /// then the same counter with the same epoch, which is what lets a captured edge be published
    /// in the same units `CMD_READ_CLOCK` answers in and `CMD_TX_AT_ABS` schedules against.
    CaptureTimer,
    /// The SysTick-derived microsecond clock — the pre-capture timebase, kept as a fallback.
    SysTick,
}

/// **The boot verdict, split into the two independent things it decides.**
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Timebase {
    /// The counter `micros64()` reads, and therefore the one [`STAMP_HZ`] on the wire describes.
    pub clock: ClockSource,
    /// May this node advertise [`stamp_kind::HARDWARE_FREE_RUNNING`] and return
    /// [`StampVerdict::Hardware`] per frame?
    pub hw_stamp: bool,
    /// Was the capture timer's rate actually measured in band, or is the declared rate unbacked?
    pub rate_verified: bool,
}

/// **Decide the timebase from the boot self-test — the one place a rate fault is allowed to act.**
///
/// ☠ The bug this exists to make impossible: the rate check was measured, folded into a single
/// `passed()` boolean, and the only thing that boolean reached was one capability byte. A TIM3
/// ticking at 2 MHz would therefore have lowered `stamp_kind` to 2 — honest about the LATCH POINT —
/// and gone right on being the node's microsecond clock, while `EVT_CAP.stamp_hz` still said
/// 1 000 000. `EVT_CLOCK`, `EVT_RX.ts`, `CMD_TX_AT`'s delay, `CMD_TX_AT_ABS`'s deadline, `late_us`,
/// `keyup_us` and the re-published `sched_gran_ns` would all have been 2× wrong with nothing on the
/// wire saying so. A DETECTED fault must not ship a number.
///
/// The two verdicts are independent and are kept independent:
///
/// * the four **flag-semantics** checks say whether an edge can be latched and read back — they
///   decide the *capture*, and a failure there leaves the counter a perfectly good clock;
/// * the **rate** check says whether this counter's ticks are microseconds at all — it decides the
///   *clock*, and a failure there invalidates every field derived from it.
///
/// So a rate failure moves the clock back to SysTick, exactly reproducing the behaviour that
/// preceded the capture path (`micros64` was `MILLIS * 1000 + (RELOAD − CVR)/8`, structurally immune
/// to an APB1 timer-tree fault), and refuses the hardware stamp — a capture in TIM3 ticks cannot be
/// published on a SysTick timebase, since the two counters have different epochs even when both are
/// 1 MHz.
///
/// **The one case that cannot fall back**: `ticks_per_ms == 0` means SysTick never moved, so the
/// *reference* is dead, not the timer. Falling back would freeze `micros64()` — deadlines that never
/// fire, an `EVT_CLOCK` that never advances — which is worse than an unverified rate. The capture
/// timer stays the clock (it is the only counter still running), the hardware stamp is refused, and
/// [`rate_verified`](Timebase::rate_verified) is false so the boot log says the rate is unbacked
/// rather than implying it was checked.
pub const fn choose_timebase(flags_ok: bool, ticks_per_ms: u16) -> Timebase {
    let rate_verified = ticks_per_ms >= TICKS_PER_MS_MIN && ticks_per_ms <= TICKS_PER_MS_MAX;
    let reference_dead = ticks_per_ms == 0;
    Timebase {
        clock: if rate_verified || reference_dead {
            ClockSource::CaptureTimer
        } else {
            ClockSource::SysTick
        },
        hw_stamp: flags_ok && rate_verified,
        rate_verified,
    }
}

// A hardware stamp may only be claimed on the counter the clock is actually read from. Pinned at
// build time because the failure it prevents — a capture published in one counter's ticks against
// another counter's epoch — is invisible to every unit check on the wire.
const _: () = assert!(!choose_timebase(true, 2_000).hw_stamp);
const _: () = assert!(matches!(
    choose_timebase(true, 2_000).clock,
    ClockSource::SysTick
));
const _: () = assert!(choose_timebase(true, TICKS_PER_MS_NOMINAL).hw_stamp);
const _: () = assert!(matches!(
    choose_timebase(true, TICKS_PER_MS_NOMINAL).clock,
    ClockSource::CaptureTimer
));
// A dead reference cannot verify anything, and must not freeze the clock either.
const _: () = assert!(!choose_timebase(true, 0).hw_stamp);
const _: () = assert!(!choose_timebase(true, 0).rate_verified);
const _: () = assert!(matches!(
    choose_timebase(true, 0).clock,
    ClockSource::CaptureTimer
));
// Flag semantics decide the capture only; they never move the clock.
const _: () = assert!(matches!(
    choose_timebase(false, TICKS_PER_MS_NOMINAL).clock,
    ClockSource::CaptureTimer
));
const _: () = assert!(!choose_timebase(false, TICKS_PER_MS_NOMINAL).hw_stamp);

/// **Per-frame outcomes of the hardware RX stamp**, reported in the EVT_STATS v3 tail.
///
/// [`hw`](Self::hw) and [`sw`](Self::sw) partition every frame `poll_rx` delivered — counted before
/// the data plane classifies it, so a frame the on-device filter drops still lands here.
///
/// [`ambig`](Self::ambig) is the subset of `sw` that was discarded for **mis-attribution**: either
/// the window held more than one edge ([`Degrade::MultipleEdges`]) or the chip completed a number
/// of packets other than one ([`Degrade::Coalesced`]). Both are the same failure — a *plausible*
/// timestamp that would have been reported against the WRONG frame — which is the failure the whole
/// capture path exists to prevent, so they share one counter and it is the number that says whether
/// the window discipline is holding on air.
///
/// ⚠ **`ambig` changed meaning when coalescing became detectable per frame.** It used to count only
/// multi-edge windows, and the coalesced case — the common one on a busy channel — was not counted
/// anywhere; the README told the operator to infer it by differencing `hw + sw` against the chip's
/// own `nbPktReceived`. That aggregate is still worth watching (it also catches frames the firmware
/// never saw at all), but it is no longer the only detector, and it never was a per-frame one.
#[derive(Clone, Copy, Default, PartialEq, Eq, Debug)]
pub struct Tally {
    /// Frames whose `ts` is the hardware capture.
    pub hw: u16,
    /// Frames that fell back to the software read. Each one that was *delivered to the host* was
    /// also announced individually, by an `EVT_RX_STAMP` immediately before its `EVT_RX`.
    pub sw: u16,
    /// ...of which, discarded because the capture could not be attributed to this frame: more than
    /// one DIO1 edge in the window, or a chip packet count that did not advance by exactly one.
    pub ambig: u16,
}

impl Tally {
    /// Fold one frame's verdict in.
    ///
    /// **Saturating, not wrapping.** These counters are the evidence for a capability claim, and a
    /// counter that silently rolls over to 0 reads as "it never happened" — which is precisely the
    /// reading they exist to prevent. A pinned `u16::MAX` is obviously a ceiling; a 3 is not
    /// obviously a wrap.
    pub fn observe(&mut self, v: StampVerdict) {
        match v.degrade() {
            None => self.hw = self.hw.saturating_add(1),
            Some(d) => {
                self.sw = self.sw.saturating_add(1);
                if matches!(d, Degrade::MultipleEdges | Degrade::Coalesced) {
                    self.ambig = self.ambig.saturating_add(1);
                }
            }
        }
    }

    /// `CMD_RESET_STATS`.
    pub fn reset(&mut self) {
        *self = Self::default();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ── extend(): the two races, and the ordinary case ───────────────────────────────────────────

    #[test]
    fn extend_is_a_plain_concatenation_when_nothing_is_pending() {
        assert_eq!(extend(0, 0, false), 0);
        assert_eq!(extend(0, 1234, false), 1234);
        assert_eq!(extend(1, 0, false), 65_536);
        assert_eq!(extend(3, 0xFFFF, false), 3 * 65_536 + 65_535);
    }

    #[test]
    fn a_pending_wrap_with_a_small_count_belongs_to_the_next_epoch() {
        // The ISR has not run, so `wraps` is stale by one. A count just past the top means the
        // counter has already turned over and the stamp must advance a whole wrap.
        assert_eq!(extend(4, 7, true), 5 * 65_536 + 7);
        assert_eq!(extend(0, 0, true), 65_536);
    }

    #[test]
    fn a_pending_wrap_with_a_large_count_does_not_advance_the_epoch() {
        // Here the wrap landed AFTER `cnt` was sampled: the count still belongs to the old epoch and
        // adding one would jump the clock 65.536 ms into the future. This is the case a naive
        // "UIF set ⇒ add a wrap" gets wrong.
        assert_eq!(extend(4, 0xFFF0, true), 4 * 65_536 + 0xFFF0);
        assert_eq!(extend(0, 0x8000, true), 0x8000);
    }

    #[test]
    fn the_epoch_split_is_exactly_half_a_wrap() {
        assert_eq!(extend(1, 0x7FFF, true), 2 * 65_536 + 0x7FFF); // last count of the new epoch
        assert_eq!(extend(1, 0x8000, true), 1 * 65_536 + 0x8000); // first count of the old
    }

    #[test]
    fn extend_is_monotone_across_a_real_wrap_sequence() {
        // Walk a wrap the way the hardware does it: count climbs, overflows, the flag is pending for
        // a few reads, then the ISR lands and bumps `wraps`. The reconstructed clock must never go
        // backwards across that whole sequence.
        let mut prev = 0u64;
        let seq: [(u32, u16, bool); 7] = [
            (7, 0xFFFC, false),
            (7, 0xFFFE, false),
            (7, 0xFFFF, false),
            (7, 0x0000, true), // wrapped; ISR still pending
            (7, 0x0003, true),
            (8, 0x0005, false), // ISR ran
            (8, 0x0009, false),
        ];
        for (w, c, p) in seq {
            let t = extend(w, c, p);
            assert!(t >= prev, "went backwards: {prev} -> {t}");
            prev = t;
        }
        assert_eq!(prev, 8 * 65_536 + 9);
    }

    // ── reconstruct(): backwards from the ISR, including across a wrap ───────────────────────────

    #[test]
    fn reconstruct_subtracts_the_isr_entry_latency() {
        let now = 5 * 65_536 + 1_000;
        let (stamp, lat) = reconstruct(now, 1_000 - 12);
        assert_eq!(lat, 12);
        assert_eq!(stamp, now - 12);
    }

    #[test]
    fn reconstruct_is_correct_when_the_edge_was_before_a_wrap_and_the_isr_after_it() {
        // The case that defeats "(wraps << 16) | ccr": the edge landed at 0xFFF0 in epoch 5, the
        // counter wrapped, and the ISR read the clock at 0x0004 of epoch 6.
        let now = 6 * 65_536 + 4;
        let (stamp, lat) = reconstruct(now, 0xFFF0);
        assert_eq!(lat, 20);
        assert_eq!(stamp, 5 * 65_536 + 0xFFF0);
        assert!(stamp < now);
    }

    #[test]
    fn reconstruct_reports_zero_latency_for_a_capture_at_the_current_count() {
        let now = 42 * 65_536 + 77;
        assert_eq!(reconstruct(now, 77), (now, 0));
    }

    // ── classify(): the attribution / discard rule ───────────────────────────────────────────────

    #[test]
    fn one_clean_edge_is_attributable() {
        assert_eq!(classify(1, 0, 9, 12_345), StampVerdict::Hardware(12_345));
        assert_eq!(
            classify(1, 0, 9, 12_345).frame_stamp_kind(),
            stamp_kind::HARDWARE_FREE_RUNNING
        );
    }

    #[test]
    fn no_edge_is_never_reported_as_a_hardware_stamp() {
        let v = classify(0, 0, 0, 999);
        assert_eq!(v, StampVerdict::Software(Degrade::NoEdge));
        assert_eq!(v.ticks(), None);
        assert_eq!(v.frame_stamp_kind(), stamp_kind::SOFTWARE_COUNTER);
    }

    #[test]
    fn a_second_edge_discards_the_stamp_rather_than_attributing_the_later_one() {
        // ☠ The register holds the LATEST edge. Two edges in one window means the value on offer may
        // belong to a later frame, so it is thrown away — never reported as approximate.
        assert_eq!(
            classify(2, 0, 3, 500),
            StampVerdict::Software(Degrade::MultipleEdges)
        );
    }

    #[test]
    fn an_overcapture_outranks_the_edge_count_because_it_means_an_edge_was_lost() {
        assert_eq!(
            classify(2, 1, 3, 500),
            StampVerdict::Software(Degrade::Overcapture)
        );
    }

    #[test]
    fn a_latency_past_half_a_wrap_is_not_reconstructable() {
        assert_eq!(classify(1, 0, LAT_SANE_US, 7), StampVerdict::Hardware(7));
        assert_eq!(
            classify(1, 0, LAT_SANE_US + 1, 7),
            StampVerdict::Software(Degrade::IsrLatency)
        );
    }

    #[test]
    fn every_degraded_verdict_carries_a_distinct_nonzero_wire_reason() {
        let all = [
            Degrade::NoEdge,
            Degrade::Overcapture,
            Degrade::MultipleEdges,
            Degrade::IsrLatency,
            Degrade::NoCapturePath,
        ];
        for (i, a) in all.iter().enumerate() {
            assert_ne!(a.code(), 0, "0 is reserved for 'not degraded'");
            for b in &all[i + 1..] {
                assert_ne!(a.code(), b.code());
            }
        }
    }

    // ── the EVT_CAP bytes ────────────────────────────────────────────────────────────────────────

    #[test]
    fn evt_cap_stamp_fields_encode_big_endian_at_the_fleet_offsets() {
        let mut cap = [0u8; 34];
        encode_stamp_fields(&mut cap, STAMP_HZ, stamp_kind_byte(true));
        // 1_000_000 = 0x000F_4240, big-endian at [12..16]; kind at [16].
        assert_eq!(&cap[12..16], &[0x00, 0x0F, 0x42, 0x40]);
        assert_eq!(cap[16], 3);
        // Nothing either side is touched — the record is fixed-width and sliced by offset.
        assert_eq!(cap[11], 0);
        assert_eq!(cap[17], 0);
    }

    #[test]
    fn a_failed_self_test_advertises_the_software_kind_it_actually_has() {
        let mut cap = [0u8; 34];
        encode_stamp_fields(&mut cap, STAMP_HZ, stamp_kind_byte(false));
        assert_eq!(cap[16], stamp_kind::SOFTWARE_COUNTER);
        // The RATE is unchanged, and that is now a THEOREM rather than an assertion: this record is
        // only ever emitted with `STAMP_HZ` because `choose_timebase` guarantees the clock behind it
        // ticks at STAMP_HZ — a measured-in-band TIM3, or SysTick. Zeroing stamp_hz here would tell
        // the host there is no per-frame stamp at all, which is a different and false claim.
        assert_eq!(&cap[12..16], &STAMP_HZ.to_be_bytes());
        assert!(matches!(
            choose_timebase(false, TICKS_PER_MS_NOMINAL).clock,
            ClockSource::CaptureTimer
        ));
    }

    #[test]
    fn a_note_is_emitted_only_when_the_frame_disagrees_with_the_capability() {
        let hw = StampVerdict::Hardware(1);
        let sw = StampVerdict::Software(Degrade::NoEdge);
        // A node advertising the hardware capture: only the fallback needs announcing.
        assert!(!note_needed(hw, stamp_kind::HARDWARE_FREE_RUNNING));
        assert!(note_needed(sw, stamp_kind::HARDWARE_FREE_RUNNING));
        // A node whose self-test failed advertises the software kind and means it — every frame
        // already matches the capability, so nothing is announced. This is the case that would
        // otherwise double the event rate on the one node least able to afford it.
        assert!(!note_needed(sw, stamp_kind::SOFTWARE_COUNTER));
    }

    #[test]
    fn the_hardware_kind_is_the_one_the_host_gates_common_view_on() {
        // `ndn-radio-drivers`' predicate is `stamp_kind == HardwareFreeRun && stamp_hz > 0`. Pinned
        // here so a renumbering of the fleet vocabulary fails a test rather than an on-air run.
        assert_eq!(stamp_kind::HARDWARE_FREE_RUNNING, 3);
        assert_eq!(stamp_kind::SOFTWARE_COUNTER, 2);
        assert!(STAMP_HZ > 0);
    }

    // ── the tally ────────────────────────────────────────────────────────────────────────────────

    #[test]
    fn the_tally_partitions_every_frame_into_hardware_or_software() {
        let mut t = Tally::default();
        t.observe(StampVerdict::Hardware(1));
        t.observe(StampVerdict::Hardware(2));
        t.observe(StampVerdict::Software(Degrade::NoEdge));
        t.observe(StampVerdict::Software(Degrade::MultipleEdges));
        t.observe(StampVerdict::Software(Degrade::Overcapture));
        assert_eq!(t.hw, 2);
        assert_eq!(t.sw, 3);
        assert_eq!(
            t.hw + t.sw,
            5,
            "hw + sw must be every frame poll_rx delivered"
        );
        assert_eq!(
            t.ambig, 1,
            "ambig counts only MultipleEdges, and is a subset of sw"
        );
        assert!(t.ambig <= t.sw);
    }

    #[test]
    fn the_tally_saturates_rather_than_wrapping_to_zero() {
        // A counter that rolls over reads as "it never happened", which is the exact misreading
        // these counters exist to prevent.
        let mut t = Tally {
            hw: u16::MAX,
            sw: u16::MAX,
            ambig: u16::MAX,
        };
        t.observe(StampVerdict::Hardware(0));
        t.observe(StampVerdict::Software(Degrade::MultipleEdges));
        assert_eq!(t.hw, u16::MAX);
        assert_eq!(t.sw, u16::MAX);
        assert_eq!(t.ambig, u16::MAX);
    }

    // ── attribute(): the chip's packet count, i.e. the coalescing detector ───────────────────────

    #[test]
    fn exactly_one_completed_packet_is_the_only_attributable_window() {
        let v = classify(1, 0, 4, 12_345);
        assert_eq!(attribute(v, 1), StampVerdict::Hardware(12_345));
    }

    #[test]
    fn a_coalesced_window_discards_a_stamp_classify_thinks_is_clean() {
        // ★ The regression this closes. Frame B raises the edge; frame C arrives while DIO1 is
        // still high and raises NOTHING, so the timer reports one clean edge — and the payload
        // about to be reported is C's. Without the chip's count this is `Hardware(t_B)` on C's
        // bytes, admitted at 1 us and wrong by up to a whole poll gap.
        let clean = classify(1, 0, 4, 12_345);
        assert_eq!(clean, StampVerdict::Hardware(12_345));
        assert_eq!(
            attribute(clean, 2),
            StampVerdict::Software(Degrade::Coalesced)
        );
        assert_eq!(attribute(clean, 7), StampVerdict::Software(Degrade::Coalesced));
        assert_eq!(attribute(clean, 2).ticks(), None);
        assert_eq!(
            attribute(clean, 2).frame_stamp_kind(),
            stamp_kind::SOFTWARE_COUNTER
        );
    }

    #[test]
    fn a_window_whose_packet_never_advanced_is_ambiguous_too() {
        // delta == 0 means the reported reception was already counted when the window opened, i.e.
        // it straddles the boundary. Not reachable while the baseline is snapshotted before the
        // ClearIrq, and not assumed benign either.
        assert_eq!(
            attribute(classify(1, 0, 4, 9), 0),
            StampVerdict::Software(Degrade::Coalesced)
        );
    }

    #[test]
    fn attribution_keeps_the_more_specific_timer_reason_when_both_fail() {
        // The frame falls back either way; the reason the operator gets should be the one that
        // names a mechanism (an edge was LOST) rather than the generic one.
        for d in [Degrade::NoEdge, Degrade::Overcapture, Degrade::MultipleEdges] {
            assert_eq!(
                attribute(StampVerdict::Software(d), 3),
                StampVerdict::Software(d)
            );
        }
    }

    #[test]
    fn a_degraded_verdict_is_never_promoted_by_a_healthy_packet_count() {
        // delta == 1 must not resurrect a stamp the timer already refused.
        assert_eq!(
            attribute(StampVerdict::Software(Degrade::NoEdge), 1),
            StampVerdict::Software(Degrade::NoEdge)
        );
    }

    #[test]
    fn a_coalesced_frame_is_counted_as_ambiguous_not_merely_software() {
        // `ambig` is "a plausible timestamp would have gone on the wrong frame". Coalescing is that
        // failure, so it belongs in the same counter as a multi-edge window.
        let mut t = Tally::default();
        t.observe(attribute(classify(1, 0, 4, 1), 2));
        t.observe(attribute(classify(2, 0, 4, 2), 1));
        t.observe(attribute(classify(1, 0, 4, 3), 1));
        assert_eq!((t.hw, t.sw, t.ambig), (1, 2, 2));
        assert!(t.ambig <= t.sw);
    }

    // ── choose_timebase(): a DETECTED rate fault must not ship a number ───────────────────────────

    #[test]
    fn a_measured_in_band_rate_puts_the_clock_and_the_capture_on_one_counter() {
        let tb = choose_timebase(true, 1_000);
        assert_eq!(tb.clock, ClockSource::CaptureTimer);
        assert!(tb.hw_stamp);
        assert!(tb.rate_verified);
    }

    #[test]
    fn a_doubled_timer_rate_moves_the_clock_back_to_systick_and_refuses_the_stamp() {
        // ★ The regression this closes. A prescaler/clock-tree error is a factor of 2 or 8; before
        // the split, the self-test DETECTED it, lowered one capability byte, and then let the 2 MHz
        // counter go on being EVT_CLOCK, EVT_RX.ts and both CMD_TX_AT deadlines at a declared 1 MHz.
        for tps in [2_000u16, 8_000, 500, 125, 979, 1_021] {
            let tb = choose_timebase(true, tps);
            assert_eq!(tb.clock, ClockSource::SysTick, "tps={tps}");
            assert!(!tb.hw_stamp, "tps={tps}");
            assert!(!tb.rate_verified, "tps={tps}");
        }
    }

    #[test]
    fn the_band_edges_are_inclusive() {
        assert!(choose_timebase(true, TICKS_PER_MS_MIN).rate_verified);
        assert!(choose_timebase(true, TICKS_PER_MS_MAX).rate_verified);
        assert!(!choose_timebase(true, TICKS_PER_MS_MIN - 1).rate_verified);
        assert!(!choose_timebase(true, TICKS_PER_MS_MAX + 1).rate_verified);
        assert!(TICKS_PER_MS_MIN <= TICKS_PER_MS_NOMINAL && TICKS_PER_MS_NOMINAL <= TICKS_PER_MS_MAX);
    }

    #[test]
    fn a_dead_reference_keeps_the_only_live_clock_but_claims_nothing_for_it() {
        // ticks_per_ms == 0 is SysTick not moving — the REFERENCE is dead, not the timer. Falling
        // back would freeze micros64() outright: deadlines that never fire, an EVT_CLOCK that never
        // advances. So the capture timer stays the clock, and the rate is reported as unverified.
        let tb = choose_timebase(true, 0);
        assert_eq!(tb.clock, ClockSource::CaptureTimer);
        assert!(!tb.hw_stamp);
        assert!(!tb.rate_verified);
    }

    #[test]
    fn a_flag_semantics_failure_costs_the_capture_and_not_the_clock() {
        // The four flag checks say whether an edge can be latched and read back. A failure there
        // leaves the counter a perfectly good microsecond clock, so nothing about the timebase moves.
        let tb = choose_timebase(false, 1_000);
        assert_eq!(tb.clock, ClockSource::CaptureTimer);
        assert!(!tb.hw_stamp);
        assert!(tb.rate_verified);
    }

    #[test]
    fn a_hardware_stamp_is_only_ever_claimed_on_the_counter_the_clock_reads() {
        // The invariant that makes `stamp_hz` honest in every branch: a capture is published in
        // TIM3 ticks, so it may only be claimed when TIM3 is also what micros64() returns.
        for flags in [false, true] {
            for tps in [0u16, 1, 500, 979, 980, 1_000, 1_020, 1_021, 2_000, 8_000, 65_535] {
                let tb = choose_timebase(flags, tps);
                if tb.hw_stamp {
                    assert_eq!(tb.clock, ClockSource::CaptureTimer, "flags={flags} tps={tps}");
                    assert!(tb.rate_verified, "flags={flags} tps={tps}");
                    assert!(flags, "flags={flags} tps={tps}");
                }
            }
        }
    }

    #[test]
    fn a_classified_window_feeds_the_tally_end_to_end() {
        // The two halves this module owns, composed the way the firmware composes them: a window's
        // raw counts -> a verdict -> a tally line. One clean edge, then a second edge in the next
        // window, then nothing at all.
        let mut t = Tally::default();
        t.observe(classify(1, 0, 4, 1_000));
        t.observe(classify(2, 0, 4, 2_000));
        t.observe(classify(0, 0, 0, 0));
        assert_eq!((t.hw, t.sw, t.ambig), (1, 2, 1));
    }
}
