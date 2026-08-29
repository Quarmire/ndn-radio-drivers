//! **Hardware RX timestamping on the GD32: TIM3_CH3 input capture on the SX1262's DIO1 line.**
//!
//! The timer latches its counter in silicon at the DIO1 edge. Interrupt entry, the SPI status
//! round-trip, the buffer readback and the 22 ms EVT_RX push all happen *after* the number already
//! exists and cannot contaminate it. That — and only that — is what this path buys; see
//! [`crate::capture`] for the size of the term it removes and for the list of terms it does not.
//!
//! Same construction as the LR2021 board's `TIMER20.CC[0]` latched by DPPI
//! (`lr2021-nrf54l15-rs/src/timing.rs`), built from what an STM32F1-class timer has instead of
//! DPPI: a capture channel whose trigger input is the pin itself. No CPU in the loop either way.
//!
//! ## The window is the chip's IRQ-clear cycle, by construction
//!
//! DIO1 stays high from the event until a `ClearIrqStatus` goes out over SPI — exactly like the
//! LR2021's DIO8 — so "edges since the last ClearIrq" and "IRQ bits since the last ClearIrq" are
//! the same window. [`arm`] is called from inside `Sx1262::clear_irq`, so every one of the eleven
//! sites that clear the chip's latch (ten `self.clear_irq(...)` calls plus `poll_rx`'s
//! `clear_irq_from`, which passes a baseline it has already read) opens a fresh capture window at
//! the same instant it opens a fresh chip-IRQ window. The two cannot drift, because there is no
//! second place to remember to do it.
//!
//! `Sx1262::poll_rx` then reads the capture **between `GetIrqStatus` and `ClearIrqStatus`**, in the
//! window where DIO1 is provably still high and a second rising edge is physically impossible. That
//! placement is not cosmetic: after the clear, `poll_rx` spends ~2 ms reading the buffer, and a
//! frame arriving in that stretch would take its own capture — which, on this timer, OVERWRITES the
//! register (see the ☠ note in [`crate::capture::classify`]). Reading the stamp at the old
//! `let ts = micros()` site would have attributed frame 2's instant to frame 1's payload, silently.
//!
//! ★ **The DIO1 mask is `IRQ_RX_DONE` alone** (`sx1262::DIO1_MASK`), so an edge on this pin IS
//! self-identifying. It used to carry `TX_DONE | RX_DONE | TIMEOUT`, and disambiguation rested
//! entirely on the arm-on-clear discipline plus an *argument* about which of those bits could
//! latch while RX was armed — an argument with a hole in it: a `TxDone` latching in the ~40 µs
//! between `start_rx`'s clear and its `SetRx` holds DIO1 high with RX armed, so the next frame's
//! `RxDone` raises no edge, and the stale non-RX edge — count 1, no overcapture — would have been
//! reported as that frame's instant. Narrowing the mask deletes the whole class instead of
//! reasoning about it. CAD contributes nothing either way: `IRQ_CAD_DONE`/`IRQ_CAD_DETECTED` were
//! never in the mask.
//!
//! **What the timer still cannot see, and what closes it.** A frame arriving while DIO1 is already
//! high raises no edge at all, while the chip's buffer and packet status advance to describe it —
//! so an edge count of exactly 1 does not prove the capture belongs to the frame being reported.
//! The second half of the rule is the chip's own completed-packet count, differenced across the
//! window and applied by [`crate::capture::attribute`] in `poll_rx`. Neither half is sufficient
//! alone.
//!
//! ## Why 1 MHz and not faster
//!
//! The timer is 16-bit. At 1 MHz it wraps every 65.536 ms; at 8 MHz (PSC = 0, 125 ns/tick) it would
//! wrap every 8.192 ms, and the main loop can block far longer than that — a `SET_*` command pays a
//! ~78 ms TCXO startup. The wrap is handled by the update interrupt rather than at read time, so
//! neither rate is *unsafe*, but 1 MHz also keeps the unit identical to [`crate::capture::STAMP_HZ`]
//! and to TIM2's scheduler tick, so no conversion exists to get wrong. The LR2021 raised its timer
//! 1 → 16 MHz only after a measurement showed the spread sitting *at* the quantisation floor; do the
//! same here, and raise it only if the observed spread is one tick.
//!
//! ## GD32 is not STM32, so the divergences are tested rather than assumed
//!
//! TIM3 is GD32 **TIMER2** at the same base (`0x4000_0400`) with the same register offsets and bit
//! positions under different field names, and this firmware already drives TIM2 with STM32
//! semantics in production. But this repo has already been bitten once by a GD32 divergence (the
//! in-app jump to the ROM bootloader does not work on this part), and the two behaviours this
//! module *depends* on are exactly the fiddly ones: that reading `CCR3` clears `CC3IF`, and that
//! `CC3OF` clears only on a write. So [`init`] proves both on the silicon at boot, using the
//! timer's own software event generator (`EGR.CC3G` performs a capture in input mode), plus a rate
//! check against SysTick. The node advertises `stamp_kind = 3` only if that passes — a capability
//! gated on a measurement, not on the code compiling.

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use stm32f1xx_hal::pac;

use crate::capture::{self, classify, Degrade, StampVerdict};

/// Free-running to the full 16-bit top: one wrap every 65 536 ticks = 65.536 ms.
const ARR_TOP: u16 = 0xFFFF;

// TIMx_SR is `rc_w0`: a bit is cleared by writing **0** to it and is unaffected by writing 1. So the
// only correct way to clear one flag is to write ones everywhere else — `w.bits(!MASK)`.
//
// ⚠ Note what this rules out. `svd2rust`'s `write(|w| w.foo().clear_bit())` starts from the
// register's RESET value, which for SR is 0, so it clears *every* flag. The `SchedTimer` pattern
// next door (`t.sr.write(|w| w.uif().clear_bit())` in `main.rs`) does exactly that, which is
// harmless on TIM2 where no other flag is in use — and would silently eat this module's capture
// flag if it were copy-pasted onto TIM3. `modify` is equally wrong: it is a read-modify-write and
// loses any flag the hardware sets between the two halves.
const SR_UIF: u32 = 1 << 0;
const SR_CC3IF: u32 = 1 << 3;
const SR_CC3OF: u32 = 1 << 11;

/// EGR bit 3, `CC3G` — the software capture/compare-3 event. In INPUT mode it makes the timer
/// perform a capture exactly as a pin edge would, setting `CC3IF` (and `CC3OF` if `CC3IF` was
/// already set). That is what lets [`init`]'s self-test exercise the flag semantics with no radio,
/// no edge and no bench instrument.
const EGR_CC3G: u32 = 1 << 3;

/// TIM3 wrap count — the high half of the microsecond clock. Written only by [`on_tim3_irq`].
static WRAPS: AtomicU32 = AtomicU32::new(0);

// The published capture, as a seqlock. `thumbv7m` has no `AtomicU64`, so the 64-bit stamp cannot be
// published in one store; a reader that simply took both halves could see a torn value straddling a
// 2^32 µs (~71 min) boundary. SEQ odd = a write is in progress, even = stable.
static SEQ: AtomicU32 = AtomicU32::new(0);
static STAMP_LO: AtomicU32 = AtomicU32::new(0);
static STAMP_HI: AtomicU32 = AtomicU32::new(0);
static STAMP_LAT: AtomicU32 = AtomicU32::new(0);

/// Total captures since boot, and its value at the last [`arm`]. The difference is *edges since this
/// window opened*, which is the honest ambiguity metric: it should always be 0 or 1, and a 2 is a
/// fact about DIO1 the host should see rather than a stamp quietly attributed to the wrong frame.
static CAPTURES: AtomicU32 = AtomicU32::new(0);
static ARMED_AT: AtomicU32 = AtomicU32::new(0);

/// Overcaptures (`CC3OF`) since boot, and its value at the last [`arm`]. Free-running, like the
/// chip's own `GetStats` counters and for the same reason: the per-window figure is a *difference*
/// of two reads, so re-baselining it under the reader would make the difference lie.
static OVERCAPTURES: AtomicU32 = AtomicU32::new(0);
static ARMED_OVER: AtomicU32 = AtomicU32::new(0);

/// Worst ISR-entry latency observed, in ticks: `CNT` at the ISR's clock read minus the latched
/// capture. **This is the number the whole module exists to make irrelevant** — it is how wrong a
/// software stamp taken in the ISR would have been, and it is measured rather than assumed. (The
/// old software stamp was worse still: it was taken in the poll loop, after the SPI readback.)
static LAT_MAX: AtomicU32 = AtomicU32::new(0);

/// **May this node claim a hardware capture?** Gates [`take`] and the `stamp_kind` byte EVT_CAP
/// advertises. Set from [`capture::Timebase::hw_stamp`] — flag semantics AND a measured rate.
static HW_OK: AtomicBool = AtomicBool::new(false);

/// **Is TIM3 allowed to be the node's microsecond clock?** Set from [`capture::Timebase::clock`].
///
/// False means the boot measurement caught the counter ticking at a rate that is not
/// [`capture::STAMP_HZ`] — a prescaler or clock-tree divergence, which is a factor of 2 or 8 — and
/// [`ticks64`] routes to the SysTick-derived clock instead, exactly as this firmware did before the
/// capture path existed. Starts **false** so that any read taken before [`init`] runs (there are
/// none, and the ordering is asserted in `main`) uses the clock that is already running rather than
/// an unconfigured timer.
static CLOCK_IS_TIM3: AtomicBool = AtomicBool::new(false);

#[inline(always)]
fn tim() -> &'static pac::tim2::RegisterBlock {
    // SAFETY: TIM3 is owned by `main` (it holds `pac::TIM3`) and only this module touches it. The
    // PAC gives TIM3 the TIM2 register-block type because they are literally the same block.
    unsafe { &*pac::TIM3::ptr() }
}

/// What [`init`] established on the silicon, rather than assumed from a reference manual.
#[derive(Clone, Copy)]
pub struct SelfTest {
    /// A software capture event set `CC3IF`.
    pub cc3if_set: bool,
    /// ...and an `rc_w0` write to `SR` cleared it — which is how the ISR clears it.
    ///
    /// ☠ **MEASURED on the GD32F103: reading `CCR3` does NOT clear `CC3IF`**, unlike the STM32F103
    /// this part is cloned from. That divergence is real and this node found it at boot (`st` bit 1
    /// read 0 while everything else passed). A capture path that assumed read-to-clear would re-enter
    /// the ISR forever and report a stale register value as every stamp after the first.
    pub cc3if_cleared_by_write: bool,
    /// A second software capture with the first unread set `CC3OF`, i.e. the overcapture detector is
    /// live. Without it, a lost edge would be invisible.
    pub cc3of_set: bool,
    /// ...and an `rc_w0` write cleared it. A `CC3OF` that could not be cleared would mark every
    /// later capture as overcaptured — a stuck flag that poisons the instrument silently.
    pub cc3of_cleared_by_write: bool,
    /// Ticks counted across exactly one SysTick period. Expect 1000 at [`capture::STAMP_HZ`].
    ///
    /// This checks the PRESCALER and the timer's clock source, not the oscillator — SysTick and TIM3
    /// are the same 8 MHz HSI divided twice, so a rate error here means the APB1 timer clock is not
    /// what `clocks.pclk1_tim()` says. **0 means SysTick never moved**, which is a broken reference
    /// rather than a broken timer, and fails the band either way.
    pub ticks_per_ms: u16,
}

impl SelfTest {
    /// The band accepted for [`SelfTest::ticks_per_ms`]; defined in [`capture`] because the
    /// *decision* it feeds is host-testable and this file is not.
    pub const TICKS_PER_MS_MIN: u16 = capture::TICKS_PER_MS_MIN;
    /// See [`SelfTest::TICKS_PER_MS_MIN`].
    pub const TICKS_PER_MS_MAX: u16 = capture::TICKS_PER_MS_MAX;

    /// **The four flag-semantics checks only** — "can an edge be latched on this silicon and read
    /// back?". Deliberately does NOT include the rate: a timer whose flags misbehave is a broken
    /// capture channel but a perfectly good counter, while a timer at the wrong rate is a broken
    /// clock, and folding the two into one boolean is what let a DETECTED rate fault ship a wrong
    /// number. See [`capture::choose_timebase`].
    pub const fn flags_ok(&self) -> bool {
        self.cc3if_set
            && self.cc3if_cleared_by_write
            && self.cc3of_set
            && self.cc3of_cleared_by_write
    }

    /// **What this silicon has earned**: which counter may be the node's microsecond clock, and
    /// whether it may claim a hardware capture. One call, so the two verdicts cannot be read apart.
    pub const fn timebase(&self) -> capture::Timebase {
        capture::choose_timebase(self.flags_ok(), self.ticks_per_ms)
    }

    /// One compact byte for the boot EVT_LOG: bit 0 `cc3if_set`, 1 `cc3if_cleared_by_write`,
    /// 2 `cc3of_set`, 3 `cc3of_cleared_by_write`, 4 the rate band. `0x1F` is a clean pass.
    pub const fn bits(&self) -> u8 {
        (self.cc3if_set as u8)
            | ((self.cc3if_cleared_by_write as u8) << 1)
            | ((self.cc3of_set as u8) << 2)
            | ((self.cc3of_cleared_by_write as u8) << 3)
            | ((self.timebase().rate_verified as u8) << 4)
    }
}

/// Everything [`init`] learned, for the boot diagnostic.
pub struct Boot {
    /// The prescaler actually programmed, so the boot log states it rather than implying it.
    pub psc: u16,
    pub self_test: SelfTest,
}

/// Bring TIM3 up as a free-running [`capture::STAMP_HZ`] counter with CH3 capturing PB0's rising
/// edge, prove the two flag behaviours this module depends on, and start the clock.
///
/// **No GPIO work is required and none is done.** On STM32F1/GD32F1 an *alternate-function input* is
/// configured as a plain input (`MODE = 00`, `CNF` = floating or pull); the `into_alternate_*`
/// family applies to outputs only. `pb0.into_floating_input()` in `main` is therefore already the
/// correct pin state for TIM3_CH3, the pin can stay moved into `Sx1262` (a capture channel reads the
/// pad, it does not take it), and no AFIO remap is needed: TIM3_CH3 is PB0 in the default mapping
/// *and* in the partial remap; only the full remap moves it, to PC8.
///
/// ⚠ If anyone ever enables `Tim3PartialRemap`, CH3 stays on PB0 but CH1 moves to **PB4**, which is
/// the RF-switch output. Don't.
pub fn init(pclk1_tim_hz: u32) -> Boot {
    let psc = (pclk1_tim_hz / capture::STAMP_HZ).saturating_sub(1) as u16;
    unsafe {
        let rcc = &*pac::RCC::ptr();
        rcc.apb1enr.modify(|_, w| w.tim3en().set_bit());
        // Read back before the first TIM3 access: a peripheral clock enable takes effect on the bus,
        // not in the instruction that writes it, and a register write issued in its shadow is lost.
        let _ = rcc.apb1enr.read();
    }
    let t = tim();

    t.cr1.reset(); // CEN=0, up-counting, edge-aligned, CKD=00 (t_DTS = t_CK_INT = 125 ns)
    t.dier.reset(); // interrupts off for the self-test below
    t.ccer.reset(); // CC3S is write-protected while CC3E = 1, so this must precede CCMR2

    // URS = 1: only a counter overflow raises UIF. Without it the `UG` below — needed to latch
    // PSC/ARR — would set UIF too and the wrap counter would start a whole wrap (65.536 ms) ahead.
    // `SchedTimer` handles the same hazard the other way, by clearing UIF after UG; that is fine for
    // a one-shot's "done" flag and wrong here, where UIF is load-bearing for the clock itself.
    t.cr1.write(|w| w.urs().set_bit());
    t.psc.write(|w| w.psc().bits(psc));
    t.arr.write(|w| w.arr().bits(ARR_TOP));

    // CC3S = 01: CH3 is an input mapped to TI3, its own pin (PB0).
    // IC3PSC = 00: capture every edge, no divide.
    // IC3F = 0001: f_SAMPLING = f_CK_INT, N = 2 — two consecutive agreeing samples at 8 MHz. This
    //   rejects a sub-250 ns glitch on DIO1 and costs at most 250 ns of DETERMINISTIC delay, a
    //   quarter of one tick, so it cannot move the stamp by a tick on its own. It is a bias, not
    //   jitter, and it is stated here rather than corrected for. (The filter clocks off f_DTS,
    //   derived from CK_INT via CR1.CKD which stays 00 — not off the prescaled 1 MHz; that is a
    //   common misreading of the reference manual.)
    t.ccmr2_input()
        .modify(|_, w| w.cc3s().ti3().ic3psc().bits(0b00).ic3f().bits(0b0001));

    // CC3P = 0: rising edge. The SX1262 asserts DIO1 HIGH and it falls only when the IRQ is cleared
    // over SPI, long after the event, so the rising edge is the one that means anything.
    t.ccer.modify(|_, w| w.cc3p().clear_bit().cc3e().set_bit());

    t.egr.write(|w| w.ug().set_bit()); // latch PSC/ARR (raises no UIF: URS = 1)
    t.sr.write(|w| unsafe { w.bits(0) }); // rc_w0: start with every flag clear
    let _ = t.ccr3().read(); //             ...and with no stale capture pending

    let self_test = run_self_test(t);
    // ★ Two independent verdicts, stored separately, because they actuate different things: the
    // capture channel, and the identity of the node's clock. See [`capture::choose_timebase`] for
    // why folding them into one boolean shipped a wrong number.
    let tb = self_test.timebase();
    HW_OK.store(tb.hw_stamp, Ordering::Release);
    CLOCK_IS_TIM3.store(
        matches!(tb.clock, capture::ClockSource::CaptureTimer),
        Ordering::Release,
    );

    // Leave no residue from the self-test: its synthetic captures must not look like a frame.
    // Only the capture flags are cleared — UIF is the clock's and is not ours to drop, even though
    // the counter has been running for barely a millisecond and cannot yet have overflowed. The
    // rule is the point: never clear a flag you do not own.
    let _ = t.ccr3().read();
    t.sr.write(|w| unsafe { w.bits(!(SR_CC3IF | SR_CC3OF)) });
    CAPTURES.store(0, Ordering::Relaxed);
    ARMED_AT.store(0, Ordering::Relaxed);
    OVERCAPTURES.store(0, Ordering::Relaxed);
    ARMED_OVER.store(0, Ordering::Relaxed);
    LAT_MAX.store(0, Ordering::Relaxed);

    t.dier.write(|w| w.uie().set_bit().cc3ie().set_bit());
    // SAFETY: the TIM3 vector is `main`'s `TIM3()`, which calls only [`on_tim3_irq`]; nothing else
    // claims it, and no shared state is touched outside the atomics above.
    unsafe { pac::NVIC::unmask(pac::Interrupt::TIM3) };

    Boot { psc, self_test }
}

/// Prove the flag semantics and the tick rate on the silicon in front of us. Runs with `DIER`
/// cleared and the NVIC still masked, so the ISR cannot consume a flag before it is read — which it
/// otherwise would, instantly, and the "is CC3IF set?" question would always read false.
fn run_self_test(t: &pac::tim2::RegisterBlock) -> SelfTest {
    // One thing that could confuse steps 1 and 2: a genuine DIO1 rising edge landing mid-test would
    // add a capture of its own. It cannot happen at this point in `main` — the SX1262 has not been
    // reset yet, so nothing is driving an edge — and if it somehow did, the only effect is a spurious
    // `CC3IF` that makes `cc3if_cleared_by_write` read false and the node advertise the SOFTWARE kind.
    // The failure direction is conservative, which is the direction a capability check must fail in.
    //
    // --- 1. a software capture raises CC3IF, and reading CCR3 clears it -------------------------
    t.egr.write(|w| unsafe { w.bits(EGR_CC3G) });
    let cc3if_set = t.sr.read().bits() & SR_CC3IF != 0;
    let _ = t.ccr3().read();
    // The read is performed for its side effect on parts where it HAS one, but it is deliberately
    // not what is asserted: on the GD32F103 in front of us it does not clear, so the ISR clears with
    // a write and this probe tests the write. Check what the code depends on, not what the STM32
    // reference manual promises for a different part.
    t.sr.write(|w| unsafe { w.bits(!SR_CC3IF) });
    let cc3if_cleared_by_write = t.sr.read().bits() & SR_CC3IF == 0;

    // --- 2. a second capture with the first unread raises CC3OF, and only a write clears it ------
    t.egr.write(|w| unsafe { w.bits(EGR_CC3G) });
    t.egr.write(|w| unsafe { w.bits(EGR_CC3G) });
    let cc3of_set = t.sr.read().bits() & SR_CC3OF != 0;
    t.sr.write(|w| unsafe { w.bits(!SR_CC3OF) });
    let cc3of_cleared_by_write = t.sr.read().bits() & SR_CC3OF == 0;

    // --- 3. the counter actually advances at the declared rate -----------------------------------
    // Start the counter for the measurement; it stays running from here on.
    let _ = t.ccr3().read();
    t.sr.write(|w| unsafe { w.bits(!(SR_CC3IF | SR_CC3OF)) });
    t.cr1.modify(|_, w| w.cen().set_bit());

    SelfTest {
        cc3if_set,
        cc3if_cleared_by_write,
        cc3of_set,
        cc3of_cleared_by_write,
        ticks_per_ms: measure_tick_rate(t),
    }
}

/// Ticks counted across exactly one SysTick period, i.e. the programmed rate as actually delivered.
///
/// **The reference is SysTick, deliberately not `cortex_m::asm::delay`.** That helper is documented
/// as "approximately n cycles" and its true cost on this core is a branch-refill question nobody
/// here has measured — gating a published capability on it would be gating it on an unmeasured
/// constant, which is the exact move this codebase keeps having to undo. SysTick is already running
/// at reload 7999 off the same 8 MHz core clock, so one increment of `MILLIS` is exactly 1000 µs.
/// Comparing the two counters directly is also precisely the comparison `stats::CLOCK_SKEW_MS` goes
/// on making at runtime, so a rate fault and a lost-wrap fault are read on one scale.
///
/// Returns **0** if SysTick never moves. Bounding the spin matters: a hang here would brick the boot
/// before the host link is even up, and "the reference clock is dead" is a diagnosis worth reporting
/// rather than a reason to stop.
fn measure_tick_rate(t: &pac::tim2::RegisterBlock) -> u16 {
    /// A few hundred milliseconds at 8 MHz — far above one millisecond, far below forever. The exact
    /// figure depends on the loop's cycle cost, which is precisely the thing this function refuses to
    /// depend on; it only has to bracket "SysTick is dead" away from "SysTick is slow".
    const SPIN_MAX: u32 = 400_000;
    fn wait_tick(from: u32) -> bool {
        let mut n = 0u32;
        while crate::millis() == from {
            n += 1;
            if n > SPIN_MAX {
                return false;
            }
        }
        true
    }
    // Align to a tick boundary first, so the interval measured is a whole SysTick period rather than
    // whatever fraction of one was left when this ran.
    if !wait_tick(crate::millis()) {
        return 0;
    }
    let c0 = t.cnt.read().cnt().bits();
    let m1 = crate::millis();
    if !wait_tick(m1) {
        return 0;
    }
    let c1 = t.cnt.read().cnt().bits();
    c1.wrapping_sub(c0)
}

/// The free-running microsecond clock — the node's ONE timebase, whichever counter it is.
///
/// `micros64()`, `EVT_RX.ts`, `EVT_CLOCK` and `CMD_TX_AT_ABS`'s deadline are all this counter, which
/// is the property the host's driver depends on without being able to check: it declares a single
/// `ClockDomainId` per port and then differences a received stamp against a `CMD_READ_CLOCK` read
/// and schedules against the result. Two "1 MHz" counters with different epochs would satisfy every
/// unit check on the wire and put an arbitrary offset into every one of those subtractions.
///
/// Deliberately lock-free rather than a critical section: USART1 has no FIFO and its ISR rescues
/// each byte within its ~87 µs window, so masking interrupts to read a clock would risk the host
/// link. (This firmware contains no `interrupt::free` and no critical section anywhere, which is
/// also what bounds the ISR latency this module measures.)
///
/// ★ **It is TIM3 only while TIM3 has earned it.** If the boot rate measurement found the counter
/// ticking at anything but [`capture::STAMP_HZ`], this routes to the SysTick-derived microsecond
/// clock and the node keeps a timebase whose unit is what `EVT_CAP.stamp_hz` says it is. That
/// branch also forces `stamp_kind` down to `SOFTWARE_COUNTER`: a capture latched in TIM3 ticks
/// cannot be published against a SysTick epoch, and both being "1 MHz" would hide it from every
/// unit check on the wire.
#[inline]
pub fn ticks64() -> u64 {
    if CLOCK_IS_TIM3.load(Ordering::Relaxed) {
        tim3_ticks64()
    } else {
        crate::systick_micros64()
    }
}

/// The raw TIM3 free-running tick count, whatever the boot verdict said about its rate.
///
/// Split out from [`ticks64`] for exactly one reason: [`on_tim3_irq`] must reconstruct a capture
/// against the counter the capture came from, never against a fallback clock. Everything else in
/// the firmware wants [`ticks64`].
///
/// The hi/lo race is handled by [`capture::extend`]; see its doc for why reading the overflow flag
/// is necessary and not merely belt-and-braces. Note the ordering here: `cnt` is read **before**
/// `uif`, so a wrap landing between the two makes `WRAPS` move and the loop retries.
#[inline]
fn tim3_ticks64() -> u64 {
    let t = tim();
    loop {
        let hi1 = WRAPS.load(Ordering::Relaxed);
        let cnt = t.cnt.read().cnt().bits();
        let uif = t.sr.read().bits() & SR_UIF != 0;
        let hi2 = WRAPS.load(Ordering::Relaxed);
        if hi1 != hi2 {
            continue; // the wrap ISR ran mid-read; re-read rather than reason about it
        }
        return capture::extend(hi1, cnt, uif);
    }
}

/// **Open a fresh capture window**: discard any pending edge and reset "edges since armed" to zero.
///
/// Called from inside `Sx1262::clear_irq`, so the timer's window and the chip's IRQ latch are opened
/// by the same instruction sequence at every clear site — including the ones that are *not*
/// receptions (`transmit`, `stage_tx`, `wait_txdone`, the three in `do_cad`). That is rule 2 of the
/// discard rule, and putting it in the driver's clear rather than at the call sites is what makes it
/// impossible to forget.
///
/// Ordering: it runs *after* the ClearIrq SPI transaction. If DIO1 has not physically fallen yet
/// that is fine — the line is already high, so it produces no new rising edge and nothing is
/// captured until the genuine next event.
pub fn arm() {
    let t = tim();
    let _ = t.ccr3().read(); // reading CCR3 is what clears CC3IF
    t.sr.write(|w| unsafe { w.bits(!(SR_CC3IF | SR_CC3OF)) }); // CC3OF is NOT cleared by the read
    ARMED_OVER.store(OVERCAPTURES.load(Ordering::Acquire), Ordering::Release);
    ARMED_AT.store(CAPTURES.load(Ordering::Acquire), Ordering::Release);
}

/// **The stamp for the window that is closing**, judged against every attribution rule.
///
/// Must be called while the chip's IRQ status still says `RxDone` and before that status is cleared
/// — i.e. from inside `Sx1262::poll_rx`, between `GetIrqStatus` and `ClearIrqStatus`. The capture
/// supplies the instant; that status word supplies the reason; they describe the same event only
/// because they are read in the same IRQ-clear cycle.
///
/// **This is only the timer's half of the verdict.** It counts edges, overcaptures and ISR latency,
/// none of which can see a frame that arrived while DIO1 was already high — that frame raises no
/// edge while the chip's buffer and packet status advance to describe it. `poll_rx` must pass the
/// result through [`capture::attribute`] with the chip's own completed-packet delta before it may be
/// believed; this function returning `Hardware` is a necessary condition, not a sufficient one.
///
/// Never returns an approximate hardware stamp. Every rejection is a
/// [`StampVerdict::Software`] carrying why, which the caller puts on the wire per frame.
pub fn take() -> StampVerdict {
    if !HW_OK.load(Ordering::Relaxed) {
        // The boot self-test failed, so this node is not claiming a capture at all. Say so per
        // frame as well as in EVT_CAP, rather than emitting software stamps that look like the
        // hardware ones a passing node emits.
        return StampVerdict::Software(Degrade::NoCapturePath);
    }
    let edges = CAPTURES
        .load(Ordering::Acquire)
        .wrapping_sub(ARMED_AT.load(Ordering::Acquire));
    let over = OVERCAPTURES
        .load(Ordering::Acquire)
        .wrapping_sub(ARMED_OVER.load(Ordering::Acquire));
    if edges == 0 {
        // Short-circuit: the published record belongs to an earlier window and must not be read.
        return classify(0, over, 0, 0);
    }
    // Seqlock read — see the statics. Bounded: the writer is an ISR that runs in a few µs.
    let (ticks, lat) = loop {
        let s1 = SEQ.load(Ordering::Acquire);
        if s1 & 1 != 0 {
            continue; // the ISR is mid-write
        }
        let lo = STAMP_LO.load(Ordering::Relaxed);
        let hi = STAMP_HI.load(Ordering::Relaxed);
        let lat = STAMP_LAT.load(Ordering::Relaxed);
        if SEQ.load(Ordering::Acquire) == s1 {
            break ((((hi as u64) << 32) | lo as u64), lat);
        }
    };
    classify(edges, over, lat, ticks)
}

/// TIM3 global interrupt: the wrap that extends the clock, and the capture that stamps a frame.
///
/// **Flag discipline, which is asymmetric and easy to get wrong:**
/// * `CC3IF` is cleared by **reading `CCR3`** (as well as by an `rc_w0` write). So this ISR *must*
///   read `CCR3` or it re-enters forever — and `CCR3` cannot be peeked at without consuming the
///   flag, which is why [`arm`] reads and discards it. Whether the GD32 honours the clear-by-read is
///   proved at boot by [`run_self_test`] rather than assumed.
/// * `CC3OF` is cleared **only** by writing 0 to it; reading `CCR3` does not touch it.
/// * Every clear is `w.bits(!BIT)`. Writing the reset value would clear the other flags too, and one
///   of them is the capture.
///
/// **Overcapture** means a second edge was latched while `CC3IF` was still set: `CCR3` holds the
/// *newer* value and the older one is gone. Given DIO1's latch-until-cleared behaviour that needs a
/// whole SPI `ClearIrq` inside one ISR latency, so it should never happen; if it does, the count
/// says an edge was **lost** — a different and worse thing than a stamp being imprecise — and it is
/// reported, never hidden.
///
/// **Wrap-vs-capture ordering** is not solved by ordering the flags but by construction: the wrap is
/// folded into `WRAPS` first, then the stamp is computed backwards as `now − (CNT − CCR3)`. See
/// [`capture::reconstruct`].
pub fn on_tim3_irq() {
    let t = tim();
    let sr = t.sr.read().bits();

    // ⚠ **Load-bearing invariant, previously unwritten: TIM3 must not be preemptible by anything
    // that calls [`ticks64`].** Clearing `UIF` and incrementing `WRAPS` are two instructions, and a
    // reader that lands between them sees `WRAPS = W` with `UIF` already gone and a `cnt` just past
    // the wrap — 65.536 ms low. (Swapping the order does not help: a reader would then see
    // `WRAPS = W+1` *and* a pending `UIF` with a small `cnt`, and add a second wrap.) It holds today
    // because nothing in this crate calls `NVIC::set_priority` — every interrupt sits at priority 0
    // and cannot preempt another — and on the normal path [`ticks64`] IS the node's clock, so anyone
    // adding a priority must revisit this pair rather than discover it on air.
    if sr & SR_UIF != 0 {
        t.sr.write(|w| unsafe { w.bits(!SR_UIF) });
        WRAPS.fetch_add(1, Ordering::Relaxed);
    }

    if sr & SR_CC3IF != 0 {
        // Order matters, and the obvious order is wrong. ☠ Deciding "was there an overcapture?"
        // from the `sr` sampled above misses the ONE interleaving the detector exists for: an edge
        // landing between that sample and the `CCR3` read below OVERWRITES the register with the
        // later edge and sets `CC3OF`, so the stale sample says "no overcapture" while `cc` already
        // holds a value belonging to a different frame — a plausible timestamp on the wrong frame,
        // reported at `stamp_kind = 3` with no note. Read `CCR3` first, then OR a fresh `SR` sample
        // in, so any flag raised across the whole read is caught. The leftover `CC3OF` would
        // otherwise be wiped by the next `arm()` and never counted at all.
        let cc = t.ccr3().read().ccr().bits();
        // ☠ **GD32 divergence, MEASURED on this silicon** (boot self-test read `st=0x1D`, bit 1
        // clear): unlike the STM32F103 this part is cloned from, reading `CCR3` does **not** clear
        // `CC3IF`. Relying on the read — as this line originally did — leaves the flag set, the ISR
        // re-enters forever, and every stamp after the first is a stale register value. So clear it
        // the way `CC3OF` is already cleared, with an `rc_w0` write.
        //
        // Placed BEFORE the fresh `SR` sample deliberately: an edge landing between the `CCR3` read
        // and this write finds `CC3IF` still set and therefore raises `CC3OF`, which this write does
        // not touch (`!SR_CC3IF` leaves that bit 1, and `rc_w0` ignores a written 1) and the sample
        // below still sees. Sampling first and clearing after would lose exactly that edge.
        t.sr.write(|w| unsafe { w.bits(!SR_CC3IF) });
        let of = sr | t.sr.read().bits();
        if of & SR_CC3OF != 0 {
            t.sr.write(|w| unsafe { w.bits(!SR_CC3OF) });
            OVERCAPTURES.fetch_add(1, Ordering::Relaxed);
        }
        let (stamp, lat) = capture::reconstruct(tim3_ticks64(), cc);

        let s = SEQ.load(Ordering::Relaxed);
        SEQ.store(s.wrapping_add(1), Ordering::Release); // odd: writing
        STAMP_LO.store(stamp as u32, Ordering::Relaxed);
        STAMP_HI.store((stamp >> 32) as u32, Ordering::Relaxed);
        STAMP_LAT.store(lat, Ordering::Relaxed);
        SEQ.store(s.wrapping_add(2), Ordering::Release); // even: stable

        if lat > LAT_MAX.load(Ordering::Relaxed) {
            LAT_MAX.store(lat, Ordering::Relaxed);
        }
        // Published LAST: it is what `take` gates on, so the record is already stable when it moves.
        CAPTURES.fetch_add(1, Ordering::Release);
    }
}

/// Is the hardware capture path live — flag semantics proved AND the tick rate measured in band?
/// This — not "the module compiled" — is what `send_cap` turns into the `stamp_kind` byte.
pub fn hw_stamp_live() -> bool {
    HW_OK.load(Ordering::Relaxed)
}

/// **Which counter [`ticks64`] is actually reading**, for the boot diagnostic. `true` = TIM3, whose
/// rate was measured at [`capture::STAMP_HZ`]; `false` = the SysTick fallback, i.e. the boot
/// measurement rejected TIM3's rate and the node moved its clock rather than shipping a wrong one.
pub fn clock_is_capture_timer() -> bool {
    CLOCK_IS_TIM3.load(Ordering::Relaxed)
}

/// `(overcaptures, worst ISR-entry latency in ticks)` for EVT_STATS.
///
/// The latency is the measured gap between the hardware stamp and what a software stamp taken *in
/// the ISR* would have been — and the old software stamp was taken later still, in the poll loop
/// after the SPI readback, so this is a lower bound on what the change bought.
pub fn counters() -> (u32, u32) {
    (
        OVERCAPTURES.load(Ordering::Relaxed),
        LAT_MAX.load(Ordering::Relaxed),
    )
}

/// Re-baseline the latency watermark (`CMD_RESET_STATS`). Deliberately does **not** touch
/// `OVERCAPTURES`: [`take`] reads it as a difference against [`arm`]'s snapshot, and moving one side
/// of a difference under the reader is how a counter starts lying.
pub fn reset_lat_watermark() {
    LAT_MAX.store(0, Ordering::Relaxed);
}
