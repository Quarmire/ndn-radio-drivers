//! **M5 TX — hardware-scheduled transmit.** The measurement that sizes the MAC's guard band.
//!
//! ## The mechanism
//!
//! ```text
//!   TIMER20.CC[2] == target tick ──event──▶ DPPI ──task──▶ GPIOTE OUT (set P1.04 high)
//!                                                                │
//!                                             LR2021 DIO8 configured as DioFunc::TxTrigger
//!                                                                │
//!                                                       transmit starts — no SPI, no CPU
//! ```
//!
//! The LR2021 supports *DIO TX/RX triggers* natively: with a DIO set to `TxTrigger` and the default
//! TX timeout programmed, an edge on that pin starts a transmission from the already-loaded FIFO.
//! So the transmit instant is decided by a timer compare in silicon — the CPU only refills the FIFO
//! between frames, and its latency lands *between* transmissions where it cannot move one.
//!
//! Contrast `m4_tx`, which calls `set_tx()` over SPI from a software `Timer::after`. There the
//! transmit instant carries executor wake-up latency *and* a whole SPI transaction. That is the
//! same approximation the Wi-Fi face makes today, and the reason its guard bands are milliseconds.
//!
//! ## Wiring constraint, and what it costs
//!
//! The shield brings out exactly **one** DIO — DIO8 → P1.04. On this node it is repurposed from
//! interrupt output to trigger input, so the TX side loses its IRQ pin. That is acceptable and
//! deliberate: transmit *completion* can be polled over SPI, and completion timing does not affect
//! the transmit *instant*, which is the only thing M5 measures. The receiver keeps its IRQ pin and
//! does the timestamping (M4).
//!
//! ## How the result is read
//!
//! This binary does not measure itself — it cannot, having given up the IRQ pin. The number comes
//! from the receiver: `m5_rx` hardware-timestamps arrivals and reports the spread of the
//! *consecutive-sequence* inter-arrival time. Because M4 established the RX capture contributes
//! nothing measurable (0 ticks at 62.5 ns), that spread **is** the transmit-instant spread, plus the
//! two nodes' relative clock drift.

#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_nrf::gpio::{Level, OutputDrive};
use embassy_nrf::gpiote::{OutputChannel, OutputChannelPolarity};
use embassy_nrf::ppi::Ppi;
use embassy_nrf::timer::Timer as HwTimer;
use embassy_time::{Duration, Timer};
use panic_probe as _;

use lr2021::system::{DioFunc, DioNum, PullDrive};
use lr2021_nrf54l15_rs::timing::MAC_CLOCK;
use lr2021_nrf54l15_rs::{flrc_link, hw};

const TAG: &[u8] = b"NDN-M4"; // same tag as m4_tx so one receiver binary reads both experiments

/// Transmit period in timer ticks. 16 MHz ⇒ 320_000 ticks = exactly 20 ms, matching `m4_tx` so the
/// software-timed and hardware-scheduled runs are directly comparable.
const PERIOD_TICKS: u32 = 320_000;

/// CC register holding the scheduled transmit instant.
const CC_SCHED: usize = 2;

/// Minimum lead time between arming a compare and the instant it fires: 2 ms at 16 MHz. Below this
/// the FIFO write might not finish first, and a compare armed in the past never fires at all.
const MIN_LEAD_TICKS: u32 = 2 * 16_000;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());
    let (mut radio, timing, _uart) = hw::init(p);

    radio.reset().await.unwrap();
    Timer::after(Duration::from_millis(50)).await;
    match radio.get_version().await {
        Ok(mut v) => defmt::info!("m5_tx: LR2021 fw {}.{} ok={}", v.major(), v.minor(), v.status().is_ok()),
        Err(e) => defmt::panic!("m5_tx: no radio: {}", defmt::Debug2Format(&e)),
    }

    flrc_link::configure(&mut radio).await.expect("FLRC configure");

    // Repurpose DIO8 from IRQ output to TX-trigger input. This overrides the `set_dio_irq` done in
    // the shared configure(); the ordering is intentional so the shared setup stays the one place
    // the link is defined.
    radio
        .set_dio_function(DioNum::Dio8, DioFunc::TxTrigger, PullDrive::PullDown)
        .await
        .expect("DIO8 -> TxTrigger");
    // A DIO-triggered TX uses the *default* timeout rather than one passed in a SetTx command.
    // 0 = no timeout; the frame is short and there is nothing to abort into.
    radio.set_default_timeout(0, 0).await.expect("default timeouts");

    // Timer + GPIOTE output + DPPI: compare fires the pin, the pin fires the radio.
    let hwt = HwTimer::new(timing.timer);
    hwt.set_frequency(MAC_CLOCK);
    hwt.start();
    let trigger = OutputChannel::new(
        timing.gpiote,
        timing.dio,
        Level::Low, // idle low; the compare drives the rising edge
        OutputDrive::Standard,
        OutputChannelPolarity::Set,
    );
    let mut route = Ppi::new_one_to_one(timing.ppi, hwt.cc(CC_SCHED).event_compare(), trigger.task_set());
    route.enable();

    defmt::info!(
        "m5_tx: HARDWARE-SCHEDULED TX armed — TIMER20.CC[{}] -> DPPI -> GPIOTE -> DIO8 TxTrigger, period {=u32} ticks (20 ms)",
        CC_SCHED,
        PERIOD_TICKS
    );

    let mut tx_done: u32 = 0;
    let mut advances: u32 = 0;
    let mut last_armed: u32 = u32::MAX;
    let mut armed: u32 = 0;
    let mut busy_err: u32 = 0;
    let mut pll_err: u32 = 0;
    let mut silent_no_err: u32 = 0;
    let mut target = hwt.cc(5).capture().wrapping_add(PERIOD_TICKS);

    loop {
        // Re-derive the next transmit instant FROM THE CLOCK rather than by accumulating.
        //
        // Accumulating (`target += PERIOD`) fails silently and permanently: the moment one iteration
        // takes longer than a period the target slips into the past, the compare never matches
        // again, and transmission stops until the 32-bit counter wraps (~4.5 min at 16 MHz).
        // Measured exactly that — 74 transmits, then nothing. This is not a quirk of this binary: it
        // is the same defect a slot scheduler has if it advances its slot pointer by addition
        // instead of recomputing the next boundary from the common-view clock. Carry it into #84/#85.
        let now = hwt.cc(5).capture();
        while target.wrapping_sub(now) > i32::MAX as u32 || target.wrapping_sub(now) < MIN_LEAD_TICKS {
            target = target.wrapping_add(PERIOD_TICKS);
            advances = advances.wrapping_add(1);
        }

        // **Arm each target exactly once.**
        //
        // Measured root cause of the dropped slots (#103), and it was not the radio: with a 15 ms
        // sleep against a 20 ms period, roughly one iteration in four came round while `target` was
        // still in the future and re-armed a compare that had ALREADY fired. Re-writing a CC value
        // that the counter has passed produces no event, so that slot silently transmitted nothing.
        // The chip's own `chip_busy` flag never asserted once in 1292 armed slots — the suspicion
        // that the part was "busy changing mode" was wrong, and only the error read disproved it.
        if target == last_armed {
            Timer::after(Duration::from_millis(2)).await;
            continue;
        }
        last_armed = target;

        // **The sequence number IS the slot index.** Deriving it from the scheduled tick rather than
        // counting loop iterations means "consecutive sequence numbers" and "consecutive slots" are
        // the same statement by construction — so the receiver's consecutive-pairs filter selects
        // exactly the pairs that were scheduled one period apart, and a skipped slot can never
        // masquerade as transmit jitter. Same reason a slot MAC keys on `clock / slot_us` rather
        // than on a counter it increments.
        let slot = target / PERIOD_TICKS;

        let mut frame = [0u8; 4 + 6];
        frame[..4].copy_from_slice(&slot.to_be_bytes());
        frame[4..].copy_from_slice(TAG);

        // Everything slow happens BEFORE the scheduled instant: the pin is released low so the next
        // compare makes a clean rising edge, then the FIFO is loaded over SPI. By the time the
        // compare fires, the radio has nothing left to do but transmit.
        trigger.clear();
        // Clear before every write: `wr_tx_fifo_from` APPENDS, so a slot whose trigger did not fire
        // leaves its frame in the FIFO. Without this the FIFO monotonically fills and TX degrades —
        // which looks like a scheduling failure but is a buffer-management one.
        let _ = radio.clear_tx_fifo().await;
        let lvl_before = radio.get_tx_fifo_lvl().await.unwrap_or(0xffff);
        if let Err(e) = radio.wr_tx_fifo_from(&frame).await {
            defmt::error!("m5_tx: fifo write: {}", defmt::Debug2Format(&e));
        }
        let lvl_after = radio.get_tx_fifo_lvl().await.unwrap_or(0xffff);
        hwt.cc(CC_SCHED).clear_events();
        hwt.cc(CC_SCHED).write(target);

        // Sleep until *past* the scheduled instant, computed from the hardware clock.
        //
        // A fixed sleep was wrong for a subtle reason worth recording: the target can be up to a
        // full period ahead, so a fixed 15 ms wake-up frequently polled `tx_done` BEFORE the
        // transmit had happened, then cleared the IRQ — making a perfectly good slot read as
        // "silent". Roughly a quarter of the apparent failures were the instrument, not the radio.
        // The sleep itself is still deliberately sloppy; it just has to land on the correct side of
        // the event it is measuring.
        let after_arm = hwt.cc(5).capture();
        let remaining_ticks = target.wrapping_sub(after_arm);
        let wait_ms = if remaining_ticks > i32::MAX as u32 { 0 } else { remaining_ticks / 16_000 };
        Timer::after(Duration::from_millis(wait_ms as u64 + 3)).await;

        let fired = radio.get_and_clear_irq().await.map(|i| i.tx_done()).unwrap_or(false);
        if fired {
            tx_done = tx_done.wrapping_add(1);
        }

        // Per-slot error read — the root-cause probe for #103.
        //
        // `chip_busy` is the chip's own statement that "a DIO TX or RX trigger could not be executed
        // because chip was busy changing mode". Errors are sticky until ClearErrors, so they are
        // cleared every slot; otherwise one early error would read as every slot failing.
        match radio.get_errors().await {
            Ok(e) => {
                if e.chip_busy() {
                    busy_err = busy_err.wrapping_add(1);
                }
                if e.pll_lock() {
                    pll_err = pll_err.wrapping_add(1);
                }
                if !fired && !e.chip_busy() {
                    // The interesting case: no transmit AND the chip does not blame busy-mode.
                    silent_no_err = silent_no_err.wrapping_add(1);
                }
            }
            Err(_) => {}
        }
        let _ = radio.cmd_wr(&lr2021::cmd::cmd_system::clear_errors_cmd()).await;

        if slot % 100 == 0 {
            defmt::info!(
                "m5_tx: slot {} | armed {} tx_done {} | chip_busy {} pll {} silent-no-err {} | period advances {}",
                slot,
                armed,
                tx_done,
                busy_err,
                pll_err,
                silent_no_err,
                advances
            );
            defmt::info!("        | tx fifo lvl before {=u16} after {=u16}", lvl_before, lvl_after);
        }
        armed = armed.wrapping_add(1);
    }
}
