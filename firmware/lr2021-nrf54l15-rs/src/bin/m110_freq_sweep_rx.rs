//! **#108 decisive test — is the residual corruption a carrier frequency offset?**
//!
//! The known-pattern dump (`m109_pattern_rx`) showed correct bytes at correct indices with
//! *alternating* runs of the ramp bit-inverted — periodic polarity inversion, consistent with a
//! carrier offset of order a few kHz (~3 ppm at 2477 MHz). That is a mechanism, and five previous
//! mechanisms in this chase were plausible, consistent, and **wrong**. So this binary does not
//! argue; it sweeps.
//!
//! For each candidate RX offset it retunes with `set_rf(FREQ_HZ + off)`, collects `N_PER_STEP`
//! frames of the known ramp, and scores them. Two scores, because they fail differently:
//!
//! - **`lead`** — leading bytes matching `00,01,02,…` before the first error. Sensitive to *where*
//!   the first polarity flip lands, so it tracks the flip period directly.
//! - **`hits`** — total bytes equal to their index, anywhere in the frame. Immune to a single early
//!   flip, so it measures overall demod health rather than the first failure.
//!
//! ## Reading the result
//!
//! | outcome | conclusion |
//! |---|---|
//! | a clear peak at some non-zero offset | **frequency offset confirmed** — and that offset is the correction |
//! | peak at 0, falling either side | the boards are already co-tuned; the inversion is *not* offset-driven and this mechanism joins the other five |
//! | flat / noise across the whole sweep | the corruption is independent of tuning — look upstream of the demod |
//!
//! The sweep deliberately runs symmetric about zero and reports every step, including the bad ones:
//! a peak is only meaningful against the shape of the curve around it.

#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use panic_probe as _;

use lr2021::system::ChipMode;
use lr2021_nrf54l15_rs::{flrc_link, hw};

/// Offsets to try, in kHz, applied to the RX centre frequency.
///
/// Range chosen from the m109 estimate (~7 kHz implied by a ~37 µs flip half-period) with generous
/// margin either side, so a peak has curve on both flanks rather than sitting at an endpoint.
const OFFSETS_KHZ: [i32; 13] = [-60, -40, -30, -20, -12, -6, 0, 6, 12, 20, 30, 40, 60];

/// Frames scored per step. Enough to average out per-frame noise without making the sweep so long
/// that oscillator drift becomes a confound within a single pass.
const N_PER_STEP: u32 = 24;

/// Give up on a step that is receiving nothing, so one dead offset cannot stall the whole sweep.
const STEP_TIMEOUT_MS: u64 = 4000;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());
    let (mut radio, _t, _u) = hw::init(p);

    radio.reset().await.unwrap();
    Timer::after(Duration::from_millis(50)).await;
    if radio.get_version().await.is_err() {
        defmt::panic!("m110: no radio");
    }
    flrc_link::configure(&mut radio).await.expect("configure");
    flrc_link::set_crc_off(&mut radio).await.expect("crc off");

    defmt::info!(
        "m110_freq_sweep_rx: {=usize} offsets x {=u32} frames, base {=u32} Hz",
        OFFSETS_KHZ.len(),
        N_PER_STEP,
        flrc_link::FREQ_HZ
    );
    defmt::info!("columns: off kHz | frames | lead_avg | lead_max | hits_avg/48 | perfect");

    let mut best = (i32::MIN, 0u32);

    for off_khz in OFFSETS_KHZ {
        let f = (flrc_link::FREQ_HZ as i64 + (off_khz as i64) * 1000) as u32;
        // `SetRfFrequency` is a standby-side command. The first pass of this sweep left the chip in
        // continuous RX and every retune after the first returned an error — which is the chip
        // correctly refusing an illegal state transition, not a tuning limit. Drop to standby first.
        if radio.set_chip_mode(ChipMode::StandbyXosc).await.is_err() {
            defmt::warn!("  {=i32}: standby FAILED", off_khz);
            continue;
        }
        if radio.set_rf(f).await.is_err() {
            defmt::warn!("  {=i32}: set_rf FAILED", off_khz);
            continue;
        }
        // Re-enter RX after retuning: set_rf is a standby-side operation, so the previous continuous
        // RX does not survive it. Clearing the FIFO first stops a frame captured at the *previous*
        // offset from being scored against this one.
        let _ = radio.clear_rx_fifo().await;
        let _ = radio.get_and_clear_irq().await;
        if radio.set_rx_continous().await.is_err() {
            defmt::warn!("  {=i32}: set_rx FAILED", off_khz);
            continue;
        }

        let (mut n, mut lead_sum, mut lead_max, mut hits_sum, mut perfect) = (0u32, 0u32, 0u32, 0u32, 0u32);
        let deadline = embassy_time::Instant::now() + Duration::from_millis(STEP_TIMEOUT_MS);

        while n < N_PER_STEP && embassy_time::Instant::now() < deadline {
            let irq = match radio.get_and_clear_irq().await {
                Ok(i) => i,
                Err(_) => continue,
            };
            if !irq.rx_done() {
                Timer::after(Duration::from_micros(200)).await;
                continue;
            }
            let mut b = [0u8; flrc_link::FRAME_LEN as usize];
            let ok = radio.rd_rx_fifo_to(&mut b).await.is_ok();
            let _ = radio.clear_rx_fifo().await;
            if !ok {
                continue;
            }

            let lead = b.iter().enumerate().take_while(|(i, v)| **v == *i as u8).count() as u32;
            let hits = b.iter().enumerate().filter(|(i, v)| **v == *i as u8).count() as u32;
            n += 1;
            lead_sum += lead;
            lead_max = lead_max.max(lead);
            hits_sum += hits;
            if hits as usize == flrc_link::FRAME_LEN as usize {
                perfect += 1;
            }
        }

        if n == 0 {
            defmt::info!("  off={=i32} kHz: 0 frames (nothing received at this offset)", off_khz);
            continue;
        }
        let lead_avg = lead_sum / n;
        defmt::info!(
            "  off={=i32} kHz  n={=u32}  lead_avg={=u32}  lead_max={=u32}  hits_avg={=u32}/48  perfect={=u32}",
            off_khz,
            n,
            lead_avg,
            lead_max,
            hits_sum / n,
            perfect
        );
        if hits_sum / n > best.1 {
            best = (off_khz, hits_sum / n);
        }
    }

    defmt::info!("m110 DONE. best offset {=i32} kHz with hits_avg {=u32}/48", best.0, best.1);
    defmt::info!("  peak at 0 => offset is NOT the mechanism; peak elsewhere => that is the correction");
    loop {
        Timer::after(Duration::from_secs(5)).await;
    }
}
