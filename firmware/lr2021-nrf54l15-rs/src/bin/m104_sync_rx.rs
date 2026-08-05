//! **#104 — is the residual jitter the radio's packet-done latency?** Measures the same frames two
//! ways and prints both spreads side by side.
//!
//! M5 left 58.9 µs of end-to-end spread and could not say how much of it was the *transmitter* and
//! how much the *receiver's* internal demodulate-to-DIO path. The DIO edge marks **packet-done**, so
//! its timing inherits payload length and post-processing. The LR2021 can do better without any
//! firmware work:
//!
//! - `SetTimestampSource(TS0, SYNC)` — stamp the **syncword detection**, a fixed and early point in
//!   the frame, independent of payload length and of packet post-processing.
//! - `GetTimestampValue(TS0)` — returns the delay from that event to the **SPI NSS falling edge of
//!   the request**, in HF clock ticks (32 MHz ⇒ 31.25 ns).
//!
//! Reconstruction onto our own 16 MHz timer:
//!
//! ```text
//!   sync_instant ≈ (software capture taken just before the request) − hf_ticks / 2
//! ```
//!
//! The offset between that software capture and the true NSS edge is *constant* — M4 measured the
//! software path at 0 ticks of jitter — so it cancels in a spread, which is all we compare.
//!
//! **Read the comparison, not the absolute values.** Both columns are inter-arrival spreads over
//! consecutive slots from the same frames; if the SYNC column is tighter, the packet-done latency was
//! a real contributor and the MAC should stamp on SYNC.

#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use panic_probe as _;

use lr2021::radio::{TimestampIndex, TimestampSource};
use lr2021_nrf54l15_rs::timing::{HwStamp, RxCapture, Spread};
use lr2021_nrf54l15_rs::{flrc_link, hw};

const TAG: &[u8] = b"NDN-M4";
const REPORT_EVERY: u32 = 100;

/// LR2021 HF clock is 32 MHz; our MAC timer is 16 MHz. Two HF ticks per timer tick.
const HF_TICKS_PER_TIMER_TICK: u32 = 2;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());
    let (mut radio, timing, _u) = hw::init(p);

    radio.reset().await.unwrap();
    Timer::after(Duration::from_millis(50)).await;
    if radio.get_version().await.is_err() {
        defmt::panic!("m104: no radio");
    }
    flrc_link::configure(&mut radio).await.expect("FLRC configure");

    // The whole point of this binary.
    radio
        .set_timestamp_source(TimestampIndex::Ts0, TimestampSource::Sync)
        .await
        .expect("timestamp source = SYNC");

    let mut cap = RxCapture::new(timing);
    radio.set_rx_continous().await.expect("rx");
    defmt::info!("m104_sync_rx: TS0=SYNC armed; comparing DIO(packet-done) vs SYNC stamps");

    let (mut got, mut ts_fail) = (0u32, 0u32);
    let mut ia_dio = Spread::default();
    let mut ia_sync = Spread::default();
    let mut prev: Option<(u32, HwStamp, u32)> = None;
    let mut buf = [0u8; 64];

    loop {
        cap.wait_edge().await;
        let hw = cap.hw_stamp();

        // Software capture immediately before the request: its offset to the real NSS edge is
        // constant, so it cancels in the spread.
        let at_req = cap.sw_stamp();
        let hf = radio.get_timestamp(TimestampIndex::Ts0).await;

        let irq = match radio.get_and_clear_irq().await {
            Ok(i) => i,
            Err(_) => continue,
        };
        if !irq.rx_done() {
            continue;
        }
        let len = radio.get_rx_pkt_len().await.unwrap_or(0) as usize;
        let n = len.min(buf.len());
        if radio.rd_rx_fifo_to(&mut buf[..n]).await.is_err() || n < 10 || &buf[4..10] != TAG {
            continue;
        }

        let hf_ticks = match hf {
            Ok(v) => v,
            Err(_) => {
                ts_fail = ts_fail.wrapping_add(1);
                continue;
            }
        };
        // Walk the software capture back to the SYNC instant, on our own timer.
        let sync_ticks = at_req.ticks.wrapping_sub(hf_ticks / HF_TICKS_PER_TIMER_TICK);

        let seq = u32::from_be_bytes([buf[0], buf[1], buf[2], buf[3]]);
        got = got.wrapping_add(1);
        if let Some((pseq, phw, psync)) = prev {
            if seq == pseq.wrapping_add(1) {
                ia_dio.push(hw.since(phw));
                ia_sync.push(sync_ticks.wrapping_sub(psync));
            }
        }
        prev = Some((seq, hw, sync_ticks));

        if got % REPORT_EVERY == 0 {
            defmt::info!(
                "m104 n={} | DIO  (packet-done) min={=u32} mean={=u32} max={=u32} p2p={=u32} ticks",
                ia_dio.n,
                ia_dio.min,
                ia_dio.mean(),
                ia_dio.max,
                ia_dio.peak_to_peak()
            );
            defmt::info!(
                "      | SYNC (chip stamp)  min={=u32} mean={=u32} max={=u32} p2p={=u32} ticks | ts_fail {}",
                ia_sync.min,
                ia_sync.mean(),
                ia_sync.max,
                ia_sync.peak_to_peak(),
                ts_fail
            );
        }
    }
}
