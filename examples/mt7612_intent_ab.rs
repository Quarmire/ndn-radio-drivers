//! **MT7612U: does `TxIntent::needs_basic_rate` reach the air?** — the sibling of
//! `examples/intent_ab.rs`, which cannot be used here because `open_named_radio` has no MT7612U
//! arm (deliberately: this part needs a cold device and a captured channel program, and a generic
//! opener must not grab it by PID).
//!
//! Same experiment, same on-air labelling, so the two results are directly comparable:
//!
//! ```text
//!   MostRobust  src 02:4e:44:4e:00:a0  -> must air at 6.0 Mb/s legacy OFDM (TXWI 0x2000)
//!   Throughput  src 02:4e:44:4e:00:b0  -> must air at the stored MCS
//! ```
//!
//! The stored rate is set FIRST, so a `MostRobust` frame airing at 6 Mbps can only be the intent
//! overriding it — seeing 6 Mbps alone would prove nothing.
//!
//! ⚠ This part is documented as needing a COLD device: a warm re-open wedges it and costs a
//! physical replug. Hand it over with `mt76_acquire.sh acquire 7612` rather than letting libusb
//! auto-detach the kernel driver, which powers the chip down under us.
//!
//!   export NDN_INTENT_FPS=100 NDN_INTENT_MCS=7 ; sudo -E ./mt7612_intent_ab [seconds]
use ndn_frame_io::{FrameFormat, FrameIo, InjectFrame, Reliability, TxIntent};
use ndn_radio_drivers::Mt7612uBackend;
use ndn_radio_hal::McsDescriptor;
use std::sync::Arc;
use std::time::{Duration, Instant};

const SRC_ROBUST: [u8; 6] = [0x02, 0x4e, 0x44, 0x4e, 0x00, 0xa0];
const SRC_BULK: [u8; 6] = [0x02, 0x4e, 0x44, 0x4e, 0x00, 0xb0];

fn main() -> Result<(), Box<dyn std::error::Error>> {
    tokio::runtime::Runtime::new()?.block_on(run())
}

async fn run() -> Result<(), Box<dyn std::error::Error>> {
    let secs: u64 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(10);
    let fps: u64 = std::env::var("NDN_INTENT_FPS")
        .ok()
        .and_then(|v| v.parse().ok())
        .filter(|&n| n > 0)
        .unwrap_or(100);
    let mcs_index: u8 = std::env::var("NDN_INTENT_MCS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(7);

    let dev =
        Arc::new(Mt7612uBackend::open()?.with_format(FrameFormat::RawNdn { ethertype: 0x8624 }));
    dev.bring_up()?;
    // 2.4 GHz ch6 — the captured baseline program, and the band a 2.4 GHz witness can watch.
    dev.set_channel_ch6()?;
    println!("MT7612U up on ch6");

    let stored = McsDescriptor::ht(mcs_index);
    match FrameIo::set_rate(dev.as_ref(), stored) {
        Ok(()) => println!("stored rate: HT MCS{mcs_index} (what MostRobust must override)"),
        Err(e) => println!("⚠ set_rate refused ({e}) — the A/B below cannot mean anything; void."),
    }

    let payload = bytes::Bytes::from(b"NDNINTENTAB0123456789".to_vec());
    let period = Duration::from_micros(1_000_000 / fps.max(1));
    let t = Instant::now();
    let (mut n_robust, mut n_bulk, mut errs) = (0u64, 0u64, 0u64);
    let mut robust_turn = true;

    while t.elapsed() < Duration::from_secs(secs) {
        let (intent, src) = if robust_turn {
            (TxIntent::broadcast(Reliability::MostRobust), SRC_ROBUST)
        } else {
            (TxIntent::broadcast(Reliability::Throughput), SRC_BULK)
        };
        let mut f = InjectFrame::broadcast(payload.clone(), intent);
        f.src = src;
        match FrameIo::inject(dev.as_ref(), f).await {
            Ok(()) => {
                if robust_turn {
                    n_robust += 1
                } else {
                    n_bulk += 1
                }
            }
            Err(_) => errs += 1,
        }
        robust_turn = !robust_turn;
        tokio::time::sleep(period).await;
    }
    let el = t.elapsed().as_secs_f64();
    println!("sent: robust={n_robust} bulk={n_bulk} err={errs} in {el:.1}s");
    println!("⚠ USB writes ACCEPTED, not radiation — the witness decides (a0 => 6.0 Mb/s, b0 => MCS{mcs_index}).");
    Ok(())
}
