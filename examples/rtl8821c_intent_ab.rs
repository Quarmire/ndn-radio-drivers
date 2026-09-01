//! **RTL8821CU: does `TxIntent::needs_basic_rate` reach the air?** — the 8821cu sibling of
//! `examples/intent_ab.rs`, which cannot be used here because `open_named_radio` has no arm for
//! this part (its `knobs` seam is deliberately `Excluded` — no hardware-verified actuator — so a
//! factory arm would hand back a half-populated `OpenRadio`).
//!
//! Same experiment and the same on-air labelling as the other three, so the results compare
//! directly:
//!
//! ```text
//!   MostRobust  src 02:4e:44:4e:00:a0  -> must air at 6.0 Mb/s legacy (DESC_RATE_OFDM6M)
//!   Throughput  src 02:4e:44:4e:00:b0  -> must air at the stored MCS
//! ```
//!
//! The stored rate is set FIRST. That matters more here than anywhere: this backend's
//! `resolved_mcs` consults the frame's intent ONLY when `cur_mcs` is unset, so before the fix a
//! stored rate silently won and `MostRobust` never reached the DESC code. Setting `cur_mcs` is
//! therefore reproducing the exact state in which the bug was invisible.
//!
//!   export NDN_INTENT_FPS=100 NDN_INTENT_MCS=7 ; sudo -E ./rtl8821c_intent_ab [channel] [seconds]
use ndn_frame_io::{FrameIo, InjectFrame, Reliability, TxIntent};
use ndn_radio_drivers::Rtl8821cuBackend;
use ndn_radio_hal::McsDescriptor;
use std::sync::Arc;
use std::time::{Duration, Instant};

const SRC_ROBUST: [u8; 6] = [0x02, 0x4e, 0x44, 0x4e, 0x00, 0xa0];
const SRC_BULK: [u8; 6] = [0x02, 0x4e, 0x44, 0x4e, 0x00, 0xb0];

fn main() -> Result<(), Box<dyn std::error::Error>> {
    tokio::runtime::Runtime::new()?.block_on(run())
}

async fn run() -> Result<(), Box<dyn std::error::Error>> {
    let mut a = std::env::args().skip(1);
    let channel: u8 = a.next().and_then(|s| s.parse().ok()).unwrap_or(6);
    let secs: u64 = a.next().and_then(|s| s.parse().ok()).unwrap_or(10);
    let fps: u64 = std::env::var("NDN_INTENT_FPS")
        .ok()
        .and_then(|v| v.parse().ok())
        .filter(|&n| n > 0)
        .unwrap_or(100);
    let mcs_index: u8 = std::env::var("NDN_INTENT_MCS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(7);

    let dev = Arc::new(Rtl8821cuBackend::open()?);
    dev.bring_up(channel)?;
    println!("RTL8821CU up on ch{channel}");

    match FrameIo::set_rate(dev.as_ref(), McsDescriptor::ht(mcs_index)) {
        Ok(()) => println!("stored cur_mcs: HT MCS{mcs_index} (the state that hid the bug)"),
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
