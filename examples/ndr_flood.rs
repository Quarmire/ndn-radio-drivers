//! **Generic saturating transmitter** — open any radio by PID through `open_named_radio` and
//! flood a marked payload, so another radio's RX ceiling can be measured against a known offered
//! load rather than against whatever the air happens to carry.
//!
//! It exists because "160 frames/s" is uninterpretable on its own: it could be the receiver's
//! ceiling or the channel's content, and only a source that can out-run the receiver tells them
//! apart. (This repo has been caught by exactly that before — a "high drop ratio" that turned out
//! to be an observer's ~200 f/s RX limit, not on-air loss.)
//!
//!   sudo ./ndr_flood <pid-hex> <channel> <seconds> [payload]
use ndn_frame_io::{FrameIo, InjectFrame, Reliability, TxIntent};
use std::time::{Duration, Instant};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    tokio::runtime::Runtime::new()?.block_on(run())
}

async fn run() -> Result<(), Box<dyn std::error::Error>> {
    let mut a = std::env::args().skip(1);
    let pid = u16::from_str_radix(&a.next().unwrap_or_else(|| "f72b".into()), 16)?;
    let channel: u8 = a.next().and_then(|s| s.parse().ok()).unwrap_or(6);
    let secs: u64 = a.next().and_then(|s| s.parse().ok()).unwrap_or(20);
    let plen: usize = a.next().and_then(|s| s.parse().ok()).unwrap_or(200);

    let radio = ndn_radio_drivers::open_radio(
        pid,
        &ndn_radio_drivers::DeviceSelect::from_env(),
        &ndn_radio_drivers::BringUpRequest::from_env(channel),
    )?;
    println!("opened {pid:#06x} on ch{channel}");
    let mut body = b"NDRFLOODSRC".to_vec();
    while body.len() < plen {
        body.push(b'*');
    }
    let payload = bytes::Bytes::from(body);

    let t = Instant::now();
    let (mut sent, mut errs) = (0u64, 0u64);
    while t.elapsed() < Duration::from_secs(secs) {
        let f = InjectFrame::broadcast(
            payload.clone(),
            TxIntent::broadcast(Reliability::MostRobust),
        );
        match radio.io().inject(f).await {
            Ok(()) => sent += 1,
            Err(_) => errs += 1,
        }
    }
    let el = t.elapsed().as_secs_f64();
    println!(
        "offered {sent} frames in {el:.1}s = {:.0} f/s ({errs} errors), payload {plen} B",
        sent as f64 / el
    );
    Ok(())
}
