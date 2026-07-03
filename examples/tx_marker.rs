//! Inject a marked RawNdn frame repeatedly on the RTL8812EU — the TX side of a
//! cross-radio link test (peer captures it and looks for the marker).
//!   cargo run --example tx_marker -- 149

fn main() -> Result<(), Box<dyn std::error::Error>> {
    tokio::runtime::Runtime::new()?.block_on(run())
}

async fn run() -> Result<(), Box<dyn std::error::Error>> {
    use bytes::Bytes;
    use ndn_frame_io::{FrameIo, InjectFrame, TxIntent};
    use ndn_radio_drivers::LibUsbRtl88xxBackend;
    use std::time::Duration;

    let ch: u8 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(149);
    let dev = LibUsbRtl88xxBackend::open_monitor(ch)?;
    let payload = Bytes::from_static(b"NDN-8812-CAFE-XMIT");
    println!("RTL8812EU injecting {payload:?} on ch{ch}, 600x…");
    for _ in 0..600 {
        let f = InjectFrame::broadcast(payload.clone(), TxIntent::CONSERVATIVE);
        let _ = dev.inject(f).await;
        tokio::time::sleep(Duration::from_millis(20)).await;
    }
    println!("done");
    Ok(())
}
