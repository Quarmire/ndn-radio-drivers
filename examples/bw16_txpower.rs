//! Verify the reimplemented TX-power knob on the Rust firmware: sweep the power
//! index, inject marked frames, and read back the mean RSSI the RTL8812EU sees.
//! A monotonic RSSI change proves `txpower patha=N` reaches the radio.
//!   cargo run --features bw16 --example bw16_txpower -- /dev/cu.usbserial-1110 149

fn main() -> Result<(), Box<dyn std::error::Error>> {
    tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()?
        .block_on(run())
}

async fn run() -> Result<(), Box<dyn std::error::Error>> {
    use bytes::Bytes;
    use ndn_frame_io::{FrameFormat, FrameIo, InjectFrame, McsDescriptor, TxIntent, WifiRadio};
    use ndn_radio_drivers::{Bw16SerialBackend, LibUsbRtl88xxBackend};
    use std::sync::Arc;
    use std::time::{Duration, Instant};

    let mut a = std::env::args().skip(1);
    let port = a.next().unwrap_or_else(|| "/dev/cu.usbserial-1110".into());
    let ch: u8 = a.next().and_then(|s| s.parse().ok()).unwrap_or(149);

    let bw = Arc::new(Bw16SerialBackend::open(&port)?);
    bw.set_channel(ch)?;
    let eu = Arc::new(LibUsbRtl88xxBackend::open_monitor(ch)?.with_format(FrameFormat::Raw80211));
    let _p = eu.spawn_rx_pump(8);
    println!("BW16 inject → 8812EU capture on ch{ch}; mean RSSI vs TX-power index:");

    for idx in [0u8, 16, 32, 48, 63] {
        bw.set_txpower(idx)?;
        tokio::time::sleep(Duration::from_millis(300)).await;
        let dev = bw.clone();
        let inj = tokio::spawn(async move {
            for _ in 0..150 {
                let f = InjectFrame::broadcast(Bytes::from_static(b"PWR-SWEEP"), TxIntent::CONSERVATIVE);
                let _ = dev.inject_at(f, McsDescriptor::ht(0)).await;
                tokio::time::sleep(Duration::from_millis(10)).await;
            }
        });
        let (mut sum, mut n) = (0i32, 0i32);
        let deadline = Instant::now() + Duration::from_millis(1800);
        while Instant::now() < deadline {
            if let Ok(Ok(f)) = tokio::time::timeout(Duration::from_millis(250), eu.recv_frame()).await {
                if f.payload.windows(9).any(|w| w == b"PWR-SWEEP") {
                    if let Some(r) = f.rssi_dbm {
                        sum += r as i32;
                        n += 1;
                    }
                }
            }
        }
        inj.abort();
        let mean = if n > 0 { sum as f32 / n as f32 } else { 0.0 };
        println!("  txpower idx {idx:2}: n={n:3}  mean RSSI = {mean:6.1} dBm");
    }
    Ok(())
}
