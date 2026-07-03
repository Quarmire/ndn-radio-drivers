//! Monitor-mode RX capture on the RTL8812EU/8822E — for cross-radio TX proofs.
//! Opens the chip on a channel and prints captured RawNdn payloads, flagging any
//! that contain a marker string.
//!   cargo run --example rx_capture -- 149 CAFE

fn main() -> Result<(), Box<dyn std::error::Error>> {
    tokio::runtime::Runtime::new()?.block_on(run())
}

async fn run() -> Result<(), Box<dyn std::error::Error>> {
    use ndn_frame_io::{FrameFormat, FrameIo};
    use ndn_radio_drivers::LibUsbRtl88xxBackend;
    use std::time::{Duration, Instant};

    let mut a = std::env::args().skip(1);
    let ch: u8 = a.next().and_then(|s| s.parse().ok()).unwrap_or(149);
    let marker = a.next().unwrap_or_else(|| "CAFE".into());
    let mk = marker.as_bytes();

    // Raw80211: recv_frame returns *every* captured 802.11 frame verbatim, so a
    // non-zero total means the RX path works; a marker hit means we heard the peer.
    let dev =
        std::sync::Arc::new(LibUsbRtl88xxBackend::open_monitor(ch)?.with_format(FrameFormat::Raw80211));
    let _pumps = dev.spawn_rx_pump(8); // full-rate background RX
    println!("RTL8812EU monitor on ch{ch} (Raw80211, pumped); capturing 20s, flagging {marker:?}…");

    let deadline = Instant::now() + Duration::from_secs(20);
    let (mut total, mut hits) = (0u32, 0u32);
    while Instant::now() < deadline {
        match tokio::time::timeout(Duration::from_millis(500), dev.recv_frame()).await {
            Ok(Ok(f)) => {
                total += 1;
                let hit = f.payload.windows(mk.len()).any(|w| w == mk);
                if hit {
                    hits += 1;
                    let txt = String::from_utf8_lossy(&f.payload);
                    println!("  ✅ HIT #{hits}: {}B rssi={:?} src={:?} payload={:?}",
                             f.payload.len(), f.rssi_dbm, f.addr, &txt[..txt.len().min(40)]);
                }
            }
            _ => {}
        }
    }
    println!("captured {total} RawNdn frames, {hits} matched {marker:?}");
    Ok(())
}
