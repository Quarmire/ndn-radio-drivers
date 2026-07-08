//! Validate RX-metadata extraction: bring up RX (monitor ch36), receive ambient frames, and
//! print the per-frame RSSI (dBm) + MCS index parsed from the RX descriptor + jgr3 phystatus.
//! Sane RSSI for ambient traffic is roughly -30 to -90 dBm.
use ndn_frame_io::FrameIo;
use ndn_radio_drivers::Rtl8733buBackend;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(36);
    let rt = tokio::runtime::Builder::new_current_thread().enable_all().build()?;
    rt.block_on(async move {
        let d = Rtl8733buBackend::open()?;
        d.bring_up_monitor(ch)?;
        let (mut n, mut rssi_seen) = (0, 0);
        while n < 40 {
            if let Ok(f) = d.recv_frame().await {
                n += 1;
                if f.rssi_dbm.is_some() { rssi_seen += 1; }
                if n <= 20 {
                    println!("frame#{n}: rssi={:?} dBm  mcs={:?}  len={}", f.rssi_dbm, f.mcs_index, f.payload.len());
                }
            }
        }
        println!("=== {rssi_seen}/{n} frames had RSSI ===");
        Ok::<(), Box<dyn std::error::Error>>(())
    })
}
