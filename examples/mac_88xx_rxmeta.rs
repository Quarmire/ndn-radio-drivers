//! Deferred HW validation on the Mac's RTL8812EU: bring up the 88xx in monitor mode, RX ambient,
//! and confirm the per-frame RXTSFL hardware stamp (via the shared realtek_rx::rx_stamp) is a
//! monotonic microsecond counter — the 88xx sibling of the 8733b RX-stamp check.
use ndn_frame_io::FrameIo;
use ndn_radio_drivers::LibUsbRtl88xxBackend;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(36);
    let rt = tokio::runtime::Builder::new_current_thread().enable_all().build()?;
    rt.block_on(async move {
        let d = LibUsbRtl88xxBackend::open_monitor(ch)?;
        println!("8812EU up on ch{ch}; capturing ambient (NDN_RX_META_DBG prints RX stamps)...");
        for _ in 0..80 {
            let _ = tokio::time::timeout(std::time::Duration::from_millis(300), d.recv_frame()).await;
        }
        Ok::<(), Box<dyn std::error::Error>>(())
    })
}
