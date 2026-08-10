//! Flood-inject on the RTL8812AU as fast as possible for N seconds — an SDR TX-radiation check.
use bytes::Bytes;
use ndn_frame_io::{FrameIo, InjectFrame, TxIntent, BROADCAST, DEFAULT_SRC};
use ndn_radio_drivers::Rtl8812auBackend;
use std::time::{Duration, Instant};
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let secs: u64 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(20);
    let ch: u8 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(36);
    let rt = tokio::runtime::Builder::new_current_thread().enable_all().build()?;
    rt.block_on(async move {
        let d = Rtl8812auBackend::open()?; // finds 0x881a (8812AU) on the Mac
        d.bring_up_monitor(ch)?;
        println!("8812AU flood ch{ch} for {secs}s (legacy 6M)");
        // addr3: None = the legacy layout (no Tier-0 filter in addr1‖addr2, so no displaced nonce).
        let frame = InjectFrame { payload: Bytes::from(vec![0x42u8; 1400]), tx: TxIntent::ROBUST, dst: BROADCAST, src: DEFAULT_SRC, addr3: None };
        let end = Instant::now() + Duration::from_secs(secs);
        let mut n = 0u64;
        while Instant::now() < end { let _ = d.inject(frame.clone()).await; n += 1; }
        println!("injected {n} frames");
        Ok::<(), Box<dyn std::error::Error>>(())
    })
}
