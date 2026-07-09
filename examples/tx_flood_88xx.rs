//! Flood-inject on the RTL8812EU/8822E (88xx) for N seconds at a chosen rate — HT-RX test source.
//! NDN_RADIO_TX_RATE=<dec> sets the DESC_RATE (12=HT-MCS0, 44=VHT-1SS-MCS0); default = intent MCS.
use bytes::Bytes;
use ndn_frame_io::{FrameIo, InjectFrame, TxIntent, BROADCAST, DEFAULT_SRC};
use ndn_radio_drivers::LibUsbRtl88xxBackend;
use std::time::{Duration, Instant};
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let secs: u64 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(20);
    let ch: u8 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(36);
    let pid: u16 = std::env::args().nth(3).and_then(|s| u16::from_str_radix(&s, 16).ok()).unwrap_or(0xa81a);
    let rt = tokio::runtime::Builder::new_current_thread().enable_all().build()?;
    rt.block_on(async move {
        let d = LibUsbRtl88xxBackend::open_monitor_pid(pid, ch)?;
        println!("88xx flood ch{ch} pid {pid:04x} for {secs}s (rate via NDN_RADIO_TX_RATE)");
        let frame = InjectFrame { payload: Bytes::from(vec![0x42u8; 300]), tx: TxIntent::ROBUST, dst: BROADCAST, src: DEFAULT_SRC };
        let end = Instant::now() + Duration::from_secs(secs);
        let mut n = 0u64;
        while Instant::now() < end { let _ = d.inject(frame.clone()).await; n += 1; }
        println!("injected {n} frames");
        Ok::<(), Box<dyn std::error::Error>>(())
    })
}
