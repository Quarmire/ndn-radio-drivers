//! Empirically verify the BW16's radio knobs affect the raw-inject path: set a
//! fixed TX rate on the BW16, inject marked frames, and read back the MCS the
//! RTL8812EU actually captures. Proves whether wifi_set_tx_data_rate reaches the
//! management-frame injection path (vs only the STA data path).
//!   cargo run --example bw16_knobs -- /dev/cu.usbserial-1110 149

fn main() -> Result<(), Box<dyn std::error::Error>> {
    tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()?
        .block_on(run())
}

async fn run() -> Result<(), Box<dyn std::error::Error>> {
    use bytes::Bytes;
    use ndn_frame_io::{FrameFormat, FrameIo, InjectFrame, McsDescriptor, TxIntent};
    use ndn_radio_drivers::{Bw16SerialBackend, LibUsbRtl88xxBackend};
    use std::collections::BTreeMap;
    use std::sync::Arc;
    use std::time::{Duration, Instant};

    let mut a = std::env::args().skip(1);
    let port = a.next().unwrap_or_else(|| "/dev/cu.usbserial-1110".into());
    let ch: u8 = a.next().and_then(|s| s.parse().ok()).unwrap_or(149);

    let bw = Arc::new(Bw16SerialBackend::open(&port)?);
    bw.set_channel(ch)?;
    let eu = Arc::new(LibUsbRtl88xxBackend::open_monitor(ch)?.with_format(FrameFormat::Raw80211));
    let _p = eu.spawn_rx_pump(8);
    println!("BW16 inject → 8812EU capture on ch{ch}; per rate, the MCS the 8812EU sees:");

    // (label, wifi_set_tx_data_rate code): OFDM 6M, HT MCS0, HT MCS7, auto.
    for (label, code) in [("OFDM_6M", 0x04u8), ("HT_MCS0", 0x0c), ("HT_MCS7", 0x13), ("AUTO", 0xFF)] {
        bw.set_tx_rate(code)?;
        tokio::time::sleep(Duration::from_millis(300)).await;
        let marker = format!("KNOB-{label}");
        let dev = bw.clone();
        let mk = marker.clone();
        let inj = tokio::spawn(async move {
            for _ in 0..120 {
                let f = InjectFrame::broadcast(Bytes::from(mk.clone().into_bytes()), TxIntent::CONSERVATIVE);
                let _ = dev.inject_at(f, McsDescriptor::ht(0)).await;
                tokio::time::sleep(Duration::from_millis(12)).await;
            }
        });
        let mut mcs: BTreeMap<Option<u8>, u32> = BTreeMap::new();
        let deadline = Instant::now() + Duration::from_millis(1800);
        while Instant::now() < deadline {
            if let Ok(Ok(f)) = tokio::time::timeout(Duration::from_millis(250), eu.recv_frame()).await {
                if f.payload.windows(marker.len()).any(|w| w == marker.as_bytes()) {
                    *mcs.entry(f.mcs_index).or_insert(0) += 1;
                }
            }
        }
        inj.abort();
        println!("  rate {label:8} (0x{code:02x}): captured MCS histogram = {mcs:?}");
    }
    Ok(())
}
