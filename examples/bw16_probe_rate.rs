//! RE the BW16's pkt_attrib rate offset: for each candidate byte offset in the
//! driver's opaque pkt_attrib, inject a frame with that byte set to MGN_MCS7
//! (0x87) and read back the MCS the RTL8812EU captures. The offset that turns the
//! frame into an HT MCS frame is the `rate` field — the key to a rate-controllable
//! inject. One 8812EU open for the whole sweep.
//!   cargo run --features bw16 --example bw16_probe_rate -- /dev/cu.usbserial-1110 149

fn main() -> Result<(), Box<dyn std::error::Error>> {
    tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()?
        .block_on(run())
}

async fn run() -> Result<(), Box<dyn std::error::Error>> {
    use bytes::Bytes;
    use ndn_frame_io::{frame, FrameFormat, FrameIo, InjectFrame, TxIntent};
    use ndn_radio_drivers::{Bw16SerialBackend, LibUsbRtl88xxBackend};
    use std::collections::BTreeMap;
    use std::sync::Arc;
    use std::time::{Duration, Instant};

    let mut a = std::env::args().skip(1);
    let port = a.next().unwrap_or_else(|| "/dev/cu.usbserial-1110".into());
    let ch: u8 = a.next().and_then(|s| s.parse().ok()).unwrap_or(149);
    let rate_code: u8 = a.next().and_then(|s| u8::from_str_radix(s.trim_start_matches("0x"), 16).ok()).unwrap_or(0x87);
    let max_off: u8 = a.next().and_then(|s| s.parse().ok()).unwrap_or(56);

    let bw = Arc::new(Bw16SerialBackend::open(&port)?);
    bw.set_channel(ch)?;
    let eu = Arc::new(LibUsbRtl88xxBackend::open_monitor(ch)?.with_format(FrameFormat::Raw80211));
    let _p = eu.spawn_rx_pump(8);

    // One marked 802.11 frame (RawNdn), reused with each pkt_attrib poke.
    let inj = InjectFrame::broadcast(Bytes::from_static(b"RATEPROBE"), TxIntent::CONSERVATIVE);
    let dot11 = frame::build_dot11(FrameFormat::default(), &inj)?;
    println!("probing pkt_attrib offsets 0..{max_off} with rate=0x{rate_code:02x}; 8812EU MCS per offset:");
    println!("(baseline mgmt inject captures as legacy → mcs_index=None; an HT hit = Some(n))");

    let mut hits = Vec::new();
    for off in 0..=max_off {
        let dev = bw.clone();
        let f = dot11.clone();
        let injector = tokio::spawn(async move {
            for _ in 0..60 {
                let _ = dev.inject_attr(&f, &[(off, rate_code)]);
                tokio::time::sleep(Duration::from_millis(10)).await;
            }
        });
        let mut mcs: BTreeMap<Option<u8>, u32> = BTreeMap::new();
        let deadline = Instant::now() + Duration::from_millis(700);
        while Instant::now() < deadline {
            if let Ok(Ok(fr)) = tokio::time::timeout(Duration::from_millis(150), eu.recv_frame()).await {
                if fr.payload.windows(9).any(|w| w == b"RATEPROBE") {
                    *mcs.entry(fr.mcs_index).or_insert(0) += 1;
                }
            }
        }
        injector.abort();
        let ht = mcs.keys().any(|k| k.is_some());
        if ht || mcs.values().sum::<u32>() == 0 {
            println!("  off {off:2} (0x{off:02x}): {mcs:?}{}", if ht { "  <== HT!" } else { "  (no capture)" });
            if ht {
                hits.push(off);
            }
        }
    }
    println!("\nHT-producing offsets (candidate rate field): {hits:?}");
    Ok(())
}
