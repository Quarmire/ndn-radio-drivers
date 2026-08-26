//! **ESP32-C5 serial-bridge FrameIo test** — the existing `Bw16SerialBackend` drives the C5 UNCHANGED.
//!
//! Opens the C5's USB-Serial-JTAG port as a `Bw16SerialBackend`, sets the channel, injects a few NDN
//! 0x8624 frames (host builds the 802.11 frame; the C5 esp_wifi_80211_tx's it — witness with the mt76),
//! then drains `recv_frame` (the C5 forwards every 0x8624 frame it hears, e.g. from the mt76 injector).
//!
//! ```sh
//! cargo run --example c5_bridge_test --features serial-radio -- /dev/cu.usbmodem1101
//! ```
use std::time::Duration;

use bytes::Bytes;
use ndn_radio_drivers::Esp32SerialBackend;
use ndn_radio_hal::{Bandwidth, FrameIo, InjectFrame, RadioKnobs, RadioProfile, TxIntent};

#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let port = std::env::args().nth(1).unwrap_or_else(|| "/dev/cu.usbmodem1101".into());
    let dev = Esp32SerialBackend::open_c5(&port)?; // dual-band C5 over native USB-Serial-JTAG (no RTS/DTR reset)
    dev.set_channel(1, Bandwidth::Bw20)?;
    let cap = dev.capability();
    println!("C5 bridge open on {port}, ch1 — bands {:?}, channels {:?}", cap.bands, cap.channels);

    // TX: inject 20 NDN 0x8624 frames through the C5 (witness with the mt76).
    for i in 0..20u32 {
        let f = InjectFrame::broadcast(Bytes::copy_from_slice(format!("\x05\x08c5br-{i:02}").as_bytes()), TxIntent::CONSERVATIVE);
        dev.inject(f).await?;
        tokio::time::sleep(Duration::from_millis(50)).await;
    }
    println!("injected 20 frames via the C5");

    // RX: drain what the C5 forwards for 6s (run the mt76 injector concurrently). Each frame now carries
    // the C5's HARDWARE per-frame RX stamp (rx_ctrl.timestamp), so cap.stamp is a device-clock LinkStamp.
    let mut got = 0u32;
    let mut rssi_sum = 0i32;
    let mut raws: Vec<u64> = Vec::new();
    let deadline = tokio::time::Instant::now() + Duration::from_secs(6);
    while tokio::time::Instant::now() < deadline {
        match tokio::time::timeout(Duration::from_millis(500), dev.recv_frame()).await {
            Ok(Ok(cap)) => {
                got += 1;
                if let Some(r) = cap.rssi_dbm { rssi_sum += r as i32; }
                if let Some(st) = &cap.stamp { raws.push(st.raw); }
                if got <= 3 {
                    println!("  RX #{got}: {} B, rssi {:?}, stamp {:?}", cap.payload.len(), cap.rssi_dbm, cap.stamp);
                }
            }
            _ => {}
        }
    }
    println!("→ received {got} NDN frames via the C5 bridge (avg rssi {})",
        if got > 0 { rssi_sum / got as i32 } else { 0 });
    if raws.len() >= 3 {
        let mono = raws.windows(2).all(|w| w[1] >= w[0]);
        let mut gaps: Vec<i64> = raws.windows(2).map(|w| w[1] as i64 - w[0] as i64).filter(|&g| g > 0 && g < 1_000_000).collect();
        gaps.sort();
        let med = gaps.get(gaps.len() / 2).copied().unwrap_or(0);
        println!("→ hardware RX stamp: monotonic={mono}, median inter-frame gap {med} µs (device clock, not host-recv)");
    }
    Ok(())
}
