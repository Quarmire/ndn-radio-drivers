//! One half of a two-node COMMON-VIEW clock comparison.
//!
//! Logs `<seq> <rx_stamp_ticks>` for every frame received, where `seq` is the transmitter's frame
//! counter and the stamp is this chip's own hardware RXTSFL latch. Run this on BOTH f72bs while a
//! THIRD radio transmits: each receiver stamps the same frames on its own clock, so matching by
//! `seq` and regressing one stamp series against the other gives the ratio of the two crystals —
//! the transmitter's clock cancels entirely, which is what makes this a common-view measurement
//! rather than a comparison against some arbitrary host reference.
//!
//! This is the frequency sensor `cfo_tail` turned out NOT to be (1488f15): that field reports the
//! residual after carrier tracking, so a static offset never appears in it. A clock ratio does.
use ndn_radio_drivers::{FrameIo, Rtl8733buBackend};
use std::sync::Arc;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(36);
    let secs: u64 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(25);
    let cap: Option<u8> = std::env::args().nth(3).and_then(|s| s.parse().ok());
    let dev = Arc::new(Rtl8733buBackend::open()?);
    dev.bring_up_monitor(ch)?;
    if let Some(c) = cap {
        dev.set_crystal_cap(c)?;
        eprintln!("crystal cap set to {} (readback {})", c, dev.crystal_cap()?);
    }
    eprintln!("cap={} collecting {secs}s on ch{ch}", dev.crystal_cap()?);
    let _pump = dev.spawn_rx_pump(4);
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(secs);
    let mut n = 0usize;
    while std::time::Instant::now() < deadline {
        if let Ok(Ok(f)) = tokio::time::timeout(std::time::Duration::from_millis(300), dev.recv_frame()).await {
            let p = &f.payload;
            if p.len() >= 8 && p[2] == 0xC3 {
                if let Some(s) = f.stamp {
                    let seq = u32::from_le_bytes([p[4], p[5], p[6], p[7]]);
                    println!("{seq} {}", s.raw);
                    n += 1;
                }
            }
        }
    }
    eprintln!("logged {n} stamped frames");
    Ok(())
}
