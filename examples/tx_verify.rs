//! Cross-radio TX proof: inject a marker frame from the RTL8733BU (DUT) and capture
//! it on the RTL8812AU witness. Both dongles on the same host + channel.
//!
//! macOS note: two libusb contexts open at once stall each other unless both are
//! actively serviced, so the witness RX runs on its own thread (keeping its context
//! live) while the main thread injects from the 8733b.
//!   cargo run --example tx_verify

use ndn_radio_drivers::{Rtl8733buBackend, Rtl8812auBackend};
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

const MARKER: &[u8] = b"NDN8733B-TEST-INJECT";

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Channel from arg (default 1). Both dongles are dual-band; a marker hit confirms
    // the witness is genuinely on this band (don't trust `iw`'s reported channel).
    let ch: u8 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(1);

    // ── Witness: RTL8812AU monitor bring-up ──
    println!("bringing up 8812au witness on ch{ch} …");
    let wit = Arc::new(Rtl8812auBackend::open()?);
    wit.bring_up_monitor(ch)?;
    // Verify the witness's ACTUAL tuned channel (RF18 low byte) — don't trust the set.
    let wrf18 = wit.rf_read(ndn_radio_drivers::RfPath::A, 0x18).unwrap_or(0);
    println!("witness RF18=0x{wrf18:05x} → tuned channel {} (asked {ch})", wrf18 & 0xff);

    // Continuous witness RX on its own thread: keeps the 8812au's libusb context
    // serviced (so the 8733b context isn't starved) and counts marker hits.
    let hits = Arc::new(AtomicU32::new(0));
    let reads = Arc::new(AtomicU32::new(0));
    let stop = Arc::new(AtomicBool::new(false));
    let pump = {
        let (wit, hits, reads, stop) = (wit.clone(), hits.clone(), reads.clone(), stop.clone());
        std::thread::spawn(move || {
            let mut buf = vec![0u8; 16384];
            while !stop.load(Ordering::Relaxed) {
                if let Ok(n) = wit.rx_raw(&mut buf) {
                    if n > 0 {
                        reads.fetch_add(1, Ordering::Relaxed);
                        // Detect by source MAC (survives CRC errors near frame start)
                        // or the payload marker.
                        let src = [0x02u8, 0x11, 0x22, 0x33, 0x44, 0x55];
                        if buf[..n].windows(6).any(|w| w == src)
                            || buf[..n].windows(MARKER.len()).any(|w| w == MARKER)
                        {
                            hits.fetch_add(1, Ordering::Relaxed);
                        }
                    }
                }
            }
        })
    };
    std::thread::sleep(Duration::from_millis(1500));
    println!("witness ambient sanity: {} reads in 1.5s", reads.load(Ordering::Relaxed));

    // ── DUT: RTL8733BU bring-up on ch{ch}. bring_up_monitor applies the whole path:
    //    40-byte-descriptor TX, full BB channel/BW config, enable_tx_path (RF mode
    //    table + CCK TX), per-rate AGC table, TRSW routing. ──
    println!("bringing up 8733b DUT on ch{ch} …");
    let dut = Rtl8733buBackend::open()?;
    dut.bring_up_monitor(ch)?;
    dut.set_tx_power_idx(0x7f)?; // max reference TX power
    dut.write8(0x0522, 0x00)?; // REG_TXPAUSE: unpause all TX queues
    // Match the vendor's exact per-rate TXAGC — my uniform table was too low, leaving
    // RF01 (the RF TX AGC) at 0 = zero TX gain = no RF output.
    dut.write32(0x3a00, 0xc0c0_c0c0)?; // CCK 1/2/5.5/11M
    dut.write32(0x3a04, 0x0808_0c0c)?; // OFDM 6/9/12/18M
    dut.write32(0x4308, 0x5c54_5c50)?; // TXAGC ref
    // Per-TX BB writes the vendor makes around each bulk-OUT (from the usbmon capture).
    dut.write32(0x1c3c, 0x0105_1f43)?;
    dut.write32(0x08a0, 0x9764_309f)?;
    dut.write32(0x2a44, 0x8020_0311)?;
    // THE fix: RF TX AGC (RF 0x01) — vendor holds 0x1a, mine idled at 0 = no TX gain.
    dut.set_rf_txagc(0x1a)?;
    let rf18 = dut.rf_read(0x18)?;
    println!(
        "8733b up on ch{ch} (RF18=0x{rf18:05x}, RF01/TXAGC=0x{:05x}); injecting for 8s …",
        dut.rf_read(0x01)?
    );

    // Broadcast data frame carrying the marker payload.
    let mut frame = vec![0x08u8, 0x00, 0x00, 0x00];
    frame.extend_from_slice(&[0xff; 6]);
    frame.extend_from_slice(&[0x02, 0x11, 0x22, 0x33, 0x44, 0x55]);
    frame.extend_from_slice(&[0x02, 0x11, 0x22, 0x33, 0x44, 0x55]);
    frame.extend_from_slice(&[0x00, 0x00]);
    frame.extend_from_slice(MARKER);

    // 2.4 GHz → 1M CCK (0x00); 5 GHz → 6M OFDM (0x04), since CCK is 2.4-only.
    let rate = if ch <= 14 { 0x00u8 } else { 0x04 };
    let hits0 = hits.load(Ordering::Relaxed);
    let mut sent = 0u16;
    let deadline = Instant::now() + Duration::from_secs(8);
    while Instant::now() < deadline {
        if dut.inject_raw(&frame, rate, sent).is_ok() {
            sent = sent.wrapping_add(1);
        }
        std::thread::sleep(Duration::from_millis(3));
    }
    stop.store(true, Ordering::Relaxed);
    let _ = pump.join();

    let marker_hits = hits.load(Ordering::Relaxed) - hits0;
    println!(
        "\nRESULT: sent {sent} frames, witness total reads {}, marker hits {marker_hits}",
        reads.load(Ordering::Relaxed)
    );
    if marker_hits > 0 {
        println!("🎉 8733b TX CONFIRMED ON AIR");
    } else {
        println!("no marker hits — TX not radiating, or ch/rate mismatch");
    }
    Ok(())
}
