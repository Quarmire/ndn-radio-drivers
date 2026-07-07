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
    let ch = 1u8;

    // ── Witness: RTL8812AU monitor bring-up ──
    println!("bringing up 8812au witness on ch{ch} …");
    let wit = Arc::new(Rtl8812auBackend::open()?);
    wit.bring_up_monitor(ch)?;

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

    // ── DUT: RTL8733BU full bring-up (cal chain primes the TX AFE/BB path) ──
    println!("bringing up 8733b DUT …");
    let dut = Rtl8733buBackend::open()?;
    dut.power_on()?;
    dut.fw_dl_setup()?;
    dut.download_firmware()?;
    dut.mac_config()?;
    dut.bb_config()?;
    dut.rf_config()?;
    dut.init_trx()?;
    dut.rfk_init()?;
    dut.tune_channel(ch)?; // tune + calibrate on the SAME channel we inject on
    let _ = dut.phy_iq_calibrate();
    dut.set_monitor()?;
    dut.set_tx_power_idx(0x7f)?; // max reference TX power
    let txagc0 = dut.read32(0x3a00)?;
    dut.set_txagc_table(0x3f)?; // per-rate TX AGC table — 0 without this = no RF
    dut.configure_trsw(true)?; // route the external TRSW antenna switch (TX path)
    dut.enable_tx_path()?; // RF mode table -> TX + enable BB CCK TX
    println!("  TXAGC table 0x3a00: 0x{txagc0:08x} -> 0x{:08x}", dut.read32(0x3a00)?);
    println!(
        "  post-enable: 0x1800=0x{:05x} (exp 33312)  0x2a00=0x{:08x} (bit1?)  0x1884=0x{:08x}  RF0=0x{:05x}",
        dut.read32(0x1800)? & 0xfffff,
        dut.read32(0x2a00)?,
        dut.read32(0x1884)?,
        dut.rf_read(0x00)?
    );
    let rf18 = dut.rf_read(0x18)?;
    let txpause = dut.read8(0x0522)?;
    dut.write8(0x0522, 0x00)?; // REG_TXPAUSE: unpause ALL TX queues (cal leaves it set)
    println!(
        "8733b up on ch{ch} (RF18=0x{rf18:05x}, low byte={}, TXPAUSE was 0x{txpause:02x}→0); injecting at 1M CCK for 8s …",
        rf18 & 0xff
    );

    // Broadcast data frame carrying the marker payload.
    let mut frame = vec![0x08u8, 0x00, 0x00, 0x00];
    frame.extend_from_slice(&[0xff; 6]);
    frame.extend_from_slice(&[0x02, 0x11, 0x22, 0x33, 0x44, 0x55]);
    frame.extend_from_slice(&[0x02, 0x11, 0x22, 0x33, 0x44, 0x55]);
    frame.extend_from_slice(&[0x00, 0x00]);
    frame.extend_from_slice(MARKER);

    let hits0 = hits.load(Ordering::Relaxed);
    let mut sent = 0u16;
    let deadline = Instant::now() + Duration::from_secs(8);
    while Instant::now() < deadline {
        if dut.inject_raw(&frame, 0x00, sent).is_ok() {
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
