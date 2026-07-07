//! Cross-radio TX proof: inject a marker frame from the RTL8733BU (DUT) and capture
//! it on the RTL8812AU witness. Both dongles on the same host + channel — a marker hit
//! in the witness's raw RX proves the 8733b is radiating on air.
//!   cargo run --example tx_verify

use ndn_radio_drivers::{Rtl8733buBackend, Rtl8812auBackend};
use std::time::{Duration, Instant};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch = 1u8;
    let marker = b"NDN8733B-TEST-INJECT";

    // ── Witness: RTL8812AU monitor bring-up ──
    println!("bringing up 8812au witness on ch{ch} …");
    let wit = Rtl8812auBackend::open()?;
    wit.power_on()?;
    wit.download_firmware()?;
    wit.mac_config()?;
    wit.bb_config()?;
    wit.rf_config()?;
    wit.mac_init_queues()?;
    wit.mac_enable_dma()?;
    wit.set_channel(ch)?;
    match wit.iq_calibrate() {
        Ok(_) => println!("  witness IQK ok"),
        Err(e) => println!("  witness IQK: {e}"),
    }
    match wit.lc_calibrate() {
        Ok(()) => println!("  witness LCK ok"),
        Err(e) => println!("  witness LCK: {e}"),
    }
    wit.start_rx_dma()?;

    // Sanity: confirm the witness hears ambient traffic before trusting a null result.
    let mut buf = vec![0u8; 16384];
    let mut ambient = 0u32;
    let t = Instant::now() + Duration::from_millis(1500);
    while Instant::now() < t {
        if let Ok(n) = wit.rx_raw(&mut buf) {
            if n > 0 {
                ambient += 1;
            }
        }
    }
    println!("witness ambient sanity: {ambient} non-empty reads/1.5s (nonzero = RX works)");

    // ── DUT: RTL8733BU full bring-up (no calibration needed to radiate) ──
    println!("bringing up 8733b DUT …");
    let dut = Rtl8733buBackend::open()?;
    dut.power_on()?;
    dut.fw_dl_setup()?;
    dut.download_firmware()?;
    dut.mac_config()?;
    dut.bb_config()?;
    dut.rf_config()?;
    dut.init_trx()?;
    dut.tune_channel(ch)?;
    dut.set_monitor()?;
    dut.set_tx_power_idx(0x50)?;
    println!("8733b up on ch{ch}; injecting a marker frame at 1M CCK for 8s …");

    // Broadcast data frame carrying the marker payload.
    let mut frame = vec![0x08u8, 0x00, 0x00, 0x00];
    frame.extend_from_slice(&[0xff; 6]); // addr1 = broadcast
    frame.extend_from_slice(&[0x02, 0x11, 0x22, 0x33, 0x44, 0x55]); // addr2
    frame.extend_from_slice(&[0x02, 0x11, 0x22, 0x33, 0x44, 0x55]); // addr3
    frame.extend_from_slice(&[0x00, 0x00]); // seq
    frame.extend_from_slice(marker);

    // Self-contained TX evidence: the MAC TX packet counter (0x2de0) and the TSSI
    // report (0x42f0) — if the counter climbs, the MAC/PHY is transmitting.
    let tx_cnt0 = dut.read32(0x2de0)? & 0xFFFF;
    let tssi0 = dut.read32(0x42f0)?;

    let (mut sent, mut hits) = (0u16, 0u32);
    let deadline = Instant::now() + Duration::from_secs(8);
    while Instant::now() < deadline {
        for _ in 0..5 {
            dut.inject_raw(&frame, 0x00, sent)?;
            sent = sent.wrapping_add(1);
        }
        if let Ok(n) = wit.rx_raw(&mut buf) {
            if n > 0 && buf[..n].windows(marker.len()).any(|w| w == marker) {
                hits += 1;
                if hits <= 3 {
                    println!("  ✅ witness caught marker #{hits}");
                }
            }
        }
    }
    let tx_cnt1 = dut.read32(0x2de0)? & 0xFFFF;
    let tssi1 = dut.read32(0x42f0)?;
    println!(
        "\n8733b TX counter 0x2de0: {tx_cnt0} → {tx_cnt1} (Δ{})   TSSI 0x42f0: 0x{tssi0:08x} → 0x{tssi1:08x}",
        tx_cnt1.wrapping_sub(tx_cnt0)
    );
    println!("RESULT: sent {sent} frames, witness caught {hits} marker frames");
    if hits > 0 {
        println!("🎉 8733b TX CONFIRMED ON AIR");
    } else {
        println!("no marker hits — TX may not be radiating, or witness ch/rate mismatch");
    }
    Ok(())
}
