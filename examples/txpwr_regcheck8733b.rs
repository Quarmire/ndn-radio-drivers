//! Is the 8733b TX-power knob writing a register that is actually in the transmit path?
//!
//! An on-air sweep of `set_tx_power_idx` moved the witness RSSI by 0.33 dB across 55 index steps
//! (2026-08-24) — but the witness was pinned near -14 dBm, so that result cannot separate "the knob
//! does nothing" from "the receiver was saturated". This asks the question with no RF involved.
//!
//! There are THREE overlapping power controls on this part and only the first is what the knob
//! writes:
//!   0x4308        `config_phydm_write_txagc_ref` — the per-path TXAGC *reference*
//!   0x3a00..0x3a13 the per-rate TXAGC *table* (`set_txagc_table`, set to 0x2d during bring-up)
//!   0x1e40..0x1e60 the datapath per-rate block `enable_tx` writes after calibration
//!
//! If `set_tx_power_idx` moves 0x4308 while 0x3a00 and 0x1e44 stay put, the knob is writing a
//! reference the active path never consults, and the fix is to drive the per-rate table instead.
//! Prints readbacks around each set so a clobber (something rewriting it) is also visible.
//!
//!   sudo ./txpwr_regcheck8733b [channel]

use ndn_radio_drivers::Rtl8733buBackend;

fn dump(dev: &Rtl8733buBackend, tag: &str) {
    let r4308 = dev.read32(0x4308).unwrap_or(0);
    let r3a00 = dev.read32(0x3a00).unwrap_or(0);
    let r3a0c = dev.read32(0x3a0c).unwrap_or(0); // covers rate 0x0c-0x0f = HT MCS0-3
    let r1e44 = dev.read32(0x1e44).unwrap_or(0);
    let r18a0 = dev.read32(0x18a0).unwrap_or(0); // OFDM swing, written by the power tracker
    println!(
        "  {tag:<22} 0x4308={r4308:08x} [ofdm={:02x} cck={:02x}]  0x3a00={r3a00:08x}  0x3a0c={r3a0c:08x}  0x1e44={r1e44:08x}  0x18a0={r18a0:08x}",
        r4308 & 0x7f,
        (r4308 >> 8) & 0x7f
    );
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(36);
    let dev = Rtl8733buBackend::open()?;
    dev.bring_up_monitor(ch)?;
    println!("after bring_up_monitor(ch{ch}) — set_txagc_table(0x2d) has run:");
    dump(&dev, "baseline");

    for idx in [0x3fu8, 0x20, 0x08] {
        dev.set_tx_power_idx(idx)?;
        dump(&dev, &format!("set_tx_power_idx {idx:#04x}"));
        // Re-read after a tracker tick to catch anything rewriting the reference behind us.
        std::thread::sleep(std::time::Duration::from_millis(900));
        dump(&dev, "  (+900ms)");
    }

    println!("\nfor contrast, drive the per-rate TABLE directly:");
    for idx in [0x3fu8, 0x20, 0x08] {
        dev.set_txagc_table(idx)?;
        dump(&dev, &format!("set_txagc_table {idx:#04x}"));
    }
    Ok(())
}
