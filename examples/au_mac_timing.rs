//! Read-only dump of the RTL8812AU's MAC timing state — the registers the DCF budget assumes.
//!
//! Written to answer one MEASURED anomaly: `ContentionPosture::Owned` vs `Shared` moved the
//! per-frame period by **+0.47%** (legacy 6M) and **+2.50%** (MCS7) where the budget predicted
//! 17.7% and 45.4%. The EDCA write verifies on read-back, so the registers hold what we asked —
//! yet the medium does not change. One candidate is that the model's *slot* is fiction:
//! `REG_SLOT 0x051b` is **never written** by this driver (absent from `mac_reg.bin` and from
//! `src/rtl8812au.rs`), while `realtek_contention::set_contention` reads it, substitutes 9 when it
//! is out of `9..=20`, and then uses that value BOTH to report `slot_us` AND to compute the AIFS
//! it programs (`encode_ac`: `aifs_us = SIFS + aifsn * slot_us`).
//!
//! Reads only. No writes, no injection.
use ndn_radio_drivers::Rtl8812auBackend;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(36);
    let d = Rtl8812auBackend::open()?;
    d.bring_up_monitor(ch)?;
    println!("RTL8812AU MAC timing, ch{ch} (read-only)\n");

    let slot = d.read8(0x051b)?;
    println!("  REG_SLOT      0x051b = {slot:#04x} ({slot} us)");
    if !(9..=20).contains(&slot) {
        println!("      ★ OUT OF THE LEGAL 9..=20 RANGE — the DCF budget's slot is fiction, and");
        println!("        `encode_ac` substituted 9 when computing the AIFS it programmed.");
    }
    println!("  REG_SIFS      0x0514 = {:#010x}", d.read32(0x0514)?);
    println!("  REG_PIFS      0x0512 = {:#04x}", d.read8(0x0512)?);
    println!("  REG_TXPAUSE   0x0522 = {:#04x}", d.read8(0x0522)?);
    println!("  REG_TX_PTCL   0x0520 = {:#010x}", d.read32(0x0520)?);
    println!("  REG_RD_CTRL   0x0524 = {:#010x}", d.read32(0x0524)?);
    println!("  REG_RETRY_LMT 0x042a = {:#06x}", d.read16(0x042a)?);
    println!("  REG_CR        0x0100 = {:#06x}", d.read16(0x0100)?);
    println!();
    for (name, reg) in [
        ("VO", 0x0500u16),
        ("VI", 0x0504),
        ("BE", 0x0508),
        ("BK", 0x050c),
    ] {
        let v = d.read32(reg)?;
        // TXOP[31:16] | ECWmax[15:12] | ECWmin[11:8] | AIFS[7:0], AIFS in MICROSECONDS.
        let (txop, cwmax, cwmin, aifs) = (v >> 16, (v >> 12) & 0xf, (v >> 8) & 0xf, v & 0xff);
        let implied_aifsn = if slot > 0 {
            (aifs as i32 - 16) / slot as i32
        } else {
            -1
        };
        println!(
            "  EDCA {name} {reg:#06x} = {v:#010x}  TXOP {txop:#06x} ECWmax {cwmax} ECWmin {cwmin} \
             AIFS {aifs} us => AIFSN {implied_aifsn} at slot {slot}"
        );
    }
    println!("\n  (AIFS is in microseconds on Realtek: AIFS = SIFS + AIFSN * slot.)");
    Ok(())
}
