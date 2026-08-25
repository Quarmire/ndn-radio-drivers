//! Dump the RTL8733BU's efuse PG TX-power calibration, and compare it against what the driver
//! actually writes.
//!
//! The driver has always written FLAT gain indices — `set_txagc_table(0x2d)` during bring-up, and a
//! `0x1c..0x38` ramp in the datapath block — with no reference to the chip's own trim. On the
//! 8812au that exact omission meant "full power" sat ~20 dB above the calibrated point and drove
//! the PA into compression rather than producing more output, which is also a candidate explanation
//! for the 2026-08-24 finding that every gain register measured flat on the SDR: if all the indices
//! being swept are above the compression knee, output cannot move.
//!
//! Prints raw PG bytes alongside the parse so a misparse is visible rather than plausible.
//! Units: gain index at txgi_pdbm = 4, i.e. 0.25 dB per step.
//!
//!   sudo ./efuse_txpwr8733b [channel]

use ndn_radio_drivers::Rtl8733buBackend;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(36);
    let dev = Rtl8733buBackend::open()?;
    dev.power_on()?; // efuse reads need the MAC powered; no PHY bring-up required

    let raw = Rtl8733buBackend::decode_efuse_pub(&dev.read_efuse(512)?);
    print!("PG bytes 0x10..0x40:");
    for (i, b) in raw.iter().enumerate().take(0x40).skip(0x10) {
        if i % 16 == 0 {
            print!("\n  0x{i:02x}: ");
        }
        print!("{b:02x} ");
    }
    println!();

    let info = dev.read_tx_power_info()?;
    println!("\n2.4 GHz CCK bases   : {:?}", info.cck_base_2g);
    println!("2.4 GHz BW40 bases  : {:?}", info.bw40_base_2g);
    println!("5 GHz  BW40 bases   : {:?}", info.bw40_base_5g);
    println!(
        "stream-0 diffs (already ×2): 2G bw20={:?} ofdm={:?} | 5G bw20={:?} ofdm={:?}",
        info.bw20_diff_2g, info.ofdm_diff_2g, info.bw20_diff_5g, info.ofdm_diff_5g
    );

    match dev.calibrated_ofdm_index(ch)? {
        Some(idx) => {
            println!("\nch{ch}: CALIBRATED OFDM index = {idx} (0x{idx:02x})  = {:.2} dB of gain index", f64::from(idx) / 4.0);
            println!("  driver currently writes: table 0x2d ({}), datapath ramp 0x1c-0x38 ({}-{})", 0x2d, 0x1c, 0x38);
            let over = i32::from(0x2du8) - i32::from(idx);
            println!("  flat table value is {} index steps = {:.1} dB {} the calibrated point",
                over.abs(), f64::from(over.abs()) / 4.0, if over > 0 { "ABOVE" } else { "below" });
        }
        None => println!("\nch{ch}: efuse cell unprogrammed — no calibrated index for this group"),
    }
    Ok(())
}
