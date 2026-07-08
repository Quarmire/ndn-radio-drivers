//! Dump path-A RF 0x00-0x7f after enable_tx, for a diff against the vendor's radiating RF
//! state (read_rfreg procfs). PA-bias differences (0x56/0x60/0x63 etc.) are the fix candidates.
use ndn_radio_drivers::Rtl8733buBackend;
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(36);
    let d = Rtl8733buBackend::open()?;
    d.bring_up_monitor(ch)?;
    d.enable_tx(ch)?;
    for a in 0..0x80u32 { println!("{a:02x} {:05x}", d.rf_read(a).unwrap_or(0xFFFFF) & 0xFFFFF); }
    Ok(())
}
