//! Dump MAC/BB/RF registers after bring_up_monitor(ch), for an apples-to-apples diff
//! against the vendor's procfs dumps on the same silicon. Lines: "M/B addr val" (hex).
//!   sudo ./opi_regdump 36 > /tmp/mine.txt

use ndn_radio_drivers::Rtl8733buBackend;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(36);
    let d = Rtl8733buBackend::open()?;
    d.bring_up_monitor(ch)?;
    for a in (0..0x800u16).step_by(4) {
        if let Ok(v) = d.read32(a) {
            println!("M {a:04x} {v:08x}");
        }
    }
    for a in (0x800..0x2000u16).step_by(4) {
        if let Ok(v) = d.read32(a) {
            println!("B {a:04x} {v:08x}");
        }
    }
    for a in 0..0x80u32 {
        if let Ok(v) = d.rf_read(a) {
            println!("R {a:02x} {:05x}", v & 0xf_ffff);
        }
    }
    Ok(())
}
