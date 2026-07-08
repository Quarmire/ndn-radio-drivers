//! Inherit the vendor's radiating state: open WITHOUT bring_up (open() only claims USB, no
//! reset), so the chip keeps whatever the just-unloaded vendor driver configured. Dump key
//! state + full RF/BB to /tmp/vendor_state.txt, then inject. If this radiates, my inject path
//! is correct and only the SETUP differs — diff the dump against my bring_up state.
use ndn_radio_drivers::Rtl8733buBackend;
use std::io::Write;
use std::time::{Duration, Instant};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let secs: u64 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(12);
    let d = Rtl8733buBackend::open()?;
    println!("INHERITED: RF18=0x{:05x} RF01=0x{:05x} 0x70(GNT)=0x{:08x} 0x4318(TSSI)=0x{:08x} 0x0522(txpause)=0x{:02x}",
        d.rf_read(0x18)?, d.rf_read(0x01)?, d.read32(0x70)?, d.read32(0x4318)?, d.read32(0x0520)? >> 16 & 0xff);
    // dump full RF + BB for diffing
    let mut f = std::fs::File::create("/tmp/vendor_state.txt")?;
    for a in 0..0x80u32 { writeln!(f, "R {a:02x} {:05x}", d.rf_read(a).unwrap_or(0xFFFFF) & 0xFFFFF)?; }
    for a in (0x1800..0x2000u16).step_by(4) { writeln!(f, "B {a:04x} {:08x}", d.read32(a).unwrap_or(0xFFFFFFFF))?; }
    for a in (0x0000..0x0800u16).step_by(4) { writeln!(f, "M {a:04x} {:08x}", d.read32(a).unwrap_or(0xFFFFFFFF))?; }
    // inject WITHOUT any bring_up
    let mut fr = vec![0x08u8, 0, 0, 0];
    fr.extend_from_slice(&[0xff; 6]);
    fr.extend_from_slice(&[0x02, 0x4d, 0x59, 0x44, 0x52, 0x56]);
    fr.extend_from_slice(&[0x02, 0x4d, 0x59, 0x44, 0x52, 0x56]);
    fr.extend_from_slice(&[0, 0]);
    fr.extend_from_slice(b"MYDRV8733-INJECT");
    let deadline = Instant::now() + Duration::from_secs(secs);
    let mut s = 0u16;
    while Instant::now() < deadline {
        let _ = d.inject_raw(&fr, 0x04, s);
        s = s.wrapping_add(1);
        std::thread::sleep(Duration::from_millis(15));
    }
    println!("sent {s} (no bring_up — inherited vendor state)");
    Ok(())
}
