//! RF-health probe: after bring-up, dump a broad RF (0x00-0x7f) + key BB state vector, then
//! inject with a per-boot source MAC (02:4d:59:44:52:<tag>) so the witness capture can score
//! each boot's radiation and correlate it with the health vector. Run across many boots and
//! diff the RF/BB lines of radiating vs dead boots to find the discriminating register.
//!   sudo ./opi_rfhealth <tag> <secs>
use ndn_radio_drivers::Rtl8733buBackend;
use std::time::{Duration, Instant};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let tag: u8 = std::env::args().nth(1).and_then(|s| u8::from_str_radix(&s, 16).ok()).unwrap_or(0x01);
    let secs: u64 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(10);
    let d = Rtl8733buBackend::open()?;
    d.bring_up_monitor(36)?;
    d.write8(0x0522, 0x00)?;

    // RF path-A 0x00..0x80
    let mut rf = String::new();
    for a in 0..0x80u32 {
        rf.push_str(&format!("{:05x} ", d.rf_read(a).unwrap_or(0xF_FFFF) & 0xF_FFFF));
    }
    println!("RF[{tag:02x}] {rf}");
    // curated BB / MAC health regs (PLL/ADDA/CCA/TXAGC/report)
    let bb = [0x1860u16, 0x1864, 0x1800, 0x180c, 0x1c60, 0x1c68, 0x1cf0, 0x1cf8,
              0x1e44, 0x1e5c, 0x2d9c, 0x1d40, 0x0f00, 0x1c38, 0x1b00, 0x1c3c];
    let mut bs = String::new();
    for a in bb {
        bs.push_str(&format!("{a:04x}={:08x} ", d.read32(a).unwrap_or(0xFFFF_FFFF)));
    }
    println!("BB[{tag:02x}] {bs}");

    // inject with src ..tag
    let mut f = vec![0x08u8, 0, 0, 0];
    f.extend_from_slice(&[0xff; 6]);
    f.extend_from_slice(&[0x02, 0x4d, 0x59, 0x44, 0x52, tag]);
    f.extend_from_slice(&[0x02, 0x4d, 0x59, 0x44, 0x52, tag]);
    f.extend_from_slice(&[0, 0]);
    f.extend_from_slice(b"MYDRV8733-INJECT");
    let deadline = Instant::now() + Duration::from_secs(secs);
    let mut s = 0u16;
    while Instant::now() < deadline {
        let _ = d.inject_raw(&f, 0x04, s);
        s = s.wrapping_add(1);
        std::thread::sleep(Duration::from_millis(15));
    }
    println!("SENT[{tag:02x}] {s}");
    Ok(())
}
