//! Thermal droop test: after enable_tx, BUMP the per-rate TXAGC (0x1e40-0x1e60 power-index
//! bytes) by env BUMP to counter PA droop as the die heats. If higher bump radiates reliably
//! (even at high die temp), thermal power droop is the intermittency and compensation fixes it.
use ndn_radio_drivers::Rtl8733buBackend;
use std::time::{Duration, Instant};

fn bump(d: &Rtl8733buBackend, addr: u16, n: u32) -> Result<(), Box<dyn std::error::Error>> {
    let v = d.read32(addr)?;
    let mut nv = 0u32;
    for i in 0..4 {
        let b = (v >> (i * 8)) & 0xff;
        let nb = if b < 0x40 { (b + n).min(0x3f) } else { b }; // bump power indices, skip flags
        nv |= nb << (i * 8);
    }
    d.write32(addr, nv)?;
    Ok(())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(36);
    let n: u32 = std::env::var("BUMP").ok().and_then(|s| s.parse().ok()).unwrap_or(0);
    let d = Rtl8733buBackend::open()?;
    d.bring_up_monitor(ch)?;
    d.enable_tx(ch)?;
    if n > 0 { for a in [0x1e44u16,0x1e48,0x1e50,0x1e54,0x1e58,0x1e5c,0x1e60] { bump(&d, a, n)?; } }
    let t = d.read_thermal().unwrap_or(0);
    println!("BUMP={n} die={t} 0x1e44=0x{:08x}", d.read32(0x1e44)?);
    let mut f = vec![0x08u8, 0, 0, 0];
    f.extend_from_slice(&[0xff; 6]);
    f.extend_from_slice(&[0x02, 0x4d, 0x59, 0x44, 0x52, 0x56]);
    f.extend_from_slice(&[0x02, 0x4d, 0x59, 0x44, 0x52, 0x56]);
    f.extend_from_slice(&[0, 0]);
    f.extend_from_slice(b"MYDRV8733-INJECT");
    let deadline = Instant::now() + Duration::from_secs(6);
    let mut s = 0u16;
    while Instant::now() < deadline {
        let _ = d.inject_raw(&f, 0x04, s);
        s = s.wrapping_add(1);
        std::thread::sleep(Duration::from_millis(15));
    }
    println!("sent {s}");
    Ok(())
}
