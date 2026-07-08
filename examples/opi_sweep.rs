//! Boot-reliability via power sweep: the per-boot power sweet-spot on 0x18a0[6:0] varies, so
//! sweep it across the range during the inject — every boot's sweet spot is hit at some point,
//! so (nearly) every boot radiates for part of the window. Measures how many boots radiate AT
//! ALL (vs ~50% at a fixed power).
use ndn_radio_drivers::Rtl8733buBackend;
use std::time::{Duration, Instant};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(36);
    let secs: u64 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(10);
    let d = Rtl8733buBackend::open()?;
    d.bring_up_monitor(ch)?;
    d.enable_tx(ch)?;
    let sweep = [0x00u32, 0x08, 0x10, 0x18, 0x20, 0x28, 0x30, 0x38];
    let mut f = vec![0x08u8, 0, 0, 0];
    f.extend_from_slice(&[0xff; 6]);
    f.extend_from_slice(&[0x02, 0x4d, 0x59, 0x44, 0x52, 0x56]);
    f.extend_from_slice(&[0x02, 0x4d, 0x59, 0x44, 0x52, 0x56]);
    f.extend_from_slice(&[0, 0]);
    f.extend_from_slice(b"MYDRV8733-INJECT");
    let deadline = Instant::now() + Duration::from_secs(secs);
    let mut s = 0u16; let mut idx = 0;
    while Instant::now() < deadline {
        if s % 12 == 0 { // change power every ~180ms
            let v = d.read32(0x18a0)?;
            let _ = d.write32(0x18a0, (v & !0x7f) | sweep[idx % sweep.len()]);
            idx += 1;
        }
        let _ = d.inject_raw(&f, 0x04, s);
        s = s.wrapping_add(1);
        std::thread::sleep(Duration::from_millis(15));
    }
    println!("sent {s}");
    Ok(())
}
