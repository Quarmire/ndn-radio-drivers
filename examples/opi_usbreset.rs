//! Does a USB-level reset re-randomize the per-boot analog TX state (giving real retry
//! convergence)? Each attempt: open (retry until re-enumerated), bring_up_tx, inject 2s,
//! usb_reset. Witness scores the whole session.
use ndn_radio_drivers::Rtl8733buBackend;
use std::time::{Duration, Instant};

fn open_retry() -> Option<Rtl8733buBackend> {
    for _ in 0..20 {
        if let Ok(d) = Rtl8733buBackend::open() { return Some(d); }
        std::thread::sleep(Duration::from_millis(300));
    }
    None
}

fn burst(d: &Rtl8733buBackend) {
    let mut f = vec![0x08u8, 0, 0, 0];
    f.extend_from_slice(&[0xff; 6]);
    f.extend_from_slice(&[0x02, 0x4d, 0x59, 0x44, 0x52, 0x56]);
    f.extend_from_slice(&[0x02, 0x4d, 0x59, 0x44, 0x52, 0x56]);
    f.extend_from_slice(&[0, 0]);
    f.extend_from_slice(b"MYDRV8733-INJECT");
    let deadline = Instant::now() + Duration::from_secs(2);
    let mut s = 0u16;
    while Instant::now() < deadline {
        let _ = d.inject_raw(&f, 0x04, s);
        s = s.wrapping_add(1);
        std::thread::sleep(Duration::from_millis(15));
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(36);
    let max: u32 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(5);
    for attempt in 1..=max {
        let Some(d) = open_retry() else { println!("open failed"); break; };
        if d.bring_up_tx(ch).is_ok() { println!("attempt {attempt}: injecting"); burst(&d); }
        let _ = d.usb_reset();
        drop(d);
        std::thread::sleep(Duration::from_millis(1500));
    }
    Ok(())
}
