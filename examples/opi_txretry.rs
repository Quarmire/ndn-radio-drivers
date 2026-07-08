//! Test whether the intermittency resets on a fresh open() (per-USB-session) vs in-process
//! re-init: drop + RE-OPEN the backend each attempt, inject a burst. If a 5-attempt session
//! radiates ~always, the retry lever is re-open (not re-init).
use ndn_radio_drivers::Rtl8733buBackend;
use std::time::{Duration, Instant};

fn burst(d: &Rtl8733buBackend, secs: u64) {
    let mut f = vec![0x08u8, 0, 0, 0];
    f.extend_from_slice(&[0xff; 6]);
    f.extend_from_slice(&[0x02, 0x4d, 0x59, 0x44, 0x52, 0x56]);
    f.extend_from_slice(&[0x02, 0x4d, 0x59, 0x44, 0x52, 0x56]);
    f.extend_from_slice(&[0, 0]);
    f.extend_from_slice(b"MYDRV8733-INJECT");
    let deadline = Instant::now() + Duration::from_secs(secs);
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
        println!("attempt {attempt}: open + bring_up_tx + 2s burst");
        match Rtl8733buBackend::open() {
            Ok(d) => {
                if d.bring_up_tx(ch).is_ok() { burst(&d, 2); }
                drop(d); // re-open next attempt to reset the USB session
            }
            Err(e) => println!("  open failed: {e}"),
        }
        std::thread::sleep(Duration::from_millis(500));
    }
    Ok(())
}
