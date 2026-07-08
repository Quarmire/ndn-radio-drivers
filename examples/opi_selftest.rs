//! Does the chip self-receive its own TX (leakage)? bring_up + enable_tx, then inject while
//! capturing RX in a bg thread, counting RX frames with MY src MAC. If self-RX>0 correlates
//! with witness radiation, it's an on-chip TX-live detector for a retry wrapper.
use ndn_radio_drivers::Rtl8733buBackend;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::time::{Duration, Instant};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(36);
    let d = Arc::new(Rtl8733buBackend::open()?);
    d.bring_up_monitor(ch)?;
    d.enable_tx(ch)?;
    let stop = Arc::new(AtomicBool::new(false));
    let selfrx = Arc::new(AtomicU64::new(0));
    let d2 = d.clone(); let s2 = stop.clone(); let c2 = selfrx.clone();
    let rx = std::thread::spawn(move || {
        while !s2.load(Ordering::Relaxed) {
            if let Ok(frames) = d2.capture(50) {
                for f in frames {
                    // my src MAC 02 4d 59 44 52 56 at 802.11 offset 10 (after FC+dur+DA)
                    if f.windows(6).any(|w| w == [0x02,0x4d,0x59,0x44,0x52,0x56]) {
                        c2.fetch_add(1, Ordering::Relaxed);
                    }
                }
            }
        }
    });
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
    stop.store(true, Ordering::Relaxed);
    let _ = rx.join();
    println!("SELFRX={} sent={s}", selfrx.load(Ordering::Relaxed));
    Ok(())
}
