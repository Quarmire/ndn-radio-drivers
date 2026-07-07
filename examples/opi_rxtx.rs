//! Inject WHILE continuously reading bulk-IN (RX/C2H) — the vendor always has RX active
//! during TX; my inject-only examples never drain the FW's C2H/RX pipe. If the FW gates TX
//! on the host consuming bulk-IN, this enables it.
use ndn_radio_drivers::Rtl8733buBackend;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let d = Arc::new(Rtl8733buBackend::open()?);
    d.bring_up_monitor(36)?;
    d.write8(0x0522, 0x00)?;
    let stop = Arc::new(AtomicBool::new(false));
    // background: drain bulk-IN (RX frames + C2H)
    let d2 = d.clone();
    let stop2 = stop.clone();
    let rx = std::thread::spawn(move || {
        let mut n = 0u64;
        while !stop2.load(Ordering::Relaxed) {
            if let Ok(frames) = d2.capture(50) {
                n += frames.len() as u64;
            }
        }
        n
    });
    println!("injecting while RXing; RF18=0x{:05x}", d.rf_read(0x18)?);
    let mut f = vec![0x08u8, 0, 0, 0];
    f.extend_from_slice(&[0xff; 6]);
    f.extend_from_slice(&[0x02, 0x4d, 0x59, 0x44, 0x52, 0x56]);
    f.extend_from_slice(&[0x02, 0x4d, 0x59, 0x44, 0x52, 0x56]);
    f.extend_from_slice(&[0, 0]);
    f.extend_from_slice(b"MYDRV8733-INJECT");
    let deadline = Instant::now() + Duration::from_secs(12);
    let mut s = 0u16;
    while Instant::now() < deadline {
        let _ = d.inject_raw(&f, 0x04, s);
        s = s.wrapping_add(1);
        std::thread::sleep(Duration::from_millis(15));
    }
    stop.store(true, Ordering::Relaxed);
    let rxn = rx.join().unwrap_or(0);
    println!("sent {s}, rx-consumed {rxn} frames");
    Ok(())
}
