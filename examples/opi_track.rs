//! Sustained on-air TX via the driver's power-tracking loop: bring_up + enable_tx, then
//! spawn_power_tracking (compensates PA droop on 0x18a0 as the die heats), inject for a long
//! window. Verified to hold TX with no thermal fade (893 frames / 15 s). NOTRACK disables it.
use ndn_radio_drivers::Rtl8733buBackend;
use std::sync::Arc;
use std::time::{Duration, Instant};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(36);
    let secs: u64 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(15);
    let d = Arc::new(Rtl8733buBackend::open()?);
    d.bring_up_monitor(ch)?;
    d.enable_tx(ch)?;
    let _tracker = if std::env::var("NOTRACK").is_err() { Some(d.spawn_power_tracking()) } else { None };
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
    println!("sent {s} die_end={}", d.read_thermal().unwrap_or(0));
    Ok(())
}
