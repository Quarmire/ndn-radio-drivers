//! Quantify the thermal fade: read the RF thermal meter (read_thermal) before/after a TX
//! burst. Ref = efuse 0xBA. If radiating runs have lower end-thermal than faded runs, the
//! fade is thermal -> a power-tracking loop fixes it.
use ndn_radio_drivers::Rtl8733buBackend;
use std::time::{Duration, Instant};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(36);
    let d = Rtl8733buBackend::open()?;
    d.bring_up_monitor(ch)?;
    let (_c, thref) = d.apply_efuse_trim().unwrap_or((0, 0x20));
    d.enable_tx(ch)?;
    let t0 = d.read_thermal().unwrap_or(0);
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
    let t1 = d.read_thermal().unwrap_or(0);
    println!("THERM ref={thref} start={t0} end={t1}");
    Ok(())
}
