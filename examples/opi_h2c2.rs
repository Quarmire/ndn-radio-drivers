//! Careful per-command H2C: bring up, send each H2C command (ext then main box) one at a
//! time, health-check (read RF18) after each, then inject. env H2CN = how many commands
//! to send (default 5) to bisect which one wedges/helps.
use ndn_radio_drivers::Rtl8733buBackend;
use std::time::{Duration, Instant};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let n: usize = std::env::var("H2CN").ok().and_then(|s| s.parse().ok()).unwrap_or(5);
    let d = Rtl8733buBackend::open()?;
    d.bring_up_monitor(36)?;
    let h2c: &[(u16, u32, u16, u32)] = &[
        (0x01f0, 0x0000_0011, 0x01d0, 0x0300_004c),
        (0x01f0, 0x0000_0011, 0x01d0, 0x0300_004c),
        (0x01f4, 0x0000_0000, 0x01d4, 0x0000_016d),
        (0x01f8, 0x0000_0000, 0x01d8, 0x0000_006d),
        (0x01fc, 0x0000_0000, 0x01dc, 0x0000_0060),
    ];
    for (i, &(er, ev, mr, mv)) in h2c.iter().enumerate().take(n) {
        let _ = d.write32(er, ev);
        let _ = d.write32(mr, mv);
        std::thread::sleep(Duration::from_millis(20));
        match d.rf_read(0x18) {
            Ok(v) => println!("h2c[{i}] main=0x{mv:08x} OK, RF18=0x{:05x}", v & 0xfffff),
            Err(e) => {
                println!("h2c[{i}] main=0x{mv:08x} WEDGED device: {e}");
                return Ok(());
            }
        }
    }
    d.write8(0x0522, 0x00)?;
    println!("sent {n} H2C; injecting");
    let mut f = vec![0x08u8, 0, 0, 0];
    f.extend_from_slice(&[0xff; 6]);
    f.extend_from_slice(&[0x02, 0x4d, 0x59, 0x44, 0x52, 0x56]);
    f.extend_from_slice(&[0x02, 0x4d, 0x59, 0x44, 0x52, 0x56]);
    f.extend_from_slice(&[0, 0]);
    f.extend_from_slice(b"MYDRV8733-INJECT");
    let deadline = Instant::now() + Duration::from_secs(15);
    let mut s = 0u16;
    while Instant::now() < deadline {
        let _ = d.inject_raw(&f, 0x04, s);
        s = s.wrapping_add(1);
        std::thread::sleep(Duration::from_millis(15));
    }
    println!("done {s}");
    Ok(())
}
