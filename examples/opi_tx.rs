//! Productized on-air TX: bring_up_monitor + enable_tx (full cal + datapath TXAGC + GNT),
//! then inject. Validates the Rtl8733buBackend::enable_tx path radiates.
use ndn_radio_drivers::Rtl8733buBackend;
use std::time::{Duration, Instant};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(36);
    let secs: u64 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(10);
    let d = Rtl8733buBackend::open()?;
    d.bring_up_monitor(ch)?;
    d.enable_tx(ch)?;
    println!("enable_tx done: RF01=0x{:05x} 0x70=0x{:08x} 0x1e44=0x{:08x}",
        d.rf_read(0x01)?, d.read32(0x70)?, d.read32(0x1e44)?);
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
    println!("sent {s}");
    Ok(())
}
