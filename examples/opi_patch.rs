//! Shotgun localization: bring up, patch the BB registers that differ from the vendor
//! (from the opi_regdump diff) to the vendor's values, then inject. If radiation appears,
//! the TX gap is in this BB set; then narrow. env RF=1 also patches the RF 0x00-0x3f diffs.
//!   sudo ./opi_patch 36

use ndn_radio_drivers::Rtl8733buBackend;
use std::time::{Duration, Instant};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(36);
    let d = Rtl8733buBackend::open()?;
    d.bring_up_monitor(ch)?;
    // MAC EDCA / media-access timing to the vendor's values — if the CSMA/CA timing is
    // wrong the MAC queues but never keys a TX. Plus the TX AGC.
    let bb: [(u16, u32); 11] = [
        (0x0500, 0x002f_a226), // EDCA VO
        (0x0504, 0x005e_a328), // EDCA VI
        (0x0508, 0x005e_a42b), // EDCA BE
        (0x050c, 0x0000_a44f), // EDCA BK
        (0x0514, 0x0e0a_0e0a), // SIFS
        (0x0520, 0x8000_2f0f), // slot/timing
        (0x0524, 0x0000_cf0f),
        (0x3a00, 0xc0c0_c0c0),
        (0x3a04, 0xc0c0_c0c0),
        (0x3a08, 0xc0c0_c0c0),
        (0x4308, 0x5c54_5c50),
    ];
    for (a, v) in bb {
        let _ = d.write32(a, v);
    }
    d.write8(0x0522, 0x00)?;
    println!("patched BB; RF18=0x{:05x}; inject 15s", d.rf_read(0x18)?);
    let rate = if ch <= 14 { 0x00u8 } else { 0x04 };
    let mut frame = vec![0x08u8, 0x00, 0x00, 0x00];
    frame.extend_from_slice(&[0xff; 6]);
    frame.extend_from_slice(&[0x02, 0x4d, 0x59, 0x44, 0x52, 0x56]);
    frame.extend_from_slice(&[0x02, 0x4d, 0x59, 0x44, 0x52, 0x56]);
    frame.extend_from_slice(&[0x00, 0x00]);
    frame.extend_from_slice(b"MYDRV8733-INJECT");
    let deadline = Instant::now() + Duration::from_secs(15);
    let mut sent = 0u16;
    while Instant::now() < deadline {
        let _ = d.inject_raw(&frame, rate, sent);
        sent = sent.wrapping_add(1);
        std::thread::sleep(Duration::from_millis(15));
    }
    println!("sent {sent} frames");
    Ok(())
}
