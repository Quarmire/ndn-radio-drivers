//! Download a reserved-page beacon template after bring-up (the FW-TX-engine prime the
//! vendor does at init that my monitor bring-up skips), then inject. Uses the driver's
//! existing dl_rsvd_page (SW-beacon download: ENSWBCN, BCN_VALID poll, ep5 bulk-OUT).
use ndn_radio_drivers::Rtl8733buBackend;
use std::time::{Duration, Instant};

fn beacon() -> Vec<u8> {
    let mut f = Vec::new();
    f.extend_from_slice(&[0x80, 0x00, 0x00, 0x00]); // FC=beacon, dur
    f.extend_from_slice(&[0xff; 6]); // DA
    f.extend_from_slice(&[0x02, 0x4d, 0x59, 0x44, 0x52, 0x56]); // SA
    f.extend_from_slice(&[0x02, 0x4d, 0x59, 0x44, 0x52, 0x56]); // BSSID
    f.extend_from_slice(&[0x00, 0x00]); // seq
    f.extend_from_slice(&[0u8; 8]); // timestamp
    f.extend_from_slice(&[0x64, 0x00]); // beacon interval
    f.extend_from_slice(&[0x01, 0x00]); // capability
    f.extend_from_slice(&[0x00, 0x04, b'M', b'Y', b'A', b'P']); // SSID IE
    f.extend_from_slice(&[0x01, 0x08, 0x8c, 0x12, 0x98, 0x24, 0xb0, 0x48, 0x60, 0x6c]); // rates
    f.extend_from_slice(&[0x03, 0x01, 0x24]); // DS param ch36
    f
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let pg: u8 = std::env::var("PG").ok().and_then(|s| s.parse().ok()).unwrap_or(0);
    let d = Rtl8733buBackend::open()?;
    d.bring_up_monitor(36)?;
    match d.dl_rsvd_page(pg, &beacon()) {
        Ok(()) => println!("rsvd-page download OK (pg={pg}) — BCN_VALID raised"),
        Err(e) => println!("rsvd-page download FAILED (pg={pg}): {e}"),
    }
    d.write8(0x0522, 0x00)?;
    println!("injecting; RF18=0x{:05x}", d.rf_read(0x18)?);
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
    println!("sent {s}");
    Ok(())
}
