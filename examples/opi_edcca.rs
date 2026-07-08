//! Smoke-test the set_edcca_ignore knob: bring up TX, toggle EDCCA-ignore, verify 0x84c
//! readback (L2H/H2L in bytes 2/3), and that TX still radiates with EDCCA ignored.
use ndn_radio_drivers::Rtl8733buBackend;
use ndn_radio_hal::RadioKnobs;
use std::time::{Duration, Instant};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(36);
    let d = Rtl8733buBackend::open()?;
    d.bring_up_monitor(ch)?;
    d.enable_tx(ch)?;
    d.set_edcca_ignore(true)?;
    println!("EDCCA ignore ON: 0x84c=0x{:08x} (bytes2/3 should be ff ff)", d.read32(0x84c)?);
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
    println!("sent {s}");
    Ok(())
}
