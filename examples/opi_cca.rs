//! Test the CSMA/CA-backoff hypothesis: after enable_tx, block CCA (force channel "clear")
//! so the MAC transmits regardless of ambient traffic. CCA=block writes 0x1c68 BIT24 (prevent
//! OFDM CCA) + 0x2a24 BIT13 (prevent CCK CCA) — the same blocks the cal uses. Compare the
//! radiation rate with/without.
use ndn_radio_drivers::Rtl8733buBackend;
use std::time::{Duration, Instant};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(36);
    let d = Rtl8733buBackend::open()?;
    d.bring_up_monitor(ch)?;
    d.enable_tx(ch)?;
    if std::env::var("BLOCKCCA").is_ok() {
        d.write32(0x1c68, d.read32(0x1c68)? | (1 << 24))?; // prevent OFDM CCA
        d.write32(0x2a24, d.read32(0x2a24)? | (1 << 13))?; // prevent CCK CCA
    }
    // CCA status snapshot (0x1c68 / a CCA counter region)
    println!("0x1c68=0x{:08x} 0x2a24=0x{:08x}", d.read32(0x1c68)?, d.read32(0x2a24)?);
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
