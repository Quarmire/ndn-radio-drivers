//! TSSI TX-power loop test: bring up, run tssi_setup (the ported halrf_do_tssi_8733b enable
//! path), verify TSSI enabled (0x4318[30:28]=7), then inject. Capture on wlu1; score by src
//! MAC 02:4d:59:44:52:56. Hypothesis: TSSI drives the per-rate TX power that my static
//! bring-up leaves undefined -> more reliable radiation.
use ndn_radio_drivers::Rtl8733buBackend;
use std::time::{Duration, Instant};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(36);
    let secs: u64 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(12);
    let d = Rtl8733buBackend::open()?;
    d.bring_up_monitor(ch)?;
    d.tssi_setup(ch)?;
    d.write8(0x0522, 0x00)?;
    println!("TSSI: 0x4318=0x{:08x} (30:28=7), DE 0x4334=0x{:08x} 0x43b0=0x{:08x}, RF01=0x{:05x}",
        d.read32(0x4318)?, d.read32(0x4334)?, d.read32(0x43b0)?, d.rf_read(0x01)?);
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
