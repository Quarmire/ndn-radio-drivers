//! Within-one-boot A/B for TSSI: after a single bring-up, Phase A injects PLAIN (src ..AA),
//! then runs tssi_setup and Phase B injects (src ..BB). One witness capture scores AA vs BB
//! by source MAC — the decisive fresh-device test of whether the TSSI TX-power loop makes
//! radiation reliable. Run on a FRESH replug (radiation only appears on a fresh device).
use ndn_radio_drivers::Rtl8733buBackend;
use std::time::{Duration, Instant};

fn burst(d: &Rtl8733buBackend, tag: u8, secs: u64) {
    let mut f = vec![0x08u8, 0, 0, 0];
    f.extend_from_slice(&[0xff; 6]);
    f.extend_from_slice(&[0x02, 0x4d, 0x59, 0x44, 0x52, tag]);
    f.extend_from_slice(&[0x02, 0x4d, 0x59, 0x44, 0x52, tag]);
    f.extend_from_slice(&[0, 0]);
    f.extend_from_slice(b"MYDRV8733-INJECT");
    let deadline = Instant::now() + Duration::from_secs(secs);
    let mut s = 0u16;
    while Instant::now() < deadline {
        let _ = d.inject_raw(&f, 0x04, s);
        s = s.wrapping_add(1);
        std::thread::sleep(Duration::from_millis(15));
    }
    println!("  burst tag=0x{tag:02x} sent {s}");
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(36);
    let secs: u64 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(12);
    let d = Rtl8733buBackend::open()?;
    d.bring_up_monitor(ch)?;
    d.write8(0x0522, 0x00)?;
    println!("Phase A PLAIN (src ..AA)");
    burst(&d, 0xAA, secs);
    d.tssi_setup(ch)?;
    d.write8(0x0522, 0x00)?;
    println!("Phase B TSSI (src ..BB); 0x4318=0x{:08x}", d.read32(0x4318)?);
    burst(&d, 0xBB, secs);
    Ok(())
}
