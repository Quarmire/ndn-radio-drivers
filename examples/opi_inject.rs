//! Minimal 8733b inject for the OPi radiation test: bring up on `ch` (arg, default 36)
//! and inject a marked broadcast frame in a loop. Capture separately on wlu1 (tcpdump,
//! radiotap-verified) to confirm radiation. src MAC 02:4d:59:44:52:56, payload
//! "MYDRV8733-INJECT".
//!   sudo ./opi_inject 36

use ndn_radio_drivers::Rtl8733buBackend;
use std::time::{Duration, Instant};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(36);
    let secs: u64 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(25);
    let d = Rtl8733buBackend::open()?;
    d.bring_up_monitor(ch)?;
    if std::env::var("CAL").is_ok() {
        // M10b efuse trim (crystal cap) → the analog baseline the cals need.
        println!("trim: {:?}", d.apply_efuse_trim());
        // M11 LOK — the TX-carrier keystone.
        let _ = d.rfk_init();
        println!("lok: {:?}", d.phy_lok(ch > 14));
    }
    d.write8(0x0522, 0x00)?; // unpause TX
    println!(
        "8733b up ch{ch} RF18=0x{:05x} RF01=0x{:05x}; injecting {secs}s…",
        d.rf_read(0x18)?,
        d.rf_read(0x01)?
    );
    let rate = if ch <= 14 { 0x00u8 } else { 0x04 }; // 1M CCK / 6M OFDM
    if std::env::var("TXDIAG").is_ok() {
        // Does the MAC transmit? Snapshot MAC status regs, inject a fast burst, re-read,
        // print deltas. A moving TX counter ⇒ frames hit the air (RF problem); nothing
        // moving ⇒ MAC accepts-and-drops (MAC-level TX enable missing).
        // A minimal set of likely TX-status/counter registers (device is fragile).
        let regs = [
            0x0200u16, 0x0230, 0x0234, 0x02d0, 0x02d4, 0x0640, 0x0660, 0x0664, 0x0668, 0x066c,
            0x01e0, 0x0210,
        ];
        let snap = |d: &Rtl8733buBackend| {
            let mut v = std::collections::BTreeMap::new();
            for a in regs {
                if let Ok(x) = d.read32(a) {
                    v.insert(a, x);
                }
            }
            v
        };
        let mut f = vec![0x08u8, 0, 0, 0];
        f.extend_from_slice(&[0xff; 6]);
        f.extend_from_slice(&[0x02, 0x4d, 0x59, 0x44, 0x52, 0x56]);
        f.extend_from_slice(&[0x02, 0x4d, 0x59, 0x44, 0x52, 0x56]);
        f.extend_from_slice(&[0, 0]);
        f.extend_from_slice(b"MYDRV8733-INJECT");
        let before = snap(&d);
        for s in 0..100u16 {
            let _ = d.inject_raw(&f, rate, s);
            std::thread::sleep(Duration::from_millis(5));
        }
        let after = snap(&d);
        println!("== MAC reg deltas after 100 injects (0=no change) ==");
        for a in regs {
            if let (Some(bv), Some(av)) = (before.get(&a), after.get(&a)) {
                println!("  0x{a:04x}: {bv:08x} -> {av:08x}{}", if bv != av { "  *" } else { "" });
            }
        }
        return Ok(());
    }
    let mut frame = vec![0x08u8, 0x00, 0x00, 0x00];
    frame.extend_from_slice(&[0xff; 6]);
    frame.extend_from_slice(&[0x02, 0x4d, 0x59, 0x44, 0x52, 0x56]); // src
    frame.extend_from_slice(&[0x02, 0x4d, 0x59, 0x44, 0x52, 0x56]);
    frame.extend_from_slice(&[0x00, 0x00]);
    frame.extend_from_slice(b"MYDRV8733-INJECT");
    let deadline = Instant::now() + Duration::from_secs(secs);
    let mut sent = 0u16;
    while Instant::now() < deadline {
        let _ = d.inject_raw(&frame, rate, sent);
        sent = sent.wrapping_add(1);
        std::thread::sleep(Duration::from_millis(15));
    }
    println!("sent {sent} frames");
    Ok(())
}
