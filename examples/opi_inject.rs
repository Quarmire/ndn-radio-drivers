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
        // The vendor runs the full RF cal suite during init (write-seq diff); test whether
        // the cals I have enable the TX in the OPi's native single-context env.
        let _ = d.rfk_init();
        println!("dpk: {:?}", d.phy_dpk());
        println!("txgapk RF01: {:?}", d.phy_txgapk());
    }
    d.write8(0x0522, 0x00)?; // unpause TX
    println!(
        "8733b up ch{ch} RF18=0x{:05x} RF01=0x{:05x}; injecting {secs}s…",
        d.rf_read(0x18)?,
        d.rf_read(0x01)?
    );
    let rate = if ch <= 14 { 0x00u8 } else { 0x04 }; // 1M CCK / 6M OFDM
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
