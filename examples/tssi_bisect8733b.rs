//! Which phase of `tssi_setup` kills the USB endpoint?
//!
//! Controlled A/B on a stable bus: the same sweep binary completes 8/8 arms without TSSI and 0/8
//! with it, failing on the first bulk transfer AFTER `tssi_setup` returns success. Setting the
//! enable bit `0x4318[30:28]=7` on its own is harmless (measured separately, 0.51 dB, no failure),
//! so one of the configuration phases is responsible.
//!
//! Runs `tssi_setup_upto(ch, PHASE)` then `enable_tx` then a short burst, and reports whether the
//! device survived. One phase per process — device state does not survive a failure, so phases must
//! not share a process.
//!
//!   sudo ./tssi_bisect8733b <channel> <phase 0..10>
//! phase 0 = no TSSI at all (the known-good control).

use std::sync::Arc;

use bytes::Bytes;
use ndn_radio_drivers::{BROADCAST, FrameIo, InjectFrame, Rtl8733buBackend, TxIntent};

#[tokio::main(flavor = "multi_thread", worker_threads = 2)]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(36);
    let phase: u8 = std::env::args()
        .nth(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(0);

    let dev = Arc::new(Rtl8733buBackend::open()?);
    dev.bring_up_monitor(ch)?;
    if phase > 0 {
        match dev.tssi_setup_upto(ch, phase) {
            Ok(()) => println!("phase {phase}: tssi_setup_upto OK"),
            Err(e) => {
                println!("RESULT phase={phase} verdict=SETUP_FAILED err={e}");
                return Ok(());
            }
        }
    }
    // `enable_tx` is where the damage first shows if the setup left the device wedged.
    if let Err(e) = dev.enable_tx(ch) {
        println!("RESULT phase={phase} verdict=ENABLE_TX_FAILED err={e}");
        return Ok(());
    }
    let f = InjectFrame {
        payload: Bytes::from(vec![0xC3u8; 300]),
        tx: TxIntent::CONSERVATIVE,
        dst: BROADCAST,
        src: [0x02, 0x42, 0x49, 0x53, phase, 0x01],
        addr3: None,
        addr4: None,
        htc: None,
    };
    let mut sent = 0u64;
    let end = std::time::Instant::now() + std::time::Duration::from_secs(3);
    while std::time::Instant::now() < end {
        if let Err(e) = dev.inject(f.clone()).await {
            println!("RESULT phase={phase} verdict=INJECT_FAILED after={sent} err={e}");
            return Ok(());
        }
        sent += 1;
    }
    println!("RESULT phase={phase} verdict=OK sent={sent}");
    Ok(())
}
