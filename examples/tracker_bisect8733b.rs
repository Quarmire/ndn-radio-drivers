//! WHICH part of the power tracker conflicts with the hardware TSSI loop?
//!
//! Established: with TSSI enabled, a sweep that spawns `spawn_power_tracking` dies on the first bulk
//! transfer, while an otherwise identical run without it completes 8/8 (and the full `tssi_setup`
//! itself is innocent — all ten phases pass). But "the tracker" is two different operations every
//! 400 ms, and they implicate very different bugs:
//!
//!   thermal — `read_thermal()`: RF register access (rf 0x42 toggles + read) via VENQT control
//!   swing   — read/modify/write of BB `0x18a0[6:0]`, the OFDM swing
//!
//! If BOTH modes fail, the fault is concurrent USB control transfers from two threads — a
//! driver-wide thread-safety problem affecting every backend, not just TSSI.
//! If only `swing` fails, it is a semantic conflict: two controllers writing the same quantity,
//! and the fix is exactly the one applied (suppress the software tracker when TSSI owns thermal).
//! If only `thermal` fails, the RF read path is disturbing the running loop.
//!
//!   sudo ./tracker_bisect8733b <channel> <mode 0..3>   (TSSI is now on by default;
//!   use NDN_8733B_NO_TSSI=1 for the loop-off arm)
//! mode 0 none (control) · 1 thermal-only · 2 swing-only · 3 both (full tracker)

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use bytes::Bytes;
use ndn_radio_drivers::{BROADCAST, FrameIo, InjectFrame, Rtl8733buBackend, TxIntent};

#[tokio::main(flavor = "multi_thread", worker_threads = 2)]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(36);
    let mode: u8 = std::env::args()
        .nth(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(0);

    let dev = Arc::new(Rtl8733buBackend::open()?);
    // `bring_up_tx` applies TSSI by default now; it does NOT spawn a tracker, so the
    // only tracker running is the one this example starts below.
    dev.bring_up_tx(ch)?;

    let stop = Arc::new(AtomicBool::new(false));
    let handle = if mode > 0 {
        let d = Arc::clone(&dev);
        let s = Arc::clone(&stop);
        Some(std::thread::spawn(move || {
            // Same cadence as the real tracker so the comparison is like-for-like.
            while !s.load(Ordering::Relaxed) {
                if mode == 1 || mode == 3 {
                    let _ = d.read_thermal();
                }
                if mode == 2 || mode == 3 {
                    if let Ok(v) = d.read32(0x18a0) {
                        let _ = d.write32(0x18a0, (v & !0x7f) | 0x20);
                    }
                }
                std::thread::sleep(std::time::Duration::from_millis(400));
            }
        }))
    } else {
        None
    };

    let f = InjectFrame {
        payload: Bytes::from(vec![0xC3u8; 300]),
        tx: TxIntent::CONSERVATIVE,
        dst: BROADCAST,
        src: [0x02, 0x54, 0x52, 0x4b, mode, 0x01],
        addr3: None,
        addr4: None,
        htc: None,
    };
    let mut sent = 0u64;
    let end = std::time::Instant::now() + std::time::Duration::from_secs(6);
    let mut verdict = "OK";
    let mut err = String::new();
    while std::time::Instant::now() < end {
        if let Err(e) = dev.inject(f.clone()).await {
            verdict = "INJECT_FAILED";
            err = format!("{e}");
            break;
        }
        sent += 1;
    }
    stop.store(true, Ordering::Relaxed);
    if let Some(h) = handle {
        let _ = h.join();
    }
    let name = ["none", "thermal-only", "swing-only", "both"][mode.min(3) as usize];
    println!("RESULT mode={mode}({name}) verdict={verdict} sent={sent} {err}");
    Ok(())
}
