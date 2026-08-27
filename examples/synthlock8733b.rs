//! Does the synth-lock bit predict whether a boot will radiate?
//!
//! ⚠ PREMISE RETRACTED (2026-08-24): bring-up measured 20/20 on a healthy bus; the ~62% was an
//! external USB fault (a failing AX88179 resetting the tree) plus per-boot `usbreset`s. This probe
//! is kept because the synth-lock read it added is still useful. Original premise below.
//! The port has long carried "~62% of cold bring-ups radiate, and there is NO on-chip signal that
//! distinguishes a radiating boot from a dead one — verification must use external feedback". That
//! note predates anyone reading RF `0xc5` BIT15, the vendor's channel-setting-ready (synth lock)
//! bit, which `tune_channel` polls for and used to discard.
//!
//! Prints, per bring-up: the latched lock state, a live re-read after the PHY is up, and the frame
//! rate achieved. **The rate is only a hint** (a boot that never radiates reports implausible rates
//! because nothing consumes airtime, but a boot that radiates then quits averages high) — the SDR
//! or a witness radio remains the authority. Run this while capturing on the B210 to correlate
//! lock against actual on-air presence.
//!
//!   sudo ./synthlock8733b [channel] [seconds-per-boot]

use std::sync::Arc;

use bytes::Bytes;
use ndn_radio_drivers::{BROADCAST, FrameIo, InjectFrame, Rtl8733buBackend, TxIntent};

#[tokio::main(flavor = "multi_thread", worker_threads = 2)]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(36);
    let secs: u64 = std::env::args()
        .nth(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(6);

    let dev = Arc::new(Rtl8733buBackend::open()?);
    let _t = dev.bring_up_tx_tracked(ch)?;
    let latched = dev.synth_locked();
    let live = dev.synth_locked_now().unwrap_or(false);
    println!("SYNTH latched={latched} live_after_bringup={live}");

    let f = InjectFrame {
        payload: Bytes::from(vec![0xC3u8; 300]),
        tx: TxIntent::CONSERVATIVE,
        dst: BROADCAST,
        src: [0x02, 0x53, 0x59, 0x4e, 0x01, 0x01],
        addr3: None,
        addr4: None,
        htc: None,
    };
    let end = std::time::Instant::now() + std::time::Duration::from_secs(secs);
    let mut sent = 0u64;
    while std::time::Instant::now() < end {
        dev.inject(f.clone()).await?;
        sent += 1;
    }
    // Re-read at the END too: if a run starts locked and finishes unlocked, losing lock is the
    // mechanism behind the chip going quiet partway through, which has voided several sweeps.
    let live_end = dev.synth_locked_now().unwrap_or(false);
    println!(
        "RESULT lock_latched={latched} lock_after={live} lock_end={live_end} rate={}/s",
        sent / secs.max(1)
    );
    Ok(())
}
