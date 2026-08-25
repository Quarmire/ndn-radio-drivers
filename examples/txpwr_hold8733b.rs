//! Transmit side of the B210 TXAGC sweep: bring the 8733b up ONCE and step the gain index on a
//! fixed dwell, announcing each step on stdout so the SDR host can align captures without a shared
//! clock.
//!
//! One boot for the whole sweep, not one boot per index. Per-index boots were tried first and are
//! unusable (PREMISE RETRACTED 2026-08-24 — measured 20/20; it was a bus fault, not the chip):
//! only ~62% of cold bring-ups radiate at all, so ~40% of indices would score as "low
//! power" purely because the chip was inert — a failure mode that manufactures a monotone-looking
//! curve out of nothing. Within a single radiating boot the transmitter is reliable (measured: the
//! a81a received 896-900 of 900 frames in all 24 arms of a one-boot sweep).
//!
//! Prints `FLOOD_START` once the PHY is up and frames are flowing, then `IDX <hex>` at each step, so
//! the capture host polls for the former and then counts dwells.
//!
//! Usage: sudo NDN_PWR_KNOB=datapath|table|ref NDN_DWELL=12 ./txpwr_hold8733b <channel>

use std::sync::Arc;

use bytes::Bytes;
use ndn_radio_drivers::{BROADCAST, FrameIo, InjectFrame, Rtl8733buBackend, TxIntent};

#[tokio::main(flavor = "multi_thread", worker_threads = 2)]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(36);
    let knob = std::env::var("NDN_PWR_KNOB").unwrap_or_else(|_| "datapath".into());
    let dwell: u64 = std::env::var("NDN_DWELL").ok().and_then(|s| s.parse().ok()).unwrap_or(3);
    // Interleaved, so a monotone thermal drift over the ~96 s pass cannot fake a monotone curve.
    // Datapath TXAGC sweep, to be run WITH the full TSSI loop enabled (NDN_8733B_TSSI=1).
    // Every gain control measured inert with TSSI off; if the loop is what consults these tables,
    // the same indices that gave 1.9 dB of unordered scatter should now produce a curve.
    let indices: Vec<u8> = vec![0x3f, 0x00, 0x30, 0x08, 0x20, 0x10, 0x28, 0x18];

    let dev = Arc::new(Rtl8733buBackend::open()?);
    let _t = dev.bring_up_tx_tracked(ch)?;
    let payload = Bytes::from(vec![0xC3u8; 300]);

    // Warm-up flood before announcing: the capture host must not start counting dwells until frames
    // are genuinely on the air, or every index is offset by the (variable) calibration time.
    let warm = InjectFrame {
        payload: payload.clone(), tx: TxIntent::CONSERVATIVE, dst: BROADCAST,
        src: [0x02, 0x50, 0x48, 0x4c, 0xff, 0x01], addr3: None,
    };
    let t0 = std::time::Instant::now();
    while t0.elapsed() < std::time::Duration::from_secs(2) {
        dev.inject(warm.clone()).await?;
    }
    println!("FLOOD_START knob={knob} dwell={dwell}");

    for &idx in &indices {
        match knob.as_str() {
            "ref" => dev.set_tx_power_idx(idx)?,
            "table" => dev.set_txagc_table(idx)?,
            "datapath" => dev.set_txagc_datapath(idx)?,
            // idx 0 = TSSI off (our current default), idx 1 = TSSI on (what the vendor runs).
            "tssi" => dev.set_tssi_enabled(idx != 0)?,
            // Per-frame descriptor offset — takes effect on the NEXT injected frame, no register
            // commit and nothing for the MAC's rate/power-group selection to bypass.
            _ => dev.set_tx_pwr_offset(idx),
        }
        println!("IDX {idx:02x}");
        // Carry the index IN THE PAYLOAD (byte 0 = index, byte 1 = knob id, rest 0xC3) so the
        // receiving side can attribute every frame with no MAC filter and no clock alignment — that
        // is the contract `rxpwr_bucket` reads. The source MAC keeps the index too, for tcpdump.
        let mut p = vec![0xC3u8; 300];
        p[0] = idx;
        p[1] = match knob.as_str() { "ref" => 0, "table" => 1, "datapath" => 2, "tssi" => 6, _ => 5 };
        let f = InjectFrame {
            payload: Bytes::from(p), tx: TxIntent::CONSERVATIVE, dst: BROADCAST,
            src: [0x02, 0x50, 0x48, 0x4c, idx, 0x01], addr3: None,
        };
        let end = std::time::Instant::now() + std::time::Duration::from_secs(dwell);
        let mut sent = 0u64;
        while std::time::Instant::now() < end {
            dev.inject(f.clone()).await?;
            sent += 1;
        }
        println!("  idx {idx:02x} sent={sent} rate={}/s", sent / dwell.max(1));
    }
    println!("SWEEP_DONE");
    Ok(())
}
