//! The two RTL8733BU gain paths that are NOT digital TXAGC, after all three digital ones measured
//! flat on a meter with headroom (2026-08-24: reference 0x4308, per-rate table 0x3a00 and datapath
//! 0x1e4x each moved <3 dB, non-monotonic, across a commanded 14-27 dB).
//!
//!   knob 3 — RF TXAGC, RF register 0x01[4:0], both paths (`set_rf_txagc`). The ANALOG PA drive;
//!            the vendor holds it near 0x1a and at 0 nothing radiates, so it gates real output.
//!   knob 4 — `0x18a0[6:0]`, the OFDM swing (`absolute_ofdm_swing_idx`).
//!
//! ⚠ Runs with the power tracker DELIBERATELY OFF (`bring_up_tx`, not `bring_up_tx_tracked`). The
//! tracker rewrites 0x18a0 every 400 ms off the die thermal — it was alive during all three earlier
//! sweeps, so if 0x18a0 is the dominant gain it was pinning the output and no other register could
//! ever have shown through. Knob 4 is untestable while it runs.
//!
//! With the tracker off, thermal droop is a real confound over a multi-minute sweep, so the index
//! order is INTERLEAVED (extremes alternate) rather than descending: a monotone drift then cannot
//! produce a monotone index-vs-RSSI relation, which a simple ramp would happily fake.
//!
//! Usage: sudo ./txpwr_rf8733b [channel] [frames-per-index]

use std::sync::Arc;

use bytes::Bytes;
use ndn_radio_drivers::{BROADCAST, FrameIo, InjectFrame, Rtl8733buBackend, TxIntent};

/// RF 0x01[4:0] is 5 bits: 0x00-0x1f. Interleaved high/low.
const RF_IDX: &[u8] = &[0x1f, 0x00, 0x18, 0x04, 0x14, 0x08, 0x10, 0x0c];
/// 0x18a0[6:0] is 7 bits. Interleaved high/low.
const SWING_IDX: &[u8] = &[0x7f, 0x00, 0x60, 0x10, 0x50, 0x20, 0x40, 0x30];

#[tokio::main(flavor = "multi_thread", worker_threads = 2)]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(36);
    let n: usize = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(900);

    let dev = Arc::new(Rtl8733buBackend::open()?);
    dev.bring_up_tx(ch)?; // NO power tracker — see above
    println!("txpwr_rf ch{ch} {n}/index, tracker OFF, interleaved order");

    for knob in 3u8..5 {
        // Restore the other path to a sane mid value so exactly one thing varies.
        if knob == 3 {
            let v = dev.read32(0x18a0)?;
            dev.write32(0x18a0, (v & !0x7f) | 0x20)?;
        } else {
            dev.set_rf_txagc(0x1a)?; // the vendor's resting value
        }
        let list: &[u8] = if knob == 3 { RF_IDX } else { SWING_IDX };
        for &idx in list {
            match knob {
                3 => dev.set_rf_txagc(idx)?,
                _ => {
                    let v = dev.read32(0x18a0)?;
                    dev.write32(0x18a0, (v & !0x7f) | u32::from(idx & 0x7f))?;
                }
            }
            tokio::time::sleep(std::time::Duration::from_millis(500)).await;
            let mut p = vec![0xC3u8; 300];
            p[0] = idx;
            p[1] = knob;
            let f = InjectFrame {
                payload: Bytes::from(p),
                tx: TxIntent::CONSERVATIVE,
                dst: BROADCAST,
                src: [0x02, 0x50, 0x33, 0x01, knob, idx],
                addr3: None,
            };
            for _ in 0..n {
                dev.inject(f.clone()).await?;
            }
            println!("  knob {knob} idx 0x{idx:02x}: {n} sent");
        }
    }
    println!("done");
    Ok(())
}
