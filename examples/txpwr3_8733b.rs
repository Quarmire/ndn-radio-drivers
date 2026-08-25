//! Which of the RTL8733BU's THREE gain controls actually moves radiated power?
//!
//!   0x4308        TXAGC *reference*   — `set_tx_power_idx`, what `RadioKnobs::set_tx_power` drives
//!   0x3a00..      per-rate *table*    — `set_txagc_table`, pinned at 0x2d by bring-up
//!   0x1e44..0x1e54 datapath per-rate  — `set_txagc_datapath`, restored by `enable_tx`; without this
//!                                       block nothing radiates at all, so it is the prime suspect
//!
//! Register readback proves the three are independent (writing one never disturbs the others), and
//! two prior on-air sweeps of the first two were FLAT — but against a witness that reported only
//! -18/-16/-14 dBm and disagreed with a second receiver by 54 dB, so they proved nothing.
//!
//! Design constraints this encodes, both learned the hard way:
//!   * ONE bring-up for the whole sweep. Whether a cold boot radiates, and how hard, varies per boot
//!     on this part, so a knob stepped across processes is confounded by boot-to-boot variance.
//!   * The index is carried in the PAYLOAD (`payload[0] = index`, `payload[1] = knob id`), not in the
//!     source MAC, so the receiver can attribute every frame without a MAC filter — and so the
//!     measuring receiver can be the a81a, whose RSSI actually varies (-73..-66 observed), instead of
//!     the kernel meter that is pinned.
//!   * Each arm restores the other two registers to their bring-up values, so exactly one thing moves.
//!
//! Pair with `rxpwr_bucket` on the receiving node.
//! Usage: sudo ./txpwr3_8733b [channel] [frames-per-index]

use std::sync::Arc;

use bytes::Bytes;
use ndn_radio_drivers::{BROADCAST, FrameIo, InjectFrame, Rtl8733buBackend, TxIntent};

const INDICES: &[u8] = &[0x3f, 0x38, 0x30, 0x28, 0x20, 0x18, 0x10, 0x08];
/// Bring-up values, restored on the two registers an arm is not sweeping.
const REF_DEFAULT: u8 = 0x40;
const TABLE_DEFAULT: u8 = 0x2d;
const DP_DEFAULT: u8 = 0x24; // mid-point of the stock 0x1c..0x38 ramp

#[tokio::main(flavor = "multi_thread", worker_threads = 2)]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(36);
    let n: usize = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(900);

    let dev = Arc::new(Rtl8733buBackend::open()?);
    let _tracker = dev.bring_up_tx_tracked(ch)?; // keep tracking alive: thermal droop would otherwise
                                                 // look like a gain effect on the later arms
    println!("txpwr3 ch{ch} {n} frames/index — payload[0]=index payload[1]=knob(0 ref,1 table,2 datapath)");

    for knob in 0u8..3 {
        // Exactly one control varies per arm.
        dev.set_tx_power_idx(if knob == 0 { REF_DEFAULT } else { REF_DEFAULT })?;
        dev.set_txagc_table(TABLE_DEFAULT)?;
        dev.set_txagc_datapath(DP_DEFAULT)?;
        for &idx in INDICES {
            match knob {
                0 => dev.set_tx_power_idx(idx)?,
                1 => dev.set_txagc_table(idx)?,
                _ => dev.set_txagc_datapath(idx)?,
            }
            tokio::time::sleep(std::time::Duration::from_millis(500)).await;
            let mut p = vec![0xC3u8; 300];
            p[0] = idx;
            p[1] = knob;
            let f = InjectFrame {
                payload: Bytes::from(p),
                tx: TxIntent::CONSERVATIVE,
                dst: BROADCAST,
                src: [0x02, 0x50, 0x33, 0x00, knob, idx],
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
