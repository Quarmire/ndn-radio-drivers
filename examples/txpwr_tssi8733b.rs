//! Is the TSSI loop the reason none of the RTL8733BU's gain registers moves radiated power?
//!
//! Measured 2026-08-24, all on a receiver with headroom, all inside one bring-up: the TXAGC
//! reference (0x4308), the per-rate table (0x3a00), the datapath block (0x1e4x), the RF analog gain
//! (RF 0x01[4:0]) and the OFDM swing (0x18a0) each moved RSSI by <3 dB, non-monotonic, against a
//! commanded 14-27 dB. Most tellingly, RF gain **0** — which the driver's own note says leaves the
//! PA with zero input so nothing radiates — transmitted at full strength.
//!
//! Five independent gain controls cannot all be individually broken. A closed feedback loop
//! regulating output to a setpoint explains all of it at once, and this chip has one: `tssi_setup`'s
//! own doc says "on the 8731bu the per-rate TX power is driven by TSSI when enabled
//! (`0x4318[30:28]=7`)". If TSSI is live after bring-up, it measures transmitted strength and pulls
//! power back to target regardless of what the host writes upstream of it.
//!
//! Prints the TSSI field first (7 = enabled), then sweeps the datapath block with TSSI forced OFF.
//! If the sweep actuates only with TSSI off, that is the answer and the production knob must either
//! disable TSSI or drive TSSI's setpoint instead of the gain registers.
//!
//! Usage: sudo ./txpwr_tssi8733b [channel] [frames-per-index]

use std::sync::Arc;

use bytes::Bytes;
use ndn_radio_drivers::{BROADCAST, FrameIo, InjectFrame, Rtl8733buBackend, TxIntent};

/// Interleaved so thermal droop (tracker is off) cannot fake a monotone trend.
const IDX: &[u8] = &[0x3f, 0x08, 0x38, 0x10, 0x30, 0x18, 0x28, 0x20];

#[tokio::main(flavor = "multi_thread", worker_threads = 2)]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(36);
    let n: usize = std::env::args()
        .nth(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(900);

    let dev = Arc::new(Rtl8733buBackend::open()?);
    dev.bring_up_tx(ch)?; // tracker off

    let r = dev.read32(0x4318)?;
    let tssi = (r >> 28) & 0x7;
    println!(
        "after bring_up_tx: 0x4318={r:08x}  TSSI[30:28]={tssi}  ({})",
        if tssi == 7 {
            "ENABLED — a closed loop is regulating TX power"
        } else {
            "not 7"
        }
    );

    for knob in 5u8..7 {
        // knob 5 = TSSI left as bring-up leaves it (the control)
        // knob 6 = TSSI forced OFF, so the datapath gain is the only thing setting power
        if knob == 6 {
            dev.set_tssi_enabled(false)?;
            let v = dev.read32(0x4318)?;
            println!(
                "  TSSI forced off -> 0x4318={v:08x} field={}",
                (v >> 28) & 0x7
            );
        }
        for &idx in IDX {
            dev.set_txagc_datapath(idx)?;
            tokio::time::sleep(std::time::Duration::from_millis(500)).await;
            let mut p = vec![0xC3u8; 300];
            p[0] = idx;
            p[1] = knob;
            let f = InjectFrame {
                payload: Bytes::from(p),
                tx: TxIntent::CONSERVATIVE,
                dst: BROADCAST,
                src: [0x02, 0x50, 0x33, 0x02, knob, idx],
                addr3: None,
                addr4: None,
                htc: None,
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
