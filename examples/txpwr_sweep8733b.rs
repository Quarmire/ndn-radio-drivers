//! Does the 8733b's TX-power knob actually move RADIATED power, and at what dB per step?
//!
//! Two claims disagree and neither has been measured on this part: `set_txagc_table`'s doc says
//! "≈ 0.5 dB/step", while the vendor's own `rtl8733b_init_hal_spec` declares `txgi_pdbm = 4`
//! (4 index units per dB = 0.25 dB/step). And there is precedent for the knob being decided but
//! unactuated: on the a81a the TXAGC register was verified held at the requested value while the
//! radiated power did not move at all.
//!
//! ⚠ Sweeping across separate processes would be worthless here: whether a given cold bring-up
//! radiates, and how hard, varies per boot on this part. So this does ONE bring-up and steps the
//! index inside it, tagging each burst with its own source MAC `02:50:57:52:<idx>:01` ("PWR") so a
//! witness can attribute every frame to the index that produced it:
//!
//!   witness:  sudo tcpdump -i <mon> -n 'wlan[10:4] = 0x02505752' -w pwr.pcap
//!   analyse:  tshark -r pwr.pcap -T fields -e wlan.sa -e radiotap.dbm_antsignal
//!
//! Usage: sudo ./txpwr_sweep8733b [channel] [frames-per-index]

use std::sync::Arc;

use bytes::Bytes;
use ndn_radio_drivers::{BROADCAST, FrameIo, InjectFrame, Rtl8733buBackend, TxIntent};

/// Indices to step, spanning the usable AGC range. If radiated power tracks the knob, the witness
/// RSSI should fall monotonically from 0x3f to 0x08 by roughly (0x3f-0x08)/4 ≈ 14 dB at 0.25 dB per
/// step, or ≈ 27 dB at 0.5 dB per step — the two candidate scales are far enough apart to separate.
const INDICES: &[u8] = &[0x3f, 0x30, 0x28, 0x20, 0x18, 0x10, 0x08];

/// Sweep BOTH candidate gain registers in one boot, tagged apart, because a flat result from either
/// one alone is ambiguous: it could mean the register is not in the transmit path, OR that the
/// witness is saturated and cannot show any change. Running them together resolves both questions —
/// if one moves the meter and the other does not, the meter is fine and the flat one is the wrong
/// register. `0x4308` is the TXAGC *reference* (`set_tx_power_idx`, what `RadioKnobs::set_tx_power`
/// currently drives); `0x3a00..` is the per-rate *table* (`set_txagc_table`), which bring-up pins at
/// 0x2d and the reference sweep was measured never to disturb.
#[derive(Clone, Copy)]
enum Knob {
    /// src MAC 02:'P':'W':'R':<idx>:01
    Reference,
    /// src MAC 02:'T':'A':'B':<idx>:01
    Table,
}

#[tokio::main(flavor = "multi_thread", worker_threads = 2)]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(36);
    let n: usize = std::env::args()
        .nth(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(1500);

    let dev = Arc::new(Rtl8733buBackend::open()?);
    // ONE bring-up for the whole sweep — see the per-boot variance note above. Keep the tracker
    // alive so thermal droop doesn't masquerade as a power-index effect across the later indices.
    let _tracker = dev.bring_up_tx_tracked(ch)?;
    println!("txpwr_sweep8733b ch{ch} {n} frames/index; src MAC = 02:50:57:52:<idx>:01");

    let payload = Bytes::from(vec![0x5Au8; 400]);
    for knob in [Knob::Reference, Knob::Table] {
        // Restore the other register to its bring-up value first, so each arm varies ONE thing.
        match knob {
            Knob::Reference => dev.set_txagc_table(0x2d)?,
            Knob::Table => dev.set_tx_power_idx(0x40)?,
        }
        for &idx in INDICES {
            match knob {
                Knob::Reference => dev.set_tx_power_idx(idx)?,
                Knob::Table => dev.set_txagc_table(idx)?,
            }
            // Let the tracker's ~400 ms tick pass so the two power paths settle before the burst.
            tokio::time::sleep(std::time::Duration::from_millis(600)).await;
            let pfx = match knob {
                Knob::Reference => [0x50, 0x57, 0x52], // "PWR" — 0x4308 reference
                Knob::Table => [0x54, 0x41, 0x42],     // "TAB" — 0x3a00 per-rate table
            };
            let f = InjectFrame {
                payload: payload.clone(),
                tx: TxIntent::CONSERVATIVE,
                dst: BROADCAST,
                src: [0x02, pfx[0], pfx[1], pfx[2], idx, 0x01],
                addr3: None,
                addr4: None,
                htc: None,
            };
            for _ in 0..n {
                dev.inject(f.clone()).await?;
            }
            println!(
                "  {} idx 0x{idx:02x}: {n} frames sent",
                match knob {
                    Knob::Reference => "ref  ",
                    Knob::Table => "table",
                }
            );
        }
    }
    println!("done — read mean RSSI per source MAC at the witness");
    Ok(())
}
