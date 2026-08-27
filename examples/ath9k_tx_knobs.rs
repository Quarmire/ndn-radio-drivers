//! **AR9271 TX knob demo — MCS/HT rate + TX power, actuated on air (our firmware).**
//!
//! With the patched firmware (`ath_tgt_send_mgt` reads a per-frame rate code + power from the mgmt
//! header), `FrameIo::set_rate` and `RadioKnobs::set_tx_power`/`set_tx_power_dbm` steer the on-air
//! rate and power. This sweeps HT MCS 0→7 (each a burst) then two power levels at a fixed MCS, so a
//! witness on the same channel sees the rate climb through the MCS ladder and the RSSI move with power.
//!
//! ```sh
//! sudo NDN_ATH9K_FW=~/ath9k-fw/target_firmware/build/k2/htc_9271.fw ./ath9k_tx_knobs [ch=1]
//! ```
use std::process::ExitCode;
use std::time::Duration;

use bytes::Bytes;
use ndn_radio_drivers::open_ath9k;
use ndn_radio_hal::{InjectFrame, McsDescriptor, TxIntent};

fn main() -> ExitCode {
    let ch: u8 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(1);
    let radio = match open_ath9k(ch) {
        Ok(r) => r,
        Err(e) => {
            eprintln!("open_ath9k FAILED: {e}");
            return ExitCode::FAILURE;
        }
    };
    let io = radio.io.clone();
    let knobs = radio.knobs.clone().expect("knobs");

    let rt = tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
        .unwrap();
    rt.block_on(async {
        let burst = |io: std::sync::Arc<dyn ndn_radio_hal::FrameIo>, tag: u8, n: usize| async move {
            for i in 0..n {
                // payload byte[2] = tag so a witness capture can correlate a burst to its knob setting
                let p = vec![0x05u8, 0x08, tag, i as u8, 0x6b, 0x6e, 0x6f, 0x62];
                let f = InjectFrame::broadcast(Bytes::copy_from_slice(&p), TxIntent::CONSERVATIVE);
                let _ = io.inject(f).await;
                tokio::time::sleep(Duration::from_millis(15)).await;
            }
        };

        // ── MCS sweep 0→7 at default power ──
        println!("MCS sweep (each burst = one HT MCS; witness rate should climb 6.5→65 Mb/s):");
        for mcs in 0u8..=7 {
            io.set_rate(McsDescriptor {
                index: mcs,
                short_gi: false,
                vht: false,
                nss: 1,
                stbc: false,
                ldpc: false,
            })
            .ok();
            println!("  MCS{mcs} (tag {mcs}) — 30 frames");
            burst(io.clone(), mcs, 30).await;
            tokio::time::sleep(Duration::from_millis(300)).await;
        }

        // ── power test at MCS0: high then low (witness RSSI should drop) ──
        io.set_rate(McsDescriptor {
            index: 0,
            short_gi: false,
            vht: false,
            nss: 1,
            stbc: false,
            ldpc: false,
        })
        .ok();
        for (tag, dbm) in [(0x20u8, 30i8), (0x21u8, 6i8)] {
            let applied = knobs.set_tx_power_dbm(dbm).unwrap_or(-1);
            println!("power {dbm} dBm (applied {applied}) tag {tag:#04x} — 40 frames");
            burst(io.clone(), tag, 40).await;
            tokio::time::sleep(Duration::from_millis(300)).await;
        }
        println!(
            "done — check the witness for MCS0-7 rates and the RSSI drop on the low-power burst."
        );
    });
    let _ = radio;
    ExitCode::SUCCESS
}
