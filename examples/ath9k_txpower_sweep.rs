//! **#8 — AR9271 TX-power gain-ladder sweep for B210 absolute-dBm calibration.**
//!
//! Steps the TX gain LUT through all 11 levels (via `RadioKnobs::set_tx_power`, whose idx→level map is
//! `idx*10/63`), flooding a high-duty 1 Mbps CCK carrier at each for a fixed window with a silent gap
//! between — so a co-located B210 capture shows 11 clean descending power plateaus separated by
//! noise-floor gaps. Segment the capture in order, measure the per-level power, anchor the top level
//! (idx 63 = reset-default max gain) to the AR9271's datasheet max, and the drops give the absolute
//! dBm↔level table `set_tx_power_dbm` should use.
//!
//! ```sh
//! sudo NDN_ATH9K_FW=/tmp/htc_9271.fw /tmp/ath9k_txpower_sweep 1
//! ```
use std::process::ExitCode;
use std::time::{Duration, Instant};

use bytes::Bytes;
use ndn_radio_drivers::{Ath9kHtcBackend, LegacyRate};
use ndn_radio_hal::{FrameIo, InjectFrame, RadioKnobs, TxIntent};

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().collect();
    let ch: u8 = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(1);
    let fw_path = std::env::var("NDN_ATH9K_FW").expect("set NDN_ATH9K_FW");
    let fw = std::fs::read(&fw_path).expect("read fw");
    let chan_mhz = if ch == 14 { 2484 } else { 2407 + 5 * ch as u16 };

    let mut dev = Ath9kHtcBackend::open().expect("open");
    dev.download_firmware(&fw)
        .and_then(|_| dev.htc_init())
        .expect("transport");
    dev.hw_reset(chan_mhz)
        .and_then(|_| dev.connect_data_services())
        .expect("bring-up");
    let _ = dev.write_target_u32s(0x0050_cf44, &[0]);
    dev.wmi_start()
        .and_then(|_| dev.start_receive())
        .expect("rx-start");

    // The gain LUT (0xa334-0xa354) is the OFDM NORMAL_POWER_TX_GAIN table — CCK (1 Mbps, the target
    // min) uses a SEPARATE power path, so a CCK flood would be flat regardless. Flood OFDM 6M so the
    // gain-ladder sweep actually bites. `NDN_SWEEP_CCK=1` forces the old CCK behaviour for comparison.
    let ofdm = std::env::var_os("NDN_SWEEP_CCK").is_none();
    if ofdm {
        dev.set_legacy_rate(LegacyRate::Ofdm6);
        println!("# flooding OFDM 6M (gain LUT is the OFDM power table)");
    } else {
        println!("# flooding CCK 1M (control — gain LUT should NOT affect CCK)");
    }

    // idx values that map to gain levels 10..0 (level = idx*10/63). Top = idx 63 = reset-default max.
    let steps: [(u32, u32); 11] = [
        (10, 63),
        (9, 57),
        (8, 51),
        (7, 45),
        (6, 38),
        (5, 32),
        (4, 26),
        (3, 19),
        (2, 13),
        (1, 7),
        (0, 0),
    ];
    const FLOOD_S: u64 = 4;
    const GAP_S: u64 = 1;

    // Large (900 B) payload → each 1 Mbps CCK frame is ~7.5 ms, so back-to-back injects give a
    // near-continuous high-duty carrier (a low-duty burst washes out under any averaging).
    let big = {
        let mut v = Vec::with_capacity(900);
        v.extend_from_slice(b"\x05\x08");
        v.resize(900, b'p');
        Bytes::from(v)
    };
    let rt = tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
        .unwrap();
    let t0 = Instant::now();
    println!("SWEEP START chan={chan_mhz}MHz  (flood {FLOOD_S}s + {GAP_S}s silent gap per level)");
    println!("# order is DESCENDING power: level 10 (max) → 0. Segment the capture the same way.");
    for (level, idx) in steps {
        dev.set_tx_power(idx).ok();
        println!(
            "LEVEL {level:>2} idx={idx:>2}  t={:.1}s  (flooding 1Mbps CCK, 900B)",
            t0.elapsed().as_secs_f64()
        );
        let end = Instant::now() + Duration::from_secs(FLOOD_S);
        rt.block_on(async {
            while Instant::now() < end {
                let f = InjectFrame::broadcast(big.clone(), TxIntent::CONSERVATIVE);
                let _ = dev.inject(f).await;
            }
        });
        // Silent gap → a clean noise-floor boundary between plateaus in the B210 capture.
        println!("  gap    t={:.1}s  (silent)", t0.elapsed().as_secs_f64());
        std::thread::sleep(Duration::from_secs(GAP_S));
    }
    println!("SWEEP DONE t={:.1}s", t0.elapsed().as_secs_f64());
    let _ = dev.detach();
    ExitCode::SUCCESS
}
