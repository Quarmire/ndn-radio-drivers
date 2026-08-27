//! **OLPC set_board_values M2 — apply the EEPROM analog cal + measure the TX-power change (B210).**
//!
//! Two phases with a B210 co-located: flood BEFORE `set_board_values` (baseline uncalibrated PA), then
//! apply the cal (antCtrl + XATTEN gain + ob/db PA-bias from the OTP) and flood AFTER. A power rise
//! between the two plateaus = the analog cal is driving the PA harder — the first real step of the fix.
//! `NDN_ATH9K_HIGHPWR=1` brings the PHY up on the high-power gain table (txGainType=1 says this module
//! wants it) so board-values + the right table compose.
//!
//! ```sh
//! sudo NDN_ATH9K_FW=/tmp/htc_9271.fw [NDN_ATH9K_HIGHPWR=1] /tmp/ath9k_setboard 1
//! ```
use std::process::ExitCode;
use std::time::{Duration, Instant};

use bytes::Bytes;
use ndn_radio_drivers::Ath9kHtcBackend;
use ndn_radio_hal::{FrameIo, InjectFrame, TxIntent};

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().collect();
    let ch: u8 = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(1);
    let fw = std::fs::read(std::env::var("NDN_ATH9K_FW").expect("NDN_ATH9K_FW")).expect("fw");
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
    let flood = |dev: &Ath9kHtcBackend, secs: u64| {
        rt.block_on(async {
            let end = Instant::now() + Duration::from_secs(secs);
            while Instant::now() < end {
                let _ = dev
                    .inject(InjectFrame::broadcast(big.clone(), TxIntent::CONSERVATIVE))
                    .await;
            }
        })
    };

    // PHASE 1: baseline (uncalibrated PA, no set_board_values).
    println!(
        "PHASE1 baseline t={:.1}s (no board-values) — flooding 6s",
        t0.elapsed().as_secs_f64()
    );
    flood(&dev, 6);
    println!("  gap t={:.1}s", t0.elapsed().as_secs_f64());
    std::thread::sleep(Duration::from_secs(1));

    // Apply the EEPROM analog cal (M2) unless skipped.
    if std::env::var_os("NDN_SKIP_BOARD").is_none() {
        match dev.set_board_values() {
            Ok(bv) => println!(
                "set_board_values ✓ txGainType={} ob={:?} db1={} db2={}",
                bv.tx_gain_type, bv.ob, bv.db1_0, bv.db2_0
            ),
            Err(e) => println!("set_board_values FAILED: {e}"),
        }
    }
    // ★ M3: the OLPC power cal (PDADC target→gain map + per-rate target power) — the actual lever.
    match dev.set_txpower_4k(chan_mhz) {
        Ok(peak) => println!(
            "set_txpower_4k ✓ peak target = {} (0.5dB) = {} dBm",
            peak,
            peak / 2
        ),
        Err(e) => println!("set_txpower_4k FAILED: {e}"),
    }

    // PHASE 2: after board-values (PA biased from the OTP cal).
    println!(
        "PHASE2 boarded t={:.1}s — flooding 6s",
        t0.elapsed().as_secs_f64()
    );
    flood(&dev, 6);
    println!("SWEEP DONE t={:.1}s", t0.elapsed().as_secs_f64());
    println!("# B210: compare the phase-1 vs phase-2 power plateaus; a rise = the PA cal took.");
    let _ = dev.detach();
    ExitCode::SUCCESS
}
