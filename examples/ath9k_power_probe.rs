//! **AR9271 TX-power reality check — which lever actually moves radiated power?**
//!
//! Injects in three time-separated windows so a witness can compare RSSI per window:
//!   W1 (t≈0-4s):  XmitPower=60 (30 dBm commanded), default power regs — the baseline.
//!   W2 (t≈5-9s):  XmitPower=8  (4 dBm commanded)  — tests the per-frame descriptor XmitPower lever.
//!   W3 (t≈10-14s): XmitPower=60 but AR_PHY_POWER_TX_RATE1..9 written to 0x08 (4 dBm) per rate — tests
//!                  the per-rate power-table lever.
//! If W1==W2 the descriptor XmitPower is dead (power cal skipped); if W3<W1 the power-rate table is the
//! real lever. 4-second gaps of silence separate the windows in the capture.
//!
//! ```sh
//! sudo /tmp/ath9k_power_probe ~/ath9k-fw/target_firmware/build/k2/htc_9271.fw
//! ```
use std::process::ExitCode;
use std::time::Duration;

use bytes::Bytes;
use ndn_radio_drivers::Ath9kHtcBackend;
use ndn_radio_hal::{FrameIo, InjectFrame, RadioKnobs, TxIntent};

// AR_PHY_POWER_TX_RATE1..9 (AR5416/AR9271 reg.h).
const PWR_TX_RATE: [u32; 9] = [0x9934, 0x9938, 0xa234, 0xa238, 0xa38c, 0xa390, 0xa3cc, 0xa3d0, 0xa3d4];

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().collect();
    let fw = std::fs::read(args.get(1).expect("usage: <fw>")).expect("read fw");
    let mut dev = Ath9kHtcBackend::open().expect("open");
    dev.download_firmware(&fw).and_then(|_| dev.htc_init()).expect("transport");
    dev.hw_reset(2412).and_then(|_| dev.connect_data_services()).expect("bring-up");
    let _ = dev.write_target_u32s(0x0050_cf44, &[0]);
    dev.wmi_start().and_then(|_| dev.start_receive()).expect("rx-start");

    // Record the default power-rate registers so W1/W2 use them and we can see what they were.
    let orig: Vec<u32> = PWR_TX_RATE.iter().map(|&a| dev.reg_read(a).unwrap_or(0)).collect();
    println!("AR_PHY_POWER_TX_RATE defaults: {:08x?}", orig);

    async fn burst(dev: &Ath9kHtcBackend, tag: u8, n: usize) {
        for i in 0..n {
            let f = InjectFrame::broadcast(Bytes::copy_from_slice(&[0x05, 0x08, tag, i as u8]), TxIntent::CONSERVATIVE);
            let _ = dev.inject(f).await;
            tokio::time::sleep(Duration::from_millis(60)).await;
        }
    }
    let rt = tokio::runtime::Builder::new_current_thread().enable_all().build().unwrap();
    rt.block_on(async {
        // Now exercise the PRODUCTION knob RadioKnobs::set_tx_power (which writes the gain LUT).
        use ndn_radio_hal::RadioKnobs;
        RadioKnobs::set_tx_power(&dev, 63).ok();
        println!("W1: set_tx_power(63) = max — tag 0xA0");
        burst(&dev, 0xA0, 60).await;
        tokio::time::sleep(Duration::from_secs(4)).await;

        RadioKnobs::set_tx_power(&dev, 30).ok();
        println!("W2: set_tx_power(30) = mid — tag 0xA1");
        burst(&dev, 0xA1, 60).await;
        tokio::time::sleep(Duration::from_secs(4)).await;

        RadioKnobs::set_tx_power(&dev, 10).ok();
        println!("W3: set_tx_power(10) = low — tag 0xA2");
        burst(&dev, 0xA2, 60).await;
    });

    // restore
    for (a, v) in PWR_TX_RATE.iter().zip(orig.iter()) {
        let _ = dev.reg_write(*a, *v);
    }
    let _ = dev.detach();
    println!("done — compare witness RSSI across the three ~4s windows (W1 t0-4, W2 t8-12, W3 t16-20).");
    ExitCode::SUCCESS
}
