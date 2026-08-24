//! **AR9271 HT40 (40 MHz) bring-up verification.** Brings the PHY up in HT40 and checks, at the
//! register/descriptor level: the synth centre offset (+10 MHz vs HT20), `AR_PHY_TURBO` DYN2040 set,
//! AGC cal convergence, and the injected frame's `ds_ctl7` AR_2040_0 (0x1) bit.
//!
//! ```sh
//! sudo /tmp/ath9k_ht40 ~/ath9k-fw/target_firmware/build/k2/htc_9271.fw [ch=1]
//! ```
use std::process::ExitCode;
use std::time::Duration;

use bytes::Bytes;
use ndn_radio_drivers::Ath9kHtcBackend;
use ndn_radio_hal::{FrameIo, InjectFrame, McsDescriptor, TxIntent};

const AR_PHY_SYNTH_CONTROL: u32 = 0x9874;
const AR_PHY_TURBO: u32 = 0x9804;
const AR_PHY_FC_DYN2040_EN: u32 = 0x0000_0004;
const AR_PHY_ACTIVE: u32 = 0x981c;
const AR_QTXDP1: u32 = 0x0800 + (1 << 2);
const AR_2040_0: u32 = 0x0000_0001; // ds_ctl7 40 MHz bit

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().collect();
    let fw = std::fs::read(args.get(1).expect("usage: <fw> [ch]")).expect("read fw");
    let ch: u16 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(1);
    let chan_mhz = 2407 + 5 * ch;
    let mut dev = Ath9kHtcBackend::open().expect("open");
    dev.download_firmware(&fw).and_then(|_| dev.htc_init()).expect("transport");

    println!("bringing up HT40 on primary ch{ch} ({chan_mhz} MHz), expect synth centre {} MHz", chan_mhz + 10);
    if let Err(e) = dev.hw_reset_ht40(chan_mhz) {
        eprintln!("hw_reset_ht40 FAILED: {e}  (cal may not converge at 40 MHz on this part)");
        return ExitCode::FAILURE;
    }
    dev.connect_data_services().expect("data svc");
    let _ = dev.write_target_u32s(0x0050_cf44, &[0]);
    dev.wmi_start().and_then(|_| dev.start_receive()).expect("rx-start");

    let synth = dev.reg_read(AR_PHY_SYNTH_CONTROL).unwrap_or(0);
    let turbo = dev.reg_read(AR_PHY_TURBO).unwrap_or(0);
    let active = dev.reg_read(AR_PHY_ACTIVE).unwrap_or(0);
    println!("SYNTH_CONTROL = {synth:#010x} (HT20 ch1 was 0x30a0cccc — should differ = +10 MHz centre)");
    println!("AR_PHY_TURBO  = {turbo:#010x} — DYN2040 {}", if turbo & AR_PHY_FC_DYN2040_EN != 0 { "SET ✔" } else { "clear ✗" });
    println!("AR_PHY_ACTIVE = {active:#x} (PHY {})", if active & 1 == 1 { "up" } else { "down" });

    dev.set_rate(McsDescriptor { index: 0, short_gi: false, vht: false, nss: 1, stbc: false, ldpc: false }).ok();
    let rt = tokio::runtime::Builder::new_current_thread().enable_all().build().unwrap();
    rt.block_on(async {
        for i in 0..4 {
            let f = InjectFrame::broadcast(Bytes::copy_from_slice(&[0x05u8, 0x08, 0x40, i as u8]), TxIntent::CONSERVATIVE);
            let _ = dev.inject(f).await;
            tokio::time::sleep(Duration::from_millis(25)).await;
        }
    });
    let qtxdp = dev.reg_read(AR_QTXDP1).unwrap_or(0);
    if let Ok(desc) = dev.read_target_u32s(qtxdp, 12) {
        if desc.len() >= 10 {
            println!("ds_ctl7 = {:#010x} — AR_2040_0 (40 MHz) {}", desc[9], if desc[9] & AR_2040_0 != 0 { "SET ✔" } else { "clear ✗" });
        }
    }
    let _ = dev.detach();
    ExitCode::SUCCESS
}
