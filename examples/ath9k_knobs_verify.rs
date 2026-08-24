//! **AR9271 knobs — register/descriptor-level verification (no witness decodability confounds).**
//!
//! Proves the `&self` control surface actuates at the hardware level:
//!   - SGI   → command MCS0+short-GI, inject, read the TX descriptor's `ds_ctl7` AR_GI0 bit (0x2).
//!   - EDCCA → `set_edcca_ignore(true)`, read `AR_DIAG_SW` and check AR_DIAG_FORCE_RX_CLEAR (0x200).
//!   - retune→ read `AR_PHY_SYNTH_CONTROL` before/after `set_channel(6)`: the synth word must change.
//!
//! ```sh
//! sudo /tmp/ath9k_knobs_verify ~/ath9k-fw/target_firmware/build/k2/htc_9271.fw
//! ```
use std::process::ExitCode;
use std::time::Duration;

use bytes::Bytes;
use ndn_radio_drivers::Ath9kHtcBackend;
use ndn_radio_hal::{Bandwidth, FrameIo, InjectFrame, McsDescriptor, RadioKnobs, TxIntent};

const AR_PHY_SYNTH_CONTROL: u32 = 0x9874;
const AR_DIAG_SW: u32 = 0x8048;
const AR_DIAG_FORCE_RX_CLEAR: u32 = 0x0000_0200;
const AR_QTXDP1: u32 = 0x0800 + (1 << 2);
const AR_GI0: u32 = 0x0000_0002; // ds_ctl7 short-GI bit

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().collect();
    let fw = std::fs::read(args.get(1).expect("usage: <fw>")).expect("read fw");
    let mut dev = Ath9kHtcBackend::open().expect("open");
    dev.download_firmware(&fw).and_then(|_| dev.htc_init()).expect("transport");
    dev.hw_reset(2412).and_then(|_| dev.connect_data_services()).expect("bring-up");
    let _ = dev.write_target_u32s(0x0050_cf44, &[0]);
    dev.wmi_start().and_then(|_| dev.start_receive()).expect("rx-start");

    let synth_ch1 = dev.reg_read(AR_PHY_SYNTH_CONTROL).unwrap_or(0);
    println!("SYNTH_CONTROL @ ch1 = {synth_ch1:#010x}");

    // ── EDCCA ──
    dev.set_edcca_ignore(true).expect("set_edcca_ignore");
    let diag = dev.reg_read(AR_DIAG_SW).unwrap_or(0);
    println!(
        "EDCCA: AR_DIAG_SW = {diag:#010x} — FORCE_RX_CLEAR {}",
        if diag & AR_DIAG_FORCE_RX_CLEAR != 0 { "SET ✔" } else { "clear" }
    );

    // ── SGI: command MCS0 + short-GI, inject, read the descriptor's GI bit ──
    dev.set_rate(McsDescriptor { index: 0, short_gi: true, vht: false, nss: 1, stbc: false, ldpc: false })
        .ok();
    let rt = tokio::runtime::Builder::new_current_thread().enable_all().build().unwrap();
    rt.block_on(async {
        for i in 0..4 {
            let p = vec![0x05u8, 0x08, 0x53, i as u8];
            let f = InjectFrame::broadcast(Bytes::copy_from_slice(&p), TxIntent::CONSERVATIVE);
            let _ = dev.inject(f).await;
            tokio::time::sleep(Duration::from_millis(25)).await;
        }
    });
    let qtxdp = dev.reg_read(AR_QTXDP1).unwrap_or(0);
    match dev.read_target_u32s(qtxdp, 12) {
        Ok(desc) if desc.len() >= 10 => {
            let ctl7 = desc[9];
            println!(
                "SGI: ds_ctl7 = {ctl7:#010x} — AR_GI0 (short-GI) {}",
                if ctl7 & AR_GI0 != 0 { "SET ✔" } else { "clear ✗" }
            );
        }
        _ => println!("SGI: could not read descriptor at {qtxdp:#010x}"),
    }

    // ── live retune to ch6 ──
    dev.set_channel(6, Bandwidth::Bw20).expect("set_channel(6)");
    let synth_ch6 = dev.reg_read(AR_PHY_SYNTH_CONTROL).unwrap_or(0);
    println!(
        "retune: SYNTH_CONTROL @ ch6 = {synth_ch6:#010x} — {}",
        if synth_ch6 != synth_ch1 { "CHANGED ✔ (synth retuned)" } else { "unchanged ✗" }
    );

    let _ = dev.detach();
    ExitCode::SUCCESS
}
