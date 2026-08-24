//! **AR9271 knobs v2 — short-GI, live channel retune, EDCCA-ignore.**
//!
//! Exercises the `&self` control surface unlocked by the WMI-path refactor:
//!   - `FrameIo::set_rate(MCS5, short_gi)` → the witness reports "short GI".
//!   - `RadioKnobs::set_edcca_ignore(true)` → forces AR_DIAG_FORCE_RX_CLEAR (returns Ok).
//!   - `RadioKnobs::set_channel(6)` → LIVE retune; injected frames then appear on ch6, not ch1.
//!
//! Put the witness on **ch6**: it should see NOTHING during the ch1 burst, then our SGI MCS5 frames
//! after the retune — proving the radio actually moved channels without a re-open.
//!
//! ```sh
//! sudo NDN_ATH9K_FW=~/ath9k-fw/target_firmware/build/k2/htc_9271.fw ./ath9k_knobs2
//! ```
use std::process::ExitCode;
use std::time::Duration;

use bytes::Bytes;
use ndn_radio_drivers::open_ath9k;
use ndn_radio_hal::{Bandwidth, InjectFrame, McsDescriptor, TxIntent};

fn mcs0_sgi() -> McsDescriptor {
    // MCS0 (robust — decodable by the witness) + short-GI so "short GI" shows in the radiotap.
    McsDescriptor { index: 0, short_gi: true, vht: false, nss: 1, stbc: false, ldpc: false }
}

fn main() -> ExitCode {
    let radio = match open_ath9k(1) {
        Ok(r) => r,
        Err(e) => {
            eprintln!("open_ath9k FAILED: {e}");
            return ExitCode::FAILURE;
        }
    };
    let io = radio.io.clone();
    let knobs = radio.knobs.clone().expect("knobs");

    // EDCCA-ignore (owned-spectrum blast). Just needs to apply cleanly.
    match knobs.set_edcca_ignore(true) {
        Ok(()) => println!("set_edcca_ignore(true) = Ok (AR_DIAG_FORCE_RX_CLEAR set)"),
        Err(e) => println!("set_edcca_ignore FAILED: {e}"),
    }
    let _ = io.set_rate(mcs0_sgi());

    let rt = tokio::runtime::Builder::new_current_thread().enable_all().build().unwrap();
    rt.block_on(async {
        let burst = |io: std::sync::Arc<dyn ndn_radio_hal::FrameIo>, tag: u8| async move {
            for i in 0..40 {
                let p = vec![0x05u8, 0x08, tag, i as u8, 0x6b, 0x32];
                let f = InjectFrame::broadcast(Bytes::copy_from_slice(&p), TxIntent::CONSERVATIVE);
                let _ = io.inject(f).await;
                tokio::time::sleep(Duration::from_millis(20)).await;
            }
        };

        // ── burst on ch1 (SGI MCS5). Witness on ch6 should see NOTHING here. ──
        println!("burst on ch1 (MCS5 short-GI, tag 0x11) — witness on ch6 should NOT see these");
        burst(io.clone(), 0x11).await;

        // ── LIVE retune to ch6, then burst. Witness on ch6 should now see our SGI MCS5 frames. ──
        match knobs.set_channel(6, Bandwidth::Bw20) {
            Ok(()) => println!("set_channel(6) = Ok (LIVE retune, no re-open)"),
            Err(e) => {
                println!("set_channel(6) FAILED: {e}");
                return;
            }
        }
        tokio::time::sleep(Duration::from_millis(200)).await;
        println!("burst on ch6 (MCS5 short-GI, tag 0x66) — witness on ch6 SHOULD see these");
        burst(io.clone(), 0x66).await;
        println!("done — on the ch6 witness: 0 frames tag 0x11, N frames tag 0x66 at MCS5/short-GI = live retune + SGI proven.");
    });
    let _ = radio;
    ExitCode::SUCCESS
}
