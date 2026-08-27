//! **AR9271 PHY-level TX proof — does the transmitter actually key the air?**
//!
//! The QCU registers say TXOK/TXDESC/TXEOL, but a witness receiver at inches captured **zero**
//! frames from us (503 environment frames, none ours). So "descriptor completed OK" ≠ "RF radiated".
//! This probe reads the MAC **cycle-profile counters** — `AR_TFCNT` counts clocks the *transmitter
//! is active* — before and after a burst. If `AR_TFCNT` jumps by a frame-sized amount per injection,
//! the PHY is keying (and the problem is power/witness); if it stays flat while descriptors complete,
//! the PHY never keys despite TXOK (a TX-enable / power-table / analog gate). It also snapshots the
//! synth (channel), PHY-active, TX-power-per-rate and antenna registers for context.
//!
//! Run on o5p-1 (AR9271), fresh cold device, kernel ath9k_htc unloaded:
//! ```sh
//! sudo /tmp/ath9k_tx_phy ~/ath9k-fw/target_firmware/build/k2/htc_9271.fw [chan_mhz=2412]
//! ```
use std::process::ExitCode;
use std::time::Duration;

use bytes::Bytes;
use ndn_radio_drivers::{Ath9kHtcBackend, FrameIo, InjectFrame, TxIntent};

// MAC cycle-profile counters (reg.h) — free-running clock counters.
const AR_RFCNT: u32 = 0x80e8; // RX-frame active cycles
const AR_TFCNT: u32 = 0x80ec; // TX-frame active cycles  ← the PHY-TX witness
const AR_RCCNT: u32 = 0x80f0; // RX-clear (medium busy) cycles
const AR_CCCNT: u32 = 0x80f4; // total cycles
// Static context registers.
const AR_DEF_ANTENNA: u32 = 0x8058;
const AR_PHY_SYNTH_CONTROL: u32 = 0x9874;
const AR_PHY_ACTIVE: u32 = 0x981c;
const AR_PHY_POWER_TX_RATE1: u32 = 0x9934;
const AR_PHY_POWER_TX_RATE2: u32 = 0x9938;
const AR_PHY_POWER_TX_RATE_MAX: u32 = 0xa3c0;
const AR_PHY_TX_PWRCTRL4: u32 = 0xa274; // openloop pwr ctrl (pd_avg / tx_gain_forced)
const AR_PHY_TX_PWRCTRL6_0: u32 = 0xa27c;

fn r(dev: &mut Ath9kHtcBackend, a: u32) -> u32 {
    dev.reg_read(a).unwrap_or(0xdead_beef)
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        eprintln!("usage: {} <htc_9271.fw> [chan_mhz=2412]", args[0]);
        return ExitCode::FAILURE;
    }
    let fw = std::fs::read(&args[1]).expect("read fw");
    let chan_mhz: u16 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(2412);
    println!("firmware {} B, channel {chan_mhz} MHz", fw.len());

    let mut dev = match Ath9kHtcBackend::open() {
        Ok(d) => d,
        Err(e) => {
            eprintln!("open failed: {e}");
            return ExitCode::FAILURE;
        }
    };
    if let Err(e) = dev.download_firmware(&fw).and_then(|_| dev.htc_init()) {
        eprintln!("transport bring-up FAILED: {e}");
        return ExitCode::FAILURE;
    }
    if let Err(e) = dev
        .hw_reset(chan_mhz)
        .and_then(|_| dev.connect_data_services())
        .and_then(|_| dev.wmi_start())
    {
        eprintln!("bring-up FAILED: {e}");
        return ExitCode::FAILURE;
    }

    println!("\n=== static TX context ===");
    println!(
        "SYNTH_CONTROL={:#010x}  PHY_ACTIVE={:#010x}  DEF_ANTENNA={:#010x}",
        r(&mut dev, AR_PHY_SYNTH_CONTROL),
        r(&mut dev, AR_PHY_ACTIVE),
        r(&mut dev, AR_DEF_ANTENNA),
    );
    println!(
        "PWR_TX_RATE1={:#010x}  PWR_TX_RATE2={:#010x}  PWR_TX_RATE_MAX={:#010x}",
        r(&mut dev, AR_PHY_POWER_TX_RATE1),
        r(&mut dev, AR_PHY_POWER_TX_RATE2),
        r(&mut dev, AR_PHY_POWER_TX_RATE_MAX),
    );
    println!(
        "TX_PWRCTRL4={:#010x}  TX_PWRCTRL6_0={:#010x}",
        r(&mut dev, AR_PHY_TX_PWRCTRL4),
        r(&mut dev, AR_PHY_TX_PWRCTRL6_0),
    );

    let snap = |dev: &mut Ath9kHtcBackend| {
        (
            r(dev, AR_TFCNT),
            r(dev, AR_RFCNT),
            r(dev, AR_RCCNT),
            r(dev, AR_CCCNT),
        )
    };
    let (t0, rf0, rc0, cc0) = snap(&mut dev);
    println!("\n=== cycle counters BEFORE inject ===");
    println!("TFCNT={t0} RFCNT={rf0} RCCNT={rc0} CCCNT={cc0}");

    let rt = tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
        .expect("tokio");
    rt.block_on(async {
        // Stay under the ~34 recycle depth so the pool block can't mask PHY behaviour.
        for i in 0..20 {
            let payload = format!("\x05\x08ath9k-{i:03}");
            let frame = InjectFrame::broadcast(
                Bytes::copy_from_slice(payload.as_bytes()),
                TxIntent::CONSERVATIVE,
            );
            if let Err(e) = dev.inject(frame).await {
                eprintln!("[tx {i:03}] inject FAILED: {e}");
                break;
            }
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
    });

    let (t1, rf1, rc1, cc1) = snap(&mut dev);
    println!("\n=== cycle counters AFTER 20 injects ===");
    println!("TFCNT={t1} RFCNT={rf1} RCCNT={rc1} CCCNT={cc1}");
    println!(
        "\nΔTFCNT={}  ΔRFCNT={}  ΔRCCNT={}  ΔCCCNT={}",
        t1.wrapping_sub(t0),
        rf1.wrapping_sub(rf0),
        rc1.wrapping_sub(rc0),
        cc1.wrapping_sub(cc0),
    );
    let dt = t1.wrapping_sub(t0);
    if dt > 1000 {
        println!(
            "→ TFCNT jumped: the PHY IS keying the transmitter. On-air absence ⇒ power/witness."
        );
    } else {
        println!("→ TFCNT ~flat while descriptors complete: the PHY is NOT keying despite TXOK.");
    }
    let _ = dev.detach();
    ExitCode::SUCCESS
}
