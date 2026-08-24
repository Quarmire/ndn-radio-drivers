//! **AR9271 TX hardware diagnostic — where does the injected frame actually die?**
//!
//! The firmware TX path (`ath_tgt_send_mgt`) has been read end-to-end and every input we feed it
//! (node index 0, vap 0, `keyix=0xff`, mgmt header, rate idx 0) is byte-verified correct. The frame
//! reaches the target, a `bf` is drawn from `sc_txbuf`, it is queued to `sc_txq[1]` and
//! `ah_startTxDma(qnum=1)` is called — yet after ~34 frames the pool drains and injection blocks,
//! meaning **completions never come back** ⇒ the descriptors never mark done ⇒ the hardware is not
//! transmitting. This probe reads the QCU/DCU hardware state directly (via `WMI_REG_READ`, a separate
//! reg pipe that works during injection) to localize the stall:
//!
//!   AR_Q_TXE   bit1  — queue-1 TX-enable. Set by `ar5416StartTxDma`. If it stays set with frames
//!                      pending, the DCU never won the medium (CCA/IFS/DCU-config); if it self-clears
//!                      and pending→0, the frames actually drained (transmitted).
//!   AR_QSTS(1)       — pending-frame count for queue 1 (`AR_Q_STS_PEND_FR_CNT`).
//!   AR_QTXDP(1)      — the descriptor pointer the DMA latched (0 ⇒ SetTxDP never ran).
//!   AR_ISR_S0/S1     — per-queue TXOK / TXERR interrupt status (nonzero on either ⇒ the PHY did fire).
//!   AR_DIAG_SW       — TX-hold / force-CCA diagnostic bits that would gate the DCU.
//!
//! Run on o5p-1 (AR9271), kernel driver unbound / fresh:
//! ```sh
//! sudo LD_LIBRARY_PATH=$(nix path-info nixpkgs#libusb1)/lib \
//!     ./ath9k_tx_diag ~/ath9k-fw/target_firmware/build/k2/htc_9271.fw [chan_mhz=2437]
//! ```
use std::process::ExitCode;
use std::time::Duration;

use bytes::Bytes;
use ndn_radio_drivers::{Ath9kHtcBackend, FrameIo, InjectFrame, TxIntent};

// QCU/DCU hardware registers (AR9271 reg.h; QCU block base 0x0800).
const AR_QTXDP1: u32 = 0x0800 + (1 << 2); // 0x0804 — queue-1 descriptor pointer
const AR_Q_TXE: u32 = 0x0840; // TX-enable bitmap (bit q)
const AR_Q_TXD: u32 = 0x0880; // TX-disable bitmap (bit q)
const AR_QSTS1: u32 = 0x0a00 + (1 << 2); // 0x0a04 — queue-1 status (pending count low bits)
const AR_QMISC1: u32 = 0x09c0 + (1 << 2); // 0x09c4 — queue-1 QCU misc config
const AR_ISR: u32 = 0x0080;
const AR_ISR_S0: u32 = 0x0084; // per-queue TXOK
const AR_ISR_S1: u32 = 0x0088; // per-queue TXERR/TXEOL
const AR_DIAG_SW: u32 = 0x8048;
const AR_TXCFG: u32 = 0x0030;

fn dump(dev: &mut Ath9kHtcBackend, tag: &str) {
    let r = |a: u32, dev: &mut Ath9kHtcBackend| dev.reg_read(a).unwrap_or(0xdead_beef);
    let txe = r(AR_Q_TXE, dev);
    let txd = r(AR_Q_TXD, dev);
    let qsts1 = r(AR_QSTS1, dev);
    let qtxdp1 = r(AR_QTXDP1, dev);
    let qmisc1 = r(AR_QMISC1, dev);
    let isr = r(AR_ISR, dev);
    let isr_s0 = r(AR_ISR_S0, dev);
    let isr_s1 = r(AR_ISR_S1, dev);
    let diag = r(AR_DIAG_SW, dev);
    let txcfg = r(AR_TXCFG, dev);
    println!(
        "[{tag:>10}] Q_TXE={txe:#06x}(q1={}) Q_TXD={txd:#06x} QSTS1={qsts1:#010x}(pend={}) QTXDP1={qtxdp1:#010x} QMISC1={qmisc1:#010x}",
        (txe >> 1) & 1,
        qsts1 & 0x3,
    );
    println!(
        "[{:>10}]  ISR={isr:#010x} ISR_S0(TXOK)={isr_s0:#010x} ISR_S1(TXERR)={isr_s1:#010x} DIAG_SW={diag:#010x} TXCFG={txcfg:#010x}",
        ""
    );
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        eprintln!("usage: {} <htc_9271.fw> [chan_mhz=2437]", args[0]);
        return ExitCode::FAILURE;
    }
    let fw = match std::fs::read(&args[1]) {
        Ok(f) => f,
        Err(e) => {
            eprintln!("cannot read {}: {e}", args[1]);
            return ExitCode::FAILURE;
        }
    };
    let chan_mhz: u16 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(2437);
    println!("firmware: {} ({} bytes), channel {chan_mhz} MHz", args[1], fw.len());

    let mut dev = match Ath9kHtcBackend::open() {
        Ok(d) => d,
        Err(e) => {
            eprintln!("open failed: {e}  (unbind ath9k_htc first)");
            return ExitCode::FAILURE;
        }
    };
    if let Err(e) = dev.download_firmware(&fw) {
        eprintln!("firmware download FAILED: {e}");
        return ExitCode::FAILURE;
    }
    if let Err(e) = dev.htc_init() {
        eprintln!("HTC handshake FAILED: {e}");
        return ExitCode::FAILURE;
    }
    if let Err(e) = dev.hw_reset(chan_mhz) {
        eprintln!("hw_reset FAILED: {e}");
        return ExitCode::FAILURE;
    }
    if let Err(e) = dev.connect_data_services() {
        eprintln!("connect_data_services FAILED: {e}");
        return ExitCode::FAILURE;
    }
    if let Err(e) = dev.wmi_start() {
        eprintln!("wmi_start FAILED: {e}");
        return ExitCode::FAILURE;
    }
    println!("[bring-up] complete — sampling QCU state before/through/after injection:\n");
    dump(&mut dev, "pre-inject");

    let rt = tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
        .expect("tokio");
    rt.block_on(async {
        // Inject in bursts, sampling the QCU after each, to watch the queue fill and (not) drain.
        for burst in 0..8 {
            for i in 0..6 {
                let n = burst * 6 + i;
                let payload = format!("\x05\x08ath9k-{n:03}");
                let frame = InjectFrame::broadcast(
                    Bytes::copy_from_slice(payload.as_bytes()),
                    TxIntent::CONSERVATIVE,
                );
                match dev.inject(frame).await {
                    Ok(()) => {}
                    Err(e) => {
                        eprintln!("[tx {n:03}] inject FAILED: {e}  (pool likely drained — the block)");
                        dump(&mut dev, "at-block");
                        return;
                    }
                }
            }
            // A beat for the DCU to have transmitted anything it could, then sample.
            tokio::time::sleep(Duration::from_millis(60)).await;
            dump(&mut dev, &format!("after#{}", (burst + 1) * 6));
        }
        println!("\ndone — all bursts accepted without blocking.");
    });
    dump(&mut dev, "final");
    let _ = dev.detach();
    ExitCode::SUCCESS
}
