//! **AR9271 TX-descriptor autopsy — what did the hardware actually transmit?**
//!
//! TFCNT proves the PHY keys the air, but a witness at inches decodes nothing from us ⇒ the emitted
//! waveform is malformed. This reads the **actual TX descriptor** the target queued (at `AR_QTXDP(1)`,
//! in target RAM, via `WMI_ACCESS_MEMORY`) and decodes the fields that define the emitted frame:
//! `AR_FrameLen`/`AR_XmitPower` (ds_ctl0), `AR_FrameType` (ds_ctl1), and `AR_XmitRate0..3` (ds_ctl3 —
//! the PHY rate code). It then follows `ds_data` to the frame buffer and dumps the queued 802.11
//! bytes. A garbage rate code, zero power, wrong length, or corrupted header bytes localizes the bug.
//!
//! ar5416_desc_20 word layout: [0]=ds_link [1]=ds_data [2]=ds_ctl0 [3]=ds_ctl1 [4]=ctl2 [5]=ctl3 …
//!
//! ```sh
//! sudo /tmp/ath9k_tx_desc ~/ath9k-fw/target_firmware/build/k2/htc_9271.fw [chan_mhz=2412]
//! ```
use std::process::ExitCode;
use std::time::Duration;

use bytes::Bytes;
use ndn_radio_drivers::{Ath9kHtcBackend, FrameIo, InjectFrame, TxIntent};
use ndn_radio_hal::{McsDescriptor, RadioKnobs};

const AR_TFCNT: u32 = 0x80ec;
const AR_QTXDP1: u32 = 0x0800 + (1 << 2);

fn r(dev: &mut Ath9kHtcBackend, a: u32) -> u32 {
    dev.reg_read(a).unwrap_or(0xdead_beef)
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().collect();
    let fw = std::fs::read(args.get(1).expect("usage: <fw> [chan]")).expect("read fw");
    let chan_mhz: u16 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(2412);
    println!("firmware {} B, channel {chan_mhz} MHz", fw.len());

    let mut dev = Ath9kHtcBackend::open().expect("open");
    dev.download_firmware(&fw)
        .and_then(|_| dev.htc_init())
        .expect("transport");
    dev.hw_reset(chan_mhz)
        .and_then(|_| dev.connect_data_services())
        .and_then(|_| dev.wmi_start())
        .expect("bring-up");
    // Command DISTINCT knob values so the descriptor read proves they reach the hardware:
    // rate = HT MCS5 ⇒ XmitRate0 should read 0x85; power = idx 24 ⇒ XmitPower should read 24.
    dev.set_rate(McsDescriptor {
        index: 5,
        short_gi: false,
        vht: false,
        nss: 1,
        stbc: false,
        ldpc: false,
    })
    .ok();
    dev.set_tx_power(24).ok();
    println!(
        "commanded: MCS5 (rate code 0x85), tx_power idx 24 — expect XmitRate0=0x85, XmitPower=24"
    );

    let t0 = r(&mut dev, AR_TFCNT);
    let rt = tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
        .unwrap();
    rt.block_on(async {
        for i in 0..4 {
            let payload = format!("\x05\x08ath9k-{i:03}");
            let f = InjectFrame::broadcast(
                Bytes::copy_from_slice(payload.as_bytes()),
                TxIntent::CONSERVATIVE,
            );
            if let Err(e) = dev.inject(f).await {
                eprintln!("[tx {i}] FAILED: {e}");
                break;
            }
            tokio::time::sleep(Duration::from_millis(30)).await;
        }
    });
    let t1 = r(&mut dev, AR_TFCNT);
    println!(
        "ΔTFCNT={} (radiation {})",
        t1.wrapping_sub(t0),
        if t1.wrapping_sub(t0) > 1000 {
            "YES"
        } else {
            "no"
        }
    );

    let qtxdp = r(&mut dev, AR_QTXDP1);
    println!("\nAR_QTXDP(1) = {qtxdp:#010x}  (last-queued descriptor address in target RAM)");
    if qtxdp == 0 || qtxdp == 0xdead_beef {
        eprintln!("no descriptor pointer — nothing to autopsy");
        let _ = dev.detach();
        return ExitCode::FAILURE;
    }

    // Read the descriptor (12 words covers ds_link..ctl9).
    let desc = match dev.read_target_u32s(qtxdp, 12) {
        Ok(v) => v,
        Err(e) => {
            eprintln!("read descriptor FAILED: {e}");
            let _ = dev.detach();
            return ExitCode::FAILURE;
        }
    };
    println!("raw descriptor words:");
    for (i, w) in desc.iter().enumerate() {
        println!("  ds[{i:2}] = {w:#010x}");
    }
    let ds_data = desc[1];
    let ds_ctl0 = desc[2];
    let ds_ctl1 = desc[3];
    let ds_ctl3 = desc[5];
    let frame_len = ds_ctl0 & 0x0000_0fff;
    let xmit_power = (ds_ctl0 >> 16) & 0x3f;
    let buf_len = ds_ctl1 & 0x0000_0fff;
    let frame_type = (ds_ctl1 >> 20) & 0xf;
    let rate0 = ds_ctl3 & 0xff;
    let rate1 = (ds_ctl3 >> 8) & 0xff;
    println!("\n── decoded ──");
    println!("FrameLen   = {frame_len} B   (incl. FCS; my ~40-byte frame ⇒ expect ~44)");
    println!("BufLen     = {buf_len} B");
    println!(
        "XmitPower  = {xmit_power} (0.5dB units ⇒ {} dBm)  {}",
        xmit_power as f32 * 0.5,
        if xmit_power == 0 {
            "← ZERO POWER"
        } else {
            ""
        }
    );
    println!("FrameType  = {frame_type} (0=Normal 1=ATIM 2=PSPOLL 3=Beacon 4=Probe_Resp)");
    println!(
        "XmitRate0  = {rate0:#04x}  (1Mb=0x1b 2Mb=0x1a 5.5=0x19 11=0x18 6Mb=0x0b OFDM; HT MCS have bit7=0x80)"
    );
    println!("XmitRate1  = {rate1:#04x}");
    if rate0 & 0x80 != 0 {
        println!(
            "  ⚠ rate0 has bit7 set ⇒ HT-MCS format — a witness in legacy-only monitor won't decode it"
        );
    }

    // Follow ds_data to the frame buffer and dump the queued 802.11 bytes.
    println!("\nds_data = {ds_data:#010x}  → queued frame bytes:");
    if ds_data != 0 && ds_data != 0xdead_beef {
        match dev.read_target_u32s(ds_data, 16) {
            Ok(words) => {
                let mut bytes = Vec::new();
                for w in &words {
                    bytes.extend_from_slice(&w.to_le_bytes());
                }
                for (i, chunk) in bytes.chunks(16).enumerate() {
                    let hex: Vec<String> = chunk.iter().map(|b| format!("{b:02x}")).collect();
                    println!("  +{:02x}: {}", i * 16, hex.join(" "));
                }
                println!(
                    "  (expect FC=08 00, A1=ff ff ff ff ff ff, A2=02 4e 44 4e 00 01, … LLC aa aa 03 00 00 00 86 24)"
                );
            }
            Err(e) => eprintln!("  read frame buffer FAILED: {e}"),
        }
    }
    let _ = dev.detach();
    ExitCode::SUCCESS
}
