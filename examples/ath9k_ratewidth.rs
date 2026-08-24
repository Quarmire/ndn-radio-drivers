//! **AR9271 #6 legacy rate + #7 live HT20↔HT40 — hardware-verified on the libusb path.**
//!
//! Two knobs, each proven at the hardware register/descriptor (independent of a witness):
//!   #6  `set_legacy_rate(r)` → inject → read the live TX descriptor at `AR_QTXDP(1)`; `XmitRate0`
//!       (ds_ctl3 byte0) must equal the commanded AR5416 legacy code (no bit7 = legacy PHY).
//!   #7  `set_bandwidth(true/false)` → the returned `CalStatus` synth + `AR_PHY_TURBO` DYN2040 bit
//!       flip between the memory-verified HT20 (synth 0x30a0cccc, TURBO no-DYN2040) and HT40
//!       (synth 0x30a17777, DYN2040 set) values, with AGC re-converging each time — a real live PHY
//!       reprogram, no re-open.
//!
//! ```sh
//! sudo /tmp/ath9k_ratewidth ~/ath9k-fw/target_firmware/build/k2/htc_9271.fw
//! ```
use std::process::ExitCode;
use std::time::Duration;

use bytes::Bytes;
use ndn_radio_drivers::{Ath9kHtcBackend, FrameIo, InjectFrame, LegacyRate, TxIntent};
use ndn_radio_hal::McsDescriptor;

const AR_QTXDP1: u32 = 0x0800 + (1 << 2);
const AR_PHY_TURBO: u32 = 0x9804;
const AR_PHY_FC_DYN2040_EN: u32 = 0x00000004;

fn xmit_rate0(dev: &Ath9kHtcBackend) -> Option<u8> {
    let qtxdp = dev.reg_read(AR_QTXDP1).ok()?;
    if qtxdp == 0 {
        return None;
    }
    let desc = dev.read_target_u32s(qtxdp, 12).ok()?;
    Some((desc[5] & 0xff) as u8) // ds_ctl3 byte0 = AR_XmitRate0
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().collect();
    let fw = std::fs::read(args.get(1).expect("usage: <fw>")).expect("read fw");
    let mut dev = Ath9kHtcBackend::open().expect("open");
    dev.download_firmware(&fw).and_then(|_| dev.htc_init()).expect("transport");
    dev.hw_reset(2412).and_then(|_| dev.connect_data_services()).expect("bring-up");
    let _ = dev.write_target_u32s(0x0050_cf44, &[0]);
    dev.note_channel(1);
    dev.wmi_start().and_then(|_| dev.start_receive()).expect("rx-start");

    let rt = tokio::runtime::Builder::new_current_thread().enable_all().build().unwrap();
    let burst = |dev: &Ath9kHtcBackend, tag: &str| {
        rt.block_on(async {
            for _ in 0..6 {
                let f = InjectFrame::broadcast(
                    Bytes::copy_from_slice(format!("\x05\x08{tag}").as_bytes()),
                    TxIntent::CONSERVATIVE,
                );
                let _ = dev.inject(f).await;
                tokio::time::sleep(Duration::from_millis(15)).await;
            }
            // Let the TX queue fully drain so AR_QTXDP(1) settles on the LAST descriptor before we
            // read it — reading mid-burst catches a stale (earlier) descriptor (a read race, not a
            // knob fault: the firmware writes series[i].Rate = pad for every code identically).
            tokio::time::sleep(Duration::from_millis(90)).await;
        })
    };

    // ── #6 legacy rate selection: each code must reach the descriptor's XmitRate0 ──
    println!("#6 legacy rate → TX-descriptor XmitRate0 (expect the commanded code, bit7 clear):");
    let cases = [
        ("Cck1", LegacyRate::Cck1, 0x1b),
        ("Cck11", LegacyRate::Cck11, 0x18),
        ("Ofdm6", LegacyRate::Ofdm6, 0x0b),
        ("Ofdm54", LegacyRate::Ofdm54, 0x0c),
    ];
    let mut ok6 = 0;
    for (name, rate, want) in cases {
        dev.set_legacy_rate(rate);
        burst(&dev, name);
        match xmit_rate0(&dev) {
            Some(got) => {
                let good = got == want && got & 0x80 == 0;
                ok6 += good as u32;
                println!("  {name:<7} commanded {want:#04x} → XmitRate0 {got:#04x}  {}", if good { "✓" } else { "✗ MISMATCH" });
            }
            None => println!("  {name:<7} — no descriptor read"),
        }
    }
    // And prove HT still overrides legacy (set_rate clears the legacy override):
    dev.set_rate(McsDescriptor { index: 3, short_gi: false, vht: false, nss: 1, stbc: false, ldpc: false }).ok();
    burst(&dev, "ht3");
    let ht = xmit_rate0(&dev).unwrap_or(0);
    println!("  ht MCS3 after legacy → XmitRate0 {ht:#04x}  {}", if ht == 0x83 { "✓ (HT overrode legacy)" } else { "✗" });
    dev.clear_legacy_rate();

    // ── #7 live HT20↔HT40 switch: synth + DYN2040 must flip, AGC re-converge ──
    println!("\n#7 live bandwidth switch (no re-open):");
    let show = |tag: &str, st: &ndn_radio_drivers::CalStatus, turbo: u32| {
        println!(
            "  {tag:<6} synth={:#010x} phy_active={} agc_converged={} TURBO={:#06x} DYN2040={}",
            st.synth_control, st.phy_active, st.agc_cal_converged, turbo, (turbo & AR_PHY_FC_DYN2040_EN != 0) as u8
        );
    };
    let st40 = dev.set_bandwidth(true).expect("→HT40");
    let turbo40 = dev.reg_read(AR_PHY_TURBO).unwrap_or(0);
    show("HT40", &st40, turbo40);
    dev.set_rate(McsDescriptor { index: 0, short_gi: false, vht: false, nss: 1, stbc: false, ldpc: false }).ok();
    burst(&dev, "w40");
    let st20 = dev.set_bandwidth(false).expect("→HT20");
    let turbo20 = dev.reg_read(AR_PHY_TURBO).unwrap_or(0);
    show("HT20", &st20, turbo20);

    let width_ok = st40.synth_control != st20.synth_control
        && (turbo40 & AR_PHY_FC_DYN2040_EN != 0)
        && (turbo20 & AR_PHY_FC_DYN2040_EN == 0);
    println!(
        "\n→ #6 {}/{} legacy codes reached the descriptor; #7 live width switch {}.",
        ok6, cases.len(),
        if width_ok { "FLIPPED synth+DYN2040 (HT40↔HT20)" } else { "did NOT flip — check apply_initvals/synth" }
    );

    // ── #5 EDCCA / force-rx-clear effect on air (ch1 is management-heavy — a busy medium) ──
    // The bit (AR_DIAG_FORCE_RX_CLEAR) does not read back, so verify by EFFECT: with carrier-sense
    // deference ON (ignore=false) the MAC defers TX while the medium is busy; forcing rx-clear
    // (ignore=true) makes the medium always look idle, so a saturating injector should sustain a
    // higher rate. If the delta is in the noise, ambient load was too low — needs a controlled
    // interferer (reported honestly rather than overclaimed).
    use ndn_radio_hal::RadioKnobs;
    dev.clear_legacy_rate(); // consistent rate (MCS0, set above) across all three phases
    let occ0 = dev.read_channel_activity().ok().flatten().unwrap_or(0);
    let flood = |dev: &Ath9kHtcBackend, secs: u64| -> u64 {
        rt.block_on(async {
            let end = std::time::Instant::now() + Duration::from_secs(secs);
            let mut n = 0u64;
            while std::time::Instant::now() < end {
                let f = InjectFrame::broadcast(Bytes::copy_from_slice(b"\x05\x08edcca"), TxIntent::CONSERVATIVE);
                if dev.inject(f).await.is_ok() {
                    n += 1;
                }
            }
            n
        })
    };
    println!("\n#5 EDCCA force-rx-clear effect (ch1, occupancy sample={occ0}):");
    dev.set_edcca_ignore(false).ok();
    let defer = flood(&dev, 3);
    dev.set_edcca_ignore(true).ok();
    let force = flood(&dev, 3);
    dev.set_edcca_ignore(false).ok();
    let defer2 = flood(&dev, 3);
    println!("  cca-defer:   {} f/3s = {}/s", defer, defer / 3);
    println!("  force-clear: {} f/3s = {}/s", force, force / 3);
    println!("  cca-defer:   {} f/3s = {}/s", defer2, defer2 / 3);
    let base = defer.max(defer2).max(1);
    let delta = (force as i64 - base as i64) * 100 / base as i64;
    println!(
        "  → force-rx-clear {:+}% vs deference: {}",
        delta,
        if delta >= 5 { "EDCCA-ignore lifts TX on the busy channel (knob has effect)" }
        else if delta <= -5 { "lower (unexpected — investigate)" }
        else { "within noise — ambient ch1 load too low to defer; needs a controlled interferer" }
    );

    let _ = dev.detach();
    ExitCode::SUCCESS
}
