//! **MT7921AU TX proof + throughput ceiling.**
//!
//! Two questions in one run, because each bring-up of this part costs real risk:
//!   * **Does it radiate?** Inject a marker payload and let a separate witness radio on another
//!     bus count it. A TX path that builds a descriptor, hands it to USB and returns `Ok` proves
//!     nothing — the RTL8821c does exactly that and never puts a photon in the air.
//!   * **How fast?** Report offered frames/s and bytes/s at several payload sizes, so the
//!     per-frame overhead separates from the per-byte rate. On a USB 2.0 host the bus, not the
//!     PHY, is expected to bind — this measures which.
//!
//!   sudo ./mt7921_txflood [channel] [seconds] [payload_bytes]
//!
//! The witness is set up outside this binary (a second radio in kernel monitor on the same
//! channel, running tcpdump for the marker), so this program's own numbers are OFFERED load and
//! the witness's are DELIVERED — the difference is the on-air loss, and conflating them is how
//! "TX works" gets claimed for a radio that is silent.
use ndn_frame_io::{FrameFormat, FrameIo, InjectFrame, Reliability, TxIntent};
use ndn_radio_drivers::Mt7921uBackend;
use ndn_radio_hal::{Bandwidth, RadioKnobs};
use std::time::{Duration, Instant};

/// Appears verbatim in every injected payload so the witness can count ours and only ours.
const MARKER: &[u8] = b"NDNMT7921TXFLOOD";

fn main() -> Result<(), Box<dyn std::error::Error>> {
    tokio::runtime::Runtime::new()?.block_on(run())
}

async fn run() -> Result<(), Box<dyn std::error::Error>> {
    let mut a = std::env::args().skip(1);
    let channel: u8 = a.next().and_then(|s| s.parse().ok()).unwrap_or(6);
    // `NDN_BW=20|40|80` — the second throughput lever. At 20 MHz the per-byte rate is a quarter
    // of what this 2x2 part can do, and no MPDU size fixes that.
    let width = match std::env::var("NDN_BW").as_deref() {
        Ok("40") => Bandwidth::Bw40,
        Ok("80") => Bandwidth::Bw80,
        _ => Bandwidth::Bw20,
    };
    let secs: u64 = a.next().and_then(|s| s.parse().ok()).unwrap_or(10);

    let dev = std::sync::Arc::new(
        Mt7921uBackend::open()?.with_format(FrameFormat::RawNdn { ethertype: 0x8624 }),
    );
    println!("chip {:#06x}", dev.rr(0x7001_0200)?);
    dev.bring_up()?;
    RadioKnobs::set_channel(dev.as_ref(), channel, width)?;
    println!("width = {width:?}");
    dev.setup_monitor_rx()?;
    println!("tuned ch{channel}; rx_health: {}", dev.rx_health()?);
    // `NDN_TX_PUMP=<n>` starts the pipelined TX path with n writer threads. Off by default so
    // the two paths can be A/B'd in one session — the whole point of the sweep below is to show
    // what the per-frame USB round trip costs, and a pump that is always on hides it.
    // ★ `NDN_CWMIN=<exp>` and `NDN_TXOP=<n>` — the EDCA levers, via MCU_CE_CMD(SET_EDCA_PARMS).
    // The firmware defaults are cw_min exponent 5 (CW=31 slots => ~140 us average backoff) and
    // txop 0 (one PPDU per contention), which together are essentially the whole ~185 us fixed
    // per-PPDU cost measured at 80 MHz.
    {
        use ndn_radio_drivers::connac2::mcu::EdcaAc;
        let cwmin: u16 = std::env::var("NDN_CWMIN")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(5);
        let txop: u16 = std::env::var("NDN_TXOP")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(0);
        let aifs: u16 = std::env::var("NDN_AIFS")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(2);
        // ★ ALWAYS send, including the defaults. EDCA is device state that persists across
        // process restarts, so a run that "uses the defaults" by sending nothing actually
        // inherits whatever the previous run left — which made a cw_min A/B read 443 vs 443 and
        // very nearly turned into a reported result.
        let ac = EdcaAc {
            cw_min: cwmin,
            cw_max: 10,
            txop,
            aifs,
            guardtime: 0,
            acm: 0,
        };
        match dev.set_edca(&[ac; 4]) {
            Ok(()) => println!(
                "EDCA: cw_min exp={cwmin} (CW={} slots) txop={txop} (x32us) aifs={aifs} — sent",
                (1u32 << cwmin) - 1
            ),
            Err(e) => println!("EDCA: set failed: {e}"),
        }
    }

    let pump_depth: usize = std::env::var("NDN_TX_PUMP")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(0);
    let _tx_pump = if pump_depth > 0 {
        println!("TX pump: {pump_depth} writer threads (inject returns on queue accept)");
        Some(dev.spawn_tx_pump(pump_depth))
    } else {
        println!("TX pump: OFF (one awaited USB round trip per frame)");
        None
    };

    // `NDN_TX_STEADY=<secs>` — one fixed rate and payload for a clean witness capture, because
    // a sweep gives a radiotap listener a moving target and the question "is this PPDU actually
    // 80 MHz" needs a stationary one. The register write says 80; only the air settles it.
    if let Some(hold) = std::env::var("NDN_TX_STEADY")
        .ok()
        .and_then(|v| v.parse::<u64>().ok())
    {
        use ndn_radio_hal::McsDescriptor;
        let plen: usize = std::env::var("NDN_TX_LEN")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(7000);
        let mcs_idx: u8 = std::env::var("NDN_TX_MCS")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(8);
        let sgi = std::env::var_os("NDN_TX_SGI").is_some();
        let he = std::env::var_os("NDN_TX_HE").is_some();
        let nss: u8 = std::env::var("NDN_TX_NSS")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(2);
        let mut m = if he {
            McsDescriptor::he(mcs_idx)
        } else {
            McsDescriptor::vht(mcs_idx)
        };
        m.nss = nss;
        m.short_gi = sgi;
        FrameIo::set_rate(dev.as_ref(), m)?;
        let mut body = MARKER.to_vec();
        while body.len() < plen {
            body.push(b'#');
        }
        let payload = bytes::Bytes::from(body);
        println!(
            "steady: {} MCS{mcs_idx} {nss}SS, sgi={sgi}, {plen} B, {width:?}, {hold}s",
            if he { "HE" } else { "VHT" }
        );
        // Diagnose the MIB block before trusting it. A witness has already PROVEN that VHT MCS9
        // at 9000 B radiates (23,774 frames captured in 6 s), so if this counter reads zero for
        // that case the counter is the thing that is wrong.
        let scr1_before = dev.rr(0x820e_d004).unwrap_or(0);
        dev.enable_mib_airtime().ok();
        let scr1_after = dev.rr(0x820e_d004).unwrap_or(0);
        let busy0 = dev.rr(0x820e_d02c).unwrap_or(0) & 0x00ff_ffff;
        println!(
            "  MIB: SCR1 {scr1_before:#010x} -> {scr1_after:#010x} (want bits 8|9), SDR9 busy={busy0}"
        );
        let air0 = dev.tx_airtime_us().unwrap_or(0);
        let t = Instant::now();
        let mut sent = 0u64;
        while t.elapsed() < Duration::from_secs(hold) {
            let f = InjectFrame::broadcast(
                payload.clone(),
                TxIntent::broadcast(Reliability::Throughput),
            );
            if FrameIo::inject(dev.as_ref(), f).await.is_ok() {
                sent += 1
            }
        }
        let el = t.elapsed().as_secs_f64();
        let air = dev.tx_airtime_us().unwrap_or(0).wrapping_sub(air0) & 0x00ff_ffff;
        let fps = sent as f64 / el;
        // What the MAC says it actually spent on air, against what the offered frame rate would
        // require. A ratio near 1 means the frames radiated; near 0 means the USB accepted
        // buffers the radio never sent.
        let per_frame_air = if air > 0 && sent > 0 {
            air as f64 / sent as f64
        } else {
            0.0
        };
        println!(
            "steady: {sent} frames in {el:.1}s = {fps:.0} f/s = {:.2} Mbit/s",
            fps * plen as f64 * 8.0 / 1.0e6
        );
        println!(
            "  ★ MAC TX airtime: {air} us over {el:.1}s = {:.1}% duty, {per_frame_air:.0} us/frame \
             (a PPDU at this rate should be ~{:.0} us). \u{26a0} A zero is AMBIGUOUS: the MIB \
             counter unarmed, or nothing radiated. Check the SCR1 readback above and use a \
             witness \u{2014} do not read it as proof either way.",
            air as f64 / (el * 1.0e6) * 100.0,
            plen as f64 * 8.0 / 780.0
        );
        return Ok(());
    }

    // ── Rate sweep at a fixed payload ────────────────────────────────────────────────────
    // Three different things can bound throughput here and one measurement cannot tell them
    // apart, so vary one axis at a time:
    //   * the PHY rate (this sweep),
    //   * the per-frame USB round trip (the payload sweep below: extrapolate to zero bytes),
    //   * the bus (USB 2.0 here, ~280 Mbit/s of usable bulk).
    // `Reliability::MostRobust` pins OFDM 6 Mbps by design (`resolved_rate`), so the first run
    // of this test measured the slowest rate the radio has and nothing else.
    {
        use ndn_radio_hal::McsDescriptor;
        let payload_len = 1400usize;
        let mut body = Vec::with_capacity(payload_len);
        body.extend_from_slice(MARKER);
        while body.len() < payload_len {
            body.push(b'#');
        }
        let payload = bytes::Bytes::from(body);
        // ⚠ HT entries are only meaningful at 20/40 MHz — an HT rate word at 80 MHz is a
        // malformed PPDU, and this MAC answers it by ceasing to transmit rather than by
        // erroring. The driver clamps now, but the sweep should not ask for nonsense either.
        let wide = matches!(width, Bandwidth::Bw80);
        let rates: Vec<(&str, Option<McsDescriptor>, f64)> = if wide {
            vec![
                ("legacy OFDM 6M (MostRobust)", None, 6.0),
                ("VHT MCS4 2SS 80MHz", Some(McsDescriptor::vht_2ss(4)), 351.0),
                ("VHT MCS7 2SS 80MHz", Some(McsDescriptor::vht_2ss(7)), 585.0),
                ("VHT MCS9 2SS 80MHz", Some(McsDescriptor::vht_2ss(9)), 780.0),
            ]
        } else {
            vec![
                ("legacy OFDM 6M (MostRobust)", None, 6.0),
                ("HT MCS3 1SS", Some(McsDescriptor::ht(3)), 26.0),
                ("HT MCS7 1SS", Some(McsDescriptor::ht(7)), 65.0),
                ("VHT MCS7 2SS", Some(McsDescriptor::vht_2ss(7)), 130.0),
                ("VHT MCS8 2SS", Some(McsDescriptor::vht_2ss(8)), 156.0),
                ("HE MCS7 1SS", Some(McsDescriptor::he(7)), 68.0),
            ]
        };
        println!("\n  ── rate sweep @ {payload_len} B ──");
        for (label, mcs, phy_mbit) in rates {
            let reliability = match mcs {
                Some(m) => {
                    FrameIo::set_rate(dev.as_ref(), m)?;
                    Reliability::Throughput
                }
                None => Reliability::MostRobust,
            };
            let base = dev.tx_written().0;
            let t = Instant::now();
            let (mut sent, mut errs) = (0u64, 0u64);
            while t.elapsed() < Duration::from_secs(3) {
                let f = InjectFrame::broadcast(payload.clone(), TxIntent::broadcast(reliability));
                match FrameIo::inject(dev.as_ref(), f).await {
                    Ok(()) => sent += 1,
                    Err(_) => errs += 1,
                }
            }
            // Let the queue drain before stopping the clock, or a pumped run reports the
            // enqueue rate as if it were the transmit rate.
            if pump_depth > 0 {
                let (mut prev, mut stable) = (dev.tx_written().0, 0);
                while stable < 3 {
                    std::thread::sleep(Duration::from_millis(50));
                    let now = dev.tx_written().0;
                    if now == prev {
                        stable += 1
                    } else {
                        stable = 0
                    }
                    prev = now;
                }
            }
            let el = t.elapsed().as_secs_f64();
            let written = dev.tx_written().0;
            let sent = if pump_depth > 0 {
                written.saturating_sub(base)
            } else {
                sent
            };
            let fps = sent as f64 / el;
            let mbit = fps * payload_len as f64 * 8.0 / 1.0e6;
            // Airtime the PHY should need for the payload alone, vs the time we actually spent.
            let air_us = payload_len as f64 * 8.0 / phy_mbit;
            let spent_us = 1.0e6 / fps;
            println!(
                "  {label:<28} {fps:>7.0} f/s  {mbit:>6.2} Mbit/s   (PHY {phy_mbit:>5.0} Mbit/s => {air_us:>6.0} us airtime; we spend {spent_us:>6.0} us/frame => {:>6.0} us of non-PHY overhead) {errs} err",
                spent_us - air_us
            );
        }
        FrameIo::set_rate(dev.as_ref(), McsDescriptor::CONSERVATIVE).ok();
    }

    // ── Payload sweep at the BEST rate, not the worst ────────────────────────────────────
    // The first version of this test swept payload only at `MostRobust` (OFDM 6 Mbps), which
    // measures the amortisation curve of a rate nobody would use for bulk. The fixed per-frame
    // cost is ~220 us of medium access; the whole point of a large MPDU is to spread that over
    // more bytes, and that only shows up when the per-byte cost is small. The sibling MT7612U's
    // notes put the difference at ~142 Mbit/s at VHT80 2x2 SGI against ~37 Mbit/s at a 1500 B
    // MTU — the same 37 this test reported before, from the same mistake.
    {
        use ndn_radio_hal::McsDescriptor;
        let sweep_mcs = McsDescriptor::vht_2ss(9);
        FrameIo::set_rate(dev.as_ref(), sweep_mcs)?;
        let sweep_phy = if matches!(width, Bandwidth::Bw80) {
            780.0
        } else {
            173.0
        };
        println!("\n  ── payload sweep @ VHT MCS9 2SS {width:?} (PHY {sweep_phy:.0} Mbit/s) ──");
        for payload_len in [256usize, 512, 1024, 1500, 2048, 3000, 4096, 5650, 7000] {
            let mut body = Vec::with_capacity(payload_len);
            body.extend_from_slice(MARKER);
            body.extend_from_slice(&(payload_len as u32).to_le_bytes());
            while body.len() < payload_len {
                body.push(b'#');
            }
            let payload = bytes::Bytes::from(body);
            let base = dev.tx_written().0;
            let t = Instant::now();
            let (mut sent, mut errs) = (0u64, 0u64);
            while t.elapsed() < Duration::from_secs(3) {
                let f = InjectFrame::broadcast(
                    payload.clone(),
                    TxIntent::broadcast(Reliability::Throughput),
                );
                match FrameIo::inject(dev.as_ref(), f).await {
                    Ok(()) => sent += 1,
                    Err(_) => errs += 1,
                }
            }
            if pump_depth > 0 {
                let (mut prev, mut stable) = (dev.tx_written().0, 0);
                while stable < 3 {
                    std::thread::sleep(Duration::from_millis(50));
                    let now = dev.tx_written().0;
                    if now == prev {
                        stable += 1
                    } else {
                        stable = 0
                    }
                    prev = now;
                }
                sent = dev.tx_written().0.saturating_sub(base);
            }
            let el = t.elapsed().as_secs_f64();
            let fps = sent as f64 / el;
            let mbit = fps * payload_len as f64 * 8.0 / 1.0e6;
            let air_us = payload_len as f64 * 8.0 / sweep_phy;
            let spent_us = if fps > 0.0 { 1.0e6 / fps } else { 0.0 };
            println!(
                "  payload {payload_len:>5} B: {fps:>7.0} f/s  {mbit:>7.2} Mbit/s   ({air_us:>6.0} us airtime, {spent_us:>6.0} us/frame => {:>6.0} us fixed) {errs} err",
                spent_us - air_us
            );
            // A payload the radio silently refuses shows up as errors or as a frame count that
            // collapses; say so rather than letting a zero row look like a slow row.
            if errs > 0 && sent == 0 {
                println!("      ^ every inject failed at this size — this is the MPDU ceiling");
                break;
            }
        }
        FrameIo::set_rate(dev.as_ref(), McsDescriptor::CONSERVATIVE).ok();
    }

    println!(
        "\nmarker = {:?} — count it at the witness to get DELIVERED, not offered.",
        std::str::from_utf8(MARKER)?
    );
    Ok(())
}
