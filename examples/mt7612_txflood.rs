//! **MT7612U throughput ladder — the same measurement as `mt7921_txflood`, on SuperSpeed.**
//!
//! The MT7921AU's 174 Mbit/s turned out to be its USB 2.0 bus, not its radio: MB/s flattened at
//! ~22 regardless of payload, and 1 vs 2 spatial streams made no difference. This part is the
//! control for that claim — a 2x2 VHT80 mt76 radio on a **5000 Mbps** bus. If the ceiling here
//! lands well past 22 MB/s, the bus diagnosis holds and the MT7921AU is simply in the wrong
//! socket; if it lands at the same place, the diagnosis was wrong and something in this driver
//! family is the limit.
//!
//!   sudo ./mt7612_txflood [seconds]
//! env: NDN_TX_PUMP=<n> writer threads, NDN_TX_LEN=<bytes>, NDN_TX_MCS=<idx>, NDN_TX_NSS=<1|2>,
//!      NDN_TX_SGI=1
use ndn_frame_io::{FrameFormat, FrameIo, InjectFrame, Reliability, TxIntent};
use ndn_radio_drivers::Mt7612uBackend;
use ndn_radio_hal::McsDescriptor;
use std::sync::Arc;
use std::time::{Duration, Instant};

const MARKER: &[u8] = b"NDNMT7612TXFLOOD";

fn main() -> Result<(), Box<dyn std::error::Error>> {
    tokio::runtime::Runtime::new()?.block_on(run())
}

async fn run() -> Result<(), Box<dyn std::error::Error>> {
    let hold: u64 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(6);

    let dev =
        Arc::new(Mt7612uBackend::open()?.with_format(FrameFormat::RawNdn { ethertype: 0x8624 }));
    println!("chip {:#06x}", dev.chip_id()?);
    dev.bring_up()?;
    // ★ `NDN_NO_RX=1` — pause the background bulk-IN drain and skip enabling RX.
    //
    // There is a recorded 50x precedent for this exact confound on this bench: an 8812au TX
    // flood collapsed to ~250 f/s not from carrier sense but from RX-pump USB contention, and
    // skipping the pump took it to 12913 f/s. `bring_up` spawns a drain thread that reads
    // bulk-IN in a tight loop forever, and `setup_monitor_rx` turns the RX engine on — both
    // compete with the TX bulk-out for the same device. Before blaming the MAC for a fixed
    // ~290 us/frame, rule the host's own receiver out.
    let no_rx = std::env::var_os("NDN_NO_RX").is_some();
    if no_rx {
        dev.pause_drain(true);
        println!("RX: drain paused, monitor RX NOT enabled");
    } else {
        dev.setup_monitor_rx()?;
    }
    // ch36 @ 80 MHz VHT, both chains — the captured 5 GHz program. `set_channel_5g80` is a delta
    // on ch6 state, so the ch6 tune has to happen first (start_high_throughput does both).
    dev.set_channel_ch6()?;
    dev.set_channel_5g80()?;
    dev.set_tx_chains(true)?;
    println!("tuned ch36 @ 80 MHz, 2 chains");

    // ★ `NDN_EDCA=wmm|edca|both|off` — the experiment that settles which EDCA block this MAC
    // actually arbitrates from. `set_edca_aggressive` writes MT_WMM_* (0x0214/18/1c) and has
    // NEVER been called on hardware; `init_replay` separately programs MT_EDCA_CFG_AC(n)
    // (0x1300 + 4n) with 0x000a4200. Two candidate blocks, one of which may be dead — and a
    // measured ~290 us/frame fixed cost on a quiet 5 GHz channel that airtime cannot explain.
    if std::env::var_os("NDN_EDCA_RESTORE").is_some() {
        dev.restore_edca_defaults()?;
        println!("EDCA: restored init defaults (WMM 0x2222/0x4444/0xaaaa, AC(n) 0x000a4200)");
    }
    match std::env::var("NDN_EDCA").as_deref() {
        Ok("wmm") | Ok("both") => {
            dev.set_edca_aggressive()?;
            println!("EDCA: MT_WMM_* set aggressive (AIFSN=1, CW=0)");
        }
        _ => {}
    }
    if matches!(
        std::env::var("NDN_EDCA").as_deref(),
        Ok("edca") | Ok("both")
    ) {
        // Layout is TXOP[7:0] AIFSN[11:8] CWMIN[15:12] CWMAX[19:16] (mt76x02_regs.h:380-386),
        // and the init value 0x000a4200 is AIFSN=2, CWMIN exponent 4 (15 slots), CWMAX exponent
        // 10. Keep TXOP, set AIFSN=1 and both CW exponents to 0 (no backoff). Per-AC, all four.
        for ac in 0..4u32 {
            let before = dev.rr(0x1300 + (ac << 2))?;
            let after = (before & 0x0000_00ff) | (1 << 8);
            dev.wr(0x1300 + (ac << 2), after)?;
            if ac == 0 {
                println!("EDCA: MT_EDCA_CFG_AC(0) {before:#010x} -> {after:#010x}");
            }
        }
    }

    // ★ `NDN_POSTURE=owned|shared|yielding` — the shipped contention knob.
    //
    // On the mt76x2 this drives the SLOT TIME only: the EDCA window stays exactly as booted,
    // because lowering it killed this part twice (see `mt76::knobs::window_floor`). The budget is
    // `SIFS 16 + AIFSN 2 x slot + E[CW 15]/2 x slot` = 206 us at slot 20, 101 us at slot 9, and
    // the MEASURED saving of 105.5 us matches that to within half a microsecond.
    if let Ok(v) = std::env::var("NDN_POSTURE") {
        use ndn_radio_hal::{ContentionPosture, RadioKnobs};
        let posture = match v.trim().to_ascii_lowercase().as_str() {
            "owned" => ContentionPosture::Owned,
            "yielding" => ContentionPosture::Yielding,
            _ => ContentionPosture::Shared,
        };
        let a = RadioKnobs::set_contention(dev.as_ref(), posture)?;
        println!(
            "contention: {posture:?} -> cw {}..{} aifsn {} slot {} us => backoff {} us, \
             medium access {} us",
            a.cw_min,
            a.cw_max,
            a.aifs,
            a.slot_us,
            a.avg_backoff_us,
            a.medium_access_us()
        );
    }

    // ★ `NDN_SLOT_US=<n>` — the MAC slot time (MT_BKOFF_SLOT_CFG[7:0]). This part boots at 20;
    // 9 is the ordinary 802.11a short slot. Every DCF term is counted in slots, so the predicted
    // slope is (AIFSN + E[CW] + CC_DELAY) = 2 + 7.5 + 1 = 10.5 us of period per us of slot.
    // Always read back: an MCU that re-asserts MAC timing during calibration would otherwise be
    // indistinguishable from a dead mechanism.
    if let Some(slot) = std::env::var("NDN_SLOT_US")
        .ok()
        .and_then(|v| v.parse::<u8>().ok())
    {
        let before = dev.slot_time()?;
        let got = dev.set_slot_time(slot)?;
        println!(
            "slot: {before} us -> asked {slot}, reads back {got} us (ACKTO {:#06x})",
            dev.rr(0x1348)? & 0xffff
        );
    } else {
        println!("slot: {} us (as booted)", dev.slot_time()?);
    }

    // ★ `NDN_TXOP=<n>` — the EDCA TXOP limit, field [7:0] of MT_EDCA_CFG_AC(n), which the
    // init leaves at **0** for AC0. A TXOP limit is what lets the MAC send several PPDUs after
    // one medium acquisition instead of contending per frame; with it at zero, every single
    // MPDU pays a full contention. Units are 32 us (mt76x02_regs.h:380-386).
    if let Some(txop) = std::env::var("NDN_TXOP")
        .ok()
        .and_then(|v| v.parse::<u32>().ok())
    {
        for ac in 0..4u32 {
            let before = dev.rr(0x1300 + (ac << 2))?;
            let after = (before & !0xff) | (txop & 0xff);
            dev.wr(0x1300 + (ac << 2), after)?;
            if ac == 0 {
                println!("TXOP: MT_EDCA_CFG_AC(0) {before:#010x} -> {after:#010x} ({txop} x 32us)");
            }
        }
    }

    let pump_depth: usize = std::env::var("NDN_TX_PUMP")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(8);
    let _pump = (pump_depth > 0).then(|| dev.spawn_tx_pump(pump_depth));
    println!("TX pump: {pump_depth} threads");

    let mcs_idx: u8 = std::env::var("NDN_TX_MCS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(9);
    let nss: u8 = std::env::var("NDN_TX_NSS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(2);
    let sgi = std::env::var_os("NDN_TX_SGI").is_some();
    let mut m = McsDescriptor::vht(mcs_idx);
    m.nss = nss;
    m.short_gi = sgi;
    FrameIo::set_rate(dev.as_ref(), m)?;

    // VHT MCS9 2SS 80 MHz: 780 Mbit/s long-GI, 866.7 short-GI.
    let phy_mbit = if sgi { 866.7 } else { 780.0 } * if nss == 2 { 1.0 } else { 0.5 };
    println!("rate: VHT MCS{mcs_idx} {nss}SS sgi={sgi} => PHY {phy_mbit:.0} Mbit/s\n");

    let sizes: Vec<usize> = match std::env::var("NDN_TX_LEN")
        .ok()
        .and_then(|v| v.parse().ok())
    {
        Some(n) => vec![n],
        None => vec![1400, 2048, 3000, 4096, 5000, 5650],
    };
    for plen in sizes {
        let mut body = MARKER.to_vec();
        while body.len() < plen {
            body.push(b'#');
        }
        let payload = bytes::Bytes::from(body);
        let base = dev.tx_count_written();
        let t = Instant::now();
        let mut offered = 0u64;
        while t.elapsed() < Duration::from_secs(hold) {
            let f = InjectFrame::broadcast(
                payload.clone(),
                TxIntent::broadcast(Reliability::Throughput),
            );
            if FrameIo::inject(dev.as_ref(), f).await.is_ok() {
                offered += 1;
            }
        }
        // Let the bounded queue drain so the figure is what reached USB, not what was enqueued.
        if pump_depth > 0 {
            let (mut prev, mut stable) = (dev.tx_count_written(), 0);
            while stable < 3 {
                std::thread::sleep(Duration::from_millis(50));
                let now = dev.tx_count_written();
                if now == prev {
                    stable += 1
                } else {
                    stable = 0
                }
                prev = now;
            }
        }
        let el = t.elapsed().as_secs_f64();
        let written = if pump_depth > 0 {
            dev.tx_count_written().saturating_sub(base)
        } else {
            offered
        };
        let fps = written as f64 / el;
        let mbit = fps * plen as f64 * 8.0 / 1.0e6;
        // ★ Airtime is preamble + data, not data alone. A VHT PPDU spends L-STF 8 + L-LTF 8 +
        // L-SIG 4 + VHT-SIG-A 8 + VHT-STF 4 + VHT-LTF 4*nss + VHT-SIG-B 4 us before the first
        // data symbol. Omitting it inflated every "fixed cost" printed here by ~44 us, which was
        // enough to hide that the fixed cost is flat in MPDU size -- the signature of DCF.
        let preamble_us = 36.0 + 4.0 * nss as f64;
        let air_us = preamble_us + plen as f64 * 8.0 / phy_mbit;
        let spent_us = if fps > 0.0 { 1.0e6 / fps } else { 0.0 };
        println!(
            "  {plen:>5} B: {fps:>7.0} f/s  {mbit:>7.2} Mbit/s = {:>5.1} MB/s   ({air_us:>5.0} us airtime, {spent_us:>5.0} us/frame => {:>5.0} us fixed)",
            mbit / 8.0,
            spent_us - air_us
        );
    }
    println!("\n★ compare against the MT7921AU on USB 2.0: it flattened at 22 MB/s.");
    Ok(())
}
