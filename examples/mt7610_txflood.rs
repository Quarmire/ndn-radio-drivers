//! **MT7610U throughput ladder + the A-MPDU probe.**
//!
//! The question this exists to answer: our monitor-inject path tops out at ~3000 PPDU/s with a
//! fixed ~290 µs per frame that is not airtime, not host USB dispatch, not DCF backoff, and not
//! per-queue serialisation — all four measured and eliminated. At 3000 PPDU/s, 400 Mbit/s needs
//! 16.7 kB per frame against an 11454 B MPDU limit, so one MPDU per PPDU cannot get there at any
//! rate or width. Several MPDUs must share a PPDU. That is A-MPDU, and A-MPDU needs a WCID.
//!
//! This part is 1x1, so it cannot reach 400 Mbit/s itself — but the WCID and TXWI registers are
//! the SHARED mt76x02 ones, so whether aggregation engages here transfers directly to the 2x2
//! MT7612U, and it is the only mt76 part currently healthy on the bench.
//!
//!   sudo ./mt7610_txflood [channel] [seconds]
//! env: NDN_BW=20|40|80, NDN_TX_MCS=<0-9>, NDN_TX_LEN=<bytes>, NDN_TX_QUEUES=<1-4>,
//!      NDN_TX_PUMP=<n> pipelined writer threads (0 = old synchronous path),
//!      NDN_TX_VHT=1, NDN_TX_SGI=1, NDN_AMPDU=<ba_window>  (unset = broadcast, no aggregation)
//!
//! ⚠ Every knob here is read from the ENVIRONMENT, and `sudo` strips those. Run it as
//! `export NDN_BW=80 ...; sudo -E ./mt7610_txflood 36 4` — `NDN_BW=80 sudo ./mt7610_txflood`
//! silently runs at defaults and quietly turns an A/B into two copies of the control arm.
use ndn_frame_io::{FrameFormat, FrameIo, InjectFrame, Reliability, TxIntent};
use ndn_radio_drivers::Mt7610uBackend;
use ndn_radio_hal::{Bandwidth, McsDescriptor, RadioKnobs};
use std::sync::Arc;
use std::time::{Duration, Instant};

/// The address programmed into the probe's WCID entry. Locally administered, never a host MAC —
/// it is an aggregation *context*, not an identity (mac-addressing-doctrine).
const PEER: [u8; 6] = [0x02, 0x4e, 0x44, 0x4e, 0x00, 0x02];

fn main() -> Result<(), Box<dyn std::error::Error>> {
    tokio::runtime::Runtime::new()?.block_on(run())
}

async fn run() -> Result<(), Box<dyn std::error::Error>> {
    let mut a = std::env::args().skip(1);
    let channel: u8 = a.next().and_then(|s| s.parse().ok()).unwrap_or(36);
    let secs: u64 = a.next().and_then(|s| s.parse().ok()).unwrap_or(6);
    let width = match std::env::var("NDN_BW").as_deref() {
        Ok("40") => Bandwidth::Bw40,
        Ok("80") => Bandwidth::Bw80,
        _ => Bandwidth::Bw20,
    };
    let mcs: u8 = std::env::var("NDN_TX_MCS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(7);
    let plen: usize = std::env::var("NDN_TX_LEN")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(1400);
    let ampdu = std::env::var("NDN_AMPDU")
        .ok()
        .and_then(|v| v.parse::<u8>().ok());

    let dev =
        Arc::new(Mt7610uBackend::open()?.with_format(FrameFormat::RawNdn { ethertype: 0x8624 }));
    dev.bring_up()?;
    dev.setup_monitor_rx()?;
    RadioKnobs::set_channel(dev.as_ref(), channel, width)?;
    // ★ `NDN_TX_VHT=1` — this part is 802.11**ac**, 1x1. Testing it at HT MCS7 measures the
    // 11n ceiling of an 11ac radio: VHT MCS8 at 20 MHz is 86.7 Mbit/s against HT MCS7's 72.2,
    // and the earlier "30 Mbit/s" figure was HT MCS7 with a 1400 B payload — the worst corner
    // of both axes, exactly the mistake already made once on the MT7921AU.
    let vht = std::env::var_os("NDN_TX_VHT").is_some();
    let sgi = std::env::var_os("NDN_TX_SGI").is_some();
    let mut m = if vht {
        McsDescriptor::vht(mcs)
    } else {
        McsDescriptor::ht(mcs)
    };
    m.nss = 1; // 1x1 silicon; a 2-stream rate word here would be a lie
    m.short_gi = sgi;
    FrameIo::set_rate(dev.as_ref(), m)?;
    println!(
        "ch{channel} {width:?} {} MCS{mcs} 1SS sgi={sgi}, {plen} B payload",
        if vht { "VHT" } else { "HT" }
    );

    // ★ `NDN_POSTURE=owned|shared|yielding` — the contention knob under test.
    //
    // Deliberately validated on THIS part rather than the MT7612U: the mt76x02 register path is
    // the one that wedged a radio when written with a zero window, and this is the part that
    // recovers from a wedge by handing it back to the kernel driver. The MT7612U needs a
    // physical replug, so it is the wrong place to learn something.
    if let Ok(v) = std::env::var("NDN_POSTURE") {
        use ndn_radio_hal::ContentionPosture;
        let posture = match v.trim().to_ascii_lowercase().as_str() {
            "owned" => ContentionPosture::Owned,
            "yielding" => ContentionPosture::Yielding,
            _ => ContentionPosture::Shared,
        };
        match RadioKnobs::set_contention(dev.as_ref(), posture) {
            Ok(a) => println!(
                "contention: {posture:?} -> cw_min exp {} (CW {} slots), cw_max {}, aifs {}, \
                 ~{} us average backoff",
                a.cw_min,
                (1u32 << a.cw_min) - 1,
                a.cw_max,
                a.aifs,
                a.avg_backoff_us
            ),
            Err(e) => println!("contention: {posture:?} not applied: {e}"),
        }
    }

    match ampdu {
        Some(win) => {
            dev.enable_ampdu(1, PEER, win)?;
            println!(
                "A-MPDU: WCID 1 = {PEER:02x?}, ba_window={win}, TXWI FLAGS_AMPDU set, unicast dst"
            );
        }
        None => println!("A-MPDU: off (broadcast, wcid 0xff)"),
    }

    // ★ `NDN_TX_PUMP=<n>` — pipelined TX. MEASURED 2026-08-31: the synchronous one-bulk-at-a-time
    // path costs ~305 us per PPDU *independent of payload and channel width* (64 B at VHT MCS9 /
    // 80 MHz is ~1 us of airtime and still took 305 us/frame), capping the part near 3000 PPDU/s.
    // That width-independent floor is precisely why 20/40/80 MHz measured identically at 1400 B.
    // 0 disables the pump and restores the old synchronous path for an A/B.
    let pump_depth: usize = std::env::var("NDN_TX_PUMP")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(8);
    let _pump = (pump_depth > 0).then(|| dev.spawn_tx_pump(pump_depth));
    println!(
        "TX pump: {}",
        if pump_depth > 0 {
            format!("{pump_depth} threads (pipelined)")
        } else {
            "off (synchronous write_bulk per frame)".to_string()
        }
    );

    // Payload sweep unless pinned: the per-frame cost is ~305 us and only a large MPDU amortises
    // it. `NDN_TX_LEN` pins a single size.
    let sizes: Vec<usize> = if std::env::var_os("NDN_TX_LEN").is_some() {
        vec![plen]
    } else {
        vec![1400, 2048, 3000, 4096, 5650, 7000]
    };
    for plen in sizes {
        let mut body = b"NDNMT7610FLOOD".to_vec();
        while body.len() < plen {
            body.push(b'#');
        }
        let payload = bytes::Bytes::from(body);
        let base = dev.tx_count_written();
        let t = Instant::now();
        let (mut sent, mut errs) = (0u64, 0u64);
        while t.elapsed() < Duration::from_secs(secs) {
            let mut f = InjectFrame::broadcast(
                payload.clone(),
                TxIntent::broadcast(Reliability::Throughput),
            );
            if ampdu.is_some() {
                // Aggregation is defined over a station relationship; addr1 must be the WCID's MAC.
                f.dst = PEER;
            }
            match FrameIo::inject(dev.as_ref(), f).await {
                Ok(()) => sent += 1,
                Err(_) => errs += 1,
            }
        }
        // ⚠ With a pump, `sent` counts ENQUEUES. Let the bounded queue drain so the figure is
        // what actually reached USB — the MT7921AU once reported a throughput that was really
        // the speed of a channel send while the radio transmitted for minutes afterwards.
        if pump_depth > 0 {
            let (mut prev, mut stable) = (dev.tx_count_written(), 0);
            while stable < 3 {
                std::thread::sleep(Duration::from_millis(20));
                let now = dev.tx_count_written();
                if now == prev {
                    stable += 1;
                } else {
                    stable = 0;
                    prev = now;
                }
            }
            sent = dev.tx_count_written().saturating_sub(base);
        }
        let el = t.elapsed().as_secs_f64();
        let fps = sent as f64 / el;
        println!(
            "  {plen:>5} B: {sent:>7} frames in {el:.1}s = {fps:>6.0} f/s = {:>6.2} Mbit/s ({errs} err), {:.0} us/frame",
            fps * plen as f64 * 8.0 / 1.0e6,
            1.0e6 / fps.max(1.0)
        );
        if errs > 0 && sent == 0 {
            println!("      ^ every inject failed — MPDU ceiling");
            break;
        }
    }
    Ok(())
}
