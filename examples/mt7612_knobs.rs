//! **MT7612U knob-surface gate** — validates, in one bring-up, every knob this
//! port added to the mt76x2 backend, plus the one open question about its RX
//! descriptor.
//!
//! One run, because each cold bring-up on this part is a risk: it is the dongle
//! that has been wedged repeatedly, and the mechanism is now understood (a
//! kernel-driver teardown racing our firmware download). Run it behind
//! `scripts/mt76_acquire.sh acquire 7612`, which takes the device from the kernel
//! *first* so there is no race, and `release 7612` afterwards.
//!
//! Stages:
//!   A — bring-up + tune through the uniform `RadioKnobs::set_channel`.
//!   B — `RadioTime`: is the port TSF real, and does it run at 1 MHz?
//!   C — `read_channel_activity` (decode-busy ‰) and the energy-detect sense
//!       beside it, over several windows, against wall clock.
//!   D — `read_ofdm_counters` and the full `RxStat`.
//!   E — `set_edcca_ignore` both directions, with before/after register readback.
//!   F — the RXWI dark bytes: 16 bytes per frame that nothing in mt76 reads,
//!       dumped beside a live TSF read to settle whether they hide a receive
//!       latch.
//!
//!   sudo ./mt7612_knobs [seconds]
use ndn_radio_drivers::Mt7612uBackend;
use ndn_radio_hal::{Bandwidth, RadioKnobs, RadioProfile, RadioTime};
use std::time::{Duration, Instant};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let secs: u64 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(20);

    println!("── A. bring-up + tune ──────────────────────────────────");
    let dev = Mt7612uBackend::open()?;
    println!("   chip {:#06x}", dev.chip_id()?);
    dev.bring_up()?;
    dev.setup_monitor_rx()?;
    let t = Instant::now();
    RadioKnobs::set_channel(&dev, 6, Bandwidth::Bw20)?;
    println!(
        "   set_channel(6, Bw20) via RadioKnobs took {:?}",
        t.elapsed()
    );
    println!("   capability = {:?}", RadioProfile::capability(&dev));

    println!("── B. RadioTime: port TSF ──────────────────────────────");
    let srcs = RadioTime::time_sources(&dev);
    println!("   time_sources = {srcs:?}");
    let dom = dev.tsf_domain();
    let a = RadioTime::read_clock(&dev, dom)?;
    std::thread::sleep(Duration::from_millis(200));
    let b = RadioTime::read_clock(&dev, dom)?;
    match (a, b) {
        (Some(x), Some(y)) => {
            let d = y.wrapping_sub(x);
            println!("   TSF {x} -> {y}  (+{d} over ~200 ms; 1 MHz would give ~200000)");
            println!(
                "   => rate {:.4} MHz {}",
                d as f64 / 200_000.0,
                if (150_000..260_000).contains(&d) {
                    "✓"
                } else {
                    "✗ NOT 1 MHz"
                }
            );
        }
        _ => println!("   ✗ read_clock returned None — the timer did not arm"),
    }
    // A domain this radio does not own must answer None, not a plausible number.
    println!(
        "   foreign domain -> {:?} (must be None)",
        RadioTime::read_clock(&dev, ndn_radio_hal::ClockDomainId(0xdead_beef))?
    );

    println!("── C. channel occupancy ────────────────────────────────");
    for i in 0..5 {
        let w = Instant::now();
        std::thread::sleep(Duration::from_millis(200));
        let (ct, window_us) = dev.sample_channel_time()?;
        let busy = ndn_radio_drivers::mt76::knobs::busy_permille(&ct);
        let ed = ndn_radio_drivers::mt76::knobs::ed_cca_permille(&ct, window_us);
        let cov = ndn_radio_drivers::mt76::knobs::window_coverage_permille(&ct, window_us);
        println!(
            "   [{i}] wall {:>7}us  busy {:>7}us idle {:>7}us ed {:>7}us  => decode-busy {busy}‰, energy-busy {ed}‰, window coverage {cov}‰",
            w.elapsed().as_micros(),
            ct.busy_us,
            ct.idle_us,
            ct.ed_cca_us
        );
    }
    // ⚠ Space this from the sampling above: the counters are read-and-clear, so a call issued
    // immediately after another sees an empty window and reports 0‰ — which reads as "quiet
    // channel" and is not.
    std::thread::sleep(Duration::from_millis(200));
    println!(
        "   RadioKnobs::read_channel_activity() = {:?} (permille busy)",
        RadioKnobs::read_channel_activity(&dev)?
    );

    println!("── D. RX error counters ────────────────────────────────");
    for i in 0..3 {
        std::thread::sleep(Duration::from_millis(200));
        let st = ndn_radio_drivers::mt76::knobs::read_rx_stat(&dev)?;
        println!("   [{i}] {st:?}");
    }
    std::thread::sleep(Duration::from_millis(200));
    println!(
        "   RadioKnobs::read_ofdm_counters() = {:?}  (ok is structurally 0 — see the impl doc)",
        RadioKnobs::read_ofdm_counters(&dev)?
    );

    println!("── E. ED-CCA knob ──────────────────────────────────────");
    // edcca_state returns (ed_cca_armed, ED_CCA_MASK, CCA_MASK) — extracted fields, not the
    // raw registers. Labelling them "txop"/"ext_cca" in an earlier run made two perfectly
    // consistent readings look like a bad register map for ten minutes.
    let st0 = ndn_radio_drivers::mt76::knobs::edcca_state(&dev)?;
    println!(
        "   as found: ed_cca_armed={} ED_CCA_MASK={:#x} CCA_MASK={:#x}",
        st0.0, st0.1, st0.2
    );
    RadioKnobs::set_edcca_ignore(&dev, true)?;
    let st1 = ndn_radio_drivers::mt76::knobs::edcca_state(&dev)?;
    println!(
        "   ignore=on : ed_cca_armed={} ED_CCA_MASK={:#x} CCA_MASK={:#x}",
        st1.0, st1.1, st1.2
    );
    RadioKnobs::set_edcca_ignore(&dev, false)?;
    let st2 = ndn_radio_drivers::mt76::knobs::edcca_state(&dev)?;
    println!(
        "   restored  : ed_cca_armed={} ED_CCA_MASK={:#x} CCA_MASK={:#x}",
        st2.0, st2.1, st2.2
    );
    println!(
        "   => restore is faithful: {}",
        if st2 == st0 {
            "✓"
        } else {
            "✗ the knob did not put the part back"
        }
    );

    println!("── F. RXWI dark bytes ──────────────────────────────────");
    // `bbp_rxinfo[4]` — 16 bytes per frame that `grep -rn bbp_rxinfo` finds exactly
    // one reference to in all of mt76: the struct declaration. If a receive-latched
    // microsecond counter hides there, this part can do common view after all.
    dev.pause_drain(true); // two readers on one bulk-IN endpoint split the frames
    std::thread::sleep(Duration::from_millis(100));
    let mut buf = vec![0u8; 8192];
    let mut rows: Vec<(u64, [u32; 4])> = Vec::new();
    let start = Instant::now();
    while rows.len() < 200 && start.elapsed() < Duration::from_secs(secs) {
        let len = match dev.read_rx(&mut buf) {
            Ok(l) if l >= 40 => l,
            _ => continue,
        };
        let tsf = ndn_radio_drivers::mt76::knobs::read_tsf(&dev).unwrap_or(0);
        let w = |i: usize| u32::from_le_bytes(buf[i * 4..i * 4 + 4].try_into().unwrap());
        let dark = [w(5), w(6), w(7), w(8)];
        if rows.len() < 10 {
            println!(
                "   tsf={tsf:>12} rxinfo={:08x} ctl={:08x} rate={:04x} rssi={:02x},{:02x} | dark {:08x} {:08x} {:08x} {:08x} (len {len})",
                w(1),
                w(2),
                u16::from_le_bytes([buf[14], buf[15]]),
                buf[16],
                buf[17],
                dark[0],
                dark[1],
                dark[2],
                dark[3]
            );
        }
        rows.push((tsf, dark));
    }
    dev.pause_drain(false);
    println!("   captured {} frames in {:?}", rows.len(), start.elapsed());
    if rows.len() >= 8 {
        let tsf_span = rows.last().unwrap().0 - rows.first().unwrap().0;
        for k in 0..4 {
            let v: Vec<u32> = rows.iter().map(|(_, d)| d[k]).collect();
            let distinct = v.iter().collect::<std::collections::HashSet<_>>().len();
            let mono = v.windows(2).filter(|w| w[1] >= w[0]).count();
            let span = v
                .iter()
                .max()
                .unwrap()
                .wrapping_sub(*v.iter().min().unwrap());
            let deltas: Vec<i64> = rows.iter().map(|(t, d)| *t as i64 - d[k] as i64).collect();
            let ratio = if tsf_span > 0 {
                span as f64 / tsf_span as f64
            } else {
                0.0
            };
            let verdict = if distinct > rows.len() / 2
                && mono > rows.len() * 9 / 10
                && (0.5..2.0).contains(&ratio)
            {
                "★★ CANDIDATE RECEIVE LATCH"
            } else {
                "not a timestamp"
            };
            println!(
                "   bbp_rxinfo[{k}] (offset {}..{}): {distinct} distinct/{}, {mono} monotone, span {span} vs TSF span {tsf_span} (ratio {ratio:.3}), tsf-word in [{}, {}] => {verdict}",
                20 + k * 4,
                24 + k * 4,
                rows.len(),
                deltas.iter().min().unwrap(),
                deltas.iter().max().unwrap()
            );
        }
    } else {
        println!("   too few frames to judge — do not conclude from this");
    }
    Ok(())
}
