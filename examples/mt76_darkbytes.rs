//! **The RXWI dark-byte hunt** — does an mt76x0/mt76x2 RX descriptor carry a
//! per-frame hardware timestamp after all?
//!
//! `struct mt76x02_rxwi` (mt76x02_mac.h:97) is 32 bytes: `rxinfo`(4) `ctl`(4)
//! `tid_sn`(2) `rate`(2) `rssi[4]` and then **`bbp_rxinfo[4]` — 16 bytes that
//! nothing in the entire mt76 tree ever reads.** `grep -rn bbp_rxinfo` returns
//! exactly one hit: the struct declaration. So the "mt76 has no per-frame RX
//! stamp" conclusion rests on those 16 bytes being uninteresting, which nobody
//! has ever checked.
//!
//! This checks it. For each received frame it prints the 36-byte RX prefix as
//! nine little-endian u32 words beside a **live TSF read** (`MT_TSF_TIMER_DW0`,
//! 0x111c — the real one), so a receive-latched microsecond counter would show up
//! as a word that tracks, and stays just below, the register.
//!
//! ⚠ The 2026-08-18 attempt at exactly this experiment compared the prefix
//! against `0x1104`, which is `MT_BKOFF_SLOT_CFG` — a constant. It read
//! `0x00000114` every time (a slot time of 20 µs with cc_delay 1). Nothing could
//! have correlated with it, so that run was incapable of finding a timestamp
//! whether or not one exists. This run enables the timer first and reads the
//! right register.
//!
//! Interpretation, decided before the data (so the analysis cannot drift):
//!   * a word advancing monotonically at ~1 MHz and sitting **below** the live TSF
//!     by a small, roughly constant amount = a receive-latched stamp. mt76x0/x2
//!     become common-view-capable and the wall is wrong.
//!   * words that are static, or that vary with signal strength rather than with
//!     time = BBP diagnostics, and the wall is confirmed **with evidence**.
//!
//!   sudo ./mt76_darkbytes [n_frames]
use ndn_radio_drivers::Mt7612uBackend;
use std::time::Instant;

const MT_BEACON_TIME_CFG: u32 = 0x1114;
const MT_TSF_TIMER_DW0: u32 = 0x111c;
const MT_TSF_TIMER_DW1: u32 = 0x1120;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let n_want: usize = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(120);

    let dev = Mt7612uBackend::open()?;
    println!("chip {:#06x}", dev.chip_id()?);
    dev.bring_up()?;
    dev.setup_monitor_rx()?;

    // Enable the free-running TSF: TIMER_EN (bit16) on, SYNC_MODE (bits 18:17)
    // cleared so a received beacon cannot slam the counter mid-experiment.
    let saved = dev.rr(MT_BEACON_TIME_CFG)?;
    dev.wr(MT_BEACON_TIME_CFG, (saved & !0x0006_0000) | 0x0001_0000)?;
    let t0 = dev.rr(MT_TSF_TIMER_DW0)?;
    std::thread::sleep(std::time::Duration::from_millis(50));
    let t1 = dev.rr(MT_TSF_TIMER_DW0)?;
    println!(
        "MT_BEACON_TIME_CFG {saved:#010x} -> {:#010x}; TSF advanced {} over ~50 ms \
         (expect ~50000 at 1 MHz)",
        dev.rr(MT_BEACON_TIME_CFG)?,
        t1.wrapping_sub(t0)
    );
    if t1.wrapping_sub(t0) < 20_000 {
        println!("TSF is not running — the rest of this experiment is meaningless. Stopping.");
        dev.wr(MT_BEACON_TIME_CFG, saved)?;
        return Ok(());
    }

    // The tune. The kernel leaves the RF wherever it last was; without a known
    // tune a silent zero-frame result is ambiguous between "no timestamp" and
    // "no reception", which is the ambiguity that makes RF work expensive.
    if std::env::args().any(|a| a == "--5g") {
        dev.set_channel_5g80()?;
    } else {
        dev.set_channel_ch6()?;
    }

    // ⚠ `bring_up` spawns a background bulk-IN drain thread on the SAME endpoint
    // we are about to read. Two readers on one endpoint split the frames — the
    // first run of this experiment saw 4 frames in 30 s on a busy 2.4 GHz channel
    // for exactly that reason, which is the classic dual-reader trap this
    // codebase keeps re-learning. Pause it and own the endpoint.
    dev.pause_drain(true);
    std::thread::sleep(std::time::Duration::from_millis(100));

    println!(
        "\n  n |  live TSF  | rxinfo    ctl       tid_sn/rate rssi      | bbp_rxinfo[0..3] (the dark bytes)"
    );
    let mut buf = vec![0u8; 8192];
    let mut rows: Vec<(u32, [u32; 4])> = Vec::new();
    let start = Instant::now();
    let mut n = 0usize;
    while n < n_want && start.elapsed().as_secs() < 30 {
        let len = match dev.read_rx(&mut buf) {
            Ok(l) if l >= 40 => l,
            _ => continue,
        };
        let tsf = dev.rr(MT_TSF_TIMER_DW0).unwrap_or(0);
        let w = |i: usize| u32::from_le_bytes(buf[i * 4..i * 4 + 4].try_into().unwrap());
        let dark = [w(5), w(6), w(7), w(8)];
        n += 1;
        if n <= 24 {
            println!(
                "  {n:>3} | {tsf:#010x} | {:08x} {:08x} {:08x} {:08x} | {:08x} {:08x} {:08x} {:08x}   (len {len})",
                w(1),
                w(2),
                w(3),
                w(4),
                dark[0],
                dark[1],
                dark[2],
                dark[3]
            );
        }
        rows.push((tsf, dark));
    }
    dev.pause_drain(false);
    dev.wr(MT_BEACON_TIME_CFG, saved)?;
    println!("\ncaptured {} frames in {:?}", rows.len(), start.elapsed());
    if rows.len() < 8 {
        println!("too few frames to judge — retune or move closer, do not conclude from this");
        return Ok(());
    }

    // ── The verdict, computed rather than eyeballed ──────────────────────────
    // A receive-latched microsecond counter must (a) be monotone across frames in
    // arrival order, (b) advance at roughly the same rate as the live TSF, and
    // (c) sit at a small, stable offset below the concurrent TSF read (the read
    // happens ~90-150 us AFTER the latch, so the delta is positive and bounded).
    for k in 0..4 {
        let vals: Vec<u32> = rows.iter().map(|(_, d)| d[k]).collect();
        let tsfs: Vec<u32> = rows.iter().map(|(t, _)| *t).collect();
        let distinct = vals.iter().collect::<std::collections::HashSet<_>>().len();
        let monotone = vals.windows(2).filter(|w| w[1] >= w[0]).count();
        let span_v = vals
            .iter()
            .max()
            .unwrap()
            .wrapping_sub(*vals.iter().min().unwrap());
        let span_t = tsfs.last().unwrap().wrapping_sub(*tsfs.first().unwrap());
        let deltas: Vec<i64> = rows
            .iter()
            .map(|(t, d)| (*t as i64) - (d[k] as i64))
            .collect();
        let dmin = deltas.iter().min().unwrap();
        let dmax = deltas.iter().max().unwrap();
        println!(
            "  bbp_rxinfo[{k}] (offset {}..{}): {distinct} distinct / {} frames, {monotone} monotone steps, \
             span {span_v} vs TSF span {span_t}, TSF-minus-word in [{dmin}, {dmax}]",
            20 + k * 4,
            24 + k * 4,
            rows.len()
        );
        let ratio = if span_t > 0 {
            span_v as f64 / span_t as f64
        } else {
            0.0
        };
        if distinct > rows.len() / 2
            && monotone > rows.len() * 9 / 10
            && (0.5..2.0).contains(&ratio)
        {
            println!(
                "     ★★ CANDIDATE TIMESTAMP: monotone, ~1 MHz rate (ratio {ratio:.2}). \
                 If TSF-minus-word is also a tight positive band, this is a receive latch."
            );
        }
    }
    println!(
        "\nIf no word qualified, the wall stands — and now it stands on a measurement of the \
         right registers rather than on the absence of one."
    );
    Ok(())
}
