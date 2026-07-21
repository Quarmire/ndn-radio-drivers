//! Task #30: frame-free sensing — can the node read channel occupancy from the
//! baseband's own detectors WITHOUT decoding frames? A named-data node shouldn't
//! have to demod every packet to know the medium is busy; the PHY already counts
//! carrier-sense and false-alarm events and tracks the noise floor (IGI).
//!
//! This is also the validation: per window we read the frame-free signals (IGI +
//! the OFDM cca/FA counter deltas) AND count decoded frames (ground truth). Run on
//! a quiet channel and a busy one — the counter that tracks the decoded frame rate
//! IS the occupancy sensor, proven on-chip rather than trusted from a header. The
//! whole 0xF48–0xF54 block is dumped so a mislabelled counter is visible, not
//! hidden.
//!
//!   sudo NDN_RADIO_NO_RESET=1 ./sense_probe <ch> [window_s] [iters]
//!   # A/B: quiet vs busy
//!   sudo ... ./sense_probe 1 2 4      # quiet
//!   sudo ... ./sense_probe 6 2 4      # busy
use ndn_radio_drivers::{PhySense, Rtl8812auBackend};
use std::sync::Arc;
use std::time::{Duration, Instant};

fn bring_up(ch: u8) -> Result<Arc<Rtl8812auBackend>, Box<dyn std::error::Error>> {
    let b = Arc::new(Rtl8812auBackend::open()?);
    b.power_on()?;
    b.mac_enable_dma()?;
    b.init_llt()?;
    b.download_firmware()?;
    b.mac_config()?;
    b.mac_init_queues()?;
    b.bb_config()?;
    b.rf_config()?;
    b.set_channel(ch)?;
    b.iq_calibrate()?;
    b.lc_calibrate()?;
    b.start_rx_dma()?;
    Ok(b)
}

/// Empirically FIND the occupancy counters: read a range of PHY status registers
/// before/after a busy window and print every word that CHANGED. The chip reveals
/// which address is the CCA/FA counter — no header guessing. Reads are 32-bit over
/// the safe PHY stat pages (0xF00-0xFFC and 0xA00-0xAFC).
fn scan(b: &Arc<Rtl8812auBackend>, window: u64) -> Result<(), Box<dyn std::error::Error>> {
    b.set_rx_group_filter(None)?;
    let ranges = [(0x0F00u16, 0x0FFCu16), (0x0A00, 0x0AFC), (0x0660, 0x06A0)];
    let addrs: Vec<u16> = ranges
        .iter()
        .flat_map(|&(lo, hi)| (lo..=hi).step_by(4))
        .collect();
    let snap = |b: &Arc<Rtl8812auBackend>| -> Result<Vec<u32>, Box<dyn std::error::Error>> {
        addrs.iter().map(|&a| Ok(b.read32(a)?)).collect()
    };
    let s0 = snap(b)?;
    println!("scan: {window}s busy window over {} PHY regs…", addrs.len());
    let mut buf = vec![0u8; 16384];
    let t0 = Instant::now();
    let mut frames = 0u64;
    while t0.elapsed() < Duration::from_secs(window) {
        if let Ok(n) = b.rx_raw(&mut buf)
            && n > 0
        {
            frames += 1;
        }
    }
    let dt = t0.elapsed().as_secs_f64();
    let s1 = snap(b)?;
    println!("decoded {frames} frames ({:.0}/s). registers that changed:", frames as f64 / dt);
    for (i, &a) in addrs.iter().enumerate() {
        if s0[i] != s1[i] {
            let d = s1[i].wrapping_sub(s0[i]);
            println!(
                "  {a:#06x}: {:#010x} → {:#010x}  Δ={d} (lo16 Δ={}, hi16 Δ={})",
                s0[i],
                s1[i],
                (s1[i] & 0xffff).wrapping_sub(s0[i] & 0xffff) as u16,
                (s1[i] >> 16).wrapping_sub(s0[i] >> 16) as u16,
            );
        }
    }
    Ok(())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let a1 = std::env::args().nth(1).unwrap_or_default();
    if a1 == "scan" {
        let ch: u8 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(6);
        let window: u64 = std::env::args().nth(3).and_then(|s| s.parse().ok()).unwrap_or(3);
        let b = bring_up(ch)?;
        println!("8812AU pid={:#06x} scanning ch{ch}", b.pid());
        scan(&b, window)?;
        return Ok(());
    }
    let ch: u8 = a1.parse().ok().unwrap_or(6);
    let window: u64 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(2);
    let iters: u32 = std::env::args().nth(3).and_then(|s| s.parse().ok()).unwrap_or(4);
    let b = bring_up(ch)?;
    b.set_rx_group_filter(None)?; // promiscuous — count all frames as ground truth
    println!("8812AU pid={:#06x} sensing ch{ch}, {window}s windows × {iters}", b.pid());
    println!(
        "  {:>3} {:>4} {:>4} | {:>10} | {:>9}",
        "it", "igiA", "igiB", "activity/s", "decoded/s"
    );

    let mut buf = vec![0u8; 16384];
    for it in 0..iters {
        let s0 = b.read_phy_sense()?;
        // Count decoded frames across the same window (bulk-IN), between the two
        // register snapshots (control transfers on ep0 — different endpoint).
        let t0 = Instant::now();
        let mut frames = 0u64;
        while t0.elapsed() < Duration::from_secs(window) {
            if let Ok(n) = b.rx_raw(&mut buf)
                && n > 0
            {
                frames += 1;
            }
        }
        let dt = t0.elapsed().as_secs_f64();
        let s1 = b.read_phy_sense()?;
        let d: PhySense = s1.delta(&s0);
        println!(
            "  {it:>3} {:>4} {:>4} | {:>10.0} | {:>9.0}",
            s1.igi_a,
            s1.igi_b,
            d.rx_activity as f64 / dt,
            frames as f64 / dt,
        );
    }
    println!(
        "\n  → activity/s (REG_RXERR_RPT 0x0664) tracks decoded/s without the host\n  \
         decoding a frame to produce it; IGI is the noise-floor level."
    );
    Ok(())
}
