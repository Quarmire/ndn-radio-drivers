//! Task #37: does honoring EDCCA (energy-detect carrier sense) cut our losses on
//! a CONTENDED channel? #36 proved the "severe loss at 3 ft" was channel
//! contention (ch14 quiet → 40/40; ch6 busy → 4–11/40), not hardware. EDCCA is
//! the direct lever: make the MAC TX engine defer until the medium falls below an
//! energy threshold instead of blasting into ongoing transmissions.
//!
//! Real hardware, apples-to-apples. Each phase sends a FIXED COUNT of frames, so
//! delivery *ratio* (received / sent) is comparable even though the honored phase
//! takes longer wall-time (that extra time IS the CSMA deferral — the mechanism
//! showing itself). The payload carries a phase byte so the receiver buckets both
//! arms from one promiscuous capture, no clock coordination needed.
//!
//!   phase 0 — EDCCA off  : blast regardless (the pre-#37 default)
//!   phase 1 — EDCCA honor: defer to energy above L2H before keying up
//!
//! Run on the BUSY channel (ch6). If ratio(phase1) > ratio(phase0), contention
//! collision was the loss source and carrier-sense fixes it. If they match,
//! either CS already dominated or the loss is RX-side desense (not TX-fixable) —
//! report honestly either way.
//!
//!   # receiver (o5p-1), promiscuous, whole window:
//!   sudo NDN_RADIO_NO_RESET=1 LD_LIBRARY_PATH=$(nix path-info nixpkgs#libusb1)/lib \
//!       EDCCA_CH=6 ./edcca_probe rx 40
//!   # transmitter (o5p-0): 500 frames/phase, 300/s pace, L2H = -11:
//!   sudo ... EDCCA_CH=6 ./edcca_probe tx 500 300 -11
use ndn_radio_drivers::Rtl8812auBackend;
use std::sync::Arc;
use std::time::{Duration, Instant};

const MAGIC: &[u8; 4] = b"EDAB";
const FRAME_LEN: usize = 200;
const DESC_RATE_6M: u32 = 0x04;
const BCAST: [u8; 6] = [0xff; 6];

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
    b.set_tx_power(0x3f)?;
    b.start_rx_dma()?;
    println!("8812AU pid={:#06x} up on ch{ch}", b.pid());
    Ok(b)
}

/// Build a broadcast data frame carrying the phase marker. Broadcast goes through
/// the same DIFS+backoff+CCA medium access as unicast but never ACKs/retries, so
/// each frame is exactly one on-air transmission — delivery == what survived the
/// air, with no retransmit confound.
fn frame(phase: u8, seq: u16) -> Vec<u8> {
    let mut f = Vec::with_capacity(FRAME_LEN);
    f.extend_from_slice(&[0x08, 0x00, 0x00, 0x00]); // data frame, FC+dur
    f.extend_from_slice(&BCAST); // addr1
    f.extend_from_slice(&[0x02, 0x4e, 0x44, 0x4e, 0x00, 0x37]); // addr2 (this node)
    f.extend_from_slice(&BCAST); // addr3
    f.extend_from_slice(&(seq << 4).to_le_bytes()); // seq
    f.extend_from_slice(MAGIC);
    f.push(phase);
    f.resize(FRAME_LEN, 0x5a);
    f
}

/// Sweep of EDCCA busy-enter thresholds, increasingly sensitive (more negative =
/// senses weaker energy as busy = should defer more). Phase 0 is the off control;
/// each later phase honors EDCCA at PHASES[p]. If TX wall-time never rises across
/// the sweep, the injection TX path does NOT honor the gate (a real negative
/// finding); if it does rise, the gate works and we read off the delivery ratio.
/// Fine sweep across the cliff between "gate does nothing" (L2H≈-11) and "gate
/// starves TX" (L2H≈-20). A send that times out means the medium never fell
/// below the threshold — the frame couldn't get on air = a deferral-drop, which
/// we COUNT (not abort on) so one stall doesn't kill the sweep.
const PHASES: &[Option<i8>] = &[None, Some(-17), Some(-18), Some(-19), Some(-20)];
/// Per-phase wall budget — a fully-stalled phase (every send timing out at 500ms)
/// would otherwise take n×0.5s. Cap it so the sweep stays bounded.
const PHASE_BUDGET: Duration = Duration::from_secs(5);

fn tx(b: &Arc<Rtl8812auBackend>, n: u32, rate: u64) -> Result<(), Box<dyn std::error::Error>> {
    let period = Duration::from_secs_f64(1.0 / rate as f64);
    for (phase, setpoint) in PHASES.iter().enumerate() {
        match setpoint {
            None => b.disable_edcca()?,
            Some(l2h) => b.enable_edcca(*l2h)?,
        }
        let (rl2h, rh2l, honored) = b.edcca_state()?;
        println!(
            "\nphase {phase} [{}]: readback L2H={rl2h} H2L={rh2l} honored={honored}  up to {n} @ {rate}/s (≤{}s)…",
            setpoint.map_or("EDCCA off".into(), |v| format!("honor L2H={v}")),
            PHASE_BUDGET.as_secs()
        );
        let t0 = Instant::now();
        let mut next = t0;
        let (mut ok, mut stalled) = (0u32, 0u32);
        for i in 0..n {
            if t0.elapsed() > PHASE_BUDGET {
                break;
            }
            // A timeout here = TX FIFO didn't drain = medium held busy by EDCCA.
            // Count it as a deferral-drop and keep going.
            match b.send_frame(&frame(phase as u8, i as u16), DESC_RATE_6M) {
                Ok(()) => ok += 1,
                Err(_) => stalled += 1,
            }
            next += period;
            if let Some(d) = next.checked_duration_since(Instant::now()) {
                std::thread::sleep(d);
            }
        }
        let wall = t0.elapsed().as_secs_f64();
        println!(
            "phase {phase}: sent-ok={ok} deferral-stalls={stalled} in {wall:.2}s  \
             (on-air ok {:.0}/s){}",
            ok as f64 / wall,
            if stalled > 0 { "  ← gate is holding TX off" } else { "" }
        );
        std::thread::sleep(Duration::from_millis(500)); // phase gap
    }
    b.disable_edcca()?; // leave the promiscuous-injection default
    Ok(())
}

fn rx(b: &Arc<Rtl8812auBackend>, secs: u64) -> Result<(), Box<dyn std::error::Error>> {
    b.set_rx_group_filter(None)?; // promiscuous — bucket all phases from the marker
    println!("rx: promiscuous, bucketing by phase for {secs}s…");
    let t0 = Instant::now();
    let mut recv = [0u64; 8];
    let mut buf = vec![0u8; 16384];
    while t0.elapsed() < Duration::from_secs(secs) {
        if let Ok(n) = b.rx_raw(&mut buf)
            && n > 0
            && let Some(pos) = buf[..n].windows(4).position(|w| w == MAGIC)
            && let Some(&p) = buf.get(pos + 4)
            && (p as usize) < recv.len()
        {
            recv[p as usize] += 1;
        }
    }
    println!("\n  received per phase (0=off control, then honored sweep):");
    for (p, c) in recv.iter().enumerate() {
        if *c > 0 {
            println!("    phase{p}: {c}");
        }
    }
    println!("  → divide by the TX's frames-per-phase for delivery ratio; a honored\n  \
              phase above phase0 ⇒ carrier-sense cut contention loss.");
    Ok(())
}

/// Scan ambient load across 2.4 GHz channels on ONE bring-up (hop + count all
/// frames per channel). Finds a mid-load channel — the regime where EDCCA's
/// cliff should become a usable knob (ch6 ≈saturated, ch14 ≈quiet).
fn amb(b: &Arc<Rtl8812auBackend>, chans: &[u8], secs: u64) -> Result<(), Box<dyn std::error::Error>> {
    b.set_rx_group_filter(None)?; // promiscuous — count everything on air
    println!("ambient scan: {secs}s per channel");
    let mut buf = vec![0u8; 16384];
    for &c in chans {
        b.set_channel(c)?;
        std::thread::sleep(Duration::from_millis(150)); // let the synth settle
        let t0 = Instant::now();
        let mut frames = 0u64;
        while t0.elapsed() < Duration::from_secs(secs) {
            if let Ok(n) = b.rx_raw(&mut buf)
                && n > 0
            {
                frames += 1;
            }
        }
        let rate = frames as f64 / t0.elapsed().as_secs_f64();
        let tag = if rate < 5.0 {
            "quiet"
        } else if rate < 60.0 {
            "MID-LOAD ← EDCCA operating point"
        } else {
            "saturated"
        };
        println!("  ch{c:>2}: {frames} frames  {rate:>5.0}/s   {tag}");
    }
    Ok(())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mode = std::env::args().nth(1).unwrap_or_else(|| "rx".into());
    let ch: u8 = std::env::var("EDCCA_CH").ok().and_then(|s| s.parse().ok()).unwrap_or(6);
    // Ambient scan brings up once on ch1 then hops; do it before the ch-specific arms.
    if mode == "amb" {
        let secs: u64 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(4);
        let chans: Vec<u8> = std::env::args()
            .nth(3)
            .map(|s| s.split(',').filter_map(|x| x.parse().ok()).collect())
            .unwrap_or_else(|| vec![1, 3, 4, 6, 8, 9, 11, 13, 14]);
        let b = bring_up(chans[0])?;
        amb(&b, &chans, secs)?;
        return Ok(());
    }
    let b = bring_up(ch)?;
    match mode.as_str() {
        "tx" => {
            let n: u32 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(400);
            let rate: u64 = std::env::args().nth(3).and_then(|s| s.parse().ok()).unwrap_or(300);
            tx(&b, n, rate)?;
        }
        "rx" => {
            let secs: u64 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(40);
            rx(&b, secs)?;
        }
        _ => {
            eprintln!("edcca_probe <tx N RATE L2H | rx SECS>   (env EDCCA_CH, default 6)");
            std::process::exit(2);
        }
    }
    Ok(())
}
