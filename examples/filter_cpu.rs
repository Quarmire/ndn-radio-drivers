//! Task #43: does a name-group receive filter make host CPU track WANTED traffic
//! instead of ambient — i.e. is the "named-data node must process everything" tax a
//! monitor-mode artifact, not the architecture?
//!
//! Real hardware, not a proxy. The 8812au's RX config (RCR) is either promiscuous
//! (AAP set — our monitor default, "process everything") or restricted to frames
//! whose addr1 matches the name-group MAC (AAP cleared + APM exact-match via
//! `set_rx_group_filter`). In the filtered arm the CHIP drops non-matching frames —
//! they never traverse USB, so the host never spends a cycle on them. That is the
//! name-group hash acting as the multicast-address filter every NIC already has.
//!
//! The transmitter injects at a fixed total rate with a controllable fraction
//! addressed to the receiver's name-group; the rest go to a different group MAC.
//! Sweep the match fraction and read the receiver's CPU:
//!   ARM A (all):    CPU should track TOTAL delivered rate, flat in match-fraction.
//!   ARM B (filter): CPU should track MATCHING delivered rate, falling with it.
//! If both land on one CPU-per-delivered-frame line, the per-frame cost is identical
//! and the only difference is how many frames reach the host — exactly MAC filtering.
//!
//!   # receiver, promiscuous:
//!   sudo NDN_RADIO_NO_RESET=1 LD_LIBRARY_PATH=$(nix path-info nixpkgs#libusb1)/lib ./filter_cpu rx all 20
//!   # receiver, chip-filtered on the name-group:
//!   sudo ... ./filter_cpu rx filter 20
//!   # transmitter: 800 frames/s total, 10% to the name-group, 18 s
//!   sudo ... ./filter_cpu tx 800 10 18
use ndn_radio_drivers::Rtl8812auBackend;
use std::sync::Arc;
use std::time::{Duration, Instant};

/// The receiver's name-group MAC (addr1 for a "matching" frame). Locally-administered
/// UNICAST (bit1 of byte0 set, multicast bit clear) so the chip's APM exact-match is
/// unambiguous. This stands in for the compiled name-group hash.
const GROUP: [u8; 6] = [0x02, 0x4e, 0x44, 0x4e, 0x11, 0x11];
/// A different group — "not for me" traffic the filter should drop.
const OTHER: [u8; 6] = [0x02, 0x4e, 0x44, 0x4e, 0x22, 0x22];
const MAGIC: &[u8; 4] = b"FCPU";
const FRAME_LEN: usize = 200;
const DESC_RATE_6M: u32 = 0x04;

/// Process-wide CPU seconds (all threads) from /proc/self/stat utime+stime.
fn cpu_secs() -> f64 {
    let s = std::fs::read_to_string("/proc/self/stat").unwrap_or_default();
    // fields after "comm" (which may contain spaces) — split on the last ')'.
    let rest = s.rsplit_once(')').map(|(_, r)| r).unwrap_or(&s);
    let f: Vec<&str> = rest.split_whitespace().collect();
    // stat field 14 = utime, 15 = stime; here index 11 and 12 (state is [0]).
    let utime: u64 = f.get(11).and_then(|x| x.parse().ok()).unwrap_or(0);
    let stime: u64 = f.get(12).and_then(|x| x.parse().ok()).unwrap_or(0);
    // _SC_CLK_TCK is 100 on these OPis (standard). CPU seconds = ticks / 100.
    (utime + stime) as f64 / 100.0
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mode = std::env::args().nth(1).unwrap_or_else(|| "rx".into());
    let ch: u8 = std::env::var("FC_CH").ok().and_then(|s| s.parse().ok()).unwrap_or(14);

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

    if mode == "tx" {
        let rate: u64 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(800);
        let match_pct: u64 = std::env::args().nth(3).and_then(|s| s.parse().ok()).unwrap_or(10);
        let secs: u64 = std::env::args().nth(4).and_then(|s| s.parse().ok()).unwrap_or(18);
        let period = Duration::from_secs_f64(1.0 / rate as f64);
        println!("tx: {rate}/s total, {match_pct}% to the name-group, {secs}s");
        let t0 = Instant::now();
        let mut next = t0;
        let mut i: u64 = 0;
        while t0.elapsed() < Duration::from_secs(secs) {
            // Deterministic interleave (no RNG in this env): every k-th frame
            // matches, where k = 100/match_pct, so exactly match_pct% are for us.
            let is_match = match_pct > 0 && (i * match_pct) % 100 < match_pct;
            let dst = if is_match { GROUP } else { OTHER };
            let mut f = Vec::with_capacity(FRAME_LEN);
            f.extend_from_slice(&[0x08, 0x00, 0x00, 0x00]); // data frame, FC+dur
            f.extend_from_slice(&dst); // addr1 — what the chip filters on
            f.extend_from_slice(&[0x02, 0x4e, 0x44, 0x4e, 0x00, 0x99]); // addr2
            f.extend_from_slice(&dst); // addr3
            f.extend_from_slice(&((i as u16) << 4).to_le_bytes()); // seq
            f.extend_from_slice(MAGIC);
            f.push(is_match as u8);
            f.resize(FRAME_LEN, 0x5a);
            b.send_frame(&f, DESC_RATE_6M)?;
            i += 1;
            next += period;
            if let Some(d) = next.checked_duration_since(Instant::now()) {
                std::thread::sleep(d);
            }
        }
        println!("tx: sent {i} frames in {:.1}s ({:.0}/s)", t0.elapsed().as_secs_f64(), i as f64 / t0.elapsed().as_secs_f64());
        return Ok(());
    }

    // rx: "all" = promiscuous (process everything); "filter" = chip filters on GROUP.
    let sub = std::env::args().nth(2).unwrap_or_else(|| "all".into());
    let secs: u64 = std::env::args().nth(3).and_then(|s| s.parse().ok()).unwrap_or(20);
    match sub.as_str() {
        "all" => b.set_rx_group_filter(None)?,
        "filter" => b.set_rx_group_filter(Some(GROUP))?,
        _ => {
            eprintln!("rx <all|filter> <secs>");
            std::process::exit(2);
        }
    }
    println!("rx [{sub}]: measuring {secs}s (chip filter {})…", if sub == "filter" { "ON" } else { "off" });

    // ONE reader via blocking rx_raw (NOT the pump — two readers on one bulk-IN
    // steal transfers and read as 50% loss). rx_raw blocks in the kernel up to
    // 200ms then returns Ok(0), so an idle receiver spends ~no CPU and a busy one
    // spends CPU in proportion to the frames the CHIP hands up — which is the whole
    // point: with the filter on, non-matching frames never reach this loop.
    std::thread::sleep(Duration::from_millis(500)); // let bring-up CPU settle out
    let cpu0 = cpu_secs();
    let t0 = Instant::now();
    let (mut total, mut matching) = (0u64, 0u64);
    let mut buf = vec![0u8; 16384];
    while t0.elapsed() < Duration::from_secs(secs) {
        if let Ok(n) = b.rx_raw(&mut buf)
            && n > 0
        {
            let f = &buf[..n];
            if let Some(pos) = f.windows(4).position(|w| w == MAGIC) {
                total += 1;
                // The userspace name-group check — the "software filter" work Arm A
                // pays on every frame and Arm B pays on almost none (chip pre-dropped).
                if f.get(pos + 4) == Some(&1u8) {
                    matching += 1;
                }
            }
        }
    }
    let wall = t0.elapsed().as_secs_f64();
    let cpu = cpu_secs() - cpu0;
    println!(
        "\n  arm={sub}  wall={wall:.1}s\n  delivered total={total} ({:.0}/s)  matching={matching} ({:.0}/s)\n  \
         CPU={cpu:.3}s  => {:.1}% of one core  |  {:.1} us/delivered-frame",
        total as f64 / wall,
        matching as f64 / wall,
        cpu / wall * 100.0,
        if total > 0 { cpu / total as f64 * 1e6 } else { 0.0 },
    );
    Ok(())
}
