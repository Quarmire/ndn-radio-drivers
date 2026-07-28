//! #75 network-wide µs sync — a 3-node line A→B→C validated on air. A (reference) emits a
//! belief-carrying hardware-TSF beacon; B (relay) receives A and re-emits its own composed belief; C
//! (leaf) receives B — **filtered** to B's BSSID so it does not compose off A directly (a logical line
//! on a co-located bench; physical range isn't separable here) — and composes a 2-hop offset, tracking
//! the reference A to ~2×0.5 µs. Uses the shipping ndn_time::NetworkTime + the driver mesh_common_view
//! (88xx for A/B, 8812au for the Alfa leaf) + the belief-carrying beacon.
//!
//!   sudo NDN_ROLE=reference NDN_PID=a81a NDN_NODE_ID=1 NDN_BSSID=aa           ./netsync_airtest 40 40  # o5p-0 a81a
//!   sudo NDN_ROLE=relay     NDN_PID=a81a NDN_NODE_ID=2 NDN_BSSID=bb NDN_PARENT=aa ./netsync_airtest 40 40  # o5p-1 a81a
//!   sudo NDN_ROLE=leaf                   NDN_NODE_ID=3            NDN_PARENT=bb ./netsync_airtest 40 40  # o5p-0 Alfa 8812au

use std::sync::Arc;
use std::time::{Duration, Instant};

use ndn_radio_drivers::{LibUsbRtl88xxBackend, Rtl8812auBackend};
use ndn_radio_hal::FrameIo;
use ndn_time::{NetworkTime, RefBelief};

fn bssid(tag_hex: &str) -> [u8; 6] {
    let t = u8::from_str_radix(tag_hex.trim_start_matches("0x"), 16).unwrap_or(0xaa);
    [0x02, 0x4e, 0x44, 0x4e, t, 0x01] // locally-administered (mesh)
}

/// 802.11 beacon carrying `belief` at body offset 8 (frame[32..49]); timestamp at body[0..8] HW-filled.
fn beacon(my: [u8; 6], belief: RefBelief) -> Vec<u8> {
    let mut f = Vec::with_capacity(64);
    f.extend_from_slice(&[0x80, 0x00, 0x00, 0x00]);
    f.extend_from_slice(&[0xff; 6]);
    f.extend_from_slice(&my);
    f.extend_from_slice(&my);
    f.extend_from_slice(&[0x00, 0x00]); // seq
    f.extend_from_slice(&[0u8; 8]); // Timestamp — HW fills
    f.extend_from_slice(&belief.to_beacon_bytes()); // #75 belief @ frame[32..49]
    f.extend_from_slice(&[0x00, 0x04, b'N', b'D', b'N', b'T']);
    f.extend_from_slice(&[0x01, 0x01, 0x8b]);
    f
}

fn env(k: &str) -> Option<String> {
    std::env::var(k).ok()
}

#[tokio::main(flavor = "multi_thread", worker_threads = 2)]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(40);
    let secs: u64 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(40);
    let role = env("NDN_ROLE").unwrap_or_else(|| "leaf".into());
    let node_id: u64 = env("NDN_NODE_ID").and_then(|s| s.parse().ok()).unwrap_or(u64::MAX);
    let my_bssid = bssid(&env("NDN_BSSID").unwrap_or_else(|| "cc".into()));
    let parent = env("NDN_PARENT").map(|t| bssid(&t)); // only compose off this neighbour (line topology)
    let deadline = Instant::now() + Duration::from_secs(secs);

    if role == "reference" {
        let pid: u16 = u16::from_str_radix(env("NDN_PID").unwrap_or_else(|| "a81a".into()).trim_start_matches("0x"), 16)?;
        let d = LibUsbRtl88xxBackend::open_monitor_pid(pid, ch)?;
        let belief = RefBelief { ref_id: node_id, stratum: 0, offset_to_ref: 0 };
        d.emit_timing_frame(&beacon(my_bssid, belief), 100)?;
        println!("REFERENCE id={node_id} bssid={my_bssid:02x?} — belief-carrying HW beacon armed {secs}s");
        while Instant::now() < deadline { tokio::time::sleep(Duration::from_millis(200)).await; }
        let _ = d.stop_timing_beacon();
        return Ok(());
    }

    // relay + leaf both run a NetworkTime and poll a driver's mesh_common_view.
    let mut net = NetworkTime::new(node_id);
    let mut last_cv = 0u64;
    let mut offsets: Vec<i64> = Vec::new();
    let warmup = Instant::now() + Duration::from_secs(5);

    // Open the right backend: relay on a81a (88xx, must also EMIT), leaf on the Alfa 8812au (receive-only).
    enum Radio { Big(Arc<LibUsbRtl88xxBackend>), Au(Arc<Rtl8812auBackend>) }
    let radio = if role == "relay" {
        let pid: u16 = u16::from_str_radix(env("NDN_PID").unwrap_or_else(|| "a81a".into()).trim_start_matches("0x"), 16)?;
        let d = Arc::new(LibUsbRtl88xxBackend::open_monitor_pid(pid, ch)?);
        let _p = d.spawn_rx_pump(8);
        std::mem::forget(_p);
        Radio::Big(d)
    } else {
        let d = Arc::new(Rtl8812auBackend::open()?);
        d.bring_up_monitor(ch)?;
        let _p = d.spawn_rx_pump(8);
        std::mem::forget(_p);
        Radio::Au(d)
    };
    let mesh = |r: &Radio| -> Option<ndn_radio_hal::MeshCv> {
        match r { Radio::Big(d) => d.mesh_common_view(), Radio::Au(d) => d.mesh_common_view() }
    };
    println!("{} id={node_id} bssid={my_bssid:02x?} parent={parent:02x?}", role.to_uppercase());

    // The relay (re)arms its beacon ONLY when its (ref_id, stratum) changes — i.e. once, when it first
    // adopts the reference — never per-offset. Re-arming disturbs the TSF/beacon timing, which would
    // corrupt the offset the beacon advertises; the offset drifts only ~µs/s so a stable armed beacon is
    // correct. The hardware timestamp in the beacon is still re-inserted fresh each TBTT.
    let mut armed: Option<(u64, u8)> = None;
    while Instant::now() < deadline {
        if let Some(mcv) = mesh(&radio)
            && mcv.count != last_cv
        {
            last_cv = mcv.count;
            // Line topology: only compose off the configured parent neighbour.
            if parent.is_none_or(|p| p == mcv.bssid) {
                let nbr = mcv.belief.unwrap_or(RefBelief { ref_id: 0, stratum: 0, offset_to_ref: 0 });
                net.observe((mcv.peer_tsf as i64) - (mcv.our_rxtsfl as i64), nbr);
                if Instant::now() >= warmup {
                    offsets.push(net.offset_to_ref());
                }
            }
        }
        if role == "relay" && !net.is_reference() {
            let b = net.belief();
            let key = (b.ref_id, b.stratum);
            if armed != Some(key) {
                if let Radio::Big(d) = &radio {
                    let _ = d.emit_timing_frame(&beacon(my_bssid, b), 100);
                }
                armed = Some(key);
            }
        }
        tokio::time::sleep(Duration::from_millis(2)).await;
    }
    if let Radio::Big(d) = &radio { let _ = d.stop_timing_beacon(); }

    let b = net.belief();
    println!("\n=== {} RESULT ===\nbelief: ref_id={} stratum={} offset_to_ref={} µs   observations={}",
        role.to_uppercase(), b.ref_id, b.stratum, b.offset_to_ref, offsets.len());
    if offsets.len() >= 3 {
        let diffs: Vec<i64> = offsets.windows(2).map(|w| w[1] - w[0]).filter(|d| d.abs() < 100_000).collect();
        let mean = diffs.iter().sum::<i64>() as f64 / diffs.len().max(1) as f64;
        let var = diffs.iter().map(|&x| (x as f64 - mean).powi(2)).sum::<f64>() / diffs.len().max(1) as f64;
        println!("offset-to-reference jitter: first-diff std={:.2} µs (over {} hops)  → {}",
            var.sqrt(), b.stratum,
            if var.sqrt() < 20.0 { "network-wide µs sync" } else { "check topology/filter" });
    } else {
        println!("no parent beacons composed — is the parent emitting + in range + BSSID matching?");
    }
    Ok(())
}
