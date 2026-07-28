//! Emit hardware-TSF-stamped timing frames ON DEMAND through the beacon engine (#74) — the
//! doctrine-clean self-contained µs clock source (no AP). Pairs with `beacon_cv` on a second node,
//! which common-views this node's BSSID. If beacon_cv reports our BSSID (02:4e:44:4e:ca:fe) with µs
//! first-diff jitter, self-contained µs common-view is proven — the hardware stamped OUR frame.
//!
//! On-demand, not periodic: this driver capability (`emit_timing_frame`) fires ONE stamped frame per
//! call; a real node would fire it with a data burst / at a slot boundary / on request. Here we call it
//! at a controlled cadence just to gather samples.
//!
//!   sudo NDN_PID=a81a ./timing_emit 40 20        # node A (emitter)
//!   sudo NDN_PID=a81a ./beacon_cv   40 22        # node B (receiver) — look for 02:4e:44:4e:ca:fe

use std::sync::Arc;
use std::time::{Duration, Instant};

use ndn_radio_drivers::LibUsbRtl88xxBackend;

/// Our locally-administered BSSID/nonce (02 = locally administered, unicast) — distinct from any
/// infrastructure AP so the receiver can pick it out.
const BSSID: [u8; 6] = [0x02, 0x4e, 0x44, 0x4e, 0xca, 0xfe];

/// A minimal 802.11 beacon whose body offset 24..32 (the Timestamp slot) is zeroed for the hardware to
/// overwrite with the live TSF at TX. Body: Timestamp(8)=0, Interval(2), Capability(2), SSID IE, Rates IE.
fn timing_beacon() -> Vec<u8> {
    let mut f = Vec::with_capacity(64);
    f.extend_from_slice(&[0x80, 0x00]); // FC: beacon, no flags
    f.extend_from_slice(&[0x00, 0x00]); // duration
    f.extend_from_slice(&[0xff; 6]); // addr1 DA = broadcast
    f.extend_from_slice(&BSSID); // addr2 SA
    f.extend_from_slice(&BSSID); // addr3 BSSID
    f.extend_from_slice(&[0x00, 0x00]); // seq ctrl
    debug_assert_eq!(f.len(), 24); // body starts here — offset 24
    f.extend_from_slice(&[0u8; 8]); // Timestamp — HARDWARE FILLS THIS at TX
    f.extend_from_slice(&[0x64, 0x00]); // beacon interval (100 TU) — informational only, we're on-demand
    f.extend_from_slice(&[0x00, 0x00]); // capability
    f.extend_from_slice(&[0x00, 0x04, b'N', b'D', b'N', b'T']); // SSID IE "NDNT"
    f.extend_from_slice(&[0x01, 0x01, 0x8b]); // supported rates IE (1 rate)
    f
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(40);
    let secs: u64 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(20);
    let pid: u16 = std::env::var("NDN_PID")
        .ok()
        .and_then(|s| u16::from_str_radix(s.trim_start_matches("0x"), 16).ok())
        .unwrap_or(0xa81a);
    let period_ms: u64 = std::env::var("NDN_EMIT_MS").ok().and_then(|s| s.parse().ok()).unwrap_or(50);

    let d = Arc::new(LibUsbRtl88xxBackend::open_monitor_pid(pid, ch)?);
    let bssid = BSSID.iter().map(|b| format!("{b:02x}")).collect::<Vec<_>>().join(":");
    println!("timing_emit: pid={pid:04x} ch{ch} bssid={bssid} — on-demand HW-stamped frames every {period_ms}ms for {secs}s");

    let frame = timing_beacon();
    let deadline = Instant::now() + Duration::from_secs(secs);
    let (mut ok, mut err) = (0u64, 0u64);
    while Instant::now() < deadline {
        match d.emit_timing_frame(&frame) {
            Ok(()) => ok += 1,
            Err(e) => {
                if err == 0 {
                    eprintln!("emit_timing_frame error: {e:?}");
                }
                err += 1;
            }
        }
        std::thread::sleep(Duration::from_millis(period_ms));
    }
    println!("emitted={ok} errors={err}  (receiver runs beacon_cv; look for bssid {bssid})");
    Ok(())
}
