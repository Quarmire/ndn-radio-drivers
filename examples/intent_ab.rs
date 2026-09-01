//! **Does this radio honour `TxIntent::needs_basic_rate` ON AIR?**
//!
//! Four backends were changed so that a `MostRobust` frame is forced to the universally decodable
//! basic rate (legacy OFDM 6 Mbps) instead of whatever rate the control plane last stored. That
//! change is worth nothing until a WITNESS has read the rate off the air: a host-side counter
//! counts USB writes the device *accepted*, not radiation.
//!
//! ## Why it must be an A/B inside one run
//!
//! Seeing "6 Mbps" on its own proves nothing — the radio may never have been set to anything else.
//! So this **stores a fast rate first** (`set_rate(HT MCS7)`, i.e. exactly the state that made the
//! bug invisible) and then alternates two arms, labelled by SOURCE ADDRESS:
//!
//! ```text
//!   MostRobust  src 02:4e:44:4e:00:a0  -> must air at 6.0 Mb/s legacy OFDM
//!   Throughput  src 02:4e:44:4e:00:b0  -> must air at the stored MCS7
//! ```
//!
//! Labelling by address rather than payload means the witness needs only the radiotap header and
//! one `tcpdump` filter per arm — no payload parsing, and no ambiguity about which frame is which.
//! **If both arms read the same rate, the intent is not reaching the rate.**
//!
//! ## Paced on purpose
//!
//! A kernel-monitor witness saturates (~1350-2500 f/s on this bench), and a saturated witness reads
//! like on-air loss. This paces to `NDN_INTENT_FPS` frames/s (default 100, split across the two
//! arms) so both sit far below any witness ceiling and the comparison is about RATE, not drops.
//!
//! ```text
//!   export NDN_INTENT_FPS=100 NDN_INTENT_MCS=7
//!   sudo -E ./intent_ab <pid-hex> <channel> <seconds>
//! ```
//!
//! ⚠ `sudo` strips `NDN_*` — use `sudo -E` with the vars exported, or the knobs are silently
//! ignored and the run measures defaults while claiming to measure something else.
use ndn_frame_io::{FrameIo, InjectFrame, Reliability, TxIntent};
use ndn_radio_hal::McsDescriptor;
use std::time::{Duration, Instant};

/// Arm labels on air. Locally administered, and ephemeral nonces rather than host identities
/// (mac-addressing doctrine) — they exist only so the witness can attribute a rate to an intent.
const SRC_ROBUST: [u8; 6] = [0x02, 0x4e, 0x44, 0x4e, 0x00, 0xa0];
const SRC_BULK: [u8; 6] = [0x02, 0x4e, 0x44, 0x4e, 0x00, 0xb0];

fn main() -> Result<(), Box<dyn std::error::Error>> {
    tokio::runtime::Runtime::new()?.block_on(run())
}

async fn run() -> Result<(), Box<dyn std::error::Error>> {
    let mut a = std::env::args().skip(1);
    let pid = u16::from_str_radix(&a.next().unwrap_or_else(|| "f72b".into()), 16)?;
    let channel: u8 = a.next().and_then(|s| s.parse().ok()).unwrap_or(6);
    let secs: u64 = a.next().and_then(|s| s.parse().ok()).unwrap_or(10);
    let fps: u64 = std::env::var("NDN_INTENT_FPS")
        .ok()
        .and_then(|v| v.parse().ok())
        .filter(|&n| n > 0)
        .unwrap_or(100);
    let mcs_index: u8 = std::env::var("NDN_INTENT_MCS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(7);

    let radio = ndn_radio_drivers::open_named_radio(pid, channel)?;
    let io = radio.io();

    // ★ The control that makes the test mean something: store a FAST rate, so a `MostRobust` frame
    // airing at 6 Mbps can only be the intent overriding it. Without this the radio might simply
    // be sitting at its default legacy rate and both arms would read 6 Mbps for the wrong reason.
    let stored = McsDescriptor::ht(mcs_index);
    match FrameIo::set_rate(io.as_ref(), stored) {
        Ok(()) => println!("stored rate: HT MCS{mcs_index} (what MostRobust must override)"),
        Err(e) => println!(
            "⚠ set_rate refused ({e}) — this radio has no rate actuator, so the A/B below cannot \
             distinguish 'intent honoured' from 'nothing was ever set'. Treat the result as void."
        ),
    }

    let payload = bytes::Bytes::from(b"NDNINTENTAB0123456789".to_vec());
    let period = Duration::from_micros(1_000_000 / fps.max(1));
    let t = Instant::now();
    let (mut n_robust, mut n_bulk, mut errs) = (0u64, 0u64, 0u64);
    let mut robust_turn = true;

    while t.elapsed() < Duration::from_secs(secs) {
        let (intent, src) = if robust_turn {
            (TxIntent::broadcast(Reliability::MostRobust), SRC_ROBUST)
        } else {
            (TxIntent::broadcast(Reliability::Throughput), SRC_BULK)
        };
        let mut f = InjectFrame::broadcast(payload.clone(), intent);
        f.src = src;
        match FrameIo::inject(io.as_ref(), f).await {
            Ok(()) => {
                if robust_turn {
                    n_robust += 1
                } else {
                    n_bulk += 1
                }
            }
            Err(_) => errs += 1,
        }
        robust_turn = !robust_turn;
        tokio::time::sleep(period).await;
    }

    let el = t.elapsed().as_secs_f64();
    println!(
        "sent: robust={n_robust} bulk={n_bulk} err={errs} in {el:.1}s \
         ({:.0} f/s total)",
        (n_robust + n_bulk) as f64 / el
    );
    println!("⚠ those are USB writes ACCEPTED, not radiation. The witness decides:");
    println!("    robust arm:  tcpdump -e -i <mon> 'wlan addr2 02:4e:44:4e:00:a0'  -> expect 6.0 Mb/s");
    println!("    bulk arm:    tcpdump -e -i <mon> 'wlan addr2 02:4e:44:4e:00:b0'  -> expect MCS{mcs_index}");
    println!("    same rate in both arms => intent is NOT reaching the rate.");
    Ok(())
}
