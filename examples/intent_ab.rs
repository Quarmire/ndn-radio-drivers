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
//!   A  MostRobust  src 02:4e:44:4e:00:a0  at MCS7  -> must air at 6.0 Mb/s legacy OFDM
//!   B  Throughput  src 02:4e:44:4e:00:b0  at MCS7  -> must air at MCS 7
//!   C  Throughput  src 02:4e:44:4e:00:c0  at MCS0  -> must air at MCS 0
//! ```
//!
//! Labelling by address rather than payload means the witness needs only the radiotap header and
//! one `tcpdump` filter per arm — no payload parsing, and no ambiguity about which frame is which.
//! **If A and B read the same rate, the intent is not reaching the rate.**
//!
//! ## ☠ Why arm C exists — it rescues the experiment from an ambiguity I published without it
//!
//! The first 8733b run read 376/431 on arm A and **3/431 on arm B**, and that was written up as
//! the one-way link demonstrating itself. It is not sound on its own: the SAME witness, same host,
//! same session, decoded the MT7612U's MCS7 arm 348/431. So "MCS7 does not survive the link" and
//! "**this witness cannot demodulate THIS transmitter's HT at all**" produce an identical capture,
//! and a two-arm harness cannot separate them.
//!
//! Arm C is the discriminator. HT MCS0 (6.5 Mb/s) needs roughly the same SNR as legacy 6 Mb/s, so:
//!
//! * witness reads C but not B  => the loss really is rate/margin; arm B was evidence.
//! * witness reads neither      => this witness's HT decode of this DUT is dead, and arm B never
//!                                 said anything about the link. Report the A-vs-B RATE result
//!                                 (which is what the fix is about) and drop the delivery claim.
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
/// Arm C: Throughput at the HT FLOOR — the control that separates "MCS7 lost on the link" from
/// "this witness cannot decode this transmitter's HT at all".
const SRC_HTFLOOR: [u8; 6] = [0x02, 0x4e, 0x44, 0x4e, 0x00, 0xc0];

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
    let (mut n_robust, mut n_bulk, mut n_floor, mut errs) = (0u64, 0u64, 0u64, 0u64);
    let mut arm = 0u8;

    while t.elapsed() < Duration::from_secs(secs) {
        // The stored rate is re-asserted per arm, so arm C really is an HT MCS0 PPDU rather than
        // MCS7 relabelled. `MostRobust` must override it in arm A exactly as it must in arm B.
        let (reliability, src, rate) = match arm {
            0 => (Reliability::MostRobust, SRC_ROBUST, stored),
            1 => (Reliability::Throughput, SRC_BULK, stored),
            _ => (Reliability::Throughput, SRC_HTFLOOR, McsDescriptor::ht(0)),
        };
        let _ = FrameIo::set_rate(io.as_ref(), rate);
        let mut f = InjectFrame::broadcast(payload.clone(), TxIntent::broadcast(reliability));
        f.src = src;
        match FrameIo::inject(io.as_ref(), f).await {
            Ok(()) => match arm {
                0 => n_robust += 1,
                1 => n_bulk += 1,
                _ => n_floor += 1,
            },
            Err(_) => errs += 1,
        }
        arm = (arm + 1) % 3;
        tokio::time::sleep(period).await;
    }

    let el = t.elapsed().as_secs_f64();
    println!(
        "sent: A robust={n_robust} B bulk={n_bulk} C htfloor={n_floor} err={errs} in {el:.1}s \
         ({:.0} f/s total)",
        (n_robust + n_bulk + n_floor) as f64 / el
    );
    println!("⚠ those are USB writes ACCEPTED, not radiation. The witness decides:");
    println!("    robust arm:  tcpdump -e -i <mon> 'wlan addr2 02:4e:44:4e:00:a0'  -> expect 6.0 Mb/s");
    println!("    B bulk arm:  tcpdump -e -i <mon> 'wlan addr2 02:4e:44:4e:00:b0'  -> expect MCS{mcs_index}");
    println!("    C floor arm: tcpdump -e -i <mon> 'wlan addr2 02:4e:44:4e:00:c0'  -> expect MCS 0");
    println!("    A and B at the same rate => intent is NOT reaching the rate.");
    println!("    B and C both absent      => the witness cannot decode this DUT's HT; B says nothing about the link.");
    Ok(())
}
