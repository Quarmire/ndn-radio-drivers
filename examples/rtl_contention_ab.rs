//! A/B/A the contention posture on an RTL8812AU, against a competitor saturating the same channel.
//!
//! This is the regression test for a *behaviour change*, not a new feature. `set_cca_ignore` used
//! to write a zero contention window (`REG_EDCA_BE_PARAM = 0x005e_0002`) as a hidden side effect,
//! and that write was MEASURED to rescue this part from being out-competed by an a81a on a busy
//! channel (~330 f/s against ~13000). The window is now a knob of its own with a legal floor, so
//! the question this example answers is: **does `Owned` at CW exponent 2 keep the win that CW 0
//! bought?** If it does not, the floor costs real capability and that has to be said out loud.
//!
//! Run a competitor flooding the same channel first, or this measures an idle medium and says
//! nothing at all.
//!
//! ```text
//! NDN_ARMS=shared,owned,shared,owned NDN_SECS=5 rtl_contention_ab 36
//! ```
use bytes::Bytes;
use ndn_frame_io::{BROADCAST, DEFAULT_SRC, FrameIo, InjectFrame, TxIntent};
use ndn_radio_drivers::Rtl8812auBackend;
use ndn_radio_hal::{ContentionPosture, RadioKnobs};
use std::time::{Duration, Instant};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(36);
    let secs: u64 = std::env::var("NDN_SECS")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(5);
    // ★ Payload length matters more than anything else here. A contention knob is a *fixed*
    // per-frame cost, so it is invisible whenever airtime dominates the period: at legacy 6M a
    // 1400 B frame is ~1867 us of air against a 72 us saving. Short frames at a high rate are
    // the regime where medium access is most of the period, and the only regime that sizes this
    // knob honestly.
    let len: usize = std::env::var("NDN_LEN")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(1400);
    let arms: Vec<String> = std::env::var("NDN_ARMS")
        .unwrap_or_else(|_| "shared,owned,shared,owned".into())
        .split(',')
        .map(|s| s.trim().to_string())
        .collect();

    let rt = tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()?;
    rt.block_on(async move {
        // ★ `FrameFormat::Raw80211` is this backend's DEFAULT (`rtl8812au.rs:4477`), and it means
        // "the payload IS the frame, verbatim" — so a `vec![0x42; N]` filler was being transmitted
        // as a malformed 802.11 MANAGEMENT frame with protocol version 2 and a **unicast**
        // `addr1 = 42:42:42:42:42:42`. The `dst: BROADCAST` below was ignored entirely.
        //
        // A unicast frame expects an ACK. It never arrived, so the MAC ran the retry ladder and
        // doubled the contention window to CWmax on every retry; once latched, ~15 retries x
        // ~4.6 ms of backoff is ~70 ms per frame, the bulk-OUT FIFO backs up, and `write_bulk`
        // hits its 100 ms `TX_TIMEOUT` — MEASURED as a dead-flat **10 f/s** after the first second.
        // The a81a looked healthy on identical air only because its backend defaults to `RawNdn`
        // (`libusb_rtl88xx.rs:386`) and therefore actually broadcasts.
        let d = Rtl8812auBackend::open()?.with_format(ndn_frame_io::FrameFormat::default());
        d.bring_up_monitor(ch)?;
        println!("8812AU contention A/B ch{ch}, {secs}s per arm, {len} B payload\n");

        let frame = InjectFrame {
            payload: Bytes::from(vec![0x42u8; len]),
            tx: if std::env::var_os("NDN_FAST").is_some() {
                TxIntent::broadcast(ndn_radio_hal::Reliability::Throughput)
            } else {
                TxIntent::ROBUST
            },
            dst: BROADCAST,
            src: DEFAULT_SRC,
            addr3: None,
            addr4: None,
            htc: None,
        };

        for (i, arm) in arms.iter().enumerate() {
            let posture = match arm.as_str() {
                "owned" => ContentionPosture::Owned,
                "yielding" => ContentionPosture::Yielding,
                _ => ContentionPosture::Shared,
            };
            // ★ Write on EVERY arm, never only when the value differs from the last one. An
            // earlier A/B in this campaign was invalid precisely because its "control" arm
            // skipped the write and silently inherited the previous arm's state.
            let applied = d.set_contention(posture)?;
            let be = d.read32(0x0508)?;
            println!(
                "arm {i} {arm:>8}: cw {}..{} aifsn {} slot {} us => backoff {} us, medium access {} us  [0x0508 = {be:#010x}]",
                applied.cw_min,
                applied.cw_max,
                applied.aifs,
                applied.slot_us,
                applied.avg_backoff_us,
                applied.medium_access_us(),
            );

            // ★ Per-second buckets, not just a total. Between-run variance and within-run decay
            // look identical in a single aggregate, and they have opposite causes: a bring-up that
            // lands in a different state each open is CONSTANT within a run and varies between
            // runs; something degrading (a filling queue, a thermal or gain walk, an autonomous
            // MAC loop) DECAYS within the run. One number cannot tell them apart, which is exactly
            // how this campaign mistook one for the other.
            let mut buckets = Vec::with_capacity(secs as usize);
            let mut n = 0u64;
            for _ in 0..secs {
                let mut m = 0u64;
                let stop = Instant::now() + Duration::from_secs(1);
                while Instant::now() < stop {
                    let _ = d.inject(frame.clone()).await;
                    m += 1;
                }
                buckets.push(m);
                n += m;
            }
            let fps = n as f64 / secs as f64;
            let per_s: Vec<String> = buckets.iter().map(|b| b.to_string()).collect();
            println!("           {n} frames = {fps:.0} f/s  | per-second: {}", per_s.join(" "));
            println!();
        }
        // Leave the radio the way we found it, not in whatever the last arm was.
        d.set_contention(ContentionPosture::Shared)?;
        Ok::<(), Box<dyn std::error::Error>>(())
    })
}
