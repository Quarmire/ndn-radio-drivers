//! #76 on-air validation at N=3 — the regime where the time-slice token actually beats contention
//! (at N=2 it tied, #73). Three nodes each saturate-broadcast their own tag; each counts the other two.
//! Slot k of an N=3 superframe is owned by node k, so in `slotted` the three transmit disjointly (no
//! collisions); in `contention` each blasts a random ~1/3 of slots (independent) → the same slot
//! collides ~2/9 of the time. So `recv_total(slotted) > recv_total(contention)` is the collision-freedom
//! the token buys once N≥3. Wall-clock epoch (the two OPis are NTP-synced to ~ms; ms slots).
//!
//!   sudo NDN_PID=a81a NDN_NODE=0 NDN_MODE=slotted ./slot3_airtest 40 22   # o5p-0 a81a
//!   sudo NDN_PID=a81a NDN_NODE=1 NDN_MODE=slotted ./slot3_airtest 40 22   # o5p-1 a81a
//!   sudo NDN_PID=au   NDN_NODE=2 NDN_MODE=slotted ./slot3_airtest 40 22   # o5p-0 Alfa 8812au
//! then all three with NDN_MODE=contention.

use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use ndn_frame_io::{BROADCAST, InjectFrame, TxIntent};

const N: u64 = 3;
const FRAME_BYTES: usize = 900;

fn env(k: &str) -> Option<String> { std::env::var(k).ok() }
fn now_us() -> u64 { SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_micros() as u64).unwrap_or(0) }

struct Rng(u64);
impl Rng { fn coin(&mut self) -> bool { let mut x=self.0; x^=x<<13; x^=x>>7; x^=x<<17; self.0=x; x&1==0 } }

#[tokio::main(flavor = "multi_thread", worker_threads = 4)]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(40);
    let secs: u64 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(22);
    let node: u8 = env("NDN_NODE").and_then(|s| s.parse().ok()).unwrap_or(0);
    let slotted = env("NDN_MODE").as_deref() != Some("contention");
    let slot_us: u64 = env("NDN_SLOT_US").and_then(|s| s.parse().ok()).unwrap_or(20_000);
    let my_slot = node as u64 % N;
    let my_tag = 0xA0u8 | node;

    // One standardized open — the canonical named-data format + monitor + pump handled beneath, so all
    // three nodes interoperate on air by construction regardless of chip.
    let pid: u16 = match env("NDN_PID").as_deref() {
        Some("au") => 0x8812,
        Some(p) => u16::from_str_radix(p.trim_start_matches("0x"), 16)?,
        None => 0xa81a,
    };
    let d = ndn_radio_drivers::open_named_radio(pid, ch)?;
    let src = [0x02, b'M', b'D', b'R', node, 0x01];
    println!("slot3 node={node} slot={my_slot}/{N} mode={} ch{ch} secs={secs}",
        if slotted { "slotted" } else { "contention" });

    let sent = Arc::new(AtomicU64::new(0));
    let recv = [Arc::new(AtomicU64::new(0)), Arc::new(AtomicU64::new(0)), Arc::new(AtomicU64::new(0))];
    let deadline = Instant::now() + Duration::from_secs(secs);
    let count_start = Instant::now() + Duration::from_secs(3); // warmup

    // RX task: drain, tally frames per source node (payload[0] = 0xA0|node, payload[1] = node).
    let rx = {
        let (d, recv) = (d.clone(), recv.clone());
        tokio::spawn(async move {
            while Instant::now() < deadline {
                if let Ok(Ok(f)) = tokio::time::timeout(Duration::from_millis(5), d.recv_frame()).await {
                    let p = &f.payload;
                    if p.len() >= 2 && (p[0] & 0xf0) == 0xA0 {
                        let n = (p[1] & 0x0f) as usize;
                        if n < 3 && n != node as usize && Instant::now() >= count_start {
                            recv[n].fetch_add(1, Ordering::Relaxed);
                        }
                    }
                }
                // Cooperate: under a flooded RX queue `recv_frame` returns instantly every time, so
                // without an explicit yield this task never releases the worker and starves the TX
                // loop's inject (the a81a's heavier RX volume made this fatal on a 2-worker runtime).
                tokio::task::yield_now().await;
            }
        })
    };

    // TX task: saturate per mode.
    let mut rng = Rng((std::process::id() as u64).wrapping_mul(0x9E37_79B9) ^ (node as u64 + 1));
    let mut cur_epoch = u64::MAX;
    let mut tx_this = false;
    let pad = vec![0u8; FRAME_BYTES];
    while Instant::now() < deadline {
        let epoch = now_us() / slot_us;
        if epoch != cur_epoch {
            cur_epoch = epoch;
            tx_this = if slotted { epoch % N == my_slot } else { rng.coin() };
        }
        if tx_this {
            let mut payload = Vec::with_capacity(FRAME_BYTES + 2);
            payload.push(my_tag);
            payload.push(node);
            payload.extend_from_slice(&pad);
            let _ = d.inject(InjectFrame { payload: payload.into(), tx: TxIntent::CONSERVATIVE, dst: BROADCAST, src }).await;
            sent.fetch_add(1, Ordering::Relaxed);
            tokio::task::yield_now().await;
        } else {
            tokio::time::sleep(Duration::from_micros(500)).await;
        }
    }
    // Don't block shutdown on the background RX task — the counters are shared atomics, so read them
    // directly and let process exit drop the task. (Awaiting a hot RX task that shares the USB handle
    // with a saturating TX loop could stall at shutdown; the tally is already in the atomics.)
    rx.abort();

    let (r0, r1, r2) = (recv[0].load(Ordering::Relaxed), recv[1].load(Ordering::Relaxed), recv[2].load(Ordering::Relaxed));
    let peers: u64 = r0 + r1 + r2;
    println!("=== node={node} mode={} === sent={} recv_from_peers={peers} (n0={r0} n1={r1} n2={r2})",
        if slotted { "slotted" } else { "contention" }, sent.load(Ordering::Relaxed));
    use std::io::Write;
    let _ = std::io::stdout().flush();
    Ok(())
}
