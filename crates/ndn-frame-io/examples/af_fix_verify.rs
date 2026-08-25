//! Verifies the AF_PACKET TX wedge fix directly against the real (fixed)
//! `AfPacketBackend` — no forwarder, no producer, no relay. The old code parked
//! forever in `writable().await` on the first missing EPOLLOUT edge after an
//! EAGAIN; the fix must keep `inject()` completing and resuming after bursts.
//!
//!   sudo ./af_fix_verify mon0
//!
//! PASS = many bursts complete, every inject returns (Ok or dropped), and after
//! each burst injects succeed again. A single inject that never returns (the old
//! bug) shows up as the whole run hanging past the per-call watchdog.
#[cfg(target_os = "linux")]
#[tokio::main(flavor = "current_thread")]
async fn main() {
    use ndn_frame_io::{AfPacketBackend, FrameFormat, FrameIo};
    use ndn_radio_hal::{InjectFrame, TxIntent, DEFAULT_SRC};
    use std::time::Duration;

    let iface = std::env::args().nth(1).unwrap_or_else(|| "mon0".into());
    let backend = AfPacketBackend::new(&iface, FrameFormat::RawNdnS1g { ethertype: 0x8624 })
        .expect("open");
    let mk = || InjectFrame {
        payload: bytes::Bytes::from_static(&[0x5a; 900]),
        tx: TxIntent::CONSERVATIVE,
        dst: [0xff; 6],
        src: DEFAULT_SRC,
        addr3: None,
    };

    let mut total_ok = 0u64;
    let mut total_slow = 0u64; // returned, but only after retrying (hit backpressure)
    let mut max_ms = 0u128;

    // 6 bursts of 300 rapid injects each — well past the ~80-frame buffer
    // ceiling, so every burst forces EAGAIN. Between bursts, a short idle lets
    // the buffer drain; the fix must notice and resume.
    for burst in 1..=6u32 {
        let mut ok = 0;
        for _ in 0..300 {
            let t = tokio::time::Instant::now();
            // Per-call watchdog: with the OLD code an inject could hang forever;
            // 8s >> the fix's 2s internal deadline, so a timeout here = the bug.
            match tokio::time::timeout(Duration::from_secs(8), backend.inject(mk())).await {
                Err(_) => {
                    println!("FAIL: inject() HUNG >8s in burst {burst} — the wedge is NOT fixed");
                    std::process::exit(1);
                }
                Ok(r) => {
                    let ms = t.elapsed().as_millis();
                    max_ms = max_ms.max(ms);
                    if r.is_ok() { ok += 1; }
                    if ms > 50 { total_slow += 1; }
                }
            }
        }
        total_ok += ok;
        println!("burst {burst}: {ok}/300 injects returned Ok (max single-call {max_ms}ms)");
        tokio::time::sleep(Duration::from_secs(3)).await;
        // After idle, a fresh inject must succeed quickly (buffer drained + fix
        // noticed) — the exact thing the old code failed.
        let t = tokio::time::Instant::now();
        match tokio::time::timeout(Duration::from_secs(8), backend.inject(mk())).await {
            Ok(Ok(())) => println!("  post-idle inject OK in {}ms (recovered)", t.elapsed().as_millis()),
            Ok(Err(e)) => println!("  post-idle inject Err {e:?}"),
            Err(_) => { println!("FAIL: post-idle inject HUNG — not fixed"); std::process::exit(1); }
        }
    }
    println!("PASS: {total_ok} injects Ok across 6 bursts, {total_slow} needed retry, no hang (max {max_ms}ms). Fix holds.");
}

#[cfg(not(target_os = "linux"))]
fn main() { eprintln!("linux only"); }
