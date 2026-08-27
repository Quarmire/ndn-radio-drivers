//! Does `REG_TXPAUSE` give the named airtime lease a real actuator? Three questions, in order:
//!
//!   1. DOES IT GATE?      pause held for a whole arm => the receiver should hear ~nothing.
//!   2. HOLD OR DROP?      the arm right after release: a burst ABOVE baseline means frames were
//!                         HELD and flushed, which would spill out of a lease window into the next
//!                         one. Suppression and deferral are very different things for a lease.
//!   3. HOW FAST?          host-side write latency, then duty-cycle arms at 20/5/1 ms half-periods.
//!                         If delivered fraction tracks ~50% down to 1 ms it can shape slots; if it
//!                         smears to 100% as the period shortens, the gate is only good for coarse
//!                         lease windows and slotting needs the hardware beacon instead.
//!
//! Arms are tagged knob 3 so `rxpwr_bucket` on the a81a counts them (its `n` per arm IS the
//! delivered count). TX side prints what it injected, so delivered fraction is rx_n / tx_sent —
//! never inferred from one side alone.
//!
//! ⚠ MEASURED on the first run: with the gate held, `inject` BLOCKS — the queue backs up and the
//! injection path stalls indefinitely (the first attempt was killed by its own timeout mid-arm).
//! That is the hold-vs-drop answer, so the gated arm is now bounded and timeout-per-frame.
//! Held frames keep arm 1's payload tag, so if they flush on release they still land in the
//! receiver's idx-1 bucket: n(idx1) ~ 0 means suppressed, n(idx1) ~ attempts means held+flushed.
use bytes::Bytes;
use ndn_radio_drivers::{BROADCAST, FrameIo, InjectFrame, Rtl8733buBackend, TxIntent};
use std::time::Instant;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(36);
    let n: u32 = std::env::args()
        .nth(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(1500);
    let dev = Rtl8733buBackend::open()?;
    dev.bring_up_tx(ch)?;
    // NDN_NAV_UPPER_US caps how long a DECODED NAV from someone else may hold our transmitter.
    if let Ok(v) = std::env::var("NDN_NAV_UPPER_US") {
        let us: u32 = v.parse().unwrap_or(0);
        dev.set_nav_upper_us(us)?;
        println!(
            "NAV_UPPER set to {us} us -> reads {} us",
            dev.nav_upper_us()?
        );
    } else {
        println!("NAV_UPPER left at {} us (default)", dev.nav_upper_us()?);
    }

    // --- Q3a: host-side cost of toggling the gate, before any on-air claim.
    let t0 = Instant::now();
    for i in 0..200u32 {
        dev.set_tx_pause(if i % 2 == 0 { 0xff } else { 0x00 })?;
    }
    let per = t0.elapsed().as_secs_f64() * 1e6 / 200.0;
    dev.set_tx_pause(0x00)?;
    println!(
        "gate toggle cost: {per:.1} us/write  (readback now 0x{:02x})",
        dev.tx_pause()?
    );
    println!(
        "=> a 50% duty cycle cannot be shaped faster than ~{:.0} us half-period",
        per * 2.0
    );

    // NDN_PAUSE_HALF_MS=<n> runs ONE arm at that half-period (0 = ungated control) for
    // NDN_PAUSE_SECS seconds, so a time-resolved receiver measures one condition at a time
    // instead of a blended histogram across six.
    if let Ok(half) = std::env::var("NDN_PAUSE_HALF_MS") {
        let half: u64 = half.parse().unwrap_or(0);
        let secs: u64 = std::env::var("NDN_PAUSE_SECS")
            .ok()
            .and_then(|s| s.parse().ok())
            .unwrap_or(15);
        let mut p = vec![0xC3u8; 300];
        p[0] = 0;
        p[1] = 3;
        let f = InjectFrame {
            payload: Bytes::from(p),
            tx: TxIntent::CONSERVATIVE,
            dst: BROADCAST,
            src: [0x02, 0x50, 0x33, 0x02, 3, 0],
            addr3: None,
            addr4: None,
            htc: None,
        };
        dev.set_tx_pause(0x00)?;
        let start = Instant::now();
        let (mut sent, mut stalled, mut gated) = (0u32, 0u32, false);
        while start.elapsed().as_secs() < secs {
            if half > 0 {
                let want = (start.elapsed().as_millis() as u64 / half) % 2 == 1;
                if want != gated {
                    dev.set_tx_pause(if want { 0xff } else { 0x00 })?;
                    gated = want;
                }
            }
            match tokio::time::timeout(std::time::Duration::from_millis(50), dev.inject(f.clone()))
                .await
            {
                Ok(Ok(())) => sent += 1,
                Ok(Err(_)) => {}
                Err(_) => stalled += 1,
            }
        }
        dev.set_tx_pause(0x00)?;
        println!("single arm: half_ms={half} secs={secs} injected={sent} stalled={stalled}");
        return Ok(());
    }
    let arms: [(&str, u64); 6] = [
        ("control (no gate)", 0),
        ("gate HELD on", 0),
        ("flush check (just released)", 0),
        ("duty 50% @20ms", 20),
        ("duty 50% @5ms", 5),
        ("duty 50% @1ms", 1),
    ];
    for (i, (name, half_ms)) in arms.iter().enumerate() {
        if i == 1 {
            dev.set_tx_pause(0xff)?;
        } else if i == 2 {
            dev.set_tx_pause(0x00)?; // release: anything held now flushes into THIS arm
        } else {
            dev.set_tx_pause(0x00)?;
        }
        let mut p = vec![0xC3u8; 300];
        p[0] = i as u8;
        p[1] = 3;
        let f = InjectFrame {
            payload: Bytes::from(p),
            tx: TxIntent::CONSERVATIVE,
            dst: BROADCAST,
            src: [0x02, 0x50, 0x33, 0x02, 3, i as u8],
            addr3: None,
            addr4: None,
            htc: None,
        };
        let start = Instant::now();
        let mut sent = 0u32;
        let mut stalled = 0u32;
        let mut gated = false;
        let attempts = if i == 1 { 300 } else { n };
        for k in 0..attempts {
            if *half_ms > 0 {
                let want = (start.elapsed().as_millis() as u64 / half_ms) % 2 == 1;
                if want != gated {
                    dev.set_tx_pause(if want { 0xff } else { 0x00 })?;
                    gated = want;
                }
            }
            // Bounded: a held queue blocks the injector, so never wait unbounded on a gated arm.
            match tokio::time::timeout(std::time::Duration::from_millis(50), dev.inject(f.clone()))
                .await
            {
                Ok(Ok(())) => sent += 1,
                Ok(Err(_)) => {}
                Err(_) => stalled += 1,
            }
            let _ = k;
        }
        if *half_ms > 0 {
            dev.set_tx_pause(0x00)?;
        }
        println!(
            "  arm {i} {name:<28} injected={sent} stalled={stalled} in {:.2}s",
            start.elapsed().as_secs_f64()
        );
        tokio::time::sleep(std::time::Duration::from_millis(400)).await;
    }
    dev.set_tx_pause(0x00)?;
    println!(
        "=== TXPAUSE SWEEP DONE (gate released: 0x{:02x}) ===",
        dev.tx_pause()?
    );
    Ok(())
}
