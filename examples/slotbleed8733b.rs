//! Does the hardware MAC gate stop the bleed a software wait cannot?
//!
//! A slot schedule's software wait stops US calling `inject` outside our turn. It cannot stop frames
//! ALREADY in the MAC queue, which drain into whoever owns the next slot — charged to a name that
//! did not cause it. `FaceScheduler::gate` now closes `RadioKnobs::set_tx_hold` for the wait; this
//! measures whether that actually helps, rather than arguing it from the TXPAUSE numbers.
//!
//! Two arms, identical offered load and identical slot pattern:
//!   A  software gating only  — inject during the open half, stop during the closed half
//!   B  software + hardware   — the same, plus the MAC gate closed across the closed half
//!
//! A witness with per-frame hardware RX stamps folds arrivals modulo the period. Concentration in
//! one half is the measure; the absolute phase does not matter and is never assumed.
use bytes::Bytes;
use ndn_radio_drivers::{BROADCAST, FrameIo, InjectFrame, Rtl8733buBackend, TxIntent};
use std::time::Instant;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(36);
    let half_ms: u64 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(20);
    let secs: u64 = std::env::args().nth(3).and_then(|s| s.parse().ok()).unwrap_or(12);
    let hw_gate = std::env::var("NDN_HW_GATE").is_ok();
    let dev = Rtl8733buBackend::open()?;
    dev.bring_up_tx(ch)?;
    dev.set_tx_pause(0x00)?;

    let mut p = vec![0xC3u8; 300];
    p[0] = 0;
    p[1] = if hw_gate { 2 } else { 1 }; // arm tag: 1 = software only, 2 = software + hardware
    let f = InjectFrame {
        payload: Bytes::from(p),
        tx: TxIntent::CONSERVATIVE,
        dst: BROADCAST,
        src: [0x02, 0x50, 0x33, 0x02, if hw_gate { 2 } else { 1 }, 0],
        addr3: None,
    };
    println!("arm: {} (half-period {half_ms} ms, {secs}s)",
             if hw_gate { "software + HARDWARE gate" } else { "software gating only" });

    let start = Instant::now();
    let (mut sent, mut closed_now) = (0u32, false);
    while start.elapsed().as_secs() < secs {
        let closed = (start.elapsed().as_millis() as u64 / half_ms) % 2 == 1;
        if closed != closed_now {
            // Both arms stop offering frames in the closed half — that is the software wait. Only
            // arm B additionally shuts the MAC, which is the whole difference under test.
            if hw_gate {
                dev.set_tx_pause(if closed { 0xff } else { 0x00 })?;
            }
            closed_now = closed;
        }
        if closed {
            tokio::time::sleep(std::time::Duration::from_millis(1)).await;
            continue;
        }
        if tokio::time::timeout(std::time::Duration::from_millis(50), dev.inject(f.clone()))
            .await
            .is_ok()
        {
            sent += 1;
        }
    }
    dev.set_tx_pause(0x00)?;
    println!("injected={sent}");
    Ok(())
}
