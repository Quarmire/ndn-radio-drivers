//! Verify the TX-power knob END TO END through the HAL, on the production `open_named_radio` path.
//!
//! Not `Rtl8733buBackend::set_tssi_de` directly — `RadioKnobs::set_tx_power`, the call cognition
//! actually makes. Until now that call reached `0x4308`, a register measured inert on this part, so
//! every caller could set any index and change nothing. This proves the wiring, not just the knob.
//!
//! Index is back-off below the ceiling, 127 = full power, ~0.111 dB/step.
use bytes::Bytes;
use ndn_radio_drivers::{BROADCAST, InjectFrame, TxIntent};

const ARMS: [u32; 10] = [127, 111, 95, 79, 63, 47, 31, 15, 0, 127]; // last = return-to-baseline

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(36);
    let n: u32 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(1200);
    let r = ndn_radio_drivers::open_named_radio(0xf72b, ch)?;
    let knobs = r.knobs.ok_or("radio exposes no RadioKnobs")?;
    if let Some(p) = &r.profile {
        let c = p.capability();
        println!("capability: max_tx_power={} tx_power_dbm={:?}", c.max_tx_power, c.tx_power_dbm);
    }
    for (i, &idx) in ARMS.iter().enumerate() {
        knobs.set_tx_power(idx)?;
        println!("  arm {i}: set_tx_power({idx}) -> expect {:.1} dB below ceiling",
                 (127 - idx) as f32 * 0.111);
        let mut p = vec![0xC3u8; 300];
        p[0] = i as u8;
        p[1] = 9; // knob 9 = HAL set_tx_power
        let f = InjectFrame {
            payload: Bytes::from(p),
            tx: TxIntent::CONSERVATIVE,
            dst: BROADCAST,
            src: [0x02, 0x50, 0x33, 0x02, 9, i as u8],
            addr3: None,
        };
        tokio::time::sleep(std::time::Duration::from_millis(500)).await;
        for _ in 0..n {
            r.io.inject(f.clone()).await?;
        }
    }
    println!("=== HAL SWEEP DONE ===");
    Ok(())
}
