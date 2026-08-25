//! Replay the vendor's TXAGC-page state wholesale, then measure.
//!
//! A register-dump diff (our live BB state after `bring_up_tx` vs 6053 captured writes from a working
//! vendor session) showed 40 differences out of 783 shared addresses — and **21 of them sit in the
//! 0x43xx TXAGC/TSSI page**, with seven registers we leave at ZERO that the vendor programs.
//!
//! Enabling TSSI alone (`0x4318[30:28] = 7`) moved the received level 0.51 dB on n≈25k frames/arm,
//! i.e. not a power-control engine starting. That is unsurprising: the enable bit without the loop's
//! configuration is not a working TSSI. This applies the whole differing set instead of one bit.
//!
//! Alternates OUR state and VENDOR state within one boot so drift cancels. If the vendor state
//! changes the level, the mechanism is in this page and the per-rate tables become meaningful; if it
//! does not, the TXAGC page is not where this part's output is decided and the search moves to RF.
//!
//!   sudo NDN_DWELL=6 ./vendor_txagc8733b [channel]

use std::sync::Arc;

use bytes::Bytes;
use ndn_radio_drivers::{BROADCAST, FrameIo, InjectFrame, Rtl8733buBackend, TxIntent};

/// Vendor final values at every address where our live state differed, restricted to the TXAGC/TSSI
/// page and the per-rate table. Taken from the capture, not guessed.
const VENDOR: &[(u16, u32)] = &[
    (0x3a00, 0xc0c0c0c0), (0x3a04, 0x08080c0c), (0x3a08, 0x04040408),
    (0x3a0c, 0x04080c0c), (0x3a10, 0x00000004), (0x3a40, 0x00000040),
    (0x4304, 0x00000000), (0x4308, 0x5c545c50), (0x430c, 0x3f3f3f3f),
    (0x4318, 0x7000807f), // TSSI enable = 7
    (0x4320, 0x02883100), (0x4328, 0x03280200), (0x432c, 0x1000ff55),
    (0x4334, 0x00200000), (0x433c, 0x00200000), (0x4368, 0x00000002),
    (0x4378, 0x00000002), (0x4380, 0x00000203), (0x438c, 0xa0a04040),
    (0x4390, 0x80808080), (0x4394, 0xa4a44040), (0x4398, 0x80808080),
    (0x439c, 0x00800801), (0x43a8, 0x77470d00), (0x43b0, 0x00020200),
    (0x43b4, 0x0000fe00), (0x43b8, 0x000000fe),
];
/// What our bring-up leaves at those same addresses, so the arms are reversible in-flight.
const OURS: &[(u16, u32)] = &[
    (0x3a00, 0x2d2d2d2d), (0x3a04, 0x2d2d2d2d), (0x3a08, 0x2d2d2d2d),
    (0x3a0c, 0x2d2d2d2d), (0x3a10, 0x2d2d2d2d), (0x3a40, 0x00000000),
    (0x4304, 0x00008080), (0x4308, 0x40404040), (0x430c, 0x0a3f3f3f),
    (0x4318, 0x0000807f),
    (0x4320, 0x02880100), (0x4328, 0x42280200), (0x432c, 0x1000ff50),
    (0x4334, 0x00000000), (0x433c, 0x00000000), (0x4368, 0x00000000),
    (0x4378, 0x00000000), (0x4380, 0x00000002), (0x438c, 0x80804040),
    (0x4390, 0x80804040), (0x4394, 0x80804040), (0x4398, 0x80804040),
    (0x439c, 0x00080801), (0x43a8, 0x7747fd00), (0x43b0, 0x00000000),
    (0x43b4, 0x00000000), (0x43b8, 0x00000000),
];

#[tokio::main(flavor = "multi_thread", worker_threads = 2)]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(36);
    let dwell: u64 = std::env::var("NDN_DWELL").ok().and_then(|s| s.parse().ok()).unwrap_or(6);
    let dev = Arc::new(Rtl8733buBackend::open()?);
    let _t = dev.bring_up_tx_tracked(ch)?;

    let payload = Bytes::from(vec![0xC3u8; 300]);
    let warm = InjectFrame {
        payload: payload.clone(), tx: TxIntent::CONSERVATIVE, dst: BROADCAST,
        src: [0x02, 0x56, 0x54, 0x58, 0xff, 0x01], addr3: None,
    };
    let t0 = std::time::Instant::now();
    while t0.elapsed() < std::time::Duration::from_secs(2) {
        dev.inject(warm.clone()).await?;
    }
    println!("FLOOD_START knob=vendorstate dwell={dwell}");

    for arm in [0u8, 1, 0, 1, 0, 1] {
        let set: &[(u16, u32)] = if arm == 1 { VENDOR } else { OURS };
        for &(a, v) in set {
            dev.write32(a, v)?;
        }
        println!("IDX {arm:02x}");
        let mut p = vec![0xC3u8; 300];
        p[0] = arm;
        p[1] = 7; // knob id for the bucketing receiver
        let f = InjectFrame {
            payload: Bytes::from(p), tx: TxIntent::CONSERVATIVE, dst: BROADCAST,
            src: [0x02, 0x56, 0x54, 0x58, arm, 0x01], addr3: None,
        };
        let end = std::time::Instant::now() + std::time::Duration::from_secs(dwell);
        let mut sent = 0u64;
        while std::time::Instant::now() < end {
            dev.inject(f.clone()).await?;
            sent += 1;
        }
        // Read one back so a rejected/clobbered write is visible rather than assumed to have stuck.
        let rb = dev.read32(0x4318).unwrap_or(0);
        println!("  arm {arm} ({}) sent={sent} rate={}/s  0x4318={rb:08x} tssi={}",
            if arm == 1 { "VENDOR" } else { "ours" }, sent / dwell.max(1), (rb >> 28) & 0x7);
    }
    println!("SWEEP_DONE");
    Ok(())
}
