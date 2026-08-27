//! Dump the 8733b's live BB register state after a TX bring-up, for diffing against the vendor's
//! expected values.
//!
//! Seven independent TX-power controls have now measured inert on air (reference 0x4308, per-rate
//! table 0x3a00, datapath 0x1e4x, RF 0x01, swing 0x18a0, TSSI already off, and the per-frame
//! descriptor TXPWR_OFSET). They share a signature: the digital chain accepts every write and the
//! analog output does not follow. That points at BB/RF state rather than at any single knob, so the
//! next move is a state comparison, not another sweep.
//!
//! Prints `addr value` per line over the TX-relevant ranges so it can be diffed mechanically against
//! (a) the vendor's static `phy_reg` table, which this repo ships, and (b) — when the dongle and the
//! vendor driver are on the same host — a live `/proc/net/rtl8733bu/<dev>/bb_reg_dump`.
//!
//!   sudo ./bb_dump8733b [channel] > ours.txt

use ndn_radio_drivers::Rtl8733buBackend;

/// TX-relevant BB windows. Deliberately not the whole map: each read is a USB control transfer, and
/// these cover the PHY/TX config, the AGC tables, the datapath block and the per-rate/ref TXAGC.
const RANGES: &[(u16, u16)] = &[
    (0x0000, 0x07ff), // MAC space — the last region of the vendor capture never diffed
    (0x0800, 0x09ff), // BB core / OFDM front-end
    (0x0c00, 0x0dff), // RF interface, LSSI
    (0x1800, 0x1aff), // RF mode table, swing, AGC
    (0x1c00, 0x1eff), // datapath, TXAGC block
    (0x2a00, 0x2aff), // CCK
    (0x3a00, 0x3aff), // per-rate TXAGC table
    (0x4300, 0x43ff), // TXAGC reference / TSSI control
];

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(36);
    let dev = Rtl8733buBackend::open()?;
    // Full TX bring-up: the point is to capture the state the transmitter actually runs in, which
    // includes everything calibration and `enable_tx` leave behind — not a freshly-initialised BB.
    dev.bring_up_tx(ch)?;
    eprintln!("bb_dump8733b: ch{ch}, synth_locked={}", dev.synth_locked());
    for &(lo, hi) in RANGES {
        let mut a = lo;
        while a <= hi {
            if let Ok(v) = dev.read32(a) {
                println!("{a:04x} {v:08x}");
            }
            a = a.saturating_add(4);
            if a == 0 {
                break;
            }
        }
    }
    Ok(())
}
