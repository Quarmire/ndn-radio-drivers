//! Read the 8733b's **per-PPDU-format BB RX counters** — the PHY's own verdict on what it saw,
//! upstream of the MAC, the RX descriptor and every line of our parsing code.
//!
//! Transcribed from the vendor driver's `hw_dump_bb_rx_cnt` (`hal/rtl8733b/rtl8733b_ops.c`). Each
//! register packs OK in the low half and ERR in the high half:
//!
//!   0x2c08  CCA   cck (lo) / ofdm (hi)      0x2c04  CCK  ok/err
//!   0x2c14  OFDM  ok/err                    0x2c10  HT   ok/err
//!   0x2c0c  VHT   ok/err                    0x2db4  BB state machine
//!
//! Why this matters for the VHT question: "the DUT delivered no VHT frames" cannot distinguish
//!   (a) the BB never recognises the VHT preamble and mis-detects it as legacy OFDM,
//!   (b) the BB recognises VHT and fails to demodulate it,
//!   (c) the BB decodes it and something above the PHY drops it.
//! These counters separate all three: (a) = vht_ok/err flat while ofdm_err climbs, (b) = vht_err
//! climbs, (c) = vht_ok climbs. Run it while `inject_ht <if> vht0` floods the channel.
//!
//!   sudo ./bb_rx_cnt8733b [channel] [seconds]

use ndn_radio_drivers::Rtl8733buBackend;

const REGS: &[(&str, u16)] = &[
    ("CCA  cck/ofdm", 0x2c08),
    ("CCK  ok/err", 0x2c04),
    ("OFDM ok/err", 0x2c14),
    ("HT   ok/err", 0x2c10),
    ("VHT  ok/err", 0x2c0c),
];

fn snap(dev: &Rtl8733buBackend) -> Vec<(u16, u16)> {
    REGS.iter()
        .map(|&(_, a)| {
            let v = dev.read32(a).unwrap_or(0);
            ((v & 0xffff) as u16, (v >> 16) as u16)
        })
        .collect()
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(36);
    let secs: u64 = std::env::args()
        .nth(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(20);

    let dev = Rtl8733buBackend::open()?;
    dev.bring_up_monitor(ch)?;
    println!("8733b BB RX counters on ch{ch}, {secs}s — deltas per second");
    println!(
        "BB state machine 0x2db4 = 0x{:08x}",
        dev.read32(0x2db4).unwrap_or(0)
    );

    // The counters are free-running 16-bit and wrap; take deltas with wrapping_sub so a wrap shows
    // as a plausible small delta rather than a 65k spike.
    let mut prev = snap(&dev);
    for t in 1..=secs {
        std::thread::sleep(std::time::Duration::from_secs(1));
        let now = snap(&dev);
        let mut line = format!("t={t:>3}s ");
        for (i, &(name, _)) in REGS.iter().enumerate() {
            let d0 = now[i].0.wrapping_sub(prev[i].0);
            let d1 = now[i].1.wrapping_sub(prev[i].1);
            line.push_str(&format!("| {name} {d0:>5}/{d1:<5} "));
        }
        println!("{line}");
        prev = now;
    }
    Ok(())
}
