//! **What contention state does the MT7610U actually run, and where does it come from?**
//!
//! Two questions this answers on silicon rather than by reading code:
//!
//!  1. **As-found, BEFORE our bring-up.** Whatever is here was left by the previous owner — the
//!     kernel `mt76x0u` driver, or a previous run of ours. This is the read that proved the
//!     cross-process EDCA leak: nothing power-cycles a USB chip between processes.
//!  2. **After bring-up**, which now pins EDCA to `Shared`. Confirms the write landed, and that
//!     `ACKTO` is still the value the init table programmed rather than another chip's constant.
//!
//!   sudo -E ./mt7610_edca_probe
use ndn_radio_drivers::Mt7610uBackend;

const REGS: [(&str, u32); 8] = [
    ("WMM_AIFSN 0x0214", 0x0214),
    ("WMM_CWMIN 0x0218", 0x0218),
    ("WMM_CWMAX 0x021c", 0x021c),
    ("EDCA_AC0  0x1300", 0x1300),
    ("EDCA_AC1  0x1304", 0x1304),
    ("BKOFF_SLOT 0x1104", 0x1104),
    ("TX_TIMEOUT 0x1348", 0x1348),
    ("XIFS_TIME  0x1100", 0x1100),
];

fn dump(dev: &Mt7610uBackend, label: &str) {
    println!("{label}");
    for (name, reg) in REGS {
        match dev.rr(reg) {
            Ok(v) => println!("    {name} = {v:#010x}"),
            Err(e) => println!("    {name} = <read failed: {e}>"),
        }
    }
    if let Ok(t) = dev.rr(0x1348) {
        println!("    -> ACKTO = {} us", (t >> 8) & 0xff);
    }
    if let Ok(x) = dev.rr(0x1100) {
        println!("    -> OFDM_SIFS = {} us", (x >> 8) & 0xff);
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let dev = Mt7610uBackend::open()?;
    // No bring_up yet: this is the chip exactly as the last owner left it.
    dump(&dev, "AS FOUND (before bring_up — the previous owner's state):");
    dev.bring_up()?;
    dump(&dev, "\nAFTER bring_up (EDCA pinned to Shared):");
    Ok(())
}
