//! **Does the MT7612U inherit contention state from the previous process?** — the MT7612U sibling
//! of `examples/mt7610_edca_probe.rs`.
//!
//! `bring_up` now calls `restore_edca_defaults()`, but that was written and shipped without an
//! on-air or on-register check. This answers two questions in one run, at the registers, where
//! there is no ambiguity:
//!
//!  1. **AS FOUND, before our bring-up** — whatever is here was left by the previous owner. USB
//!     never power-cycles a dongle between processes, so if a prior run set an aggressive posture
//!     and this read shows it, the leak is real on this part, not just on the MT7610U (where it
//!     MEASURED a 2.5x throughput swing decided purely by run order).
//!  2. **After bring-up** — must be the boot window regardless of (1). That is the fix working.
//!
//! ⚠ This part needs a COLD device; hand it over with `mt76_acquire.sh acquire 7612` rather than
//! letting libusb auto-detach the kernel driver, which powers the chip down under us.
//!
//!   sudo -E ./mt7612_edca_probe
use ndn_radio_drivers::Mt7612uBackend;

/// mt76x02 EDCA block: the per-AC WMM words and the four `MT_EDCA_CFG_AC(n)` registers.
const REGS: [(&str, u32); 7] = [
    ("WMM_AIFSN 0x0214", 0x0214),
    ("WMM_CWMIN 0x0218", 0x0218),
    ("WMM_CWMAX 0x021c", 0x021c),
    ("EDCA_AC0  0x1300", 0x1300),
    ("EDCA_AC1  0x1304", 0x1304),
    ("BKOFF_SLOT 0x1104", 0x1104),
    ("TX_TIMEOUT 0x1348", 0x1348),
];

fn dump(dev: &Mt7612uBackend, label: &str) {
    println!("{label}");
    for (name, reg) in REGS {
        match dev.rr(reg) {
            Ok(v) => println!("    {name} = {v:#010x}"),
            Err(e) => println!("    {name} = <read failed: {e}>"),
        }
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let dev = Mt7612uBackend::open()?;
    // No bring_up yet: the chip exactly as the last owner left it.
    dump(&dev, "AS FOUND (before bring_up — the previous owner's state):");
    dev.bring_up()?;
    dump(&dev, "\nAFTER bring_up (must be the boot window, whatever (1) showed):");
    Ok(())
}
