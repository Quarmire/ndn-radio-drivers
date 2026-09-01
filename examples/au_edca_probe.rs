//! **Does the RTL8812AU inherit contention state from the previous process?**
//!
//! The MT7610U did: it never wrote the EDCA registers at bring-up, and nothing power-cycles a USB
//! chip between runs, so each process silently inherited the previous one's posture — MEASURED as a
//! 2.5x throughput swing decided purely by run order. `src/rtl8812au.rs` has the same two
//! preconditions on paper (no EDCA write anywhere in the file; `power_on` does NOT bring the MAC
//! down first — its own doc says the card-emulation step is "not auto-invoked").
//!
//! Throughput is the wrong instrument for this question: on a contended channel it varied 22x
//! between identical runs and told us nothing. So read the registers instead.
//!
//!   sudo -E ./au_edca_probe [channel]
//! env: NDN_POSTURE=owned|shared|yielding — apply a posture AFTER reporting the as-found state.
use ndn_radio_drivers::Rtl8812auBackend;
use ndn_radio_hal::RadioKnobs;

const REGS: [(&str, u16); 4] = [
    ("VO 0x0500", 0x0500),
    ("VI 0x0504", 0x0504),
    ("BE 0x0508", 0x0508),
    ("BK 0x050c", 0x050c),
];

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(36);
    let dev = Rtl8812auBackend::open()?;
    dev.bring_up_monitor(ch)?;

    // The whole point: read these AFTER a full bring-up and BEFORE this process expresses any
    // opinion. Whatever is here was put here by something other than this process.
    print!("as-found after bring_up:");
    for (name, reg) in REGS {
        print!(" {name}={:08x}", dev.read32(reg)?);
    }
    println!(" slot=0x{:02x}", dev.read8(0x051b)?);

    if let Ok(v) = std::env::var("NDN_POSTURE") {
        use ndn_radio_hal::ContentionPosture;
        let p = match v.trim().to_ascii_lowercase().as_str() {
            "owned" => ContentionPosture::Owned,
            "yielding" => ContentionPosture::Yielding,
            _ => ContentionPosture::Shared,
        };
        match RadioKnobs::set_contention(&dev, p) {
            Ok(a) => println!("applied {p:?} -> aifs {} cw {}..{}", a.aifs, a.cw_min, a.cw_max),
            Err(e) => println!("posture {p:?} NOT applied: {e}"),
        }
        print!("after apply:            ");
        for (name, reg) in REGS {
            print!(" {name}={:08x}", dev.read32(reg)?);
        }
        println!(" slot=0x{:02x}", dev.read8(0x051b)?);
    }
    Ok(())
}
