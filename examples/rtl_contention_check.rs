//! Verify the contention knob actuates on the Realtek backends, by register read-back.
//!
//! A companion to `rtl_contention_ab`, which measures the *effect* on the 8812au. This one only
//! establishes that the register lands correctly on the other backends sharing
//! [`ndn_radio_drivers::realtek_contention`] — a knob that returns `Ok` without changing silicon
//! is the `decided-but-unactuated` defect, and reading the register back is the only thing that
//! rules it out. Both the applied report and the four AC words are printed, because a driver that
//! lies in the same direction in both is the failure this cannot catch any other way.
//!
//! ```text
//! rtl_contention_check a81a     # LibUsbRtl88xxBackend
//! rtl_contention_check f72b     # Rtl8733buBackend
//! ```
use ndn_radio_drivers::realtek_contention::EDCA_REGS;
use ndn_radio_drivers::{LibUsbRtl88xxBackend, Rtl8733buBackend};
use ndn_radio_hal::{ContentionPosture, FaceError, RadioKnobs};

const POSTURES: [ContentionPosture; 4] = [
    ContentionPosture::Shared,
    ContentionPosture::Owned,
    ContentionPosture::Yielding,
    // Back to Shared last: the final row must reproduce the first exactly, or "restore" is a
    // guess. On the a81a the four ACs boot with *different* AIFS and TXOP, so this also proves
    // the saved snapshot is per-AC rather than one hardcoded stock word.
    ContentionPosture::Shared,
];

fn sweep<D>(dev: &D, rd32: impl Fn(u16) -> Result<u32, FaceError>) -> Result<(), FaceError>
where
    D: RadioKnobs,
{
    for posture in POSTURES {
        let a = dev.set_contention(posture)?;
        println!(
            "{posture:>10?}: cw {}..{} aifsn {} slot {} us => backoff {:>3} us, medium access {:>3} us",
            a.cw_min,
            a.cw_max,
            a.aifs,
            a.slot_us,
            a.avg_backoff_us,
            a.medium_access_us()
        );
        let mut line = String::from("            ");
        for (name, reg) in ["VO", "VI", "BE", "BK"].iter().zip(EDCA_REGS) {
            line.push_str(&format!("{name} {:#010x}  ", rd32(reg)?));
        }
        println!("{line}");
    }
    Ok(())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let arg = std::env::args().nth(1).unwrap_or_else(|| "a81a".into());
    let pid = u16::from_str_radix(arg.trim_start_matches("0x"), 16)?;
    println!("Realtek {pid:#06x} contention read-back\n");
    if pid == 0xf72b {
        let d = Rtl8733buBackend::open()?;
        sweep(&d, |r| d.read32(r))?;
    } else {
        let d = LibUsbRtl88xxBackend::open_pid(pid)?;
        sweep(&d, |r| d.read32(r))?;
    }
    Ok(())
}
