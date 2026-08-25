//! One tick of the closed frequency-discipline loop, on real hardware.
//!
//! Usage: `freqdisc8733b <measured_skew_ppm>` — where the measurement comes from an INDEPENDENT
//! two-node common-view comparison (`rxseq8733b` + the analysis), not from this radio's own idea of
//! time. Sense and actuate must not share an instrument, or the loop measures its own opinion.
//!
//! Seeds the controller from what the hardware is ALREADY expressing (current cap vs the efuse
//! factory cap), because the trim survives process restarts and a false zero would make the first
//! correction of every run wrong.
use ndn_radio_drivers::{FreqDiscipline, Rtl8733buBackend};
use std::sync::Arc;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let skew: f32 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .ok_or("usage: freqdisc8733b <measured_skew_ppm>")?;
    let dev = Arc::new(Rtl8733buBackend::open()?);
    let factory = dev.factory_crystal_cap()?;
    let cur = dev.crystal_cap()?;
    let already = Rtl8733buBackend::xtal_ppm_at_cap(f32::from(cur))
        - Rtl8733buBackend::xtal_ppm_at_cap(f32::from(factory));
    let mut d = FreqDiscipline::new(dev.clone())
        .ok_or("radio advertises no clock steering")?
        .with_applied_ppm(already);
    println!(
        "factory cap {factory}, current cap {cur} => hardware already at {already:+.3} ppm; \
         limits +-{:.1} ppm, step {:.2} ppm",
        d.limits().range_ppm,
        d.limits().resolution_ppm
    );
    let action = d.update(skew)?;
    println!("measured skew {skew:+.3} ppm -> {action:?}");
    println!("now commanded {:+.3} ppm (cap {})", d.applied_ppm(), dev.crystal_cap()?);
    Ok(())
}
