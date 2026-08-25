//! Does the crystal trim actually change the clock RATE?
//!
//! The first attempt tried to answer this with the chip's CFO register and failed for an
//! instructive reason: this part's PROTOCOL is 802.11n but its BASEBAND IP is Jaguar-3
//! (`ODM_IC_JGR3_1SS = ODM_RTL8733B`), and the vendor's `phydm_get_cfo_info` has no JGR3 case at
//! all — it reads no CFO here. The 11n CFO registers returned a frozen -250344 Hz across a full
//! cap sweep, which is what reading the wrong generation's register file looks like.
//!
//! So measure the actuator against a sensor that DOES exist on this chip: the TSF counter is
//! driven by the same crystal, so trimming the crystal must change TSF ticks per host second.
//! Needs no traffic, no second radio, and no CFO.
//!
//! Method: at each cap, regress TSF against the host monotonic clock over a window (many samples,
//! so USB read jitter averages down rather than dominating a single pair), and report the slope in
//! ppm relative to the power-on cap.
use ndn_radio_drivers::Rtl8733buBackend;
use std::time::Instant;

fn slope_ppm(dev: &Rtl8733buBackend, secs: f64, n: usize) -> Result<f64, Box<dyn std::error::Error>> {
    let t0 = Instant::now();
    let mut xs = Vec::with_capacity(n);
    let mut ys = Vec::with_capacity(n);
    let gap = std::time::Duration::from_secs_f64(secs / n as f64);
    for _ in 0..n {
        let host = t0.elapsed().as_secs_f64();
        let tsf = dev.read_tsf()? as f64 * 1e-6; // TSF is microseconds
        xs.push(host);
        ys.push(tsf);
        std::thread::sleep(gap);
    }
    let m = xs.len() as f64;
    let mx = xs.iter().sum::<f64>() / m;
    let my = ys.iter().sum::<f64>() / m;
    let num: f64 = xs.iter().zip(&ys).map(|(a, b)| (a - mx) * (b - my)).sum();
    let den: f64 = xs.iter().map(|a| (a - mx).powi(2)).sum();
    Ok(num / den)
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(11);
    let secs: f64 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(8.0);
    let dev = Rtl8733buBackend::open()?;
    dev.bring_up_monitor(ch)?;
    // The port-0 TSF is GATED by default — the first run of this test regressed a frozen counter
    // and produced a confident 26603 ppm/step out of dividing by ~zero. Start it, then VERIFY it
    // runs before any ppm is computed.
    dev.set_tsf_run(true)?;
    std::thread::sleep(std::time::Duration::from_millis(200));
    let base = dev.crystal_cap()?;
    println!("ch{ch}  power-on crystal cap = {base}  ({secs}s regression per arm)");
    println!("{:>5} {:>6} {:>14} {:>12}", "cap", "rdbk", "TSF/host slope", "ppm vs base");

    let mut rows = vec![];
    let mut base_slope = None;
    // Last arm repeats the first: without it, drift over the ~60 s sweep is indistinguishable
    // from curvature in the trim. (First run omitted it and showed a suspicious flat segment.)
    for d in [0i32, -48, -32, -16, 16, 32, 47, 0] {
        let cap = (base as i32 + d).clamp(0, 127) as u8;
        dev.set_crystal_cap(cap)?;
        std::thread::sleep(std::time::Duration::from_millis(300));
        let s = slope_ppm(&dev, secs, 60)?;
        if base_slope.is_none() {
            // The counter must be ADVANCING for any ppm below to mean anything; a frozen one
            // produced a confident 26603 ppm/step on the first run. It need not advance at 1.0:
            // MEASURED 0.250008796 on this part, i.e. read_tsf's unit is 4 us, not 1 us. That
            // scale cancels here because every ppm is relative to this same base slope.
            println!("(TSF base slope {s:.9} tsf-units per host us => unit is {:.2} us)", 1.0 / s);
            if !(0.05..2.0).contains(&s) {
                println!("TSF slope {s:.9} is not advancing plausibly; refusing to report ppm \
                          (this is what produced 26603 ppm/step before).");
                dev.set_crystal_cap(base)?;
                dev.set_tsf_run(false)?;
                return Ok(());
            }
            base_slope = Some(s);
        }
        let ppm = (s / base_slope.unwrap() - 1.0) * 1e6;
        println!("{:>5} {:>6} {:>14.9} {:>12.2}", cap, dev.crystal_cap()?, s, ppm);
        rows.push((cap as f64, ppm));
        if rows.len() == 8 {
            println!("  ^ return-to-baseline arm: {:.2} ppm from the first (drift over the sweep)", ppm);
        }
    }
    dev.set_crystal_cap(base)?;
    dev.set_tsf_run(false)?;
    println!("restored cap = {}", dev.crystal_cap()?);
    let rows: Vec<(f64, f64)> = rows[..rows.len() - 1].to_vec();
    let n = rows.len() as f64;
    let mx = rows.iter().map(|r| r.0).sum::<f64>() / n;
    let my = rows.iter().map(|r| r.1).sum::<f64>() / n;
    let num: f64 = rows.iter().map(|r| (r.0 - mx) * (r.1 - my)).sum();
    let den: f64 = rows.iter().map(|r| (r.0 - mx).powi(2)).sum();
    if den > 0.0 {
        println!("\nslope = {:.3} ppm per cap step over {} steps", num / den, 95);
    }
    Ok(())
}
