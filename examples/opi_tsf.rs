//! TSF primitives: the always-on per-frame RX stamp is the primary clock (see opi_rxmeta).
//! read_tsf reads the port-0 beacon TSF, which only advances with set_tsf_run(true).
use ndn_radio_drivers::Rtl8733buBackend;
use std::time::Duration;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(36);
    let d = Rtl8733buBackend::open()?;
    d.bring_up_monitor(ch)?;
    println!("domain={:?}", d.tsf_domain());
    println!("port TSF (passive): {}", d.read_tsf()?);
    d.set_tsf_run(true)?;
    let a = d.read_tsf()?;
    std::thread::sleep(Duration::from_millis(100));
    let b = d.read_tsf()?;
    println!("port TSF running: {a} -> {b} (delta {})", b.wrapping_sub(a));
    Ok(())
}
