//! Validate the RadioTime reference impl: enumerate the radio's link clocks, and read the
//! readable one (port TSF) vs the latch-only one (RX stamp -> None).
use ndn_radio_drivers::Rtl8733buBackend;
use ndn_radio_hal::RadioTime;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let d = Rtl8733buBackend::open()?;
    d.bring_up_monitor(36)?;
    let srcs = d.time_sources();
    for s in &srcs {
        println!("{:?} domain={:?} monotonic={} read_now={} tick_ns={}",
            s.kind, s.domain, s.monotonic, s.read_now, s.tick_ns);
    }
    d.set_tsf_run(true)?;
    let rx_dom = srcs[0].domain;
    let port_dom = srcs[1].domain;
    println!("read_clock(FreeRunRxStamp) = {:?} (expect None)", d.read_clock(rx_dom)?);
    println!("read_clock(PortTsf) = {:?} (expect Some)", d.read_clock(port_dom)?);
    Ok(())
}
