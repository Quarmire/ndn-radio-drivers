//! Override my RF PA/gain registers to the vendor's radiating values (from read_rfreg diff)
//! after enable_tx, then inject. If it radiates reliably, the per-boot dead condition was the
//! PA-bias RF state landing wrong. RFSET env picks the register subset.
use ndn_radio_drivers::Rtl8733buBackend;
use std::time::{Duration, Instant};

// (addr, vendor value) — the RF regs where mine != vendor (excl dynamic 0x01 TXAGC / 0x42 therm)
const RF: &[(u32, u32)] = &[
    (0x21, 0x1d600), (0x23, 0xafd40), (0x25, 0x403c4), (0x26, 0xccccc), (0x3f, 0x10808),
    (0x41, 0x00001), (0x51, 0xbd280), (0x55, 0x00080), (0x56, 0x86065), (0x5d, 0x04087),
    (0x5f, 0x06065), (0x60, 0x33000), (0x7f, 0x00100),
];
// PA-only subset (RFSET=pa)
const PA: &[(u32, u32)] = &[(0x55, 0x00080), (0x56, 0x86065), (0x5f, 0x06065), (0x60, 0x33000), (0x5d, 0x04087)];

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(36);
    let set = std::env::var("RFSET").unwrap_or_default();
    let d = Rtl8733buBackend::open()?;
    d.bring_up_monitor(ch)?;
    d.enable_tx(ch)?;
    let regs: &[(u32,u32)] = if set == "pa" { PA } else { RF };
    for &(a, v) in regs { d.rf_write_full(a, v)?; }
    println!("applied {} RF regs; 0x60={:05x}", regs.len(), d.rf_read(0x60)?);
    let mut f = vec![0x08u8, 0, 0, 0];
    f.extend_from_slice(&[0xff; 6]);
    f.extend_from_slice(&[0x02, 0x4d, 0x59, 0x44, 0x52, 0x56]);
    f.extend_from_slice(&[0x02, 0x4d, 0x59, 0x44, 0x52, 0x56]);
    f.extend_from_slice(&[0, 0]);
    f.extend_from_slice(b"MYDRV8733-INJECT");
    let deadline = Instant::now() + Duration::from_secs(6);
    let mut s = 0u16;
    while Instant::now() < deadline {
        let _ = d.inject_raw(&f, 0x04, s);
        s = s.wrapping_add(1);
        std::thread::sleep(Duration::from_millis(15));
    }
    println!("sent {s}");
    Ok(())
}
