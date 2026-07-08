//! Test the CORRECT TX power knob: 0x18a0[6:0] = absolute_ofdm_swing_idx (what the vendor
//! power-tracking writes). Set it to env SWING after enable_tx and measure radiation — if
//! higher swing radiates reliably, my base power was marginal and this is the fix.
use ndn_radio_drivers::Rtl8733buBackend;
use std::time::{Duration, Instant};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(36);
    let sw: u32 = std::env::var("SWING").ok().and_then(|s| u32::from_str_radix(&s, 16).ok()).unwrap_or(0xff);
    let d = Rtl8733buBackend::open()?;
    d.bring_up_monitor(ch)?;
    d.enable_tx(ch)?;
    let orig = d.read32(0x18a0)? & 0x7f;
    if sw != 0xff {
        let v = d.read32(0x18a0)?;
        d.write32(0x18a0, (v & !0x7f) | (sw & 0x7f))?;
    }
    println!("SWING orig=0x{:02x} now=0x{:02x} die={}", orig, d.read32(0x18a0)? & 0x7f, d.read_thermal().unwrap_or(0));
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
