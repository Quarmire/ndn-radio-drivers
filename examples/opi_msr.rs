//! A2: activate a MACID via the media-status H2C (cmd 0x01) so the firmware manages its TX,
//! then inject. Tests whether firmware-managed MACID state fixes the ~50% boot variance.
//! MSR=1 sends the H2C; MSRMACID/ROLE tune it. A/B against MSR unset.
use ndn_radio_drivers::Rtl8733buBackend;
use std::time::{Duration, Instant};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(36);
    let macid: u8 = std::env::var("MSRMACID").ok().and_then(|s| s.parse().ok()).unwrap_or(0);
    let role: u8 = std::env::var("ROLE").ok().and_then(|s| s.parse().ok()).unwrap_or(0);
    let d = Rtl8733buBackend::open()?;
    d.bring_up_monitor(ch)?;
    d.enable_tx(ch)?;
    if std::env::var("MSR").is_ok() {
        // opmode=connect(bit0) | role<<4, macid, macid_end
        d.send_h2c_box(0x01, &[0x01 | (role << 4), macid, macid])?;
        println!("media-status H2C sent: macid={macid} role={role}");
    }
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
