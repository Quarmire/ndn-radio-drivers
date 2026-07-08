//! FW-coordinated RF grant: send the RFK-start H2C (0x6d param=1 = ODM_H2C_WIFI_CALIBRATION)
//! to tell the FW "WiFi owns the RF", force GNT_WL (0x70[31:28]=9 + bit26, and 0x73[7:4]=9),
//! then inject. Never send the RFK-end, so the FW keeps hands off. Re-read the grant after
//! injecting to detect FW override. TSSI=1 also runs tssi_setup.
use ndn_radio_drivers::Rtl8733buBackend;
use std::time::{Duration, Instant};

fn send_h2c(d: &Rtl8733buBackend, main: u32, ext: u32) -> Result<(), Box<dyn std::error::Error>> {
    for _ in 0..200 { if d.read32(0x1cc)? & 1 == 0 { break; } std::thread::sleep(Duration::from_millis(1)); }
    d.write32(0x1f0, ext)?; // ext box
    d.write32(0x1d0, main)?; // main box (triggers)
    std::thread::sleep(Duration::from_millis(5));
    Ok(())
}

fn set_gnt_wl(d: &Rtl8733buBackend) -> Result<(), Box<dyn std::error::Error>> {
    let v = d.read32(0x70)?;
    d.write32(0x70, (v & !0xF000_0000) | (1 << 26) | (0x9 << 28))?; // GNT_WL=1,GNT_BT=0 + SW ctrl
    let b = d.read32(0x73 & !3)?; // 0x70 dword; nibble 0x73[7:4] == 0x70[31:28] already set
    let _ = b;
    Ok(())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(36);
    let secs: u64 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(12);
    let d = Rtl8733buBackend::open()?;
    d.bring_up_monitor(ch)?;
    send_h2c(&d, 0x6d | (1 << 8), 0)?; // RFK start: WiFi owns RF
    set_gnt_wl(&d)?;
    if std::env::var("TSSI").is_ok() { d.tssi_setup(ch)?; }
    set_gnt_wl(&d)?; // re-assert after tssi
    d.write8(0x0522, 0x00)?;
    println!("after setup: 0x70=0x{:08x} HMETFR=0x{:08x}", d.read32(0x70)?, d.read32(0x1cc)?);
    let mut f = vec![0x08u8, 0, 0, 0];
    f.extend_from_slice(&[0xff; 6]);
    f.extend_from_slice(&[0x02, 0x4d, 0x59, 0x44, 0x52, 0x56]);
    f.extend_from_slice(&[0x02, 0x4d, 0x59, 0x44, 0x52, 0x56]);
    f.extend_from_slice(&[0, 0]);
    f.extend_from_slice(b"MYDRV8733-INJECT");
    let deadline = Instant::now() + Duration::from_secs(secs);
    let mut s = 0u16;
    while Instant::now() < deadline {
        let _ = d.inject_raw(&f, 0x04, s);
        s = s.wrapping_add(1);
        std::thread::sleep(Duration::from_millis(15));
    }
    println!("sent {s}; 0x70 after inject=0x{:08x}", d.read32(0x70)?);
    Ok(())
}
