//! RF-grant test: my bring-up never grants the shared RF to WiFi. phy_set_rf_path_switch /
//! btc_set_gnt_wl_bt force GNT_WL=1,GNT_BT=0 via MAC 0x70[31:28]=0x9 + bit26 (SW-ctrl enable).
//! Set it after bring-up, then inject. Capture on wlu1, score src 02:4d:59:44:52:56.
use ndn_radio_drivers::Rtl8733buBackend;
use std::time::{Duration, Instant};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(36);
    let secs: u64 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(12);
    let with_tssi = std::env::var("TSSI").is_ok();
    let d = Rtl8733buBackend::open()?;
    d.bring_up_monitor(ch)?;
    let before = d.read32(0x70)?;
    // GNT_WL=1, GNT_BT=0 + SW-control enable (bit26)
    let nv = (before & !0xF000_0000) | (1 << 26) | (0x9 << 28);
    d.write32(0x70, nv)?;
    if with_tssi { d.tssi_setup(ch)?; }
    d.write8(0x0522, 0x00)?;
    println!("GNT 0x70: before=0x{:08x} after=0x{:08x} (want [31:28]=9,bit26=1)", before, d.read32(0x70)?);
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
    println!("sent {s}");
    Ok(())
}
