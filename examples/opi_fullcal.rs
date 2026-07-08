//! Full calibration chain + inject. Runs efuse-trim, rfk_init (KIP microcode), IQK
//! (phy_iq_calibrate), TXGAPK (phy_txgapk), DPK (phy_dpk) — each gateable via env
//! (SKIP_IQK/SKIP_TXGAPK/SKIP_DPK) to bisect — then injects. Score on wlu1 by src MAC.
use ndn_radio_drivers::Rtl8733buBackend;
use std::time::{Duration, Instant};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(36);
    let secs: u64 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(12);
    let d = Rtl8733buBackend::open()?;
    d.bring_up_monitor(ch)?;
    println!("trim: {:?}", d.apply_efuse_trim());
    d.rfk_init()?;
    if std::env::var("SKIP_IQK").is_err() { println!("iqk: {:?}", d.phy_iq_calibrate().map(|_| "ok")); }
    if std::env::var("SKIP_TXGAPK").is_err() { println!("txgapk: {:?}", d.phy_txgapk()); }
    if std::env::var("SKIP_DPK").is_err() { println!("dpk: {:?}", d.phy_dpk()); }
    d.write8(0x0522, 0x00)?;
    // grant the shared RF to WiFi (phy_set_rf_path_switch main: GNT_WL=1,GNT_BT=0)
    if std::env::var("NOGNT").is_err() {
        let v = d.read32(0x70)?;
        d.write32(0x70, (v & !0xF000_0000) | (1 << 26) | (0x9 << 28))?;
    }
    if std::env::var("TSSI").is_ok() { d.tssi_setup(ch)?; }
    println!("post-cal: RF01=0x{:05x} RF18=0x{:05x} 0x70=0x{:08x}", d.rf_read(0x01)?, d.rf_read(0x18)?, d.read32(0x70)?);
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
