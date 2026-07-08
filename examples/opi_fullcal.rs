//! Full calibration chain + inject. Runs efuse-trim, rfk_init (KIP microcode), IQK
//! (phy_iq_calibrate), TXGAPK (phy_txgapk), DPK (phy_dpk) — each gateable via env
//! (SKIP_IQK/SKIP_TXGAPK/SKIP_DPK) to bisect — then injects. Score on wlu1 by src MAC.
use ndn_radio_drivers::Rtl8733buBackend;
use std::time::{Duration, Instant};

// Vendor final values for the TXAGC/datapath regs the cal leaves zeroed/un-restored.
const DATAPATH: &[(u16, u32)] = &[
    (0x180c, 0x17f43863), (0x18ac, 0x00065a60), (0x1968, 0x36632640),
    (0x1c38, 0xffb5005e), (0x1c3c, 0x01051f43), (0x1c80, 0x0f38e000), (0x1c84, 0x24512054),
    (0x1ca4, 0xe0000000), (0x1d70, 0x2020201c), (0x1e1c, 0x8400b000),
    (0x1e40, 0xfffeffff), (0x1e44, 0x2824201c), (0x1e48, 0x3834302c), (0x1e50, 0x2824201c),
    (0x1e54, 0x3834302c), (0x1e58, 0xfe44403c), (0x1e5c, 0xc13c00ff), (0x1e60, 0x4440413f),
    (0x1e88, 0x0000fc1c), (0x1e8c, 0x00007000), (0x1eb8, 0x00000b00),
    (0x1ed4, 0x800c0040), (0x1ed8, 0x8005000c), (0x1edc, 0x80020005), (0x1ee0, 0x80000002),
    (0x1ee4, 0xf0000000), (0x1ef0, 0x30000a80), (0x1ef4, 0x40001266), (0x1ef8, 0x3b000100),
];

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
    // apply the vendor TXAGC/datapath the cal leaves zeroed (0x1e40-0x1e60 etc.)
    if std::env::var("NODP").is_err() {
        for &(a, v) in DATAPATH { d.write32(a, v)?; }
        d.tune_channel(ch)?;
        for &(a, v) in DATAPATH { d.write32(a, v)?; }
    }
    d.write8(0x0522, 0x00)?;
    // grant the shared RF to WiFi (phy_set_rf_path_switch main: GNT_WL=1,GNT_BT=0)
    if std::env::var("NOGNT").is_err() {
        let v = d.read32(0x70)?;
        d.write32(0x70, (v & !0xF000_0000) | (1 << 26) | (0x9 << 28))?;
    }
    if std::env::var("TSSI").is_ok() { d.tssi_setup(ch)?; }
    println!("post-cal: RF01=0x{:05x} RF18=0x{:05x} 0x70=0x{:08x}", d.rf_read(0x01)?, d.rf_read(0x18)?, d.read32(0x70)?);
    if std::env::var("DUMP").is_ok() {
        use std::io::Write;
        let mut df = std::fs::File::create("/tmp/mycal_bb.txt")?;
        for a in (0x1800..0x2000u16).step_by(4) { writeln!(df, "0x{a:04x} 0x{:08x}", d.read32(a).unwrap_or(0))?; }
        for a in 0..0x80u32 { writeln!(df, "R{a:02x} {:05x}", d.rf_read(a).unwrap_or(0) & 0xfffff)?; }
        println!("dumped /tmp/mycal_bb.txt");
    }
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
