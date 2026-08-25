//! Validate the per-frame PHY metrics (SNR / EVM / CFO) parsed out of the Jaguar-3 status block.
//!
//! The sharp test is CFO, because we can move it on purpose: retune OUR OWN crystal and the
//! measured offset to a fixed transmitter must shift by the same amount. SNR and EVM should NOT
//! move — they describe the link, not our clock. So this checks the parse and demonstrates the
//! frequency-discipline loop (measure offset per frame -> trim crystal to null it) in one run.
//!
//! Also prints the drvinfo size, because the whole thing is inert unless the chip reports >= 28
//! bytes of status: RSSI only needs byte 1, these fields live at 16/20/24.
use ndn_radio_drivers::{FrameIo, Rtl8733buBackend};
use std::sync::Arc;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(11);
    let per: usize = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(120);
    let dev = Arc::new(Rtl8733buBackend::open()?);
    dev.bring_up_monitor(ch)?;
    let _pump = dev.spawn_rx_pump(4);
    let base = dev.crystal_cap()?;
    println!("ch{ch}  crystal cap = {base}   ({per} frames per arm)");
    println!("{:>6} {:>8} {:>9} {:>9} {:>10} {:>7}", "cap", "frames", "snr_db", "evm_db", "cfo_hz", "rssi");

    for d in [0i32, -40, 40, 0] {
        let cap = (base as i32 + d).clamp(0, 127) as u8;
        dev.set_crystal_cap(cap)?;
        tokio::time::sleep(std::time::Duration::from_millis(400)).await;
        let (mut n, mut snr, mut evm, mut cfo, mut rssi, mut nphy) = (0usize, 0i64, 0i64, 0i64, 0i64, 0usize);
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(6);
        while n < per && std::time::Instant::now() < deadline {
            if let Ok(Ok(f)) = tokio::time::timeout(std::time::Duration::from_millis(200), dev.recv_frame()).await {
                n += 1;
                if let Some(r) = f.rssi_dbm { rssi += i64::from(r); }
                if let Some(p) = f.phy {
                    nphy += 1;
                    if let Some(v) = p.snr_db { snr += i64::from(v); }
                    if let Some(v) = p.evm_db { evm += i64::from(v); }
                    if let Some(v) = p.cfo_hz { cfo += i64::from(v); }
                }
            }
        }
        // Distinguish the two failures. The first version of this printed "no PHY metrics on any
        // frame" whenever nphy==0, which is what it says even when NO FRAMES ARRIVED AT ALL — and
        // that misdirected two rounds of debugging toward the parser when the real cause was a
        // transmitter that had gone down. Say which one it is.
        if n == 0 {
            println!("{cap:>6} {n:>8}   NO FRAMES RECEIVED — nothing to measure. Note ambient \
                      traffic does NOT count here: only frames in our own format (RawNdn 0x8624) \
                      survive parse_dot11, so this needs a live transmitter sending our frames.");
            continue;
        }
        if nphy == 0 {
            println!("{cap:>6} {n:>8}   frames arrived but carried NO PHY metrics — check drvinfo \
                      >= 28 with NDN_RX_META_DBG=1 (RSSI needs only 8, these fields need 28)");
            continue;
        }
        let k = nphy as i64;
        println!("{:>6} {:>8} {:>9.1} {:>9.1} {:>10} {:>7.1}", cap, n,
                 snr as f64 / k as f64, evm as f64 / k as f64, cfo / k,
                 rssi as f64 / n.max(1) as f64);
    }
    dev.set_crystal_cap(base)?;
    println!("restored cap = {}  (last arm repeats the first: CFO should return, SNR/EVM never moved)",
             dev.crystal_cap()?);
    Ok(())
}
