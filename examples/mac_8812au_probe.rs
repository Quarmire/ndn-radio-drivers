//! Does the rtl8812au backend bring up + RX on the Mac's RTL8812AU? Arg: [channel] (default 36).
use ndn_frame_io::FrameIo;
use ndn_radio_drivers::Rtl8812auBackend;
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(36);
    let rt = tokio::runtime::Builder::new_multi_thread().enable_all().build()?;
    rt.block_on(async move {
        let dev = match Rtl8812auBackend::open() {
            Ok(d) => d,
            Err(e) => { println!("open ERR: {e}"); return; }
        };
        match dev.bring_up_monitor(ch) {
            Ok(_) => println!("8812AU bring_up_monitor({ch}) OK"),
            Err(e) => { println!("bring_up ERR: {e}"); return; }
        }
        let mut frames = 0;
        for _ in 0..40 {
            if let Ok(Ok(_)) = tokio::time::timeout(std::time::Duration::from_millis(250), dev.recv_frame()).await { frames += 1; }
        }
        println!("captured {frames} frames in ~10s on ch{ch}");
    });
    Ok(())
}
