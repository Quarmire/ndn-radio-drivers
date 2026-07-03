//! BW16 host-side probe: open the serial-bridged board, set a channel, inject a
//! test broadcast NDN frame, and print captured frames for ~5 s.
//!
//! Flash `firmware/bw16-ndn-bridge` first, then:
//!   cargo run --features bw16 --example bw16_probe -- /dev/cu.usbserial-1110 6

#[cfg(feature = "bw16")]
fn main() -> Result<(), Box<dyn std::error::Error>> {
    tokio::runtime::Runtime::new()?.block_on(run())
}

#[cfg(feature = "bw16")]
async fn run() -> Result<(), Box<dyn std::error::Error>> {
    use bytes::Bytes;
    use ndn_frame_io::{FrameIo, InjectFrame, TxIntent};
    use ndn_radio_drivers::Bw16SerialBackend;
    use std::sync::Arc;
    use std::time::Duration;

    let mut args = std::env::args().skip(1);
    let path = args.next().unwrap_or_else(|| "/dev/cu.usbserial-1110".into());
    let ch: u8 = args.next().and_then(|s| s.parse().ok()).unwrap_or(6);

    let dev = Arc::new(Bw16SerialBackend::open(&path)?);
    dev.set_channel(ch)?;
    println!("bw16 open on {path} @ ch{ch}");

    let frame = InjectFrame::broadcast(
        Bytes::from_static(b"hello-from-bw16"),
        TxIntent::CONSERVATIVE,
    );
    dev.inject(frame).await?;
    println!("injected 1 test broadcast frame; listening 5s for captures…");

    let mut heard = 0u32;
    for _ in 0..25 {
        match tokio::time::timeout(Duration::from_millis(200), dev.recv_frame()).await {
            Ok(Ok(c)) => {
                heard += 1;
                println!(
                    "  RX {}B rssi={:?} src={:?} grp={:?}",
                    c.payload.len(),
                    c.rssi_dbm,
                    c.addr,
                    c.group
                );
            }
            _ => {}
        }
    }
    println!("done — {heard} frames captured");
    Ok(())
}

#[cfg(not(feature = "bw16"))]
fn main() {
    eprintln!("rebuild with --features bw16");
}
