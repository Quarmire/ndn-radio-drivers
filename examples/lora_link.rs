//! Two-node NDN-over-LoRa link exercising [`LoraSerialBackend`] through the `FrameIo` seam.
//!
//! Run one node as `rx` and another as `tx` (or both as `both`) on two Waveshare USB-TO-LoRa
//! dongles sharing the factory radio params:
//!
//! ```text
//! # on node A (listener):
//! lora_link /dev/ttyACM0 rx
//! # on node B (sender):
//! lora_link /dev/ttyACM0 tx B
//! ```
//!
//! Each `tx` payload is a stand-in NDN packet body; the backend frames it, the SX1262 sends it as
//! one LoRa frame, and the peer's reader deframes it back to a `CapturedFrame` (HostRecv-stamped).

use std::time::Duration;

use ndn_frame_io::{FrameIo, InjectFrame, TxIntent};
use ndn_radio_drivers::LoraSerialBackend;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let rt = tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()?;
    rt.block_on(run())
}

async fn run() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let path = args.next().unwrap_or_else(|| "/dev/ttyACM0".into());
    let role = args.next().unwrap_or_else(|| "both".into());
    let node = args.next().unwrap_or_else(|| "A".into());

    // Optional 4th arg `sf=N`: retune the spreading factor at runtime through the RadioKnobs seam
    // (proves cognition's LoRa reach/rate dial actuates the live radio).
    let sf: Option<u8> = args
        .next()
        .and_then(|a| a.strip_prefix("sf=").and_then(|s| s.parse().ok()));

    println!("opening LoRa dongle at {path} (role={role}, node={node})…");
    let dev = std::sync::Arc::new(LoraSerialBackend::open(&path)?);
    println!("open OK, params = {:?}", dev.params());
    if let Some(sf) = sf {
        use ndn_radio_hal::RadioKnobs;
        println!("retuning spreading factor -> SF{sf} via RadioKnobs…");
        dev.set_spreading_factor(sf)
            .map_err(|e| format!("set_spreading_factor: {e:?}"))?;
        println!("retuned; live params = {:?}", dev.params());
    }

    if role != "tx" {
        let rx = dev.clone();
        tokio::spawn(async move {
            loop {
                match rx.recv_frame().await {
                    Ok(f) => {
                        let body = String::from_utf8_lossy(&f.payload);
                        let age = f.stamp.map(|s| s.raw).unwrap_or(0);
                        println!("RX [{} B, host_ns={age}] {body}", f.payload.len());
                    }
                    Err(e) => {
                        eprintln!("recv ended: {e:?}");
                        break;
                    }
                }
            }
        });
    }

    if role != "rx" {
        for seq in 0u32.. {
            let body = format!("LORA-NDN node={node} seq={seq}");
            let frame = InjectFrame::broadcast(body.clone().into_bytes().into(), TxIntent::CONSERVATIVE);
            match dev.inject(frame).await {
                Ok(()) => println!("TX {body}"),
                Err(e) => eprintln!("inject err: {e:?}"),
            }
            tokio::time::sleep(Duration::from_millis(2000)).await;
        }
    } else {
        // rx-only: park forever while the reader task runs.
        std::future::pending::<()>().await;
    }
    Ok(())
}
