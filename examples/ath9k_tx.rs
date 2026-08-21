//! **AR9271 M3 TX probe — inject N broadcast NDN frames a monitor receiver can capture.**
//!
//! Full bring-up (transport + firmware + reset + RX-start) then a burst of broadcast injections via
//! the `FrameIo::inject` path. This is the UNCERTAIN half of M3: RX + the RX-stamp clock are proven,
//! but the TX header layout (`tx_frame_hdr` + the target's node/rate handling) is bench-territory —
//! see `ath9k_htc::build_tx_frame_bytes` for exactly what to watch.
//!
//! ## What to expect / watch on the bench
//!  - The 802.11 frame is `FC=Data | A1=ff:ff:ff:ff:ff:ff | A2=02:4e:44:4e:00:01 | LLC/SNAP(0x8624)`.
//!    On a monitor receiver (another radio in monitor mode on the same channel) look for a broadcast
//!    Data frame with EtherType 0x8624 and the `\x05..` NDN payload.
//!  - **Rate is target-controlled**, not set here: the ath9k HTC `tx_frame_hdr` carries no rate, so
//!    the target's rate control picks it (likely a basic rate). `set_rate` is stored but inert.
//!  - **If nothing radiates** (frames accepted over USB, no capture): the most likely causes, in
//!    order — (1) the target needs a `WMI_NODE_CREATE`'d node/vif for `node_idx` (we send 0);
//!    (2) a wrong `data_type`; (3) a missing 4-byte endpoint prefix. None are more descriptor bytes.
//!
//! Run on o5p-1 (AR9271), kernel driver unbound / fresh:
//! ```sh
//! sudo LD_LIBRARY_PATH=$(nix path-info nixpkgs#libusb1)/lib \
//!     ./ath9k_tx ~/ath9k-fw/target_firmware/build/k2/htc_9271.fw [chan_mhz=2437] [count=20]
//! ```
use std::process::ExitCode;
use std::time::Duration;

use bytes::Bytes;
use ndn_radio_drivers::{Ath9kHtcBackend, FrameIo, InjectFrame, McsDescriptor, TxIntent};

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        eprintln!("usage: {} <htc_9271.fw> [chan_mhz=2437] [count=20]", args[0]);
        return ExitCode::FAILURE;
    }
    let fw = match std::fs::read(&args[1]) {
        Ok(f) => f,
        Err(e) => {
            eprintln!("cannot read {}: {e}", args[1]);
            return ExitCode::FAILURE;
        }
    };
    let chan_mhz: u16 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(2437);
    let count: usize = args.get(3).and_then(|s| s.parse().ok()).unwrap_or(20);
    println!(
        "firmware: {} ({} bytes), channel {chan_mhz} MHz, injecting {count} broadcast frames",
        args[1],
        fw.len()
    );

    let mut dev = match Ath9kHtcBackend::open() {
        Ok(d) => d,
        Err(e) => {
            eprintln!("open failed: {e}  (unbind ath9k_htc first)");
            return ExitCode::FAILURE;
        }
    };
    if let Err(e) = dev.download_firmware(&fw) {
        eprintln!("firmware download FAILED: {e}");
        return ExitCode::FAILURE;
    }
    if let Err(e) = dev.htc_init() {
        eprintln!("HTC handshake FAILED: {e}");
        return ExitCode::FAILURE;
    }
    // Faithful ath9k_hw_reset (reset + initvals + cal) then the post-reset RX-start steps. TX needs
    // the data services connected (for the DataBE endpoint id the frame rides) and the target's WLAN
    // app started; the RX-start path brings both up.
    if let Err(e) = dev.hw_reset(chan_mhz) {
        eprintln!("hw_reset FAILED: {e}");
        return ExitCode::FAILURE;
    }
    if let Err(e) = dev.connect_data_services() {
        eprintln!("connect_data_services FAILED: {e}");
        return ExitCode::FAILURE;
    }
    if let Err(e) = dev.wmi_start() {
        eprintln!("wmi_start FAILED: {e}");
        return ExitCode::FAILURE;
    }
    let (mgmt, data_be, beacon) = dev.data_endpoints();
    println!("[bring-up] data endpoints: mgmt={mgmt} data_be={data_be} beacon={beacon}");

    // Store a robust rate as bearer state (documentation only — inert on this HTC part).
    let _ = dev.set_rate(McsDescriptor::ht(0));

    let rt = match tokio::runtime::Builder::new_current_thread().enable_all().build() {
        Ok(r) => r,
        Err(e) => {
            eprintln!("tokio runtime: {e}");
            return ExitCode::FAILURE;
        }
    };
    rt.block_on(async {
        for i in 0..count {
            let payload = format!("\x05\x08ath9k-{i:03}");
            let frame = InjectFrame::broadcast(
                Bytes::copy_from_slice(payload.as_bytes()),
                TxIntent::CONSERVATIVE,
            );
            match dev.inject(frame).await {
                Ok(()) => println!("[tx {i:03}] injected {} B", payload.len()),
                Err(e) => {
                    eprintln!("[tx {i:03}] inject FAILED: {e}");
                    return ExitCode::FAILURE;
                }
            }
            tokio::time::sleep(Duration::from_millis(100)).await;
        }
        println!("done — {count} frames handed to the chip. Confirm on a monitor receiver.");
        ExitCode::SUCCESS
    })
}
