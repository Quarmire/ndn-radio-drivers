//! **AR9271 M0 transport probe** — the smallest possible first-contact test of our libusb
//! host L1 against real hardware: USB open → firmware download → HTC handshake → WMI
//! `GET_FW_VERSION`. No `WMI_ACCESS_MEMORY`, no PHY init, no target writes — so nothing here
//! can wedge the target on a wrong address; the worst case is a clean failure at one stage.
//!
//! This exists because the L1 in `ath9k_htc.rs` was code-complete but had never been run
//! against silicon (see the module doc). It answers exactly one question: does our own
//! firmware-download + HTC + WMI path work end to end on the dongle? If yes, the PHY port
//! (M1) has a proven transport to build on.
//!
//! Run on o5p-1 (AR9271 at 2-1.2:1.0), kernel driver unbound:
//!
//! ```sh
//! echo -n 2-1.2:1.0 | sudo tee /sys/bus/usb/drivers/ath9k_htc/unbind
//! sudo LD_LIBRARY_PATH=$(nix path-info nixpkgs#libusb1)/lib \
//!     ./ath9k_m0 ~/ath9k-fw/target_firmware/build/k2/htc_9271.fw
//! # rebind afterwards:
//! echo -n 2-1.2:1.0 | sudo tee /sys/bus/usb/drivers/ath9k_htc/bind
//! ```
use std::process::ExitCode;

use ndn_radio_drivers::Ath9kHtcBackend;

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        eprintln!("usage: {} <htc_9271.fw>", args[0]);
        return ExitCode::FAILURE;
    }
    let fw = match std::fs::read(&args[1]) {
        Ok(f) => f,
        Err(e) => {
            eprintln!("cannot read {}: {e}", args[1]);
            return ExitCode::FAILURE;
        }
    };
    println!("firmware: {} ({} bytes)", args[1], fw.len());

    let mut dev = match Ath9kHtcBackend::open() {
        Ok(d) => d,
        Err(e) => {
            eprintln!("open failed: {e}  (is ath9k_htc still bound? unbind it first)");
            return ExitCode::FAILURE;
        }
    };
    println!("[1/4] opened AR9271");

    if let Err(e) = dev.download_firmware(&fw) {
        eprintln!("[2/4] firmware download FAILED: {e}");
        return ExitCode::FAILURE;
    }
    println!("[2/4] firmware downloaded + entry handed off");

    if let Err(e) = dev.htc_init() {
        eprintln!("[3/4] HTC handshake FAILED: {e}");
        return ExitCode::FAILURE;
    }
    let (credits, credit_size) = dev.credits();
    println!("[3/4] HTC up: {credits} credits of {credit_size} B");

    // The target should raise WMI_TGT_RDY_EVENTID (0x1001) unprompted after SETUP_COMPLETE.
    let events = dev.drain_events(2500);
    if events.is_empty() {
        println!("      post-setup: target sent NOTHING in 2.5 s (may be normal)");
    } else {
        for (ep, payload) in &events {
            let id = if payload.len() >= 2 {
                u16::from_be_bytes([payload[0], payload[1]])
            } else {
                0
            };
            println!("      post-setup event: ep={ep} id={id:#06x} len={}", payload.len());
        }
    }

    let rc = match dev.fw_version() {
        Ok((major, minor)) => {
            println!("[4/4] WMI GET_FW_VERSION -> {major}.{minor}  ✓ TRANSPORT LIVE");
            ExitCode::SUCCESS
        }
        Err(e) => {
            eprintln!("[4/4] WMI GET_FW_VERSION FAILED: {e}");
            ExitCode::FAILURE
        }
    };

    // Always detach — skipping this is what costs a physical replug before the next bring-up.
    match dev.detach() {
        Ok(()) => println!("target detached cleanly"),
        Err(e) => eprintln!("WARNING: clean detach failed ({e}) — a replug may be needed"),
    }
    rc
}
