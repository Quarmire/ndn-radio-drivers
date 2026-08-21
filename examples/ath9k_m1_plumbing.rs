//! **AR9271 M1.0 probe — data-service plumbing.** Second increment after `ath9k_m0`: on top of the
//! proven transport (fw-download + HTC + WMI), connect the HTC **data** services the frame path
//! rides on (Mgmt / DataBE / Beacon, all on the bulk WLAN pipes dl=2/ul=1) and claim the bulk RX
//! pipe. No PHY is up, so no frames flow — the success signal is that all three service connects
//! return an endpoint id and the RX pipe reads time out cleanly (it is claimable).
//!
//! It also answers one empirical ordering question: does the target accept `CONNECT_SERVICE` *after*
//! `SETUP_COMPLETE` (which `htc_init` sends)? ath9k connects every service *before* SETUP_COMPLETE.
//! If the connects fail here, that ordering is why, and `htc_init` needs splitting.
//!
//! Run on o5p-1 (AR9271 at 2-1.2:1.0), kernel driver unbound:
//! ```sh
//! echo -n 2-1.2:1.0 | sudo tee /sys/bus/usb/drivers/ath9k_htc/unbind
//! sudo LD_LIBRARY_PATH=$(nix path-info nixpkgs#libusb1)/lib \
//!     ./ath9k_m1_plumbing ~/ath9k-fw/target_firmware/build/k2/htc_9271.fw
//! echo -n 2-1.2:1.0 | sudo tee /sys/bus/usb/drivers/ath9k_htc/bind
//! ```
use std::process::ExitCode;
use std::time::Duration;

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
    let (credits, credit_size) = dev.credits();
    println!("[transport] HTC up: {credits} credits of {credit_size} B, FW {:?}", dev.fw_version());
    let _ = dev.drain_events(800);

    // M1.0 — connect the data services.
    let rc = match dev.connect_data_services() {
        Ok(()) => {
            let (mgmt, be, beacon) = dev.data_endpoints();
            println!("[M1.0] data services connected: mgmt_ep={mgmt} data_be_ep={be} beacon_ep={beacon}");
            if mgmt == 0 || be == 0 || beacon == 0 {
                eprintln!("[M1.0] WARNING: a data endpoint id came back 0 (unexpected)");
            }

            // Claim + poll the bulk RX pipe. No PHY => expect clean timeouts. If anything DOES
            // arrive, dump the first 48 bytes ([HTC 8B][ath_htc_rx_status 40B]) as a bonus.
            println!("[M1.0] polling bulk RX (EP 0x82) for 3 s — timeouts are the expected result...");
            let mut got = 0u32;
            let deadline = std::time::Instant::now() + Duration::from_secs(3);
            while std::time::Instant::now() < deadline {
                match dev.recv_raw_frame(Duration::from_millis(400)) {
                    Ok(b) if !b.is_empty() => {
                        let head = &b[..b.len().min(48)];
                        println!("[M1.0] ★ RX {} B (unexpected without PHY): {head:02x?}", b.len());
                        got += 1;
                    }
                    Ok(_) => {}
                    Err(_) => {} // timeout — the expected, healthy case
                }
            }
            println!("[M1.0] RX poll done: {got} frames (0 = healthy; pipe is claimable). PLUMBING OK ✓");
            ExitCode::SUCCESS
        }
        Err(e) => {
            eprintln!("[M1.0] data-service connect FAILED: {e}");
            eprintln!("        if this is a CONNECT_SERVICE_RESPONSE timeout/refusal, the likely cause");
            eprintln!("        is ordering — connect the data services BEFORE SETUP_COMPLETE (split htc_init).");
            ExitCode::FAILURE
        }
    };

    match dev.detach() {
        Ok(()) => println!("target detached cleanly"),
        Err(e) => eprintln!("WARNING: clean detach failed ({e}) — a replug may be needed"),
    }
    rc
}
