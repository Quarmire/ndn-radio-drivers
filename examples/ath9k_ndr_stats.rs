//! Bring up an AR9271 from userspace and read the Tier-0 filter's counters out of target RAM.
//!
//! This exercises the whole L1 stack with **no PHY initialisation**: firmware download, HTC
//! handshake, WMI echo, then `WMI_ACCESS_MEMORY`. Reading `ndr_stats` needs none of `ath9k_hw`'s
//! reset/initvals/calibration work, which is why this is reachable now and a full driver is not.
//!
//! The kernel driver must not hold the device:
//!
//! ```sh
//! echo -n 2-1.2:1.0 | sudo tee /sys/bus/usb/drivers/ath9k_htc/unbind
//! sudo ./ath9k_ndr_stats ~/ath9k-fw/target_firmware/htc_9271.fw 0x0050d880
//! # rebind afterwards:
//! echo -n 2-1.2:1.0 | sudo tee /sys/bus/usb/drivers/ath9k_htc/bind
//! ```
//!
//! `ndr_stats` moves between firmware builds — take the address from the image you loaded:
//! `xtensa-elf-nm build/k2/fw.elf | grep ndr_stats`.

use std::process::ExitCode;

use ndn_radio_drivers::Ath9kHtcBackend;

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 3 {
        eprintln!("usage: {} <htc_9271.fw> <ndr_stats_addr>", args[0]);
        eprintln!("  e.g. {} ~/ath9k-fw/target_firmware/htc_9271.fw 0x0050d880", args[0]);
        return ExitCode::FAILURE;
    }

    let fw_path = &args[1];
    let addr = match u32::from_str_radix(args[2].trim_start_matches("0x"), 16) {
        Ok(a) => a,
        Err(e) => {
            eprintln!("bad address {:?}: {e}", args[2]);
            return ExitCode::FAILURE;
        }
    };

    let fw = match std::fs::read(fw_path) {
        Ok(f) => f,
        Err(e) => {
            eprintln!("cannot read {fw_path}: {e}");
            return ExitCode::FAILURE;
        }
    };
    println!("firmware: {fw_path} ({} bytes)", fw.len());

    let mut dev = match Ath9kHtcBackend::open() {
        Ok(d) => d,
        Err(e) => {
            eprintln!("open failed: {e}");
            eprintln!("(is ath9k_htc still bound? unbind it first)");
            return ExitCode::FAILURE;
        }
    };
    println!("opened AR9271");

    if let Err(e) = dev.download_firmware(&fw) {
        eprintln!("firmware download failed: {e}");
        return ExitCode::FAILURE;
    }
    println!("firmware downloaded");

    if let Err(e) = dev.htc_init() {
        eprintln!("HTC handshake failed: {e}");
        return ExitCode::FAILURE;
    }
    let (credits, credit_size) = dev.credits();
    println!("HTC up: {credits} credits of {credit_size} B");

    // Before blaming our command framing, find out whether the target says anything at all. It
    // should raise WMI_TGT_RDY_EVENTID (0x1001) unprompted after SETUP_COMPLETE.
    let events = dev.drain_events(2500);
    if events.is_empty() {
        println!("post-setup: target sent NOTHING in 2.5 s");
    } else {
        for (ep, payload) in &events {
            let id = if payload.len() >= 2 {
                u16::from_be_bytes([payload[0], payload[1]])
            } else {
                0
            };
            println!("post-setup: ep={ep} id={id:#06x} payload={payload:02x?}");
        }
    }

    // Proof the command path works, before trusting a memory read. GET_FW_VERSION and not ECHO:
    // upstream's echo handler replies through the wrong pointer (see fw_version's doc comment).
    match dev.fw_version() {
        Ok((major, minor)) => println!("WMI GET_FW_VERSION -> {major}.{minor}"),
        Err(e) => {
            eprintln!("WMI GET_FW_VERSION failed: {e}");
            return ExitCode::FAILURE;
        }
    }

    // All-zero counters are the expected result before the receiver is started -- and they are
    // also what a read of the wrong address, or of nothing at all, would look like. Prove the path
    // by writing a sentinel and reading it back. `short_frame` is inert while nothing is being
    // received, and it is restored immediately afterwards.
    let sentinel_addr = addr + 16; // ndr_stats.short_frame
    const SENTINEL: u32 = 0xDEAD_BEEF;
    match dev.write_target_u32s(sentinel_addr, &[SENTINEL]) {
        Ok(_) => match dev.read_target_u32s(sentinel_addr, 1) {
            Ok(v) if v[0] == SENTINEL => {
                println!("sentinel: wrote {SENTINEL:#010x} to {sentinel_addr:#010x}, read back {:#010x} OK", v[0]);
                if let Err(e) = dev.write_target_u32s(sentinel_addr, &[0]) {
                    eprintln!("WARNING: could not restore short_frame to 0: {e}");
                }
            }
            Ok(v) => {
                eprintln!("sentinel MISMATCH: wrote {SENTINEL:#010x}, read {:#010x}", v[0]);
                return ExitCode::FAILURE;
            }
            Err(e) => {
                eprintln!("sentinel read-back failed: {e}");
                return ExitCode::FAILURE;
            }
        },
        Err(e) => {
            eprintln!("sentinel write failed: {e}");
            return ExitCode::FAILURE;
        }
    }

    let rc = match dev.read_ndr_stats(addr) {
        Ok(s) => {
            println!("\nndr_stats @ {addr:#010x}");
            println!("  seen            {}", s.seen);
            println!("  passed          {}", s.passed);
            println!("  dropped_filter  {}", s.dropped_filter);
            println!("  dropped_foreign {}", s.dropped_foreign);
            println!("  short_frame     {}", s.short_frame);
            ExitCode::SUCCESS
        }
        Err(e) => {
            eprintln!("read_ndr_stats failed: {e}");
            ExitCode::FAILURE
        }
    };

    // Always detach. Skipping this is what costs a physical replug before the next bring-up.
    match dev.detach() {
        Ok(()) => println!("target detached cleanly"),
        Err(e) => eprintln!("WARNING: clean detach failed ({e}) — a replug may be needed"),
    }

    rc
}
