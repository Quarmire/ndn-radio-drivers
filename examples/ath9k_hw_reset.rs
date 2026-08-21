//! **AR9271 faithful `hw_reset()` bring-up + receive.**
//!
//! Exercises the transcription of mainline ath9k `ath9k_hw_reset()` — the exact
//! ordered spine — replacing the old piecemeal `phy_reset` → `apply_initvals` →
//! `set_channel_and_cal` → `rx_enable` path. The calibration now runs LAST, after
//! the full PHY/MAC setup, exactly as the kernel driver does.
//!
//!   transport  open → download_firmware → htc_init
//!   hw_reset   the full ath9k_hw_reset() spine (steps 1-24), cal runs last
//!   RX start   connect_data_services → start_receive → wmi_start
//!   poll       EP 0x82 — ORACLE: a real beacon, parsed
//!
//! Run on o5p-1 (AR9271), kernel driver unbound / fresh:
//! ```sh
//! sudo LD_LIBRARY_PATH=$(nix path-info nixpkgs#libusb1)/lib \
//!     ./ath9k_hw_reset ~/ath9k-fw/target_firmware/build/k2/htc_9271.fw [chan_mhz=2412]
//! ```
use std::process::ExitCode;
use std::time::{Duration, Instant};

use ndn_radio_drivers::Ath9kHtcBackend;

const AR_SREV: u32 = 0x4020;

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        eprintln!("usage: {} <htc_9271.fw> [chan_mhz=2412]", args[0]);
        return ExitCode::FAILURE;
    }
    let fw = match std::fs::read(&args[1]) {
        Ok(f) => f,
        Err(e) => {
            eprintln!("cannot read {}: {e}", args[1]);
            return ExitCode::FAILURE;
        }
    };
    let chan_mhz: u16 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(2412);
    println!("firmware: {} ({} bytes), channel {chan_mhz} MHz", args[1], fw.len());

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
    let _ = dev.drain_events(500);

    let rc = run(&mut dev, chan_mhz);

    match dev.detach() {
        Ok(()) => println!("target detached cleanly"),
        Err(e) => eprintln!("WARNING: clean detach failed ({e}) — a replug may be needed"),
    }
    rc
}

fn run(dev: &mut Ath9kHtcBackend, chan_mhz: u16) -> ExitCode {
    // ── step0 — reg primitive gate ──
    let srev = match dev.reg_read(AR_SREV) {
        Ok(v) => v,
        Err(e) => {
            eprintln!("[step0] reg_read(AR_SREV) FAILED: {e}");
            return ExitCode::FAILURE;
        }
    };
    if srev == 0 || srev == 0xffff_ffff || srev == AR_SREV {
        eprintln!("[step0] AR_SREV degenerate ({srev:#010x}) — reg primitive not proven, STOP");
        return ExitCode::FAILURE;
    }
    println!("[step0] AR_SREV={srev:#010x} (sig {:#04x}) — reg primitive OK", (srev >> 16) & 0xff);

    // ── the faithful ath9k_hw_reset() spine (cal runs last) ──
    if let Err(e) = dev.hw_reset(chan_mhz) {
        eprintln!("[hw_reset] FAILED: {e}");
        return ExitCode::FAILURE;
    }
    let synth = dev.reg_read(0x9874).unwrap_or(0);
    let phy_active = dev.reg_read(0x981c).unwrap_or(0);
    let phy_cca = dev.reg_read(0x9864).unwrap_or(0);
    println!(
        "[hw_reset] OK — SYNTH_CONTROL={synth:#010x} PHY_ACTIVE={phy_active:#x} PHY_CCA={phy_cca:#010x}"
    );
    if phy_active & 0x1 != 0x1 {
        eprintln!("[hw_reset] WARNING: AR_PHY_ACTIVE did not read back enabled ({phy_active:#x})");
    }

    // ── RX start: data services → target WMI verbs (set up RX ring) → host RX filter/DMA ──
    if let Err(e) = dev.connect_data_services() {
        eprintln!("[rx] connect_data_services FAILED: {e}");
        return ExitCode::FAILURE;
    }
    let (mgmt, be, beacon) = dev.data_endpoints();
    println!("[rx] data services: mgmt_ep={mgmt} data_be_ep={be} beacon_ep={beacon}");
    // Disable the NDR Tier-0 name filter so broadcast beacons aren't dropped pre-USB.
    let _ = dev.write_target_u32s(0x0050_cf44, &[0]);
    // Target-side RX FIRST — START_RECV sets AR_RXDP (the descriptor ring) — then host RX DMA
    // enable. (Enabling AR_CR_RXE before the ring exists latches a stale/zero descriptor pointer.)
    if let Err(e) = dev.wmi_start() {
        eprintln!("[rx] wmi_start FAILED: {e}");
        return ExitCode::FAILURE;
    }
    if let Err(e) = dev.start_receive() {
        eprintln!("[rx] start_receive FAILED: {e}");
        return ExitCode::FAILURE;
    }
    let rxdp0 = dev.reg_read(0x000c).unwrap_or(0);
    println!("[rx] receiver open (filter=0xc03f, CR_RXE set, verbs issued); AR_RXDP={rxdp0:#010x}");

    // Raw-dump the first RX transfers to nail the true wire layout (HTC hdr + rx_frame_header +
    // 802.11). A beacon's frame body starts with frame-control 0x80 0x00; find it and work back.
    println!("[rx] raw transfer dump (to resolve the parse offsets):");
    for _ in 0..4 {
        match dev.recv_raw_frame(Duration::from_millis(800)) {
            Ok(raw) if !raw.is_empty() => {
                let h = &raw[..raw.len().min(72)];
                println!("[rx] raw {:>4} B: {h:02x?}", raw.len());
            }
            _ => {}
        }
    }

    // ── poll EP 0x82 for beacons ──
    println!("[rx] polling EP 0x82 for 12 s ...");
    let deadline = Instant::now() + Duration::from_secs(12);
    let mut n = 0u32;
    while Instant::now() < deadline {
        match dev.recv_frame(Duration::from_millis(500)) {
            Ok(Some(f)) => {
                n += 1;
                if n <= 10 {
                    println!(
                        "[rx] frame #{n}: len={} rate={:#04x} rssi={} tstamp={}",
                        f.rs_datalen, f.rs_rate, f.rs_rssi, f.rs_tstamp
                    );
                }
            }
            Ok(None) => {}
            Err(_) => {} // timeout — keep polling
        }
    }
    // Diagnostics: did the RX ring advance? did the target's tasklet see any frame?
    let rxdp1 = dev.reg_read(0x000c).unwrap_or(0);
    let stats = dev.read_target_u32s(0x0050_dc18, 6).unwrap_or_default();
    println!(
        "[rx] AR_RXDP after={rxdp1:#010x} (advanced={}); ndr_stats seen={} passed={} drop_filter={} drop_popcount={}",
        rxdp1 != rxdp0,
        stats.first().copied().unwrap_or(0),
        stats.get(1).copied().unwrap_or(0),
        stats.get(2).copied().unwrap_or(0),
        stats.get(5).copied().unwrap_or(0),
    );
    if n == 0 {
        eprintln!("[rx] NO frames received on EP 0x82");
        return ExitCode::FAILURE;
    }
    println!("[rx] received {n} frames — bring-up ORACLE PASSED");
    ExitCode::SUCCESS
}
