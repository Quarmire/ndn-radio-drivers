//! **AR9271 M1.3/M1.4/M1-done probe — channel + cal + RX enable + receive a beacon.**
//!
//! Full bring-up on top of the proven transport + reset + initvals:
//!   step0  SREV oracle (reg primitive)          — gate before writing anything
//!   M1.1   phy_reset()                           — chip reset
//!   M1.2   apply_initvals()                      — 2.4 GHz HT20 initvals
//!   M1.3   set_channel_and_cal(chan)             — synth + AGC/offset cal + NF
//!   M1.4   connect_data_services() + rx_enable() — filter/opmode/CR_RXE + WMI verbs
//!   M1     poll EP 0x82                          — ORACLE: a real beacon, parsed
//!
//! On a live 2.4 GHz channel beacons arrive within seconds. We print rs_datalen / rs_rate /
//! rs_rssi / rs_tstamp and the first 802.11 addresses; rs_tstamp should advance frame-to-frame
//! (that is the M2 clock).
//!
//! Run on o5p-1 (AR9271 at 2-1.2), kernel driver unbound / fresh:
//! ```sh
//! sudo LD_LIBRARY_PATH=$(nix path-info nixpkgs#libusb1)/lib \
//!     ./ath9k_m1_rx ~/ath9k-fw/target_firmware/build/k2/htc_9271.fw [chan_mhz]
//! ```
//! chan_mhz defaults to 2437 (ch6). Try 2412 (ch1) or 2462 (ch11) to match the nearest AP.
use std::process::ExitCode;
use std::time::{Duration, Instant};

use ndn_radio_drivers::Ath9kHtcBackend;

const AR_SREV: u32 = 0x4020;

/// Every register the kernel wrote during its (working) monitor bring-up, final value — from the
/// usbmon golden trace. Used both to force-match state (NDR_FORCE_KERNEL) and to diff on failure.
const KERNEL_REGS: &[(u32, u32)] = &[(0x0008,0x00000004), (0x000c,0x00000000), (0x0014,0x0000000a), (0x0030,0x00020045), (0x00a0,0x81800964), (0x00a4,0x010f0000), (0x00a8,0x010f0000), (0x00ac,0x00800000), (0x0920,0xffffec00), (0x1000,0x00000001), (0x1004,0x00000002), (0x1030,0x00000160), (0x103c,0x00000000), (0x1040,0x002ffc0f), (0x1044,0x002ffc0f), (0x1048,0x002ffc0f), (0x104c,0x002ffc0f), (0x1060,0x002ffc0f), (0x1064,0x00100401), (0x1070,0x0000018c), (0x1080,0x0008200a), (0x1084,0x0008200a), (0x1088,0x0008200a), (0x108c,0x0008200a), (0x10a0,0x0008200a), (0x10a4,0x0008200a), (0x10b0,0x00003e38), (0x17f8,0x00000000), (0x1f04,0x00000003), (0x4080,0x00000008), (0x7014,0x0000142c), (0x7048,0x00000002), (0x704c,0x00000003), (0x8000,0x99cac000), (0x8004,0x88808609), (0x803c,0x0000c03f), (0x8040,0xffffffff), (0x8044,0xffffffff), (0x810c,0x00000000), (0x8114,0x00000200), (0x8118,0x000100aa), (0x811c,0x00003210), (0x8318,0x00000000), (0x8328,0x00000000), (0x832c,0x00000001), (0x8344,0x00581083), (0x9800,0x00000007), (0x9804,0x000003c0), (0x981c,0x00000001), (0x9874,0x30a0cccc), (0x9934,0x32323232), (0x9938,0x32323232), (0x9964,0x11111441), (0x9970,0x192bb514), (0x99a4,0x00000001), (0x99ac,0x2cef0400), (0x9a58,0x00058300), (0x9a5c,0x00058304), (0x9b50,0x000eb7c3), (0x9b54,0x000eb7c7), (0xa200,0x00000004), (0xa204,0x00000004), (0xa208,0x803e48c8), (0xa26c,0x0ebafed4), (0xa280,0x7f7e7d7c), (0xa300,0x00010000), (0xa304,0x00016200), (0xa39c,0x00000001), (0xa3ec,0x00f70081), (0xa3f0,0x01036a2f), (0x50040,0x00000304), (0x50044,0x00004000)];

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        eprintln!("usage: {} <htc_9271.fw> [chan_mhz=2437]", args[0]);
        return ExitCode::FAILURE;
    }
    let fw = match std::fs::read(&args[1]) {
        Ok(f) => f,
        Err(e) => {
            eprintln!("cannot read {}: {e}", args[1]);
            return ExitCode::FAILURE;
        }
    };
    // One channel if given, else sweep the three common 2.4 GHz AP channels in a single run
    // (each userspace run costs a physical replug, so make it count).
    let channels: Vec<u16> = match args.get(2).and_then(|s| s.parse().ok()) {
        Some(c) => vec![c],
        None => vec![2412, 2437, 2462], // ch1 / ch6 / ch11
    };
    println!("firmware: {} ({} bytes), channels {:?} MHz", args[1], fw.len(), channels);

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
    println!(
        "[transport] HTC up: {credits} credits of {credit_size} B, FW {:?}",
        dev.fw_version()
    );
    let _ = dev.drain_events(500);

    // ★ Disable the NDR Tier-0 name filter before RX. Our firmware's `ath_tgt_rx_tasklet` runs
    // `ndr_rx_accept()` on every received frame and DROPS rejects before they cross USB — and a
    // broadcast beacon (addr1 = ff:ff:ff:ff:ff:ff, all-ones) is rejected by the popcount guard.
    // `ndr_cfg.enabled = 0` is documented "every frame goes to the host" (stock behaviour).
    // Address from `xtensa-elf-gcc-nm build/k2/fw.elf`: ndr_cfg=0x0050cf40, .enabled at +4.
    const NDR_CFG_ENABLED: u32 = 0x0050_cf44; // fw 1.98
    match dev.write_target_u32s(NDR_CFG_ENABLED, &[0]) {
        Ok(_) => match dev.read_target_u32s(NDR_CFG_ENABLED, 1) {
            Ok(v) => println!("[tier0] ndr_cfg.enabled={} (0 = every frame to host)", v[0]),
            Err(e) => eprintln!("[tier0] enabled read-back failed: {e}"),
        },
        Err(e) => eprintln!("[tier0] WARNING could not write ndr_cfg.enabled (AccessMemory): {e}"),
    }

    let rc = run(&mut dev, &channels);

    match dev.detach() {
        Ok(()) => println!("target detached cleanly"),
        Err(e) => eprintln!("WARNING: clean detach failed ({e}) — a replug may be needed"),
    }
    rc
}

fn run(dev: &mut Ath9kHtcBackend, channels: &[u16]) -> ExitCode {
    let chan_mhz = channels[0];
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

    // ── M1.1 reset ──
    let st = match dev.phy_reset() {
        Ok(s) => s,
        Err(e) => {
            eprintln!("[M1.1] phy_reset FAILED: {e}");
            return ExitCode::FAILURE;
        }
    };
    if (st.rtc_status & 0x0f) != 0x02 || st.rtc_rc != 0 {
        eprintln!(
            "[M1.1] reset verify FAILED: STATUS={:#x} RC={:#x}",
            st.rtc_status, st.rtc_rc
        );
        return ExitCode::FAILURE;
    }
    println!("[M1.1] reset OK (STATUS=ON, RC=0)");

    // ── M1.2 initvals ──
    let iv = match dev.apply_initvals() {
        Ok(v) => v,
        Err(e) => {
            eprintln!("[M1.2] apply_initvals FAILED: {e}");
            return ExitCode::FAILURE;
        }
    };
    println!(
        "[M1.2] initvals: {} writes, {}/{} sentinels matched{}",
        iv.total_written,
        iv.matched,
        iv.checked,
        if iv.mismatches.is_empty() { "" } else { " (analog 0x7804 hw-bits expected)" }
    );

    // ── M1.3 channel + cal ──
    let cal = match dev.set_channel_and_cal(chan_mhz) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("[M1.3] set_channel_and_cal FAILED: {e}");
            return ExitCode::FAILURE;
        }
    };
    println!(
        "[M1.3] chan={} MHz  SYNTH_CONTROL={:#010x}  PHY_MODE={:#x}  PHY_ACTIVE={:#x}",
        cal.chan_mhz, cal.synth_control, cal.phy_mode, cal.phy_active
    );
    println!(
        "[M1.3] AGC cal {}  AGC_CONTROL={:#010x}  PHY_CCA={:#010x}  noise_floor={} dBm",
        if cal.agc_cal_converged { "CONVERGED ✓" } else { "DID NOT CONVERGE ✗" },
        cal.agc_control,
        cal.phy_cca,
        cal.noise_floor_dbm
    );
    if !cal.agc_cal_converged {
        eprintln!("[M1.3] AGC cal hung — AR_PHY_AGC_CONTROL={:#010x} (CAL bit stuck). RX may be deaf.", cal.agc_control);
        // Continue anyway to see if any frame arrives, but flag it.
    }
    if cal.phy_active & 0x1 != 0x1 {
        eprintln!("[M1.3] WARNING: AR_PHY_ACTIVE did not read back enabled ({:#x})", cal.phy_active);
    }

    // ── M1.4 RX enable ──
    if let Err(e) = dev.connect_data_services() {
        eprintln!("[M1.4] connect_data_services FAILED: {e}");
        return ExitCode::FAILURE;
    }
    let (mgmt, be, beacon) = dev.data_endpoints();
    println!("[M1.4] data services: mgmt_ep={mgmt} data_be_ep={be} beacon_ep={beacon}");
    if let Err(e) = dev.rx_enable() {
        eprintln!("[M1.4] rx_enable FAILED: {e}");
        return ExitCode::FAILURE;
    }
    println!("[M1.4] RX enabled (filter=0xc03f monitor, CR_RXE set, verbs issued)");
    // AR_RXDP advance test: if this pointer moves during the poll, the hardware IS consuming RX
    // descriptors (frames arrive; the gap is delivery/tasklet). If it stays put, RX isn't receiving.
    let rxdp0 = dev.reg_read(0x000c).unwrap_or(0);
    println!("[M1] AR_RXDP before poll = {rxdp0:#010x}");

    // ★ BRUTE-FORCE test: overwrite every register with the kernel's captured value, then re-assert
    // AR_CR_RXE. If frames flow now, the gap IS register state (bisect from the diff list); if not,
    // the issue is ordering/timing/target-side, not what any single register holds.
    if std::env::var_os("NDR_FORCE_KERNEL").is_some() {
        let mut n = 0;
        for &(a, v) in KERNEL_REGS {
            if a == 0x000c { continue; } // RXDP owned by the target
            if dev.reg_write(a, v).is_ok() { n += 1; }
        }
        let _ = dev.reg_write(0x0008, 0x00000004); // re-assert AR_CR_RXE last
        println!("[M1] ★ forced {n} kernel register values (NDR_FORCE_KERNEL)");
    }

    // ── M1 — sweep channels, poll EP 0x82 on each ──
    let mut total = 0u32;
    let mut last_tstamp: Option<u64> = None;
    let per_chan = if channels.len() > 1 { 6 } else { 12 };
    for (i, &ch) in channels.iter().enumerate() {
        if i > 0 {
            // Re-tune: reprogram synth + re-cal + reload NF for the new channel.
            match dev.set_channel_and_cal(ch) {
                Ok(c) => println!(
                    "[M1] retune ch {ch} MHz: SYNTH={:#010x} AGC_cal={} NF={} dBm",
                    c.synth_control,
                    if c.agc_cal_converged { "ok" } else { "HUNG" },
                    c.noise_floor_dbm
                ),
                Err(e) => {
                    eprintln!("[M1] retune to {ch} FAILED: {e}");
                    continue;
                }
            }
        }
        println!("[M1] channel {ch} MHz — polling EP 0x82 for {per_chan} s ...");
        let deadline = Instant::now() + Duration::from_secs(per_chan);
        let mut n = 0u32;
        while Instant::now() < deadline {
            match dev.recv_frame(Duration::from_millis(400)) {
                Ok(Some(f)) => {
                    n += 1;
                    total += 1;
                    let d = &f.frame;
                    let fc = if d.len() >= 2 { u16::from_le_bytes([d[0], d[1]]) } else { 0 };
                    let (ftype, fsub) = ((fc >> 2) & 0x3, (fc >> 4) & 0xf);
                    let a = |off: usize| -> String {
                        if d.len() >= off + 6 {
                            format!("{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
                                d[off], d[off+1], d[off+2], d[off+3], d[off+4], d[off+5])
                        } else { "??".into() }
                    };
                    let dt = last_tstamp.map(|p| f.rs_tstamp.wrapping_sub(p));
                    last_tstamp = Some(f.rs_tstamp);
                    let kind = match (ftype, fsub) {
                        (0, 8) => "beacon", (0, 5) => "probe-resp", (0, 4) => "probe-req",
                        (0, _) => "mgmt", (1, _) => "ctrl", (2, _) => "data", _ => "?",
                    };
                    println!(
                        "[M1] ch{ch} #{total} {kind} datalen={} rate={:#04x} rssi={} dBm tstamp={} \
                         dt={} fc={:#06x} a1={} a2={} a3={}",
                        f.rs_datalen, f.rs_rate, f.rs_rssi, f.rs_tstamp,
                        dt.map(|v| v.to_string()).unwrap_or_else(|| "-".into()),
                        fc, a(4), a(10), a(16)
                    );
                    if total == 1 {
                        let head = &d[..d.len().min(40)];
                        println!("[M1]    first-frame head: {head:02x?}");
                    }
                }
                Ok(None) => {}
                Err(_) => {}
            }
        }
        println!("[M1] channel {ch}: {n} frame(s)");
    }

    // ★ Tier-0 counters — the definitive RX diagnostic. `seen` = frames the target RX tasklet
    // actually processed: seen>0 proves the RF front end + DMA ring deliver (RX works); if delivered
    // frames are still 0 with the filter disabled, the gap is downstream (HTC/USB/parse). seen==0 =
    // nothing reached the tasklet at all (RX ring/DMA not delivering — the gap is in ath_startrecv).
    const NDR_STATS: u32 = 0x0050_dc18; // fw 1.98; seen,passed,drop_filter,drop_foreign,short,drop_popcount
    match dev.read_target_u32s(NDR_STATS, 6) {
        Ok(s) => println!(
            "[tier0] ndr_stats: seen={} passed={} drop_filter={} drop_foreign={} short={} drop_popcount={}",
            s[0], s[1], s[2], s[3], s[4], s[5]
        ),
        Err(e) => eprintln!("[tier0] ndr_stats read failed: {e}"),
    }

    if total > 0 {
        println!("\n[M1] ★ RECEIVED {total} frame(s) off EP 0x82 — RX PATH LIVE ✓");
        ExitCode::SUCCESS
    } else {
        eprintln!("\n[M1] no frames. FULL golden-trace diff (our read-back vs kernel's written value):");
        // KERNEL_REGS is the module-level golden-trace table. Known-noisy: 0x78xx/0x79xx analog + NF
        // regs (live cal bits), 0x000c RXDP (target-set async) — flagged but expected to differ.
        let mut miss = 0;
        for &(a, kv) in KERNEL_REGS {
            if a == 0x000c { continue; }
            match dev.reg_read(a) {
                Ok(ov) if ov != kv => {
                    eprintln!("[diff] {a:#06x}: ours={ov:#010x}  kernel={kv:#010x}");
                    miss += 1;
                }
                Err(e) => eprintln!("[diff] {a:#06x} read err: {e}"),
                _ => {}
            }
        }
        eprintln!("[diff] {miss} mismatches vs kernel golden trace");
        ExitCode::FAILURE
    }
}
