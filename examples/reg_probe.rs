//! Root-cause probe for the ~30 dB TX deficit (#36): dump the 8812au TX-power /
//! RFE / BB-swing registers after a full bring-up, and compare to the reference
//! values a correct `PHY_SetTxPowerLevel8812` + `phy_SetRFEReg8812` programs
//! (devourer / aircrack-ng). Optionally poke a register and re-inject, so an RSSI
//! delta can be measured without editing+rebuilding the driver per experiment.
//!
//!   # dump current state
//!   sudo NDN_RADIO_NO_RESET=1 LD_LIBRARY_PATH=$(nix path-info nixpkgs#libusb1)/lib \
//!       ./reg_probe dump
//!   # apply candidate fixes then dump (POKE="addr=val,addr=val", hex)
//!   sudo ... POKE="0xcb0=0x77777777,0xeb0=0x77777777,0xcb4=0,0xeb4=0" ./reg_probe dump
//!   # after poking, beacon so a peer can measure the RSSI delta
//!   sudo ... POKE="..." ./reg_probe beacon 60
fn main() -> Result<(), Box<dyn std::error::Error>> {
    use ndn_radio_drivers::Rtl8812auBackend;
    use std::time::Duration;

    // (addr, reference value for rfe_type 0 / 2.4 GHz / full power, note)
    let refs: &[(u16, u32, &str)] = &[
        (0xcb0, 0x7777_7777, "RFE pinmux A (ext-PA/TRSW routing)"),
        (0xeb0, 0x7777_7777, "RFE pinmux B"),
        (0xcb4, 0x0000_0000, "RFE inv A"),
        (0xeb4, 0x0000_0000, "RFE inv B"),
        (0xc1c, 0x4000_0003, "BB TX swing A [31:21]=0x200=0dB"),
        (0xe1c, 0x4000_0003, "BB TX swing B"),
        (0xc20, 0x3f3f_3f3f, "TXAGC A CCK/OFDM (we set 0x3f)"),
        (0xc24, 0x3f3f_3f3f, "TXAGC A OFDM"),
        (0xc30, 0x3f3f_3f3f, "TXAGC A MCS7-4"),
        (0xe20, 0x3f3f_3f3f, "TXAGC B CCK/OFDM"),
    ];

    let mode = std::env::args().nth(1).unwrap_or_else(|| "dump".into());
    let secs: u64 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(60);
    let ch: u8 = std::env::var("RP_CH").ok().and_then(|s| s.parse().ok()).unwrap_or(6);

    let b = std::sync::Arc::new(Rtl8812auBackend::open()?);
    b.power_on()?;
    b.mac_enable_dma()?;
    b.init_llt()?;
    b.download_firmware()?;
    b.mac_config()?;
    b.mac_init_queues()?;
    b.bb_config()?;
    b.rf_config()?;
    b.set_channel(ch)?;
    b.iq_calibrate()?;
    b.lc_calibrate()?;
    b.set_tx_power(0x3f)?;
    b.start_rx_dma()?;
    println!("8812AU pid={:#06x} up on ch{ch}\n", b.pid());

    // Optional pokes (applied AFTER bring-up, so they override whatever init left).
    if let Ok(poke) = std::env::var("POKE") {
        println!("applying POKE:");
        for kv in poke.split(',').filter(|s| !s.is_empty()) {
            let (a, v) = kv.split_once('=').expect("POKE = addr=val");
            let addr = parse_u32(a) as u16;
            let val = parse_u32(v);
            b.write32(addr, val)?;
            println!("  wrote {addr:#06x} = {val:#010x}");
        }
        println!();
    }

    println!("{:>7}  {:>12}  {:>12}   {}", "reg", "actual", "reference", "meaning");
    for &(addr, refval, note) in refs {
        let got = b.read32(addr)?;
        let flag = if got == refval { "ok" } else { "<< DIFF" };
        println!("{addr:#07x}  {got:#012x}  {refval:#012x}   {flag}  {note}");
    }

    if mode == "listen" {
        // Report RSSI of REGPROBE-BEACON frames AND count ALL ambient frames — a
        // busy channel (lots of ambient) means monitor injection (no carrier-sense)
        // collides with lab traffic, which is a contention loss, not weak signal.
        b.spawn_rx_pump(1);
        println!("\nlistening {secs}s (REGPROBE rssi + ambient frame rate) …");
        let t0 = std::time::Instant::now();
        let (mut n, mut sum, mut ambient) = (0i64, 0i64, 0i64);
        while t0.elapsed() < Duration::from_secs(secs) {
            if let Ok(Some(cf)) = b.poll_frame() {
                if cf.payload.windows(9).any(|w| w == b"REGPROBE-") {
                    if let Some(r) = cf.rssi_dbm {
                        n += 1;
                        sum += r as i64;
                    }
                } else {
                    ambient += 1;
                }
            }
        }
        let secs_f = t0.elapsed().as_secs_f64().max(1.0);
        println!(
            "\nTOTAL: {n} beacons, rssi avg {} dBm | AMBIENT: {ambient} frames ({:.0}/s) — \
             busy channel if high",
            if n > 0 { sum / n } else { 0 },
            ambient as f64 / secs_f
        );
        return Ok(());
    }

    if mode == "beacon" {
        // Inject a periodic tagged frame so a peer running `fec_fork rx` /
        // `burst_fork rx` (or nan_ndp measure) can read the RSSI under this
        // register state. 6 Mbps, broadcast.
        const SRC: [u8; 6] = [0x02, 0x4e, 0x44, 0x4e, 0x88, 0x20];
        const DST: [u8; 6] = [0xff; 6];
        let mut f = Vec::new();
        f.extend_from_slice(&[0x08, 0x00, 0x00, 0x00]);
        f.extend_from_slice(&DST);
        f.extend_from_slice(&SRC);
        f.extend_from_slice(&DST);
        f.extend_from_slice(&[0, 0]);
        f.extend_from_slice(b"REGPROBE-BEACON");
        f.resize(200, 0x5a);
        println!("\nbeaconing 200 B @ 6M for {secs}s (measure RSSI on the peer) …");
        let t0 = std::time::Instant::now();
        while t0.elapsed() < Duration::from_secs(secs) {
            b.send_frame(&f, 0x04)?;
            std::thread::sleep(Duration::from_millis(20));
        }
    }
    Ok(())
}

fn parse_u32(s: &str) -> u32 {
    let s = s.trim();
    if let Some(h) = s.strip_prefix("0x") {
        u32::from_str_radix(h, 16).expect("hex")
    } else {
        s.parse().expect("dec")
    }
}
