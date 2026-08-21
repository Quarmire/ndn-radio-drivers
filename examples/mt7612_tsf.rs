//! **MT7612U hardware-timestamp probe** (RadioTime bring-up, #P1a).
//!
//! Two measurements, both decide whether the mt76x2 can be a common-view clock participant:
//!  1. Is register `0x1104` (candidate `MT_TSF_TIMER_DW0`) a free-running ~1 MHz µs counter? Read it
//!     at known host intervals; a real TSF advances ~10000 per 10 ms step.
//!  2. Does the 36-byte RX descriptor prefix (MT_RX_INFO + RXWI) carry a *receive-latched* timestamp
//!     (like the Realtek RXTSFL at descriptor dword5)? Dump the prefix next to a concurrent `0x1104`
//!     read; a hidden RX TSF would be a 4-byte field tracking (just below) the live register.
//!
//! Run on minidronesys-05 (the MT7612U host), kernel driver unbound:
//!   echo -n 2-1.3.4:1.0 | sudo tee /sys/bus/usb/drivers/mt76x2u/unbind
//!   sudo LD_LIBRARY_PATH=$(nix path-info nixpkgs#libusb1)/lib ./mt7612_tsf
fn main() -> Result<(), Box<dyn std::error::Error>> {
    use ndn_radio_drivers::Mt7612uBackend;
    use ndn_radio_drivers::rx_pump::Pumpable; // pump_handle / pump_bulk_in
    use std::time::{Duration, Instant};

    let dev = Mt7612uBackend::open()?;
    dev.bring_up()?;
    dev.set_channel_ch6()?;
    dev.setup_monitor_rx()?;
    dev.pause_drain(true); // don't let the init drain steal the transfers we inspect

    // (0) rr() latency floor — the jitter of a per-frame register-read stamp vs air arrival.
    let t = Instant::now();
    for _ in 0..200 {
        let _ = dev.rr(0x1004)?;
    }
    println!("rr() latency: {:.1} us/read (200 reads) — the per-frame register-read jitter floor", t.elapsed().as_micros() as f64 / 200.0);

    // (0b) Start the TSF timer if it's disabled in monitor mode. MT_BEACON_TIME_CFG (0x1100):
    // bit4 = TIMER_EN; sync-mode (bits 8-9) = 0 → free-run. Clear DW0/DW1 first.
    let bc = dev.rr(0x1100)?;
    println!("MT_BEACON_TIME_CFG(0x1100) before = {bc:#010x}");
    dev.wr(0x1104, 0)?;
    dev.wr(0x1108, 0)?;
    dev.wr(0x1100, (bc & !0x0000_0300) | 0x0000_0010)?; // TIMER_EN, free-run
    println!("MT_BEACON_TIME_CFG(0x1100) after  = {:#010x}", dev.rr(0x1100)?);

    // (1) Is 0x1104 a µs TSF? Read the candidate DW0/DW1 (+ 0x110c MT_TIME_STAMP) at 10 ms steps.
    println!("=== TSF register advance (10 ms steps; a µs TSF adds ~10000/step to 0x1104) ===");
    let t0 = Instant::now();
    let mut last = dev.rr(0x1104)?;
    for _ in 0..8 {
        std::thread::sleep(Duration::from_millis(10));
        let el = t0.elapsed().as_micros();
        let dw0 = dev.rr(0x1104)?;
        let dw1 = dev.rr(0x1108)?;
        let ts = dev.rr(0x110c)?;
        let d = dw0.wrapping_sub(last);
        last = dw0;
        println!("  host={el:>9}us  0x1104={dw0:#010x} (+{d:<8})  0x1108={dw1:#010x}  0x110c={ts:#010x}");
    }

    // (2) Does the RX descriptor carry a receive-latched timestamp? Dump the 36-byte prefix beside a
    // concurrent 0x1104 read. Look (offline) for a 4-byte little-endian field ≈ (just below) tsf.
    println!("\n=== RXD prefix (36 B) vs live TSF 0x1104 — hunt a receive-latched field ===");
    let handle = dev.pump_handle();
    let ep = dev.pump_bulk_in();
    let mut buf = vec![0u8; 32768];
    let mut got = 0u32;
    let deadline = Instant::now() + Duration::from_secs(10);
    while got < 16 && Instant::now() < deadline {
        match handle.read_bulk(ep, &mut buf, Duration::from_millis(200)) {
            Ok(n) if n >= 40 => {
                let tsf = dev.rr(0x1104).unwrap_or(0);
                // Print the 36-byte RXD prefix as 9 little-endian u32 words (so a TSF field is
                // readable at a glance), plus the raw transfer length.
                let w = |o: usize| u32::from_le_bytes([buf[o], buf[o + 1], buf[o + 2], buf[o + 3]]);
                let words: Vec<String> = (0..9).map(|i| format!("{:08x}", w(i * 4))).collect();
                println!("  n={n:<5} tsf=0x{tsf:08x}  rxd[0..9]= {}", words.join(" "));
                got += 1;
            }
            Ok(_) => {}
            Err(_) => {}
        }
    }
    if got == 0 {
        println!("  (no frames captured — check channel has traffic)");
    }
    Ok(())
}
