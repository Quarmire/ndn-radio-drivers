//! **AR9271 sustained-RX ceiling diagnostic — where and why does RX stall (~1226 frames)?**
//!
//! Counts RAW ambient 802.11 RX (the inherent unfiltered `recv_frame` → `parse_rx`, so no NDN sender
//! is needed — ch1 ambient traffic drives it) for 60 s, and every second queries `WMI_RX_STATS`:
//!   ast_rx_send  — frames the target handed to the host
//!   ast_rx_done  — HTC send-completions that RECYCLED a buffer (fires when the host reads bulk-IN)
//!   ast_rx_nobuf — RX-buffer alloc failures = the pool drained (the leak/ceiling witness)
//!
//! Reading the three at the stall localizes it: nobuf climbing ⇒ pool exhausted; send racing ahead of
//! done ⇒ host isn't draining bulk-IN fast enough to recycle; send frozen ⇒ the MAC RX itself stopped.
//!
//! ```sh
//! sudo /tmp/ath9k_rx_ceiling ~/ath9k-fw/target_firmware/build/k2/htc_9271.fw 2412
//! ```
use std::process::ExitCode;
use std::time::{Duration, Instant};

use ndn_radio_drivers::Ath9kHtcBackend;

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().collect();
    let fw = std::fs::read(args.get(1).expect("usage: <fw> [chan_mhz]")).expect("read fw");
    let chan_mhz: u16 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(2412);
    let secs: u64 = args.get(3).and_then(|s| s.parse().ok()).unwrap_or(60);
    println!("firmware {} B, ch {chan_mhz} MHz, {secs}s raw-RX ceiling probe", fw.len());

    let mut dev = match Ath9kHtcBackend::open() {
        Ok(d) => d,
        Err(e) => {
            eprintln!("open FAILED: {e}");
            return ExitCode::FAILURE;
        }
    };
    if let Err(e) = dev.download_firmware(&fw).and_then(|_| dev.htc_init()) {
        eprintln!("transport FAILED: {e}");
        return ExitCode::FAILURE;
    }
    // Confirm WHICH firmware is running (instrument-trap guard): the RX-pool-slack build sets minor=80.
    println!("running firmware version: {:?}", dev.fw_version());
    // Same bring-up as open_ath9k (order: wmi_start before start_receive; filter off).
    if let Err(e) = dev.hw_reset(chan_mhz).and_then(|_| dev.connect_data_services()) {
        eprintln!("bring-up FAILED: {e}");
        return ExitCode::FAILURE;
    }
    let _ = dev.write_target_u32s(0x0050_cf44, &[0]);
    if let Err(e) = dev.wmi_start().and_then(|_| dev.start_receive()) {
        eprintln!("rx-start FAILED: {e}");
        return ExitCode::FAILURE;
    }

    println!("time  frames  rate/s   rx_send  rx_done  rx_nobuf  (send-done)");
    let start = Instant::now();
    let mut total = 0u64;
    let mut last_total = 0u64;
    let mut stalled_secs = 0u32;
    let deadline = start + Duration::from_secs(secs);
    while Instant::now() < deadline {
        let sec_start = Instant::now();
        // Drain RX for ~1 s (blocking reads with a short timeout so the loop stays ~1 s).
        while sec_start.elapsed() < Duration::from_millis(1000) {
            match dev.recv_frame(Duration::from_millis(40)) {
                Ok(Some(_f)) => total += 1,
                Ok(None) => {}
                Err(_) => {} // timeout — keep going
            }
        }
        let (nobuf, send, done) = dev.rx_stats().unwrap_or((u32::MAX, u32::MAX, u32::MAX));
        let t = start.elapsed().as_secs();
        let rate = total - last_total;
        println!(
            "{t:>3}s  {total:>6}  {rate:>5}   {send:>7}  {done:>7}  {nobuf:>7}    {}",
            send.wrapping_sub(done)
        );
        if total == last_total {
            stalled_secs += 1;
            if stalled_secs == 3 {
                println!(
                    "→ STALLED at {total} frames after {t}s. rx_send={send} rx_done={done} \
                     rx_nobuf={nobuf}. {}",
                    if nobuf > 0 {
                        "nobuf>0 ⇒ RX-buffer pool EXHAUSTED (recycle can't keep up / leak)."
                    } else if send.wrapping_sub(done) > 8 {
                        "send≫done ⇒ host not draining bulk-IN → buffers not recycled."
                    } else {
                        "send frozen with buffers free ⇒ MAC RX stopped (not a buffer issue)."
                    }
                );
            }
        } else {
            stalled_secs = 0;
        }
        last_total = total;
    }
    let (nobuf, send, done) = dev.rx_stats().unwrap_or((0, 0, 0));
    println!("\nFINAL: {total} frames | rx_send={send} rx_done={done} rx_nobuf={nobuf}");
    let _ = dev.detach();
    ExitCode::SUCCESS
}
