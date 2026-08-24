//! **AR9271 named airtime lease (#1 / §8.5) — hardware-scheduled TX on libusb.**
//!
//! Measures the saturating inject rate with the lease disarmed vs armed. When the generic-timer quiet
//! block is armed to keep the MAC quiet for most of each period, TX is gated to the leased slot and the
//! injected frames/s collapses — the airtime the MAC itself removed, no host involvement. (Proven on
//! air earlier via the kernel path; this exercises it through the libusb driver.)
//!
//! ```sh
//! sudo /tmp/ath9k_lease ~/ath9k-fw/target_firmware/build/k2/htc_9271.fw
//! ```
use std::process::ExitCode;
use std::time::{Duration, Instant};

use bytes::Bytes;
use ndn_radio_drivers::Ath9kHtcBackend;
use ndn_radio_hal::{FrameIo, InjectFrame, TxIntent};

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().collect();
    let fw = std::fs::read(args.get(1).expect("usage: <fw>")).expect("read fw");
    let mut dev = Ath9kHtcBackend::open().expect("open");
    dev.download_firmware(&fw).and_then(|_| dev.htc_init()).expect("transport");
    dev.hw_reset(2412).and_then(|_| dev.connect_data_services()).expect("bring-up");
    let _ = dev.write_target_u32s(0x0050_cf44, &[0]);
    dev.wmi_start().and_then(|_| dev.start_receive()).expect("rx-start");

    let rt = tokio::runtime::Builder::new_current_thread().enable_all().build().unwrap();
    let flood = |dev: &Ath9kHtcBackend, secs: u64| -> u64 {
        rt.block_on(async {
            let deadline = Instant::now() + Duration::from_secs(secs);
            let mut n = 0u64;
            while Instant::now() < deadline {
                let f = InjectFrame::broadcast(Bytes::copy_from_slice(b"\x05\x08lease"), TxIntent::CONSERVATIVE);
                if dev.inject(f).await.is_ok() {
                    n += 1;
                }
            }
            n
        })
    };

    // ndr_mac_state @ 0x0050cd7c: [magic, arm_count, quiet1, quiet2, quiet1_rb, quiet2_rb, quiet1_pre,
    //                              tsf_lo, tsf_hi, ifs_misc_rb, lease_slot, lease_base, lease_epoch, ...]
    const NDR_MAC_STATE: u32 = 0x0050_cd7c;
    let mac_state = |dev: &Ath9kHtcBackend, tag: &str| {
        if let Ok(w) = dev.read_target_u32s(NDR_MAC_STATE, 13) {
            println!(
                "  [{tag:>7}] arm_count={} quiet1_rb={:#010x} quiet2_rb={:#010x} slot={} base={} epoch={}",
                w[1], w[4], w[5], w[10], w[11], w[12]
            );
        }
    };

    // Phase 1 — no lease: full inject rate.
    dev.disarm_airtime_lease().ok();
    mac_state(&dev, "off");
    let free = flood(&dev, 3);
    println!("no lease:   {} frames / 3s = {}/s", free, free / 3);

    // Phase 2 — lease armed: 4 slots × 8 TU (period 32768 µs); this node owns slot 0 ⇒ MAC quiet 3/4
    // of every period ⇒ a saturating transmitter is gated to ~1/4 of the airtime.
    dev.arm_airtime_lease(4, 8, 0).expect("arm lease");
    mac_state(&dev, "armed");
    let leased = flood(&dev, 3);
    mac_state(&dev, "armed'");
    println!("lease armed:{} frames / 3s = {}/s  (4 slots × 8 TU, own slot 0)", leased, leased / 3);

    // Phase 3 — disarmed again: rate recovers.
    dev.disarm_airtime_lease().ok();
    let free2 = flood(&dev, 3);
    println!("disarmed:   {} frames / 3s = {}/s", free2, free2 / 3);

    let drop = if free > 0 { 100 - (leased * 100 / free.max(1)) } else { 0 };
    println!("\n→ lease removed ~{drop}% of transmit opportunity (MAC-gated, no host involvement) — §8.5.");
    println!("  (expected ~75% for 1-of-4 slots; arm_count>0 + quiet1_rb bit16 set = the PCU enforced it.)");
    let _ = dev.detach();
    ExitCode::SUCCESS
}
