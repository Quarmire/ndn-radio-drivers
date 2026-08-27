//! **AR9271 Tier-0 name filter over libusb (#2 / §8.2) — the pre-USB drop, live on the allowed path.**
//!
//! Brings up RX, then toggles the firmware Tier-0 filter via `set_name_filter` and reads `ndr_stats`
//! across two windows: filter OFF (every frame crosses USB, passed≈seen) then ON with `drop_foreign`
//! (ambient non-group frames dropped ON THE DONGLE — `dropped_foreign` climbs, `passed` grows slower).
//! `seen - passed` = USB transfers and host wakeups that did NOT happen — the §8.2 win.
//!
//! ```sh
//! sudo /tmp/ath9k_tier0 ~/ath9k-fw/target_firmware/build/k2/htc_9271.fw
//! ```
use std::process::ExitCode;
use std::thread::sleep;
use std::time::Duration;

use ndn_radio_drivers::Ath9kHtcBackend;

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().collect();
    let fw = std::fs::read(args.get(1).expect("usage: <fw>")).expect("read fw");
    let mut dev = Ath9kHtcBackend::open().expect("open");
    dev.download_firmware(&fw)
        .and_then(|_| dev.htc_init())
        .expect("transport");
    // NOTE: deliberately NOT disabling the filter here — this test controls it via set_name_filter.
    dev.hw_reset(2412)
        .and_then(|_| dev.connect_data_services())
        .expect("bring-up");
    dev.wmi_start()
        .and_then(|_| dev.start_receive())
        .expect("rx-start");

    let show = |dev: &Ath9kHtcBackend, tag: &str| match dev.ndr_stats() {
        Ok(s) => println!(
            "  [{tag:>9}] seen={} passed={} dropped_filter={} dropped_foreign={} short={} popcount={}",
            s.seen,
            s.passed,
            s.dropped_filter,
            s.dropped_foreign,
            s.short_frame,
            s.dropped_popcount
        ),
        Err(e) => println!("  [{tag:>9}] ndr_stats read FAILED: {e}"),
    };

    println!("filter OFF (stock — every frame crosses USB):");
    dev.set_name_filter(false, false)
        .expect("set_name_filter off");
    show(&dev, "t=0");
    sleep(Duration::from_secs(3));
    show(&dev, "t=3s");

    println!("filter ON + drop_foreign (drop non-group frames ON THE DONGLE, pre-USB):");
    dev.set_name_filter(true, true).expect("set_name_filter on");
    show(&dev, "t=3s");
    sleep(Duration::from_secs(3));
    show(&dev, "t=6s");
    sleep(Duration::from_secs(3));
    show(&dev, "t=9s");
    println!(
        "\n→ dropped_foreign climbing while passed lags seen ⇒ frames dropped before the USB transfer (§8.2)."
    );

    let _ = dev.set_name_filter(false, false); // restore stock before exit
    let _ = dev.detach();
    ExitCode::SUCCESS
}
