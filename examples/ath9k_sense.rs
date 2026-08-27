//! **AR9271 sensing surface — readable TSF clock (#3) + frame-free occupancy (#4).**
//!
//! Samples `RadioTime::read_clock` (the hardware TSF, µs) and `RadioKnobs::read_channel_activity`
//! (the MIB rx-busy cycle counter) once a second. The TSF should advance ~1e6/s; the occupancy
//! counter should climb with ambient channel activity (differenced by the cognition sampler).
//!
//! ```sh
//! sudo /tmp/ath9k_sense ~/ath9k-fw/target_firmware/build/k2/htc_9271.fw
//! ```
use std::process::ExitCode;
use std::thread::sleep;
use std::time::Duration;

use ndn_radio_drivers::Ath9kHtcBackend;
use ndn_radio_hal::{RadioKnobs, RadioTime};

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().collect();
    let fw = std::fs::read(args.get(1).expect("usage: <fw>")).expect("read fw");
    let mut dev = Ath9kHtcBackend::open().expect("open");
    dev.download_firmware(&fw)
        .and_then(|_| dev.htc_init())
        .expect("transport");
    dev.hw_reset(2412)
        .and_then(|_| dev.connect_data_services())
        .expect("bring-up");
    let _ = dev.write_target_u32s(0x0050_cf44, &[0]);
    dev.wmi_start()
        .and_then(|_| dev.start_receive())
        .expect("rx-start");

    let domain = dev.tsf_domain();
    println!("time  TSF(us)         dTSF(us)   occupancy  d(occ)");
    let (mut last_tsf, mut last_occ) = (0u64, 0u16);
    for i in 0..6 {
        let tsf = RadioTime::read_clock(&dev, domain)
            .ok()
            .flatten()
            .unwrap_or(0);
        let occ = RadioKnobs::read_channel_activity(&dev)
            .ok()
            .flatten()
            .unwrap_or(0);
        let dtsf = if i == 0 {
            0
        } else {
            tsf.wrapping_sub(last_tsf)
        };
        let docc = if i == 0 {
            0
        } else {
            occ.wrapping_sub(last_occ)
        };
        println!("{i:>3}s  {tsf:>12}   {dtsf:>9}   {occ:>8}   {docc:>6}");
        last_tsf = tsf;
        last_occ = occ;
        sleep(Duration::from_secs(1));
    }
    println!(
        "\ndTSF ~1_000_000/s ⇒ the µs clock is READABLE (read_clock live); d(occ) tracks ambient busy."
    );
    let _ = dev.detach();
    ExitCode::SUCCESS
}
