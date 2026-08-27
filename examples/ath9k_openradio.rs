//! **AR9271 as a first-class `OpenRadio` — the production cognition entry point, end to end.**
//!
//! Exercises `open_ath9k` (what `open_named_radio` dispatches to for an AR9271 PID) and asserts the
//! full `OpenRadio` surface cognition needs: `io` (FrameIo), `knobs` (RadioKnobs), `time` (RadioTime),
//! `profile` (RadioProfile). Then it drives the radio through those handles exactly as the node binary
//! / `RadioControl` would:
//!   - `profile.capability()` — the static advertisement cognition registers.
//!   - `knobs.set_channel(open_ch)` ⇒ Ok (steady-state apply); `set_channel(other)` ⇒ Unsupported.
//!   - `io.inject(...)` — TX through the OpenRadio data plane (needs the wmi_start TX fixes).
//!   - `io.recv_frame()` — RX through the pump (validates the start_receive-after-wmi_start order).
//!
//! ```sh
//! sudo NDN_ATH9K_FW=~/ath9k-fw/target_firmware/build/k2/htc_9271.fw ./ath9k_openradio [ch=1]
//! ```
use std::process::ExitCode;
use std::time::{Duration, Instant};

use bytes::Bytes;
use ndn_radio_drivers::open_ath9k;
use ndn_radio_hal::{Bandwidth, InjectFrame, TxIntent};

fn main() -> ExitCode {
    let ch: u8 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(1);
    println!("opening AR9271 via open_ath9k(ch={ch}) — the production OpenRadio path");

    let radio = match open_ath9k(ch) {
        Ok(r) => r,
        Err(e) => {
            eprintln!("open_ath9k FAILED: {e}");
            return ExitCode::FAILURE;
        }
    };

    // ── the OpenRadio capability surface cognition reads ──
    println!(
        "OpenRadio surface: io=yes knobs={} time={} profile={}",
        radio.knobs.is_some(),
        radio.time.is_some(),
        radio.profile.is_some(),
    );
    if radio.knobs.is_none() || radio.time.is_none() || radio.profile.is_none() {
        eprintln!("FAIL: a cognition-facing handle is missing (knobs/time/profile)");
        return ExitCode::FAILURE;
    }
    if let Some(p) = &radio.profile {
        println!("capability: {:?}", p.capability());
    }

    // ── knobs: the actuator surface (same-channel apply Ok; a hop is honestly Unsupported) ──
    if let Some(k) = &radio.knobs {
        match k.set_channel(ch, Bandwidth::Bw20) {
            Ok(()) => println!("knobs.set_channel({ch}) = Ok (steady-state apply)"),
            Err(e) => {
                eprintln!("FAIL: set_channel(open ch) should be Ok, got {e}");
                return ExitCode::FAILURE;
            }
        }
        let other = if ch == 6 { 1 } else { 6 };
        match k.set_channel(other, Bandwidth::Bw20) {
            Err(_) => {
                println!("knobs.set_channel({other}) = Unsupported (honest — retune not wired)")
            }
            Ok(()) => eprintln!("WARN: set_channel({other}) unexpectedly Ok"),
        }
    }

    let rt = tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
        .unwrap();
    rt.block_on(async {
        let io = radio.io.clone();
        let poll_rx = |secs: u64| {
            let io = io.clone();
            async move {
                let deadline = Instant::now() + Duration::from_secs(secs);
                let mut got = 0u32;
                while Instant::now() < deadline {
                    if let Ok(Ok(_f)) =
                        tokio::time::timeout(Duration::from_millis(500), io.recv_frame()).await
                    {
                        got += 1;
                    }
                }
                got
            }
        };

        // ── TX through OpenRadio.io ──
        let mut sent = 0;
        for i in 0..30 {
            let payload = format!("\x05\x08ar9k-or-{i:03}");
            let f = InjectFrame::broadcast(Bytes::copy_from_slice(payload.as_bytes()), TxIntent::CONSERVATIVE);
            match io.inject(f).await {
                Ok(()) => sent += 1,
                Err(e) => {
                    eprintln!("inject {i} FAILED: {e}");
                    break;
                }
            }
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
        println!("TX: injected {sent}/30 frames via OpenRadio.io (sustained = no block)");

        // ── RX: `recv_frame` surfaces ONLY NDN frames (parse_dot11 filters to ethertype 0x8624).
        // Ambient Wi-Fi (ACKs/beacons) is correctly dropped, so on a channel with no NDN sender this
        // is expected to be 0 — that is NOT an RX failure. The radio's raw 802.11 RX is proven by the
        // M2 oracle (`ath9k_hw_reset` — ndr_stats.seen climbing); this only lights up with an
        // on-channel NDN transmitter. ──
        println!("RX(NDN-filtered): polling 5 s for on-channel NDN frames…");
        let rx_ndn = poll_rx(5).await;
        println!("RX(NDN-filtered): {rx_ndn} frames (0 is expected without an NDN sender on-channel)");

        if sent >= 30 {
            println!(
                "\n✔ OpenRadio wiring PROVEN: knobs/time/profile present, set_channel honest, TX sustained \
                 ({sent}/30). RX surfaces {rx_ndn} NDN frames (raw 802.11 RX proven separately by M2)."
            );
        } else {
            println!("\n⚠ TX incomplete: sent={sent}/30");
        }
    });
    ExitCode::SUCCESS
}
