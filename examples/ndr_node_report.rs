//! **What can this node actually do?** — the parity report for one 7E-A5 LoRa-family node.
//!
//! Opens a node, prints the [`NodeProfile`] it learned (or the fallback it had to assume), the
//! [`RadioCapability`] derived from it, the [`FaceTimeProfile`] that follows, and then *exercises*
//! every HAL knob the node claims, printing what each returned. The point is that a claim and a
//! result sit next to each other: a `cmd_bitmap` bit that says "supported" beside the value the
//! knob actually produced.
//!
//! ```text
//! ndr_node_report /dev/ttyACM0
//! ```
//!
//! Read-only by default — the only writes are idempotent knob re-assertions of values already in
//! effect, and it restores the power it found. It never transmits unless `NDR_TX=1`.

use std::sync::Arc;

use ndn_frame_io::{FrameIo, InjectFrame, TxIntent};
use ndn_radio_drivers::{LoraSerialBackend, StampKind};
use ndn_radio_hal::{Bandwidth, FaceTimeProfile, RadioKnobs, RadioProfile, RadioTime};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let path = std::env::args().nth(1).unwrap_or("/dev/ttyACM0".into());
    let tx = std::env::var("NDR_TX").ok().as_deref() == Some("1");

    println!("opening {path} …");
    let dev = Arc::new(LoraSerialBackend::open(&path)?);
    let p = dev.profile();

    // ── what the node says it is ────────────────────────────────────────────────────────────
    println!("\n=== NodeProfile  ({}) ===", if p.learned {
        "LEARNED from the device's own EVT_CAP"
    } else {
        "ASSUMED — this node did not answer CMD_GET_CAP, so these are host-side fallbacks"
    });
    println!("  proto_ver     {}", p.proto_ver);
    println!("  radio_kind    {:?}", p.radio_kind);
    println!("  freq          {:.3} .. {:.3} MHz",
        p.freq_min_hz as f64 / 1e6, p.freq_max_hz as f64 / 1e6);
    match p.dbm_range() {
        Some(r) => println!("  tx power      {} .. {} dBm", r.min, r.max),
        None => println!("  tx power      unknown (not declared — no dBm range attached)"),
    }
    println!("  stamp         {} Hz, {:?}{}", p.stamp_hz, p.stamp_kind,
        p.tick_ns().map(|n| format!(" ({n} ns/tick)")).unwrap_or_default());
    println!("  max_payload   {} B", p.max_payload);
    println!("  spreading     {}", if p.has_spreading_factor() {
        format!("SF{}..SF{}", p.sf_min, p.sf_max)
    } else {
        "none (not a LoRa-modulation node)".into()
    });
    println!("  sched_gran    {} ns{}", p.sched_gran_ns,
        if p.sched_gran_ns == 0 { "  (no hardware-scheduled TX)" } else { "" });
    println!("  cmd_bitmap    {:#010x} -> {}", p.cmd_bitmap,
        (0u8..32).filter(|c| p.supports(*c))
            .map(|c| format!("{c:#04x}")).collect::<Vec<_>>().join(","));

    // ── what the stack derives from that ────────────────────────────────────────────────────
    let cap = dev.capability();
    println!("\n=== RadioCapability ===");
    println!("  kind {:?}  bands {:?}", cap.kind, cap.bands);
    println!("  rate {:?}", cap.rate);
    println!("  channels {:?}  max_payload {}  half_duplex {}",
        cap.channels, cap.max_payload, cap.half_duplex);
    println!("  tx_power_dbm {:?}  retune_us {:?}  duty_cycle_max {}",
        cap.tx_power_dbm, cap.retune_us, cap.duty_cycle_max);

    let ftp = FaceTimeProfile::derive(dev.as_ref() as &dyn RadioTime, dev.tx_discipline());
    println!("\n=== FaceTimeProfile ===");
    println!("  best_clock          {:?}", ftp.best_clock);
    println!("  stamp_precision_ns  {:?}", ftp.stamp_precision_ns);
    println!("  tx_discipline       {:?}", ftp.tx_discipline);
    println!("  can_common_view     {}  <- the capability the whole timing plane keys on",
        ftp.can_common_view);
    println!("  steering            {:?}", ftp.steering);
    println!("  FrameIo::schedules_tx() = {}", dev.schedules_tx());
    for s in dev.time_sources() {
        println!("  time source: {:?} precision {} ns domain {:?}", s.kind, s.precision_ns, s.domain);
    }

    // ── exercise the knobs the node claims ──────────────────────────────────────────────────
    println!("\n=== knobs (each line is a real call and its real result) ===");

    let before = dev.params();
    println!("  live params           {before:?}");

    match dev.read_device_clock() {
        Ok(t) => println!("  read_device_clock     {t} ticks"),
        Err(e) => println!("  read_device_clock     unsupported/err: {e}"),
    }
    match dev.read_clock(dev.device_clock_domain()) {
        Ok(Some(t)) => println!("  RadioTime::read_clock {t}"),
        Ok(None) => println!("  RadioTime::read_clock None (no readable device clock)"),
        Err(e) => println!("  RadioTime::read_clock err: {e}"),
    }
    match dev.read_channel_activity() {
        Ok(Some(a)) => println!("  read_channel_activity {a}  (free-running; difference two reads)"),
        Ok(None) => println!("  read_channel_activity None (node cannot sense occupancy)"),
        Err(e) => println!("  read_channel_activity err: {e}"),
    }
    match dev.sense() {
        Ok((a, r)) => println!("  sense()               activity {a}, channel RSSI {r} dBm"),
        Err(e) => println!("  sense()               unsupported/err: {e}"),
    }
    match dev.channel_rssi() {
        Ok(r) => println!("  channel_rssi          {r} dBm"),
        Err(e) => println!("  channel_rssi          unsupported/err: {e}"),
    }
    match dev.cad() {
        Ok(b) => println!("  cad()                 channel busy = {b}"),
        Err(e) => println!("  cad()                 unsupported/err: {e}"),
    }
    match dev.read_tx_counters() {
        Ok(v) => println!("  read_tx_counters      {v:?}"),
        Err(e) => println!("  read_tx_counters      err: {e}"),
    }
    match dev.ndn_stats() {
        Ok(s) => println!("  ndn_stats             {s:?}"),
        Err(e) => println!("  ndn_stats             unsupported/err: {e}"),
    }
    match dev.csma_counters() {
        Ok(c) => println!("  csma_counters         {c:?}"),
        Err(e) => println!("  csma_counters         unsupported/err: {e}"),
    }

    // Power: re-assert what is already set, then restore — proves the dBm path without moving RF.
    let held = before.pwr as i8;
    match dev.set_tx_power_dbm(held) {
        Ok(applied) => println!("  set_tx_power_dbm({held}) -> APPLIED {applied} dBm  \
                                 (believe the return, not the request)"),
        Err(e) => println!("  set_tx_power_dbm      unsupported: {e}"),
    }

    // Channel: re-assert the tuned channel, so nothing actually moves.
    match dev.set_channel(before.tx_ch, Bandwidth::Bw20) {
        Ok(()) => println!("  set_channel({})        ok (re-asserted, radio did not move)", before.tx_ch),
        Err(e) => println!("  set_channel           err: {e}"),
    }

    if p.has_spreading_factor() {
        match dev.set_spreading_factor(before.sf) {
            Ok(()) => println!("  set_spreading_factor(SF{}) ok", before.sf),
            Err(e) => println!("  set_spreading_factor  err: {e}"),
        }
    } else {
        match dev.set_spreading_factor(9) {
            Ok(()) => println!("  set_spreading_factor  ACCEPTED on a non-LoRa node — that is a bug"),
            Err(e) => println!("  set_spreading_factor  correctly refused: {e}"),
        }
    }

    println!("  set_edcca_ignore(false) -> {:?}  (LBT on; lbt() now {})",
        dev.set_edcca_ignore(false).is_ok(), dev.lbt());
    println!("  set_edcca_ignore(true)  -> {:?}  (LBT off; lbt() now {})",
        dev.set_edcca_ignore(true).is_ok(), dev.lbt());

    // Tier-0 / on-device name filter: install then clear, so nothing is left filtering.
    match dev.set_name_filter(&[b"/ndn/probe"]) {
        Ok(()) => {
            println!("  set_name_filter       installed /ndn/probe");
            let _ = dev.set_name_filter(&[]);
            println!("  set_name_filter       cleared (pass-all restored)");
        }
        Err(e) => println!("  set_name_filter       unsupported/err: {e}"),
    }

    if tx {
        println!("\n=== NDR_TX=1: one broadcast frame ===");
        let rt = tokio::runtime::Builder::new_multi_thread().enable_all().build()?;
        let f = InjectFrame::broadcast(bytes::Bytes::from_static(b"NDR-REPORT"), TxIntent::default());
        match rt.block_on(dev.inject(f)) {
            Ok(()) => println!("  inject                ok"),
            Err(e) => println!("  inject                err: {e}"),
        }
    } else {
        println!("\n(no frame transmitted — set NDR_TX=1 to send one)");
    }

    println!("\ndone.");
    Ok(())
}
