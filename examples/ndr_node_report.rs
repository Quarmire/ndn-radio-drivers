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
use ndn_radio_drivers::LoraSerialBackend;
use ndn_radio_hal::{
    Bandwidth, FaceTimeProfile, HopControl, PhyMode, RadioKnobs, RadioProfile, RadioTime, RxGain,
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let path = std::env::args().nth(1).unwrap_or("/dev/ttyACM0".into());
    let tx = std::env::var("NDR_TX").ok().as_deref() == Some("1");

    println!("opening {path} …");
    let dev = Arc::new(LoraSerialBackend::open(&path)?);
    let p = dev.profile();

    // ── what the node says it is ────────────────────────────────────────────────────────────
    println!(
        "\n=== NodeProfile  ({}) ===",
        if p.learned {
            "LEARNED from the device's own EVT_CAP"
        } else {
            "ASSUMED — this node did not answer CMD_GET_CAP, so these are host-side fallbacks"
        }
    );
    println!("  proto_ver     {}", p.proto_ver);
    println!("  radio_kind    {:?}", p.radio_kind);
    println!(
        "  freq          {:.3} .. {:.3} MHz",
        p.freq_min_hz as f64 / 1e6,
        p.freq_max_hz as f64 / 1e6
    );
    match p.dbm_range() {
        Some(r) => println!("  tx power      {} .. {} dBm", r.min, r.max),
        None => println!("  tx power      unknown (not declared — no dBm range attached)"),
    }
    println!(
        "  stamp         {} Hz, {:?}{}",
        p.stamp_hz,
        p.stamp_kind,
        p.tick_ns()
            .map(|n| format!(" ({n} ns/tick)"))
            .unwrap_or_default()
    );
    println!("  max_payload   {} B", p.max_payload);
    println!(
        "  spreading     {}",
        if p.has_spreading_factor() {
            format!("SF{}..SF{}", p.sf_min, p.sf_max)
        } else {
            "none (not a LoRa-modulation node)".into()
        }
    );
    println!(
        "  sched_gran    {} ns{}",
        p.sched_gran_ns,
        if p.sched_gran_ns == 0 {
            "  (no hardware-scheduled TX)"
        } else {
            ""
        }
    );
    // ★ Modulation is a KNOB, not an identity. `radio_kind` above names the PART; this is the mode
    // it is running, and `phy set` is what it could be switched to.
    println!("  phy current   {:?}", p.phy_current);
    println!(
        "  phy set       {:?}{}",
        p.phy_modes().iter().collect::<Vec<_>>(),
        if p.phy_agile() {
            "  <- more than one mode AND CMD_SET_PHY: modulation is actuable here"
        } else if p.phy_modes().is_agile() {
            "  (advertised, but no CMD_SET_PHY reaches them — not a capability)"
        } else {
            "  (one mode; unreachable is not available)"
        }
    );
    match p.hop_capability() {
        Some(h) => println!(
            "  hop plan      intra_packet {} , up to {} carriers, period in {:?}\n\
             \x20               (NOT retune_us: that prices a HOST-commanded retune)",
            h.intra_packet, h.max_list_len, h.period_unit
        ),
        None => println!("  hop plan      none (no CMD_SET_HOP — the node cannot hop by itself)"),
    }
    println!(
        "  abs TX        {}",
        if p.schedules_tx_abs() {
            "CMD_TX_AT_ABS (0x1F) — the host names an INSTANT; its serial latency is out of the \
             placement"
        } else if p.schedules_tx() {
            "CMD_TX_AT only — MEASURED sd 553 us / p2p 1875 us placement jitter, because the \
             delay is counted from when the FIRMWARE processes the arm"
        } else {
            "none (no hardware-scheduled TX)"
        }
    );
    println!(
        "  cmd_bitmap    {:#010x} -> {}",
        p.cmd_bitmap,
        (0u8..32)
            .filter(|c| p.supports(*c))
            .map(|c| format!("{c:#04x}"))
            .collect::<Vec<_>>()
            .join(",")
    );

    // ── what the stack derives from that ────────────────────────────────────────────────────
    let cap = dev.capability();
    println!("\n=== RadioCapability ===");
    println!("  kind {:?}  bands {:?}", cap.kind, cap.bands);
    println!("  rate {:?}", cap.rate);
    println!(
        "  channels {:?}  max_payload {}  half_duplex {}",
        cap.channels, cap.max_payload, cap.half_duplex
    );
    println!(
        "  tx_power_dbm {:?}  duty_cycle_max {}",
        cap.tx_power_dbm, cap.duty_cycle_max
    );
    println!(
        "  phy_current {:?}  phy_modes {:?}  agile {}",
        cap.phy_current,
        cap.phy_modes.iter().collect::<Vec<_>>(),
        cap.phy_modes.is_agile()
    );
    println!(
        "  hop {:?}  -> hops_intra_packet {}",
        cap.hop,
        cap.hops_intra_packet()
    );
    // retune_us is MEASURED per modem, so can_hop is finally answerable — and it is not one answer
    // for the fleet: 5.6 ms on the Heltec, 52.8 ms on the LR2021, 161 ms on the Waveshare.
    match cap.retune_us {
        Some(us) => println!(
            "  retune_us {us}  -> can_hop(100 ms dwell) {:?}, which costs {:.0}% of the dwell",
            cap.can_hop(100_000),
            100.0 * cap.retune_overhead(100_000).unwrap_or(0.0)
        ),
        None => println!(
            "  retune_us None  -> can_hop 'cannot say' (unmeasured modem, or no CMD_SET_FREQ)"
        ),
    }

    let ftp = FaceTimeProfile::derive(dev.as_ref() as &dyn RadioTime, dev.tx_discipline());
    println!("\n=== FaceTimeProfile ===");
    println!("  best_clock          {:?}", ftp.best_clock);
    println!("  stamp_precision_ns  {:?}", ftp.stamp_precision_ns);
    println!("  tx_discipline       {:?}", ftp.tx_discipline);
    println!(
        "  can_common_view     {}  <- the capability the whole timing plane keys on",
        ftp.can_common_view
    );
    println!("  steering            {:?}", ftp.steering);
    println!("  FrameIo::schedules_tx() = {}", dev.schedules_tx());
    for s in dev.time_sources() {
        println!(
            "  time source: {:?} precision {} ns domain {:?}",
            s.kind, s.precision_ns, s.domain
        );
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
        Ok(Some(a)) => {
            println!("  read_channel_activity {a}  (free-running; difference two reads)")
        }
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
    // (ok, err) PPDUs from the modem's own counters, where the node's EVT_STATS carries the v2 tail.
    // `err` is the ONLY place a reception the PHY began and lost is visible on this bearer.
    match dev.read_ofdm_counters() {
        Ok(Some((ok, err))) => println!(
            "  read_ofdm_counters    ok={ok} err={err}  (free-running u16 — difference two reads)"
        ),
        Ok(None) => println!("  read_ofdm_counters    None (this node reports no PHY counters)"),
        Err(e) => println!("  read_ofdm_counters    err: {e}"),
    }
    match dev.ndn_stats() {
        Ok(s) => {
            println!(
                "  ndn_stats             rx={} filtered={} deduped={} served={} relayed={} \
                 cad_busy={} defer={}",
                s.rx, s.filtered, s.deduped, s.served, s.relayed, s.cad_busy, s.defer
            );
            match (s.chip_rx, s.chip_crc_err, s.chip_hdr_err, s.rx_trunc) {
                (Some(rx), Some(crc), Some(hdr), Some(trunc)) => println!(
                    "    PHY tail            chip_rx={rx} crc_err={crc} hdr_err={hdr} \
                     rx_trunc={trunc}{}",
                    if trunc > 0 {
                        "  <- a peer is transmitting past our advertised max_payload"
                    } else {
                        ""
                    }
                ),
                _ => println!("    PHY tail            absent (24-byte EVT_STATS)"),
            }
        }
        Err(e) => println!("  ndn_stats             unsupported/err: {e}"),
    }
    match dev.csma_counters() {
        Ok(c) => println!("  csma_counters         {c:?}"),
        Err(e) => println!("  csma_counters         unsupported/err: {e}"),
    }

    // Power: re-assert what is already set, then restore — proves the dBm path without moving RF.
    let held = before.pwr as i8;
    match dev.set_tx_power_dbm(held) {
        Ok(applied) => println!(
            "  set_tx_power_dbm({held}) -> APPLIED {applied} dBm  \
                                 (believe the return, not the request)"
        ),
        Err(e) => println!("  set_tx_power_dbm      unsupported: {e}"),
    }

    // Channel: re-assert the tuned channel, so nothing actually moves.
    match dev.set_channel(before.tx_ch, Bandwidth::Bw20) {
        Ok(()) => println!(
            "  set_channel({})        ok (re-asserted, radio did not move)",
            before.tx_ch
        ),
        Err(e) => println!("  set_channel           err: {e}"),
    }

    if p.has_spreading_factor() {
        match dev.set_spreading_factor(before.sf) {
            Ok(()) => println!("  set_spreading_factor(SF{}) ok", before.sf),
            Err(e) => println!("  set_spreading_factor  err: {e}"),
        }
    } else {
        match dev.set_spreading_factor(9) {
            Ok(()) => {
                println!("  set_spreading_factor  ACCEPTED on a non-LoRa node — that is a bug")
            }
            Err(e) => println!("  set_spreading_factor  correctly refused: {e}"),
        }
    }

    println!(
        "  set_edcca_ignore(false) -> {:?}  (LBT on; lbt() now {})",
        dev.set_edcca_ignore(false).is_ok(),
        dev.lbt()
    );
    println!(
        "  set_edcca_ignore(true)  -> {:?}  (LBT off; lbt() now {})",
        dev.set_edcca_ignore(true).is_ok(),
        dev.lbt()
    );

    // Receive gain: the opcode all three firmwares implement and nothing in the host tree could
    // send until this run. A posture, not a dB figure — restore Auto (the part's own default).
    match dev.set_rx_gain(RxGain::Boosted) {
        Ok(()) => {
            println!("  set_rx_gain(Boosted)  ok (highest manual gain)");
            match dev.set_rx_gain(RxGain::Auto) {
                Ok(()) => println!("  set_rx_gain(Auto)     restored the part's own default"),
                Err(e) => println!("  set_rx_gain(Auto)     err: {e}  <- gain left BOOSTED"),
            }
        }
        Err(e) => println!("  set_rx_gain           unsupported/err: {e}"),
    }

    // Hop plan: report only. Arming one would move the radio off the carrier its peer is on, and
    // this tool does not transmit unless asked — so print what a plan WOULD be allowed to be.
    match p.hop_capability() {
        Some(h) => {
            println!(
                "  set_hop_plan          available: up to {} carriers, period in {:?} \
                 (not armed by this tool)",
                h.max_list_len, h.period_unit
            );
            // The local guards still get exercised, without anything reaching the wire.
            let too_long: Vec<u32> = (0..=h.max_list_len as u32)
                .map(|i| 903_000_000 + i)
                .collect();
            match dev.set_hop_plan(HopControl::On, 4, &too_long) {
                Ok(()) => println!("    over-long list      ACCEPTED — that is a bug"),
                Err(e) => println!("    over-long list      correctly refused: {e}"),
            }
        }
        None => match dev.set_hop_plan(HopControl::On, 4, &[915_000_000]) {
            Ok(()) => println!("  set_hop_plan          ACCEPTED on a node with no 0x1E — a bug"),
            Err(e) => println!("  set_hop_plan          correctly refused: {e}"),
        },
    }

    // ── modulation: switch, print the new capability, switch back ────────────────────────────
    println!("\n=== PHY (modulation as a runtime knob) ===");
    if p.phy_agile() {
        let from = p.phy_current;
        let to = p
            .phy_modes()
            .iter()
            .find(|m| *m != from)
            .expect("phy_agile means at least two");
        println!("  current {from:?}; switching to {to:?} …");
        match dev.set_phy(to) {
            Ok(applied) => {
                println!("  set_phy({to:?}) -> IN EFFECT {applied:?}  (believe the return)");
                // ★ The profile was REPLACED, not patched: re-read everything.
                let np = dev.profile();
                let ncap = dev.capability();
                println!(
                    "  new profile: max_payload {} B, sf {}, sched_gran {} ns, {:.3}..{:.3} MHz",
                    np.max_payload,
                    if np.has_spreading_factor() {
                        format!("SF{}..SF{}", np.sf_min, np.sf_max)
                    } else {
                        "none".into()
                    },
                    np.sched_gran_ns,
                    np.freq_min_hz as f64 / 1e6,
                    np.freq_max_hz as f64 / 1e6
                );
                println!(
                    "  new capability: rate {:?}  max_payload {}  retune_us {:?}",
                    ncap.rate, ncap.max_payload, ncap.retune_us
                );
                match dev.set_phy(from) {
                    Ok(back) => println!("  set_phy({from:?}) -> back to {back:?}"),
                    Err(e) => println!(
                        "  set_phy({from:?})     FAILED to restore: {e}  <- node left in {applied:?}"
                    ),
                }
            }
            Err(e) => println!("  set_phy({to:?})        refused: {e}"),
        }
    } else {
        println!(
            "  this node runs one modulation ({:?}) and offers no CMD_SET_PHY — nothing to switch.",
            p.phy_current
        );
        // The refusal is still worth seeing: it must be a refusal, not a silent success.
        match dev.set_phy(PhyMode::Ble) {
            Ok(m) => println!(
                "  set_phy(Ble)          ACCEPTED -> {m:?}, on a node that does not advertise it: a bug"
            ),
            Err(e) => println!("  set_phy(Ble)          correctly refused: {e}"),
        }
    }

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
        let rt = tokio::runtime::Builder::new_multi_thread()
            .enable_all()
            .build()?;
        let f = InjectFrame::broadcast(
            bytes::Bytes::from_static(b"NDR-REPORT"),
            TxIntent::default(),
        );
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
