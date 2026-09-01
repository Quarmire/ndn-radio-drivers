//! Userspace USB Wi-Fi monitor-mode driver backends over the `ndn-radio-hal` contract.
//!
//! Split out of `ndn-face-monitor-wifi` so drivers have a dedicated home; each
//! backend implements `FrameIo` + `WifiRadio` against the HAL and does no NDN
//! forwarding.

// Re-export the contract surface the backend modules reference as `crate::…`
// (they were written as modules of ndn-face-monitor-wifi, which re-exported these).
pub use ndn_frame_io::{
    BROADCAST, CapturedFrame, DEFAULT_SRC, FaceError, FaceId, FrameFormat, FrameIo, InjectFrame,
    MAX_RELIABLE_MCS, McsDescriptor, McsPolicy, Reach, Reliability, TxIntent, frame, mcs_for_rssi,
    mcs_phy_rate_bps, radiotap,
};
// #78: the capability traits `OpenRadio` hands out. Re-exported so a caller of `open_named_radio`
// needs exactly one import to use everything the opener returns.
// #78: `OpenRadio` and the capability traits it carries live in the HAL, beside the traits they
// aggregate — a driver crate builds one and a face crate consumes one, so neither should need a
// dependency on the other to name it.
pub use ndn_radio_hal::{OpenRadio, RadioKnobs, RadioProfile, RadioTime};

pub mod mt76;
/// Shared MediaTek mt76x02 layer: the register map, USB transport and the MEASURED knob
/// implementations common to the mt76x0 (MT7610U) and mt76x2 (MT7612U) parts. The two families
/// ship one register header upstream, so a knob validated on either is validated for both.
pub mod realtek_contention;

/// Selecting one dongle among several identical ones (by index or USB bus:port) + a guard against
/// claiming the device that currently carries a live kernel link. Shared by the Realtek backends.
pub mod usb_select;
pub use usb_select::{DeviceSelect, usb_addr};

/// Closed-loop frequency discipline: spend the time layer's skew estimate on a radio's clock trim.
pub mod freq_discipline;
mod libusb_rtl88xx;
/// Shared Realtek RX-descriptor field decode (RSSI/MCS/timestamp) used by the USB backends.
mod realtek_rx;
/// Shared async-URB RX pump (bulk-IN pipelining) used by the USB backends.
pub mod rx_pump;
pub use freq_discipline::{FreqAction, FreqDiscipline};
pub use libusb_rtl88xx::{
    CHIP_ID_8822E, ChannelBw, FwVersion, LibUsbRtl88xxBackend, REALTEK_VID, REG_SYS_CFG,
    RTL88XX_PIDS, RfPath,
};
// AR9271 (ath9k_htc) — the one Wi-Fi part whose FIRMWARE is ours, so Tier-0 can reject a frame
// before it crosses USB (design §8.2) and TX can be scheduled off the hardware TSF (§8.5).
// L1: USB transport + firmware download + HTC handshake + WMI. Does not yet replace ath9k_htc.
mod ath9k_htc;
pub mod coverage;
// PHY-init data for the M1 bring-up port, transcribed verbatim from mainline ath9k v6.12.33:
// AR9271 initval tables (ar9002_initvals.h), the register offsets/bits the reset+cal path writes
// (reg.h / ar9002_phy.h / mac.h), and the HTC wire structs (htc.h). Consumed by ath9k_htc.rs.
mod ath9k_htc_structs;
mod ath9k_initvals;
mod ath9k_reg;
pub use ath9k_htc::{
    AR9271_FIRMWARE, AR9271_FIRMWARE_TEXT, AR9271_IDS, ATHEROS_VID, Ath9kHtcBackend, BoardValues,
    CalStatus, FW_NAME, HTC_RX_STATUS_LEN, HtcService, IEEE80211_MODE_11NG, IniVerify, LegacyRate,
    NDR_MEM_MAX_TUPLES, NdrStats, REG_WRITE_MAX_PAIRS, ResetStatus, RxFrame, WmiCmd,
};
mod rtl8821c;
pub use rtl8821c::{RTL8821CU_PIDS, Rtl8821cuBackend};
mod mt7612;
pub use mt7612::{MT7612U_PIDS, Mt7612uBackend};
// MT7610U (mt76x0u, 1x1 dual-band 802.11ac) — the sibling port. Shares `mt76x02_regs.h` with the
// MT7612U above, so the two share `crate::mt76`'s register map, transport and knob layer; what
// differs is the firmware (no ROM patch), the RF programming model (host-programmable via
// MT_RF_CSR_CFG, MEASURED working over USB) and the 1x1 chain configuration.
mod mt76x0;
pub use mt76x0::{MT7610U_PIDS, Mt7610uBackend};
/// Shared MediaTek **connac2** layer (MT7921/MT792x): a different architecture from `mt76`, not a
/// newer revision of one — extended vendor requests, a composite BT+WLAN device, patch+RAM
/// firmware, and a variable-length RX descriptor whose group 2 carries a per-frame hardware
/// timestamp.
pub mod connac2;
// MT7921AU (connac2, 2x2 802.11ax) — the only Wi-Fi part in this crate that can actuate the HAL's
// HE levers, and the only MediaTek one that can source common view.
mod mt7921;
pub use mt7921::{MT7921U_PIDS, Mt7921uBackend};
mod rtl8812au;
pub use rtl8812au::{ChipInfo, IqkResult, PhySense, RTL8812AU_PIDS, Rtl8812auBackend};
// RTL8731BU / RTL8733BU (halmac_87xx, 1x1 802.11n, dual-band, 20/40) — ground-up port, complete
// against the HAL: power-on,
// firmware download, MAC/BB/RF init, 1x1 calibration (IQK/TXGAPK/DPK), channel/power, RX capture and
// on-air TX, impl'ing FrameIo + RadioKnobs + RadioTime + RadioProfile. It is the *reference* RadioTime
// implementation (both link clocks: free-run RX stamp + port TSF). Open it through `open_named_radio`.
// REALTEK_VID is already re-exported above.
mod libusb_rtl8733b;
pub use libusb_rtl8733b::{
    ChipVersion, FW_NIC_8733B, FwHeader, PowerTracker, RTL8733B_PIDS, Rtl8733buBackend,
};

// Serial-bridged 802.11 backend — a raw injector/capturer driven over USB-serial (the ND wire protocol),
// backing a BW16 (RTL8720DN, firmware/bw16-ndn-bridge) or an ESP32-C5 (firmware/esp32c5-ndn), implementing
// the same FrameIo/WifiRadio/RadioKnobs contract as the USB drivers.
#[cfg(feature = "serial-radio")]
mod serial_radio;
#[cfg(feature = "serial-radio")]
pub use serial_radio::{
    Bw16SerialBackend, ChannelProfile, Esp32SerialBackend, SERIAL_RADIO_BAUD, SerialRadioBackend,
    bw16_clock_domain,
};

// The 7E-A5 serial sub-GHz fleet — Waveshare SX1262, Heltec SX1276, and the nRF54L15+LR2021 bridge —
// behind ONE capability-driven backend implementing the same FrameIo/RadioKnobs/RadioTime/RadioProfile
// contract as the USB drivers. What each node can do is learned from its own EVT_CAP at open
// (`NodeProfile`), never assumed from its family; see src/lora_serial.rs.
//
// v3 of that protocol adds the **modulation** axis: a node reports which PHYs it can run and which
// it is in, and `RadioKnobs::set_phy` switches it — so `LoraRadioKind` names the PART again rather
// than the part-plus-mode it was compiled with. The PhyMode/PhyModeSet/HopCapability/RxGain types
// live in `ndn-radio-hal` (they are not LoRa-specific) and are reached through it.
#[cfg(feature = "lora")]
mod lora_serial;
#[cfg(feature = "lora")]
pub use lora_serial::{
    HOP_LIST_MAX, LORA_BAUD, LoraParams, LoraRadioKind, LoraSerialBackend, MAX_LORA_PAYLOAD,
    NdnStats, NodeProfile, PROTO_VER, RadioKindHint, StampKind, lora_clock_domain, name_hash,
};

// Newracom NRC7292 (802.11ah/S1G) read-now clock. The AF_PACKET monitor backend surfaces this
// radio's per-frame radiotap TSFT but reports `read_clock() = None`, which is true of a packet
// socket in general and false of this radio: its firmware keeps a microsecond counter in chip RAM
// that the vendor CLI can sample on demand, MEASURED to be the same clock that stamps frames. That
// pair (per-frame stamps + a readable clock) is what a common-view estimator needs; a two-node
// common view over ordinary beacons measured sd = 5.8 µs with no firmware change (see nrc7292.rs).
// Also this radio's cognition control surface (`Nrc7292Knobs`): channel (via `iw`, verified by
// read-back), the genuine 1-30 dBm TX-power axis, airtime shaping through `set tx_time`, the dBm
// CCA threshold, and the FCS-error / carrier-sense counters. Eleven RadioKnobs methods stay at the
// trait default with a written reason each. ★ `cli_app` ALWAYS EXITS 0 and has NO interface
// selector — both traps are handled in one place; read the type docs before adding a knob.
// Morse Micro MM6108 (802.11ah/S1G) channel control. The generic `iw dev … set channel` CANNOT
// tune this radio — it returns -16 EBUSY on a Morse monitor vif (measured) — so tuning must go
// through the vendor nl80211 command that `morse_cli` wraps. S1G needs FOUR parameters, and the
// primary-channel index is the one that silently decides whether anything is heard at all
// (measured: index 1 heard a 4 MHz AP, indices 0/2/3 heard nothing). See src/morse.rs.
pub mod morse;

pub mod nrc7292;

// The 802.11ah (HaLow / S1G) DATA plane for both sub-GHz radios — injection, capture, and the
// per-frame metadata that comes back with a frame. Kept as one module because the two parts share
// the body format, the radiotap TX header and (verified from both vendors' sources) a byte-identical
// S1G radiotap TLV; what they do NOT share is the shape of the data plane itself, and getting that
// wrong is silent on both. ☠ The MM6108's `morse0` accepts a `sendto()` and radiates nothing, so its
// TX and RX must be different netdevs. Also the single home for what the two HaLow backends must
// answer IDENTICALLY: the capability skeleton (`halow_base`) and the `Bandwidth` reading
// (`s1g_width_request`).
// ⚠ OPEN HAL GAP: `ndn_radio_hal::Bandwidth` enumerates 20/40/80/10/5 MHz and cannot express S1G's
// 1/2/4/8 MHz at all, so `set_channel`'s width argument is unusable on this bearer and width has to
// travel with the channel number. `s1g_width_request` is the agreed workaround, not a fix.
// See src/halow.rs.
pub mod halow;

/// The canonical named-data-over-802.11 EtherType — the LLC/SNAP protocol id every backend uses so a
/// payload injected on one radio de-frames identically on any other. (Matches `FrameFormat::default()`.)
pub const NDN_ETHERTYPE: u16 = 0x8624;

/// Diagnostic: total RX units the pump has PULLED off USB (incremented per subframe in
/// `parse_transfer`, BEFORE the CRC/name filter) — the pump's raw throughput, directly comparable to a
/// kernel monitor iface's `rx_packets`. Read via [`rx_raw_frames`] to isolate pump speed from parse
/// acceptance (the kernel counts bad-FCS frames; our parse drops them).
///
/// ⚠ **NOT every backend increments this**, despite the name. Implemented by **RTL8812AU** and
/// **RTL8733BU** only. `LibUsbRtl88xxBackend` (the a81a) and `Ath9kHtcBackend` are pumped but never
/// touch it, so `rx_raw_frames()` is structurally **0** for them — which reads exactly like a dead
/// receiver and cost a full debugging detour on 2026-08-24 before the 8733b was wired in. On those
/// two backends, judge RX by decoded-frame counts, never by this. A backend added to the pump must
/// increment this in its `parse_transfer` or inherit the same trap.
pub static RX_RAW_FRAMES: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
/// Snapshot of [`RX_RAW_FRAMES`].
pub fn rx_raw_frames() -> u64 {
    RX_RAW_FRAMES.load(std::sync::atomic::Ordering::Relaxed)
}

/// **The standardized way to open a named-data radio.** Dispatches by USB product id to the right
/// chip-specific backend, runs *that* chip's own power-on / monitor / calibration sequence beneath, sets
/// the **one canonical on-air format** (`RawNdn { ethertype: 0x8624 }`) so any two radios opened this way
/// interoperate on air by construction, brings up monitor on `channel`, and starts the RX pump. Returns
/// an [`ndn_radio_hal::FrameIo`] — the caller holds one uniform handle and never touches chip specifics
/// (the leak that made "both implement FrameIo" not mean "they interoperate"). Broadcast rate is legacy
/// 6 Mbps by default (universally decodable); override per-driver with `NDN_RADIO_TX_RATE`.
///
/// Chip family from the PID: the AR9271 set = **ath9k_htc** (via [`open_ath9k`]); `0xf72b`/`0xb733` =
/// **RTL8731BU/8733BU** (halmac_87xx, 1×1); `0xa81a`/`0xa811`/`0x8814` = **RTL8822E** (chip 0x17, the
/// 88xx backend); everything else in the 8812au PID set (`0x8812`/`0x881a`/…, chip 0x04) = **RTL8812AU**.
/// (The 8812au and 8733bu backends open the first matching device; for several identical ones on a host
/// the 8812au/88xx arms honour `NDN_USB_ADDR`/`NDN_USB_INDEX`, the 8733bu arm does not yet.)
pub fn open_named_radio(pid: u16, channel: u8) -> Result<OpenRadio, FaceError> {
    use std::sync::Arc;
    let fmt = FrameFormat::RawNdn {
        ethertype: NDN_ETHERTYPE,
    };
    // Which dongle to claim when several identical ones share the host — `NDN_USB_ADDR="<bus>-<port>"`
    // (stable) or `NDN_USB_INDEX=<n>` (enumeration order). Both branches honour it, so a node with two
    // `0bda:a81a` can pin the spare and leave the kernel mesh on the other (see the multi-radio note).
    // AR9271 (ath9k_htc) — the one Wi-Fi part whose firmware is ours (M3 FrameIo). Its firmware is
    // NOT embedded (it lives at `~/ath9k-fw/...` on the node), so `open_ath9k` is gated behind the
    // `NDN_ATH9K_FW` env var pointing at `htc_9271-1.4.0.fw`. Keyed on the AR9271 PID set.
    if AR9271_IDS.iter().any(|&(_, p)| p == pid) {
        return open_ath9k(channel);
    }
    // RTL8731BU/8733BU (halmac_87xx, 1×1 802.11n) — the ground-up port. Its bring-up is the one that does
    // NOT collapse into "monitor mode and you're done": `bring_up_monitor` gets RX + inject-to-MAC, but
    // *radiating* additionally needs `enable_tx`'s full cal (IQK → TXGAPK → DPK, then the datapath TXAGC
    // block the cal zeroes) plus a background power-tracking loop that trims the OFDM swing off the die
    // thermal so output doesn't fade as the PA heats. `bring_up_tx_tracked` is that whole path, and its
    // `PowerTracker` guard is leaked deliberately so tracking outlives this function — the same lifetime
    // discipline `start_pump` uses for the RX pump.
    //
    // ⚠ RETRACTED: this used to warn that "only ~62% of cold bring-ups radiate" and treat that as
    // per-boot analog variance in the silicon. It is not a property of this chip. MEASURED 2026-08-24
    // on a healthy bus, scored against a real receiver (a81a, not an airtime proxy): **20/20 sequential
    // bring-ups radiated**, 98.9% delivery (77080/77964 frames) at -71.4 dBm, zero USB re-enumerations,
    // and no downward trend across the run. The old figure was produced while a failing AX88179
    // USB-Ethernet NIC on the same host was resetting the whole USB tree, and while the harness issued
    // a `usbreset` before every boot. Remove both and bring-up is reliable.
    // `bring_up_tx_until` / `scripts/supervise_tx.sh` are kept as insurance, not as a required
    // workaround. See [[rtl8733b-port]] and [[lab-node-inventory]].
    if RTL8733B_PIDS.contains(&pid) {
        // No `DeviceSelect` arm: `Rtl8733buBackend::open` claims the first match and has no
        // `open_select` sibling. Fine while a host carries one f72b; a second would need it added.
        let d = Arc::new(Rtl8733buBackend::open()?.with_format(fmt));
        // `NDN_8733B_RX_ONLY=1` stops at monitor RX and skips the cal — a witness/receiver node
        // doesn't need the TX path, and the cal is both the slow part and the variable part.
        if std::env::var_os("NDN_8733B_RX_ONLY").is_some() {
            d.bring_up_monitor(channel)?;
        } else {
            std::mem::forget(d.bring_up_tx_tracked(channel)?);
        }
        // Same `NDN_TX_PWR` contract as the other Realtek arms — here it is the per-rate TXAGC index.
        if let Some(p) = std::env::var("NDN_TX_PWR")
            .ok()
            .and_then(|s| s.parse::<u32>().ok())
        {
            let _ = d.set_tx_power(p);
        }
        apply_bw_override(d.as_ref(), channel);
        start_pump(&d);
        return Ok(OpenRadio {
            io: d.clone(),
            knobs: Some(d.clone()),
            time: Some(d.clone()),
            profile: Some(d),
        });
    }
    let sel = crate::DeviceSelect::from_env();
    // MT7610U (mt76x0u, 1×1 dual-band 11ac). Unlike the MT7612U beside it, this port programs the
    // RF itself — no captured channel replay — so `set_channel` reaches any channel the frequency
    // plan covers, and the caller's `channel` is honoured rather than snapped to a captured one.
    if MT7610U_PIDS.contains(&pid) {
        let d = Arc::new(Mt7610uBackend::open_selected(sel.clone())?.with_format(fmt));
        d.bring_up()?;
        d.setup_monitor_rx()?;
        // 2.4 GHz below 15, else 5 GHz; both are in this part's plan. A tune failure is fatal
        // here on purpose: an untuned monitor receives nothing, and returning a working-looking
        // handle that hears silence is the failure this repo keeps paying for.
        ndn_radio_hal::RadioKnobs::set_channel(
            d.as_ref(),
            channel,
            ndn_radio_hal::Bandwidth::Bw20,
        )?;
        apply_bw_override(d.as_ref(), channel);
        // ★★ Pipelined TX — the difference between this radio's benchmark and its real traffic.
        //
        // MEASURED 2026-08-31: `inject`'s synchronous `write_bulk` costs `≈295 + 0.031·B µs` per
        // PPDU — a width-INDEPENDENT constant plus USB bus time — capping the part near
        // 3000 PPDU/s and pinning every channel width to the same period. Frames confirmed on air
        // by a witness reading MCS 9 / 80 MHz on 100% of them; the pumped period tracks airtime
        // per width (294/194/142 µs at 20/40/80), which no host-side artifact could do.
        //
        // Worth in context (posture PINNED, 3 reps): ~+24% at 1400 B under `Shared`, within noise
        // at 11400 B under `Shared` (there the medium binds, not USB), and the full lever under an
        // aggressive posture — peak ~250 Mbit/s at `Owned` + 11400 B + Bw80 + VHT MCS9. Kept on by
        // default because it costs nothing when the medium is the limit and is worth 2x when USB
        // is.
        //
        // This is spawned HERE, in the factory, and not only in the flood example, because that
        // asymmetry is this repo's characteristic defect: a lever that is measured, documented and
        // reaches no actuator. Before this line the benchmark had the fix and production did not.
        // `NDN_TX_PUMP=0` restores the synchronous path for an A/B.
        //
        // ⚠ Frames may be reordered across pump threads. That is acceptable for connectionless
        // NDN broadcast (and the MT7612U's pump already made the same trade), but it is the reason
        // the knob exists rather than being unconditional.
        let tx_depth = std::env::var("NDN_TX_PUMP")
            .ok()
            .and_then(|s| s.parse::<usize>().ok())
            .unwrap_or(8);
        if tx_depth > 0 {
            std::mem::forget(d.spawn_tx_pump(tx_depth));
        }
        start_pump(&d);
        return Ok(OpenRadio {
            io: d.clone(),
            knobs: Some(d.clone()),
            time: Some(d.clone()),
            profile: Some(d),
        });
    }
    // MT7921AU (connac2, 2x2 802.11ax). The only radio this opener returns that can source
    // common view from a per-frame hardware RX stamp *and* actuate the HAL's HE levers.
    if MT7921U_PIDS.contains(&pid) {
        let d = Arc::new(Mt7921uBackend::open_selected(sel.clone())?.with_format(fmt));
        d.bring_up()?;
        // ★ ORDER IS LOAD-BEARING, and this arm had it backwards (fixed 2026-09-01).
        // `setup_monitor_rx` refuses outright while the channel is still 0 — "the sniffer carries
        // its own copy of the channel and has nothing to be told" (mt7921/mod.rs:1245-1250) — so
        // this arm returned an error for EVERY caller and `open_named_radio` was simply broken for
        // the MT7921AU. Found by trying to use the factory on the part rather than by reading it:
        // the other five arms tune first, and this one drifted. That is the cost of five
        // hand-written bring-up sequences with no shared checklist.
        ndn_radio_hal::RadioKnobs::set_channel(
            d.as_ref(),
            channel,
            ndn_radio_hal::Bandwidth::Bw20,
        )?;
        d.setup_monitor_rx()?;
        apply_bw_override(d.as_ref(), channel);
        start_pump(&d);
        return Ok(OpenRadio {
            io: d.clone(),
            knobs: Some(d.clone()),
            time: Some(d.clone()),
            profile: Some(d),
        });
    }
    let radio: Arc<dyn FrameIo> = if matches!(pid, 0xa81a | 0xa811 | 0x8814) {
        // RTL8822E: `open_monitor_pid_select` claims the selected device + BB/RF-inits + monitors +
        // channel in one call, and its default format is already the canonical RawNdn(0x8624).
        let d = Arc::new(LibUsbRtl88xxBackend::open_monitor_pid_select(
            pid, &sel, channel,
        )?);
        // `NDN_TX_PWR=<idx>` lowers this radio's TX power (e.g. to dial an RX peer out of front-end
        // overload for a clean-RSSI measurement); the 88xx set_tx_power is a per-rate TXAGC index.
        if let Some(p) = std::env::var("NDN_TX_PWR")
            .ok()
            .and_then(|s| s.parse::<u32>().ok())
        {
            let _ = d.set_tx_power(p);
        }
        apply_bw_override(d.as_ref(), channel);
        start_pump(&d); // async (NDN_ASYNC_PUMP) or sync pump, lives for the process
        return Ok(OpenRadio {
            io: d.clone(),
            knobs: Some(d.clone()),
            time: Some(d.clone()),
            profile: Some(d),
        });
    } else {
        // Dispatch by PID, don't silently fall through: an unknown/unsupported pid (e.g. an
        // MT7612U's 0x7612) must NOT open the first 8812au on the bus. Only pids in the 8812au set
        // reach this branch; anything else is a caller error, named as such. (mt7612 has no arm here
        // on purpose — it's ch6-only and needs separate design; #110.)
        if !RTL8812AU_PIDS.contains(&pid) {
            return Err(FaceError::Io(std::io::Error::other(format!(
                "open_named_radio: pid 0x{pid:04x} is not a dispatchable radio \
                 (supported: 8822E 0xa81a/0xa811/0x8814, 8812AU {RTL8812AU_PIDS:#06x?})"
            ))));
        }
        // RTL8812AU: force the canonical format (its own default is Raw80211 for the NAN path), then
        // bring up monitor (MAC/BB/RF + IQK/LCK) on the channel. `sel` (NDN_USB_ADDR / NDN_USB_INDEX)
        // selects which adapter when several identical 8812au dongles share the host.
        let d = Arc::new(Rtl8812auBackend::open_select(&sel)?.with_format(fmt));
        d.bring_up_monitor(channel)?;
        // `NDN_NO_PUMP=1` skips the RX pump — a pure TX-blast node needs no RX, and the pump's bulk-IN
        // threads otherwise contend with inject for USB bandwidth on a busy channel (measured: an 8812au
        // TX collapses to ~250 f/s under heavy RX while the pump drains thousands of frames/s).
        if std::env::var_os("NDN_NO_PUMP").is_some() {
            if std::env::var_os("NDN_CCA_OFF").is_some() {
                let _ = d.set_cca_ignore(true);
            }
            // Same full handle on the no-pump path — a TX-only node still has knobs, a clock and a
            // profile, and the earlier code silently returned a bare FrameIo here too.
            return Ok(OpenRadio {
                io: d.clone(),
                knobs: Some(d.clone()),
                time: Some(d.clone()),
                profile: Some(d),
            });
        }
        // `bring_up_monitor` sets TXAGC to full 0x3f; on a USB-power-limited host a full-power 2-chain
        // TX can brown the PA out so the FIFO never drains. `NDN_TX_PWR=<0..63>` overrides the index.
        if let Some(p) = std::env::var("NDN_TX_PWR")
            .ok()
            .and_then(|s| s.parse::<u8>().ok())
        {
            let _ = d.set_tx_power(p.min(63));
        }
        // `NDN_CCA_OFF=1` forces full carrier-sense off (EDCCA + OFDM packet CCA) so this radio blasts
        // regardless of a busy medium — the doctrine's monitor-mode-without-CSMA sender for the token
        // test, where the slot (not CSMA) is the only collision-avoidance.
        if std::env::var_os("NDN_CCA_OFF").is_some() {
            let _ = d.set_cca_ignore(true);
        }
        start_pump(&d);
        return Ok(OpenRadio {
            io: d.clone(),
            knobs: Some(d.clone()),
            time: Some(d.clone()),
            profile: Some(d),
        });
    };
}

/// 2.4 GHz Wi-Fi channel number → centre frequency (MHz). Ch14 is the 2484 special case; the rest
/// are `2407 + 5·ch` (ch1 = 2412, ch6 = 2437, ch11 = 2462).
fn ath9k_channel_to_mhz(ch: u8) -> u16 {
    if ch == 14 {
        2484
    } else {
        2407 + 5 * (ch as u16)
    }
}

/// Open the AR9271 as a full [`OpenRadio`] (M3): download firmware, HTC/WMI handshake, the faithful
/// `ath9k_hw_reset` PHY/MAC bring-up on `channel`, connect the data services, start receive, and
/// start the RX pump. Returns the backend cloned into `io` / `time` / `profile`.
///
/// **Firmware is not embedded.** The AR9271 image lives on the node at `~/ath9k-fw/...`, so this is
/// gated behind `NDN_ATH9K_FW=<path to htc_9271-1.4.0.fw>`. Without it, this errors with that
/// instruction rather than half-wiring the dispatch.
///
/// `knobs = Some` (M3): the AR9271 impls `RadioKnobs`, so cognition binds it as an actuator. Only
/// `set_channel` is wired (validates a same-channel apply; a live retune is still `hw_reset(&mut self)`
/// — re-open to change channel); power/EDCCA/occupancy keep the trait defaults pending the `&self` WMI
/// path. TX (`FrameIo::inject`) is on-air proven (needs `WMI_TARGET_IC_UPDATE` + queue-1 TXOK, both in
/// `wmi_start`); RX + the RX-stamp common-view clock are the proven halves.
pub fn open_ath9k(channel: u8) -> Result<OpenRadio, FaceError> {
    use std::sync::Arc;
    let fw_path = std::env::var("NDN_ATH9K_FW").map_err(|_| {
        FaceError::Io(std::io::Error::other(
            "open_ath9k: set NDN_ATH9K_FW=<path to htc_9271-1.4.0.fw> — the AR9271 firmware is not \
             embedded (it lives at ~/ath9k-fw/target_firmware/build/k2/htc_9271.fw on the node)",
        ))
    })?;
    let fw = std::fs::read(&fw_path).map_err(|e| {
        FaceError::Io(std::io::Error::other(format!(
            "open_ath9k: cannot read firmware {fw_path}: {e}"
        )))
    })?;
    let chan_mhz = ath9k_channel_to_mhz(channel);

    let mut dev = Ath9kHtcBackend::open()?;
    dev.download_firmware(&fw)?;
    dev.htc_init()?;
    // ★ TX-POWER FIX: read the EEPROM `txGainType` BEFORE hw_reset so `apply_initvals` streams the right
    // gain table. A high-power module (txGainType==1) on the NORMAL table radiates ~50 dB low; the HIGH
    // table + the full board/OLPC cal (applied after wmi_start) = a normal ~+12 dBm link (MEASURED: max
    // −18 dBm at 1 ft, 4500× the frames). `NDN_ATH9K_HIGHPWR` forces high; `NDN_ATH9K_NORMPWR` forces
    // normal (skips the fix). The reg path is up after htc_init (hw_reset itself uses it).
    let high_power = std::env::var_os("NDN_ATH9K_HIGHPWR").is_some()
        || (std::env::var_os("NDN_ATH9K_NORMPWR").is_none() && dev.eeprom_tx_gain_type() == 1);
    dev.set_high_power(high_power);
    // Faithful ath9k_hw_reset (reset + initvals + cal) on the requested channel, then the post-reset
    // RX-start steps, matching `ath9k_htc_start`'s order. `NDN_ATH9K_HT40=1` brings the PHY up at
    // 40 MHz (HT40+) — EXPERIMENTAL, cal convergence unverified on this HT20-class part.
    if std::env::var_os("NDN_ATH9K_HT40").is_some() {
        dev.hw_reset_ht40(chan_mhz)?;
    } else {
        dev.hw_reset(chan_mhz)?;
    }
    dev.connect_data_services()?;
    // Disable the NDR Tier-0 name filter so broadcast/ambient frames aren't dropped in firmware
    // before the USB handoff (the proven RX path in `examples/ath9k_hw_reset.rs` does this). Best
    // effort: on a build where the symbol has moved the write is harmless (the filter defaults off).
    let _ = dev.write_target_u32s(0x0050_cf44, &[0]);
    // Order is load-bearing (proven in `examples/ath9k_hw_reset.rs`): the target's `WMI_START_RECV`
    // (inside `wmi_start`) programs `AR_RXDP` — the RX descriptor ring — so the host RX-DMA enable
    // (`AR_CR_RXE` in `start_receive`) must come AFTER it, or it latches a stale/zero pointer and the
    // ring never advances (seen=0). `wmi_start` also sends `WMI_TARGET_IC_UPDATE` + arms queue-1 TXOK,
    // both required for the injected-TX path to actually radiate and sustain.
    dev.wmi_start()?;
    dev.start_receive()?;
    // Record the channel the PHY came up on so `RadioKnobs::set_channel` can validate cognition's
    // fixed-channel applies (a live retune is `hw_reset(&mut self)`, not yet on the `&self` path).
    dev.note_channel(channel);
    // `NDN_ATH9K_SETPOWER=1`: apply the OLPC power cal (`set_txpower_4k` — PDADC target→gain map +
    // per-rate target power from the EEPROM). hw_reset skips the EEPROM cal, leaving the PA on the
    // initval-default gain; this programs the real target. Opt-in (still proving its on-air effect via
    // the two-radio link RSSI); a bad EEPROM read is non-fatal (leaves the default).
    // ★ Apply the full board + OLPC power cal — the AR9271 TX-power fix. `set_board_values` (antCtrl RF
    // switch + XPA external-PA enable + ob/db bias) and `set_txpower_4k` (PDADC target→gain map +
    // per-rate power) compose with the HIGH gain table to give a normal ~+12 dBm link. Default-ON for a
    // high-power module (where it's the fix); `NDN_ATH9K_NORMPWR` / `NDN_ATH9K_NO_CAL` skip it.
    if (high_power || std::env::var_os("NDN_ATH9K_SETBOARD").is_some())
        && std::env::var_os("NDN_ATH9K_NO_CAL").is_none()
    {
        match dev.set_board_values() {
            Ok(bv) => eprintln!(
                "open_ath9k: board cal applied (txGainType={} ob={:?})",
                bv.tx_gain_type, bv.ob
            ),
            Err(e) => eprintln!("open_ath9k: board cal skipped: {e}"),
        }
        match dev.set_txpower_4k(chan_mhz) {
            Ok(peak) => {
                eprintln!("open_ath9k: power cal applied (peak target {peak} dBm)");
                // ★ Remember it. A later HT20<->HT40 change re-streams the gain tables and wipes
                // this cal; `reapply_power_state` needs to know whether to put it back, or whether
                // this dongle deliberately came up on the initval defaults. The peak itself is the
                // only per-chip absolute anchor the EEPROM gives us and used to be printed and
                // discarded.
                dev.note_cal_applied(peak);
            }
            Err(e) => eprintln!("open_ath9k: power cal skipped: {e}"),
        }
    }

    let dev = Arc::new(dev);
    // RX delivery: default to the on-demand path (`FrameIo::recv_frame` does a single blocking
    // bulk-IN read when no pump is marked) — proven to read 802.11 on this HTC pipe (the M2 oracle:
    // `ndr_stats.seen` climbing, dozens of frames/s). `NDN_ATH9K_PUMP=1` opts into the concurrent
    // submit-ahead pump for higher throughput; it uses the same `parse_transfer` and is the standard
    // Realtek path, but the 8-reader HTC bulk-IN pattern isn't yet load-tested here, so it stays
    // opt-in. Either path surfaces only NDN frames (`parse_dot11` filters to ethertype 0x8624), so a
    // channel with only ambient Wi-Fi yields no `recv_frame` output by design — that is correct, not
    // a fault; RX-of-NDN needs an on-channel NDN sender to observe.
    if std::env::var_os("NDN_ATH9K_PUMP").is_some() {
        start_pump(&dev); // async (NDN_ASYNC_PUMP) or sync pump, lives for the process
    }
    Ok(OpenRadio {
        io: dev.clone(),
        // `knobs` is now populated (M3): the AR9271 impls `RadioKnobs` (`set_channel` wired; power /
        // EDCCA / occupancy keep the trait defaults pending the `&self` WMI-register path). This is
        // what lets `RadioControl::libusb_actuator` bind it and cognition drive it like the 8812au.
        knobs: Some(dev.clone()),
        time: Some(dev.clone()),
        profile: Some(dev),
    })
}

/// `NDN_RADIO_BW` — bring a radio up at a non-default channel width: `5` / `10` (narrowband),
/// `20` (default), `40`. Applied through `RadioKnobs::set_channel` after the chip's own bring-up,
/// which is the ordering the narrowband path requires: on the 8733b the 5/10 MHz BB registers must
/// be written AFTER the RF registers or, in the vendor's words, the MAC rate is right but nothing
/// comes out of the RF.
///
/// Narrowband trades rate for link budget — a quarter-clocked 5 MHz channel puts the same energy in
/// a quarter of the bandwidth, so the noise floor drops ~6 dB. Both the RTL8733BU and the
/// RTL8812EU/8822E implement it; the 8812au path does not, and an unsupported width surfaces as the
/// backend's own error rather than being silently ignored.
fn apply_bw_override(knobs: &dyn RadioKnobs, channel: u8) {
    use ndn_radio_hal::Bandwidth;
    let Some(v) = std::env::var("NDN_RADIO_BW").ok() else {
        return;
    };
    let bw = match v.trim() {
        "5" => Bandwidth::Nb5,
        "10" => Bandwidth::Nb10,
        "20" => Bandwidth::Bw20,
        "40" => Bandwidth::Bw40,
        // ★ 80 was missing entirely, so no caller could ask for it even on a part that supports
        // it. MEASURED on the MT7610U 2026-08-31: Bw80 is real on air (witness radiotap) and worth
        // +84% at 7000 B over Bw20. A radio refusing a width it can actuate is the same
        // declaration/actuator gap as declaring a width it cannot.
        "80" => Bandwidth::Bw80,
        other => {
            tracing::warn!("NDN_RADIO_BW={other}: expected 5|10|20|40|80, ignoring");
            return;
        }
    };
    // `eprintln!`, NOT `tracing`: an operator running a bring-up binary that never installs a
    // subscriber would see NOTHING — and this message exists precisely to stop a narrowband run
    // from silently measuring 20 MHz twice. (Learned the hard way on 2026-08-24: the first attempt
    // at this experiment ran both arms at 20 MHz because the override was not deployed, and the
    // tracing-based confirmation could not have reported that either way.)
    match knobs.set_channel(channel, bw) {
        Ok(()) => eprintln!("NDN_RADIO_BW: channel {channel} set to {bw:?}"),
        Err(e) => eprintln!("NDN_RADIO_BW={v} NOT APPLIED: {e}"),
    }
}

/// RX-pump reader-thread / transfer-pool count. Default 8; `NDN_RX_PUMP_DEPTH` overrides.
fn pump_depth() -> usize {
    std::env::var("NDN_RX_PUMP_DEPTH")
        .ok()
        .and_then(|s| s.parse().ok())
        .filter(|&n| n > 0)
        .unwrap_or(8)
}

/// Start the RX pump for a backend. `NDN_ASYNC_PUMP=1` uses the libusb async submit-ahead pump (keeps
/// the transfer pool continuously in flight — matches the kernel driver's throughput, ~2× the sync
/// pump on the 8812au); default is the synchronous read_bulk pump. The pump lives for the process.
fn start_pump<B: rx_pump::Pumpable>(backend: &std::sync::Arc<B>) {
    let depth = pump_depth();
    if std::env::var_os("NDN_ASYNC_PUMP").is_some() {
        std::mem::forget(rx_pump::spawn_rx_pump_async(backend, depth));
    } else {
        std::mem::forget(rx_pump::spawn_rx_pump(backend, depth));
    }
}

// The control-plane `RadioKnobs` impls for the driver backends. These live with
// the driver types (the trait is from `ndn-radio-hal`, the types are declared
// here) — the orphan rule requires the impl travel with the local type. The
// data-plane `FrameIo`/`WifiRadio` impls live in each backend module.
mod radio_knobs {
    use ndn_radio_hal::{Bandwidth, RadioKnobs};
    use ndn_transport::FaceError;

    // ── Contention knob plumbing for the Realtek backends ───────────────────────────────────────
    //
    // One trait impl per backend, all three identical because all three already expose the same
    // inherent register accessors. The postures themselves live in [`crate::realtek_contention`].

    macro_rules! rtl_edca_regs {
        ($t:ty) => {
            impl crate::realtek_contention::RtlEdcaRegs for $t {
                fn rd32(&self, addr: u16) -> Result<u32, FaceError> {
                    self.read32(addr)
                }
                fn wr32(&self, addr: u16, val: u32) -> Result<(), FaceError> {
                    self.write32(addr, val)
                }
                fn rd8(&self, addr: u16) -> Result<u8, FaceError> {
                    self.read8(addr)
                }
            }
        };
    }

    rtl_edca_regs!(crate::Rtl8812auBackend);
    rtl_edca_regs!(crate::LibUsbRtl88xxBackend);
    rtl_edca_regs!(crate::Rtl8733buBackend);

    impl RadioKnobs for crate::LibUsbRtl88xxBackend {
        /// ★ **The dBm clear-channel threshold** — the RX half of spatial reuse, and the only
        /// genuinely dBm-denominated RX knob in the fleet.
        ///
        /// The encoding (`(dBm + 110) + 0x80` into `0x84c` bytes 2/3) has existed on this backend
        /// all along and matches the vendor exactly; it was simply off-trait, reachable only from
        /// an example, so cognition could never pair a power back-off with a matching change in
        /// what this node defers to. Backing off power alone shrinks this node's reach without
        /// buying any concurrency — the two must move together.
        fn set_edcca_threshold_dbm(&self, l2h: i8, h2l: i8) -> Result<(), FaceError> {
            crate::LibUsbRtl88xxBackend::set_edcca_threshold(self, l2h, h2l)
        }

        /// RX sensitivity as a posture, via IGI. See
        /// [`LibUsbRtl88xxBackend::set_rx_gain_igi`](crate::LibUsbRtl88xxBackend::set_rx_gain_igi)
        /// — in particular for why the autonomous DIG walk must stand down while this is in force.
        fn set_rx_gain(&self, gain: ndn_radio_hal::RxGain) -> Result<(), FaceError> {
            crate::LibUsbRtl88xxBackend::set_rx_gain_igi(self, gain).map(|_| ())
        }

        /// Contention posture. See [`crate::realtek_contention`] — in particular for why this is
        /// a knob in its own right rather than something `set_edcca_ignore` did on the side.
        fn set_contention(
            &self,
            posture: ndn_radio_hal::ContentionPosture,
        ) -> Result<ndn_radio_hal::ContentionApplied, FaceError> {
            crate::realtek_contention::set_contention(self, posture, &self.ndr_contention)
        }

        fn set_channel(&self, channel: u8, bw: Bandwidth) -> Result<(), FaceError> {
            let cbw = match bw {
                Bandwidth::Bw20 => crate::ChannelBw::Bw20,
                Bandwidth::Bw40 => crate::ChannelBw::Bw40,
                Bandwidth::Bw80 => crate::ChannelBw::Bw80,
                Bandwidth::Nb10 => crate::ChannelBw::Nb10,
                Bandwidth::Nb5 => crate::ChannelBw::Nb5,
            };
            crate::LibUsbRtl88xxBackend::set_channel(self, channel, cbw)
        }
        fn set_tx_power(&self, idx: u32) -> Result<(), FaceError> {
            crate::LibUsbRtl88xxBackend::set_tx_power(self, idx)
        }
        fn set_tx_csd(&self, on: bool) -> Result<(), FaceError> {
            crate::LibUsbRtl88xxBackend::set_tx_csd(self, on)
        }
        fn set_edcca_ignore(&self, on: bool) -> Result<(), FaceError> {
            crate::LibUsbRtl88xxBackend::set_edcca_ignore(self, on)
        }
        fn read_channel_activity(&self) -> Result<Option<u16>, FaceError> {
            crate::LibUsbRtl88xxBackend::read_channel_activity(self).map(Some)
        }
    }

    impl RadioKnobs for crate::Mt7612uBackend {
        /// Two channels, because two RF programs have been captured: 2.4 GHz ch6/20 MHz and
        /// 5 GHz ch36/80 MHz. Anything else needs its own capture (docs/RADIO_SUBSYSTEM.md,
        /// "Adding a channel") — or the programmatic tune, which upstream shows is entirely
        /// reachable (`mt76x2/usb_phy.c:62` is register writes plus documented MCU commands,
        /// and the payloads decode identically out of our own capture blobs).
        ///
        /// ★ The ch36 arm matters beyond one more channel: this backend **declares**
        /// `max_bw = 2` and `max_nss = 2` to the planner, and until now `set_channel` could
        /// not reach the VHT80 program at all — the 2x2/80 MHz path existed only as an
        /// inherent method no uniform caller could name. A capability the knob cannot
        /// actuate is the decided-but-unactuated defect, in the one place a planner believes.
        ///
        /// ⚠ Ordering: the 5 GHz blob is a **delta on ch6 state**, not a standalone program —
        /// `start_high_throughput` calls `set_channel_ch6()` first for exactly this reason, so
        /// this arm replicates it rather than relying on the caller to know.
        fn set_channel(&self, channel: u8, bw: Bandwidth) -> Result<(), FaceError> {
            // ★★ **This clamps rather than errors, and that is a bug fix, not laxity.**
            //
            // `RadioCapability` models channels and width as INDEPENDENT — a `Vec<u8>` of
            // channels plus one `max_bw` scalar — but this backend's two captured op-streams are
            // COUPLED pairs: ch6 exists only at 20 MHz and ch36 only at 80. Declaring
            // `channels: [6, 36]` with `max_bw: 2` therefore advertises four combinations of
            // which two do not exist, and cognition reaches a non-existent one immediately:
            // `pick_channel` takes `min_by_key` over the channel list, which returns the FIRST
            // minimum on a tie, so an unsensed radio picks **ch6**; `tx_params` then sets
            // `bw = cap.max_bw()` = **2**; and the first `apply_knobs` tick calls
            // `set_channel(6, Bw80)`.
            //
            // Returning `Err` there did far more damage than it looks: `apply_knobs` propagates
            // with `?`, so one impossible width also skipped `set_tx_csd`, `set_edcca_ignore`
            // and the TX-power call for that tick — a single bad combination silently disarmed
            // the whole control surface. The contention path failed symmetrically, since
            // narrowing under load does `bw.saturating_sub(1)` and asks ch36 for 40 MHz.
            //
            // So: tune the width this channel actually has, and say so. A radio that is 20 MHz
            // when 80 was requested is a radio; one that refuses to tune is not.
            let want = bw;
            let (r, actual) = match channel {
                6 => (
                    crate::Mt7612uBackend::set_channel_ch6(self),
                    Bandwidth::Bw20,
                ),
                36 => (
                    crate::Mt7612uBackend::set_channel_ch6(self)
                        .and_then(|()| crate::Mt7612uBackend::set_channel_5g80(self)),
                    Bandwidth::Bw80,
                ),
                _ => {
                    return Err(FaceError::Io(std::io::Error::other(format!(
                        "mt7612u: only ch6 (20 MHz) and ch36 (80 MHz) have a captured RF program \
                         (requested ch{channel}/{bw:?}); adding one means capturing it — see \
                         docs/RADIO_SUBSYSTEM.md"
                    ))));
                }
            };
            r?;
            if want != actual {
                tracing::info!(
                    requested = ?want,
                    applied = ?actual,
                    channel,
                    "mt7612u: this channel has exactly one captured width; clamping"
                );
            }
            Ok(())
        }

        /// Channel occupancy as **decode-busy per-mille**, from `MT_CH_BUSY`/`MT_CH_IDLE`.
        ///
        /// ★ A genuinely better sense than the frame-count proxy the 8812au uses
        /// (`REG_RXERR_RPT`): these are the MAC's own busy/idle **microsecond** accumulators,
        /// so the answer is the fraction of airtime the medium was unavailable — which is the
        /// quantity a scheduler competes for — rather than a frame rate standing in for it.
        /// MEASURED read-and-clear on both mt76 families: `(busy + idle) / elapsed = 1.00`
        /// over 100 ms windows.
        ///
        /// ⚠ **Unit substitution, stated rather than hidden.** `ChannelOccupancy::from_activity`
        /// expects frames-per-second; this returns per-mille busy time. Feeding microseconds
        /// into a frames-per-second consumer would be nonsense, so the conversion is done here
        /// and the unit is documented at both ends. ⚠ Read-and-clear also means **two readers
        /// split the count** — do not run this alongside a bound kernel driver and believe it.
        fn read_channel_activity(&self) -> Result<Option<u16>, FaceError> {
            let (ct, _window_us) = crate::Mt7612uBackend::sample_channel_time(self)?;
            Ok(Some(crate::mt76::knobs::busy_permille(&ct)))
        }

        /// `(ok, err)` PPDU counters, from `MT_RX_STAT_0`/`_1`.
        ///
        /// ⚠ **`ok` is structurally 0 and that is the honest answer, not a stub.** The mt76x02
        /// MIB block counts only failures — CRC, PHY, false-CCA and PLCP errors — and exposes
        /// no successful-PPDU counter (`MT_CH_TIME_CFG`'s `MDRDY_CNT_EN` hints at one, but
        /// nothing in the mt76 tree enables it and no register is named as its readout). The
        /// caller already knows how many frames it decoded; what it cannot see from the RX side
        /// is the energy that failed, and that is exactly what `err` reports. Reporting a
        /// fabricated `ok` would make the pair look complete and the ratio meaningless.
        ///
        /// `err` sums CRC and PLCP errors: both are "a real PPDU began and did not survive",
        /// which is the collision / marginal-link signature. False-CCA is deliberately excluded
        /// — it is interference with no PPDU behind it, a different question, and it is
        /// available separately through `read_rx_stat`.
        fn read_ofdm_counters(&self) -> Result<Option<(u16, u16)>, FaceError> {
            let st = crate::mt76::knobs::read_rx_stat(self)?;
            Ok(Some((0, st.crc_err.saturating_add(st.plcp_err))))
        }

        /// Energy-detect CCA on/off (`MT_TXOP_CTRL_CFG` bit 20 + `MT_EXT_CCA_CFG` ED mask).
        ///
        /// ★ MEASURED as-found on this part: `MT_TXOP_CTRL_CFG = 0x04001b3f` — bit 20
        /// (`MT_TXOP_ED_CCA_EN`) is **already clear**, because our replayed init writes
        /// `0x04001b3f` where upstream writes `0x04101b3f` after every channel set. So
        /// ED-CCA has been off on this radio the whole time, `set_edcca_ignore(true)` is
        /// close to a no-op, and the knob's real work is the *other* direction. Any past
        /// hypothesis that energy-detect CCA was deferring our transmissions is refuted by
        /// our own init table.
        ///
        /// The on-air effect is UNVALIDATED. The prior from the 8812au is that arming ED-CCA
        /// on a saturated channel trades collision loss for TX starvation (237 → 26 delivered
        /// frames/s), so treat this as something to A/B, not as a fix.
        fn set_edcca_ignore(&self, on: bool) -> Result<(), FaceError> {
            let change = crate::mt76::knobs::set_edcca_ignore_with(
                self,
                on,
                crate::Mt7612uBackend::edcca_slot(self),
            )?;
            if change.changed {
                tracing::info!(
                    txop = format!(
                        "{:#010x}->{:#010x}",
                        change.txop_ctrl_before, change.txop_ctrl_after
                    ),
                    ext_cca = format!(
                        "{:#010x}->{:#010x}",
                        change.ext_cca_before, change.ext_cca_after
                    ),
                    ed_cca_armed = change.ed_cca_armed(),
                    "mt7612u ED-CCA knob applied"
                );
            }
            Ok(())
        }

        /// Contention window as a posture — the actuator for the slot decision.
        ///
        /// Shared mt76x02 implementation ([`crate::mt76::knobs::set_contention`]).
        ///
        /// ☠ **On this part the posture is carried by the SLOT TIME, and the EDCA window is left
        /// exactly as booted.** [`crate::mt76::knobs::window_floor`] pins it there. Writing a
        /// *zero* window into these registers killed this radio once; writing a perfectly legal
        /// `cw_min` exponent of **2** — the value that is measured good on the MT7610U, the
        /// MT7921AU and all three Realtek parts — killed it twice more, collapsing TX to 19 f/s
        /// and then stopping it entirely, past the reach of `restore_edca_defaults`, our cold
        /// bring-up and the kernel driver's own probe alike. Each cost a physical replug.
        ///
        /// Nothing is given up by that: slot 20 → 9 MEASURED **131.9 → 190.6 Mbit/s (+45 %)** at
        /// VHT80, which is the entire win the window was reaching for and then some.
        fn set_contention(
            &self,
            posture: ndn_radio_hal::ContentionPosture,
        ) -> Result<ndn_radio_hal::ContentionApplied, FaceError> {
            crate::mt76::knobs::set_contention(
                self,
                posture,
                crate::mt76::Family::Mt76x2,
                crate::Mt7612uBackend::edca_slot(self),
            )
        }

        // set_tx_power / set_tx_power_dbm: still the trait defaults. The registers are known
        // (MT_TX_PWR_CFG_0..9 at 0x1314.., MT_TX_ALC_CFG_0..4) but the per-rate packing needs
        // the EEPROM target-power/delta parse to mean anything in dBm, and an index knob whose
        // dB effect nobody has measured is worse than no knob — it invites a planner to spend
        // link budget it does not have. Left absent deliberately, not overlooked.
    }

    impl RadioKnobs for crate::Rtl8812auBackend {
        /// Contention posture. See [`crate::realtek_contention`] — in particular for why this is
        /// a knob in its own right rather than something `set_edcca_ignore` did on the side.
        fn set_contention(
            &self,
            posture: ndn_radio_hal::ContentionPosture,
        ) -> Result<ndn_radio_hal::ContentionApplied, FaceError> {
            crate::realtek_contention::set_contention(self, posture, &self.ndr_contention)
        }

        fn set_channel(&self, channel: u8, bw: Bandwidth) -> Result<(), FaceError> {
            // Monitor bring-up tunes 20 MHz; other bandwidths need their per-channel
            // RF/BB program captured (docs/RADIO_SUBSYSTEM.md "Adding a channel").
            if bw == Bandwidth::Bw20 {
                crate::Rtl8812auBackend::set_channel(self, channel)
            } else {
                Err(FaceError::Io(std::io::Error::other(format!(
                    "rtl8812au: only 20 MHz tuned so far (requested ch{channel}/{bw:?})"
                ))))
            }
        }
        fn set_tx_power(&self, idx: u32) -> Result<(), FaceError> {
            // Per-rate TXAGC index (0.5 dB/step) — the devourer jaguar1 power knob,
            // validated monotone on air (#38). This is the actuator behind the
            // cognition policy's reciprocity `decide_power` backoff.
            crate::Rtl8812auBackend::set_tx_power(self, idx.min(63) as u8)
        }
        /// ★ **Hold or release transmissions at the MAC** — the hardware half of a slot MAC.
        ///
        /// This was previously the HAL default, which returns **`Ok(())`**. So
        /// `FaceScheduler`'s `TxHoldGuard` was told its airtime lease was enforced while nothing
        /// was ever written: frames already queued in the MAC went out inside somebody else's
        /// slot, charged to a name that did not cause them. That default's own doc says it — "the
        /// default must never be 'pretend it worked'".
        ///
        /// See [`Rtl8812auBackend::set_tx_pause`] for the MEASURED hold-not-drop semantics a
        /// caller must design around.
        fn set_tx_hold(&self, hold: bool) -> Result<(), FaceError> {
            let mask = if hold {
                crate::Rtl8812auBackend::TXPAUSE_DATA_QUEUES
            } else {
                0x00
            };
            crate::Rtl8812auBackend::set_tx_pause(self, mask)
        }

        fn set_edcca_ignore(&self, on: bool) -> Result<(), FaceError> {
            // ignore == TX does not defer to carrier sense at all — both the energy-detect EDCCA and
            // the OFDM packet CCA (the latter is what still deferred an 8812au on a busy channel).
            crate::Rtl8812auBackend::set_cca_ignore(self, on)
        }
        /// RX sensitivity as a posture, via the Jaguar1 initial-gain index.
        ///
        /// ★ Unlike every other `set_rx_gain` in this crate, this part's axis is **absolute dBm**:
        /// `floor_dBm = IGI - 110`. See [`Rtl8812auBackend::set_rx_floor_dbm`], which cognition
        /// should prefer — this posture form exists so the generic seam works.
        fn set_rx_gain(&self, gain: ndn_radio_hal::RxGain) -> Result<(), FaceError> {
            use ndn_radio_hal::RxGain;
            // -78 dBm is the value most captured programs settle on (IGI 0x20 = the vendor's
            // dm_dig_min); Reduced raises the floor for spatial reuse, Boosted lowers it.
            let dbm = match gain {
                RxGain::Auto => -78,
                RxGain::Reduced => -66,
                RxGain::Boosted => -78, // already at dm_dig_min; going lower is not available
            };
            crate::Rtl8812auBackend::set_rx_floor_dbm(self, dbm).map(|_| ())
        }

        fn read_channel_activity(&self) -> Result<Option<u16>, FaceError> {
            // REG_RXERR_RPT occupancy counter — frame-free channel-activity sensing.
            crate::Rtl8812auBackend::read_phy_sense(self).map(|s| Some(s.rx_activity))
        }
    }
}

#[cfg(test)]
mod capability_declarations {
    use ndn_radio_hal::{Band, RateCapability};

    /// **A declared capability must be non-degenerate and match the part.** #79's trait matrix
    /// exists to show which backends can describe themselves; a backend that implements
    /// `RadioProfile` but returns an empty or default-shaped capability would fill the matrix while
    /// telling a planner nothing — the decided-but-unactuated defect wearing a completeness badge.
    ///
    /// Written against the free `declared_capability()` rather than the trait method precisely so it
    /// runs with no dongle attached; an assertion only checkable on hardware is one nothing checks.
    #[test]
    fn mt7612_and_8821c_declare_something_true() {
        let mt = crate::Mt7612uBackend::declared_capability();
        assert!(
            mt.bands.contains(&Band::Band2_4GHz) && mt.bands.contains(&Band::Band5GHz),
            "the MT7612U is dual-band, as its own module header says: {:?}",
            mt.bands
        );
        assert!(
            !mt.channels.is_empty(),
            "a channel list nothing can tune is not a capability"
        );
        match mt.rate {
            // 2x2 11ac: the driver's captured tune streams include 5 GHz ch36 VHT80.
            RateCapability::Wifi {
                max_nss, max_bw, ..
            } => {
                assert_eq!(max_nss, 2, "MT7612U is a 2x2 part");
                assert_eq!(max_bw, 2, "and reaches VHT80 on the ch36 path");
            }
            other => panic!("a Wi-Fi part must declare a Wi-Fi rate capability, got {other:?}"),
        }

        let rtl = crate::Rtl8821cuBackend::declared_capability();
        assert!(
            rtl.bands.contains(&Band::Band2_4GHz) && rtl.bands.contains(&Band::Band5GHz),
            "the RTL8821CU is dual-band: {:?}",
            rtl.bands
        );
        match rtl.rate {
            // 1x1 11ac — the distinction from the MT7612U that a planner needs.
            RateCapability::Wifi { max_nss, .. } => {
                assert_eq!(max_nss, 1, "RTL8821CU is a 1x1 part")
            }
            other => panic!("expected a Wi-Fi rate capability, got {other:?}"),
        }

        assert_ne!(
            mt, rtl,
            "two different parts must not declare the same capability — that would mean the \
             declaration carries no information"
        );
    }
}
