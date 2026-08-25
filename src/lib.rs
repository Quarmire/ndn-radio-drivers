//! Userspace USB Wi-Fi monitor-mode driver backends over the `ndn-radio-hal` contract.
//!
//! Split out of `ndn-face-monitor-wifi` so drivers have a dedicated home; each
//! backend implements `FrameIo` + `WifiRadio` against the HAL and does no NDN
//! forwarding.

// Re-export the contract surface the backend modules reference as `crate::…`
// (they were written as modules of ndn-face-monitor-wifi, which re-exported these).
pub use ndn_frame_io::{
    frame, radiotap, BROADCAST, CapturedFrame, DEFAULT_SRC, FaceError, FaceId, FrameFormat,
    FrameIo, InjectFrame, MAX_RELIABLE_MCS, McsDescriptor, McsPolicy, Reach, Reliability,
    TxIntent, mcs_for_rssi, mcs_phy_rate_bps,
};
// #78: the capability traits `OpenRadio` hands out. Re-exported so a caller of `open_named_radio`
// needs exactly one import to use everything the opener returns.
// #78: `OpenRadio` and the capability traits it carries live in the HAL, beside the traits they
// aggregate — a driver crate builds one and a face crate consumes one, so neither should need a
// dependency on the other to name it.
pub use ndn_radio_hal::{OpenRadio, RadioKnobs, RadioProfile, RadioTime};

/// Selecting one dongle among several identical ones (by index or USB bus:port) + a guard against
/// claiming the device that currently carries a live kernel link. Shared by the Realtek backends.
pub mod usb_select;
pub use usb_select::{DeviceSelect, usb_addr};

mod libusb_rtl88xx;
/// Shared Realtek RX-descriptor field decode (RSSI/MCS/timestamp) used by the USB backends.
mod realtek_rx;
/// Shared async-URB RX pump (bulk-IN pipelining) used by the USB backends.
pub mod rx_pump;
pub use libusb_rtl88xx::{
    CHIP_ID_8822E, ChannelBw, FwVersion, LibUsbRtl88xxBackend, REALTEK_VID, REG_SYS_CFG,
    RTL88XX_PIDS, RfPath,
};
// AR9271 (ath9k_htc) — the one Wi-Fi part whose FIRMWARE is ours, so Tier-0 can reject a frame
// before it crosses USB (design §8.2) and TX can be scheduled off the hardware TSF (§8.5).
// L1: USB transport + firmware download + HTC handshake + WMI. Does not yet replace ath9k_htc.
pub mod coverage;
mod ath9k_htc;
// PHY-init data for the M1 bring-up port, transcribed verbatim from mainline ath9k v6.12.33:
// AR9271 initval tables (ar9002_initvals.h), the register offsets/bits the reset+cal path writes
// (reg.h / ar9002_phy.h / mac.h), and the HTC wire structs (htc.h). Consumed by ath9k_htc.rs.
mod ath9k_htc_structs;
mod ath9k_initvals;
mod ath9k_reg;
pub use ath9k_htc::{
    AR9271_FIRMWARE, AR9271_FIRMWARE_TEXT, AR9271_IDS, ATHEROS_VID, Ath9kHtcBackend, BoardValues,
    CalStatus,
    FW_NAME, HTC_RX_STATUS_LEN, HtcService, IEEE80211_MODE_11NG, IniVerify, LegacyRate,
    NDR_MEM_MAX_TUPLES, NdrStats, REG_WRITE_MAX_PAIRS, ResetStatus, RxFrame, WmiCmd,
};
mod rtl8821c;
pub use rtl8821c::{RTL8821CU_PIDS, Rtl8821cuBackend};
mod mt7612;
pub use mt7612::{MT7612U_PIDS, Mt7612uBackend};
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

// BW16 (RTL8720DN) serial-bridged backend — a dual-band 802.11 injector/capturer
// driven over USB-serial (firmware/bw16-ndn-bridge), implementing the same
// FrameIo/WifiRadio/RadioKnobs contract as the USB drivers.
#[cfg(feature = "bw16")]
mod bw16_serial;
#[cfg(feature = "bw16")]
pub use bw16_serial::{BW16_BAUD, Bw16SerialBackend};

// Waveshare USB-TO-LoRa (SX1262) serial-bridged sub-GHz backend: a transparent-mode byte pipe with
// host-supplied framing and AT-programmed radio params, implementing the same FrameIo/RadioTime/
// RadioProfile contract as the USB drivers (see src/lora_serial.rs).
#[cfg(feature = "lora")]
mod lora_serial;
#[cfg(feature = "lora")]
pub use lora_serial::{LORA_BAUD, LoraParams, LoraSerialBackend, MAX_LORA_PAYLOAD};

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
    let fmt = FrameFormat::RawNdn { ethertype: NDN_ETHERTYPE };
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
        if let Some(p) = std::env::var("NDN_TX_PWR").ok().and_then(|s| s.parse::<u32>().ok()) {
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
    let radio: Arc<dyn FrameIo> = if matches!(pid, 0xa81a | 0xa811 | 0x8814) {
        // RTL8822E: `open_monitor_pid_select` claims the selected device + BB/RF-inits + monitors +
        // channel in one call, and its default format is already the canonical RawNdn(0x8624).
        let d = Arc::new(LibUsbRtl88xxBackend::open_monitor_pid_select(pid, &sel, channel)?);
        // `NDN_TX_PWR=<idx>` lowers this radio's TX power (e.g. to dial an RX peer out of front-end
        // overload for a clean-RSSI measurement); the 88xx set_tx_power is a per-rate TXAGC index.
        if let Some(p) = std::env::var("NDN_TX_PWR").ok().and_then(|s| s.parse::<u32>().ok()) {
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
        if let Some(p) = std::env::var("NDN_TX_PWR").ok().and_then(|s| s.parse::<u8>().ok()) {
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
            Ok(bv) => eprintln!("open_ath9k: board cal applied (txGainType={} ob={:?})", bv.tx_gain_type, bv.ob),
            Err(e) => eprintln!("open_ath9k: board cal skipped: {e}"),
        }
        match dev.set_txpower_4k(chan_mhz) {
            Ok(peak) => eprintln!("open_ath9k: power cal applied (peak target {} dBm)", peak / 2),
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
    let Some(v) = std::env::var("NDN_RADIO_BW").ok() else { return };
    let bw = match v.trim() {
        "5" => Bandwidth::Nb5,
        "10" => Bandwidth::Nb10,
        "20" => Bandwidth::Bw20,
        "40" => Bandwidth::Bw40,
        other => {
            tracing::warn!("NDN_RADIO_BW={other}: expected 5|10|20|40, ignoring");
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
    std::env::var("NDN_RX_PUMP_DEPTH").ok().and_then(|s| s.parse().ok()).filter(|&n| n > 0).unwrap_or(8)
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

    impl RadioKnobs for crate::LibUsbRtl88xxBackend {
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
        fn set_channel(&self, channel: u8, bw: Bandwidth) -> Result<(), FaceError> {
            // Only channel 6 / 20 MHz has been captured + replayed so far. Other
            // channels need the per-channel RF program captured the same way
            // (see docs/RADIO_SUBSYSTEM.md "Adding a channel"). This is the
            // "capability added incrementally" boundary made explicit.
            if channel == 6 && bw == Bandwidth::Bw20 {
                crate::Mt7612uBackend::set_channel_ch6(self)
            } else {
                Err(FaceError::Io(std::io::Error::other(format!(
                    "mt7612u: only ch6/20MHz tuned so far (requested ch{channel}/{bw:?})"
                ))))
            }
        }
        // set_tx_power / set_tx_csd / set_edcca_ignore: default no-ops until the
        // mt76x2 power-table / TXOP-CTRL / ED-CCA registers are ported.
    }

    impl RadioKnobs for crate::Rtl8812auBackend {
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
        fn set_edcca_ignore(&self, on: bool) -> Result<(), FaceError> {
            // ignore == TX does not defer to carrier sense at all — both the energy-detect EDCCA and
            // the OFDM packet CCA (the latter is what still deferred an 8812au on a busy channel).
            crate::Rtl8812auBackend::set_cca_ignore(self, on)
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
        assert!(!mt.channels.is_empty(), "a channel list nothing can tune is not a capability");
        match mt.rate {
            // 2x2 11ac: the driver's captured tune streams include 5 GHz ch36 VHT80.
            RateCapability::Wifi { max_nss, max_bw, .. } => {
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
