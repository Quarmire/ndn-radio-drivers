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
    TxIntent, WifiRadio, mcs_for_rssi, mcs_phy_rate_bps, name_group_mac, name_group_uni,
};

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
mod ath9k_htc;
pub use ath9k_htc::{
    AR9271_FIRMWARE, AR9271_FIRMWARE_TEXT, AR9271_IDS, ATHEROS_VID, Ath9kHtcBackend, FW_NAME,
    HtcService, NDR_MEM_MAX_TUPLES, NdrStats, WmiCmd,
};
mod rtl8821c;
pub use rtl8821c::{RTL8821CU_PIDS, Rtl8821cuBackend};
mod mt7612;
pub use mt7612::{MT7612U_PIDS, Mt7612uBackend};
mod rtl8812au;
pub use rtl8812au::{ChipInfo, IqkResult, PhySense, RTL8812AU_PIDS, Rtl8812auBackend};
// RTL8731BU / RTL8733BU (halmac_87xx, 1x1 11ac) — ground-up port, M1 (open +
// reg-I/O + chip-version). REALTEK_VID is already re-exported above.
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
/// Chip family from the PID: `0xa81a`/`0xa811`/`0x8814` = **RTL8822E** (chip 0x17, the 88xx backend);
/// everything else in the 8812au PID set (`0x8812`/`0x881a`/…, chip 0x04) = **RTL8812AU**. (The 8812au
/// backend opens the first matching device; for multiple 8812au on one host it takes the first.)
pub fn open_named_radio(
    pid: u16,
    channel: u8,
) -> Result<std::sync::Arc<dyn FrameIo>, FaceError> {
    use std::sync::Arc;
    let fmt = FrameFormat::RawNdn { ethertype: NDN_ETHERTYPE };
    let radio: Arc<dyn FrameIo> = if matches!(pid, 0xa81a | 0xa811 | 0x8814) {
        // RTL8822E: `open_monitor_pid` claims + BB/RF-inits + monitors + channel in one call, and its
        // default format is already the canonical RawNdn(0x8624).
        let d = Arc::new(LibUsbRtl88xxBackend::open_monitor_pid(pid, channel)?);
        // `NDN_TX_PWR=<idx>` lowers this radio's TX power (e.g. to dial an RX peer out of front-end
        // overload for a clean-RSSI measurement); the 88xx set_tx_power is a per-rate TXAGC index.
        if let Some(p) = std::env::var("NDN_TX_PWR").ok().and_then(|s| s.parse::<u32>().ok()) {
            let _ = d.set_tx_power(p);
        }
        start_pump(&d); // async (NDN_ASYNC_PUMP) or sync pump, lives for the process
        d
    } else {
        // RTL8812AU: force the canonical format (its own default is Raw80211 for the NAN path), then
        // bring up monitor (MAC/BB/RF + IQK/LCK) on the channel. `NDN_USB_INDEX` selects which adapter
        // when several identical 8812au dongles share the host (0 = first enumerated).
        let index = std::env::var("NDN_USB_INDEX").ok().and_then(|s| s.parse().ok()).unwrap_or(0);
        let d = Arc::new(Rtl8812auBackend::open_nth(index)?.with_format(fmt));
        d.bring_up_monitor(channel)?;
        // `NDN_NO_PUMP=1` skips the RX pump — a pure TX-blast node needs no RX, and the pump's bulk-IN
        // threads otherwise contend with inject for USB bandwidth on a busy channel (measured: an 8812au
        // TX collapses to ~250 f/s under heavy RX while the pump drains thousands of frames/s).
        if std::env::var_os("NDN_NO_PUMP").is_some() {
            if std::env::var_os("NDN_CCA_OFF").is_some() {
                let _ = d.set_cca_ignore(true);
            }
            return Ok(d);
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
        d
    };
    Ok(radio)
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
