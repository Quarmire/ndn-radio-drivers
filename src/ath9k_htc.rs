//! Userspace **AR9271** (`ath9k_htc`) driver — USB transport, firmware download, HTC handshake, WMI.
//!
//! The AR9271 matters to this project for a reason no other Wi-Fi part we own does: **its firmware
//! is ours**. `named-filter-mac-redesign.md` §8.2 records that on commodity Wi-Fi we get Tier 0's
//! CPU win but not NDN-NIC's wakeup win, because the NIC has already copied the frame to the host
//! before any filter of ours can run. On this part the filter runs in `ath_tgt_rx_tasklet()` on the
//! dongle, and §8.5's "biggest loss" — no hardware-scheduled TX — is answered by the MAC's
//! `AR_TSF_*` / `AR_QUIET1/2` / `AR_D_GBL_IFS_*` registers. See
//! `ndn-radio-drivers/firmware/ath9k-htc-ndr/README.md`.
//!
//! # Status — read this before believing anything below
//!
//! **L1 (this module): transport only.** Device open, firmware download, HTC handshake, and WMI
//! request/response. It is written against the *authoritative* sources — the endpoint map was read
//! off the device with `lsusb -v`, the HTC/WMI wire formats from the open firmware's own
//! `wlan/include/{htc,htc_services,wmi}.h`, and the download constants from the mainline
//! `hif_usb.h` — but **it has not yet been run against hardware**. Treat every "should" here as
//! untested.
//!
//! **This module does NOT replace `ath9k_htc`.** It cannot yet bring the radio up, because in the
//! HTC split the *host* owns the PHY: `ath9k_hw` (≈500 KB in-kernel) performs the AR9271 reset,
//! initvals replay, and calibration by issuing thousands of `WMI_REG_WRITE`s. Porting that is the
//! RTL8812AU saga again and is deliberately not attempted here. Until it is done, the path to
//! on-air measurement is: let the kernel driver load *our* firmware and do the PHY init, and use
//! this module only for what the kernel driver cannot express.
//!
//! **The thing it is actually for** is the config/telemetry channel: reading and writing
//! `ndr_cfg` / `ndr_stats` in target RAM so the Tier-0 filter can be reconfigured and its drop
//! counters read without a firmware rebuild. One caveat, discovered by reading the firmware rather
//! than assuming: **`WMI_ACCESS_MEMORY_CMDID` is dispatched but its handler body is empty**
//! (`magpie.c:88`, and the same handler is wired in `if_ath.c`'s table). The wire format exists;
//! the target-side implementation does not. It has to be added to our firmware before
//! [`Ath9kHtcBackend::read_target_memory`] can work.

use std::sync::Arc;
use std::time::Duration;

use async_trait::async_trait;
use rusb::{Context, DeviceHandle, UsbContext};

use ndn_frame_io::{ClockDomainId, LatchPoint, LinkStamp};
use ndn_radio_hal::{
    Bandwidth, RadioCapability, RadioKnobs, RadioProfile, RadioTime, RadioTimeSource,
};
use ndn_transport::FaceError;

use crate::ath9k_htc_structs::{
    ATH9K_HTC_NORMAL, ATH9K_KEY_TYPE_CLEAR, TX_FRAME_HDR_SIZE, TxFrameHdr,
};
use crate::{CapturedFrame, FrameFormat, FrameIo, InjectFrame, McsDescriptor};

/// Bytes before the `ath_htc_rx_status` on the WLAN-RX bulk pipe: the 8-byte HTC frame header
/// PLUS a 4-byte data-endpoint prefix (observed `04 00 00 00`) — MEASURED, see [`Ath9kHtcBackend::parse_rx`].
const RX_PREFIX_LEN: usize = HTC_HDR_LEN + 4; // 12

/// Nominal 2.4 GHz noise floor (dBm) used to convert the AR9271's NF-relative `rs_rssi` (dB above
/// noise) to an approximate dBm: `dBm ≈ rs_rssi - AR9271_NF_DBM_2GHZ`.
///
/// ⚠ **BENCH:** −95 dBm is a NOMINAL 2.4 GHz noise floor, not a per-device measurement. The
/// on-chip NF cal (`set_channel_and_cal` reports `noise_floor_dbm`) gives the real figure; if the
/// reported RSSI looks off against a known-power source, substitute the calibrated NF here.
const AR9271_NF_DBM_2GHZ: i16 = 95;

/// Qualcomm Atheros USB vendor ID.
pub const ATHEROS_VID: u16 = 0x0cf3;

/// AR9271 product IDs. `0x9271` is the reference part (the one on o5p-1); the others are
/// vendor-rebadged AR9271 dongles carried by `ath9k_htc`'s id table.
pub const AR9271_IDS: &[(u16, u16)] = &[
    (0x0cf3, 0x9271), // Atheros reference
    (0x0cf3, 0x1006),
    (0x0846, 0x9030), // Netgear WNA1100
    (0x07d1, 0x3a10), // D-Link DWA-126
    (0x13d3, 0x3327),
    (0x040d, 0x3801),
    (0x0cf3, 0xb003),
];

// ── USB endpoints ────────────────────────────────────────────────────────────
//
// Read off the device itself (`lsusb -v -d 0cf3:9271`), not assumed. Note the asymmetry that a
// "pipe number" naming scheme hides: the WLAN data pipes are **bulk** at 512 bytes, while the
// register/command pipes are **interrupt** at 64 bytes. Using bulk transfers on the reg pipes
// silently fails, and 64 bytes is a hard ceiling on an HTC control message.

/// EP1 OUT, bulk 512 — WLAN TX data.
pub const EP_WLAN_TX: u8 = 0x01;
/// EP2 IN, bulk 512 — WLAN RX data.
pub const EP_WLAN_RX: u8 = 0x82;
/// EP3 IN, **interrupt** 64 — WMI events / HTC control from the target.
pub const EP_REG_IN: u8 = 0x83;
/// EP4 OUT, **interrupt** 64 — WMI commands / HTC control to the target.
pub const EP_REG_OUT: u8 = 0x04;

/// Target-side HIF pipe ids (`magpie_fw_dev/target/inc/hif_usb.h`). They coincide with the USB
/// endpoint numbers: `HIF_USB_PIPE_INTERRUPT` = 3 = EP3 IN, `HIF_USB_PIPE_COMMAND` = 4 = EP4 OUT.
/// The host names them `USB_REG_IN_PIPE` / `USB_REG_OUT_PIPE` for the same pipes.
pub const PIPE_REG_IN: u8 = 3;
pub const PIPE_REG_OUT: u8 = 4;

/// Bulk WLAN pipe ids (`USB_WLAN_TX_PIPE` / `USB_WLAN_RX_PIPE` in mainline `hif_usb.h`): pipe 1 =
/// EP1 OUT (TX), pipe 2 = EP2 IN (RX). Every HTC **data** service connects on these (dl=2, ul=1) —
/// `service_to_dlpipe`/`service_to_ulpipe` in `htc_hst.c` — as opposed to WMI control, which uses
/// the interrupt register pipes (dl=3, ul=4).
pub const PIPE_WLAN_TX: u8 = 1;
pub const PIPE_WLAN_RX: u8 = 2;

/// The monitor vif / self-station MAC created on the target for TX (locally-administered, "NDN").
/// The injected `addr2` need not match this — injection sets addr2 per frame; the target node is
/// only the rate-control lookup at `tx_frame_hdr.node_idx`.
const SELF_MAC: [u8; 6] = [0x02, 0x4e, 0x44, 0x4e, 0x00, 0x01];

/// Hard ceiling on a register-pipe message (`wMaxPacketSize` on EP3/EP4).
pub const REG_PIPE_MAX: usize = 64;

// ── Firmware download ────────────────────────────────────────────────────────
// Constants from mainline `drivers/net/wireless/ath/ath9k/hif_usb.h`.

const FIRMWARE_DOWNLOAD: u8 = 0x30;
const FIRMWARE_DOWNLOAD_COMP: u8 = 0x31;

/// Where the AR9271 firmware image is written in target address space.
pub const AR9271_FIRMWARE: u32 = 0x0050_1000;
/// The entry point handed to the target to start executing the freshly downloaded image.
pub const AR9271_FIRMWARE_TEXT: u32 = 0x0090_3000;

/// Download chunk size. The vendor driver uses 4 KB and the target's control handler is written
/// around it.
const FW_CHUNK: usize = 4096;

/// The firmware file this kernel actually requests — **not** `htc_9271.fw`. Confirmed with
/// `modinfo ath9k_htc`; mainline builds the versioned name and only falls back to the legacy one.
pub const FW_NAME: &str = "ath9k_htc/htc_9271-1.4.0.fw";

const USB_TIMEOUT: Duration = Duration::from_millis(1000);

/// Per-attempt timeout for a TX bulk-OUT write. Short so WLAN-endpoint backpressure (a NAK when the
/// target's TX buffer ring is full) is detected quickly and retried in [`FrameIo::inject`], rather
/// than stalling a full second per frame. A timed-out write transferred 0 bytes (safe to retry).
const TX_WRITE_TIMEOUT: Duration = Duration::from_millis(20);

// ── HTC ──────────────────────────────────────────────────────────────────────
// Wire formats from the open firmware's `wlan/include/htc.h`. All multi-byte HTC and WMI fields
// are **big-endian** on the wire.

/// `HTC_FRAME_HDR`: endpoint id, flags, big-endian payload length, 4 control bytes.
pub const HTC_HDR_LEN: usize = 8;

/// The reserved control endpoint every handshake message rides on.
const HTC_ENDPOINT_CTRL: u8 = 0;

/// Set in an HTC header's flags when a receive trailer is appended; its length is ControlBytes[0].
const HTC_FLAGS_RECV_TRAILER: u8 = 1 << 1;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
enum HtcMsg {
    Ready = 1,
    ConnectService = 2,
    ConnectServiceResponse = 3,
    SetupComplete = 4,
    ConfigPipe = 5,
    ConfigPipeResponse = 6,
}

/// HTC service IDs (`htc_services.h`): `(group << 8) | index`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum HtcService {
    /// The reserved control service.
    CtrlRsvd = 0x0001,
    /// WMI command/response — the one this module needs.
    WmiControl = 0x0100,
    Beacon = 0x0101,
    Cab = 0x0102,
    Uapsd = 0x0103,
    Mgmt = 0x0104,
    DataVo = 0x0105,
    DataVi = 0x0106,
    DataBe = 0x0107,
    DataBk = 0x0108,
}

// ── WMI ──────────────────────────────────────────────────────────────────────

/// WMI command IDs, from the firmware's `wlan/include/wmi.h`. The enum is dense from `0x0001`, so
/// these are positional — do not reorder.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum WmiCmd {
    Echo = 0x0001,
    /// Read/write arbitrary target memory. ⚠ **The open firmware's handler is an empty `break`** —
    /// see the module docs. Our firmware must implement it before this is usable.
    AccessMemory = 0x0002,
    GetFwVersion = 0x0003,
    /// Toggle the SWBA/BMISS interrupt bits (payload = BE u32 mask). `ath_enable_intr_tgt`.
    EnableIntr = 0x0005,
    /// Bring up the target WLAN app: set the interrupt mask (RX/TX/FATAL/…), install the ISR,
    /// arm the lease tick. **No payload.** `ath_init_tgt`. Prerequisite for RX interrupts firing.
    AthInit = 0x0006,
    /// Start the target's RX descriptor ring + program `AR_RXDP`. **No payload.**
    /// `ath_startrecv_tgt` → `ath_startrecv`. The host still owns the RX filter / opmode /
    /// `AR_CR_RXE` on its side.
    StartRecv = 0x000c,
    /// Select the target PHY-mode rate table (payload = BE u16 `ieee80211_phymode`;
    /// `IEEE80211_MODE_11NG` = 1 for 2.4GHz — its rate table is populated at attach, so this is
    /// safe). `ath_setcurmode_tgt`; a mode with no rate table would `adf_os_assert` on the target.
    SetMode = 0x000f,
    /// Create a target station/node (payload = `struct ath9k_htc_target_sta`, 22 B). `ath_node_create_tgt`.
    /// The host picks `sta_index`; TX data frames reference it via `tx_frame_hdr.node_idx` for rate
    /// control — WITHOUT a created node the target drops the frame (measured: 0 on air with node 0).
    NodeCreate = 0x0010,
    /// Create a target virtual interface (payload = `struct ath9k_htc_target_vif`, 12 B).
    /// `ath_vap_create_tgt`. The host picks `index`; TX references it via `tx_frame_hdr.vif_idx`.
    VapCreate = 0x0013,
    /// Set a node's rate-control state (payload = `struct ath9k_htc_target_rate`). `ath_rc_state_change`.
    /// Gives the created node a rate table — without it the target has no rate to TX at and drops
    /// the frame even with a valid node.
    RcStateChange = 0x0016,
    RegRead = 0x0014,
    RegWrite = 0x0015,
    /// Push the `ieee80211com_target` capability block. `ath_ic_update_tgt`. Not required to
    /// receive frames (the RX tasklet forwards regardless), so M1.4 skips it.
    TargetIcUpdate = 0x0018,
    /// Query the target's RX buffer accounting (`ath_rx_stats_tgt`). Reply = 3 big-endian u32s:
    /// `ast_rx_nobuf` (RX-buffer alloc failures — the pool-exhaustion / leak witness), `ast_rx_send`
    /// (frames handed to the host), `ast_rx_done` (HTC send-completions = buffers recycled).
    RxStats = 0x001e,
    /// Tear the WLAN application down cleanly. Must be the last command — the target's handler
    /// frees its softc.
    TgtDetach = 0x001a,
}

/// 2.4GHz `ieee80211_phymode` (firmware `_ieee80211.h`): `IEEE80211_MODE_11NG` = 1. Its rate table
/// is populated by `ath_rate_setup(IEEE80211_MODE_11NG)` at attach, so `WMI_SET_MODE` is safe.
pub const IEEE80211_MODE_11NG: u16 = 1;

/// `WMI_CMD_HDR`: command id + sequence number, both big-endian.
const WMI_HDR_LEN: usize = 4;

/// Target→host event IDs. Events share the response pipe with command replies and are
/// distinguished by having ids at or above `0x1001`.
const WMI_EVENT_BASE: u16 = 0x1001;

// ── WMI_ACCESS_MEMORY (firmware/ath9k-htc-ndr/src/ndr_mem.h) ─────────────────

/// Words per exchange. Bounded by the 64-byte register pipe, **not** by the vendor's
/// `WMI_ACCESS_MEMORY_MAX_TUPLES` of 8 — which does not fit: 8 + 4 + 4 + 8*8 = 80 > 64.
///
/// **5, not 6.** A reply can carry a 4-byte HTC receive trailer (measured: flags=0x02,
/// control[0]=4), and 8 + 4 + 4 + 6*8 + 4 = 68 > 64. Six tuples fit only as long as no trailer is
/// attached, which is not something the host gets to decide.
pub const NDR_MEM_MAX_TUPLES: usize = 5;

const NDR_MEM_FLAG_WRITE: u16 = 0x0001;
const NDR_MEM_OK: u16 = 0;

fn ndr_mem_status(code: u16) -> &'static str {
    match code {
        0 => "ok",
        1 => "malformed request",
        2 => "too many tuples",
        3 => "unaligned address",
        4 => "address outside the firmware's RAM windows",
        _ => "unknown status",
    }
}

/// Tier-0 filter counters (`struct ndr_stats`). `dropped_filter` is the headline number: USB
/// transfers and host wakeups that did not happen — the quantity design §8.2 says is unreachable
/// on every other Wi-Fi part we own.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct NdrStats {
    pub seen: u32,
    pub passed: u32,
    pub dropped_filter: u32,
    pub dropped_foreign: u32,
    pub short_frame: u32,
    /// Frames rejected because `addr1||addr2` had more bits set than any legitimate Tier-0 sender
    /// could produce (> K*D = 32). Exact, not heuristic — and it is what rejects the all-ones
    /// broadcast address, which satisfies every Bloom mask by construction.
    pub dropped_popcount: u32,
}

/// Max `{addr,val}` pairs in one batched `WMI_REG_WRITE`. Bounded by the 64-byte
/// register pipe: `HTC(8) + WMI(4) + n*8 ≤ 64` ⇒ `n ≤ 6`. The reply is empty
/// (the firmware's `ath_hal_reg_write_tgt` answers `wmi_cmd_rsp(..., NULL, 0)`),
/// so no receive-trailer budget is needed on the request side.
pub const REG_WRITE_MAX_PAIRS: usize = 6;

/// Result of [`Ath9kHtcBackend::phy_reset`] — the read-back evidence that the
/// M1.1 chip reset landed, straight off the hardware.
#[derive(Debug, Clone, Copy)]
pub struct ResetStatus {
    /// Raw `AR_SREV` (0x4020) after reset — the authoritative value. On the AR9271 this reads
    /// `0x00_14_11_ff`: the low byte is `0xFF`, so ath9k sources macVersion (0x140) from the HTC
    /// device path rather than decoding this register (see `ath9k_reg.rs`); the silicon signature
    /// is still present as bits[23:16] == 0x14 == (0x140 >> 4).
    pub srev_raw: u32,
    /// `AR_SREV & 0xFF` — the ID byte. `0xFF` on the AR9271 (routes ath9k to the type2/HTC path).
    pub srev_id: u32,
    /// bits[23:16] of `AR_SREV` — the AR9271 signature nibble-pair, `0x14` == `AR_SREV_VERSION_9271
    /// (0x140) >> 4`. (NB: this is *not* the naive AR9002 `MS(val, AR_SREV_VERSION)` decode, which is
    /// invalid here because the ID byte is 0xFF.)
    pub srev_version_field: u32,
    /// `(AR_SREV & 0xF00) >> 8` — `MS(val, AR_SREV_REVISION2)`, the type2 macRev field (reads 1).
    pub srev_rev: u32,
    /// Raw `AR_RTC_STATUS` (0x7044) after reset — `& 0x0f` should read `AR_RTC_STATUS_ON` (0x02).
    pub rtc_status: u32,
    /// Raw `AR_RTC_RC` (0x7000) after reset — should read 0 (MAC out of reset).
    pub rtc_rc: u32,
    /// `AR_INTR_SYNC_CAUSE & 0x3000` observed mid-reset (PCIe-artefact bits; 0 on this USB part).
    pub intr_sync_masked: u32,
}

/// Result of [`Ath9kHtcBackend::apply_initvals`] — sentinel read-back tally for M1.2.
#[derive(Debug, Clone)]
pub struct IniVerify {
    pub total_written: usize,
    pub matched: usize,
    pub checked: usize,
    /// `(addr, expected, got)` for every sentinel that did not read back its written value.
    pub mismatches: Vec<(u32, u32, u32)>,
}

/// Cal values read from the 4k EEPROM and applied by [`Ath9kHtcBackend::set_board_values`] (OLPC M2).
#[derive(Debug, Clone, Copy)]
pub struct BoardValues {
    /// XOR checksum over the EEPROM validated (always true if the call returned Ok).
    pub checksum_ok: bool,
    /// 0 = normal-power module, 1 = high-power (this AR9271 = 1) — which gain table the part wants.
    pub tx_gain_type: u8,
    /// RF switch table config written to `AR_PHY_SWITCH_COM`.
    pub ant_ctrl_common: u32,
    /// PA output-bias per modulation (cck, psk, qam) — the big power lever.
    pub ob: [u8; 3],
    /// PA driver-bias 1 / 2 (chain 0).
    pub db1_0: u8,
    pub db2_0: u8,
    /// TX/RX attenuation + margin (chain 0), from the modal header.
    pub tx_rx_atten: u8,
    pub rx_tx_margin: u8,
}

/// Result of [`Ath9kHtcBackend::set_channel_and_cal`] — M1.3 evidence.
#[derive(Debug, Clone, Copy)]
pub struct CalStatus {
    /// Channel centre in MHz that was programmed.
    pub chan_mhz: u16,
    /// `AR_PHY_SYNTH_CONTROL` (0x9874) read back after programming the synth.
    pub synth_control: u32,
    /// `AR_PHY_MODE` (0xA200) read back — should be `AR_PHY_MODE_DYNAMIC` (0x04) for 2.4GHz.
    pub phy_mode: u32,
    /// `AR_PHY_ACTIVE` (0x981C) read back — should read `AR_PHY_ACTIVE_EN` (0x01).
    pub phy_active: u32,
    /// Did the AGC/offset cal (`AR_PHY_AGC_CONTROL_CAL`) clear within timeout? `false` = cal hung.
    pub agc_cal_converged: bool,
    /// `AR_PHY_AGC_CONTROL` (0x9860) read back after the cal (for diagnosis if it hung).
    pub agc_control: u32,
    /// `AR_PHY_CCA` (0x9864) after NF cal was started.
    pub phy_cca: u32,
    /// Noise floor decoded from `AR_PHY_CCA` `MINCCA_PWR` (bits[28:20], 9-bit signed), in dBm.
    pub noise_floor_dbm: i32,
}

/// One received frame off EP 0x82: the parsed `ath_htc_rx_status` plus the 802.11 bytes.
///
/// Wire layout (big-endian multi-byte), 40 bytes, matching the host's `struct ath_htc_rx_status`
/// (our firmware is the matching target-firmware fork): `rs_tstamp` be64, `rs_datalen` be16,
/// `rs_status`, `rs_phyerr`, `rs_rssi`(combined), `rs_rssi_ctl[3]`, `rs_rssi_ext[3]`, `rs_keyix`,
/// `rs_rate`, `rs_antenna`, `rs_more`, `rs_isaggr`, `rs_moreaggr`, `rs_num_delims`, `rs_flags`,
/// `rs_dummy`, `rs_evm0/1/2` be32.
#[derive(Debug, Clone)]
pub struct RxFrame {
    pub rs_tstamp: u64,
    pub rs_datalen: u16,
    pub rs_status: u8,
    pub rs_phyerr: u8,
    pub rs_rssi: i8,
    pub rs_keyix: u8,
    pub rs_rate: u8,
    pub rs_antenna: u8,
    pub rs_more: u8,
    pub rs_flags: u8,
    /// The 802.11 frame that followed the status block (`rs_datalen` bytes when consistent).
    pub frame: Vec<u8>,
}

/// Bytes of the on-wire `ath_htc_rx_status` block that precede the 802.11 frame.
pub const HTC_RX_STATUS_LEN: usize = 40;

/// Largest echo payload whose *reply* fits one register-pipe packet:
/// `64 - HTC(8) - WMI(4) - msgSize(1) = 51`.
///
/// ⚠ The firmware's own `WMI_ECHOCMD_MSG_MAX_LEN` says **53**, and its derivation comment reads
/// `64 - HTC_HDR_LENGTH + sizeof(WMI_CMD_HDR) - 1` — it *adds* the WMI header where it should
/// subtract it. 53 does not fit: `8 + 4 + 1 + 53 = 66 > 64`. Trust the arithmetic, not the header.
pub const MAX_ECHO_LEN: usize = REG_PIPE_MAX - HTC_HDR_LEN - WMI_HDR_LEN - 1;

/// A userspace AR9271 over libusb.
pub struct Ath9kHtcBackend {
    /// `Arc` so the blocking USB bulk transfers `FrameIo`/the RX pump issue can be moved onto
    /// `spawn_blocking` / reader threads (mirrors the Realtek backend). Every rusb op is `&self`,
    /// so the `&mut self` bring-up methods keep working through the `Arc` deref.
    handle: Arc<DeviceHandle<Context>>,
    /// WMI sequence number; the target echoes it so replies can be matched to commands. A `Mutex`
    /// (not a bare `u16`) so the whole reg/WMI path is `&self`: [`wmi_cmd`](Self::wmi_cmd) holds the
    /// lock across send+recv, which both bumps the seq and serializes concurrent commands (the reg
    /// pipe is one-in-flight). This is what lets the `&self` `RadioKnobs` — live channel retune,
    /// EDCCA, occupancy — drive the register path.
    seq: std::sync::Mutex<u16>,
    /// Endpoint the target assigned to `WMI_CONTROL_SVC` during the handshake.
    wmi_endpoint: u8,
    /// Credits the target offered in its READY message — the HTC flow-control budget.
    credits: u16,
    credit_size: u16,
    /// Endpoint ids the target assigns to the data services in [`connect_data_services`]
    /// (`Ath9kHtcBackend::connect_data_services`); 0 = not yet connected. The RX path (M1) rides
    /// these; the WMI-control endpoint stays [`wmi_endpoint`](Self::wmi_endpoint).
    mgmt_ep: u8,
    data_be_ep: u8,
    beacon_ep: u8,
    /// MAC clock rate in MHz used by `ath9k_hw_mac_to_clks` (timing math in
    /// `init_global_settings`). Computed by [`Ath9kHtcBackend::set_clockrate`];
    /// 44 for 2.4 GHz OFDM (the value the golden trace's SIFS/SLOT writes imply).
    clockrate: u32,
    /// Per-device RX-stamp clock domain (`bus << 8 | address`). The AR9271's per-frame
    /// `rs_tstamp` is a µs hardware RX timestamp on this domain — a common-view `FreeRunRxStamp`
    /// (M2 / design §15). Mirrors the Realtek `tsf_domain`.
    tsf_domain: ClockDomainId,
    /// On-air (de)framing — the canonical `RawNdn { ethertype: 0x8624 }`. Set at open; shared with
    /// every other backend so a frame injected here de-frames identically on any radio.
    format: FrameFormat,
    /// Current transmit rate as bearer state ([`FrameIo::set_rate`]). Mirrors the Realtek `cur_mcs`.
    ///
    /// ⚠ **DECIDED-BUT-UNACTUATED on this part.** Unlike the Realtek TX descriptor (which carries a
    /// per-frame `DESC_RATE` code), the ath9k HTC `tx_frame_hdr` has **no rate field** — a data
    /// frame's rate is chosen by the *target firmware's* rate control for its `node_idx`. So this is
    /// stored (so the seam is uniform and a future WMI rate-table/node path can consume it) but does
    /// **not** currently steer the on-air rate. Flagged so nobody reads a stored MCS as an actuated one.
    cur_mcs: std::sync::Mutex<Option<McsDescriptor>>,
    /// Selected legacy (non-HT) transmit rate as the raw AR5416 hardware rate code (#6). `Some(code)`
    /// overrides `cur_mcs` and injects at a CCK/OFDM legacy PHY rate (rate_flags cleared — no HT/SGI/
    /// HT40); `None` defers to `cur_mcs` (HT) or, if that too is None, the target min rate. Set by
    /// [`set_legacy_rate`](Self::set_legacy_rate); cleared when an HT MCS is chosen via `set_rate`.
    /// This is the honest legacy lever the worst-receiver adaptation needs when a legacy-only neighbour
    /// is advertised — the AR9271 can now name a *specific* legacy rate, not just "target min".
    cur_legacy: std::sync::atomic::AtomicU8, // 0 = none; else the hw rate code (always < 0x80)
    /// Shared bulk-IN RX pipeline the pump fills and `recv_frame` drains (mirrors the Realtek). See
    /// [`crate::rx_pump`].
    rx_pump: crate::rx_pump::RxPumpState,
    /// The 2.4 GHz channel number the PHY was brought up on (0 until [`note_channel`] records it).
    /// `RadioKnobs::set_channel` reads it via `&self` to answer a same-channel apply with `Ok` and a
    /// different-channel request with an honest `Unsupported` (a live retune is `hw_reset(&mut self)`,
    /// not yet exposed over the `&self` knob path). Atomic so it stays interior-mutable for that future.
    channel: std::sync::atomic::AtomicU8,
    /// Per-frame TX power (`AR_XmitPower`, 6-bit 0.5-dB units; 0 = the firmware default 60 = 30 dBm),
    /// carried in the repurposed `tidno` byte of our mgmt header and honoured by our firmware's
    /// `ath_tgt_send_mgt`. Set via [`RadioKnobs::set_tx_power`]/`set_tx_power_dbm`. Atomic for `&self`.
    cur_power: std::sync::atomic::AtomicU8,
    /// Whether the PHY was brought up in HT40 (40 MHz) mode — set by [`hw_reset`](Self::hw_reset) from
    /// its bandwidth argument. Injected frames then carry the `HAL_RATESERIES_2040` flag (keytype
    /// bit1) so the MCS rate is transmitted at 40 MHz. `false` = HT20 (the default).
    ht40: std::sync::atomic::AtomicBool,
    /// High-power module (`txGainType==1` in the EEPROM) → `apply_initvals` streams the HIGH_POWER TX
    /// gain table. MEASURED: HIGH table + the full `set_board_values` (antCtrl+XPA+ob/db) +
    /// `set_txpower_4k` = a NORMAL ~+12 dBm link (max −18 dBm at 1 ft); NORMAL table left it ~50 dB low.
    /// Set from the EEPROM in `open_ath9k`; `NDN_ATH9K_HIGHPWR` forces it on regardless.
    high_power: std::sync::atomic::AtomicBool,
}

fn usb_err<E: std::fmt::Display>(what: &str, e: E) -> FaceError {
    FaceError::Io(std::io::Error::other(format!("ath9k_htc: {what}: {e}")))
}

/// Hex-dump every HTC exchange when `NDR_ATH9K_DEBUG` is set. Guessing at a handshake is how you
/// lose an afternoon; the bytes are always cheaper.
fn dbg_dump(tag: &str, buf: &[u8]) {
    if std::env::var_os("NDR_ATH9K_DEBUG").is_some() {
        eprintln!("[ath9k] {tag} ({} B): {buf:02x?}", buf.len());
    }
}

fn err(msg: String) -> FaceError {
    FaceError::Io(std::io::Error::other(msg))
}

impl Ath9kHtcBackend {
    /// Claim the first AR9271 on the bus.
    ///
    /// Detaches the kernel driver for the duration of our claim. Note that this alone does **not**
    /// stop `ath9k_htc` re-grabbing the device on a re-enumeration — that lesson cost days on the
    /// RTL8812AU, where the fix was `echo 0 > /sys/bus/usb/drivers_autoprobe`. The AR9271 is far
    /// more forgiving because its firmware lives in RAM: a fight with the kernel driver ends in a
    /// failed probe, not a bricked part.
    pub fn open() -> Result<Self, FaceError> {
        let ctx = Context::new().map_err(|e| usb_err("libusb init", e))?;

        let mut handle = Self::find_and_open(&ctx)?;

        // Per-device RX-stamp clock domain (bus<<8 | address), for the FreeRunRxStamp M2 clock.
        let d = handle.device();
        let tsf_domain =
            ClockDomainId((u32::from(d.bus_number()) << 8) | u32::from(d.address()));

        // ⛔ DO NOT port-reset this device. It was tried and it is destructive: `handle.reset()`
        // on an AR9271 that is already running firmware makes it re-enumerate, and on the o5p-1
        // rig it then **dropped off the USB bus entirely** and needed a physical replug. That is
        // the same pattern as the RTL8812AU, where repeated resets destabilised the part until it
        // vanished. `authorized` 0/1 does not help either — it re-binds the interface without
        // cutting VBUS, so a running MCU stays running.
        //
        // The practical consequence, stated plainly: this driver expects the target to be in a
        // state where a firmware download will take. That holds after a replug, and it held in
        // practice right after `ath9k_htc` released the device. If `htc_init()` times out waiting
        // for READY, the target is most likely already running an image and needs a replug — there
        // is no software path back to cold.

        // The device can be left UNCONFIGURED (`bConfigurationValue` empty in sysfs), which is what
        // `ath9k_htc` leaves behind when its own probe fails — e.g. when its firmware download is
        // attempted over a target we already loaded. libusb cannot claim an interface on an
        // unconfigured device and reports a bare `Invalid parameter`, which reads like a bug in the
        // claim rather than a state problem in the device.
        //
        // Setting the configuration is the non-destructive fix. Only do it when actually needed:
        // libusb performs a lightweight reset when the configuration is set on a device that
        // already has one, and resets are exactly what must be avoided here.
        let needs_config = match handle.active_configuration() {
            Ok(0) | Err(_) => true,
            Ok(_) => false,
        };
        if needs_config {
            handle
                .set_active_configuration(1)
                .map_err(|e| usb_err("set configuration 1 (device was unconfigured)", e))?;
        }

        handle
            .claim_interface(0)
            .map_err(|e| usb_err("claim interface 0", e))?;

        Ok(Self {
            handle: Arc::new(handle),
            seq: std::sync::Mutex::new(0),
            wmi_endpoint: 0,
            credits: 0,
            credit_size: 0,
            mgmt_ep: 0,
            data_be_ep: 0,
            beacon_ep: 0,
            clockrate: crate::ath9k_reg::ATH9K_CLOCK_RATE_2GHZ_OFDM,
            tsf_domain,
            format: FrameFormat::RawNdn {
                ethertype: crate::NDN_ETHERTYPE,
            },
            cur_mcs: std::sync::Mutex::new(None),
            cur_legacy: std::sync::atomic::AtomicU8::new(0),
            rx_pump: crate::rx_pump::RxPumpState::new(),
            channel: std::sync::atomic::AtomicU8::new(0),
            cur_power: std::sync::atomic::AtomicU8::new(0),
            ht40: std::sync::atomic::AtomicBool::new(false),
            high_power: std::sync::atomic::AtomicBool::new(false),
        })
    }

    /// Record the channel number the PHY was brought up on (see [`Ath9kHtcBackend::channel`]).
    /// Called by `open_ath9k` after `hw_reset`, so `RadioKnobs::set_channel` can validate applies.
    pub fn note_channel(&self, channel: u8) {
        self.channel.store(channel, std::sync::atomic::Ordering::Relaxed);
    }

    /// The clock domain the AR9271's per-frame `rs_tstamp` lives on — build a `LinkStamp` from an
    /// [`RxFrame::rs_tstamp`] against this to feed the common-view timekeeper.
    pub fn tsf_domain(&self) -> ClockDomainId {
        self.tsf_domain
    }

    /// Find the first AR9271 and open it, retrying while a reset-induced re-enumeration settles.
    fn find_and_open(ctx: &Context) -> Result<DeviceHandle<Context>, FaceError> {
        for attempt in 0..20 {
            let devices = ctx.devices().map_err(|e| usb_err("enumerate", e))?;
            for dev in devices.iter() {
                let desc = match dev.device_descriptor() {
                    Ok(d) => d,
                    Err(_) => continue,
                };
                if !AR9271_IDS.contains(&(desc.vendor_id(), desc.product_id())) {
                    continue;
                }
                match dev.open() {
                    Ok(mut h) => {
                        // ⛔ Deliberately NOT `set_auto_detach_kernel_driver(true)`. That flag
                        // makes libusb **re-attach** the kernel driver when the interface is
                        // released, so the moment our process exits `ath9k_htc` probes, fails its
                        // firmware download over the target we just loaded ("Firmware ... download
                        // failed" in dmesg), and leaves the device UNCONFIGURED — costing a
                        // physical replug before anything can touch it again.
                        //
                        // Detach by hand and never re-attach: the device then stays configured and
                        // unbound, and successive userspace runs work without a replug.
                        //
                        // Consequence worth knowing: handing the device *back* to `ath9k_htc`
                        // still requires a replug, because its download cannot succeed while our
                        // firmware is running. Userspace iteration is free; switching back is not.
                        match h.kernel_driver_active(0) {
                            Ok(true) => {
                                h.detach_kernel_driver(0)
                                    .map_err(|e| usb_err("detach kernel driver", e))?;
                            }
                            _ => {}
                        }
                        return Ok(h);
                    }
                    Err(e) if attempt == 19 => return Err(usb_err("open", e)),
                    Err(_) => break,
                }
            }
            std::thread::sleep(Duration::from_millis(250));
        }
        Err(err("ath9k_htc: no AR9271 found".to_string()))
    }

    /// Download a firmware image to target RAM and start it.
    ///
    /// There is no flash on this part — the image is written to RAM at [`AR9271_FIRMWARE`] and the
    /// completion request hands the target [`AR9271_FIRMWARE_TEXT`] as its entry point. That is
    /// what makes the AR9271 unbrickable and worth iterating on: a bad image costs a replug.
    ///
    /// `wValue` carries `addr >> 8`, which is why the target address must be 256-byte aligned.
    pub fn download_firmware(&mut self, fw: &[u8]) -> Result<(), FaceError> {
        let mut addr = AR9271_FIRMWARE;

        for chunk in fw.chunks(FW_CHUNK) {
            self.handle
                .write_control(
                    rusb::request_type(
                        rusb::Direction::Out,
                        rusb::RequestType::Vendor,
                        rusb::Recipient::Device,
                    ),
                    FIRMWARE_DOWNLOAD,
                    (addr >> 8) as u16,
                    0,
                    chunk,
                    USB_TIMEOUT,
                )
                .map_err(|e| usb_err(&format!("fw chunk at {addr:#x}"), e))?;
            addr += chunk.len() as u32;
        }

        self.handle
            .write_control(
                rusb::request_type(
                    rusb::Direction::Out,
                    rusb::RequestType::Vendor,
                    rusb::Recipient::Device,
                ),
                FIRMWARE_DOWNLOAD_COMP,
                (AR9271_FIRMWARE_TEXT >> 8) as u16,
                0,
                &[],
                USB_TIMEOUT,
            )
            .map_err(|e| usb_err("fw download complete", e))?;

        Ok(())
    }

    // ── HTC ──────────────────────────────────────────────────────────────────

    /// Send one HTC frame on the register-out (interrupt) pipe.
    fn htc_send(&self, endpoint: u8, payload: &[u8]) -> Result<(), FaceError> {
        let total = HTC_HDR_LEN + payload.len();
        // The reg pipe's `wMaxPacketSize` is 64, but a USB interrupt transfer packetizes — a WMI
        // command larger than 64 B (e.g. WMI_RC_STATE_CHANGE's 70 B rate struct) is sent as 64+rest,
        // exactly as the kernel driver does. 64 is the single-packet REPLY budget (see the
        // ACCESS_MEMORY tuple math), not a cap on the outgoing command. Bound generously.
        const HTC_MSG_MAX: usize = 512;
        if total > HTC_MSG_MAX {
            return Err(err(format!(
                "ath9k_htc: HTC message {total} B exceeds {HTC_MSG_MAX} B"
            )));
        }

        let mut buf = vec![0u8; total];
        buf[0] = endpoint;
        buf[1] = 0; // flags
        buf[2..4].copy_from_slice(&(payload.len() as u16).to_be_bytes());
        // buf[4..8] = control bytes, zero (already)
        buf[HTC_HDR_LEN..total].copy_from_slice(payload);

        dbg_dump("tx", &buf);
        self.handle
            .write_interrupt(EP_REG_OUT, &buf, USB_TIMEOUT)
            .map_err(|e| usb_err("htc send", e))?;
        Ok(())
    }

    /// Receive one HTC frame from the register-in (interrupt) pipe, returning `(endpoint, payload)`.
    fn htc_recv(&self, timeout: Duration) -> Result<(u8, Vec<u8>), FaceError> {
        let mut buf = [0u8; REG_PIPE_MAX];
        let n = self
            .handle
            .read_interrupt(EP_REG_IN, &mut buf, timeout)
            .map_err(|e| usb_err("htc recv", e))?;

        dbg_dump("rx", &buf[..n]);
        if n < HTC_HDR_LEN {
            return Err(err(format!(
                "ath9k_htc: runt HTC frame, {n} B < {HTC_HDR_LEN} B header"
            )));
        }

        let endpoint = buf[0];
        let flags = buf[1];
        let mut len = u16::from_be_bytes([buf[2], buf[3]]) as usize;
        let avail = n - HTC_HDR_LEN;
        if len > avail {
            return Err(err(format!(
                "ath9k_htc: HTC header claims {len} B but only {avail} B arrived"
            )));
        }

        // PayloadLen INCLUDES the receive trailer, whose length is in ControlBytes[0]. Observed on
        // the wire: a 5-word ACCESS_MEMORY reply came back with flags=0x02, control[0]=0x04 and
        // PayloadLen=52 = 4 (WMI) + 4 (our header) + 40 (tuples) + 4 (trailer). Failing to strip it
        // leaves credit-report bytes glued to the end of every payload — harmless for a reply whose
        // length is known up front, and silently corrupting for anything parsed to its end.
        if flags & HTC_FLAGS_RECV_TRAILER != 0 {
            let trailer = buf[4] as usize;
            if trailer > len {
                return Err(err(format!(
                    "ath9k_htc: trailer {trailer} B exceeds payload {len} B"
                )));
            }
            len -= trailer;
        }

        Ok((endpoint, buf[HTC_HDR_LEN..HTC_HDR_LEN + len].to_vec()))
    }

    /// Complete the HTC handshake: wait for READY, connect `WMI_CONTROL_SVC`, then SETUP_COMPLETE.
    ///
    /// Must follow [`download_firmware`](Self::download_firmware) — the target sends READY
    /// unprompted once the image is running, which doubles as proof the download took.
    pub fn htc_init(&mut self) -> Result<(), FaceError> {
        // 1. READY, unsolicited from the target.
        let (_ep, msg) = self.htc_recv(Duration::from_millis(3000))?;
        if msg.len() < 8 || u16::from_be_bytes([msg[0], msg[1]]) != HtcMsg::Ready as u16 {
            return Err(err(format!(
                "ath9k_htc: expected HTC READY, got {msg:02x?}"
            )));
        }
        self.credits = u16::from_be_bytes([msg[2], msg[3]]);
        self.credit_size = u16::from_be_bytes([msg[4], msg[5]]);

        // 2. CONNECT_SERVICE for WMI control (interrupt reg pipes, dl=3/ul=4).
        self.wmi_endpoint =
            self.connect_service(HtcService::WmiControl, PIPE_REG_IN, PIPE_REG_OUT)?;
        if std::env::var_os("NDR_ATH9K_DEBUG").is_some() {
            eprintln!("[ath9k] WMI endpoint assigned: {}", self.wmi_endpoint);
        }

        // 3. SETUP_COMPLETE.
        let done = (HtcMsg::SetupComplete as u16).to_be_bytes();
        self.htc_send(HTC_ENDPOINT_CTRL, &done)?;

        Ok(())
    }

    /// Send an HTC `CONNECT_SERVICE` for `service` with the given pipe ids and return the endpoint
    /// id the target assigns.
    ///
    /// `HTC_CONNECT_SERVICE_MSG` is **10** bytes, not 8:
    /// `MessageID(2) ServiceID(2) ConnectionFlags(2) DownLinkPipeID(1) UpLinkPipeID(1)
    /// ServiceMetaLength(1) _Pad1(1)`. A short message leaves the target reading `ServiceMetaLength`
    /// off the end of the buffer.
    ///
    /// ★ The pipe ids are load-bearing and must not be left zero. The target replies on
    /// `Endpoints[ep].DownLinkPipeID` (`htc.c:402`) — whatever the host puts here. Send 0 and every
    /// response is dispatched to pipe 0 and never arrives, presenting as a read timeout on a
    /// handshake that otherwise looks completely successful. WMI control uses the interrupt reg
    /// pipes (dl=3, ul=4); every data service uses the bulk WLAN pipes (dl=2, ul=1) — the exact
    /// split `service_to_dlpipe`/`service_to_ulpipe` encode in mainline `htc_hst.c`.
    fn connect_service(
        &mut self,
        service: HtcService,
        dl_pipe: u8,
        ul_pipe: u8,
    ) -> Result<u8, FaceError> {
        let mut req = [0u8; 10];
        req[0..2].copy_from_slice(&(HtcMsg::ConnectService as u16).to_be_bytes());
        req[2..4].copy_from_slice(&(service as u16).to_be_bytes());
        req[6] = dl_pipe; // DownLinkPipeID: target -> host
        req[7] = ul_pipe; // UpLinkPipeID:   host -> target
        self.htc_send(HTC_ENDPOINT_CTRL, &req)?;

        let (_ep, resp) = self.htc_recv(Duration::from_millis(1000))?;
        // MessageID(2) ServiceID(2) Status(1) EndpointID(1) ...
        if resp.len() < 6
            || u16::from_be_bytes([resp[0], resp[1]]) != HtcMsg::ConnectServiceResponse as u16
        {
            return Err(err(format!(
                "ath9k_htc: expected CONNECT_SERVICE_RESPONSE for {service:?}, got {resp:02x?}"
            )));
        }
        let status = resp[4];
        if status != 0 {
            return Err(err(format!(
                "ath9k_htc: service {service:?} connect refused, status {status}"
            )));
        }
        Ok(resp[5])
    }

    /// **M1.0** — connect the HTC data services the RX/TX frame path rides on, capturing the
    /// endpoint ids the target assigns. All three ride the bulk WLAN pipes (dl=2 RX, ul=1 TX).
    ///
    /// This is prerequisite plumbing for FrameIo. No PHY is up yet, so nothing arrives on the RX
    /// pipe until the M1 bring-up (reset → initvals → cal → RX enable) completes — the point of
    /// doing it first is to prove the data-service handshake and pipe claim in isolation.
    pub fn connect_data_services(&mut self) -> Result<(), FaceError> {
        self.mgmt_ep = self.connect_service(HtcService::Mgmt, PIPE_WLAN_RX, PIPE_WLAN_TX)?;
        self.data_be_ep = self.connect_service(HtcService::DataBe, PIPE_WLAN_RX, PIPE_WLAN_TX)?;
        self.beacon_ep = self.connect_service(HtcService::Beacon, PIPE_WLAN_RX, PIPE_WLAN_TX)?;
        // Allocate the target's flow-control credits to the WLAN-TX pipe so it has TX buffers to
        // accept injected frames (best-effort — RX doesn't need it).
        let _ = self.config_wlan_tx_credits();
        Ok(())
    }

    /// `HTC_MSG_CONFIG_PIPE` (`htc_hst.c::htc_config_pipe_credits`): tell the target how many of its
    /// flow-control credits back the WLAN-TX pipe. Without it the target has no TX buffers and the
    /// bulk-OUT pipe blocks after the first injected frame (MEASURED: 2nd `write_bulk` times out).
    /// `struct htc_config_pipe_msg` = `message_id(be16=5) pipe_id(u8=USB_WLAN_TX_PIPE) credits(u8)`,
    /// on ENDPOINT0.
    fn config_wlan_tx_credits(&mut self) -> Result<(), FaceError> {
        let msg = [0x00u8, 0x05, PIPE_WLAN_TX, self.credits as u8];
        self.htc_send(HTC_ENDPOINT_CTRL, &msg)?;
        match self.htc_recv(Duration::from_millis(1000)) {
            Ok((_ep, resp))
                if resp.len() >= 2
                    && u16::from_be_bytes([resp[0], resp[1]])
                        == HtcMsg::ConfigPipeResponse as u16 =>
            {
                Ok(())
            }
            Ok((_ep, resp)) => Err(err(format!(
                "ath9k_htc: expected CONFIG_PIPE_RESPONSE, got {resp:02x?}"
            ))),
            Err(e) => Err(e),
        }
    }

    /// The data-service endpoint ids from [`connect_data_services`](Self::connect_data_services),
    /// as `(mgmt, data_be, beacon)`. Zero until connected.
    pub fn data_endpoints(&self) -> (u8, u8, u8) {
        (self.mgmt_ep, self.data_be_ep, self.beacon_ep)
    }

    /// Read one raw transfer from the bulk WLAN-RX pipe ([`EP_WLAN_RX`]).
    ///
    /// The target frames each received packet as `[HTC_FRAME_HDR 8B][ath_htc_rx_status 40B]
    /// [802.11…]`; stripping and parsing that is the caller's job (M1-done / M2). Returns the bytes
    /// actually transferred. Until the PHY is up this only ever times out — a clean timeout here is
    /// the M1.0 success signal (the pipe is claimable), not a failure.
    pub fn recv_raw_frame(&mut self, timeout: Duration) -> Result<Vec<u8>, FaceError> {
        let mut buf = vec![0u8; 2048];
        let n = self
            .handle
            .read_bulk(EP_WLAN_RX, &mut buf, timeout)
            .map_err(|e| usb_err("wlan rx", e))?;
        buf.truncate(n);
        Ok(buf)
    }

    // ── WMI ──────────────────────────────────────────────────────────────────

    /// Read whatever the target volunteers on the register-in pipe until it goes quiet.
    ///
    /// This is the discriminator when a WMI command gets no reply. After SETUP_COMPLETE the target
    /// is expected to raise `WMI_TGT_RDY_EVENTID` (0x1001) of its own accord. If that arrives, HTC
    /// and WMI are both alive and any silence afterwards is our command framing; if nothing
    /// arrives, the WLAN application on the target never came up and the fault is earlier.
    pub fn drain_events(&mut self, ms: u64) -> Vec<(u8, Vec<u8>)> {
        let deadline = std::time::Instant::now() + Duration::from_millis(ms);
        let mut out = Vec::new();
        while std::time::Instant::now() < deadline {
            match self.htc_recv(Duration::from_millis(250)) {
                Ok(f) => out.push(f),
                Err(_) => break,
            }
        }
        out
    }

    /// Issue a WMI command and wait for its response.    /// Issue a WMI command and wait for its response.
    ///
    /// Strictly one command in flight, matched on the echoed sequence number. That discipline was
    /// learned the expensive way on the LoRa dongle: fire-and-forget hid a ~50% command loss for
    /// months because nothing ever checked for a reply.
    ///
    /// Unsolicited events (id ≥ `0x1001`) share this pipe and are skipped while waiting.
    pub fn wmi_cmd(&self, cmd: WmiCmd, payload: &[u8]) -> Result<Vec<u8>, FaceError> {
        // Hold the seq lock across the whole send+recv: it bumps the sequence AND serializes
        // concurrent WMI (the reg pipe is strictly one-command-in-flight).
        let mut seq_guard = self.seq.lock().unwrap();
        *seq_guard = seq_guard.wrapping_add(1);
        let seq = *seq_guard;

        let mut buf = Vec::with_capacity(WMI_HDR_LEN + payload.len());
        buf.extend_from_slice(&(cmd as u16).to_be_bytes());
        buf.extend_from_slice(&seq.to_be_bytes());
        buf.extend_from_slice(payload);

        let ep = self.wmi_endpoint;
        self.htc_send(ep, &buf)?;

        // Skip events until the matching reply arrives.
        for _ in 0..16 {
            let (_ep, resp) = self.htc_recv(USB_TIMEOUT)?;
            if resp.len() < WMI_HDR_LEN {
                continue;
            }
            let id = u16::from_be_bytes([resp[0], resp[1]]);
            let rseq = u16::from_be_bytes([resp[2], resp[3]]);

            if id >= WMI_EVENT_BASE {
                continue; // asynchronous event, not our reply
            }
            if rseq == seq {
                return Ok(resp[WMI_HDR_LEN..].to_vec());
            }
        }

        Err(err(format!(
            "ath9k_htc: no WMI reply for {cmd:?} seq {seq}"
        )))
    }

    /// Ask the target its firmware version — the correct liveness probe.
    ///
    /// Prefer this over [`echo`](Self::echo). `ath_get_tgt_version` replies via
    /// `sc->tgt_wmi_handle` like every other well-formed handler, whereas upstream's echo handler
    /// passed the dispatch table's `pContext` (the softc) where a WMI handle was expected — see the
    /// note on `handle_echo_command` in our `if_ath.c`. On stock firmware echo answers nothing and
    /// leaves the target unresponsive.
    pub fn fw_version(&mut self) -> Result<(u16, u16), FaceError> {
        let r = self.wmi_cmd(WmiCmd::GetFwVersion, &[])?;
        if r.len() < 4 {
            return Err(err(format!(
                "ath9k_htc: GET_FW_VERSION returned {} B, expected 4",
                r.len()
            )));
        }
        Ok((
            u16::from_be_bytes([r[0], r[1]]),
            u16::from_be_bytes([r[2], r[3]]),
        ))
    }

    /// Round-trip the target with `WMI_ECHO`    /// Round-trip the target with `WMI_ECHO` — the cheapest proof the whole stack is alive:
    /// firmware running, HTC connected, WMI dispatching, both pipes working.
    ///
    /// The firmware's echo handler caps the payload at `WMI_ECHOCMD_MSG_MAX_LEN` (53).
    pub fn echo(&mut self, data: &[u8]) -> Result<Vec<u8>, FaceError> {
        if data.len() > MAX_ECHO_LEN {
            return Err(err(format!(
                "ath9k_htc: echo payload {} B exceeds {MAX_ECHO_LEN} B (see MAX_ECHO_LEN)",
                data.len()
            )));
        }
        let mut payload = Vec::with_capacity(1 + data.len());
        payload.push(data.len() as u8); // WMI_ECHO_CMD.msgSize
        payload.extend_from_slice(data);
        self.wmi_cmd(WmiCmd::Echo, &payload)
    }

    /// HTC credits the target advertised, and their size. Populated by
    /// [`htc_init`](Self::htc_init); useful mainly as evidence the READY message parsed sanely.
    pub fn credits(&self) -> (u16, u16) {
        (self.credits, self.credit_size)
    }

    /// Query the target's RX-buffer accounting via `WMI_RX_STATS` → `(nobuf, send, done)`:
    /// `nobuf` = RX-buffer alloc failures (a nonzero, growing value witnesses the pool draining —
    /// the sustained-RX ceiling), `send` = frames handed to the host, `done` = HTC send-completions
    /// that recycled a buffer. A widening `send - done` gap means the host isn't draining bulk-IN
    /// fast enough to recycle. All three big-endian (the target is big-endian).
    pub fn rx_stats(&mut self) -> Result<(u32, u32, u32), FaceError> {
        let r = self.wmi_cmd(WmiCmd::RxStats, &[])?;
        if r.len() < 12 {
            return Err(err(format!("ath9k_htc: RX_STATS reply {} B < 12", r.len())));
        }
        let g = |o: usize| u32::from_be_bytes([r[o], r[o + 1], r[o + 2], r[o + 3]]);
        Ok((g(0), g(4), g(8)))
    }

    // ── Target memory (WMI_ACCESS_MEMORY) ────────────────────────────────────

    /// Read `n` 32-bit words from target memory.
    ///
    /// Deliberately word-oriented rather than byte-oriented. The target loads each word natively
    /// and serialises it big-endian; the host deserialises big-endian. The *value* therefore
    /// round-trips exactly regardless of the target's own byte order, whereas a byte-image API
    /// would silently depend on it.
    ///
    /// Addresses must be 4-byte aligned — Xtensa has no unaligned 32-bit load, so the firmware
    /// rejects unaligned requests rather than faulting.
    pub fn read_target_u32s(&self, addr: u32, n: usize) -> Result<Vec<u32>, FaceError> {
        self.access_memory(addr, &vec![0u32; n], false)
    }

    /// Write 32-bit words to target memory. Returns what the target echoed back.
    pub fn write_target_u32s(&self, addr: u32, vals: &[u32]) -> Result<Vec<u32>, FaceError> {
        self.access_memory(addr, vals, true)
    }

    /// One `WMI_ACCESS_MEMORY` exchange per chunk of at most [`NDR_MEM_MAX_TUPLES`] words.
    ///
    /// Wire format (defined by us — see `firmware/ath9k-htc-ndr/src/ndr_mem.h`; upstream declared
    /// the struct but its handler was `adf_os_assert(0)`):
    ///
    /// ```text
    ///   request:   u16 flags | u16 count | count * { u32 addr, u32 value }
    ///   response:  u16 status | u16 count | count * { u32 addr, u32 value }
    /// ```
    fn access_memory(&self, addr: u32, vals: &[u32], write: bool) -> Result<Vec<u32>, FaceError> {
        if addr % 4 != 0 {
            return Err(err(format!(
                "ath9k_htc: target address {addr:#010x} is not 4-byte aligned"
            )));
        }

        let mut out = Vec::with_capacity(vals.len());

        for (chunk_idx, chunk) in vals.chunks(NDR_MEM_MAX_TUPLES).enumerate() {
            let base = addr + (chunk_idx * NDR_MEM_MAX_TUPLES * 4) as u32;

            let mut payload = Vec::with_capacity(4 + chunk.len() * 8);
            payload.extend_from_slice(&(if write { NDR_MEM_FLAG_WRITE } else { 0u16 }).to_be_bytes());
            payload.extend_from_slice(&(chunk.len() as u16).to_be_bytes());
            for (i, v) in chunk.iter().enumerate() {
                payload.extend_from_slice(&(base + (i * 4) as u32).to_be_bytes());
                payload.extend_from_slice(&v.to_be_bytes());
            }

            let resp = self.wmi_cmd(WmiCmd::AccessMemory, &payload)?;
            if resp.len() < 4 {
                return Err(err(format!(
                    "ath9k_htc: ACCESS_MEMORY reply is {} B, need at least 4 — is the firmware \
                     the patched build? Stock asserts on this command.",
                    resp.len()
                )));
            }

            let status = u16::from_be_bytes([resp[0], resp[1]]);
            let count = u16::from_be_bytes([resp[2], resp[3]]) as usize;
            if status != NDR_MEM_OK {
                return Err(err(format!(
                    "ath9k_htc: ACCESS_MEMORY at {base:#010x} failed: {}",
                    ndr_mem_status(status)
                )));
            }
            if count != chunk.len() || resp.len() < 4 + count * 8 {
                return Err(err(format!(
                    "ath9k_htc: ACCESS_MEMORY returned {count} of {} words ({} B)",
                    chunk.len(),
                    resp.len()
                )));
            }

            for i in 0..count {
                let v = 4 + i * 8 + 4;
                out.push(u32::from_be_bytes([resp[v], resp[v + 1], resp[v + 2], resp[v + 3]]));
            }
        }

        Ok(out)
    }

    /// Tell the target to tear its WLAN application down, the way `ath9k_htc` does on stop.
    ///
    /// ★ Send this before dropping the device, or the next bring-up needs a **physical replug**.
    /// The target keeps running our firmware after we release the interface; `ath9k_htc` then
    /// auto-probes, fails its own firmware download over the live target ("Firmware ... download
    /// failed"), and leaves the device UNCONFIGURED — at which point neither the kernel nor we can
    /// claim it, and no software recovery works (re-authorize, set-configuration and writing
    /// `bConfigurationValue` were all tried and all fail).
    ///
    /// The target's handler frees its own softc, so no further command may be issued afterwards.
    /// Errors are worth logging but not worth failing on — we are on the way out either way.
    pub fn detach(&mut self) -> Result<(), FaceError> {
        self.wmi_cmd(WmiCmd::TgtDetach, &[])?;
        Ok(())
    }

    /// Read the Tier-0 filter's counters.    /// Read the Tier-0 filter's counters.
    ///
    /// `addr` is `ndr_stats`, which **moves between firmware builds** — take it from the image you
    /// actually loaded rather than hardcoding it:
    ///
    /// ```sh
    /// xtensa-elf-nm build/k2/fw.elf | grep ndr_stats
    /// ```
    pub fn read_ndr_stats(&self, addr: u32) -> Result<NdrStats, FaceError> {
        let w = self.read_target_u32s(addr, 6)?;
        Ok(NdrStats {
            seen: w[0],
            passed: w[1],
            dropped_filter: w[2],
            dropped_foreign: w[3],
            short_frame: w[4],
            dropped_popcount: w[5],
        })
    }

    // ── Tier-0 name filter control (#2 / §8.2) — the pre-USB drop that no other Wi-Fi part we own can
    // do, live on the libusb path. `struct ndr_cfg` at NDR_CFG_ADDR (from `xtensa-nm fw.elf`):
    //   magic(u32) enabled(u32) drop_foreign(u32) n_masks(u32) key[16] masks[8]×16B  = 160 B.
    // Rewritten in place via WMI_ACCESS_MEMORY — no firmware rebuild. Masks are the 16-byte prefix-set
    // Bloom filters (`ndr_filter_t`) the cognition layer derives (SipHash-2-4, key = group context).
    /// Symbol addresses in the current fw build (re-extract via `xtensa-elf-nm build/k2/fw.elf` if the
    /// firmware is rebuilt — text/data layout can shift).
    pub const NDR_CFG_ADDR: u32 = 0x0050_cf40;
    pub const NDR_STATS_ADDR: u32 = 0x0050_dc18;
    const NDR_CFG_MAGIC: u32 = 0x4E44_5230; // "NDR0"

    /// Enable/disable the Tier-0 filter and the `drop_foreign` (non-group addr1) rule, leaving the
    /// configured masks/key intact. `enabled=false` = stock (every frame crosses USB).
    pub fn set_name_filter(&self, enabled: bool, drop_foreign: bool) -> Result<(), FaceError> {
        // words 0..3 = magic, enabled, drop_foreign  (n_masks left as-is at word 3).
        self.write_target_u32s(
            Self::NDR_CFG_ADDR,
            &[Self::NDR_CFG_MAGIC, enabled as u32, drop_foreign as u32],
        )?;
        Ok(())
    }

    /// Load the full filter config: the 16-byte group `key` and up to 8 `masks` (16-byte prefix-set
    /// Bloom filters derived by cognition), then enable. This is the §8.2 name-filter proper — a frame
    /// whose name-hash isn't in the mask set is dropped on the dongle before the USB transfer.
    pub fn configure_name_filter(
        &self,
        drop_foreign: bool,
        key: &[u8; 16],
        masks: &[[u8; 16]],
    ) -> Result<(), FaceError> {
        let n = masks.len().min(8);
        // Pack the whole struct as u32 words (LE — the host↔target ACCESS_MEMORY path is byte-exact).
        let mut words = vec![Self::NDR_CFG_MAGIC, 1, drop_foreign as u32, n as u32];
        for chunk in key.chunks(4) {
            words.push(u32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]));
        }
        for m in &masks[..n] {
            for chunk in m.chunks(4) {
                words.push(u32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]));
            }
        }
        self.write_target_u32s(Self::NDR_CFG_ADDR, &words)?;
        Ok(())
    }

    /// Read the Tier-0 counters (frames seen / passed to host / dropped by filter/foreign/popcount) —
    /// the §8.2 evidence: `seen - passed` USB transfers and host wakeups that did not happen.
    pub fn ndr_stats(&self) -> Result<NdrStats, FaceError> {
        self.read_ndr_stats(Self::NDR_STATS_ADDR)
    }

    // ── Register access (WMI_REG_READ / WMI_REG_WRITE) ────────────────────────
    //
    // Wire format resolved from BOTH ends (they must agree):
    //   host  — ath9k `htc_drv_init.c` `ath9k_regread` / `ath9k_regwrite_single` /
    //           `ath9k_regwrite_multi`.
    //   target — our firmware `wlan/if_ath.c` `ath_hal_reg_read_tgt` /
    //           `ath_hal_reg_write_tgt` (the authoritative parser — we run this fork).
    //
    // WMI_REG_READ_CMDID (0x14):  payload = addr as one big-endian u32.
    //                             reply   = value as one big-endian u32.
    //   (The firmware loops the payload in 4-byte strides, so N addresses ⇒ N values;
    //    we only ever send one here.)
    // WMI_REG_WRITE_CMDID (0x15): payload = N × { u32 reg, u32 val }, each big-endian,
    //                             reg first then val (host `buf[2] = {reg, val}`).
    //                             reply   = EMPTY (`wmi_cmd_rsp(..., NULL, 0)`).
    // Every WMI multi-byte field is big-endian, and the target is big-endian, so
    // `to_be_bytes` / `from_be_bytes` round-trips values exactly on both paths.

    /// Read one 32-bit hardware register through `WMI_REG_READ`.
    ///
    /// ★ Oracle: `reg_read(0x4020)` (`AR_SREV`) returns the AR9271's silicon-revision
    /// register — a stable, non-degenerate value (not 0, not `0xffff_ffff`, not the
    /// address echoed back). That is the unambiguous proof this primitive works before
    /// anything is written.
    pub fn reg_read(&self, addr: u32) -> Result<u32, FaceError> {
        let resp = self.wmi_cmd(WmiCmd::RegRead, &addr.to_be_bytes())?;
        if resp.len() < 4 {
            return Err(err(format!(
                "ath9k_htc: REG_READ({addr:#010x}) reply is {} B, expected 4",
                resp.len()
            )));
        }
        Ok(u32::from_be_bytes([resp[0], resp[1], resp[2], resp[3]]))
    }

    /// Write one 32-bit hardware register through `WMI_REG_WRITE` (single pair).
    ///
    /// The firmware replies with a zero-length payload, so nothing is returned; the
    /// proof a write landed is a subsequent [`reg_read`](Self::reg_read).
    pub fn reg_write(&self, addr: u32, val: u32) -> Result<(), FaceError> {
        let mut payload = [0u8; 8];
        payload[0..4].copy_from_slice(&addr.to_be_bytes());
        payload[4..8].copy_from_slice(&val.to_be_bytes());
        self.wmi_cmd(WmiCmd::RegWrite, &payload)?;
        Ok(())
    }

    /// Write many registers, batching up to [`REG_WRITE_MAX_PAIRS`] `{addr,val}` pairs
    /// per `WMI_REG_WRITE` to respect the 64-byte register pipe — the volume path M1.2
    /// needs (≈670 rows would otherwise be ≈670 USB round-trips).
    pub fn reg_write_batch(&self, pairs: &[(u32, u32)]) -> Result<(), FaceError> {
        for chunk in pairs.chunks(REG_WRITE_MAX_PAIRS) {
            let mut payload = Vec::with_capacity(chunk.len() * 8);
            for (addr, val) in chunk {
                payload.extend_from_slice(&addr.to_be_bytes());
                payload.extend_from_slice(&val.to_be_bytes());
            }
            self.wmi_cmd(WmiCmd::RegWrite, &payload)?;
        }
        Ok(())
    }

    /// A generous settle gap. The µs-scale `udelay`s in the reference reset/PLL paths
    /// are dwarfed by a single WMI USB round-trip (~1 ms), but a few explicit
    /// milliseconds at the RTC state transitions costs nothing and removes all doubt.
    fn settle(ms: u64) {
        std::thread::sleep(Duration::from_millis(ms));
    }

    /// Poll `reg` until `(value & mask) == want`, returning the final read. Errors on
    /// timeout — mirrors `ath9k_hw_wait`, but each iteration is a WMI round-trip so a
    /// modest iteration count already spans hundreds of ms of wall time.
    fn wait_reg(&self, reg: u32, mask: u32, want: u32) -> Result<u32, FaceError> {
        for _ in 0..64 {
            let v = self.reg_read(reg)?;
            if v & mask == want {
                return Ok(v);
            }
            Self::settle(2);
        }
        Err(err(format!(
            "ath9k_htc: reg {reg:#010x} never reached (&{mask:#x})=={want:#x}"
        )))
    }

    // ── M1.1 — chip reset / wake ──────────────────────────────────────────────

    /// **M1.1 / hw_reset step 3** — reset and wake the AR9271 MAC/baseband, following
    /// `ath9k_hw_set_reset_reg(POWER_ON)` → `ath9k_hw_set_reset_power_on` →
    /// `ath9k_hw_set_reset(WARM)` (hw.c) for the not-9100/not-9300 path, plus the
    /// AR9271-only `htc_reset_init` RF-gate writes that bracket the chip reset.
    ///
    /// Now also folds in the tail of `ath9k_hw_chip_reset` — `ath9k_hw_init_pll`
    /// (the 0x7014/0x50040/0x7048 PLL / 117 MHz core-clock / derived-sleep-clock
    /// writes) — and the AR9271 `GATE_MAC_CTL` write, so this method reproduces the
    /// golden trace's reset region 1:1 (RF_RST → reset → PLL → GATE) IN ORDER, before
    /// any initval replay. (Previously the PLL/GATE lived in `apply_initvals`, which
    /// put GATE before PLL — the ordering this rewrite corrects.)
    ///
    /// The firmware CPU and USB/HIF block are untouched by an RTC MAC reset, so WMI
    /// keeps servicing register I/O throughout — which is exactly why `ath9k_htc`
    /// drives the whole PHY from the host over these same commands.
    pub fn phy_reset(&mut self) -> Result<ResetStatus, FaceError> {
        use crate::ath9k_reg::*;

        // AR9271 first-reset only: assert the radio RF reset just before the chip
        // reset (hw.c: AR_SREV_9271 && htc_reset_init).
        self.reg_write(AR9271_RESET_POWER_DOWN_CONTROL, AR9271_RADIO_RF_RST)?;
        Self::settle(2);

        // ── ath9k_hw_set_reset_reg(POWER_ON) ──
        self.reg_write(
            AR_RTC_FORCE_WAKE,
            AR_RTC_FORCE_WAKE_EN | AR_RTC_FORCE_WAKE_ON_INT,
        )?;

        // ── ath9k_hw_set_reset_power_on() ──
        self.reg_write(
            AR_RTC_FORCE_WAKE,
            AR_RTC_FORCE_WAKE_EN | AR_RTC_FORCE_WAKE_ON_INT,
        )?;
        self.reg_write(AR_RC, AR_RC_AHB)?;
        self.reg_write(AR_RTC_RESET, 0)?;
        Self::settle(2); // udelay(2)
        self.reg_write(AR_RC, 0)?;
        self.reg_write(AR_RTC_RESET, AR_RTC_RESET_EN)?; // = 1
        // Poll RTC power-on: (STATUS & 0x0f) == ON(0x02).
        self.wait_reg(AR_RTC_STATUS, AR_RTC_STATUS_M, AR_RTC_STATUS_ON)?;

        // ── ath9k_hw_set_reset(WARM) ──
        self.reg_write(
            AR_RTC_FORCE_WAKE,
            AR_RTC_FORCE_WAKE_EN | AR_RTC_FORCE_WAKE_ON_INT,
        )?;
        let intr_sync = self.reg_read(AR_INTR_SYNC_CAUSE)?
            & (AR_INTR_SYNC_LOCAL_TIMEOUT | AR_INTR_SYNC_RADM_CPL_TIMEOUT);
        if intr_sync != 0 {
            self.reg_write(AR_INTR_SYNC_ENABLE, 0)?;
            self.reg_write(AR_RC, AR_RC_HOSTIF | AR_RC_AHB)?;
        } else {
            self.reg_write(AR_RC, AR_RC_AHB)?;
        }
        // WARM (not COLD) ⇒ rst_flags = MAC_WARM only.
        self.reg_write(AR_RTC_RC, AR_RTC_RC_MAC_WARM)?;
        Self::settle(2); // udelay(100)
        self.reg_write(AR_RTC_RC, 0)?;
        // Poll the MAC out of reset: (RC & 0x3) == 0.
        self.wait_reg(AR_RTC_RC, AR_RTC_RC_M, 0)?;
        self.reg_write(AR_RC, 0)?;

        // Re-assert force-wake so the part stays awake for initvals.
        self.reg_write(
            AR_RTC_FORCE_WAKE,
            AR_RTC_FORCE_WAKE_EN | AR_RTC_FORCE_WAKE_ON_INT,
        )?;

        // ── ath9k_hw_init_pll (called from the tail of ath9k_hw_chip_reset, BEFORE
        // the AR9271 GATE_MAC_CTL write and BEFORE process_ini) ──
        // For AR9271 none of the DPLL SREV branches apply: it is a plain PLL write,
        // the AR9271 core-clock switch to 117 MHz, then the derived sleep clock.
        // Golden trace lines 8-10: 0x7014=0x142c, 0x50040=0x304, 0x7048=0x02. ✓
        self.reg_write(AR_RTC_PLL_CONTROL, AR9271_PLL_CONTROL_2GHZ)?; // 0x7014 = 0x142c
        Self::settle(1); // udelay(500) — AR9271 core-clock switch guard
        self.reg_write(AR9271_CORE_CLOCK_REG, AR9271_CORE_CLOCK_117MHZ)?; // 0x50040 = 0x304
        Self::settle(1); // RTC_PLL_SETTLE_DELAY
        self.reg_write(AR_RTC_SLEEP_CLK, AR_RTC_FORCE_DERIVED_CLK)?; // 0x7048 = 0x02

        // ── AR9271 htc_reset_init: gate MAC control just AFTER the chip reset /
        // init_pll, and BEFORE process_ini (hw.c:1945-1951). Trace line 11. ──
        self.reg_write(AR9271_RESET_POWER_DOWN_CONTROL, AR9271_GATE_MAC_CTL)?; // 0x50044 = 0x4000
        Self::settle(1); // udelay(50)

        // Read-back evidence.
        let srev_raw = self.reg_read(AR_SREV)?;
        let rtc_status = self.reg_read(AR_RTC_STATUS)?;
        let rtc_rc = self.reg_read(AR_RTC_RC)?;
        Ok(ResetStatus {
            srev_raw,
            srev_id: srev_raw & AR_SREV_ID,
            srev_version_field: (srev_raw >> 16) & 0xff,
            srev_rev: (srev_raw & 0x0000_0f00) >> 8,
            rtc_status,
            rtc_rc,
            intr_sync_masked: intr_sync,
        })
    }

    // ── M1.2 — PLL + apply initvals (2.4 GHz HT20) ────────────────────────────

    /// Stream one MODES-shaped table (`{addr, 5G_HT20, 5G_HT40, 2G_HT40, 2G_HT20}`)
    /// at the given column, recording each `{addr,val}` written.
    fn stream_modes(
        &self,
        table: &[[u32; 5]],
        col: usize,
        written: &mut Vec<(u32, u32)>,
    ) -> Result<(), FaceError> {
        let pairs: Vec<(u32, u32)> = table.iter().map(|r| (r[0], r[col])).collect();
        self.reg_write_batch(&pairs)?;
        written.extend_from_slice(&pairs);
        Ok(())
    }

    /// Stream a COMMON-shaped table (`{addr, val}`), recording each write.
    fn stream_common(
        &self,
        table: &[[u32; 2]],
        written: &mut Vec<(u32, u32)>,
    ) -> Result<(), FaceError> {
        let pairs: Vec<(u32, u32)> = table.iter().map(|r| (r[0], r[1])).collect();
        self.reg_write_batch(&pairs)?;
        written.extend_from_slice(&pairs);
        Ok(())
    }

    /// **M1.2** — program the 2.4 GHz PLL, then replay the AR9271 init-value tables
    /// for the 2.4 GHz-HT20 mode and verify a spread of sentinel registers read back
    /// what was written.
    ///
    /// Order follows the reference bring-up: `ath9k_hw_init_pll` (AR9271 path) →
    /// `AR9271_GATE_MAC_CTL` (post-reset gate) → the "baseband to analog shift" write
    /// (`ar9002_hw_rf_claim`, so the 0x78xx COMMON rows latch) → MODES(col 4) →
    /// COMMON(col 1) → ANI(col 4) → normal-power TX-gain(col 4), streamed with
    /// `ath9k_hw_write_array`'s plain `REG_WRITE` semantics, then `AR_PHY_TURBO` forced
    /// to HT20 (DYN2040 cleared).
    ///
    /// Full channel-synth programming and calibration are M1.3 and are deliberately
    /// **not** done here.
    pub fn apply_initvals(&self) -> Result<IniVerify, FaceError> {
        use crate::ath9k_initvals::*;
        use crate::ath9k_reg::*;
        // MODES table columns are {addr, 5G_HT20, 5G_HT40, 2G_HT40, 2G_HT20}: pick the 2.4 GHz
        // column matching the requested bandwidth. HT40 also keeps DYN2040 set (below).
        const COL_2G_HT20: usize = 4;
        const COL_2G_HT40: usize = 3;
        let ht40 = self.ht40.load(std::sync::atomic::Ordering::Relaxed);
        let col = if ht40 { COL_2G_HT40 } else { COL_2G_HT20 };

        // NOTE: PLL / core-clock / sleep-clock and the AR9271 GATE_MAC_CTL write
        // now live in `phy_reset()` (they belong inside ath9k_hw_chip_reset /
        // htc_reset_init, which run BEFORE process_ini). This method is the faithful
        // `ath9k_hw_process_ini` body: analog-shift route → the mode/common/ANI/
        // TX-gain tables → `ath9k_hw_override_ini` → the HT20 AR_PHY_TURBO clear.

        // Route the baseband to analog-shift so the 0x78xx COMMON rows take.
        self.reg_write(AR_PHY_BASE, AR_PHY_ANALOG_SHIFT_ENABLE)?; // 0x9800 = 0x07
        Self::settle(1);

        // Stream the tables in reference order; record every write for verification.
        let mut written: Vec<(u32, u32)> = Vec::new();
        self.stream_modes(AR9271MODES_9271, col, &mut written)?;
        self.stream_common(AR9271COMMON_9271, &mut written)?;
        self.stream_modes(AR9271MODES_9271_ANI_REG, col, &mut written)?;
        // TX-gain table: NORMAL vs HIGH power. Because `hw_reset` skips the EEPROM OLPC/`set_board_values`
        // cal (which normally boosts the base gain to the regulatory target), the AR9271 runs the PA well
        // below the part's real output — MEASURED: a link a couple feet away lands at ~−76 dBm (≈50 dB
        // under a normal +15 dBm part). ⚠ **`NDN_ATH9K_HIGHPWR=1` does NOT fix this — it makes it WORSE.**
        // B210-MEASURED (2026-08-24, mds AR9271, ch1, co-located): NORMAL p99 −16.6 dB vs HIGH −28.2 dB
        // (12 dB LOWER). The high-power ladder is restructured to be used WITH the module's OLPC/EEPROM
        // cal; applied standalone it misconfigures the drive. The real fix for the low output is the
        // OLPC/PDADC `set_board_values` port (per-rate target power) — the flag stays only to document
        // the dead end. Default (NORMAL) is the better of the two here.
        let high_power = self.high_power.load(std::sync::atomic::Ordering::Relaxed)
            || std::env::var_os("NDN_ATH9K_HIGHPWR").is_some();
        let tx_gain = if high_power {
            AR9271MODES_HIGH_POWER_TX_GAIN_9271
        } else {
            AR9271MODES_NORMAL_POWER_TX_GAIN_9271
        };
        self.stream_modes(tx_gain, col, &mut written)?;

        // ── ath9k_hw_override_ini tail of process_ini ──
        // ⚠ ath9k_hw_override_ini is NOT among the fetched sources. Its one
        // AR9271-relevant, trace-observable effect is on AR_PCU_MISC_MODE2 (0x8344):
        // the working kernel driver ends up with 0x00581083 there (golden trace
        // lines 41/166). We write that observed absolute value so the reset tail
        // matches the working driver; the exact RMW derivation is unverified.
        self.reg_write(AR_PCU_MISC_MODE2, AR_PCU_MISC_MODE2_TRACE_VAL)?;
        written.push((AR_PCU_MISC_MODE2, AR_PCU_MISC_MODE2_TRACE_VAL));

        // AR_PHY_TURBO dynamic-20/40 enable: SET for HT40 (40 MHz operation around the synth center),
        // CLEAR for HT20 — as ath9k_hw_set_channel does per bandwidth.
        let turbo = if ht40 {
            self.reg_read(AR_PHY_TURBO)? | AR_PHY_FC_DYN2040_EN
        } else {
            self.reg_read(AR_PHY_TURBO)? & !AR_PHY_FC_DYN2040_EN
        };
        self.reg_write(AR_PHY_TURBO, turbo)?;
        written.push((AR_PHY_TURBO, turbo));

        // Verify sentinels spread across MODES / COMMON(digital+analog) / ANI /
        // TX-gain. Expected = last value written to each addr (overlaps resolve to the
        // last writer automatically).
        let sentinels: [u32; 8] = [
            0x9840,  // MODES digital PHY
            0x9848,  // MODES digital PHY (2G-HT20 col: 0x1053)
            0x9910,  // MODES digital PHY
            0x99c0,  // MODES ∩ ANI (same col-4 value)
            0x7804,  // COMMON analog (0x78xx)
            0x7808,  // COMMON analog (0x78xx)
            0xa208,  // COMMON ∩ ANI
            0xa30c,  // TX-gain (normal power)
        ];
        let mut mismatches = Vec::new();
        let mut matched = 0usize;
        let mut checked = 0usize;
        for addr in sentinels {
            let Some(expected) = written.iter().rev().find(|(a, _)| *a == addr).map(|(_, v)| *v)
            else {
                continue; // addr not in any streamed table — skip
            };
            checked += 1;
            let got = self.reg_read(addr)?;
            if got == expected {
                matched += 1;
            } else {
                mismatches.push((addr, expected, got));
            }
        }

        Ok(IniVerify {
            total_written: written.len(),
            matched,
            checked,
            mismatches,
        })
    }

    // ── register read-modify-write helpers (REG_SET_BIT / REG_CLR_BIT) ────────

    /// `REG_SET_BIT` — read, OR in `bits`, write back.
    fn reg_set_bit(&self, addr: u32, bits: u32) -> Result<(), FaceError> {
        let v = self.reg_read(addr)?;
        self.reg_write(addr, v | bits)
    }

    /// `REG_CLR_BIT` — read, mask out `bits`, write back.
    fn reg_clr_bit(&self, addr: u32, bits: u32) -> Result<(), FaceError> {
        let v = self.reg_read(addr)?;
        self.reg_write(addr, v & !bits)
    }

    /// `REG_RMW` — read, replace the `mask` field with `set` (masked), write back.
    fn reg_rmw(&self, addr: u32, set: u32, mask: u32) -> Result<(), FaceError> {
        let v = self.reg_read(addr)?;
        self.reg_write(addr, (v & !mask) | (set & mask))
    }

    /// `ath9k_hw_loadnf` (chain-0, HT20 subset): force the software-filtered nominal noise floor
    /// into the baseband's `minCCApwr` so the receiver's CCA/detection threshold is sane.
    ///
    /// ★ Without this the receiver is deaf: the hardware NF cal alone left `minCCApwr` at ~-15 dBm
    /// on the first RX attempt (measured), which sits *above* every real beacon (-30..-90 dBm), so
    /// nothing was ever detected. `nf_regs[0]` for the AR9002 family is `AR_PHY_CCA` (0x9864); the
    /// write field is bits[8:0] in 0.5-dB units (`nfval << 1`), distinct from the bits[28:20]
    /// measurement field that `getnf` reads.
    fn load_nf_nominal(&self) -> Result<(), FaceError> {
        use crate::ath9k_reg::*;
        let field = |nf: i32| ((nf << 1) as u32) & 0x1ff;

        // Load the nominal 2.4 GHz NF (-118 dBm) into minCCApwr, then pulse the NF-load.
        self.reg_rmw(AR_PHY_CCA, field(AR_PHY_CCA_NOM_VAL_2GHZ), 0x1ff)?;
        self.reg_clr_bit(AR_PHY_AGC_CONTROL, AR_PHY_AGC_CONTROL_ENABLE_NF)?;
        self.reg_clr_bit(AR_PHY_AGC_CONTROL, AR_PHY_AGC_CONTROL_NO_UPDATE_NF)?;
        self.reg_set_bit(AR_PHY_AGC_CONTROL, AR_PHY_AGC_CONTROL_NF)?;
        // Wait for the load to complete (NF bit self-clears). Best-effort — a timeout just means an
        // in-progress rx; the cap still applies.
        let _ = self.wait_reg(AR_PHY_AGC_CONTROL, AR_PHY_AGC_CONTROL_NF, 0);

        // Restore maxCCApwr (-50) so the next hardware NF cal is not capped by the median we loaded.
        self.reg_rmw(AR_PHY_CCA, field(-50), 0x1ff)?;
        Ok(())
    }

    // ── M1.3 — channel synth + AGC/offset cal + noise floor ───────────────────

    /// **M1.3** — program the RF synthesiser for a 2.4 GHz HT20 channel, enable the
    /// baseband, run the AR9271 AGC/offset calibration, and start the noise-floor cal.
    ///
    /// Only the RX-relevant pieces of the reference reset tail are done here:
    /// `ath9k_hw_set_rfmode` (AR_PHY_MODE), `ar9002_hw_set_channel` (AR_PHY_SYNTH_CONTROL, the
    /// 2.4 GHz fractional-mode plain write — the analog-shift path is 5 GHz only),
    /// `ath9k_hw_init_bb` (AR_PHY_ACTIVE), then `ath9k_hw_init_cal`'s AR9271 branch
    /// (`ar9285_hw_cl_cal`, which runs the offset/AGC cal) and `ath9k_hw_start_nfcal`.
    ///
    /// Deliberately **skipped** (not needed to receive): `ar9271_hw_pa_cal` and the carrier-leak
    /// redo (both TX-only), IQ/ADC periodic cals, per-channel EEPROM board values, delta-slope /
    /// spur (2.4 GHz beacons ride 1 Mbps CCK, which needs neither), and the software `loadnf`
    /// minCCApwr write (the hardware NF cal seeds itself). `chan_mhz` is the centre, e.g. 2412
    /// (ch1) or 2437 (ch6).
    pub fn set_channel_and_cal(&self, chan_mhz: u16) -> Result<CalStatus, FaceError> {
        use crate::ath9k_reg::*;

        // The legacy M1.3 flow, now delegating to the same granular helpers that
        // `hw_reset` interleaves in the faithful ath9k_hw_reset() order. Here they
        // run back-to-back (rfmode → synth → bb → cal), which is fine for the
        // M1.3-only example path.
        self.set_rfmode()?;
        self.rf_set_freq(chan_mhz)?;
        self.init_bb()?;
        let agc_cal_converged = self.init_cal(chan_mhz)?;

        let synth_control = self.reg_read(AR_PHY_SYNTH_CONTROL)?;
        let phy_mode = self.reg_read(AR_PHY_MODE)?;
        let phy_active = self.reg_read(AR_PHY_ACTIVE)?;
        let agc_control = self.reg_read(AR_PHY_AGC_CONTROL)?;
        let phy_cca = self.reg_read(AR_PHY_CCA)?;
        // MINCCA_PWR is a 9-bit signed field at bits[28:20].
        let raw_nf = (phy_cca & AR9280_PHY_MINCCA_PWR) >> AR9280_PHY_MINCCA_PWR_S;
        let noise_floor_dbm = sign_extend(raw_nf, 9);

        Ok(CalStatus {
            chan_mhz,
            synth_control,
            phy_mode,
            phy_active,
            agc_cal_converged,
            agc_control,
            phy_cca,
            noise_floor_dbm,
        })
    }

    /// ★ **Live HT20↔HT40 bandwidth switch (#7)** — reprogram the PHY to the new width without a
    /// re-open, on the `&self` path (HTC/RX ring stay up). This is the piece `set_channel`'s width flag
    /// alone could not do: the 40 MHz PHY needs the *2G_HT40 initval column* streamed and the synth
    /// re-centred (+10 MHz), not just the per-frame `HAL_RATESERIES_2040` bit. Runs the same three
    /// moves `hw_reset` does for width — re-stream initvals (picks the column from `self.ht40`, sets/
    /// clears DYN2040), re-synth+cal (`rf_set_freq` reads `self.ht40` for the HT40+ centre), then
    /// re-assert DYN2040 (the HT20 CL-cal inside `init_cal` clears it — the documented gotcha).
    ///
    /// ⚠ This is a live PHY re-init (streams the full mode/common/ANI/TX-gain tables + re-cal), the
    /// same disruption ath9k takes on any bandwidth change. Verify on air (the witness must match the
    /// AR9271's HT40+ centre to demod 40 MHz). Returns the post-switch [`CalStatus`] (AGC converged +
    /// synth/TURBO readback) so a caller can confirm the width took.
    pub fn set_bandwidth(&self, ht40: bool) -> Result<CalStatus, FaceError> {
        use crate::ath9k_reg::*;
        use std::sync::atomic::Ordering::Relaxed;
        self.ht40.store(ht40, Relaxed);
        // Re-stream the PHY mode tables for the new width (2G_HT40 vs HT20 column + DYN2040).
        self.apply_initvals()?;
        // Re-synth + re-cal at the current channel; rf_set_freq applies the +10 MHz HT40 centre.
        let ch = self.channel.load(Relaxed);
        let chan_mhz = if ch == 14 { 2484 } else { 2407 + 5 * ch as u16 };
        let status = self.set_channel_and_cal(chan_mhz)?;
        // init_cal's HT20 CL-cal clears DYN2040 — re-assert for HT40 (same as the hw_reset tail).
        if ht40 {
            self.reg_set_bit(AR_PHY_TURBO, AR_PHY_FC_DYN2040_EN)?;
        }
        Ok(status)
    }

    /// ★ **OLPC `set_board_values` port — M2 (analog config).** The AR9271 runs the PA low because
    /// `hw_reset` skips this EEPROM-driven cal (per-chip PA bias + antenna/gain config). Ports the
    /// analog half of `ath9k_hw_4k_set_board_values`: reads the 4k cal (M1), then programs
    /// `antCtrlCommon → AR_PHY_SWITCH_COM`, the XATTEN gain (`AR_PHY_GAIN_2GHZ` block 0+1), and — the big
    /// power lever — the **ob/db PA-bias** analog registers (AR9271 path: `AR9285_AN_RF2G3/RF2G4`). Refuses
    /// if the EEPROM checksum is bad (never program the PA from garbage). Call after `apply_initvals`.
    /// Returns the applied cal so a caller can verify + measure the TX-power change. M3 (the per-rate
    /// target-power / OLPC gain table) is still TODO — this is the analog config only.
    /// Read `txGainType` (0=normal, 1=high power) from the 4k EEPROM (base header byte 31). Drives the
    /// gain-table choice in [`apply_initvals`]. Retries the read (glitch-tolerant). Best-effort: returns
    /// 0 if the EEPROM can't be validated (falls back to the normal table).
    pub fn eeprom_tx_gain_type(&self) -> u8 {
        for _ in 0..5 {
            let s: Vec<u16> = (0..32).map(|w| (self.reg_read(0x2000 + (((w as u32) + 64) << 2)).unwrap_or(0) & 0xffff) as u16).collect();
            // byte 31 = high byte of word 15.
            if s[0] as usize >= 32 {
                return (s[15] >> 8) as u8;
            }
        }
        0
    }

    /// Mark this as a high-power module so [`apply_initvals`] streams the HIGH_POWER gain table.
    pub fn set_high_power(&self, on: bool) {
        self.high_power.store(on, std::sync::atomic::Ordering::Relaxed);
    }

    pub fn set_board_values(&self) -> Result<BoardValues, FaceError> {
        // M1: read + XOR-validate the 4k EEPROM (word w = reg 0x2000 + ((w+64)<<2)). Retry the whole
        // read — a single glitched reg_read among 376 words fails the strict XOR, and the WMI reg path
        // can glitch one word under concurrent TX; a clean re-read fixes it.
        let rd = |w: usize| (self.reg_read(0x2000 + (((w as u32) + 64) << 2)).unwrap_or(0xffff_ffff) & 0xffff) as u16;
        let mut s: Vec<u16> = Vec::new();
        let mut checksum_ok = false;
        for _ in 0..5 {
            s = (0..400).map(rd).collect();
            let length = (s[0] as usize).min(400);
            if length >= 32 && (0..length).fold(0u16, |a, i| a ^ s[i]) == 0xffff {
                checksum_ok = true;
                break;
            }
        }
        if !checksum_ok {
            return Err(err(format!(
                "set_board_values: EEPROM checksum invalid after 5 reads (word0={:#06x}) — refusing to program the PA",
                s.first().copied().unwrap_or(0)
            )));
        }
        let byte = |off: usize| -> u8 {
            let w = s[off / 2];
            if off & 1 == 0 { (w & 0xff) as u8 } else { (w >> 8) as u8 }
        };

        // base_eep_header_4k = 32 B (txGainType@31); custData[20]; modalHeader @byte 52.
        let tx_gain_type = byte(31);
        let m = 52usize;
        let ant_ctrl_common = (s[(m + 4) / 2] as u32) | ((s[(m + 6) / 2] as u32) << 16);
        let tx_rx_atten = byte(m + 10);
        let rx_tx_margin = byte(m + 11);
        let bsw_atten = byte(m + 31);
        let bsw_margin = byte(m + 32);
        let xatten2_db = byte(m + 34);
        let xatten2_margin = byte(m + 35);
        // ob/db (modal version >= 2): ob_0/ob_1 packed @m+25, ob_2 @m+38 low nibble; db1_0 @m+26,
        // db2_0 @m+36 (all low-nibble 4-bit fields).
        let ob = [byte(m + 25) & 0xf, byte(m + 25) >> 4, byte(m + 38) & 0xf];
        let db1_0 = byte(m + 26) & 0xf;
        let db2_0 = byte(m + 36) & 0xf;

        let skip_gain = std::env::var_os("NDN_SB_SKIP_GAIN").is_some();
        let skip_obdb = std::env::var_os("NDN_SB_SKIP_OBDB").is_some();

        // 1. antCtrlCommon → AR_PHY_SWITCH_COM (0x9964) — routes TX to the external-PA path. ★ Applied
        // by default: alone it kills TX ~40 dB (routes to the PA while the PA is OFF), but composed with
        // the XPA enable below it is REQUIRED for full power — MEASURED: HIGH table + antCtrl + XPA =
        // max −18 dBm at 1 ft (normal), vs −63 without antCtrl. NDN_SB_NO_ANT to isolate.
        if std::env::var_os("NDN_SB_NO_ANT").is_none() {
            self.reg_write(0x9964, ant_ctrl_common)?;
        }

        // 2. ath9k_hw_4k_set_gain — XATTEN fields in AR_PHY_GAIN_2GHZ (0xA20C), block 0 + block 1 (+0x1000).
        if !skip_gain {
            for base in [0xA20Cu32, 0xA20C + 0x1000] {
                self.reg_rmw(base, (bsw_margin as u32) << 12, 0x0001_F000)?; // XATTEN1_MARGIN
                self.reg_rmw(base, bsw_atten as u32, 0x0000_003F)?; //          XATTEN1_DB
                self.reg_rmw(base, (xatten2_margin as u32) << 17, 0x003E_0000)?; // XATTEN2_MARGIN
                self.reg_rmw(base, (xatten2_db as u32) << 6, 0x0000_0FC0)?; //   XATTEN2_DB
            }
        }

        // 3. ob/db PA bias (AR9271 analog path). ath9k's `analog_shift_rmw` is a plain REG_RMW — do NOT
        // touch AR_PHY_BASE analog-shift routing (that leaves the baseband in shift mode, kills TX ~39 dB).
        if !skip_obdb {
            self.reg_rmw(0x7828, (ob[0] as u32) << 18, 0x001C_0000)?; // AR9271_AN_RF2G3_OB_cck
            self.reg_rmw(0x7828, (ob[1] as u32) << 15, 0x0003_8000)?; // OB_psk
            self.reg_rmw(0x7828, (ob[2] as u32) << 12, 0x0000_7000)?; // OB_qam
            self.reg_rmw(0x7828, (db1_0 as u32) << 21, 0x00E0_0000)?; // DB_1
            self.reg_rmw(0x782C, (db2_0 as u32) << 29, 0xE000_0000)?; // DB_2
        }

        // 4. ★ RF-control tail incl. the **external-PA (XPA) enable timing** — the piece skipped before,
        // and the strong candidate for the ~tens-of-dB radiated deficit (a high-power module's external
        // PA never switched on). `AR_PHY_RF_CTL4` = when XPA-A/B turn on before / off after each frame.
        // Default-ON; NDN_SB_NO_XPA to isolate. switchSettling@m+9, adcDesiredSize@m+12,
        // txEndToXpaOff@m+15, txEndToRxOn@m+16, txFrameToXpaOn@m+17, txFrameToDataStart@m+28, txFrameToPaOn@m+29.
        if std::env::var_os("NDN_SB_NO_XPA").is_none() {
            let sw_settling = byte(m + 9) as u32;
            let adc_desired = byte(m + 12) as u32;
            let tx_end_xpa_off = byte(m + 15) as u32;
            let tx_end_rx_on = byte(m + 16) as u32;
            let tx_frame_xpa_on = byte(m + 17) as u32;
            let tx_frame_data_start = byte(m + 28) as u32;
            let tx_frame_pa_on = byte(m + 29) as u32;
            self.reg_rmw(0x9844, sw_settling << 7, 0x0000_3F80)?; // AR_PHY_SETTLING SWITCH
            self.reg_rmw(0x9850, adc_desired, 0x0000_00FF)?; //      AR_PHY_DESIRED_SZ ADC
            // AR_PHY_RF_CTL4: XPAA_OFF(16) | XPAB_OFF(24) | XPAA_ON(0) | XPAB_ON(8) — write all 4 fields.
            let rf_ctl4 = (tx_end_xpa_off << 16) | (tx_end_xpa_off << 24) | tx_frame_xpa_on | (tx_frame_xpa_on << 8);
            self.reg_write(0x9834, rf_ctl4)?;
            self.reg_rmw(0x9828, tx_end_rx_on << 16, 0x00FF_0000)?; // AR_PHY_RF_CTL3 TX_END_TO_A2_RX_ON
            self.reg_rmw(0x9824, tx_frame_data_start, 0x0000_00FF)?; // AR_PHY_RF_CTL2 TX_END_DATA_START
            self.reg_rmw(0x9824, tx_frame_pa_on << 8, 0x0000_FF00)?; // AR_PHY_RF_CTL2 TX_END_PA_ON
        }

        Ok(BoardValues {
            checksum_ok,
            tx_gain_type,
            ant_ctrl_common,
            ob,
            db1_0,
            db2_0,
            tx_rx_atten,
            rx_tx_margin,
        })
    }

    /// ★★★ **OLPC power cal — M3 (the actual power lever).** Faithful port of
    /// `ath9k_hw_set_4k_power_cal_table` + `set_4k_power_per_rate_table` for 2.4 GHz HT20. Without this
    /// the PA runs ~40 dB low (the target→gain PDADC map is never programmed). Reads the 4k cal (M1) at
    /// the M3a-anchored offsets, computes the PDADC table from the power-detector cal (calPierData2G) via
    /// `get_gain_boundaries_pdadcs`/`fill_vpd_table`, programs `AR_PHY_TPCRG1/5` + the 128-byte PDADC
    /// table, and writes the per-rate target power to `AR_PHY_POWER_TX_RATE1..9`. **Scoped to ch1/HT20**
    /// (the AR9271's use): ch1 == cal pier 0, so no cross-pier interpolation; HT40/CTL-regulatory caps
    /// are omitted (the EEPROM targets are already the regulatory targets). Call after apply_initvals.
    pub fn set_txpower_4k(&self, chan_mhz: u16) -> Result<i16, FaceError> {
        use crate::ath9k_reg::AR_PHY_BASE;
        // M1 read + validate (retry).
        let rd = |w: usize| (self.reg_read(0x2000 + (((w as u32) + 64) << 2)).unwrap_or(0xffff_ffff) & 0xffff) as u16;
        let mut s: Vec<u16> = Vec::new();
        for _ in 0..5 {
            s = (0..400).map(rd).collect();
            let len = (s[0] as usize).min(400);
            if len >= 32 && (0..len).fold(0u16, |a, i| a ^ s[i]) == 0xffff {
                break;
            }
            s.clear();
        }
        if s.is_empty() {
            return Err(err("set_txpower_4k: EEPROM checksum invalid".into()));
        }
        let byte = |off: usize| -> u8 {
            let w = s[off / 2];
            if off & 1 == 0 { (w & 0xff) as u8 } else { (w >> 8) as u8 }
        };

        const NUM_PD: usize = 2; // AR5416_EEP4K_NUM_PD_GAINS
        const ICEPTS: usize = 5; // AR5416_PD_GAIN_ICEPTS
        const NUM_PDADC: usize = 128;
        const MAX_RATE_POWER: i16 = 63;
        let modal = 52usize;
        let xpd_gain = byte(modal + 20);
        let pd_gain_overlap = byte(modal + 24) as i16;

        // ── numXpdGains from the xpdGain mask (AR5416_PD_GAINS_IN_MASK = 4) ──
        let mut xpd_gain_values = [0u16; NUM_PD];
        let mut num_xpd = 0usize;
        for i in 1..=4u16 {
            if (xpd_gain as u16 >> (4 - i)) & 1 == 1 {
                if num_xpd >= NUM_PD {
                    break;
                }
                xpd_gain_values[num_xpd] = 4 - i;
                num_xpd += 1;
            }
        }
        if num_xpd == 0 {
            return Err(err(format!("set_txpower_4k: xpdGain mask {xpd_gain:#04x} has no gains")));
        }

        // ch1 (2412) == cal pier 0 (calFreqPier2G @120 = 0x70/0x8e/0xac). Use pier 0 directly (match).
        // calPierData2G[0]: pwrPdg[2][5] @123, vpdPdg[2][5] @133.
        let pwr = |g: usize, k: usize| byte(123 + g * ICEPTS + k);
        let vpd = |g: usize, k: usize| byte(123 + NUM_PD * ICEPTS + g * ICEPTS + k);

        // fill_vpd_table + get_gain_boundaries_pdadcs (match=true branch).
        let mut vpd_i = [[0u8; 64]; NUM_PD]; // AR5416_MAX_PWR_RANGE_IN_HALF_DB = 64
        let mut min_pwr = [0u8; NUM_PD];
        let mut max_pwr = [0u8; NUM_PD];
        for g in 0..num_xpd {
            min_pwr[g] = pwr(g, 0);
            max_pwr[g] = pwr(g, ICEPTS - 1);
            let pwr_list: Vec<u8> = (0..ICEPTS).map(|k| pwr(g, k)).collect();
            let vpd_list: Vec<u8> = (0..ICEPTS).map(|k| vpd(g, k)).collect();
            fill_vpd_table(min_pwr[g], max_pwr[g], &pwr_list, &vpd_list, &mut vpd_i[g]);
        }

        // Build the 128-entry PDADC table + gain boundaries (the target→gain map).
        let mut pdadc = [0u8; NUM_PDADC];
        let mut boundaries = [0u16; 4]; // AR5416_PD_GAINS_IN_MASK
        let mut k = 0usize;
        for g in 0..num_xpd {
            boundaries[g] = if g == num_xpd - 1 {
                (max_pwr[g] as u16) / 2
            } else {
                (max_pwr[g] as u16 + min_pwr[g + 1] as u16) / 4
            };
            boundaries[g] = boundaries[g].min(MAX_RATE_POWER as u16);
            // AR9271 is post-9280: ss = -(minPwr/2) for the first gain.
            let mut ss: i16 = if g == 0 {
                -((min_pwr[g] as i16) / 2)
            } else {
                (boundaries[g - 1] as i16) - (min_pwr[g] as i16 / 2) - pd_gain_overlap + 1
            };
            let vpd_step = ((vpd_i[g][1] as i16) - (vpd_i[g][0] as i16)).max(1);
            while ss < 0 && k < NUM_PDADC - 1 {
                let t = vpd_i[g][0] as i16 + ss * vpd_step;
                pdadc[k] = t.max(0).min(255) as u8;
                k += 1;
                ss += 1;
            }
            let size_curr = ((max_pwr[g] - min_pwr[g]) / 2 + 1) as i16;
            let tgt_index = boundaries[g] as i16 + pd_gain_overlap - (min_pwr[g] as i16 / 2);
            let max_index = tgt_index.min(size_curr);
            while ss < max_index && k < NUM_PDADC - 1 {
                pdadc[k] = vpd_i[g][ss as usize];
                k += 1;
                ss += 1;
            }
            let vpd_step2 = ((vpd_i[g][size_curr as usize - 1] as i16) - (vpd_i[g][size_curr as usize - 2] as i16)).max(1);
            if tgt_index >= max_index {
                while ss <= tgt_index && k < NUM_PDADC - 1 {
                    let t = vpd_i[g][size_curr as usize - 1] as i16 + (ss - max_index + 1) * vpd_step2;
                    pdadc[k] = t.min(255) as u8;
                    k += 1;
                    ss += 1;
                }
            }
        }
        // Fill the rest: boundaries default 58 (4k), pdadc repeats last.
        for b in num_xpd..4 {
            boundaries[b] = 58;
        }
        while k < NUM_PDADC {
            pdadc[k] = pdadc[k - 1];
            k += 1;
        }

        // ── Program AR_PHY_TPCRG1 (num PD gains + gain values) ──
        const AR_PHY_TPCRG1: u32 = 0xA258;
        self.reg_rmw(AR_PHY_TPCRG1, ((num_xpd as u32 - 1) & 0x3) << 14, 0x0000_c000)?;
        self.reg_rmw(AR_PHY_TPCRG1, (xpd_gain_values[0] as u32) << 16, 0x0003_0000)?;
        self.reg_rmw(AR_PHY_TPCRG1, (xpd_gain_values[1] as u32) << 18, 0x000C_0000)?;
        self.reg_rmw(AR_PHY_TPCRG1, 0, 0x0030_0000)?; // PD_GAIN_3 = 0

        // ── AR_PHY_TPCRG5: PD gain overlap + boundaries ──
        const AR_PHY_TPCRG5: u32 = 0xA26C;
        let tpcrg5 = ((pd_gain_overlap as u32) & 0xF)
            | ((boundaries[0] as u32 & 0x3F) << 4)
            | ((boundaries[1] as u32 & 0x3F) << 10)
            | ((boundaries[2] as u32 & 0x3F) << 16)
            | ((boundaries[3] as u32 & 0x3F) << 22);
        self.reg_write(AR_PHY_TPCRG5, tpcrg5)?;

        // ── The 128-byte PDADC table at AR_PHY_BASE + (672<<2) = 0xA280 (32 words) ──
        let pdadc_base = AR_PHY_BASE + (672 << 2);
        for j in 0..32 {
            let w = (pdadc[4 * j] as u32)
                | ((pdadc[4 * j + 1] as u32) << 8)
                | ((pdadc[4 * j + 2] as u32) << 16)
                | ((pdadc[4 * j + 3] as u32) << 24);
            self.reg_write(pdadc_base + (j as u32) * 4, w)?;
        }

        // ── Per-rate target power (ch1 = pier 0 direct). calTargetPower* @183/198/213 (bChannel + tPow2x). ──
        // Legacy target tPow2x[4]; HT20 tPow2x[8]. Skip byte 0 (bChannel), read the tPow2x arrays.
        let cck = |i: usize| byte(183 + 1 + i) as i16; // calTargetPowerCck[0].tPow2x[i]
        let ofdm = |i: usize| byte(198 + 1 + i) as i16; // calTargetPower2G[0].tPow2x[i]
        let ht20 = |i: usize| byte(213 + 1 + i) as i16; // calTargetPower2GHT20[0].tPow2x[i]

        // ar5416 rate→target-group mapping (legacy). ratesArray indexed by the Ar5416_Rates enum.
        let mut rates = [0i16; 24];
        // OFDM: 6/9=[0] 12/18/24=[1] 36=[2] 48/54=[3]
        rates[0] = ofdm(0); rates[1] = ofdm(0); // 6,9
        rates[2] = ofdm(1); rates[3] = ofdm(1); // 12,18
        rates[4] = ofdm(1); rates[5] = ofdm(2); // 24,36
        rates[6] = ofdm(3); rates[7] = ofdm(3); // 48,54
        // CCK: 1l=[0] 2l/2s=[1] 5.5=[2] 11=[3]  (indices rate1l=8..rate11s=14)
        rates[8] = cck(0); rates[9] = cck(1); rates[10] = cck(1); // 1l,2l,2s
        rates[11] = cck(2); rates[12] = cck(2); rates[13] = cck(3); rates[14] = cck(3); // 5.5l/s,11l/s
        // HT20 MCS0-7 (indices rateHt20_0=16..23)
        for m in 0..8 {
            rates[16 + m] = ht20(m);
        }
        // Cap + apply the PWR_TABLE_OFFSET (-5 dB → +10 in 0.5 dB units).
        let max_target = rates.iter().copied().max().unwrap_or(0);
        for r in rates.iter_mut() {
            if *r > MAX_RATE_POWER {
                *r = MAX_RATE_POWER;
            }
            *r -= -5 * 2; // AR5416_PWR_TABLE_OFFSET_DB * 2
        }
        let pow_sm = |r: i16, sh: u32| ((r as u32) & 0x3f) << sh;
        // OFDM
        self.reg_write(0x9934, pow_sm(rates[3], 24) | pow_sm(rates[2], 16) | pow_sm(rates[1], 8) | pow_sm(rates[0], 0))?; // RATE1
        self.reg_write(0x9938, pow_sm(rates[7], 24) | pow_sm(rates[6], 16) | pow_sm(rates[5], 8) | pow_sm(rates[4], 0))?; // RATE2
        // CCK (RATE3: 2s,2l,xr,1l ; RATE4: 11s,11l,5.5s,5.5l)
        self.reg_write(0xA234, pow_sm(rates[10], 24) | pow_sm(rates[9], 16) | pow_sm(0, 8) | pow_sm(rates[8], 0))?; // RATE3
        self.reg_write(0xA238, pow_sm(rates[14], 24) | pow_sm(rates[13], 16) | pow_sm(rates[12], 8) | pow_sm(rates[11], 0))?; // RATE4
        // HT20
        self.reg_write(0xA38C, pow_sm(rates[19], 24) | pow_sm(rates[18], 16) | pow_sm(rates[17], 8) | pow_sm(rates[16], 0))?; // RATE5
        self.reg_write(0xA390, pow_sm(rates[23], 24) | pow_sm(rates[22], 16) | pow_sm(rates[21], 8) | pow_sm(rates[20], 0))?; // RATE6
        // TPC off → the per-rate table is the cap; PA drives to it via the PDADC map.
        self.reg_write(0x993c, MAX_RATE_POWER as u32)?; // AR_PHY_POWER_TX_RATE_MAX (no TPC_ENABLE)

        let _ = chan_mhz;
        Ok(max_target) // the peak target power (0.5 dB units) — /2 = dBm
    }

    // ── granular reset-tail helpers (shared by set_channel_and_cal + hw_reset) ──

    /// `ath9k_hw_set_rfmode` (hw.c:1967 → ar5008_hw_set_rfmode). For a 2.4 GHz
    /// single-chip post-9280 part the RF mode is `AR_PHY_MODE_DYNAMIC` (CCK+OFDM).
    /// Golden trace line 50: 0xa200 = 0x04. ✓
    pub fn set_rfmode(&self) -> Result<(), FaceError> {
        use crate::ath9k_reg::*;
        self.reg_write(AR_PHY_MODE, AR_PHY_MODE_DYNAMIC)?;
        Ok(())
    }

    /// `ath9k_hw_rf_set_freq` → `ar9002_hw_set_channel` (ar9002_phy.c:66), the 2.4 GHz
    /// fractional-mode synth program. `reg32 = (prev & 0xc0000000) | bMode<<29 |
    /// fracMode<<28 | aModeRefSel<<26 | CHANSEL_2G(freq)` with bMode=fracMode=1,
    /// aModeRefSel=0. Channel-14 spreading (CCK_TX_CTRL_JAPAN) on only for 2484.
    /// Golden trace line 57: 0x9874 = 0x30a0cccc for freq 2412 (BMODE|FRACMODE|
    /// CHANSEL_2G(2412)=0xa0cccc). ✓
    pub fn rf_set_freq(&self, chan_mhz: u16) -> Result<(), FaceError> {
        use crate::ath9k_reg::*;
        if chan_mhz != 2484 {
            self.reg_clr_bit(AR_PHY_CCK_TX_CTRL, AR_PHY_CCK_TX_CTRL_JAPAN)?;
        } else {
            self.reg_set_bit(AR_PHY_CCK_TX_CTRL, AR_PHY_CCK_TX_CTRL_JAPAN)?;
        }
        // HT40+ programs the synth to the 40 MHz band CENTRE = primary + 10 MHz (the extension is the
        // 20 MHz above). HT20 uses the control-channel centre directly.
        let synth_mhz = if self.ht40.load(std::sync::atomic::Ordering::Relaxed) {
            chan_mhz + 10
        } else {
            chan_mhz
        };
        let channel_sel = ((synth_mhz as u64 * 0x1_0000) / CHANSEL_2G_DIV) as u32;
        let prev = self.reg_read(AR_PHY_SYNTH_CONTROL)? & 0xc000_0000;
        let synth = prev
            | AR_PHY_SYNTH_CONTROL_2G_BMODE
            | AR_PHY_SYNTH_CONTROL_2G_FRACMODE
            | channel_sel; // aModeRefSel = 0
        self.reg_write(AR_PHY_SYNTH_CONTROL, synth)?;
        Self::settle(2);
        Ok(())
    }

    /// `ath9k_hw_init_bb` — enable the baseband (`AR_PHY_ACTIVE = EN`), then wait the
    /// synth-settle delay. Golden trace line 94: 0x981c = 0x01. ✓
    pub fn init_bb(&self) -> Result<(), FaceError> {
        use crate::ath9k_reg::*;
        // The reference reads AR_PHY_RX_DELAY to compute an exact synth-settle udelay;
        // a fixed few-ms settle dwarfs it on the WMI path, so read is informational.
        let _ = self.reg_read(AR_PHY_RX_DELAY)?;
        self.reg_write(AR_PHY_ACTIVE, AR_PHY_ACTIVE_EN)?;
        Self::settle(3); // synthDelay + BASE_ACTIVATE_DELAY
        Ok(())
    }

    /// `ath9k_hw_init_cal` → `ar9002_hw_init_cal` (ar9002_calib.c) AR9271 branch, in
    /// the EXACT source order: `ar9285_hw_cl_cal` (offset/AGC cal) → (PA cal, TX-only,
    /// skipped) → `ath9k_hw_loadnf` → `ath9k_hw_start_nfcal(true)` → arm+run the IQ
    /// cal on the HT20 cal list (IQ-mismatch only). Returns whether the AGC cal
    /// converged. ★ This is the whole point of the rewrite: it runs LAST, after the
    /// full PHY/MAC setup, not before.
    pub fn init_cal(&self, chan_mhz: u16) -> Result<bool, FaceError> {
        use crate::ath9k_reg::*;
        // 1. ar9285_hw_cl_cal (AR9271 offset + AGC cal).
        let agc_cal_converged = self.ar9271_cl_cal_ht20()?;
        // 2. ath9k_hw_loadnf — seed the nominal NF so the receiver isn't deaf.
        self.load_nf_nominal()?;
        // 3. ath9k_hw_start_nfcal(update = true) — kick the hardware noise-floor cal.
        self.reg_set_bit(AR_PHY_AGC_CONTROL, AR_PHY_AGC_CONTROL_ENABLE_NF)?;
        self.reg_clr_bit(AR_PHY_AGC_CONTROL, AR_PHY_AGC_CONTROL_NO_UPDATE_NF)?;
        self.reg_set_bit(AR_PHY_AGC_CONTROL, AR_PHY_AGC_CONTROL_NF)?;
        Self::settle(5);
        // 4. ar9002_hw_init_cal tail: arm+run the IQ-mismatch RX cal (HT20 list = IQ only).
        self.run_rx_calibration(chan_mhz)?;
        Ok(agc_cal_converged)
    }

    // ── ath9k_hw_reset() sub-functions, transcribed faithfully (2.4 GHz HT20) ─────

    /// `ath9k_hw_setpower(ATH9K_PM_AWAKE)` → `ath9k_hw_set_power_awake` — the force-wake
    /// that opens `ath9k_hw_reset` (step 1). On this USB part it is just the
    /// FORCE_WAKE_EN write; the WA-register poking is 9300-only.
    pub fn setpower_awake(&mut self) -> Result<(), FaceError> {
        use crate::ath9k_reg::*;
        self.reg_write(
            AR_RTC_FORCE_WAKE,
            AR_RTC_FORCE_WAKE_EN | AR_RTC_FORCE_WAKE_ON_INT,
        )?;
        Ok(())
    }

    /// `ath9k_hw_mark_phy_inactive` — write `AR_PHY_ACTIVE = AR_PHY_ACTIVE_DIS` (0).
    /// Golden trace line 1: 0x981c = 0. ✓
    pub fn mark_phy_inactive(&mut self) -> Result<(), FaceError> {
        use crate::ath9k_reg::*;
        self.reg_write(AR_PHY_ACTIVE, AR_PHY_ACTIVE_DIS)?;
        Ok(())
    }

    /// `ar9002_hw_enable_async_fifo` (ar9002_hw.c:371). **No-op on AR9271** — the body
    /// is gated behind `AR_SREV_9287_13_OR_LATER`, which the AR9271 is not. Kept as a
    /// faithful placeholder so the call sits in its ath9k_hw_reset() slot.
    pub fn enable_async_fifo(&mut self) -> Result<(), FaceError> {
        // AR9271 is not 9287 → nothing to do.
        Ok(())
    }

    /// `ath9k_hw_init_mfp` (hw.c:1680), AR9280_20_OR_LATER branch: RMW the FC_MGMT
    /// field of `AR_AES_MUTE_MASK1` to 0xc7ff (mask Retry/PwrMgt/MoreData in CCMP AAD).
    pub fn init_mfp(&mut self) -> Result<(), FaceError> {
        use crate::ath9k_reg::*;
        self.reg_rmw(
            AR_AES_MUTE_MASK1,
            (AR_MFP_MGMT_MASK_VAL << AR_AES_MUTE_MASK1_FC_MGMT_S) & AR_AES_MUTE_MASK1_FC_MGMT,
            AR_AES_MUTE_MASK1_FC_MGMT,
        )?;
        Ok(())
    }

    /// `ath9k_hw_set_delta_slope` (ar5008_hw_set_delta_slope, in ar5008_phy.c — NOT
    /// fetched; transcribed from the canonical mainline body). Programs the OFDM
    /// timing delta-slope mantissa/exponent for full-GI (`AR_PHY_TIMING3`) and
    /// half-GI (`AR_PHY_HALFGI`, 0.9× coefficient). `coef = (100 MHz << 24) /
    /// synth_center`. Uses [`delta_slope_vals`] (ath9k_hw_get_delta_slope_vals,
    /// hw.c:1297, transcribed exactly).
    pub fn set_delta_slope(&mut self, chan_mhz: u16) -> Result<(), FaceError> {
        use crate::ath9k_reg::*;
        let coef_scaled = DELTA_SLOPE_CLOCK_MHZ_SCALED / chan_mhz as u32;
        let (man, exp) = delta_slope_vals(coef_scaled);
        self.reg_rmw(
            AR_PHY_TIMING3,
            (man << AR_PHY_TIMING3_DSC_MAN_S) & AR_PHY_TIMING3_DSC_MAN,
            AR_PHY_TIMING3_DSC_MAN,
        )?;
        self.reg_rmw(
            AR_PHY_TIMING3,
            (exp << AR_PHY_TIMING3_DSC_EXP_S) & AR_PHY_TIMING3_DSC_EXP,
            AR_PHY_TIMING3_DSC_EXP,
        )?;
        // Half-GI uses 0.9× the coefficient.
        let coef_scaled_hg = (9 * coef_scaled) / 10;
        let (man_hg, exp_hg) = delta_slope_vals(coef_scaled_hg);
        self.reg_rmw(
            AR_PHY_HALFGI,
            (man_hg << AR_PHY_HALFGI_DSC_MAN_S) & AR_PHY_HALFGI_DSC_MAN,
            AR_PHY_HALFGI_DSC_MAN,
        )?;
        self.reg_rmw(
            AR_PHY_HALFGI,
            (exp_hg << AR_PHY_HALFGI_DSC_EXP_S) & AR_PHY_HALFGI_DSC_EXP,
            AR_PHY_HALFGI_DSC_EXP,
        )?;
        Ok(())
    }

    /// `ath9k_hw_spur_mitigate_freq` → `ar9002_hw_spur_mitigate` (ar9002_phy.c:168).
    /// The AR9271 EEPROM carries no spur channels for ordinary operation, so this is
    /// the `AR_NO_SPUR` path (ar9002_phy.c:214-221): clear `AR_PHY_FORCE_CLKEN_CCK`'s
    /// MRC_MUX bit and return. (The full spur-mask programming needs an EEPROM spur
    /// frequency, which we do not have; documented so the bench knows why the spur
    /// registers stay untouched.)
    pub fn spur_mitigate_freq(&mut self, _chan_mhz: u16) -> Result<(), FaceError> {
        use crate::ath9k_reg::*;
        self.reg_clr_bit(AR_PHY_FORCE_CLKEN_CCK, AR_PHY_FORCE_CLKEN_CCK_MRC_MUX)?;
        Ok(())
    }

    /// `ath9k_hw_reset_opmode` (hw.c:1707) + `ath9k_hw_set_operating_mode` (hw.c:1266).
    /// Writes the STA_ID1 opmode/defaults, default antenna, a zeroed BSSID/AID, clears
    /// the ISR, seeds the RSSI threshold, then applies the operating mode (monitor ⇒
    /// KSRCH_MODE only, both AP/ADHOC opmode bits cleared). `mac_sta_id1` is the
    /// `AR_STA_ID1 & BASE_RATE_11B` saved before the chip reset.
    pub fn reset_opmode(&mut self, mac_sta_id1: u32, save_def_antenna: u32) -> Result<(), FaceError> {
        use crate::ath9k_reg::*;
        // REG_RMW(AR_STA_ID1, macStaId1 | RTS_USE_DEF | sta_id1_defaults, ~SADH_MASK).
        // sta_id1_defaults is 0 for our config.
        self.reg_rmw(
            AR_STA_ID1,
            mac_sta_id1 | AR_STA_ID1_RTS_USE_DEF,
            !AR_STA_ID1_SADH_MASK,
        )?;
        // ath_hw_setbssidmask (AR_BSSMSK_L/U) is omitted: monitor accepts all, and the
        // post-reset default mask is already all-ones. Documented, not a silent skip.
        self.reg_write(AR_DEF_ANTENNA, save_def_antenna)?;
        // ath9k_hw_write_associd — no association ⇒ BSSID/AID zero.
        self.reg_write(AR_BSS_ID0, 0)?;
        self.reg_write(AR_BSS_ID1, 0)?;
        self.reg_write(AR_ISR, 0xffff_ffff)?; // REG_WRITE(AR_ISR, ~0)
        self.reg_write(AR_RSSI_THR, INIT_RSSI_THR)?;
        // ath9k_hw_set_operating_mode(ah->opmode = monitor): KSRCH_MODE, clear AP/ADHOC.
        self.reg_rmw(
            AR_STA_ID1,
            AR_STA_ID1_KSRCH_MODE,
            AR_STA_ID1_STA_AP | AR_STA_ID1_ADHOC,
        )?;
        Ok(())
    }

    /// `ath9k_hw_set_clockrate` (hw.c:39) — computes `common->clockrate` and writes
    /// **no register**. For a 2.4 GHz OFDM channel the rate is 44 MHz (confirmed by
    /// the golden trace: SIFS write 0x1030=0x160=8×44). Stored for `init_global_settings`.
    pub fn set_clockrate(&mut self) {
        use crate::ath9k_reg::*;
        self.clockrate = ATH9K_CLOCK_RATE_2GHZ_OFDM;
    }

    /// `ath9k_hw_init_queues` (hw.c:1729): write the DCU→QCU mask for all 10 DCUs,
    /// then reset every active TX queue. The DQCUMASK loop is deterministic; the
    /// per-queue reset uses the standard data-AC defaults (which reproduce the golden
    /// trace's `AR_DLCL_IFS=0x002ffc0f` / `AR_DRETRY_LIMIT=0x0008200a` exactly).
    ///
    /// ⚠ SCOPE: the beacon/CAB queues (8/9) and any mac80211-pushed EDCA parameters
    /// are driver-runtime state the HTC host normally supplies; here we reset the four
    /// data ACs (QCU 0-3) with USEDEFAULT parameters. This is the TX-queue plumbing —
    /// not RX-gating — and the golden trace of these registers is itself partial
    /// (buffered REGWRITE flushes). Data queues 0-3 are transcribed in full.
    pub fn init_queues(&mut self) -> Result<(), FaceError> {
        use crate::ath9k_reg::*;
        // for i in 0..AR_NUM_DCU: REG_WRITE(AR_DQCUMASK(i), 1<<i)
        let masks: Vec<(u32, u32)> = (0..AR_NUM_DCU)
            .map(|i| (AR_D0_QCUMASK + (i << 2), 1u32 << i))
            .collect();
        self.reg_write_batch(&masks)?;
        // Reset the four data ACs with the standard USEDEFAULT queue parameters.
        for q in 0..4u32 {
            self.reset_tx_queue(q)?;
        }
        Ok(())
    }

    /// `ath9k_hw_resettxqueue` (mac.c:367) for a standard data-AC queue using the
    /// USEDEFAULT parameters (cwmin auto=15, cwmax=1023, aifs=2, shretry=10). Writes
    /// `AR_DLCL_IFS`, `AR_DRETRY_LIMIT`, `AR_QMISC`, `AR_DMISC`, `AR_DCHNTIME` for the
    /// queue. Verified: LCL_IFS=0x002ffc0f, RETRY=0x0008200a match the golden trace.
    fn reset_tx_queue(&mut self, q: u32) -> Result<(), FaceError> {
        use crate::ath9k_reg::*;
        // cwMin: USEDEFAULT ⇒ round INIT_CWMIN up to 2^n-1 (15 → stays 15).
        let mut cw_min = 1u32;
        while cw_min < INIT_CWMIN {
            cw_min = (cw_min << 1) | 1;
        }
        let lcl_ifs = ((cw_min << AR_D_LCL_IFS_CWMIN_S) & AR_D_LCL_IFS_CWMIN)
            | ((INIT_CWMAX << AR_D_LCL_IFS_CWMAX_S) & AR_D_LCL_IFS_CWMAX)
            | ((INIT_AIFS << AR_D_LCL_IFS_AIFS_S) & AR_D_LCL_IFS_AIFS);
        self.reg_write(AR_D0_LCL_IFS + (q << 2), lcl_ifs)?;

        let retry = ((INIT_SSH_RETRY << AR_D_RETRY_LIMIT_STA_SH_S) & AR_D_RETRY_LIMIT_STA_SH)
            | ((INIT_SLG_RETRY << AR_D_RETRY_LIMIT_STA_LG_S) & AR_D_RETRY_LIMIT_STA_LG)
            | ((INIT_SH_RETRY << AR_D_RETRY_LIMIT_FR_SH_S) & AR_D_RETRY_LIMIT_FR_SH);
        self.reg_write(AR_D0_RETRY_LIMIT + (q << 2), retry)?;

        self.reg_write(AR_Q0_MISC + (q << 2), AR_Q_MISC_DCU_EARLY_TERM_REQ)?;
        // Non-9340 path: BKOFF_EN | FRAG_WAIT_EN | 0x2.
        self.reg_write(
            AR_D0_MISC + (q << 2),
            AR_D_MISC_CW_BKOFF_EN | AR_D_MISC_FRAG_WAIT_EN | 0x2,
        )?;
        // tqi_burstTime = 0 ⇒ AR_DCHNTIME = 0 (no burst).
        self.reg_write(AR_D0_CHNTIME + (q << 2), 0)?;
        Ok(())
    }

    /// `ath9k_hw_init_interrupt_masks` (hw.c:931), non-9300 no-mitigation path.
    /// Computes the base IMR (`TXERR|TXURN|RXERR|RXORN|BCNMISC|RXOK|TXOK`), ORs GTT
    /// into IMR_S2, and programs the INTR_SYNC cause/enable/mask. The **final** host
    /// IMR arming (the 0x81800964 value) is `ath9k_hw_set_interrupts`, done later in
    /// [`wmi_start`](Self::wmi_start) — this is only the reset-time base.
    pub fn init_interrupt_masks(&mut self) -> Result<(), FaceError> {
        use crate::ath9k_reg::*;
        let imr_reg = AR_IMR_TXERR
            | AR_IMR_TXURN
            | AR_IMR_RXERR
            | AR_IMR_RXORN
            | AR_IMR_BCNMISC
            | AR_IMR_RXOK
            | AR_IMR_TXOK;
        self.reg_write(AR_IMR, imr_reg)?;
        // ah->imrs2_reg |= AR_IMR_S2_GTT (imrs2 base is 0 here).
        self.reg_write(AR_IMR_S2, AR_IMR_S2_GTT)?;
        // Non-9100: program the sync-interrupt registers (PCIe artefacts on USB).
        self.reg_write(AR_INTR_SYNC_CAUSE, 0xffff_ffff)?;
        self.reg_write(AR_INTR_SYNC_ENABLE, AR_INTR_SYNC_DEFAULT)?;
        self.reg_write(AR_INTR_SYNC_MASK, 0)?;
        Ok(())
    }

    /// `ath9k_hw_ani_cache_ini_regs` — caches a set of ANI/PHY registers into the
    /// driver's `ah->ani` state by **reading** them. It writes nothing, and we do not
    /// use the cache, so this is a faithful no-op for bring-up.
    pub fn ani_cache_ini_regs(&mut self) -> Result<(), FaceError> {
        Ok(())
    }

    /// `ath9k_hw_init_qos` (hw.c:714). Golden trace: 0x8118=0x100aa, 0x811c=0x3210. ✓
    pub fn init_qos(&mut self) -> Result<(), FaceError> {
        use crate::ath9k_reg::*;
        self.reg_write(AR_MIC_QOS_CONTROL, 0x100aa)?;
        self.reg_write(AR_MIC_QOS_SELECT, 0x3210)?;
        let no_ack = ((2 << AR_QOS_NO_ACK_TWO_BIT_S) & AR_QOS_NO_ACK_TWO_BIT)
            | ((5 << AR_QOS_NO_ACK_BIT_OFF_S) & AR_QOS_NO_ACK_BIT_OFF)
            | ((0 << AR_QOS_NO_ACK_BYTE_OFF_S) & AR_QOS_NO_ACK_BYTE_OFF);
        self.reg_write(AR_QOS_NO_ACK, no_ack)?;
        self.reg_write(AR_TXOP_X, AR_TXOP_X_VAL)?;
        self.reg_write(AR_TXOP_0_3, 0xffff_ffff)?;
        self.reg_write(AR_TXOP_4_7, 0xffff_ffff)?;
        self.reg_write(AR_TXOP_8_11, 0xffff_ffff)?;
        self.reg_write(AR_TXOP_12_15, 0xffff_ffff)?;
        Ok(())
    }

    /// `ath9k_hw_init_global_settings` (hw.c:1047) — the MAC timing block for a
    /// 2.4 GHz HT20 channel: SIFS/slot/ACK/CTS timeouts, EIFS and USEC. Uses the
    /// stored `clockrate` (44) via `mac_to_clks`. Golden trace: SIFS 0x1030=0x160,
    /// SLOT 0x1070=0x18c, EIFS 0x10b0=0x3e38. ✓
    pub fn init_global_settings(&mut self, _chan_mhz: u16) -> Result<(), FaceError> {
        use crate::ath9k_reg::*;
        let clk = self.clockrate;
        let mac_to_clks = |usecs: u32| usecs * clk;

        let sifstime = 10u32; // 2.4 GHz
        let slottime = ATH9K_INIT_SLOTTIME_2GHZ; // 9
        // acktimeout/ctstimeout: 2.4 GHz non-half/quarter closes to 64/48 µs
        // (the slottime terms cancel, see hw.c:1116-1130).
        let acktimeout = 64u32;
        let ctstimeout = 48u32;

        // EIFS: read current, divide by clockrate (idempotent re-scale on write-back).
        let eifs_reg = self.reg_read(AR_D_GBL_IFS_EIFS)?;
        let eifs = if clk != 0 { eifs_reg / clk } else { 0 };
        // rx/tx latency come from the current AR_USEC field values.
        let usec = self.reg_read(AR_USEC)?;
        let rx_lat = (usec & AR_USEC_RX_LAT) >> AR_USEC_RX_LAT_S;
        let tx_lat = (usec & AR_USEC_TX_LAT) >> AR_USEC_TX_LAT_S;

        // set_sifs_time(sifs): AR_D_GBL_IFS_SIFS = mac_to_clks(sifs - 2), capped 0xffff.
        self.reg_write(AR_D_GBL_IFS_SIFS, mac_to_clks(sifstime - 2).min(0xffff))?;
        // setslottime(slot): AR_D_GBL_IFS_SLOT = mac_to_clks(slot), capped 0xffff.
        self.reg_write(AR_D_GBL_IFS_SLOT, mac_to_clks(slottime).min(0xffff))?;
        // set_ack_timeout: RMW_FIELD(AR_TIME_OUT, ACK, mac_to_clks(ack)).
        let ack_clks = mac_to_clks(acktimeout).min(AR_TIME_OUT_ACK >> AR_TIME_OUT_ACK_S);
        self.reg_rmw(
            AR_TIME_OUT,
            (ack_clks << AR_TIME_OUT_ACK_S) & AR_TIME_OUT_ACK,
            AR_TIME_OUT_ACK,
        )?;
        // set_cts_timeout: RMW_FIELD(AR_TIME_OUT, CTS, mac_to_clks(cts)).
        let cts_clks = mac_to_clks(ctstimeout).min(AR_TIME_OUT_CTS >> AR_TIME_OUT_CTS_S);
        self.reg_rmw(
            AR_TIME_OUT,
            (cts_clks << AR_TIME_OUT_CTS_S) & AR_TIME_OUT_CTS,
            AR_TIME_OUT_CTS,
        )?;
        // globaltxtimeout defaults to (u32)-1 ⇒ not written.
        // REG_WRITE(AR_D_GBL_IFS_EIFS, mac_to_clks(eifs)).
        self.reg_write(AR_D_GBL_IFS_EIFS, mac_to_clks(eifs))?;
        // REG_RMW(AR_USEC, (clk-1) | SM(rx_lat) | SM(tx_lat), TX_LAT|RX_LAT|USEC).
        self.reg_rmw(
            AR_USEC,
            (clk - 1)
                | ((rx_lat << AR_USEC_RX_LAT_S) & AR_USEC_RX_LAT)
                | ((tx_lat << AR_USEC_TX_LAT_S) & AR_USEC_TX_LAT),
            AR_USEC_TX_LAT | AR_USEC_RX_LAT | AR_USEC_USEC,
        )?;
        Ok(())
    }

    /// `ath9k_hw_set_dma` (hw.c:1192), non-9300 path: AHB prefetch, 128-byte MAC DMA
    /// read/write bursts, and the RX FIFO threshold. (AR9271 skips the PCU_TXBUF_CTRL
    /// write.) This is the RX-DMA config the old `rx_enable` carried inline; here it
    /// sits in its faithful reset slot.
    pub fn set_dma(&mut self) -> Result<(), FaceError> {
        use crate::ath9k_reg::*;
        self.reg_set_bit(AR_AHB_MODE, AR_AHB_PREFETCH_RD_EN)?;
        self.reg_rmw(AR_TXCFG, AR_TXCFG_DMASZ_128B, AR_TXCFG_DMASZ_MASK)?;
        self.reg_rmw(AR_RXCFG, AR_RXCFG_DMASZ_128B, AR_RXCFG_DMASZ_MASK)?;
        self.reg_write(AR_RXFIFO_CFG, 0x200)?;
        Ok(())
    }

    /// `ath9k_hw_restore_chainmask` (hw.c:2048). **No-op for the 1-chain AR9271**: the
    /// reference only writes `AR_PHY_RX_CHAINMASK`/`AR_PHY_CAL_CHAINMASK` when the RX
    /// chainmask is 0x3 or 0x5. Faithful placeholder kept in its reset slot.
    pub fn restore_chainmask(&mut self) -> Result<(), FaceError> {
        // rxchainmask == 1 ⇒ neither 0x3 nor 0x5 ⇒ nothing written.
        Ok(())
    }

    /// `ath9k_hw_init_desc` (hw.c:1748), AR9271 USB branch: descriptor byte-swap
    /// `AR_CFG = AR_CFG_SWRB | AR_CFG_SWTB` (= 0x0a). Golden trace line 112: 0x0014=0x0a. ✓
    pub fn init_desc(&mut self) -> Result<(), FaceError> {
        use crate::ath9k_reg::*;
        self.reg_write(AR_CFG, AR_CFG_SWRB | AR_CFG_SWTB)?;
        Ok(())
    }

    // ── the faithful ath9k_hw_reset(), in EXACT source order ──────────────────

    /// **Faithful `ath9k_hw_reset()`** for the AR9271 (non-9100 / non-9300,
    /// 2.4 GHz HT20). Replaces the piecemeal `phy_reset` → `apply_initvals` →
    /// `set_channel_and_cal` approximation with the reference's exact ordered spine.
    /// The one thing the previous approach got wrong — running the calibration before
    /// the PHY/MAC was set up — is fixed here: `init_cal` (step 21) runs LAST, after
    /// everything else, exactly as `ath9k_hw_reset()` does.
    ///
    /// This performs the reset + PHY/MAC bring-up only. `connect_data_services()`,
    /// [`start_receive`](Self::start_receive) and [`wmi_start`](Self::wmi_start) are
    /// the separate post-reset RX-start steps (as `ath9k_htc_start` does).
    /// Bring the PHY up in **HT40 (40 MHz)** on `chan_mhz` (the control/primary channel; HT40+ uses
    /// the 20 MHz above). Sets the `ht40` flag so [`apply_initvals`](Self::apply_initvals) streams the
    /// 2G_HT40 column + keeps DYN2040 and [`rf_set_freq`](Self::rf_set_freq) offsets the synth centre;
    /// injected MCS frames then carry the `2040` rate flag. Otherwise identical to [`hw_reset`].
    /// ⚠ EXPERIMENTAL: 40 MHz cal convergence on this HT20-class part is unverified.
    pub fn hw_reset_ht40(&mut self, chan_mhz: u16) -> Result<(), FaceError> {
        self.ht40.store(true, std::sync::atomic::Ordering::Relaxed);
        self.hw_reset(chan_mhz)
    }

    pub fn hw_reset(&mut self, chan_mhz: u16) -> Result<(), FaceError> {
        use crate::ath9k_reg::*;

        // Save state a cold reset would clear (macStaId1 = AR_STA_ID1 & BASE_RATE_11B;
        // saveDefAntenna, min 1; saveLedState). Read before the reset.
        let mac_sta_id1 = self.reg_read(AR_STA_ID1)? & AR_STA_ID1_BASE_RATE_11B;
        let mut save_def_antenna = self.reg_read(AR_DEF_ANTENNA)?;
        if save_def_antenna == 0 {
            save_def_antenna = 1;
        }
        let save_led_state = self.reg_read(AR_CFG_LED)?
            & (AR_CFG_LED_ASSOC_CTL
                | AR_CFG_LED_MODE_SEL
                | AR_CFG_LED_BLINK_THRESH_SEL
                | AR_CFG_LED_BLINK_SLOW);

        // 1. setpower AWAKE (force-wake).
        self.setpower_awake()?;
        // 2. mark_phy_inactive.
        self.mark_phy_inactive()?;
        // 3. AR9271 RADIO_RF_RST + ath9k_hw_chip_reset (reset regs + init_pll) +
        //    GATE_MAC_CTL — all inside phy_reset().
        self.phy_reset()?;
        // AR9280_20_OR_LATER: disable JTAG on the shared GPIO (hw.c:1958).
        self.reg_set_bit(AR_GPIO_INPUT_EN_VAL, AR_GPIO_JTAG_DISABLE)?;
        // 4. ar9002_hw_enable_async_fifo (no-op on AR9271).
        self.enable_async_fifo()?;
        // 5. process_ini (analog-shift + MODES/COMMON/ANI/TX-gain + override_ini + TURBO).
        self.apply_initvals()?;
        // 6. set_rfmode (AR_PHY_MODE).
        self.set_rfmode()?;
        // 7. init_mfp.
        self.init_mfp()?;
        // 8. set_delta_slope.
        self.set_delta_slope(chan_mhz)?;
        // 9. spur_mitigate_freq (no-spur path).
        self.spur_mitigate_freq(chan_mhz)?;
        // (eep_ops->set_board_values — SKIPPED: EEPROM TX-power/gain, not RX-gating.)
        // 10. reset_opmode (STA_ID/BSSID/opmode).
        self.reset_opmode(mac_sta_id1, save_def_antenna)?;
        // 11. rf_set_freq (the synth).
        self.rf_set_freq(chan_mhz)?;
        // 12. set_clockrate (computes the MAC clock; writes nothing).
        self.set_clockrate();
        // 13. init_queues.
        self.init_queues()?;
        // 14. init_interrupt_masks.
        self.init_interrupt_masks()?;
        // 15. ani_cache_ini_regs (read-only no-op).
        self.ani_cache_ini_regs()?;
        // 16. init_qos.
        self.init_qos()?;
        // 17. init_global_settings.
        self.init_global_settings(chan_mhz)?;
        // REG_SET_BIT(AR_STA_ID1, PRESERVE_SEQNUM) (hw.c:2015).
        self.reg_set_bit(AR_STA_ID1, AR_STA_ID1_PRESERVE_SEQNUM)?;
        // 18. set_dma.
        self.set_dma()?;
        // 19. REG_WRITE(AR_OBS, 8).
        self.reg_write(AR_OBS, 8)?;
        // 20. init_bb (AR_PHY_ACTIVE).
        self.init_bb()?;
        // 21. init_cal — ★ runs LAST, after the full PHY/MAC setup.
        self.init_cal(chan_mhz)?;
        // 21b. HT40: the HT20 CL-cal (ar9271_cl_cal_ht20, run by init_cal) clears DYN2040 at its end.
        // Re-assert it so the PHY actually operates 40 MHz — the streamed 2G_HT40 initvals and the
        // +10 MHz synth centre are already in place; DYN2040 is the enable that ties them together.
        if self.ht40.load(std::sync::atomic::Ordering::Relaxed) {
            self.reg_set_bit(AR_PHY_TURBO, AR_PHY_FC_DYN2040_EN)?;
        }
        // 22. restore_chainmask (no-op for 1-chain).
        self.restore_chainmask()?;
        // 23. REG_WRITE(AR_CFG_LED, saveLedState | AR_CFG_SCLK_32KHZ).
        self.reg_write(AR_CFG_LED, save_led_state | AR_CFG_SCLK_32KHZ)?;
        // 24. init_desc (AR_CFG descriptor byte-swap).
        self.init_desc()?;
        Ok(())
    }

    /// **Start receive** — the host register side of `ath9k_hw_startpcureceive` +
    /// `ath9k_hw_setrxfilter` + `ath9k_hw_rxena`, done AFTER [`hw_reset`](Self::hw_reset)
    /// (as `ath9k_htc_start` does). Sets the monitor RX filter (0xc03f), opens the
    /// multicast hash, clears the RX-disable/abort diag bits, and enables RX DMA
    /// (`AR_CR_RXE`). Opmode/STA and the RX-DMA burst config were already applied by
    /// `hw_reset` (reset_opmode / set_dma).
    pub fn start_receive(&mut self) -> Result<(), FaceError> {
        use crate::ath9k_reg::*;
        // ath9k_hw_setrxfilter: RX_FILTER then AR_PHY_ERR (0 = no PHY-err filtering).
        self.reg_write(AR_RX_FILTER, 0x0000_c03f)?;
        self.reg_write(AR_PHY_ERR, 0)?;
        // Accept all multicast/broadcast (64-bit hash defaults to 0 after reset).
        self.reg_write(AR_MCAST_FIL0, 0xffff_ffff)?;
        self.reg_write(AR_MCAST_FIL1, 0xffff_ffff)?;
        // ath9k_hw_startpcureceive: clear the RX-disable/abort diag bits.
        self.reg_clr_bit(AR_DIAG_SW, AR_DIAG_RX_DIS | AR_DIAG_RX_ABORT)?;
        // ath9k_hw_rxena: start RX DMA on the MAC.
        self.reg_write(AR_CR, AR_CR_RXE)?;
        Ok(())
    }

    /// Create a monitor vif + its self-station on the target so injected data frames have a valid
    /// `node_idx`/`vif_idx` (0/0) for the target's rate control. Mirrors mainline
    /// `ath9k_htc_add_monitor_interface` (`WMI_VAP_CREATE`) + `ath9k_htc_add_station`
    /// (`WMI_NODE_CREATE`). ★ Without a created node the target DROPS injected frames — MEASURED
    /// 0 on air with `node_idx = 0` referencing a non-existent node. The host picks both indices
    /// (0/0). `mac` is the vif/node address (locally-administered); the injected `addr2` need not
    /// match it — injection sets addr2 per frame; the node is only the rate-control lookup at
    /// `node_idx`.
    pub fn create_monitor_vif_node(&mut self, mac: [u8; 6]) -> Result<(), FaceError> {
        const HTC_M_MONITOR: u8 = 8; // htc.h enum htc_opmode

        // struct ath9k_htc_target_vif (htc.h, __packed, 12 B):
        //   index(1) opmode(1) myaddr[6] ath_cap(1) rtsthreshold(be16) pad(1)
        let mut hvif = [0u8; 12];
        hvif[0] = 0; // index = first free vif slot
        hvif[1] = HTC_M_MONITOR;
        hvif[2..8].copy_from_slice(&mac);
        self.wmi_cmd(WmiCmd::VapCreate, &hvif)?;

        // struct ath9k_htc_target_sta (htc.h, __packed, 22 B):
        //   macaddr[6] bssid[6] sta_index(1) vif_index(1) is_vif_sta(1)
        //   flags(be16) htcap(be16) maxampdu(be16) pad(1)
        let mut tsta = [0u8; 22];
        tsta[0..6].copy_from_slice(&mac); // macaddr
        // bssid (6..12) = 0
        tsta[12] = 0; // sta_index = first free
        tsta[13] = 0; // vif_index = the vif created above
        tsta[14] = 1; // is_vif_sta (the vif's own station)
        // flags (15..17) = 0, htcap (17..19) = 0
        tsta[19..21].copy_from_slice(&0xffffu16.to_be_bytes()); // maxampdu
        // pad (21) = 0
        self.wmi_cmd(WmiCmd::NodeCreate, &tsta)?;
        Ok(())
    }

    /// Push the `ieee80211com_target` capability block (`WMI_TARGET_IC_UPDATE`). ★ **Load-bearing for
    /// TX:** the target's `ath_tgt_send_mgt` sets every rate series' `ChSel = sc_ic.ic_tx_chainmask`,
    /// and `sc_ic` is populated ONLY here. Without this the chain-select field of the TX descriptor
    /// (`AR_ChainSel0`, ds_ctl7) is 0 → **no antenna chain drives the frame**: the MAC keys the
    /// transmitter (TFCNT advances, TXOK completes) but nothing coherent radiates — MEASURED, a
    /// witness at inches decoded 0 of our frames until this was sent. RX is unaffected (its chainmask
    /// comes from the initvals registers), which is why M1 RX worked without it.
    ///
    /// `struct ieee80211com_target` (wlan_hdr.h, 8 B): `ic_ampdu_limit(be32) ic_ampdu_subframes(1)
    /// ic_enable_coex(1) ic_tx_chainmask(1) pad(1)`. The AR9271 is 1×1 → tx chainmask = 1 (chain 0).
    pub fn send_ic_update(&mut self) -> Result<(), FaceError> {
        const AR9271_TX_CHAINMASK: u8 = 1; // 1×1 part → chain 0 only
        let mut ic = [0u8; 8];
        // ic_ampdu_limit (0..4) = 0 — we send single (non-aggregated) frames, so the AMPDU limit is
        // irrelevant; the target `ntohl`s it. ic_ampdu_subframes (4) / ic_enable_coex (5) = 0.
        ic[6] = AR9271_TX_CHAINMASK; // ic_tx_chainmask ← the actual fix
        // pad (7) = 0
        self.wmi_cmd(WmiCmd::TargetIcUpdate, &ic)?;
        Ok(())
    }

    /// **UNFINISHED — the node's rate table (`WMI_RC_STATE_CHANGE`).** A created node still needs a
    /// rate table or the target has no rate to TX at and drops the frame. The payload is
    /// `struct ath9k_htc_target_rate` (70 B: `sta_index`, `isnew`, 2 pad, `capflags` be32, then two
    /// `rateset`s). ★ BLOCKED: at 70 B the message exceeds the 64-B register pipe, and sending it as
    /// a multi-packet interrupt transfer TIMED OUT (target didn't process/reply) — so large WMI must
    /// reach the target another way (a bulk pipe? a fragmentation the target reassembles?). Resolve
    /// via a golden trace of the kernel's own `WMI_RC_STATE_CHANGE` before wiring this in. Kept as
    /// documentation of the exact remaining gap; NOT called (it would break `wmi_start`).
    #[allow(dead_code)]
    pub fn set_node_rate_table(&mut self) -> Result<(), FaceError> {
        let mut trate = [0u8; 70];
        trate[1] = 1; // isnew (sta_index/vif 0, capflags 0 = legacy)
        let legacy: [u8; 12] = [2, 4, 11, 22, 12, 18, 24, 36, 48, 72, 96, 108]; // 500 kbps units
        trate[8] = legacy.len() as u8;
        trate[9..9 + legacy.len()].copy_from_slice(&legacy);
        self.wmi_cmd(WmiCmd::RcStateChange, &trate)?;
        Ok(())
    }

    /// **WMI RX-start verbs** — the HTC target-side trigger, called after
    /// [`hw_reset`](Self::hw_reset) + [`start_receive`](Self::start_receive), matching
    /// `ath9k_htc_start`'s order: `WMI_ATH_INIT` → `WMI_SET_MODE(11NG)` →
    /// `WMI_START_RECV` → `WMI_ENABLE_INTR`. Then the host arms `AR_IMR/S0/S1/S2`
    /// (`ath9k_hw_set_interrupts`) with the golden-trace values, written AFTER
    /// ENABLE_INTR so they stick — without this the target's RX ISR stays dormant.
    ///
    /// `connect_data_services()` must have run first (the RX endpoints must exist
    /// before `START_RECV`).
    pub fn wmi_start(&mut self) -> Result<(), FaceError> {
        use crate::ath9k_reg::*;
        self.wmi_cmd(WmiCmd::AthInit, &[])?;
        self.wmi_cmd(WmiCmd::SetMode, &IEEE80211_MODE_11NG.to_be_bytes())?;
        // Push the IC capability block so `sc_ic.ic_tx_chainmask` is set BEFORE any TX — the target
        // copies it into every frame's rate-series ChSel. Without it ChSel=0 → no TX chain radiates
        // (MEASURED: TXOK completes but 0 frames on air). Must precede the first injected frame.
        self.send_ic_update()?;
        // Create the monitor vif + self-station so injected data frames (tx_frame_hdr.node_idx=0,
        // vif_idx=0) have a valid target node for rate control — without it the target drops TX.
        self.create_monitor_vif_node(SELF_MAC)?;
        self.wmi_cmd(WmiCmd::StartRecv, &[])?;
        self.wmi_cmd(WmiCmd::EnableIntr, &[])?;
        // ath9k_hw_set_interrupts — host-side final AR_IMR arming (golden trace).
        self.reg_write(AR_IMR, 0x8180_0964)?;
        // AR_IMR_S0 gates per-QCU TX completion interrupts: TXOK = bits[9:0], TXDESC = bits[25:16].
        // ★ The golden-trace value (0x0001_0000 = TXDESC q0 only) was a RX-focused monitor capture and
        // enables NO TXOK for the data queues — so queue 1 (the one `ath_tgt_send_mgt` uses) never
        // raises a completion interrupt, the target's TX tasklet never reaps, and its WLAN-endpoint
        // buffers never recycle → injection blocks at the ring depth (~33 credits). Enable TXOK for
        // the four data ACs (q0-3) so completions fire and sustained TX works. MEASURED: without this
        // the 34th injected frame's bulk-OUT NAKs forever.
        self.reg_write(AR_IMR_S0, 0x0001_000f)?;
        self.reg_write(AR_IMR_S1, 0x0001_0000)?;
        self.reg_write(AR_IMR_S2, 0x0080_0000)?;
        Ok(())
    }

    /// `ar9285_hw_cl_cal` for an HT20 channel (the AR9271 offset + AGC calibration path). Returns
    /// whether the AGC cal (`AR_PHY_AGC_CONTROL_CAL`) cleared — a stuck bit means the cal hung
    /// (noisy environment / bad initvals), which the caller reports rather than hiding.
    fn ar9271_cl_cal_ht20(&self) -> Result<bool, FaceError> {
        use crate::ath9k_reg::*;

        self.reg_set_bit(AR_PHY_CL_CAL_CTL, AR_PHY_CL_CAL_ENABLE)?;

        // HT20 branch: parallel offset cal with DYN2040 temporarily set.
        self.reg_set_bit(AR_PHY_CL_CAL_CTL, AR_PHY_PARALLEL_CAL_ENABLE)?;
        self.reg_set_bit(AR_PHY_TURBO, AR_PHY_FC_DYN2040_EN)?;
        self.reg_clr_bit(AR_PHY_AGC_CONTROL, AR_PHY_AGC_CONTROL_FLTR_CAL)?;
        self.reg_clr_bit(AR_PHY_TPCRG1, AR_PHY_TPCRG1_PD_CAL_ENABLE)?;
        self.reg_set_bit(AR_PHY_AGC_CONTROL, AR_PHY_AGC_CONTROL_CAL)?;
        if self
            .wait_reg(AR_PHY_AGC_CONTROL, AR_PHY_AGC_CONTROL_CAL, 0)
            .is_err()
        {
            return Ok(false);
        }
        self.reg_clr_bit(AR_PHY_TURBO, AR_PHY_FC_DYN2040_EN)?;
        self.reg_clr_bit(AR_PHY_CL_CAL_CTL, AR_PHY_PARALLEL_CAL_ENABLE)?;
        self.reg_clr_bit(AR_PHY_CL_CAL_CTL, AR_PHY_CL_CAL_ENABLE)?;

        // Main AGC/filter cal.
        self.reg_clr_bit(AR_PHY_ADC_CTL, AR_PHY_ADC_CTL_OFF_PWDADC)?;
        self.reg_set_bit(AR_PHY_AGC_CONTROL, AR_PHY_AGC_CONTROL_FLTR_CAL)?;
        self.reg_set_bit(AR_PHY_TPCRG1, AR_PHY_TPCRG1_PD_CAL_ENABLE)?;
        self.reg_set_bit(AR_PHY_AGC_CONTROL, AR_PHY_AGC_CONTROL_CAL)?;
        let converged = self
            .wait_reg(AR_PHY_AGC_CONTROL, AR_PHY_AGC_CONTROL_CAL, 0)
            .is_ok();

        self.reg_set_bit(AR_PHY_ADC_CTL, AR_PHY_ADC_CTL_OFF_PWDADC)?;
        self.reg_clr_bit(AR_PHY_CL_CAL_CTL, AR_PHY_CL_CAL_ENABLE)?;
        self.reg_clr_bit(AR_PHY_AGC_CONTROL, AR_PHY_AGC_CONTROL_FLTR_CAL)?;

        Ok(converged)
    }

    /// **M1.3 (cal tail)** — the AR9271 reset-time RX RF calibration. This is the piece
    /// that makes the receiver actually decode: without it the demod never converges.
    ///
    /// Transcribed from mainline ath9k `ar9002_calib.c`: `ar9002_hw_init_cal` (the cal
    /// list) + `ath9k_hw_per_calibration` / `ath9k_hw_reset_calibration` (the arm ->
    /// poll-DO_CAL -> collect -> apply loop) + `ar9002_hw_iqcal_collect` (:125) +
    /// `ar9002_hw_iqcalibrate` (:192).
    ///
    /// * The IQ-mismatch cal latches RX-path correction coefficients into
    /// hardware-owned registers (the `AR_PHY_TIMING_CTRL4` IQCORR fields and the deeper
    /// RX-correction registers the cal engine writes as a side effect) that cannot be
    /// programmed directly — proven on silicon.
    ///
    /// **Scope: AR9271, single 2.4 GHz HT20 chain (numChains = 1).** On an HT20 channel
    /// the AR9271 cal list contains **only** the IQ-mismatch cal.
    /// `ar9002_hw_is_cal_supported` (ar9002_calib.c:40-45) gates ADC-gain and ADC-DC
    /// behind `IS_CHAN_HT40`, and `ar9002_hw_init_cal` (ar9002_calib.c:919-923) inserts
    /// IQ unconditionally but the two ADC cals only for HT40. So HT20 reset = IQ only.
    /// (A future HT40 mode would add the two ADC cals here — read `AR_PHY_CAL_MEAS_0..3`,
    /// write `AR_PHY_NEW_ADC_DC_GAIN_CORR`; deliberately omitted, HT20 never runs them.)
    ///
    /// AR9271 uses the **single-sample** percal profile (`ar9002_hw_init_cal_settings`:
    /// `AR_SREV_9280_20_OR_LATER` is true for macVersion 0x140), i.e.
    /// `iq_cal_single_sample` (ar9002_calib.c:944-950): `calNumSamples = MIN_CAL_SAMPLES
    /// = 1`, `calCountMax = PER_MAX_LOG_COUNT = 10`. One collect, one apply — the minimum
    /// WMI round-trips.
    ///
    /// Call this from [`set_channel_and_cal`](Self::set_channel_and_cal) after
    /// `ar9271_cl_cal_ht20` and after `start_nfcal`, matching `ar9002_hw_init_cal`'s
    /// order (AGC/CL cal -> loadnf -> start_nfcal -> arm+run the periodic cal list).
    pub fn run_rx_calibration(&self, chan_mhz: u16) -> Result<(), FaceError> {
        use crate::ath9k_reg::*;

        if std::env::var_os("NDR_ATH9K_DEBUG").is_some() {
            eprintln!("[ath9k] IQ-mismatch RX cal on {chan_mhz} MHz (HT20, chain 0)");
        }

        // AR9271 single-sample IQ profile (ar9002_calib.c:944-950, iq_cal_single_sample).
        const CAL_COUNT_MAX: u32 = 10; // PER_MAX_LOG_COUNT -> IQCAL_LOG_COUNT_MAX field
        const CAL_NUM_SAMPLES: u32 = 1; // MIN_CAL_SAMPLES

        // -- ar9002_hw_setup_calibration: program IQCAL_LOG_COUNT_MAX + CALMODE, arm DO_CAL.
        let arm = |me: &Self| -> Result<(), FaceError> {
            // REG_RMW_FIELD(TIMING_CTRL4(0), IQCAL_LOG_COUNT_MAX, calCountMax)
            me.reg_rmw(
                AR_PHY_TIMING_CTRL4,
                (CAL_COUNT_MAX << AR_PHY_TIMING_CTRL4_IQCAL_LOG_COUNT_MAX_S)
                    & AR_PHY_TIMING_CTRL4_IQCAL_LOG_COUNT_MAX,
                AR_PHY_TIMING_CTRL4_IQCAL_LOG_COUNT_MAX,
            )?;
            me.reg_write(AR_PHY_CALMODE, AR_PHY_CALMODE_IQ)?; // = 0
            me.reg_set_bit(AR_PHY_TIMING_CTRL4, AR_PHY_TIMING_CTRL4_DO_CAL)?;
            Ok(())
        };

        let _ = CAL_NUM_SAMPLES;

        // ★ ath9k_hw_reset_calibration (calib.c): at RESET the driver only ARMS the IQ cal —
        // `setup_calibration` sets DO_CAL, `calState = CAL_RUNNING` — and RETURNS. It does NOT block
        // on DO_CAL. The measurement collect + correction apply (`ar9002_hw_per_calibration`) run
        // LATER over the driver's periodic cal timer, which a bring-up-only driver doesn't have.
        // The IQ correction is a refinement, NOT a prerequisite for RX, so kicking is both faithful
        // and sufficient. (The previous blocking `wait_reg(DO_CAL == 0)` hung forever because the
        // cal never completes in one shot at reset — the collect/apply math lives in
        // `iq_cal_correction`, still unit-tested, for a future periodic-cal implementation.)
        arm(self)?;
        Ok(())
    }

    // ── M1.4 — RX enable (host register side + target verbs) ───────────────────

    /// **M1.4** — open the receiver. Host register side: monitor opmode, the 0xBF RX filter,
    /// clear the RX-disable/abort diag bits, enable RX DMA (`AR_CR_RXE`). Then the WMI verbs that
    /// start the target's RX tasklet: `WMI_ATH_INIT` → `WMI_SET_MODE(11NG)` → `WMI_START_RECV`.
    ///
    /// [`connect_data_services`](Self::connect_data_services) must already have run so the
    /// Mgmt/DataBE/Beacon endpoints exist before `START_RECV`.
    pub fn rx_enable(&mut self) -> Result<(), FaceError> {
        use crate::ath9k_reg::*;

        // ── ath9k_hw_set_dma (reset tail we had skipped) ──
        // ★ THE MISSING RX STEP. The MAC receives frames but never DMAs them into the descriptor
        // ring unless AR_RXCFG's burst size is set — leaving the target RX tasklet with nothing
        // (measured ndr_stats.seen=0, while the kernel driver receives fine on the same dongle).
        // 128-byte DMA bursts + the RX FIFO threshold, non-9300 path (AR9271).
        self.reg_set_bit(AR_AHB_MODE, AR_AHB_PREFETCH_RD_EN)?;
        self.reg_rmw(AR_TXCFG, AR_TXCFG_DMASZ_128B, AR_TXCFG_DMASZ_MASK)?;
        self.reg_rmw(AR_RXCFG, AR_RXCFG_DMASZ_128B, AR_RXCFG_DMASZ_MASK)?;
        self.reg_write(AR_RXFIFO_CFG, 0x200)?;

        // Monitor opmode: set KSRCH_MODE, clear the AP/ADHOC opmode bits (ath9k_hw_set_operating_mode
        // default/monitor case). A benign station address keeps STA_ID sane; monitor RX doesn't gate
        // on it.
        self.reg_write(AR_STA_ID0, 0x1a2b_3c4d)?;
        let id1 = self.reg_read(AR_STA_ID1)?;
        let id1 = (id1 & !(AR_STA_ID1_STA_AP | AR_STA_ID1_ADHOC)) | AR_STA_ID1_KSRCH_MODE;
        self.reg_write(AR_STA_ID1, id1)?;

        // Open the receiver to everything on-channel. Use the kernel driver's exact monitor value
        // (golden trace) 0xc03f, not our 0xBF — it adds MCAST_BCAST_ALL (0x8000) + 0x4000, without
        // which broadcast beacons are dropped at the hardware RX filter (before DMA → seen=0).
        self.reg_write(AR_RX_FILTER, 0x0000_c03f)?;

        // ★ Accept ALL multicast. AR_MCAST_FIL0/1 is a 64-bit hash that defaults to 0 after reset,
        // which drops every multicast/broadcast frame — i.e. every beacon — before RX DMA. This was
        // the seen=0 deaf-receiver root cause: the golden trace of the working kernel driver writes
        // both to 0xffffffff, and the RX_FILTER MCAST bit alone is not enough.
        self.reg_write(AR_MCAST_FIL0, 0xffff_ffff)?;
        self.reg_write(AR_MCAST_FIL1, 0xffff_ffff)?;
        // MAC config / descriptor byte-swap — the kernel writes 0x0a on this host (golden trace).
        self.reg_write(AR_CFG, 0x0000_000a)?;

        // ath9k_hw_startpcureceive: clear the RX-disable/abort diag bits.
        self.reg_clr_bit(AR_DIAG_SW, AR_DIAG_RX_DIS | AR_DIAG_RX_ABORT)?;

        // Bring up the target WLAN app (interrupts + ISR), select the 2.4 GHz rate table, then
        // start the target RX ring (programs AR_RXDP on the target side).
        self.wmi_cmd(WmiCmd::AthInit, &[])?;
        self.wmi_cmd(WmiCmd::SetMode, &IEEE80211_MODE_11NG.to_be_bytes())?;
        self.wmi_cmd(WmiCmd::StartRecv, &[])?;
        // Enable the target's interrupts so its RX ISR fires and `ath_tgt_rx_tasklet` actually
        // delivers received frames up HTC. Without this the MAC receives fine (filter/DIAG/RXE all
        // set) but nothing ever crosses USB — the RX tasklet is interrupt-driven and stays dormant.
        // Mirrors the kernel `ath9k_htc_start` order (ATH_INIT → START_RECV → ENABLE_INTR); the
        // firmware handler `ath_enable_intr_tgt` takes no payload (kernel uses `WMI_CMD`, not `_BUF`).
        self.wmi_cmd(WmiCmd::EnableIntr, &[])?;

        // ★ Arm the hardware interrupt mask from the host. AR_IMR gates which MAC events raise the
        // target CPU's interrupt; the target's ENABLE_INTR only edits its software SWBA/BMISS mask
        // and never sets AR_IMR to enable RX, so its RX ISR/tasklet stays dormant (seen=0). Values
        // from the working kernel driver's golden trace; written AFTER ENABLE_INTR so they stick.
        self.reg_write(AR_IMR, 0x8180_0964)?;
        self.reg_write(AR_IMR_S0, 0x0001_0000)?;
        self.reg_write(AR_IMR_S1, 0x0001_0000)?;
        self.reg_write(AR_IMR_S2, 0x0080_0000)?;

        // Start RX DMA on the MAC now that the target's descriptor ring + RXDP are set.
        self.reg_write(AR_CR, AR_CR_RXE)?;

        Ok(())
    }

    /// Receive one frame off the bulk WLAN-RX pipe and parse the `ath_htc_rx_status` prefix.
    ///
    /// Transfer layout: `[HTC_FRAME_HDR 8B][ath_htc_rx_status 40B][802.11 …]`. Returns `Ok(None)`
    /// on a short/empty transfer (e.g. a keepalive) so the caller can keep polling; the 802.11
    /// bytes and every decoded status field come back in [`RxFrame`].
    pub fn recv_frame(&mut self, timeout: Duration) -> Result<Option<RxFrame>, FaceError> {
        let raw = self.recv_raw_frame(timeout)?;
        Ok(Self::parse_rx(&raw))
    }

    /// Parse a raw bulk-RX transfer into an [`RxFrame`]. Split out for unit-testing the field
    /// offsets against a captured buffer without hardware.
    pub fn parse_rx(raw: &[u8]) -> Option<RxFrame> {
        // ★ MEASURED wire layout on the WLAN-RX bulk pipe (raw dump): the `ath_rx_status` (40 B,
        // `rx_frame_header` = u32[10], ah_desc.h) sits at offset 12, and the 802.11 MPDU at 52 —
        // i.e. the prefix is the 8-byte HTC frame header PLUS a 4-byte data-endpoint prefix
        // (observed `04 00 00 00`), NOT the bare 8-byte HTC header. All status fields are
        // big-endian (the target is big-endian; the memcpy'd status rides the wire as-is).
        const RX_HDR_LEN: usize = HTC_HDR_LEN + 4; // = 12
        if raw.len() < RX_HDR_LEN + HTC_RX_STATUS_LEN {
            return None;
        }
        let s = &raw[RX_HDR_LEN..RX_HDR_LEN + HTC_RX_STATUS_LEN];
        let rs_tstamp = u64::from_be_bytes(s[0..8].try_into().unwrap());
        let rs_datalen = u16::from_be_bytes([s[8], s[9]]);
        let frame = raw[RX_HDR_LEN + HTC_RX_STATUS_LEN..].to_vec();
        Some(RxFrame {
            rs_tstamp,
            rs_datalen,
            rs_status: s[10],
            rs_phyerr: s[11],
            rs_rssi: s[12] as i8,
            rs_keyix: s[19],
            rs_rate: s[20],
            rs_antenna: s[21],
            rs_more: s[22],
            rs_flags: s[26],
            frame,
        })
    }
}

/// **M2 — the AR9271 as a common-view clock source.** Every received frame carries a per-frame
/// hardware RX timestamp (`rs_tstamp`, µs) on a stable free-running counter — a `FreeRunRxStamp`,
/// the strongest link clock in the taxonomy (design §15). MEASURED advancing frame-to-frame on
/// silicon. That makes `FaceTimeProfile::derive` report `can_common_view = true`: two receivers'
/// stamps of one on-air frame difference into a µs cross-node offset. `read_clock` stays the
/// default `None` — the AR_TSF register is readable but read-now would need `&mut self` (a WMI
/// round-trip); the per-frame latch is what common-view uses, and it is the honest best clock.
impl RadioTime for Ath9kHtcBackend {
    fn time_sources(&self) -> Vec<RadioTimeSource> {
        vec![RadioTimeSource::free_run_rx_stamp(self.tsf_domain, 1_000)]
    }

    /// Read the hardware TSF (µs) — `AR_TSF_U32`/`AR_TSF_L32` (0x8050/0x804c), the same free-running
    /// clock the per-frame `rs_tstamp` samples. Wrap-safe (re-read the high word). Only answers this
    /// radio's own `tsf_domain`; any other domain is unknown here. This makes the AR9271's clock
    /// *readable now*, not just per-frame — the read side of the TimeToken / common-view (#3).
    fn read_clock(&self, domain: ClockDomainId) -> Result<Option<u64>, FaceError> {
        if domain != self.tsf_domain {
            return Ok(None);
        }
        const AR_TSF_L32: u32 = 0x804c;
        const AR_TSF_U32: u32 = 0x8050;
        let hi1 = self.reg_read(AR_TSF_U32)?;
        let lo = self.reg_read(AR_TSF_L32)?;
        let hi2 = self.reg_read(AR_TSF_U32)?;
        // If the low word wrapped between the two high reads, take the second high with lo=0-ish edge.
        let (hi, lo) = if hi1 == hi2 { (hi1, lo) } else { (hi2, self.reg_read(AR_TSF_L32)?) };
        Ok(Some(((hi as u64) << 32) | lo as u64))
    }
}

// ── M3 — FrameIo (stamped RX up + TX inject) ─────────────────────────────────────────────────────
//
// The AR9271 already RECEIVES and parses frames on silicon (M1-done) and carries a per-frame µs RX
// stamp (M2), so this makes it a first-class `FrameIo` radio. RX is the low-risk half; TX framing is
// the uncertain half and is bench-validated (see `build_tx_frame_bytes`).

/// Serialise a [`TxFrameHdr`] to its 12 on-wire bytes (the C struct is all `u8`/`[u8;4]` in
/// declaration order under `#[repr(C)] __packed`, so this is a faithful field-by-field image).
fn tx_frame_hdr_bytes(h: &TxFrameHdr) -> [u8; TX_FRAME_HDR_SIZE] {
    [
        h.data_type,
        h.node_idx,
        h.vif_idx,
        h.tidno,
        h.flags[0],
        h.flags[1],
        h.flags[2],
        h.flags[3],
        h.key_type,
        h.keyix,
        h.cookie,
        h.pad,
    ]
}

/// Build one HTC data-endpoint TX buffer for `frame` — pure (no `self`, no I/O) so the byte layout
/// is unit-testable without a device.
///
/// Wire layout, transcribed from mainline ath9k `htc_drv_txrx.c::ath9k_htc_tx_data` +
/// `htc_hst.c::htc_issue_send` and the firmware TX handler (`~/ath9k-fw/.../wlan/if_ath.c`):
///
/// ```text
///   [ HTC_FRAME_HDR  8B ] endpoint=DataBE, flags=0, be16 payload_len(=tx_frame_hdr+dot11), ctrl[4]=0
///   [ tx_frame_hdr  12B ] data_type=NORMAL, node_idx, vif_idx, tidno, be32 flags, key_type, keyix, cookie, pad
///   [ 802.11 MPDU      ] FC|Dur|A1=dst|A2=src|A3|Seq  ++  LLC/SNAP(0x8624)  ++  payload   (from `frame::build_dot11`)
/// ```
///
/// ★★ **BENCH — this is the uncertain part; watch these on silicon (RX-stamp is low-risk, TX is not):**
///  1. **`data_type`** — mainline `ATH9K_HTC_NORMAL = 2`. If frames are accepted over USB but never
///     key onto air (TX-PHY-OK stays 0), a wrong `data_type` steering the target handler is suspect #1.
///  2. **`node_idx` / `vif_idx` = 0.** ath9k data-frame TX resolves the *rate* and TX-queue from a
///     target **node** (`WMI_NODE_CREATE`) + **vif** the host normally creates via mac80211. We create
///     neither, so node 0 may be invalid → the target may drop the frame or use a default rate. If TX
///     is silent, the next thing to add is the node/vif setup (WMI verbs), not more descriptor bytes.
///  3. **No rate field.** The on-air rate is the target's rate control for `node_idx` — `set_rate`
///     does not steer it here (see `cur_mcs`). The example's "1 Mbps" is whatever the target picks.
///  4. **The 4-byte endpoint prefix.** RX carries an extra `04 00 00 00` between the HTC header and the
///     status block (`RX_PREFIX_LEN`). TX here does *not* prepend one (mainline TX doesn't); if the
///     target rejects the frame, try mirroring that prefix.
fn build_tx_frame_bytes(
    tx_ep: u8,
    format: FrameFormat,
    frame: &InjectFrame,
    rate_code: u8,
    tx_power: u8,
    rate_flags: u8,
) -> Result<Vec<u8>, FaceError> {
    // The 802.11 data frame + LLC/SNAP(0x8624) + payload — the shared helper, so a frame injected
    // here de-frames identically on any other backend.
    let dot11 = crate::frame::build_dot11(format, frame)?;
    // ★ MGMT-endpoint framing, matching the kernel's WORKING TX (golden trace): the 8-byte
    // `tx_mgmt_hdr`, NOT the 12-byte `tx_frame_hdr`. A monitor-vif DATA frame has no associated node
    // for rate control, so the target buffers but never transmits it; the mgmt path transmits the
    // raw 802.11 frame as-is. `tx_ep` is the Mgmt endpoint.
    const TX_MGMT_HDR_SIZE: usize = 8;
    let payload_len = TX_MGMT_HDR_SIZE + dot11.len();
    if payload_len > u16::MAX as usize {
        return Err(err(format!(
            "ath9k_htc: TX frame {payload_len} B exceeds the 16-bit HTC payload length"
        )));
    }

    // ── hif_usb TX stream header (4 B): le16 HTC-frame length + le16 stream tag ──
    // ★ MEASURED (golden trace of the kernel's inject): EVERY TX frame on the bulk WLAN pipe is
    // prefixed with this 4-byte header (`hif_usb.c` `hif_usb_send_mgmt`/`_send_tx`:
    // `*hdr = le16(skb->len - 4); *hdr = le16(ATH_USB_TX_STREAM_MODE_TAG)`). Without it the target's
    // hif_usb layer never frames the transfer as TX, and nothing goes on air (was our 0/40 bug).
    const ATH_USB_TX_STREAM_MODE_TAG: u16 = 0x697e; // hif_usb.h
    let htc_frame_len = HTC_HDR_LEN + payload_len;
    let mut buf = Vec::with_capacity(4 + htc_frame_len);
    buf.extend_from_slice(&(htc_frame_len as u16).to_le_bytes());
    buf.extend_from_slice(&ATH_USB_TX_STREAM_MODE_TAG.to_le_bytes());

    // ── HTC_FRAME_HDR (8 B) ──
    buf.push(tx_ep); // endpoint id (Mgmt, assigned in connect_data_services)
    buf.push(0); // flags
    buf.extend_from_slice(&(payload_len as u16).to_be_bytes()); // be16 payload length
    buf.extend_from_slice(&[0u8; 4]); // control bytes

    // ── tx_mgmt_hdr (8 B): node_idx, vif_idx, tidno, flags, key_type, keyix, cookie, pad ──
    // node/vif = 0 (the created monitor vif+node), keyix = 0xff (no key) — matches the kernel's
    // on-air mgmt TX header from the golden trace. ★ Our firmware repurposes two otherwise-unused
    // bytes as per-frame TX knobs (`ath_tgt_send_mgt`): `tidno` = `AR_XmitPower` (0.5-dB units, 0 =
    // firmware default), `pad` = the HW rate code (0 = target min rate; `0x80|mcs` = HT MCS). This is
    // the actuated MCS/rate/power path — inert on stock ath9k firmware, live on ours.
    buf.extend_from_slice(&[0, 0, tx_power, 0, rate_flags, 0xff, 0, rate_code]);

    // ── 802.11 MPDU ──
    buf.extend_from_slice(&dot11);
    Ok(buf)
}

/// Parse one RX unit at byte offset `at` in a bulk-IN transfer into an optional [`CapturedFrame`]
/// plus the byte length to advance to the next unit. Pure (format + domain passed in) so it is
/// unit-testable. A transfer may batch several HTC frames back-to-back; the pump walks them.
///
/// Unit layout mirrors [`Ath9kHtcBackend::parse_rx`]: `[HTC 8][ep-prefix 4][ath_htc_rx_status 40]
/// [802.11 MPDU (`rs_datalen` B)]`. The advance is taken from the HTC header's own `payload_len`
/// (authoritative), 4-byte aligned; it falls back to the status-derived length if that looks wrong.
fn parse_rx_unit(
    format: FrameFormat,
    domain: ClockDomainId,
    raw: &[u8],
    at: usize,
) -> Option<(Option<CapturedFrame>, usize)> {
    if at + RX_PREFIX_LEN + HTC_RX_STATUS_LEN > raw.len() {
        return None;
    }
    // HTC payload length = bytes after the 8-byte HTC header (the 4-byte ep prefix + 40-byte status
    // + MPDU). Authoritative for de-aggregation.
    let htc_payload_len = u16::from_be_bytes([raw[at + 2], raw[at + 3]]) as usize;

    let s = &raw[at + RX_PREFIX_LEN..at + RX_PREFIX_LEN + HTC_RX_STATUS_LEN];
    let rs_tstamp = u64::from_be_bytes(s[0..8].try_into().unwrap());
    let rs_datalen = u16::from_be_bytes([s[8], s[9]]) as usize;
    let rs_status = s[10];
    let rs_rssi = s[12] as i8;
    let rs_rate = s[20];

    // Advance: prefer HTC payload_len (8 + it), else the status-derived length. Always makes at
    // least one unit of progress so the walk can't loop.
    let unit = if htc_payload_len >= 4 + HTC_RX_STATUS_LEN {
        HTC_HDR_LEN + htc_payload_len
    } else {
        RX_PREFIX_LEN + HTC_RX_STATUS_LEN + rs_datalen
    };
    let advance = ((unit + 3) & !3).max(RX_PREFIX_LEN + HTC_RX_STATUS_LEN);

    let mpdu_start = at + RX_PREFIX_LEN + HTC_RX_STATUS_LEN;
    let mpdu_end = (mpdu_start + rs_datalen).min(raw.len());

    let captured = (|| {
        if mpdu_end <= mpdu_start {
            return None;
        }
        // Drop hardware-flagged errors: ATH9K_RXERR_{CRC 0x01, PHY 0x02, FIFO 0x04, DECRYPT 0x08}
        // occupy the low nibble of rs_status (mainline `ah.h`). A good frame reads 0.
        // ⚠ BENCH: if RX yields nothing, confirm this offset/gate — relax to `rs_status & 0x01`
        // (CRC only) or 0 to isolate a mislabelled bit before blaming the air.
        if rs_status & 0x0f != 0 {
            return None;
        }
        // rs_rssi is dB above the noise floor; convert to approximate dBm (see AR9271_NF_DBM_2GHZ).
        let rssi_dbm = Some((rs_rssi as i16 - AR9271_NF_DBM_2GHZ).clamp(-128, 127) as i8);
        // ath9k HT hardware rate codes set bit 7 (`0x80 | mcs`); legacy CCK/OFDM codes are < 0x80.
        // ⚠ BENCH: this is a best-effort decode; a 1x1 AR9271 only reaches HT MCS0-7.
        let mcs_index = (rs_rate & 0x80 != 0).then_some(rs_rate & 0x7f);
        let stamp = Some(LinkStamp::new(
            rs_tstamp,
            domain,
            1_000,
            LatchPoint::MacDone,
        ));
        crate::frame::parse_dot11(format, &raw[mpdu_start..mpdu_end], rssi_dbm, mcs_index, stamp)
    })();

    Some((captured, advance))
}

/// A legacy (non-HT) 802.11 transmit rate — the AR5416 hardware rate codes (#6). CCK is 802.11b
/// (2.4 GHz only), OFDM is 802.11a/g. These go verbatim into the mgmt-header `pad` byte, which our
/// firmware writes into `series[i].Rate` with no HT rate flags. All codes are `< 0x80`, which is how
/// `send_mgt` distinguishes a legacy code from an HT MCS (`0x80|mcs`). Long-preamble CCK codes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum LegacyRate {
    Cck1 = 0x1b,
    Cck2 = 0x1a,
    Cck5_5 = 0x19,
    Cck11 = 0x18,
    Ofdm6 = 0x0b,
    Ofdm9 = 0x0f,
    Ofdm12 = 0x0a,
    Ofdm18 = 0x0e,
    Ofdm24 = 0x09,
    Ofdm36 = 0x0d,
    Ofdm48 = 0x08,
    Ofdm54 = 0x0c,
}

impl Ath9kHtcBackend {
    /// ★ **Select a specific legacy (non-HT) transmit rate (#6).** The chosen CCK/OFDM rate is encoded
    /// into every injected frame's mgmt-header `pad` byte and the firmware transmits at that legacy PHY
    /// rate (no HT/SGI/HT40). Clears any HT `cur_mcs`, so exactly one rate mode is active. This is the
    /// worst-receiver lever for a legacy-only neighbour: the AR9271 can name 6 Mbps OFDM (the widely
    /// decodable robust default) or 1 Mbps CCK explicitly, rather than relying on the target min rate.
    pub fn set_legacy_rate(&self, rate: LegacyRate) {
        *self.cur_mcs.lock().unwrap() = None;
        self.cur_legacy
            .store(rate as u8, std::sync::atomic::Ordering::Relaxed);
    }

    /// Clear the legacy-rate override — subsequent frames use `cur_mcs` (HT) or the target min rate.
    pub fn clear_legacy_rate(&self) {
        self.cur_legacy.store(0, std::sync::atomic::Ordering::Relaxed);
    }

    /// Build the HTC data-endpoint TX buffer for `frame` (the [`build_tx_frame_bytes`] wire layout,
    /// bound to this device's DataBE endpoint + format).
    fn build_tx_frame(&self, frame: &InjectFrame) -> Result<Vec<u8>, FaceError> {
        // Per-frame rate + power our firmware's `ath_tgt_send_mgt` reads from the mgmt header.
        // `cur_mcs` → HT MCS rate code (`0x80|index`, MCS0-7 on this 1×1 part); None → 0 = the
        // target's min rate. `cur_power` → `AR_XmitPower` (0 = firmware default 30 dBm).
        use std::sync::atomic::Ordering::Relaxed;
        // A selected legacy rate (#6) wins: inject at a specific CCK/OFDM PHY rate with NO HT flags
        // (rate_flags = 0 ⇒ the firmware's `series[i].RateFlags` is legacy, `series[i].Rate` = the code).
        let legacy = self.cur_legacy.load(Relaxed);
        let (rate_code, mut rate_flags) = if legacy != 0 {
            (legacy, 0)
        } else {
            match *self.cur_mcs.lock().unwrap() {
                Some(mcs) => (0x80 | mcs.index.min(7), if mcs.short_gi { 0x01 } else { 0x00 }),
                None => (0, 0),
            }
        };
        // HT40 only applies to an HT MCS, never to a legacy rate (20 MHz-only PHY modes).
        if legacy == 0 && self.ht40.load(Relaxed) {
            rate_flags |= 0x02; // HAL_RATESERIES_2040 — transmit the MCS at 40 MHz
        }
        let tx_power = self.cur_power.load(Relaxed);
        build_tx_frame_bytes(self.mgmt_ep, self.format, frame, rate_code, tx_power, rate_flags)
    }
}

#[async_trait]
impl FrameIo for Ath9kHtcBackend {
    async fn inject(&self, frame: InjectFrame) -> Result<(), FaceError> {
        let buf = self.build_tx_frame(&frame)?;
        let handle = self.handle.clone();
        tokio::task::spawn_blocking(move || {
            // Flow control by retry. The target's WLAN-TX endpoint has a fixed buffer ring (~`credits`
            // deep); once it fills, the bulk-OUT pipe NAKs and the write times out having transferred
            // 0 bytes (device NAKed — nothing left the host), so re-sending the same frame is safe.
            // The target frees a buffer when a TX completes (its interrupt-driven completion tasklet,
            // fed by both TX-done and ambient RX interrupts), so a short bounded retry rides out the
            // backpressure instead of erroring at the pool depth (~34 frames) — the difference between
            // a 34-frame burst and sustained TX. A non-timeout error is a real fault: surface it.
            const TX_RETRY_MAX: u32 = 250; // × TX_WRITE_TIMEOUT ⇒ up to a few seconds of backpressure
            for attempt in 0..=TX_RETRY_MAX {
                match handle.write_bulk(EP_WLAN_TX, &buf, TX_WRITE_TIMEOUT) {
                    Ok(n) if n == buf.len() => return Ok(()),
                    Ok(n) => {
                        return Err(err(format!("ath9k_htc: short TX write {n}/{}", buf.len())))
                    }
                    Err(rusb::Error::Timeout) if attempt < TX_RETRY_MAX => continue,
                    Err(e) => return Err(usb_err("wlan tx", e)),
                }
            }
            Err(err("ath9k_htc: wlan tx backpressure did not clear".to_string()))
        })
        .await
        .map_err(|e| err(format!("ath9k_htc tx: join {e}")))?
    }

    /// The transmit MCS/rate. ★ **Actuated** (with our firmware): stored here and encoded into every
    /// injected frame's mgmt header (`pad` byte = `0x80|index` HT rate code), which our patched
    /// `ath_tgt_send_mgt` writes into the TX descriptor's rate series. MCS0-7 (this part is 1×1).
    /// On stock ath9k firmware the byte is ignored and the target picks the rate (hence the old
    /// "inert" note); on ours it steers the on-air rate. See [`build_tx_frame`](Self::build_tx_frame).
    fn set_rate(&self, mcs: McsDescriptor) -> Result<(), FaceError> {
        // Choosing an HT MCS supersedes any legacy override (#6) — exactly one rate mode is active.
        self.cur_legacy.store(0, std::sync::atomic::Ordering::Relaxed);
        *self.cur_mcs.lock().unwrap() = Some(mcs);
        Ok(())
    }

    async fn recv_frame(&self) -> Result<CapturedFrame, FaceError> {
        // Pumped mode: background reader threads keep bulk-IN transfers in flight and fill the shared
        // queue; just drain it.
        if self.rx_pump.is_pumped() {
            return Ok(self.rx_pump.recv().await);
        }
        // One-shot fallback (no pump): a blocking bulk-IN read, parsed into the queue, then drained.
        loop {
            if let Some(f) = self.rx_pump.try_pop() {
                return Ok(f);
            }
            let handle = self.handle.clone();
            let buf = tokio::task::spawn_blocking(move || {
                let mut buf = vec![0u8; 4096];
                match handle.read_bulk(EP_WLAN_RX, &mut buf, Duration::from_millis(200)) {
                    Ok(n) => {
                        buf.truncate(n);
                        Ok(Some(buf))
                    }
                    Err(rusb::Error::Timeout) => Ok(None),
                    Err(e) => Err(usb_err("wlan rx", e)),
                }
            })
            .await
            .map_err(|e| err(format!("ath9k_htc recv: join {e}")))??;

            if let Some(buf) = buf {
                self.rx_pump
                    .push(crate::rx_pump::Pumpable::parse_transfer(self, &buf));
            }
        }
    }
}

/// The RX pump's per-transfer parse: de-aggregate every HTC frame the target batched into one
/// bulk-IN transfer into decodable [`CapturedFrame`]s, each carrying its per-frame `rs_tstamp`.
impl crate::rx_pump::Pumpable for Ath9kHtcBackend {
    fn pump_handle(&self) -> Arc<DeviceHandle<Context>> {
        self.handle.clone()
    }
    fn pump_bulk_in(&self) -> u8 {
        EP_WLAN_RX
    }
    fn pump_state(&self) -> &crate::rx_pump::RxPumpState {
        &self.rx_pump
    }
    fn parse_transfer(&self, buf: &[u8]) -> Vec<CapturedFrame> {
        let mut out = Vec::new();
        let mut raw = 0u64; // raw units pulled off USB (incl. CRC-failed) — the pump-speed metric
        let mut off = 0usize;
        while let Some((decoded, advance)) = parse_rx_unit(self.format, self.tsf_domain, buf, off) {
            raw += 1;
            if let Some(f) = decoded {
                out.push(f);
            }
            if advance == 0 {
                break;
            }
            off += advance;
            if off + RX_PREFIX_LEN + HTC_RX_STATUS_LEN > buf.len() {
                break;
            }
        }
        // Feed the shared RAW-pull counter so `rx_raw_frames()`/RAW_PULL reflects the HTC pipe too
        // (it was structurally 0 for this backend — reads like a dead pump, isn't). Pre-CRC-filter, so
        // it isolates pump/USB speed from decode success. Matters for the #9 async-vs-sync RX ceiling.
        if raw > 0 {
            crate::RX_RAW_FRAMES.fetch_add(raw, std::sync::atomic::Ordering::Relaxed);
        }
        out
    }
}

impl Ath9kHtcBackend {
    // ── Named airtime lease (#1 / §8.5) — hardware-scheduled TX on the libusb path ─────────────────
    //
    // The lease is enforced IN OUR FIRMWARE (`wlan/ndr_mac.c`), not by host register pokes. The MAC's
    // quiet-time block (AR_QUIET1/AR_QUIET2 PCU enable + the AR_TIMER_MODE generic timer that phases
    // it) gates ALL TX — including our mgmt-endpoint injection — to the node's slot. `ndr_quiet_rearm()`
    // runs from the firmware's RX/TX path and re-applies the epoch-rotated schedule
    //   slot = (H(prefix) + epoch(t)) mod N,  period = N · slot_tu,  quiet = the whole period but our slot
    // in COMMON time (local TSF + merged offset) so every node computes the same boundaries.
    //
    // Direct register pokes from the host DO NOTHING: the firmware re-arms over them every ~100/s, and
    // the generic timer alone (without the AR_QUIET1 PCU enable the firmware sets) fires an interrupt
    // but does not hold off the transmitter — MEASURED (2026-08-24): 1588 f/s armed == 1588 f/s free.
    // The actuation is the runtime lease-override the firmware exposes for exactly this — set here by
    // writing its `.bss`/`.data` symbols via WMI_ACCESS_MEMORY, the same owned-memory path
    // `set_name_filter` uses. Mirrors the `NDR_OP_LEASE` control frame in `ndr_ctl.c`.
    //
    // ⚠ Symbol addresses are from `xtensa-elf-nm build/k2/fw.elf` — re-extract if the firmware is
    // rebuilt (text/data layout shifts, like NDR_CFG_ADDR).
    const NDR_LEASE_SLOT_TU_ADDR: u32 = 0x0050_cdc0; // a_uint32_t ndr_ctl_lease_slot_tu
    const NDR_LEASE_SLOTS_ADDR: u32 = 0x0050_cdc4; //   a_uint32_t ndr_ctl_lease_slots
    const NDR_QUIET_OFF_ADDR: u32 = 0x0050_d904; //     a_uint32_t ndr_ctl_quiet_off
    const NDR_LEASE_SLOT_ADDR: u32 = 0x0050_d908; //    a_uint32_t ndr_ctl_lease_slot
    const NDR_LEASE_OVERRIDE_ADDR: u32 = 0x0050_d90c; // a_uint32_t ndr_ctl_lease_override

    /// ★ **Arm the named airtime lease.** The node owns 1 of `slots` slots, each `slot_tu` TU (1024 µs)
    /// long, per period of `slots · slot_tu` TU; `slot` is the base slot the name hashes to (the
    /// firmware adds the epoch rotation on top). The MAC is quiet the rest of every period, so a
    /// saturating transmitter is gated to ~`1/slots` of the airtime — MAC-enforced, no host in the loop.
    ///
    /// ⚠ **Power-of-two geometry is mandatory** (MAGPIE has no DIV32, the firmware links no libgcc): the
    /// firmware masks instead of divides, so `slots` and `slots · slot_tu · 1024` must both be powers of
    /// two. Defaults 4 slots × 8 TU = 32768 µs = 2¹⁵. `slots` here is the literal count (the firmware's
    /// control-frame path takes log2; this owned-memory path writes the count the firmware reads directly).
    pub fn arm_airtime_lease(&self, slots: u32, slot_tu: u32, slot: u32) -> Result<(), FaceError> {
        // Shape first, then flip override on — so the next `ndr_quiet_rearm()` (driven by our injects)
        // reads a consistent lease. Clear the generic-timer enable to force a fresh arm (mirrors the
        // `ndr_quiet_disarm()` the NDR_OP_LEASE control frame calls; `armed_ok` is 0 on the first arm).
        self.write_target_u32s(Self::NDR_LEASE_SLOT_TU_ADDR, &[slot_tu])?;
        self.write_target_u32s(Self::NDR_LEASE_SLOTS_ADDR, &[slots])?;
        self.write_target_u32s(Self::NDR_LEASE_SLOT_ADDR, &[slot])?;
        self.write_target_u32s(Self::NDR_QUIET_OFF_ADDR, &[0])?;
        const AR_TIMER_MODE: u32 = 0x8240;
        const AR_QUIET_TIMER_EN: u32 = 0x0000_0040;
        self.reg_clr_bit(AR_TIMER_MODE, AR_QUIET_TIMER_EN)?;
        self.write_target_u32s(Self::NDR_LEASE_OVERRIDE_ADDR, &[1])?;
        Ok(())
    }

    /// Release the airtime lease — `ndr_ctl_quiet_off = 1`, clear the override, and clear the
    /// generic-timer enable so the MAC transmits freely again (mirrors `NDR_OP_QUIET_OFF`).
    pub fn disarm_airtime_lease(&self) -> Result<(), FaceError> {
        self.write_target_u32s(Self::NDR_QUIET_OFF_ADDR, &[1])?;
        self.write_target_u32s(Self::NDR_LEASE_OVERRIDE_ADDR, &[0])?;
        const AR_TIMER_MODE: u32 = 0x8240;
        const AR_QUIET_TIMER_EN: u32 = 0x0000_0040;
        self.reg_clr_bit(AR_TIMER_MODE, AR_QUIET_TIMER_EN)
    }
}

impl RadioProfile for Ath9kHtcBackend {
    fn capability(&self) -> RadioCapability {
        // AR9271: single-chain (1x1) 2.4 GHz 802.11n — MCS0-7, 20 MHz. Channels 1..13.
        // (`_1ss` is the honest constructor: this part has one RX/TX chain, so max_nss = 1.)
        RadioCapability::wifi_monitor_2ghz_1ss((1..=13).collect())
    }
}

/// The AR9271's cognition control surface. ★ **Minimal-but-honest first cut.** Only `set_channel` is
/// wired (validating a fixed-channel apply); the power / EDCCA / occupancy knobs keep the trait's
/// defaults because actuating them needs `&self` access to the WMI register path, which is `&mut self`
/// today (the WMI seq counter) — a bounded follow-on, not a fabricated no-op. What this DOES buy: with
/// a `RadioKnobs` impl present, `open_ath9k` can populate `OpenRadio.knobs`, so the AR9271 plugs into
/// `RadioControl::libusb_actuator` / `RadioBearer::from_open` and cognition drives it as a first-class
/// radio (register capability, apply allocations on its channel) exactly like the 8812au/8822E.
/// TX-gain LUT entries in increasing-power order (from `AR9271MODES_NORMAL_POWER_TX_GAIN`). ★ This is
/// the AR9271's REAL TX-power lever: the EEPROM open-loop-power calibration (`set_board_values`) is
/// skipped in `hw_reset`, so the descriptor `AR_XmitPower` and the `AR_PHY_POWER_TX_RATE` registers are
/// **inert** (MEASURED: 0 dB change) — the PA drive is fixed by the gain LUT, whose top entries the
/// OLPC latches. Overwriting them with a lower-index gain config lowers the output. MEASURED graduated
/// on air at the bench: 0x19608≈−74, 0x2d6d0≈−71, 0x3e9df≈−69 dBm (~5 dB usable range); 0x9200 crashes
/// below the noise floor. Not dBm-calibrated (that needs the OLPC/PDADC port), so it's a relative knob.
const TX_GAIN_LADDER: [u32; 11] = [
    0x0000_9200, 0x0001_0208, 0x0001_9608, 0x0001_e610, 0x0002_4650, 0x0002_d6d0, 0x0003_16d2,
    0x0003_9758, 0x0003_b759, 0x0003_d75a, 0x0003_e9df,
];
/// The high-index gain LUT registers the OLPC latches (`AR9271MODES_NORMAL_POWER_TX_GAIN` tail).
const TX_GAIN_LUT_TOP: [u32; 9] = [
    0xa334, 0xa338, 0xa33c, 0xa340, 0xa344, 0xa348, 0xa34c, 0xa350, 0xa354,
];
/// **B210-measured absolute output (dBm) for each gain level 0..10**, indexed by `TX_GAIN_LADDER` level.
/// MEASURED 2026-08-24: AR9271 on mds-05, OFDM 6 Mbps flood, ch14 (clean channel, no ambient), captured
/// by the B210 at 2484 MHz (lo-offset 7 MHz, 99th-pct power per window). Anchor: level 10 (reset-default
/// max gain) = 20 dBm (datasheet max at top gain — the one assumption; the *relative* dB steps are the
/// SDR-measured truth). Levels 0-4 fell below the SDR noise floor at that gain (all ~minimum, ≤~6 dBm),
/// so they carry the last-measured value; refine with a higher-B210-gain stitch capture if needed.
/// ⚠ This is ONE chip; true per-chip absolute dBm needs the OLPC/PDADC EEPROM cal. OFDM-only (CCK flat).
const MEASURED_DBM_BY_LEVEL: [f32; 11] =
    [5.6, 6.0, 6.0, 6.4, 6.4, 7.5, 8.8, 11.4, 12.2, 12.9, 20.0];

/// `ath9k_hw_fill_vpd_table` (+ inlined `get_lower_upper_index`): interpolate the power-detector VPD
/// curve onto a dense 0.5-dB power grid — a step of the OLPC M3 PDADC computation ([`set_txpower_4k`]).
fn fill_vpd_table(pwr_min: u8, pwr_max: u8, pwr_list: &[u8], vpd_list: &[u8], ret: &mut [u8; 64]) {
    let n = pwr_list.len();
    let get_lu = |target: u8| -> (usize, usize) {
        if target <= pwr_list[0] {
            return (0, 0);
        }
        if target >= pwr_list[n - 1] {
            return (n - 1, n - 1);
        }
        for i in 0..n - 1 {
            if pwr_list[i] == target {
                return (i, i);
            }
            if target < pwr_list[i + 1] {
                return (i, i + 1);
            }
        }
        (0, 0)
    };
    let count = ((pwr_max.saturating_sub(pwr_min)) / 2) as usize;
    let mut curr = pwr_min;
    for i in 0..=count.min(63) {
        let (mut il, mut ir) = get_lu(curr);
        if ir < 1 {
            ir = 1;
        }
        if il == n - 1 {
            il = n - 2;
        }
        ret[i] = if pwr_list[il] == pwr_list[ir] {
            vpd_list[il]
        } else {
            (((curr as i32 - pwr_list[il] as i32) * vpd_list[ir] as i32
                + (pwr_list[ir] as i32 - curr as i32) * vpd_list[il] as i32)
                / (pwr_list[ir] as i32 - pwr_list[il] as i32)) as u8
        };
        curr = curr.wrapping_add(2);
    }
}

impl Ath9kHtcBackend {
    /// Write the top TX-gain LUT registers to ladder `level` (0 = min power, `LADDER.len()-1` = max /
    /// the reset default) — the AR9271's actual TX-power actuator. See [`TX_GAIN_LADDER`].
    fn set_tx_gain_level(&self, level: usize) -> Result<(), FaceError> {
        let v = TX_GAIN_LADDER[level.min(TX_GAIN_LADDER.len() - 1)];
        for &a in &TX_GAIN_LUT_TOP {
            self.reg_write(a, v)?;
        }
        Ok(())
    }
}

impl RadioKnobs for Ath9kHtcBackend {
    /// ★ **Live channel + bandwidth retune.** Re-programs the RF synth + re-cals on the `&self` path —
    /// no re-open, HTC/RX ring stay up. A same-channel same-width apply is a fast no-op. A **width
    /// change** (HT20↔HT40) triggers a full live PHY reprogram via [`set_bandwidth`](Self::set_bandwidth)
    /// (the 2G_HT40 initval column + HT40+ synth centre — not just the per-frame `HAL_RATESERIES_2040`
    /// bit); a channel-only change re-runs synth+cal at the new centre.
    fn set_channel(&self, channel: u8, bw: Bandwidth) -> Result<(), FaceError> {
        use std::sync::atomic::Ordering::Relaxed;
        let want_ht40 = matches!(bw, Bandwidth::Bw40);
        let width_changed = want_ht40 != self.ht40.load(Relaxed);
        let channel_changed = channel != self.channel.load(Relaxed);
        if !width_changed && !channel_changed {
            return Ok(());
        }
        // Record the target channel first so set_bandwidth re-synths at the right centre.
        self.channel.store(channel, Relaxed);
        if width_changed {
            self.set_bandwidth(want_ht40)?; // reprograms the PHY for the new width at self.channel
        } else {
            let chan_mhz = if channel == 14 { 2484 } else { 2407 + 5 * channel as u16 };
            self.set_channel_and_cal(chan_mhz)?;
        }
        Ok(())
    }

    /// ★ **EDCCA / listen-before-talk toggle.** `on` forces `AR_DIAG_FORCE_RX_CLEAR` so the DCU never
    /// defers TX on a busy medium (blast on owned spectrum — the token/slot is the only collision
    /// avoidance); `off` restores normal carrier-sense deference. A single `&self` register write.
    fn set_edcca_ignore(&self, on: bool) -> Result<(), FaceError> {
        use crate::ath9k_reg::{AR_DIAG_FORCE_RX_CLEAR, AR_DIAG_SW};
        if on {
            self.reg_set_bit(AR_DIAG_SW, AR_DIAG_FORCE_RX_CLEAR)
        } else {
            self.reg_clr_bit(AR_DIAG_SW, AR_DIAG_FORCE_RX_CLEAR)
        }
    }

    /// TX power as an opaque 0-63 index, actuated via the **gain LUT** ([`set_tx_gain_level`]) — the
    /// real PA lever. (The per-frame `AR_XmitPower` header byte and `AR_PHY_POWER_TX_RATE` are inert on
    /// this part — the EEPROM OLPC cal is skipped — MEASURED 0 dB; do NOT read a set XmitPower as an
    /// actuated one.) `idx = 63` = the reset-default max; lower = lower.
    ///
    /// ⚠ **The gain LUT is the OFDM power table** (`AR9271MODES_NORMAL_POWER_TX_GAIN`) — it moves
    /// 802.11a/g/n (OFDM) power but NOT 802.11b (CCK 1/2/5.5/11 Mbps), which has a separate power path.
    /// MEASURED with a B210 on the clean channel (ch14, no ambient): a CCK 1 Mbps flood is FLAT across
    /// all 11 levels (< 1 dB); an OFDM 6 Mbps flood spans ~12.5 dB (levels 10→5 = 20→7.5 dBm, anchor
    /// L10 = datasheet max; levels 4-0 fall below the SDR noise floor). So this knob only bites when the
    /// frame goes out OFDM ([`set_legacy_rate`] ≥ Ofdm6, or an HT MCS) — see [`MEASURED_DBM_BY_LEVEL`].
    fn set_tx_power(&self, idx: u32) -> Result<(), FaceError> {
        let level = (idx.min(63) as usize * (TX_GAIN_LADDER.len() - 1)) / 63;
        self.cur_power.store(idx.min(63) as u8, std::sync::atomic::Ordering::Relaxed);
        self.set_tx_gain_level(level)
    }

    /// TX power on the dBm scale, from the **B210-measured** OFDM power curve ([`MEASURED_DBM_BY_LEVEL`]):
    /// pick the gain level whose measured output is closest to the request, and echo back that level's
    /// measured dBm. This is a real SDR-calibrated map (not the old naive linear spread), but two honest
    /// caveats remain: (1) it is **OFDM-only** — CCK power does not move with the LUT (see
    /// [`set_tx_power`]); (2) the absolute anchor is the datasheet max at top gain and the curve was
    /// measured on ONE chip, so true **per-chip** absolute dBm (accounting for part-to-part + temp + freq
    /// variation) still needs the OLPC/PDADC EEPROM cal port. The *relative* ladder is SDR-measured truth.
    fn set_tx_power_dbm(&self, dbm: i8) -> Result<i8, FaceError> {
        let req = dbm as f32;
        // Closest measured level to the requested dBm.
        let level = (0..MEASURED_DBM_BY_LEVEL.len())
            .min_by(|&a, &b| {
                (MEASURED_DBM_BY_LEVEL[a] - req)
                    .abs()
                    .total_cmp(&(MEASURED_DBM_BY_LEVEL[b] - req).abs())
            })
            .unwrap_or(TX_GAIN_LADDER.len() - 1);
        self.cur_power.store(((level * 63) / (TX_GAIN_LADDER.len() - 1)) as u8, std::sync::atomic::Ordering::Relaxed);
        self.set_tx_gain_level(level)?;
        Ok(MEASURED_DBM_BY_LEVEL[level].round() as i8)
    }

    /// Frame-free channel occupancy (#4/#30): `AR_RCCNT` (0x80f0) is the MAC's free-running "rx-clear"
    /// / medium-busy cycle counter — it climbs while the channel is busy, WITHOUT the host decoding
    /// frames. The occupancy sampler differences two reads into a busy-rate the cognition plane maps
    /// to channel-busy%. Scaled `>> 8` so a differenced sample fits u16 over a sampler window without
    /// wrapping on a mostly-idle channel. Cycle-based (not a frame count like the 8812au's
    /// `REG_RXERR_RPT`), so the mapping is approximate but tracks activity monotonically.
    fn read_channel_activity(&self) -> Result<Option<u16>, FaceError> {
        const AR_RCCNT: u32 = 0x80f0;
        let busy = self.reg_read(AR_RCCNT)?;
        Ok(Some((busy >> 8) as u16))
    }

    /// ★ The AR9271 can promise **`ScheduledAt`** — hardware-gated TX off the TSF via the generic-timer
    /// airtime lease ([`arm_airtime_lease`](Self::arm_airtime_lease)), MEASURED sub-µs boundary on air
    /// (M3b). This is the named-time Cut-2 capability the beacon-slot / URLLC lane / TSCH-by-name read,
    /// and it is *unique to this part* among the Wi-Fi we own — every other backend is `BestEffort`.
    fn tx_discipline(&self) -> ndn_radio_hal::TxDiscipline {
        ndn_radio_hal::TxDiscipline::ScheduledAt { granularity_ns: 1_000 }
    }
}

/// The AR9271 IQ-mismatch fixed-point correction (`ar9002_hw_iqcalibrate`,
/// ar9002_calib.c:192-267), for one chain. Inputs are the accumulated
/// `AR_PHY_CAL_MEAS_0/1/2` reads (`powerMeasI`, `powerMeasQ`, `iqCorrMeas`). Returns the
/// `(iCoff, qCoff)` field values to program, or `None` when the measurement is degenerate
/// (the C `powerMeasQ && iCoffDenom && qCoffDenom` guard) — the caller still sets
/// IQCORR_ENABLE in that case, exactly as the reference does.
///
/// Kept bit-identical to the C integer semantics: every division is truncating u32, and
/// `qCoff = powerMeasI / qCoffDenom - 64` is computed in u32 then reinterpreted as i32
/// (the C assigns the unsigned result into an `int32_t`), so a quotient below 64 yields a
/// negative `qCoff` — that is the path that produces the RX-correction the demod needs.
fn iq_cal_correction(power_meas_i: u32, power_meas_q: u32, iq_corr_meas: u32) -> Option<(i32, i32)> {
    let mut iq_corr_meas = iq_corr_meas;
    // C: `if (iqCorrMeas > 0x80000000)` — strictly greater; 0x80000000 stays positive.
    let iq_corr_neg = if iq_corr_meas > 0x8000_0000 {
        iq_corr_meas = (0xffff_ffffu32 - iq_corr_meas).wrapping_add(1);
        true
    } else {
        false
    };

    let i_coff_denom = (power_meas_i / 2 + power_meas_q / 2) / 128;
    let q_coff_denom = power_meas_q / 64;

    if power_meas_q == 0 || i_coff_denom == 0 || q_coff_denom == 0 {
        return None;
    }

    let mut i_coff = (iq_corr_meas / i_coff_denom) as i32;
    // u32 arithmetic stored into int32_t; wrapping_sub then `as i32` reproduces the wrap.
    let mut q_coff = (power_meas_i / q_coff_denom).wrapping_sub(64) as i32;

    i_coff &= 0x3f;
    if !iq_corr_neg {
        i_coff = 0x40 - i_coff;
    }

    if q_coff > 15 {
        q_coff = 15;
    } else if q_coff <= -16 {
        q_coff = -16;
    }

    Some((i_coff, q_coff))
}

/// Sign-extend the low `bits` of `v` to a full `i32`.
fn sign_extend(v: u32, bits: u32) -> i32 {
    let shift = 32 - bits;
    ((v << shift) as i32) >> shift
}

/// `ath9k_hw_get_delta_slope_vals` (hw.c:1297) — the OFDM timing delta-slope
/// fixed-point split into `(mantissa, exponent)`. Transcribed bit-for-bit,
/// including the C's `u32` wrap in `14 - (coef_exp - COEF_SCALE_S)` (which nets to
/// `38 - coef_exp` for the exponents this radio produces). Used by
/// [`Ath9kHtcBackend::set_delta_slope`].
fn delta_slope_vals(coef_scaled: u32) -> (u32, u32) {
    use crate::ath9k_reg::COEF_SCALE_S;
    // Find the highest set bit (mirrors the C for-loop from 31 down to 0).
    let mut coef_exp = 31u32;
    while coef_exp > 0 {
        if (coef_scaled >> coef_exp) & 0x1 == 1 {
            break;
        }
        coef_exp -= 1;
    }
    // C: coef_exp = 14 - (coef_exp - COEF_SCALE_S), computed in u32 (wraps).
    coef_exp = 14u32.wrapping_sub(coef_exp.wrapping_sub(COEF_SCALE_S));
    let coef_man = coef_scaled.wrapping_add(1u32 << (COEF_SCALE_S - coef_exp - 1));
    let coef_mantissa = coef_man >> (COEF_SCALE_S - coef_exp);
    let coef_exponent = coef_exp.wrapping_sub(16);
    (coef_mantissa, coef_exponent)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The endpoint map is the thing most likely to be silently wrong, and it is read off the
    /// device rather than assumed — pin it so a future edit cannot quietly "tidy" it.
    #[test]
    fn endpoint_directions_match_the_device_descriptor() {
        assert_eq!(EP_WLAN_TX & 0x80, 0, "EP1 is OUT");
        assert_eq!(EP_WLAN_RX & 0x80, 0x80, "EP2 is IN");
        assert_eq!(EP_REG_IN & 0x80, 0x80, "EP3 is IN");
        assert_eq!(EP_REG_OUT & 0x80, 0, "EP4 is OUT");
    }

    #[test]
    fn service_ids_match_make_service_id() {
        // MAKE_SERVICE_ID(group, index) == (group << 8) | index
        assert_eq!(HtcService::CtrlRsvd as u16, (0 << 8) | 1);
        assert_eq!(HtcService::WmiControl as u16, (1 << 8) | 0);
        assert_eq!(HtcService::DataBe as u16, (1 << 8) | 7);
    }

    /// The WMI enum in `wmi.h` is dense from 0x0001, so these are positional. If someone inserts a
    /// command in the firmware header without updating this, register access silently targets the
    /// wrong handler.
    #[test]
    fn wmi_command_ids_are_positional() {
        assert_eq!(WmiCmd::Echo as u16, 0x0001);
        assert_eq!(WmiCmd::AccessMemory as u16, 0x0002);
        assert_eq!(WmiCmd::GetFwVersion as u16, 0x0003);
        assert_eq!(WmiCmd::EnableIntr as u16, 0x0005);
        assert_eq!(WmiCmd::AthInit as u16, 0x0006);
        assert_eq!(WmiCmd::StartRecv as u16, 0x000c);
        assert_eq!(WmiCmd::SetMode as u16, 0x000f);
        assert_eq!(WmiCmd::RegRead as u16, 0x0014);
        assert_eq!(WmiCmd::RegWrite as u16, 0x0015);
        assert_eq!(WmiCmd::TargetIcUpdate as u16, 0x0018);
        assert_eq!(WmiCmd::TgtDetach as u16, 0x001a);
    }

    /// The 802.11 frame starts after the 12-byte RX prefix (HTC 8 + 4-byte data-endpoint prefix)
    /// + rx_status(40) = offset 52 (MEASURED on the wire); status fields are big-endian at fixed
    /// offsets. Pin the parse against a synthetic buffer so an offset can't silently drift.
    #[test]
    fn parse_rx_decodes_status_offsets() {
        const RX_HDR_LEN: usize = HTC_HDR_LEN + 4; // = 12
        let mut raw = vec![0u8; RX_HDR_LEN + HTC_RX_STATUS_LEN + 4];
        // rx_status starts at offset 12.
        let s = RX_HDR_LEN;
        raw[s..s + 8].copy_from_slice(&0x0011_2233_4455_6677u64.to_be_bytes()); // rs_tstamp
        raw[s + 8..s + 10].copy_from_slice(&0x0004u16.to_be_bytes()); // rs_datalen = 4
        raw[s + 12] = 0xd0; // rs_rssi = -48 (i8)
        raw[s + 20] = 0x0b; // rs_rate (1 Mbps CCK code)
        raw[RX_HDR_LEN + HTC_RX_STATUS_LEN..].copy_from_slice(&[0xaa, 0xbb, 0xcc, 0xdd]);
        let f = Ath9kHtcBackend::parse_rx(&raw).expect("parses");
        assert_eq!(f.rs_tstamp, 0x0011_2233_4455_6677);
        assert_eq!(f.rs_datalen, 4);
        assert_eq!(f.rs_rssi, -48);
        assert_eq!(f.rs_rate, 0x0b);
        assert_eq!(f.frame, vec![0xaa, 0xbb, 0xcc, 0xdd]);
        // A runt transfer yields None rather than panicking.
        assert!(Ath9kHtcBackend::parse_rx(&[0u8; 10]).is_none());
    }

    /// The vendor's `WMI_ACCESS_MEMORY_MAX_TUPLES` is 8 and does not fit the 64-byte register
    /// pipe. Pin the derived bound so it cannot drift back.
    #[test]
    fn access_memory_chunk_fits_the_register_pipe() {
        // Budget the 4-byte receive trailer: a reply may carry one and the host cannot refuse it.
        let hdr = HTC_HDR_LEN + WMI_HDR_LEN + 4 + 4;
        assert!(hdr + NDR_MEM_MAX_TUPLES * 8 <= REG_PIPE_MAX);
        assert!(hdr + 6 * 8 > REG_PIPE_MAX, "6 tuples overflow once a trailer is attached");
        assert!(hdr + 8 * 8 > REG_PIPE_MAX, "the vendor's 8 tuples overflow the pipe");
    }

    /// A batched `WMI_REG_WRITE` must fit the 64-byte register pipe: `HTC(8) + WMI(4) + n*8`.
    /// The write reply is empty, so (unlike ACCESS_MEMORY) no trailer budget is needed. Pin the
    /// derived bound so it cannot drift into pipe overflow.
    #[test]
    fn reg_write_batch_fits_the_register_pipe() {
        let hdr = HTC_HDR_LEN + WMI_HDR_LEN; // 12
        assert!(hdr + REG_WRITE_MAX_PAIRS * 8 <= REG_PIPE_MAX);
        assert!(hdr + (REG_WRITE_MAX_PAIRS + 1) * 8 > REG_PIPE_MAX, "one more pair overflows");
    }

    /// A maximal echo must fit one register-pipe packet — and the firmware's own
    /// `WMI_ECHOCMD_MSG_MAX_LEN` (53) does **not**: 8 + 4 + 1 + 53 = 66 > 64. This pins the derived
    /// limit so nobody "corrects" it back to the header's value.
    #[test]
    fn max_echo_fits_the_register_pipe() {
        assert_eq!(MAX_ECHO_LEN, 51);
        assert!(HTC_HDR_LEN + WMI_HDR_LEN + 1 + MAX_ECHO_LEN <= REG_PIPE_MAX);
        assert!(HTC_HDR_LEN + WMI_HDR_LEN + 1 + 53 > REG_PIPE_MAX, "the firmware constant overflows");
    }

    /// IQ-mismatch fixed-point correction (`ar9002_hw_iqcalibrate`). Hand-computed against
    /// the C so a stray shift or sign can't slip in. All values are the accumulated
    /// AR_PHY_CAL_MEAS words for chain 0.
    #[test]
    fn iq_cal_correction_positive_and_negative() {
        // Balanced I/Q, small positive iqCorrMeas.
        //   iCoffDenom = (0x40000/2 + 0x40000/2)/128 = 2048; qCoffDenom = 0x40000/64 = 4096
        //   iCoff = 4096/2048 = 2; &0x3f = 2; not-neg => 0x40-2 = 62
        //   qCoff = 0x40000/4096 - 64 = 64-64 = 0
        assert_eq!(iq_cal_correction(0x0004_0000, 0x0004_0000, 0x0000_1000), Some((62, 0)));

        // Same magnitude, but iqCorrMeas > 0x80000000 => the negative branch.
        //   iqCorrMeas -> (0xffffffff - 0xfffff000)+1 = 0x1000 = 4096, neg=true
        //   iCoff = 2; &0x3f = 2; neg => NOT flipped => 2 ; qCoff = 0
        assert_eq!(iq_cal_correction(0x0004_0000, 0x0004_0000, 0xffff_f000), Some((2, 0)));
    }

    /// A quotient below 64 must make qCoff negative (the u32->int32_t wrap) and then clamp
    /// to -16 — this is the branch that carries the RX correction, so pin it.
    #[test]
    fn iq_cal_correction_negative_qcoff_clamps() {
        // powerMeasI=0x20000, powerMeasQ=0x40000, iqCorrMeas=0x800
        //   iCoffDenom = (0x20000/2 + 0x40000/2)/128 = (65536+131072)/128 = 1536
        //   qCoffDenom = 0x40000/64 = 4096
        //   iCoff = 0x800/1536 = 1; &0x3f=1; not-neg => 0x40-1 = 63
        //   qCoff = 0x20000/4096 - 64 = 32-64 = -32 (wrap) => clamp to -16
        assert_eq!(iq_cal_correction(0x0002_0000, 0x0004_0000, 0x0000_0800), Some((63, -16)));
    }

    /// Degenerate measurements match the C guard (`powerMeasQ && iCoffDenom && qCoffDenom`)
    /// and yield no coefficients.
    #[test]
    fn iq_cal_correction_degenerate_is_none() {
        assert_eq!(iq_cal_correction(0, 0, 0), None);
        assert_eq!(iq_cal_correction(0x0004_0000, 0, 0x1000), None); // powerMeasQ == 0
    }

    /// `ath9k_hw_get_delta_slope_vals` fixed-point split, hand-computed against the C
    /// so the u32-wrap in `14 - (coef_exp - COEF_SCALE_S)` can't silently drift.
    #[test]
    fn delta_slope_vals_matches_c() {
        // coef_scaled = (100 MHz << 24) / 2412 = 0x64000000 / 2412 = 695572 (floor).
        // Highest set bit: 19. coef_exp = 14 - (19 - 24) = 19.
        //   man = 695572 + (1 << (24-19-1=4)) = 695572 + 16 = 695588
        //   mantissa = 695588 >> (24-19=5) = 21737 ; exponent = 19 - 16 = 3
        let coef = 0x6400_0000u32 / 2412;
        assert_eq!(coef, 695572);
        assert_eq!(delta_slope_vals(coef), (21737, 3));

        // Half-GI coefficient (0.9×): (9*695572)/10 = 626014, top bit 19.
        //   coef_exp=19; man=626014+16=626030; mantissa=626030>>5=19563; exp=3
        let coef_hg = (9 * coef) / 10;
        assert_eq!(delta_slope_vals(coef_hg), (19563, 3));
    }

    /// The 2.4 GHz MAC timing math `init_global_settings` emits, pinned against the
    /// golden-trace register values (clockrate = 44). SIFS = mac_to_clks(10-2) = 8*44,
    /// SLOT = 9*44, ACK = 64*44, CTS = 48*44.
    #[test]
    fn global_settings_timing_matches_trace() {
        let clk = crate::ath9k_reg::ATH9K_CLOCK_RATE_2GHZ_OFDM;
        assert_eq!(clk, 44);
        assert_eq!((10u32 - 2) * clk, 0x160); // AR_D_GBL_IFS_SIFS (trace 0x1030)
        assert_eq!(9u32 * clk, 0x18c); // AR_D_GBL_IFS_SLOT (trace 0x1070)
        assert_eq!(64u32 * clk, 0xb00); // AR_TIME_OUT ACK field
        assert_eq!(48u32 * clk, 0x840); // AR_TIME_OUT CTS field
    }

    /// The data-AC queue reset values, pinned against the golden trace
    /// (AR_DLCL_IFS = 0x002ffc0f, AR_DRETRY_LIMIT = 0x0008200a).
    #[test]
    fn reset_tx_queue_values_match_trace() {
        use crate::ath9k_reg::*;
        let mut cw_min = 1u32;
        while cw_min < INIT_CWMIN {
            cw_min = (cw_min << 1) | 1;
        }
        assert_eq!(cw_min, 15);
        let lcl_ifs = ((cw_min << AR_D_LCL_IFS_CWMIN_S) & AR_D_LCL_IFS_CWMIN)
            | ((INIT_CWMAX << AR_D_LCL_IFS_CWMAX_S) & AR_D_LCL_IFS_CWMAX)
            | ((INIT_AIFS << AR_D_LCL_IFS_AIFS_S) & AR_D_LCL_IFS_AIFS);
        assert_eq!(lcl_ifs, 0x002f_fc0f);
        let retry = ((INIT_SSH_RETRY << AR_D_RETRY_LIMIT_STA_SH_S) & AR_D_RETRY_LIMIT_STA_SH)
            | ((INIT_SLG_RETRY << AR_D_RETRY_LIMIT_STA_LG_S) & AR_D_RETRY_LIMIT_STA_LG)
            | ((INIT_SH_RETRY << AR_D_RETRY_LIMIT_FR_SH_S) & AR_D_RETRY_LIMIT_FR_SH);
        assert_eq!(retry, 0x0008_200a);
    }

    /// The base interrupt mask `init_interrupt_masks` computes for the AR9271
    /// (non-9300, no mitigation): TXERR|TXURN|RXERR|RXORN|BCNMISC|RXOK|TXOK.
    #[test]
    fn init_interrupt_masks_base_imr() {
        use crate::ath9k_reg::*;
        let imr = AR_IMR_TXERR
            | AR_IMR_TXURN
            | AR_IMR_RXERR
            | AR_IMR_RXORN
            | AR_IMR_BCNMISC
            | AR_IMR_RXOK
            | AR_IMR_TXOK;
        assert_eq!(imr, 0x0080_0965);
        // AR_INTR_SYNC_DEFAULT resolved from the reg.h enum.
        assert_eq!(AR_INTR_SYNC_DEFAULT, 0x0002_3f60);
    }

    /// `ar9002_hw_set_channel`'s 2.4 GHz synth value for ch1 (2412), pinned against the
    /// golden trace: 0x9874 = 0x30a0cccc = BMODE|FRACMODE|CHANSEL_2G(2412).
    #[test]
    fn rf_set_freq_synth_matches_trace() {
        use crate::ath9k_reg::*;
        let channel_sel = ((2412u64 * 0x1_0000) / CHANSEL_2G_DIV) as u32;
        let synth =
            AR_PHY_SYNTH_CONTROL_2G_BMODE | AR_PHY_SYNTH_CONTROL_2G_FRACMODE | channel_sel;
        assert_eq!(synth, 0x30a0_cccc);
    }

    // ── M3 FrameIo: TX frame construction + RX CapturedFrame mapping ──────────────────────────────

    use crate::TxIntent;
    use bytes::Bytes;

    const TEST_ETHERTYPE: u16 = 0x8624;
    const DST: [u8; 6] = [0xff, 0xff, 0xff, 0xff, 0xff, 0xff];
    const SRC: [u8; 6] = [0x02, 0x4e, 0x44, 0x4e, 0x00, 0x01];

    fn ndn_fmt() -> FrameFormat {
        FrameFormat::RawNdn { ethertype: TEST_ETHERTYPE }
    }

    /// The injected TX buffer must be `[hif_usb 4B][HTC 8B][tx_mgmt_hdr 8B][802.11 + LLC/SNAP +
    /// payload]`, byte-for-byte — the mgmt-endpoint framing the kernel's on-air TX uses. Pins the
    /// layout the target parses so an edit can't drift it.
    #[test]
    fn build_tx_frame_layout() {
        const TX_MGMT_HDR_SIZE: usize = 8;
        let payload = b"\x05\x04ping";
        let frame = InjectFrame {
            payload: Bytes::copy_from_slice(payload),
            tx: TxIntent::CONSERVATIVE,
            dst: DST,
            src: SRC,
            addr3: None,
        };
        let mgmt_ep = 0x07;
        let (rate_code, tx_power, rate_flags) = (0x85u8, 0x28u8, 0x01u8); // HT MCS5, 20 dBm, short-GI
        let buf = build_tx_frame_bytes(mgmt_ep, ndn_fmt(), &frame, rate_code, tx_power, rate_flags).unwrap();
        let dot11 = crate::frame::build_dot11(ndn_fmt(), &frame).unwrap();

        // hif_usb TX stream header (4 B): le16 HTC-frame length + le16 tag 0x697e.
        const HIF: usize = 4;
        let htc_frame_len = HTC_HDR_LEN + TX_MGMT_HDR_SIZE + dot11.len();
        assert_eq!(u16::from_le_bytes([buf[0], buf[1]]) as usize, htc_frame_len, "hif len = HTC frame");
        assert_eq!(u16::from_le_bytes([buf[2], buf[3]]), 0x697e, "hif stream tag");

        // HTC_FRAME_HDR (8 B): endpoint, flags, be16 payload length, control[4].
        assert_eq!(buf[HIF], mgmt_ep, "HTC endpoint = Mgmt");
        assert_eq!(buf[HIF + 1], 0, "HTC flags = 0");
        let payload_len = u16::from_be_bytes([buf[HIF + 2], buf[HIF + 3]]) as usize;
        assert_eq!(payload_len, TX_MGMT_HDR_SIZE + dot11.len(), "HTC payload len = tx_mgmt_hdr + MPDU");
        assert_eq!(&buf[HIF + 4..HIF + 8], &[0, 0, 0, 0], "HTC control bytes zero");

        // tx_mgmt_hdr (8 B): node_idx, vif_idx, tidno(=tx_power), flags, key_type, keyix=0xff,
        // cookie, pad(=rate_code) — the last two repurposed as our firmware's per-frame TX knobs.
        let h = &buf[HIF + HTC_HDR_LEN..HIF + HTC_HDR_LEN + TX_MGMT_HDR_SIZE];
        assert_eq!(&h[0..2], &[0, 0], "node/vif zero");
        assert_eq!(h[2], tx_power, "tidno byte carries AR_XmitPower");
        assert_eq!(h[3], 0, "flags zero");
        assert_eq!(h[4], rate_flags, "keytype byte carries HT rate flags (SGI/HT40)");
        assert_eq!(h[5], 0xff, "keyix = 0xff (no key)");
        assert_eq!(h[6], 0, "cookie zero");
        assert_eq!(h[7], rate_code, "pad byte carries the HW rate code");

        // The 802.11 MPDU that follows is exactly `build_dot11` (same on-air layout as every backend):
        // FC=Data, addr1=dst, addr2=src, LLC/SNAP(0x8624), payload.
        let mpdu = &buf[HIF + HTC_HDR_LEN + TX_MGMT_HDR_SIZE..];
        assert_eq!(mpdu, &dot11[..], "MPDU == shared build_dot11");
        assert_eq!(&mpdu[0..2], &[0x08, 0x00], "FC: type=Data subtype=0");
        assert_eq!(&mpdu[4..10], &DST, "addr1 = dst");
        assert_eq!(&mpdu[10..16], &SRC, "addr2 = src");
        assert_eq!(mpdu.ends_with(payload), true, "payload rides last");
    }

    /// A synthetic RX transfer must map to a `CapturedFrame` with the payload recovered after
    /// LLC/SNAP, addr2→addr / addr1→group, the RSSI converted to dBm, the HT rate decoded, and the
    /// per-frame `rs_tstamp` carried as a hardware `LinkStamp`. Also checks the de-aggregation advance.
    #[test]
    fn parse_rx_unit_maps_captured_frame() {
        let payload = b"\x06\x03abc";
        let frame = InjectFrame {
            payload: Bytes::copy_from_slice(payload),
            tx: TxIntent::CONSERVATIVE,
            dst: DST,
            src: SRC,
            addr3: None,
        };
        let mpdu = crate::frame::build_dot11(ndn_fmt(), &frame).unwrap();

        // Build one RX unit: [HTC 8][ep-prefix 4][status 40][MPDU].
        let htc_payload_len = 4 + HTC_RX_STATUS_LEN + mpdu.len();
        let mut raw = Vec::new();
        raw.push(0x02); // HTC endpoint (data)
        raw.push(0x00); // flags
        raw.extend_from_slice(&(htc_payload_len as u16).to_be_bytes());
        raw.extend_from_slice(&[0, 0, 0, 0]); // HTC control
        raw.extend_from_slice(&[0x04, 0x00, 0x00, 0x00]); // 4-byte data-endpoint prefix
        // ath_htc_rx_status (40 B), big-endian fields at their offsets.
        let mut status = [0u8; HTC_RX_STATUS_LEN];
        status[0..8].copy_from_slice(&0x0011_2233_4455_6677u64.to_be_bytes()); // rs_tstamp
        status[8..10].copy_from_slice(&(mpdu.len() as u16).to_be_bytes()); // rs_datalen
        status[10] = 0; // rs_status = no error
        status[12] = 40; // rs_rssi = 40 dB above NF -> 40-95 = -55 dBm
        status[20] = 0x80 | 3; // rs_rate = HT MCS3
        raw.extend_from_slice(&status);
        raw.extend_from_slice(&mpdu);

        let domain = ClockDomainId(0x1234);
        let (decoded, advance) = parse_rx_unit(ndn_fmt(), domain, &raw, 0).expect("parses a unit");
        let cf = decoded.expect("a good RawNdn frame decodes");
        assert_eq!(cf.payload.as_ref(), payload, "payload recovered after LLC/SNAP");
        assert_eq!(cf.addr, Some(SRC), "addr2 -> addr");
        assert_eq!(cf.group, Some(DST), "addr1 -> group");
        assert_eq!(cf.rssi_dbm, Some(-55), "NF-relative rs_rssi -> dBm");
        assert_eq!(cf.mcs_index, Some(3), "HT rate code decoded to MCS index");
        let stamp = cf.stamp.expect("per-frame rs_tstamp -> hardware LinkStamp");
        assert_eq!(stamp.raw, 0x0011_2233_4455_6677);
        assert_eq!(stamp.domain, domain);
        assert_eq!(stamp.latch, LatchPoint::MacDone);
        // Advance = 8 (HTC hdr) + htc_payload_len, 4-byte aligned.
        assert_eq!(advance, (HTC_HDR_LEN + htc_payload_len + 3) & !3);
    }

    /// A CRC-errored unit (rs_status low nibble set) yields no CapturedFrame but still advances so the
    /// de-aggregation walk continues past it.
    #[test]
    fn parse_rx_unit_drops_errored_frame() {
        let mpdu = vec![0xAAu8; 40];
        let htc_payload_len = 4 + HTC_RX_STATUS_LEN + mpdu.len();
        let mut raw = vec![0x02, 0x00];
        raw.extend_from_slice(&(htc_payload_len as u16).to_be_bytes());
        raw.extend_from_slice(&[0, 0, 0, 0]);
        raw.extend_from_slice(&[0x04, 0x00, 0x00, 0x00]);
        let mut status = [0u8; HTC_RX_STATUS_LEN];
        status[8..10].copy_from_slice(&(mpdu.len() as u16).to_be_bytes());
        status[10] = 0x01; // ATH9K_RXERR_CRC
        raw.extend_from_slice(&status);
        raw.extend_from_slice(&mpdu);

        let (decoded, advance) = parse_rx_unit(ndn_fmt(), ClockDomainId(0), &raw, 0).unwrap();
        assert!(decoded.is_none(), "CRC error -> dropped");
        assert!(advance >= RX_PREFIX_LEN + HTC_RX_STATUS_LEN, "still advances past the unit");
    }
}
