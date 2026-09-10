//! Minimal radiotap codec for monitor-mode injection and capture.
//!
//! Radiotap is the de-facto header that drivers prepend to captured 802.11
//! frames and honour on injected ones (<https://www.radiotap.org>). This face
//! needs exactly two operations from it:
//!
//! * **TX** — build a header that tells the driver *which rate/MCS* to transmit
//!   at. This is the lever that defeats the legacy-rate wall: an injected frame
//!   carries its own MCS in the radiotap `MCS` field, so there is no AP basic-
//!   rate floor (see the crate-level docs). wfb-ng / OpenIPC inject this exact
//!   shape to push 10–50 Mbps of "broadcast" video.
//! * **RX** — parse the header the driver prepends to a captured frame to pull
//!   the per-frame **RSSI** (and the MCS it arrived at), feeding the cross-layer
//!   signal store and the adaptive-MCS picker.
//!
//! Layout: an 8-byte fixed header (`u8 version`, `u8 pad`, `le16 len`,
//! `le32 present`) optionally followed by more `le32` present words (when the
//! extension bit is set), then the present fields in ascending bit order, each
//! aligned to its natural size **relative to the start of the header**. We
//! implement the field table up to the fields we read; if an unknown present
//! bit appears we stop walking and return what was gathered so far — RSSI
//! (bit 5) is low enough that it is always reached before vendor/extension
//! fields. The 802.11 frame always begins at byte `it_len`, so payload
//! extraction never depends on understanding every field.
//!
//! # The S1G (802.11ah / HaLow) extension — where the sub-GHz metadata lives
//!
//! Both HaLow drivers in this rig put **100% of their per-frame PHY metadata behind present
//! bit 28 (TLVs)**, which a bit-0..21 field table never reaches. The consequence before this was
//! concrete and silent: `rssi_dbm` was *always* `None` on the Newracom NRC7292 (it emits no
//! `DBM_ANTSIGNAL` at all — RSSI exists only inside the TLV) and `mcs_index` was always `None` on
//! both parts. So [`parse`] walks TLVs after the standard fields; see [`S1gInfo`].
//!
//! ★ **One decoder serves both vendors, and that is verified from their sources, not assumed.**
//! Newracom's `struct nrc_radiotap_hdr` (`nrc7292_sw_pkg` `nrc.h:455`, filled at `nrc-trx.c:1282`)
//! and Morse Micro's `struct radiotap_s1g_tlv` (`morse_driver/s1g_radiotap.h`, filled in
//! `monitor.c:morse_mon_rx`) emit a byte-identical TLV: type 32, length 6, `{le16 known, le16 data1,
//! le16 data2}` — and even their bandwidth encodings coincide (Newracom's `WIM_BW_1M..WIM_BW_8M` =
//! 0..3 equals Morse's `DOT11_RT_S1G_BW_1MHZ..8MHZ`).
//!
//! ⚠ **`FLAGS` is not decoration on this bearer.** Both drivers set
//! `IEEE80211_RADIOTAP_F_FCS` and append a 4-byte FCS to every data frame — Newracom
//! unconditionally (`rt_flags = ((ndp_ind) ? 0x00 : 0x10)` plus `skb_put(skb, 4)`), Morse from
//! `MORSE_RX_STATUS_FLAGS_FCS_INCLUDED`. Ignoring the flag delivers four bytes of FCS *inside*
//! the NDN payload on every HaLow frame; ignoring `F_BADFCS` (which Morse sets on
//! `MORSE_RX_STATUS_FLAGS_CRC_ERROR`) delivers corrupt frames as good ones. [`RadiotapInfo::flags`]
//! carries it and [`crate::frame::parse`] acts on it.

/// Present-bitmap bit indices (subset we care about).
const BIT_TSFT: u32 = 0;
const BIT_FLAGS: u32 = 1;
const BIT_RATE: u32 = 2;
const BIT_DBM_ANTSIGNAL: u32 = 5;
const BIT_TX_FLAGS: u32 = 15;
const BIT_MCS: u32 = 19;
/// Bit 28: the field area is followed by a **TLV** region (radiotap.org "TLV fields").
const BIT_TLVS: u32 = 28;
/// Bit 31 of a present word: another present word follows.
const BIT_EXT: u32 = 31;

/// Present bits we do not model *and* that would sit between the last field we do model (bit 21,
/// VHT) and the TLV region: 22..27 plus the two namespace-switch bits 29/30. If any of them is set
/// we cannot know where the TLV region begins, so we do not guess — TLV decoding is skipped and
/// only the standard fields are reported.
///
/// Bit 26 (`ZERO_LEN_PSDU`) is the live case, not a theoretical one: **both** HaLow drivers set it
/// on NDP (null-data-packet) frames, and neither sets bit 28 on those, so the gate simply keeps us
/// from mis-walking a frame shape that carries no TLVs anyway.
const UNMODELLED_BEFORE_TLVS: u32 = 0x6FC0_0000;

/// `IEEE80211_RADIOTAP_F_FCS` — the frame's trailing 4-byte FCS was *not* stripped by the driver.
pub const F_FCS: u8 = 0x10;
/// `IEEE80211_RADIOTAP_F_BADFCS` — the frame failed its FCS check; the payload is corrupt.
pub const F_BADFCS: u8 = 0x40;

/// Radiotap TLV type carrying the **S1G** (802.11ah) PHY descriptor: `{le16 known, le16 data1,
/// le16 data2}`. `DOT11_RT_TLV_S1G_TYPE` in Morse's `s1g_radiotap.h`; the literal `32` in
/// Newracom's `nrc-trx.c`.
pub const TLV_TYPE_S1G: u16 = 32;
/// `IEEE80211_RADIOTAP_VENDOR_NAMESPACE` as a TLV type.
pub const TLV_TYPE_VENDOR: u16 = 30;
/// Morse Micro's OUI (`MORSE_OUI` = `0x0CBF74`, `morse_driver/vendor.h:20`).
pub const MORSE_OUI: [u8; 3] = [0x0c, 0xbf, 0x74];
/// Morse vendor TLV sub-type carrying the exact RX frequency in kHz
/// (`MORSE_VENDOR_TLV_FREQ_KHZ_TYPE`).
const MORSE_VENDOR_TYPE_FREQ_KHZ: u16 = 35;

// S1G TLV `known` mask bits (Morse `DOT11_RT_S1G_KNOWN_*`; Newracom writes the literal 0x007F).
const S1G_KNOWN_PPDU_FMT: u16 = 0x0001;
const S1G_KNOWN_RES_IND: u16 = 0x0002;
const S1G_KNOWN_GI: u16 = 0x0004;
const S1G_KNOWN_BW: u16 = 0x0010;
const S1G_KNOWN_MCS: u16 = 0x0020;
const S1G_KNOWN_COLOR: u16 = 0x0040;
const S1G_KNOWN_UPL_IND: u16 = 0x0080;

/// `IEEE80211_RADIOTAP_F_TX_NOACK` — inject as broadcast, do not wait for an
/// ACK that will never come (there is no associated peer).
const TX_FLAG_NOACK: u16 = 0x0008;

// MCS "known" mask: which of the MCS sub-fields we are actually specifying.
const MCS_HAVE_MCS: u8 = 0x01;
const MCS_HAVE_BW: u8 = 0x02;
const MCS_HAVE_GI: u8 = 0x04;
// MCS flags: bandwidth in bits 0-1 (0 = 20 MHz), short-GI in bit 2.
const MCS_FLAG_BW_20: u8 = 0x00;
const MCS_FLAG_SGI: u8 = 0x04;

/// Total length of the TX header produced by [`build_tx_header`].
pub const TX_HEADER_LEN: usize = 13;

/// Build a radiotap **TX** header selecting an 802.11n MCS rate.
///
/// The frame the driver transmits is `build_tx_header(..) ++ <802.11 frame>`.
/// `mcs_index` is the 11n modulation-and-coding index (0–7 for a single
/// spatial stream, 20 MHz). `short_gi` requests the 400 ns guard interval.
///
/// Field layout (no padding needed — TX_FLAGS is 2-byte aligned at offset 8,
/// MCS is 1-byte aligned right after):
/// ```text
/// off 0  : version = 0
/// off 1  : pad     = 0
/// off 2  : le16 len = 13
/// off 4  : le32 present = (1<<TX_FLAGS) | (1<<MCS)
/// off 8  : le16 TX_FLAGS = NOACK
/// off 10 : u8 MCS.known = HAVE_MCS|HAVE_BW|HAVE_GI
/// off 11 : u8 MCS.flags = BW_20 | (SGI if short_gi)
/// off 12 : u8 MCS.index = mcs_index
/// ```
pub fn build_tx_header(mcs_index: u8, short_gi: bool) -> [u8; TX_HEADER_LEN] {
    let present: u32 = (1 << BIT_TX_FLAGS) | (1 << BIT_MCS);
    let mut h = [0u8; TX_HEADER_LEN];
    // fixed header
    h[0] = 0; // version
    h[1] = 0; // pad
    h[2..4].copy_from_slice(&(TX_HEADER_LEN as u16).to_le_bytes());
    h[4..8].copy_from_slice(&present.to_le_bytes());
    // TX_FLAGS (bit 15)
    h[8..10].copy_from_slice(&TX_FLAG_NOACK.to_le_bytes());
    // MCS (bit 19)
    h[10] = MCS_HAVE_MCS | MCS_HAVE_BW | MCS_HAVE_GI;
    h[11] = MCS_FLAG_BW_20 | if short_gi { MCS_FLAG_SGI } else { 0 };
    h[12] = mcs_index;
    h
}

/// Total length of the legacy-rate TX header produced by [`build_tx_legacy`].
pub const TX_LEGACY_HEADER_LEN: usize = 12;

/// Build a radiotap **TX** header selecting a **legacy (non-HT) rate** instead
/// of an MCS. `rate_500kbps` is in 500 kbps units (`2` = 1 Mbps DSSS). This is
/// the robust, ESP-NOW-native path: 1 Mbps has a far better link budget than
/// the lowest MCS (≈9 dB), and it is the rate an ESP32 expects for ESP-NOW.
///
/// Layout: RATE (bit 2, 1 byte) then TX_FLAGS (bit 15, 2-byte aligned → one
/// pad byte at offset 9).
pub fn build_tx_legacy(rate_500kbps: u8) -> [u8; TX_LEGACY_HEADER_LEN] {
    const BIT_RATE: u32 = 2;
    let present: u32 = (1 << BIT_RATE) | (1 << BIT_TX_FLAGS);
    let mut h = [0u8; TX_LEGACY_HEADER_LEN];
    h[2..4].copy_from_slice(&(TX_LEGACY_HEADER_LEN as u16).to_le_bytes());
    h[4..8].copy_from_slice(&present.to_le_bytes());
    h[8] = rate_500kbps; // RATE @ offset 8
    // offset 9 is a pad byte for the u16 TX_FLAGS alignment
    h[10..12].copy_from_slice(&TX_FLAG_NOACK.to_le_bytes());
    h
}

/// Total length of the VHT TX header produced by [`build_tx_vht`].
pub const TX_VHT_HEADER_LEN: usize = 22;

/// Build a radiotap **TX** header selecting an 802.11**ac** (VHT) rate.
///
/// The sibling of [`build_tx_header`] for VHT: `mcs` is the VHT modulation-and-coding index
/// (0–8 at 20 MHz, 0–9 for wider), `nss` the number of spatial streams, `short_gi` the 400 ns
/// guard interval. Bandwidth is fixed at 20 MHz, which is what a 1×1 20 MHz part can receive.
///
/// Exists because the HT builder cannot express VHT at all — `McsDescriptor::vht` was carried
/// through the API but silently dropped at the radiotap boundary, so every "VHT" injection went
/// out as HT. That made VHT RX untestable rather than failing (2026-08-24).
///
/// Field layout (VHT is 2-byte aligned; it starts at offset 10, which is already even):
/// ```text
/// off 0  : version = 0, pad = 0
/// off 2  : le16 len = 22
/// off 4  : le32 present = (1<<TX_FLAGS) | (1<<VHT)
/// off 8  : le16 TX_FLAGS = NOACK
/// off 10 : le16 VHT.known = BANDWIDTH | GI
/// off 12 : u8  VHT.flags = SGI?
/// off 13 : u8  VHT.bandwidth = 0 (20 MHz)
/// off 14 : u8[4] VHT.mcs_nss — user 0 = (mcs << 4) | nss
/// off 18 : u8  VHT.coding = 0 (BCC)
/// off 19 : u8  VHT.group_id = 0
/// off 20 : le16 VHT.partial_aid = 0
/// ```
pub fn build_tx_vht(mcs: u8, nss: u8, short_gi: bool) -> [u8; TX_VHT_HEADER_LEN] {
    const BIT_VHT: u32 = 21;
    const VHT_KNOWN_GI: u16 = 0x0004;
    const VHT_KNOWN_BANDWIDTH: u16 = 0x0040;
    const VHT_FLAG_SGI: u8 = 0x04;

    let present: u32 = (1 << BIT_TX_FLAGS) | (1 << BIT_VHT);
    let mut h = [0u8; TX_VHT_HEADER_LEN];
    h[2..4].copy_from_slice(&(TX_VHT_HEADER_LEN as u16).to_le_bytes());
    h[4..8].copy_from_slice(&present.to_le_bytes());
    h[8..10].copy_from_slice(&TX_FLAG_NOACK.to_le_bytes());
    h[10..12].copy_from_slice(&(VHT_KNOWN_BANDWIDTH | VHT_KNOWN_GI).to_le_bytes());
    h[12] = if short_gi { VHT_FLAG_SGI } else { 0 };
    h[13] = 0; // 20 MHz
    h[14] = (mcs << 4) | (nss & 0x0f); // user 0
    h
}

/// Total length of the S1G TX header produced by [`build_tx_s1g`].
pub const TX_S1G_HEADER_LEN: usize = 10;

/// Build a radiotap **TX** header for **802.11ah (S1G / Wi-Fi HaLow)** injection.
///
/// S1G is a different PHY: its rate is not an 11n/ac MCS index and the on-chip
/// MAC (e.g. the Newracom NRC7292) sets the sub-GHz rate itself. So this header
/// names **no** rate — it carries only `TX_FLAGS = NOACK` (broadcast injection,
/// no ACK to wait for). Naming an HT MCS here would be semantically wrong for an
/// S1G frame; leaving rate unset lets the firmware transmit at its configured
/// S1G rate. Verified on-air: an NRC7292 in monitor mode injects a frame carried
/// by this header and a second NRC7292 receives it.
///
/// Layout: `TX_FLAGS` (bit 15, 2-byte aligned at offset 8), nothing else.
pub fn build_tx_s1g() -> [u8; TX_S1G_HEADER_LEN] {
    let present: u32 = 1 << BIT_TX_FLAGS;
    let mut h = [0u8; TX_S1G_HEADER_LEN];
    h[2..4].copy_from_slice(&(TX_S1G_HEADER_LEN as u16).to_le_bytes());
    h[4..8].copy_from_slice(&present.to_le_bytes());
    h[8..10].copy_from_slice(&TX_FLAG_NOACK.to_le_bytes()); // TX_FLAGS @ offset 8
    h
}

/// The S1G PPDU format the preamble announced (`data1` bits 0–1).
///
/// Not cosmetic: `S1g1M` is the 1 MHz preamble, which is the only one that can carry **MCS10**
/// (the repetition-coded BPSK reach rate), and it is a different symbol structure from the
/// 2/4/8/16 MHz "short" and "long" preambles. Together with [`S1gInfo::bandwidth_mhz`] it is what
/// makes an S1G MCS index mean a data rate at all.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum S1gPpduFormat {
    /// `DOT11_RT_S1G_PPDU_S1G_1M` — the 1 MHz preamble.
    S1g1M,
    /// `DOT11_RT_S1G_PPDU_S1G_SHORT` — the ≥2 MHz short preamble.
    S1gShort,
    /// `DOT11_RT_S1G_PPDU_S1G_LONG` — the ≥2 MHz long preamble.
    S1gLong,
}

/// The 802.11ah **S1G radiotap TLV** (type 32, length 6), decoded.
///
/// Every field is `Option` and is populated only when the driver set the matching `known` bit —
/// "the driver did not say" and "the driver said zero" are different answers and this crate does
/// not conflate them.
///
/// ★ **There is deliberately no `nss` field, and that is the honest reading of both drivers.**
/// The wire format has one (`data1` bits 6–7, gated by `DOT11_RT_S1G_KNOWN_NSS`), but Morse never
/// sets the known bit (`monitor.c` omits it from the `known` mask it builds) and Newracom sets it
/// — `rt_s1g_known = 0x007F` includes 0x0008 — while writing the field as the literal `0x0 << 6`
/// (`nrc-trx.c`). Decoding that would launder a driver-side constant into a measurement. Both
/// parts are single-stream anyway (`RadioCapability::rate.max_nss = 1`), so nothing is lost.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct S1gInfo {
    /// Preamble/PPDU format (`data1` bits 0–1). `None` when the encoded value is the reserved 3.
    pub ppdu_format: Option<S1gPpduFormat>,
    /// Response indication (`data1` bits 2–3): 0 = none, 1 = NDP, 2 = normal, 3 = long.
    pub response_indication: Option<u8>,
    /// Short guard interval (`data1` bit 5).
    pub short_gi: Option<bool>,
    /// Received channel width in MHz (`data1` bits 8–11), one of 1/2/4/8/16. `None` for the
    /// encoded `DOT11_RT_S1G_BW_INVALID` (5) and anything above it.
    ///
    /// ★ **This is the established on-air oracle for this bearer** — reading it back is how the
    /// Morse `inject_bw` sweep was proven to actually change the transmitted width (0..3 read back
    /// as 1/2/4/8 MHz), after an SDR had produced byte-identical numbers for four different
    /// configurations. It is also the missing half of the rate: an S1G MCS without a width is not
    /// a bit rate.
    pub bandwidth_mhz: Option<u8>,
    /// S1G MCS index (`data1` bits 12–15), 0–10.
    ///
    /// ⚠ **This is NOT an 802.11n MCS.** The S1G table is a different one, its rate depends on
    /// [`bandwidth_mhz`](Self::bandwidth_mhz), and MCS10 is *below* MCS0 in rate (1 MHz-only,
    /// repetition-coded BPSK — the most robust mode, not the fastest). Feeding it to an 11n rate
    /// table overstates the rate several-fold. See [`crate::frame::parse`], which surfaces it on
    /// `CapturedFrame.mcs_index` for want of anywhere else to put it, and says so there too.
    pub mcs: Option<u8>,
    /// BSS colour (`data2` bits 0–2) — the 3-bit S1G spatial-reuse identifier.
    pub bss_color: Option<u8>,
    /// Uplink indication (`data2` bit 3).
    pub uplink: Option<bool>,
    /// Per-frame signal level (`data2` bits 8–15, read as `i8`).
    ///
    /// ⚠ **Units are firm on Morse and UNVERIFIED on Newracom.** Morse writes
    /// `(s8)hdr_rx_status->rssi`, the same value it puts in `DBM_ANTSIGNAL`, and that path has
    /// been read on air as real dBm (−21/−32/−46, with ≈−109 for a wrong-channel receiver).
    /// Newracom writes `rxi->rcpi << 8` — a field still *named* `rcpi`, next to a driver comment
    /// stating the reference was changed from RCPI (`RSSI = RCPI/2 − 122`) to the Rx-vector RSSI.
    /// If that comment is stale the values are ~2× and offset. Cross-check against
    /// `cli_app show signal` before treating an NRC7292 number as calibrated dBm.
    pub rssi_dbm: Option<i8>,
}

impl S1gInfo {
    /// Decode the 6-byte S1G TLV body `{le16 known, le16 data1, le16 data2}`.
    fn decode(data: &[u8]) -> Option<Self> {
        if data.len() < 6 {
            return None;
        }
        let known = u16::from_le_bytes([data[0], data[1]]);
        let d1 = u16::from_le_bytes([data[2], data[3]]);
        let d2 = u16::from_le_bytes([data[4], data[5]]);
        let has = |m: u16| known & m != 0;
        Some(Self {
            ppdu_format: has(S1G_KNOWN_PPDU_FMT)
                .then(|| match d1 & 0x0003 {
                    0 => Some(S1gPpduFormat::S1g1M),
                    1 => Some(S1gPpduFormat::S1gShort),
                    2 => Some(S1gPpduFormat::S1gLong),
                    _ => None,
                })
                .flatten(),
            response_indication: has(S1G_KNOWN_RES_IND).then(|| ((d1 >> 2) & 0x0003) as u8),
            short_gi: has(S1G_KNOWN_GI).then(|| d1 & 0x0020 != 0),
            // 0..4 => 1/2/4/8/16 MHz; 5 is DOT11_RT_S1G_BW_INVALID.
            bandwidth_mhz: has(S1G_KNOWN_BW)
                .then(|| match (d1 >> 8) & 0x000f {
                    0 => Some(1u8),
                    1 => Some(2),
                    2 => Some(4),
                    3 => Some(8),
                    4 => Some(16),
                    _ => None,
                })
                .flatten(),
            mcs: has(S1G_KNOWN_MCS).then(|| ((d1 >> 12) & 0x000f) as u8),
            bss_color: has(S1G_KNOWN_COLOR).then(|| (d2 & 0x0007) as u8),
            uplink: has(S1G_KNOWN_UPL_IND).then(|| d2 & 0x0008 != 0),
            // No `known` bit exists for the signal field; both drivers always write it.
            rssi_dbm: Some((d2 >> 8) as u8 as i8),
        })
    }
}

/// What we extract from a captured frame's radiotap header.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RadiotapInfo {
    /// Received signal strength in dBm (`IEEE80211_RADIOTAP_DBM_ANTSIGNAL`).
    pub rssi_dbm: Option<i8>,
    /// 11n MCS index the frame was received at, if the `MCS` field was present.
    pub mcs_index: Option<u8>,
    /// Legacy rate in 500 kbps units, if the `RATE` field was present.
    pub rate_500kbps: Option<u8>,
    /// The 64-bit TSFT (MAC Timing Synchronization Function) counter latched by
    /// the NIC at reception, if the header carried it. This is the hardware
    /// receive timestamp named-time builds a [`LinkStamp`](ndn_radio_hal::LinkStamp)
    /// from — ~1 µs resolution, latched before host software touches the frame.
    pub tsft: Option<u64>,
    /// The `FLAGS` field (bit 1) verbatim, when present. Read it with
    /// [`fcs_included`](Self::fcs_included) / [`bad_fcs`](Self::bad_fcs) rather than by hand.
    pub flags: Option<u8>,
    /// The decoded 802.11ah S1G TLV, when the header carried one. See [`S1gInfo`].
    pub s1g: Option<S1gInfo>,
    /// Exact RX frequency in **kHz**, from Morse Micro's vendor TLV (`MORSE_VENDOR_TLV_FREQ_KHZ`).
    ///
    /// Strictly better than the standard `CHANNEL` field, which the same driver truncates to whole
    /// MHz (`KHZ100_TO_MHZ`) — and at S1G, where channels are 1 MHz apart and a 902.5 MHz raster is
    /// in play, whole MHz is not enough to say which channel a frame arrived on. Newracom emits no
    /// vendor TLV, so this is `None` on an NRC7292.
    pub freq_khz: Option<u32>,
    /// Offset where the 802.11 frame begins (== `it_len`).
    pub header_len: usize,
}

impl RadiotapInfo {
    /// The frame's trailing 4 bytes are its FCS and must be removed before the payload is used
    /// (`IEEE80211_RADIOTAP_F_FCS`). True on essentially every HaLow data frame.
    pub fn fcs_included(&self) -> bool {
        self.flags.is_some_and(|f| f & F_FCS != 0)
    }

    /// The frame failed its FCS check (`IEEE80211_RADIOTAP_F_BADFCS`) — the bytes are corrupt and
    /// must not be delivered. Set by the Morse driver from `MORSE_RX_STATUS_FLAGS_CRC_ERROR`;
    /// Newracom never sets it.
    pub fn bad_fcs(&self) -> bool {
        self.flags.is_some_and(|f| f & F_BADFCS != 0)
    }
}

/// `(alignment, size)` for each radiotap field we know how to skip, indexed by
/// present-bit. `None` = a field whose size we don't model; hitting a set bit
/// with `None` stops the walk (we return what was gathered, payload offset is
/// still known from `it_len`).
const FIELD_DEFS: [Option<(usize, usize)>; 22] = [
    Some((8, 8)),  // 0  TSFT
    Some((1, 1)),  // 1  FLAGS
    Some((1, 1)),  // 2  RATE
    Some((2, 4)),  // 3  CHANNEL
    Some((2, 2)),  // 4  FHSS
    Some((1, 1)),  // 5  DBM_ANTSIGNAL
    Some((1, 1)),  // 6  DBM_ANTNOISE
    Some((2, 2)),  // 7  LOCK_QUALITY
    Some((2, 2)),  // 8  TX_ATTENUATION
    Some((2, 2)),  // 9  DB_TX_ATTENUATION
    Some((1, 1)),  // 10 DBM_TX_POWER
    Some((1, 1)),  // 11 ANTENNA
    Some((1, 1)),  // 12 DB_ANTSIGNAL
    Some((1, 1)),  // 13 DB_ANTNOISE
    Some((2, 2)),  // 14 RX_FLAGS
    Some((2, 2)),  // 15 TX_FLAGS
    Some((1, 1)),  // 16 RTS_RETRIES
    Some((1, 1)),  // 17 DATA_RETRIES
    None,          // 18 (XChannel / reserved — ambiguous, stop here)
    Some((1, 3)),  // 19 MCS
    Some((4, 8)),  // 20 AMPDU_STATUS
    Some((2, 12)), // 21 VHT
];

fn align_up(off: usize, align: usize) -> usize {
    (off + align - 1) & !(align - 1)
}

/// Parse a captured frame's radiotap header. Returns `None` only when the
/// buffer is too short or the version/length are malformed; a header that
/// simply lacks RSSI yields `RadiotapInfo { rssi_dbm: None, .. }` with a valid
/// `header_len`.
pub fn parse(buf: &[u8]) -> Option<RadiotapInfo> {
    if buf.len() < 8 || buf[0] != 0 {
        return None;
    }
    let it_len = u16::from_le_bytes([buf[2], buf[3]]) as usize;
    if it_len < 8 || it_len > buf.len() {
        return None;
    }

    // Read the first present word; skip any extension words (bit 31 set).
    let present0 = u32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]);
    let mut off = 8;
    let mut word = present0;
    while word & (1 << BIT_EXT) != 0 {
        if off + 4 > it_len {
            return None;
        }
        word = u32::from_le_bytes([buf[off], buf[off + 1], buf[off + 2], buf[off + 3]]);
        off += 4;
    }

    let mut info = RadiotapInfo {
        header_len: it_len,
        ..RadiotapInfo::default()
    };

    // Walk only the standard fields advertised by the first present word. `complete` records
    // whether we got through them all — a break leaves `off` meaningless, so the TLV region
    // (whose start is `off` rounded up) can only be located when it is still true.
    let mut complete = true;
    for (bit, def) in FIELD_DEFS.iter().enumerate() {
        if present0 & (1 << bit) == 0 {
            continue;
        }
        let (align, size) = match def {
            Some(d) => *d,
            None => {
                complete = false;
                break; // unknown field; payload offset already known
            }
        };
        off = align_up(off, align);
        if off + size > it_len {
            complete = false;
            break;
        }
        match bit as u32 {
            BIT_TSFT => {
                info.tsft = Some(u64::from_le_bytes([
                    buf[off],
                    buf[off + 1],
                    buf[off + 2],
                    buf[off + 3],
                    buf[off + 4],
                    buf[off + 5],
                    buf[off + 6],
                    buf[off + 7],
                ]));
            }
            BIT_FLAGS => info.flags = Some(buf[off]),
            BIT_RATE => info.rate_500kbps = Some(buf[off]),
            BIT_DBM_ANTSIGNAL => info.rssi_dbm = Some(buf[off] as i8),
            BIT_MCS => info.mcs_index = Some(buf[off + 2]), // known, flags, index
            _ => {}
        }
        off += size;
    }

    // ── The TLV region (present bit 28) ─────────────────────────────────────────────────────
    // Everything the two HaLow drivers know about a frame lives here. Only walk it when the
    // standard-field walk actually reached the end (so `off` is real) and no unmodelled field
    // could sit between there and the TLVs.
    if complete && present0 & (1 << BIT_TLVS) != 0 && present0 & UNMODELLED_BEFORE_TLVS == 0 {
        parse_tlvs(buf, align_up(off, 4), it_len, &mut info);
    }

    Some(info)
}

/// Walk the radiotap TLV region `[start, it_len)`, filling the TLV-borne fields of `info`.
///
/// Each TLV is `{le16 type, le16 length, u8 data[length]}` with the *next* TLV starting at the
/// following 4-byte boundary. Verified arithmetically against both drivers' structs:
///
/// ```text
/// NRC7292 (nrc_radiotap_hdr, it_len 34): TSFT 8..16, FLAGS 16, pad 17, CHANNEL 18..22,
///     rt_pad2 22..24  =>  TLV at 24: type 32, len 6, body 28..34.
/// NRC7292 A-MPDU (nrc_radiotap_hdr_agg, it_len 42): + AMPDU_STATUS 24..32 => TLV at 32.
/// MM6108  (morse_radiotap_hdr, it_len 52): TSFT 8..16, FLAGS 16, RATE 17, CHANNEL 18..22,
///     DBM_ANTSIGNAL 22, align_padding 23  =>  TLV at 24: vendor 24..40, then S1G 40..52.
/// MM6108 A-MPDU (it_len 60): + AMPDU_STATUS 24..32 => vendor 32..48, S1G 48..60.
/// ```
///
/// ⚠ **Do not assume the S1G TLV comes first.** Morse `skb_push`es the S1G TLV *before* the vendor
/// TLV, which puts the vendor TLV first on the wire; Newracom emits only the S1G one. The loop is
/// order-independent for exactly this reason.
fn parse_tlvs(buf: &[u8], start: usize, it_len: usize, info: &mut RadiotapInfo) {
    let mut off = start;
    while off + 4 <= it_len {
        let ttype = u16::from_le_bytes([buf[off], buf[off + 1]]);
        let tlen = u16::from_le_bytes([buf[off + 2], buf[off + 3]]) as usize;
        let data_start = off + 4;
        let Some(data_end) = data_start.checked_add(tlen) else {
            return;
        };
        if data_end > it_len {
            return; // truncated TLV — stop rather than read past the header
        }
        let data = &buf[data_start..data_end];
        match ttype {
            TLV_TYPE_S1G => info.s1g = S1gInfo::decode(data),
            TLV_TYPE_VENDOR => parse_vendor_tlv(data, info),
            _ => {}
        }
        // Next TLV begins at the next 4-byte boundary after this one's data.
        let Some(next) = data_start.checked_add(align_up(tlen, 4)) else {
            return;
        };
        if next <= off {
            return; // zero-length, non-advancing TLV — never loop forever on a malformed header
        }
        off = next;
    }
}

/// Decode a vendor-namespace TLV body (`struct radiotap_vendor_tlv_hdr` + vendor data):
/// `oui[3] ‖ subtype ‖ le16 vendor_type ‖ le16 reserved ‖ data…`.
///
/// The only one we understand is Morse Micro's frequency-in-kHz TLV. An unrecognised vendor is
/// skipped in silence — that is what a namespace is for.
fn parse_vendor_tlv(data: &[u8], info: &mut RadiotapInfo) {
    if data.len() < 8 {
        return;
    }
    let oui = [data[0], data[1], data[2]];
    let vendor_type = u16::from_le_bytes([data[4], data[5]]);
    if oui == MORSE_OUI && vendor_type == MORSE_VENDOR_TYPE_FREQ_KHZ && data.len() >= 12 {
        info.freq_khz = Some(u32::from_le_bytes([data[8], data[9], data[10], data[11]]));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tx_header_carries_mcs_and_noack() {
        let h = build_tx_header(3, true);
        assert_eq!(h.len(), TX_HEADER_LEN);
        assert_eq!(h[0], 0, "version");
        assert_eq!(u16::from_le_bytes([h[2], h[3]]) as usize, TX_HEADER_LEN);
        let present = u32::from_le_bytes([h[4], h[5], h[6], h[7]]);
        assert_eq!(present, (1 << BIT_TX_FLAGS) | (1 << BIT_MCS));
        assert_eq!(u16::from_le_bytes([h[8], h[9]]), TX_FLAG_NOACK);
        assert_eq!(h[10], MCS_HAVE_MCS | MCS_HAVE_BW | MCS_HAVE_GI);
        assert_eq!(h[11] & MCS_FLAG_SGI, MCS_FLAG_SGI, "short GI requested");
        assert_eq!(h[12], 3, "mcs index");
    }

    /// A TX header is itself valid radiotap, so parsing it back must recover the
    /// MCS index and find the payload at `it_len`.
    #[test]
    fn tx_header_round_trips_through_parse() {
        let h = build_tx_header(5, false);
        let info = parse(&h).expect("tx header is valid radiotap");
        assert_eq!(info.mcs_index, Some(5));
        assert_eq!(info.header_len, TX_HEADER_LEN);
        assert_eq!(info.rssi_dbm, None, "TX header carries no RSSI");
    }

    /// Synthesise a realistic capture header (FLAGS + RATE + CHANNEL +
    /// DBM_ANTSIGNAL + ANTENNA, the ath9k-ish set) and confirm RSSI extraction
    /// with correct field alignment.
    #[test]
    fn parse_extracts_rssi_from_capture_header() {
        let present: u32 = (1 << 1) | (1 << 2) | (1 << 3) | (1 << 5) | (1 << 11);
        let mut h = vec![0u8, 0]; // version, pad
        let mut body = Vec::new();
        // bit 1 FLAGS (1,1) @ off 8
        body.push(0x10);
        // bit 2 RATE (1,1) @ off 9
        body.push(0x02); // 1 Mbps in 500kbps units
        // bit 3 CHANNEL (2,4): off 10 -> align 2 -> 10, write 4 bytes
        body.extend_from_slice(&2412u16.to_le_bytes());
        body.extend_from_slice(&0x00a0u16.to_le_bytes());
        // bit 5 DBM_ANTSIGNAL (1,1) @ off 14
        body.push((-67i8) as u8);
        // bit 11 ANTENNA (1,1) @ off 15
        body.push(0);
        let it_len = (8 + body.len()) as u16;
        h.extend_from_slice(&it_len.to_le_bytes());
        h.extend_from_slice(&present.to_le_bytes());
        h.extend_from_slice(&body);

        let info = parse(&h).expect("valid header");
        assert_eq!(info.rssi_dbm, Some(-67));
        assert_eq!(info.rate_500kbps, Some(0x02));
        assert_eq!(info.header_len, it_len as usize);
    }

    #[test]
    fn parse_extracts_tsft() {
        // A header carrying only TSFT (bit 0), an 8-byte counter aligned to 8.
        let present: u32 = 1 << 0;
        let tsft: u64 = 0x0123_4567_89ab_cdef;
        let mut h = vec![0u8, 0]; // version, pad
        let body = tsft.to_le_bytes(); // TSFT @ off 8 (already aligned)
        let it_len = (8 + body.len()) as u16;
        h.extend_from_slice(&it_len.to_le_bytes());
        h.extend_from_slice(&present.to_le_bytes());
        h.extend_from_slice(&body);

        let info = parse(&h).expect("valid header");
        assert_eq!(info.tsft, Some(tsft));
        // A header without TSFT leaves it None.
        let mut h2 = vec![0u8, 0];
        let present2: u32 = 1 << 5; // DBM_ANTSIGNAL only
        let it_len2 = 8u16 + 1;
        h2.extend_from_slice(&it_len2.to_le_bytes());
        h2.extend_from_slice(&present2.to_le_bytes());
        h2.push((-50i8) as u8);
        assert_eq!(parse(&h2).unwrap().tsft, None);
    }

    // ── 802.11ah / S1G: the TLV region both HaLow drivers hide their metadata in ──────────
    //
    // The byte sequences below are assembled field-by-field from the drivers' own structs
    // (`nrc7292_sw_pkg` `nrc.h:455` / `nrc-trx.c:1282`, `morse_driver/s1g_radiotap.h` +
    // `monitor.c:morse_mon_rx`), so each test is a statement about what those drivers actually
    // put on the wire, not about what our own encoder happens to produce.

    /// Assemble the S1G TLV (type 32, len 6) as both vendors emit it: `{le16 type, le16 length,
    /// le16 known, le16 data1, le16 data2}`.
    ///
    /// ⚠ The two vendors differ in exactly one byte-level respect, and it is worth encoding here:
    /// Morse's `struct radiotap_s1g_tlv` carries an explicit `__padding[2]` to the next 4-byte
    /// boundary (it has to — a vendor TLV may follow), while Newracom's `nrc_radiotap_hdr` simply
    /// **ends** at `rt_s1g_data2`, giving `it_len = 34` with an unpadded final TLV. A walker that
    /// required the padding would reject every NRC7292 frame.
    fn s1g_tlv(known: u16, data1: u16, data2: u16, padded: bool) -> Vec<u8> {
        let mut v = Vec::new();
        v.extend_from_slice(&TLV_TYPE_S1G.to_le_bytes());
        v.extend_from_slice(&6u16.to_le_bytes());
        v.extend_from_slice(&known.to_le_bytes());
        v.extend_from_slice(&data1.to_le_bytes());
        v.extend_from_slice(&data2.to_le_bytes());
        if padded {
            v.extend_from_slice(&[0, 0]); // struct radiotap_s1g_tlv::__padding
        }
        v
    }

    /// Morse's frequency-in-kHz vendor TLV (`struct radiotap_morse_freq_khz`, 16 B total).
    fn morse_freq_tlv(freq_khz: u32) -> Vec<u8> {
        let mut v = Vec::new();
        v.extend_from_slice(&TLV_TYPE_VENDOR.to_le_bytes());
        v.extend_from_slice(&12u16.to_le_bytes()); // MORSE_VENDOR_TLV_FREQ_KHZ_SIZE
        v.extend_from_slice(&MORSE_OUI);
        v.push(0); // MORSE_VENDOR_TLV_SUBNS_0
        v.extend_from_slice(&MORSE_VENDOR_TYPE_FREQ_KHZ.to_le_bytes());
        v.extend_from_slice(&0u16.to_le_bytes()); // reserved
        v.extend_from_slice(&freq_khz.to_le_bytes());
        v
    }

    fn radiotap(present: u32, body: &[u8]) -> Vec<u8> {
        let mut h = vec![0u8, 0];
        h.extend_from_slice(&((8 + body.len()) as u16).to_le_bytes());
        h.extend_from_slice(&present.to_le_bytes());
        h.extend_from_slice(body);
        h
    }

    /// `struct nrc_radiotap_hdr` — the NRC7292's non-aggregated monitor header, 34 bytes.
    /// Present = TSFT | FLAGS | CHANNEL | TLVS; `rt_pad2` is what aligns the TLV to 24.
    fn nrc_header(tsft: u64, data1: u16, data2: u16) -> Vec<u8> {
        let mut b = Vec::new();
        b.extend_from_slice(&tsft.to_le_bytes()); //  8..16 TSFT
        b.push(F_FCS); //                            16    FLAGS (rt_flags = 0x10, always)
        b.push(0); //                                17    rt_pad
        b.extend_from_slice(&925u16.to_le_bytes()); // 18..20 CHANNEL freq (MHz)
        b.extend_from_slice(&0x0140u16.to_le_bytes()); // 20..22 CHANNEL flags
        b.extend_from_slice(&[0, 0]); //             22..24 rt_pad2
        b.extend_from_slice(&s1g_tlv(0x007f, data1, data2, false)); // 24..34
        radiotap((1 << 0) | (1 << 1) | (1 << 3) | (1 << 28), &b)
    }

    /// `struct morse_radiotap_hdr` + TLVs — the MM6108's monitor header, 52 bytes.
    /// Present = TSFT | FLAGS | RATE | CHANNEL | DBM_ANTSIGNAL | TLVS.
    fn morse_header(
        tsft: u64,
        rssi: i8,
        mcs: u8,
        freq_khz: u32,
        data1: u16,
        data2: u16,
    ) -> Vec<u8> {
        let mut b = Vec::new();
        b.extend_from_slice(&tsft.to_le_bytes()); //   8..16 TSFT
        b.push(F_FCS); //                              16    rt_flags
        b.push(0x80 | mcs); //                         17    rt_rate_or_zl_psdu = BIT(7) | mcs
        b.extend_from_slice(&918u16.to_le_bytes()); // 18..20 CHANNEL freq, truncated to MHz
        b.extend_from_slice(&0x0004u16.to_le_bytes()); // 20..22 IEEE80211_CHAN_900MHZ
        b.push(rssi as u8); //                         22    DBM_ANTSIGNAL
        b.push(0); //                                  23    align_padding
        b.extend_from_slice(&morse_freq_tlv(freq_khz)); // 24..40 (pushed second ⇒ first on wire)
        b.extend_from_slice(&s1g_tlv(0x00f7, data1, data2, true)); // 40..52
        radiotap(
            (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 5) | (1 << 28),
            &b,
        )
    }

    /// The NRC7292 case. Before the TLV walk existed this radio reported `rssi_dbm: None` on every
    /// single frame — it emits no `DBM_ANTSIGNAL` at all — and `mcs_index: None`.
    #[test]
    fn nrc7292_header_yields_rssi_mcs_and_width() {
        // data1 as nrc-trx.c builds it: ppdu SHORT(1), response_ind 2, short-GI, NSS literal 0,
        // bandwidth WIM_BW_2M(1), MCS 7. data2: colour 0, uplink 0, signal −46.
        let data1 = 1 | (2 << 2) | (1 << 5) | (1 << 8) | (7 << 12);
        let data2 = ((-46i8) as u8 as u16) << 8;
        let h = nrc_header(0x0000_0001_2345_6789, data1, data2);
        assert_eq!(h.len(), 34, "struct nrc_radiotap_hdr is 34 bytes");

        let info = parse(&h).expect("valid radiotap");
        assert_eq!(info.header_len, 34);
        assert_eq!(info.tsft, Some(0x0000_0001_2345_6789));
        assert!(info.fcs_included(), "rt_flags = 0x10 on every data frame");
        assert!(!info.bad_fcs(), "the NRC driver never sets F_BADFCS");
        assert_eq!(info.freq_khz, None, "Newracom emits no vendor TLV");

        let s1g = info.s1g.expect("the S1G TLV is the whole point");
        assert_eq!(s1g.mcs, Some(7));
        assert_eq!(s1g.bandwidth_mhz, Some(2));
        assert_eq!(s1g.ppdu_format, Some(S1gPpduFormat::S1gShort));
        assert_eq!(s1g.short_gi, Some(true));
        assert_eq!(s1g.response_indication, Some(2));
        assert_eq!(s1g.rssi_dbm, Some(-46));
        assert_eq!(s1g.bss_color, Some(0));
        assert_eq!(
            s1g.uplink, None,
            "known = 0x007F excludes UPL_IND, so we must not claim it"
        );
    }

    /// `struct nrc_radiotap_hdr_agg`: an 8-byte `AMPDU_STATUS` field slides the TLV region from
    /// offset 24 to 32. The walker must find it by arithmetic, not by a constant.
    #[test]
    fn nrc7292_ampdu_header_shifts_the_tlv_region() {
        let mut b = Vec::new();
        b.extend_from_slice(&7u64.to_le_bytes()); //     8..16 TSFT
        b.push(F_FCS); //                                16
        b.push(0); //                                    17 rt_pad
        b.extend_from_slice(&925u16.to_le_bytes()); //   18..20
        b.extend_from_slice(&0x0140u16.to_le_bytes()); //20..22
        b.extend_from_slice(&[0, 0]); //                 22..24 rt_pad2
        b.extend_from_slice(&1u32.to_le_bytes()); //     24..28 rt_ampdu_ref
        b.extend_from_slice(&2u16.to_le_bytes()); //     28..30 rt_ampdu_flags
        b.push(3); //                                    30    rt_ampdu_crc
        b.push(0); //                                    31    rt_ampdu_reserved
        b.extend_from_slice(&s1g_tlv(0x007f, 4 << 12, 0, false)); // 32..42
        let h = radiotap((1 << 0) | (1 << 1) | (1 << 3) | (1 << 20) | (1 << 28), &b);
        assert_eq!(h.len(), 42, "struct nrc_radiotap_hdr_agg is 42 bytes");
        assert_eq!(parse(&h).unwrap().s1g.unwrap().mcs, Some(4));
    }

    /// The MM6108 case, including the two facts that make it different from the NRC7292: it emits
    /// a *vendor* TLV before the S1G one, and that vendor TLV carries the exact frequency in kHz
    /// where the standard CHANNEL field has been truncated to whole MHz.
    #[test]
    fn morse_header_yields_exact_frequency_and_s1g_fields() {
        // data1: ppdu SHORT(1), long GI, bandwidth 8 MHz(3), MCS 5.
        let data1 = 1 | (3 << 8) | (5 << 12);
        // data2: colour 3, uplink 1, signal −32.
        let data2 = ((-32i8) as u8 as u16) << 8 | 0x0008 | 3;
        let h = morse_header(0xdead_beef, -32, 5, 918_500, data1, data2);
        assert_eq!(h.len(), 52, "morse_radiotap_hdr + vendor TLV + S1G TLV");

        let info = parse(&h).expect("valid radiotap");
        assert_eq!(info.tsft, Some(0xdead_beef));
        assert_eq!(info.rssi_dbm, Some(-32), "Morse also fills DBM_ANTSIGNAL");
        assert!(info.fcs_included());
        assert_eq!(
            info.freq_khz,
            Some(918_500),
            "the vendor TLV is the only place the sub-MHz part of the frequency survives"
        );
        assert_eq!(
            info.rate_500kbps,
            Some(0x85),
            "★ the RATE field is BIT(7)|mcs on this driver — NOT 500 kbps units; \
             reading it as a legacy rate yields ≥64 Mbit/s of nonsense"
        );

        let s1g = info.s1g.expect("S1G TLV present");
        assert_eq!(s1g.mcs, Some(5), "the honest MCS source on this radio");
        assert_eq!(s1g.bandwidth_mhz, Some(8));
        assert_eq!(s1g.short_gi, Some(false));
        assert_eq!(s1g.bss_color, Some(3));
        assert_eq!(s1g.uplink, Some(true), "Morse DOES set KNOWN_UPL_IND");
        assert_eq!(s1g.rssi_dbm, Some(-32), "same value as DBM_ANTSIGNAL");
    }

    /// Order-independence, stated as a test because the two vendors disagree: Morse pushes the S1G
    /// TLV first (so it lands *last* on the wire) and Newracom emits only the S1G one. A walker
    /// that assumed "S1G is the first TLV" would read Morse's vendor header as an S1G descriptor.
    #[test]
    fn tlv_walk_is_order_independent() {
        let mut b = Vec::new();
        b.extend_from_slice(&0u64.to_le_bytes());
        b.push(0);
        b.push(0);
        b.extend_from_slice(&900u16.to_le_bytes());
        b.extend_from_slice(&0u16.to_le_bytes());
        b.extend_from_slice(&[0, 0]); // pad to 24
        // S1G first, vendor second — the mirror image of what Morse emits.
        b.extend_from_slice(&s1g_tlv(0x00f7, 2 | (2 << 8) | (9 << 12), 0, true));
        b.extend_from_slice(&morse_freq_tlv(904_500));
        let h = radiotap((1 << 0) | (1 << 1) | (1 << 3) | (1 << 28), &b);
        let info = parse(&h).unwrap();
        assert_eq!(info.freq_khz, Some(904_500));
        let s1g = info.s1g.unwrap();
        assert_eq!(s1g.mcs, Some(9));
        assert_eq!(s1g.bandwidth_mhz, Some(4));
        assert_eq!(s1g.ppdu_format, Some(S1gPpduFormat::S1gLong));
    }

    /// A `known` mask with a bit clear means "the driver did not say", and must not become a zero
    /// that reads like data. Newracom's 0x007F omits UPL_IND; Morse's 0x00F7 omits NSS — which is
    /// why there is no `nss` field at all (see [`S1gInfo`]).
    #[test]
    fn unknown_s1g_subfields_stay_none() {
        // known = 0 ⇒ every gated field is None, but the ungated signal byte is still read.
        let h = nrc_header(0, 0xffff, 0x7f00);
        let mut raw = h.clone();
        // Overwrite the TLV's `known` word (offset 24 + 4) with zero.
        raw[28] = 0;
        raw[29] = 0;
        let s1g = parse(&raw).unwrap().s1g.unwrap();
        assert_eq!(s1g.mcs, None);
        assert_eq!(s1g.bandwidth_mhz, None);
        assert_eq!(s1g.ppdu_format, None);
        assert_eq!(s1g.short_gi, None);
        assert_eq!(s1g.bss_color, None);
        assert_eq!(
            s1g.rssi_dbm,
            Some(127),
            "no known bit gates the signal field"
        );
    }

    /// `DOT11_RT_S1G_BW_INVALID` (5) and the reserved PPDU format (3) are refusals, not values —
    /// the driver logs "Packet with invalid BW" and still emits the frame.
    #[test]
    fn invalid_s1g_encodings_are_none_not_guesses() {
        let h = nrc_header(0, 3 | (5 << 8), 0);
        let s1g = parse(&h).unwrap().s1g.unwrap();
        assert_eq!(s1g.bandwidth_mhz, None, "BW enum 5 = INVALID");
        assert_eq!(s1g.ppdu_format, None, "PPDU format 3 is reserved");
    }

    /// An NDP frame sets `ZERO_LEN_PSDU` (bit 26), which we do not model. Because we cannot know
    /// where a TLV region would start behind it, we decode the standard fields and stop — rather
    /// than walking at a guessed offset and reporting a fabricated S1G descriptor.
    #[test]
    fn unmodelled_field_before_the_tlvs_suppresses_the_walk() {
        // `struct nrc_radiotap_hdr_ndp`: TSFT, FLAGS, pad, CHANNEL, then a 1-byte 0-len-PSDU.
        let mut b = Vec::new();
        b.extend_from_slice(&5u64.to_le_bytes());
        b.push(0x00); // rt_flags = 0 for NDP (no FCS appended)
        b.push(0);
        b.extend_from_slice(&925u16.to_le_bytes());
        b.extend_from_slice(&0x0140u16.to_le_bytes());
        b.push(0x02); // rt_zero_length_psdu
        let h = radiotap((1 << 0) | (1 << 1) | (1 << 3) | (1 << 26) | (1 << 28), &b);
        let info = parse(&h).expect("still a valid header");
        assert_eq!(info.tsft, Some(5));
        assert!(!info.fcs_included(), "NDP frames carry no FCS");
        assert_eq!(info.s1g, None, "must not guess a TLV offset");
    }

    /// Malformed TLVs must terminate the walk, never over-read the buffer and never spin.
    #[test]
    fn malformed_tlvs_terminate_the_walk() {
        // (a) a TLV whose declared length runs past it_len.
        let mut b = Vec::new();
        b.extend_from_slice(&0u64.to_le_bytes());
        b.push(0);
        b.push(0);
        b.extend_from_slice(&0u16.to_le_bytes());
        b.extend_from_slice(&0u16.to_le_bytes());
        b.extend_from_slice(&[0, 0]);
        b.extend_from_slice(&TLV_TYPE_S1G.to_le_bytes());
        b.extend_from_slice(&4096u16.to_le_bytes()); // absurd length
        let h = radiotap((1 << 0) | (1 << 1) | (1 << 3) | (1 << 28), &b);
        assert_eq!(parse(&h).unwrap().s1g, None);

        // (b) a zero-length TLV of an unknown type must still advance (4 bytes of header),
        //     and a run of them must terminate at it_len.
        let mut b2 = Vec::new();
        b2.extend_from_slice(&0u64.to_le_bytes());
        b2.push(0);
        b2.push(0);
        b2.extend_from_slice(&0u16.to_le_bytes());
        b2.extend_from_slice(&0u16.to_le_bytes());
        b2.extend_from_slice(&[0, 0]);
        for _ in 0..4 {
            b2.extend_from_slice(&99u16.to_le_bytes());
            b2.extend_from_slice(&0u16.to_le_bytes());
        }
        b2.extend_from_slice(&s1g_tlv(0x0020, 6 << 12, 0, false));
        let h2 = radiotap((1 << 0) | (1 << 1) | (1 << 3) | (1 << 28), &b2);
        assert_eq!(parse(&h2).unwrap().s1g.unwrap().mcs, Some(6));
    }

    /// The S1G TX header we build is still valid radiotap, carries no S1G descriptor (we name no
    /// rate on this bearer — the on-chip MAC owns it) and, crucially, sets no FCS flag, so the
    /// round trip cannot accidentally trim four bytes off an injected frame.
    #[test]
    fn s1g_tx_header_round_trips_without_claiming_metadata() {
        let h = build_tx_s1g();
        let info = parse(&h).expect("TX header is valid radiotap");
        assert_eq!(info.header_len, TX_S1G_HEADER_LEN);
        assert_eq!(info.s1g, None);
        assert_eq!(info.flags, None);
        assert!(!info.fcs_included());
        assert!(!info.bad_fcs());
    }

    /// `F_BADFCS` is the one flag whose whole job is to make us drop a frame.
    #[test]
    fn bad_fcs_is_visible() {
        let h = {
            let data1 = 1;
            let mut raw = morse_header(1, -50, 0, 900_000, data1, 0);
            raw[16] = F_FCS | F_BADFCS; // rt_flags, at offset 16
            raw
        };
        let info = parse(&h).unwrap();
        assert!(info.fcs_included());
        assert!(info.bad_fcs());
    }

    #[test]
    fn parse_rejects_garbage() {
        assert_eq!(parse(&[]), None);
        assert_eq!(parse(&[1, 0, 8, 0, 0, 0, 0, 0]), None, "bad version");
        assert_eq!(parse(&[0, 0, 0xff, 0xff, 0, 0, 0, 0]), None, "len > buf");
    }
}
