//! The **connac2 descriptor layer**: the variable-length RX descriptor (RXD) and the
//! 8-dword TX descriptor (TXD) that the MT7921AU (`0e8d:7961`, `mt7961` firmware) speaks.
//!
//! Everything in this file is a **pure function over a byte slice** — no device, no I/O, no
//! locks. That is deliberate: the descriptor layer is where a Wi-Fi port is usually wrong in
//! a way that no register dump can expose (an off-by-one group advance shifts every field
//! after it, and the result still *looks* like a plausible frame), so it is the one layer
//! that must be testable without hardware. [`parse_rxd`] and [`build_txd`] are exercised by
//! the unit tests at the bottom of this file with the dongle unplugged.
//!
//! # ★ Why this part earns its place in the crate
//!
//! **A per-frame hardware RX timestamp.** `mt7921/mac.c:307-309` latches
//!
//! ```text
//!     if (rxd1 & MT_RXD1_NORMAL_GROUP_2) {
//!             status->timestamp = le32_to_cpu(rxd[0]);
//!             status->flag |= RX_FLAG_MACTIME_START;
//! ```
//!
//! The mt76x0/mt76x2 parts this crate already carries have **no such field** — `struct
//! mt76x02_rxwi` (`mt76x02_mac.h:97-108`) is `rxinfo/ctl/tid_sn/rate/rssi[4]/bbp_rxinfo[4]`
//! and not one of those is a time — which is why those two backends can only ever offer a
//! *read-now* `PortTsf` costing a 151 µs EP0 round trip. This part stamps the frame in the
//! MAC and ships the stamp inside the descriptor, so it can be a genuine
//! `RadioClockKind::FreeRunRxStamp` and therefore a **common-view participant**, like the
//! Realtek a81a/8733b (`RXTSFL`) and the AR9271. See [`Rxd::timestamp`] for the width, the
//! tick, the wrap and exactly how much of that is measured (none of it, yet).
//!
//! **802.11ax.** [`encode_rate`] is the first Wi-Fi actuator in this crate for
//! `McsDescriptor::{he, dcm, er_su}`. Read its doc comment before believing it: upstream
//! **never transmits a fixed HT/VHT/HE rate on this chip** (`mt76_connac_mac.c:321-324`
//! short-circuits `is_connac2` straight to the legacy branch), so the HE encoding here is
//! assembled from the field masks plus the mt7915 testmode path, not from a code path that
//! has ever run on a 7961.
//!
//! # MEASURED vs CODE-READ
//!
//! **MEASURED on mds-o5p-3's MT7921AU (2026-08-27), and this file depends on it:**
//!   * `MT_HW_CHIPID (0x70010200) = 0x7961`. That is what makes `is_connac2(dev)` true
//!     (`mt76_connac.h:280-284`), and every `is_connac2` branch quoted below is therefore
//!     the branch this silicon takes. Three of them change the descriptor:
//!     `MT_TXD1_VTA` is **not** set, `MT_TXD3_SW_POWER_MGMT` is **not** set
//!     (`mt76_connac_mac.c:560-570`), and the fixed-rate word is legacy-only
//!     (`:321-324`).
//!   * `MT_HW_REV (0x70010204) = 0x8a10`, matching the patch header's `hw_sw_ver`.
//!
//! **CODE-READ, unvalidated on silicon — everything else in this file**, in particular:
//!   * the group sizes and their order in the descriptor,
//!   * that group 2 is present at all on a monitor-mode RX (nothing upstream gates it; see
//!     [`Rxd::timestamp`]),
//!   * the 1 µs tick of that timestamp,
//!   * every HE bit in [`encode_rate`].
//!
//! The gate that turns these into measurements is a capture on the target: dump the first
//! 144 bytes of a bulk-IN transfer, check `rxd1` bits 11-15 against the byte offset at which
//! the 802.11 header actually starts, and difference the group-2 dword against
//! `MT_LPON_UTTR0` read immediately after.
//!
//! # The two configuration facts a caller must act on
//!
//! 1. **`MT_MDP_DCR0_RX_HDR_TRANS_EN` (bit 19 of `0x820cd000`) must be CLEARED.**
//!    `mt7921/init.c:72` *sets* it, and with it set the hardware rewrites every data
//!    frame's 802.11 header into an 802.3 one before the host sees it — [`Rxd::hdr_trans`]
//!    goes true and there is no 802.11 header left to parse. mt7915 clears it for monitor
//!    mode for exactly this reason (`mt7915/main.c:509`). A named-radio face that leaves it
//!    set receives Ethernet frames and reports zero NDN frames, which reads identically to a
//!    dead receiver.
//! 2. **`MT_DMA_DCR0_RXD_G5_EN` (bit 23 of `0x820e7000`) must be SET for monitor RSSI.**
//!    `mt792x_mac.c:302-304` clears it — *"disable rx rate report by default due to hw
//!    issues"* — and with it clear there is no group 5, so the per-chain RCPI comes from
//!    P-RXV DW1 rather than the C-RXV. Upstream's own comment at `mt7921/mac.c:356-358`
//!    says monitor mode wants the group-5 RCPI. Both paths are implemented here
//!    ([`Rxd::rcpi_from_crxv`] says which one produced the numbers); the choice is the
//!    backend's.
//!
//! # Deliberate omissions, so they are visible rather than lost
//!
//!   * **Decryption / IV handling.** Group 1 carries the CCMP/TKIP/GCMP IV
//!     (`mt7921/mac.c:276-305`) and is *walked* here so the cursor lands right, but the six
//!     IV bytes are not extracted: this driver runs a promiscuous monitor with no hardware
//!     keys, so `SEC_MODE` is always 0 and the group is always absent. The walk is
//!     implemented anyway because "always absent" is a claim about a configuration, not
//!     about the descriptor format.
//!   * **A-MSDU de-aggregation.** [`Rxd::amsdu`] is decoded and the 2-byte subframe pad is
//!     removed, but splitting an A-MSDU into its subframes is a frame-layer job, not a
//!     descriptor one.
//!   * **`mt76_connac2_reverse_frag0_hdr_trans`** (`mt76_connac_mac.c:964-1037`): rebuilding
//!     an 802.11 header for a *translated* fragment. It needs the vif's addresses, which a
//!     header-translation-free monitor never has and never wants. See fact 1 above.
//!   * **TXS (transmit status) parsing.** `MT_TXS4_TIMESTAMP` (`mt76_connac2_mac.h:174`) is
//!     a 32-bit stamp in the *same* domain as [`Rxd::timestamp`], which is what would make
//!     a TX/RX common view possible on one part — the AR9271 lesson (a firmware TX counter
//!     was the enabler there, not the RX path). [`decode_rate`] is the half of the TXS
//!     decoder this file needs; the rest belongs with the TXS ring, not here.
#![allow(dead_code)]

use crate::FaceError;
use crate::McsDescriptor;

use super::regs::{field_get, field_prep};

// ════════════════════════════════════════════════════════════════════════════
// RX — the variable-length RXD
// ════════════════════════════════════════════════════════════════════════════

// ── RXD DW0 (mt76_connac2_mac.h:184-191) ────────────────────────────────────

/// Total length of this RX unit **in bytes, including the RXD itself**.
///
/// This is also the USB framing length: `mt7921u` sets `MT_DRV_RX_DMA_HDR`
/// (`mt7921/usb.c:153`), so `mt76u_get_rx_entry_len` (`usb.c:466-480`) reads the same
/// little-endian u16 straight off byte 0 of the bulk-IN transfer and takes it as the whole
/// entry — there is **no separate 4-byte DMA header** in front of the descriptor the way
/// there is on the mt76x0/mt76x2 (`crate::mt76x0`'s `MT_DMA_HDR_LEN`). Byte 0 of the
/// transfer *is* `rxd[0]`.
pub const MT_RXD0_LENGTH: u32 = 0x0000_ffff;
/// Packet flag; `mt7921/mac.c:597-600` uses it only to re-label a `PKT_TYPE_RX_EVENT`
/// carrying flag 1 as `PKT_TYPE_NORMAL_MCU`.
pub const MT_RXD0_PKT_FLAG: u32 = 0x000f_0000;
/// `enum rx_pkt_type` (`mt76_connac.h:9-21`) — which of the several descriptor shapes this
/// unit is. Only [`PKT_TYPE_NORMAL`] and [`PKT_TYPE_NORMAL_MCU`] carry an 802.11 frame.
pub const MT_RXD0_PKT_TYPE: u32 = 0xf800_0000;

/// Checksum-offload results, MMIO-only (`mt7921/mac.c:240-242` gates them on
/// `mt76_is_mmio`). Recorded because they alias the group-4 `ETH_TYPE_OFS` field and a
/// reader who forgets that will "find" a checksum bit in an 802.11 frame.
pub const MT_RXD0_NORMAL_IP_SUM: u32 = 1 << 23;
/// See [`MT_RXD0_NORMAL_IP_SUM`].
pub const MT_RXD0_NORMAL_UDP_TCP_SUM: u32 = 1 << 24;

/// A normal 802.11 frame (`mt76_connac.h:12`).
pub const PKT_TYPE_NORMAL: u8 = 2;
/// A normal frame that arrived via the MCU path (`mt76_connac.h:18`). `mt7921/mac.c:621`
/// hands it to exactly the same parser, so this file treats the two identically.
pub const PKT_TYPE_NORMAL_MCU: u8 = 8;
/// A standalone RX-vector report (`mt76_connac.h:11`). Not an 802.11 frame; it is the only
/// packet on this chip that carries a C-RXV long enough to hold SNR/CFO — see
/// [`crxv_snr_db`].
pub const PKT_TYPE_TXRXV: u8 = 1;
/// A transmit-status report (`mt76_connac.h:10`); `mt7921/mac.c:614-618` walks it in
/// 8-dword strides starting at `rxd + 2`.
pub const PKT_TYPE_TXS: u8 = 0;

// ── RXD DW1 (mt76_connac2_mac.h:193-210) ────────────────────────────────────

/// Station index this frame was matched to; `0x3ff` / no match on a promiscuous monitor.
pub const MT_RXD1_NORMAL_WLAN_IDX: u32 = 0x0000_03ff;
/// Group 1 present: 4 dwords of CCMP/TKIP/GCMP IV + EIV.
pub const MT_RXD1_NORMAL_GROUP_1: u32 = 1 << 11;
/// ★ Group 2 present: 2 dwords, the first of which is the MAC timestamp. See
/// [`Rxd::timestamp`].
pub const MT_RXD1_NORMAL_GROUP_2: u32 = 1 << 12;
/// Group 3 present: 2 dwords of P-RXV (the rate/NSS/BW/RCPI vector).
pub const MT_RXD1_NORMAL_GROUP_3: u32 = 1 << 13;
/// Group 4 present: 4 dwords holding a copy of frame-control / TA / seq-ctl / QoS-ctl.
pub const MT_RXD1_NORMAL_GROUP_4: u32 = 1 << 14;
/// Group 5 present: 18 dwords of C-RXV (the full baseband receive vector).
pub const MT_RXD1_NORMAL_GROUP_5: u32 = 1 << 15;
/// Cipher suite the hardware used, `enum mt76_cipher_type`. Non-zero implies group 1.
pub const MT_RXD1_NORMAL_SEC_MODE: u32 = 0x001f_0000;
/// Key index within the cipher suite.
pub const MT_RXD1_NORMAL_KEY_ID: u32 = 0x0060_0000;
/// "Cipher mismatch" — the frame was encrypted and we had no matching key.
pub const MT_RXD1_NORMAL_CM: u32 = 1 << 23;
/// "Cipher / length mismatch".
pub const MT_RXD1_NORMAL_CLM: u32 = 1 << 24;
/// Integrity-check-value error (or a CCMP/BIP/WPI MIC error).
pub const MT_RXD1_NORMAL_ICV_ERR: u32 = 1 << 25;
/// TKIP Michael MIC error.
pub const MT_RXD1_NORMAL_TKIP_MIC_ERR: u32 = 1 << 26;
/// ★ The hardware demodulated a PPDU and its FCS failed. The FCS itself is **not** in the
/// buffer — `mt7921_mac_fill_rx` never sets `RX_FLAG_INCLUDE_FCS`, so the four trailing
/// bytes are stripped by hardware.
pub const MT_RXD1_NORMAL_FCS_ERR: u32 = 1 << 27;
/// Which band/PHY received the frame. `mt7921/mac.c:195-196` rejects the frame outright
/// when it is set: the 7921 is a single-PHY part, so a set bit means the descriptor is not
/// what we think it is.
pub const MT_RXD1_NORMAL_BAND_IDX: u32 = 1 << 28;

// ── RXD DW2 (mt76_connac2_mac.h:212-230) ────────────────────────────────────

/// MAC header length as the hardware measured it. **Units undetermined**: nothing in the
/// upstream tree reads this field — `mt7921_mac_fill_rx` calls
/// `ieee80211_get_hdrlen_from_skb` on the payload instead — so there is no attested
/// interpretation to port. Exposed raw as [`Rxd::mac_hdr_len_raw`] and used by nothing;
/// [`dot11_hdr_len`] derives the length from frame-control the way upstream does.
pub const MT_RXD2_NORMAL_MAC_HDR_LEN: u32 = 0x0000_1f00;
/// ★ The hardware rewrote the 802.11 header into an 802.3 one. See fact 1 in the module
/// header — a monitor must clear `MT_MDP_DCR0_RX_HDR_TRANS_EN` so this is never set.
pub const MT_RXD2_NORMAL_HDR_TRANS: u32 = 1 << 13;
/// Padding inserted **before** the 802.11 header, in units of 2 bytes (`hdr_gap = cursor +
/// 2 * remove_pad`, `mt7921/mac.c:389`). This is the alignment pad, not the A-MSDU pad.
pub const MT_RXD2_NORMAL_HDR_OFFSET: u32 = 0x0000_c000;
/// QoS TID.
pub const MT_RXD2_NORMAL_TID: u32 = 0x000f_0000;
/// A-MSDU parse error — `mt7921/mac.c:201-202` drops the frame.
pub const MT_RXD2_NORMAL_AMSDU_ERR: u32 = 1 << 23;
/// Frame longer than the configured `MT_DMA_DCR0_MAX_RX_LEN` — dropped (`:259-260`).
pub const MT_RXD2_NORMAL_MAX_LEN_ERROR: u32 = 1 << 24;
/// Header translation was attempted and failed.
pub const MT_RXD2_NORMAL_HDR_TRANS_ERROR: u32 = 1 << 25;
/// The MPDU is a fragment (used upstream only to decide whether to synthesise a CCMP
/// header, `mt7921/mac.c:284-285`).
pub const MT_RXD2_NORMAL_FRAG: u32 = 1 << 27;
/// ★ Inverted A-MPDU flag: **clear** means this MPDU was part of an A-MPDU
/// (`mt7921/mac.c:311`). [`Rxd::ampdu`] flips it so the field reads the way it is named.
pub const MT_RXD2_NORMAL_NON_AMPDU: u32 = 1 << 30;

// ── RXD DW3 (mt76_connac2_mac.h:248-266) ────────────────────────────────────

/// Sequence number of the RX vector this frame's P-RXV/C-RXV belongs to.
pub const MT_RXD3_NORMAL_RXV_SEQ: u32 = 0x0000_00ff;
/// The channel the frame arrived on, as a **hardware channel index**, not a frequency.
/// `mt792x_get_status_freq_info` (`mt792x_core.c`) turns it into a band + centre frequency;
/// this file passes it through raw because the mapping is a per-band table, not a
/// descriptor fact.
pub const MT_RXD3_NORMAL_CH_FREQ: u32 = 0x0000_ff00;
/// Address-match class: `1` (= [`MT_RXD3_NORMAL_U2M`]) means unicast-to-us.
pub const MT_RXD3_NORMAL_ADDR_TYPE: u32 = 0x0003_0000;
/// The `ADDR_TYPE` value meaning unicast-to-us (`mt7921/mac.c:213`).
pub const MT_RXD3_NORMAL_U2M: u32 = 1;
/// The frame carried an HT Control field.
pub const MT_RXD3_NORMAL_HTC_VLD: u32 = 1;
/// The frame is an A-MSDU.
pub const MT_RXD3_NORMAL_AMSDU: u32 = 1 << 22;

// ── RXD DW4 (mt76_connac2_mac.h:232-244) ────────────────────────────────────

/// A-MSDU subframe position, 2 bits (`mt7921/mac.c:382-387`): 0 = not an A-MSDU,
/// [`MT_RXD4_FIRST_AMSDU_FRAME`] = first, [`MT_RXD4_LAST_AMSDU_FRAME`] = last.
pub const MT_RXD4_NORMAL_PAYLOAD_FORMAT: u32 = 0x0000_0003;
/// The `PAYLOAD_FORMAT` value for the first subframe of an A-MSDU. Upstream spells this
/// `GENMASK(1, 0)` = 3 (`mt76_connac2_mac.h:234`) and compares it for equality, which is
/// how a two-bit position field ends up with a "mask" as one of its values.
pub const MT_RXD4_FIRST_AMSDU_FRAME: u32 = 3;
/// The `PAYLOAD_FORMAT` value for the last subframe of an A-MSDU.
pub const MT_RXD4_LAST_AMSDU_FRAME: u32 = 1;

// ── RXD group 4 (mt76_connac2_mac.h:268-277) ────────────────────────────────

/// Frame-control, copied out of the MPDU into RXD6 by the hardware.
pub const MT_RXD6_FRAME_CONTROL: u32 = 0x0000_ffff;
/// Low 16 bits of the transmitter address (RXD6), high 32 in RXD7.
pub const MT_RXD6_TA_LO: u32 = 0xffff_0000;
/// Sequence-control (RXD8 low half).
pub const MT_RXD8_SEQ_CTRL: u32 = 0x0000_ffff;
/// QoS-control (RXD8 high half).
pub const MT_RXD8_QOS_CTL: u32 = 0xffff_0000;

// ── P-RXV DW0 (mt76_connac2_mac.h:279-293) ──────────────────────────────────

/// The 7-bit transmit-rate field. Its interpretation depends on [`MT_PRXV_TX_MODE`]:
/// CCK/OFDM/HT use the whole field as a rate index, VHT/HE use only bits 3:0 as an MCS and
/// bits 4/5 as [`MT_PRXV_TX_DCM`] / [`MT_PRXV_TX_ER_SU_106T`].
///
/// ★ Bit-for-bit the same encoding as the **TX** side's [`MT_TX_RATE_IDX`] /
/// [`MT_TX_RATE_DCM`] / [`MT_TX_RATE_SU_EXT_TONE`]. That correspondence is the strongest
/// evidence available for [`encode_rate`]'s HE layout, since no upstream path transmits a
/// fixed HE rate on this chip.
pub const MT_PRXV_TX_RATE: u32 = 0x0000_007f;
/// HE dual-carrier modulation, **inside** the rate field (`mt76_connac_mac.c:1060`:
/// `dcm = !!(idx & MT_PRXV_TX_DCM)` on the already-extracted index).
pub const MT_PRXV_TX_DCM: u32 = 1 << 4;
/// HE ER-SU 106-tone variant, also inside the rate field (`mt76_connac_mac.c:1122`).
pub const MT_PRXV_TX_ER_SU_106T: u32 = 1 << 5;
/// Number of space-time streams **minus one** (`mt76_connac_mac.c:1053`: `nss = NSTS + 1`).
pub const MT_PRXV_NSTS: u32 = 0x0000_0380;
/// The PPDU was beamformed.
pub const MT_PRXV_TXBF: u32 = 1 << 10;
/// LDPC (the "advanced code") rather than BCC — `mt7921/mac.c:342-343`.
pub const MT_PRXV_HT_AD_CODE: u32 = 1 << 11;
/// Bandwidth: 0/1/2/3 = 20/40/80/160 MHz (`IEEE80211_STA_RX_BW_*`).
pub const MT_PRXV_FRAME_MODE: u32 = 0x0000_7000;
/// Guard interval. Non-zero = short GI for HT/VHT; for HE it is the 2-bit HE-GI code.
pub const MT_PRXV_HT_SGI: u32 = 0x0001_8000;
/// Space-time block coding.
pub const MT_PRXV_HT_STBC: u32 = 0x00c0_0000;
/// PHY mode, `enum mt76_phy_type` (`mt76.h:338-353`) — see [`RateMode`].
pub const MT_PRXV_TX_MODE: u32 = 0x0f00_0000;

// P-RXV DW0 also defines `MT_PRXV_DCM` = BIT(17) and `MT_PRXV_NUM_RX`
// (`mt76_connac2_mac.h:292-293`). Neither is ported:
//   * `MT_PRXV_DCM` is the **non**-connac2 source of the DCM bit
//     (`mt76_connac_mac.c:1059-1062` picks `MT_PRXV_TX_DCM` when `is_connac2`), and this
//     chip is connac2 (MEASURED chip id 0x7961).
//   * `MT_PRXV_NUM_RX` is written `BIT(20, 18)` upstream — two arguments to a one-argument
//     macro. It is a `GENMASK` typo that compiles only because nothing expands it. There is
//     no attested field here to port.

// ── P-RXV DW1 (mt76_connac2_mac.h:295-300) ──────────────────────────────────

/// Received-channel-power indicator for chain 0, one byte per chain across DW1. Raw RCPI;
/// [`rcpi_to_dbm`] converts.
pub const MT_PRXV_RCPI0: u32 = 0x0000_00ff;
/// Chain 1 RCPI.
pub const MT_PRXV_RCPI1: u32 = 0x0000_ff00;
/// Chain 2 RCPI (always 0 on this 2×2 part).
pub const MT_PRXV_RCPI2: u32 = 0x00ff_0000;
/// Chain 3 RCPI (always 0 on this 2×2 part).
pub const MT_PRXV_RCPI3: u32 = 0xff00_0000;

// ── C-RXV (mt76_connac2_mac.h:302-333) ──────────────────────────────────────

/// C-RXV dword 0: STBC, as the baseband saw it (the mt7915 branch of
/// `mt76_connac2_mac_fill_rx_rate` reads this dword; `mt76_connac_mac.c:1065-1069`).
pub const MT_CRXV_HT_STBC: u32 = 0x0000_0003;
/// C-RXV dword 0: PHY mode.
pub const MT_CRXV_TX_MODE: u32 = 0x0000_00f0;
/// C-RXV dword 0: bandwidth.
pub const MT_CRXV_FRAME_MODE: u32 = 0x0000_0700;
/// C-RXV dword 0: guard interval.
pub const MT_CRXV_HT_SHORT_GI: u32 = 0x0000_6000;
/// C-RXV dword 0: HE LTF size (`mt76_connac_mac.c:898` reads it as `size + 1`).
pub const MT_CRXV_HE_LTF_SIZE: u32 = 0x0006_0000;
/// C-RXV dword 20: signal-to-noise ratio, biased by 16 (`mt7915/mac.c:586`:
/// `snr = FIELD_GET(MT_CRXV_SNR, v20) - 16`). **Not reachable from a normal RXD** — see
/// [`crxv_snr_db`].
pub const MT_CRXV_SNR: u32 = 0x0007_e000;
/// C-RXV dword 20: low 13 bits of the frequency-offset estimate.
pub const MT_CRXV_FOE_LO: u32 = 0xfff8_0000;
/// C-RXV dword 21: high 7 bits of the frequency-offset estimate.
pub const MT_CRXV_FOE_HI: u32 = 0x0000_007f;
/// Shift applied to [`MT_CRXV_FOE_HI`] when recombining (`mt76_connac2_mac.h:333`).
pub const MT_CRXV_FOE_SHIFT: u32 = 13;

// ── RXD geometry ────────────────────────────────────────────────────────────
//
// ★ The whole point of this block. The RXD is a fixed 6-dword head followed by up to five
// optional groups, each selected by a bit in `rxd[1]`. `mt7921_mac_fill_rx` walks them in a
// fixed order — 4, 1, 2, 3, 5 (`mt7921/mac.c:262-364`) — advancing the cursor by a constant
// per group. That walk order **is** the physical order in the descriptor: the code has one
// cursor and never seeks backwards.
//
// Get one advance wrong and every field after it is read from the wrong dword, and the
// frame still parses into something that looks like a frame. Hence the constants below,
// each with the line that establishes it, and the cursor tests at the bottom of the file.

/// The always-present head: `rxd[0..6]`. Dwords 0-4 are decoded; dword 5 is not read by
/// upstream at all and is not decoded here either. (`mt7921/mac.c:262`: `rxd += 6`.)
pub const RXD_FIXED_DWORDS: usize = 6;
/// Group 4 — frame-control / TA / seq-ctl / QoS-ctl (`mt7921/mac.c:271`: `rxd += 4`).
pub const RXD_GROUP4_DWORDS: usize = 4;
/// Group 1 — CCMP/TKIP IV + EIV (`mt7921/mac.c:302`: `rxd += 4`).
pub const RXD_GROUP1_DWORDS: usize = 4;
/// ★ Group 2 — the MAC timestamp (`mt7921/mac.c:324`: `rxd += 2`).
pub const RXD_GROUP2_DWORDS: usize = 2;
/// Group 3 — the P-RXV (`mt7921/mac.c:335`: `rxd += 2`).
pub const RXD_GROUP3_DWORDS: usize = 2;
/// Group 5 — the C-RXV. `mt7921/mac.c:351,361` splits the advance as `6 + 12` because it
/// wants the dword in between; `mt7915/mac.c:448` does the same region in one `rxd += 18`.
/// Two independent statements of 18, which is why this constant is trusted.
pub const RXD_GROUP5_DWORDS: usize = 18;
/// Dword offset of the monitor-mode RCPI **within** the C-RXV (`mt7921/mac.c:351-359`
/// advances 6, then reads `rxv[0]`). Cross-checked by `mt7915/mac.c:567`, which reads
/// `rcpi = rxv[6]` out of a standalone RX-vector report.
pub const CRXV_RCPI_DWORD: usize = 6;
/// The largest an RXD can be: head + every group = 36 dwords = 144 bytes.
pub const RXD_MAX_DWORDS: usize = RXD_FIXED_DWORDS
    + RXD_GROUP4_DWORDS
    + RXD_GROUP1_DWORDS
    + RXD_GROUP2_DWORDS
    + RXD_GROUP3_DWORDS
    + RXD_GROUP5_DWORDS;

// ── Decoded types ───────────────────────────────────────────────────────────

/// `enum mt76_phy_type` (`mt76.h:338-353`) — the PPDU format. The discriminants are the
/// wire values and are shared by the RX P-RXV ([`MT_PRXV_TX_MODE`]) and the TX rate word
/// ([`MT_TX_RATE_MODE`]), which is what lets [`encode_rate`] and [`decode_rate`] be
/// inverses.
///
/// The EHT modes (13-15) exist in the enum upstream but not on this silicon; they are not
/// represented here, and [`RateMode::from_code`] returns `None` for them rather than
/// inventing a variant an 11ax part can never report.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum RateMode {
    /// DSSS/CCK — 1/2/5.5/11 Mbps, 2.4 GHz only.
    Cck = 0,
    /// Legacy OFDM — 6…54 Mbps.
    Ofdm = 1,
    /// 802.11n mixed-mode.
    Ht = 2,
    /// 802.11n greenfield. Received but never transmitted here.
    HtGf = 3,
    /// 802.11ac.
    Vht = 4,
    /// ★ 802.11ax single-user.
    HeSu = 8,
    /// ★ 802.11ax extended-range single-user — the `McsDescriptor::er_su` lever.
    HeExtSu = 9,
    /// 802.11ax trigger-based (uplink OFDMA). Receive-only for us: transmitting one
    /// requires a trigger frame from an AP.
    HeTb = 10,
    /// 802.11ax multi-user (downlink OFDMA/MU-MIMO). Receive-only for us.
    HeMu = 11,
}

impl RateMode {
    /// Decode the 4-bit wire value, or `None` if it is one this part cannot produce.
    ///
    /// `mt76_connac2_mac_fill_rx_rate` returns `-EINVAL` for anything outside this set
    /// (`mt76_connac_mac.c:1112-1113`), and so does [`parse_rxd`].
    pub const fn from_code(v: u8) -> Option<Self> {
        match v {
            0 => Some(RateMode::Cck),
            1 => Some(RateMode::Ofdm),
            2 => Some(RateMode::Ht),
            3 => Some(RateMode::HtGf),
            4 => Some(RateMode::Vht),
            8 => Some(RateMode::HeSu),
            9 => Some(RateMode::HeExtSu),
            10 => Some(RateMode::HeTb),
            11 => Some(RateMode::HeMu),
            _ => None,
        }
    }

    /// The wire value.
    pub const fn code(self) -> u8 {
        self as u8
    }

    /// Is this one of the four 802.11ax formats? Matches upstream's `mode >=
    /// MT_PHY_TYPE_HE_SU` test (`mt7921/mac.c:434`), which works only because HE_SU is 8
    /// and nothing between 5 and 7 exists.
    pub const fn is_he(self) -> bool {
        (self as u8) >= (RateMode::HeSu as u8)
    }

    /// Does the index field carry an MCS (HT/VHT/HE) rather than a legacy rate code?
    pub const fn has_mcs(self) -> bool {
        !matches!(self, RateMode::Cck | RateMode::Ofdm)
    }
}

/// Receive bandwidth, decoded from [`MT_PRXV_FRAME_MODE`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RxBw {
    /// 20 MHz.
    Bw20,
    /// 40 MHz.
    Bw40,
    /// 80 MHz.
    Bw80,
    /// 160 MHz.
    Bw160,
    /// ★ Not a bandwidth at all: an HE ER-SU PPDU on the 106-tone resource unit. The
    /// hardware reports `FRAME_MODE = 40 MHz` and sets [`MT_PRXV_TX_ER_SU_106T`] inside the
    /// rate field; `mt76_connac_mac.c:1121-1125` translates that pair into
    /// `RATE_INFO_BW_HE_RU` + `HE_RU_ALLOC_106`. Kept as its own variant because calling it
    /// "40 MHz" would be wrong in the direction that matters — a 106-tone ER-SU is
    /// *narrower* than 20 MHz, not wider.
    HeRu106,
}

/// The decoded P-RXV rate vector — one PPDU's modulation, as the baseband demodulated it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RxRate {
    /// PPDU format.
    pub mode: RateMode,
    /// The raw 7-bit [`MT_PRXV_TX_RATE`] field, before per-mode interpretation. Kept
    /// because for HE it also carries DCM and the 106-tone bit, and a caller that wants to
    /// re-encode the same rate needs the bits, not the interpretation.
    pub raw_idx: u8,
    /// MCS index for HT (0-15 on this 2×2 part) / VHT (0-9) / HE (0-11), masked to 4 bits
    /// for HE per `mt76_connac_mac.c:1105`; `None` for CCK/OFDM, which have no MCS.
    /// Reporting a legacy rate code as an MCS is exactly how a legacy frame comes to look
    /// like an HT one in a rate histogram.
    pub mcs: Option<u8>,
    /// Legacy rate code for CCK/OFDM (the `mt76_rates` `hw_value` low byte, `mt76.h:1180-
    /// 1192`); `None` for HT/VHT/HE. See [`LegacyRate::from_code`] to name it.
    pub legacy_code: Option<u8>,
    /// Spatial streams. The hardware reports space-*time* streams; upstream halves it when
    /// STBC is on (`mt76_connac_mac.c:1072-1074`) because an STBC PPDU carries one spatial
    /// stream over two space-time streams. That correction is applied here.
    pub nss: u8,
    /// Bandwidth (or the ER-SU 106-tone marker).
    pub bw: RxBw,
    /// Short guard interval — HT/VHT only (`mt76_connac_mac.c:1141-1142` only sets it below
    /// `MT_PHY_TYPE_HE_SU`).
    pub short_gi: bool,
    /// HE guard-interval code 0-3 (0.8/1.6/3.2 µs and the reserved value); `None` outside
    /// HE. `mt76_connac_mac.c:1107-1108` accepts it only when `gi <= HE_GI_3_2`.
    pub he_gi: Option<u8>,
    /// HE dual-carrier modulation.
    pub dcm: bool,
    /// Raw 2-bit STBC field (0 = off).
    pub stbc: u8,
    /// LDPC rather than BCC (`MT_PRXV_HT_AD_CODE`).
    pub ldpc: bool,
    /// The PPDU was beamformed.
    pub txbf: bool,
}

/// Where a frame's A-MSDU subframe sits, from [`MT_RXD4_NORMAL_PAYLOAD_FORMAT`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AmsduPos {
    /// Not an A-MSDU.
    None,
    /// First subframe.
    First,
    /// A middle subframe.
    Mid,
    /// Last subframe.
    Last,
}

/// Everything [`parse_rxd`] could establish about one RX unit.
///
/// Fields that the descriptor did not carry are `Option::None` — never a fabricated zero.
/// That rule is the whole reason this struct exists rather than a flat `[u32; N]`: a
/// timestamp of 0 and "no timestamp" are the difference between a working common-view clock
/// and a silently broken one.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Rxd {
    /// Total bytes of this RX unit, descriptor included ([`MT_RXD0_LENGTH`]). A caller
    /// walking a bulk-IN transfer advances by exactly this.
    pub unit_len: usize,
    /// `enum rx_pkt_type` — [`PKT_TYPE_NORMAL`] / [`PKT_TYPE_NORMAL_MCU`] are the only two
    /// this struct's frame fields mean anything for.
    pub pkt_type: u8,
    /// Size of the descriptor in bytes, i.e. where the pre-header pad begins.
    pub rxd_len: usize,
    /// ★ Byte offset within the unit at which the 802.11 header starts:
    /// `rxd_len + 2 * remove_pad` (`mt7921/mac.c:389`).
    pub hdr_gap: usize,
    /// Bytes of A-MSDU pad sitting **between** the 802.11 header and the frame body: 2 when
    /// this is a non-translated A-MSDU subframe, else 0. `mt7921/mac.c:406-410` removes it
    /// by sliding the header forward two bytes; [`mpdu`] does the same by copy.
    pub body_pad: usize,
    /// Station index the frame matched.
    pub wcid: u16,
    /// Raw group bitmap, bits 11-15 of `rxd[1]`, kept for diagnostics: if the timestamp
    /// ever goes missing this is the field that says so.
    pub group_bits: u32,
    /// FCS failed. The frame is still delivered by this parser (it is real information
    /// about the channel); dropping it is the backend's call.
    pub fcs_err: bool,
    /// ICV / MIC error — `mt7921/mac.c:209-210` marks the frame monitor-only.
    pub icv_err: bool,
    /// TKIP Michael MIC error.
    pub tkip_mic_err: bool,
    /// Cipher suite the hardware applied; 0 on a keyless monitor.
    pub sec_mode: u8,
    /// Key index within that suite.
    pub key_id: u8,
    /// ★ The payload is 802.3, not 802.11 — see fact 1 in the module header.
    pub hdr_trans: bool,
    /// Raw [`MT_RXD2_NORMAL_MAC_HDR_LEN`]; units undetermined upstream, used by nothing.
    pub mac_hdr_len_raw: u8,
    /// QoS TID.
    pub tid: u8,
    /// This MPDU arrived inside an A-MPDU (the inverse of [`MT_RXD2_NORMAL_NON_AMPDU`]).
    pub ampdu: bool,
    /// A-MSDU subframe position.
    pub amsdu: AmsduPos,
    /// Hardware channel index ([`MT_RXD3_NORMAL_CH_FREQ`]) — not a frequency.
    pub ch_idx: u8,
    /// The frame was addressed to us as a unicast.
    pub unicast: bool,
    /// Frame-control, from group 4 if present. `None` when group 4 is absent — the copy in
    /// the MPDU itself is then the only source, and it is the caller who holds the bytes.
    pub frame_control: Option<u16>,
    /// Sequence-control, from group 4.
    pub seq_ctrl: Option<u16>,
    /// QoS-control, from group 4.
    pub qos_ctl: Option<u16>,
    /// ★ **The MAC receive timestamp** — see the long note on this field below.
    ///
    /// **Which group.** Group 2, dword 0 (`mt7921/mac.c:307-308`). Present only when
    /// [`MT_RXD1_NORMAL_GROUP_2`] is set in `rxd[1]`; `None` otherwise, never 0.
    ///
    /// **Width: 32 bits.** The descriptor carries the *low half* of a 64-bit TSF — the same
    /// counter `mt792x_get_tsf` reads as `MT_LPON_UTTR0` (low) + `MT_LPON_UTTR1` (high)
    /// (`mt792x_core.c:258-260`). At a 1 µs tick a 32-bit field wraps every 2³² µs =
    /// **4294.967296 s ≈ 71 min 35 s**. For common view that means: differences are valid
    /// only modulo 2³², a session must either re-read the LPON high word periodically or
    /// unwrap in software, and a pair of nodes whose stamps differ by more than ~35 minutes
    /// cannot be disambiguated from this field alone.
    ///
    /// **Tick rate, and how it was determined — CODE-READ, not measured.** Three links:
    /// (1) `mt7921/mac.c:309` sets `RX_FLAG_MACTIME_START`, and mac80211's contract for
    /// `mactime` is microseconds of the TSF at the first symbol of the PPDU; (2) the same
    /// counter is exposed to userspace as the vif TSF through `MT_LPON_UTTR0/1`; (3) 802.11
    /// mandates a 1 MHz TSF. So: **1 µs per tick, latched at PPDU start.** None of that has
    /// been checked on the target silicon. The measurement is one capture long — stamp a
    /// frame, immediately read `MT_LPON_UTTR0`, difference, repeat — and until it is done
    /// this is an inference, however well-supported.
    ///
    /// **What gates group 2 — undetermined, and that is the honest answer.** The *only*
    /// RXD-group gate anywhere in the upstream tree is `MT_DMA_DCR0_RXD_G5_EN` for group 5
    /// (`mt792x_mac.c:304`, `mt7915/main.c:507`). Nothing turns groups 1-4 on or off. The
    /// evidence points to a per-frame, content-dependent bitmap — group 1 tracks
    /// `SEC_MODE`, group 4 tracks header translation — with group 2 seemingly always on for
    /// a normal frame. That is a guess, so this parser treats the bit as authoritative on
    /// every frame and this field as optional. If a capture shows group 2 missing, the
    /// place to look is the firmware `CHIP_CONFIG` / RX-header-translation MCU commands,
    /// not a register in this file.
    ///
    /// **Relation to the host-readable TSF.** `MT_LPON_UTTR0/1` is the *same* counter, so a
    /// host read establishes the high word and the epoch; this field then supplies the
    /// per-frame precision the host read cannot (an EP0 round trip on this bus was MEASURED
    /// at 268 µs, i.e. ~268 ticks of uncertainty). Note the direction of the dependency:
    /// the register read is the coarse anchor, the descriptor is the measurement.
    pub timestamp: Option<u32>,
    /// The rate vector from group 3, or `None` when group 3 is absent.
    pub rate: Option<RxRate>,
    /// Per-chain RCPI, raw. Only chains 0-1 are physically present on this 2×2 part; 2 and
    /// 3 read 0, which [`rcpi_to_dbm`] maps to −110 dBm, so a caller must consult
    /// [`Rxd::CHAINS`] rather than trusting all four.
    pub rcpi: Option<[u8; 4]>,
    /// ★ Which of the two RCPI sources filled [`Rxd::rcpi`]: `true` = the C-RXV copy from
    /// group 5, which upstream says monitor mode should prefer (`mt7921/mac.c:356-358`);
    /// `false` = P-RXV DW1. They are different measurements of the same PPDU and a log that
    /// mixes them without saying which is which is a log that cannot be trusted.
    pub rcpi_from_crxv: bool,
    /// Byte offset of C-RXV dword 0 within the unit, when group 5 is present. Exposed so a
    /// caller can pull HE radiotap extras out of the vector without re-walking the
    /// descriptor — and so the alignment argument in [`crxv_snr_db`] can be checked against
    /// real bytes.
    pub crxv_offset: Option<usize>,
}

impl Rxd {
    /// Number of RX chains that physically exist on this part. The MT7921AU is 2×2.
    pub const CHAINS: usize = 2;

    /// Strongest per-chain signal in dBm, over the chains that exist, or `None` when there
    /// is no RCPI or every chain reported an implausible (≥ 0 dBm) level.
    ///
    /// Mirrors `mt7921/mac.c:371-379`, including its rule that a non-negative chain level
    /// means "no measurement" — an RCPI of 220 or more decodes to 0 dBm or better, which no
    /// receiver in this lab has ever genuinely seen.
    pub fn signal_dbm(&self) -> Option<i8> {
        let rcpi = self.rcpi?;
        let mut best: Option<i8> = None;
        for &r in rcpi.iter().take(Self::CHAINS) {
            let v = rcpi_to_dbm(r);
            if v >= 0 {
                continue;
            }
            best = Some(match best {
                Some(b) if b >= v => b,
                _ => v,
            });
        }
        best
    }

    /// Bytes of 802.11 MPDU in this unit: everything after [`Rxd::hdr_gap`], less the
    /// A-MSDU pad. The FCS is not included — the hardware strips it (see
    /// [`MT_RXD1_NORMAL_FCS_ERR`]).
    pub fn mpdu_len(&self) -> usize {
        self.unit_len
            .saturating_sub(self.hdr_gap)
            .saturating_sub(self.body_pad)
    }
}

/// Why an RXD could not be parsed.
///
/// A `Copy` enum rather than a [`FaceError`] because malformed descriptors arrive on the
/// per-frame path: formatting a `String` per bad frame is a cost the RX pump cannot pay, and
/// a backend wants to *bucket* these into counters anyway, which needs a discriminant, not a
/// message. An `impl From<RxdError> for FaceError` exists below for the callers that want
/// the error type the rest of the crate uses.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RxdError {
    /// Fewer bytes than the fixed 6-dword head.
    TooShort,
    /// [`MT_RXD0_LENGTH`] is zero, not 4-aligned, or longer than the buffer.
    BadLength,
    /// `MT_RXD1_NORMAL_BAND_IDX` set on a single-PHY part (`mt7921/mac.c:195-196`).
    BandIdx,
    /// `MT_RXD2_NORMAL_AMSDU_ERR` (`:201-202`).
    AmsduErr,
    /// Header translation plus a cipher mismatch (`:205-206`) — the payload is neither a
    /// usable 802.3 frame nor a recoverable 802.11 one.
    HdrTransCipherMismatch,
    /// `MT_RXD2_NORMAL_MAX_LEN_ERROR` (`:259-260`).
    MaxLenError,
    /// A group's advance ran past the end of the unit. Carries the group number so a
    /// counter can say *which* advance — the difference between "the descriptor is
    /// truncated" and "our size for group N is wrong".
    GroupTruncated(u8),
    /// `MT_PRXV_TX_MODE` held a value this part cannot produce
    /// (`mt76_connac_mac.c:1112-1113`).
    BadRateMode(u8),
    /// `MT_PRXV_FRAME_MODE` held a reserved bandwidth (`:1136-1137`).
    BadBandwidth(u8),
    /// The HT/VHT MCS index exceeded its ceiling — 31 for HT, 11 for VHT
    /// (`:1088-1089`, `:1096-1097`).
    McsOutOfRange(u8),
    /// The descriptor plus its alignment pad consumed the whole unit, leaving no 802.11
    /// frame behind it. Upstream's per-group bound check (`>=`, not `>`) has the same
    /// effect one group earlier; this catches the case where only `remove_pad` pushes the
    /// cursor over the end.
    NoPayload,
}

impl std::fmt::Display for RxdError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RxdError::TooShort => write!(f, "RXD shorter than the 24-byte fixed head"),
            RxdError::BadLength => write!(f, "RXD length field is zero, unaligned or oversized"),
            RxdError::BandIdx => write!(f, "RXD BAND_IDX set on a single-PHY part"),
            RxdError::AmsduErr => write!(f, "RXD reports an A-MSDU parse error"),
            RxdError::HdrTransCipherMismatch => {
                write!(f, "RXD reports header translation with a cipher mismatch")
            }
            RxdError::MaxLenError => write!(f, "RXD reports a max-length error"),
            RxdError::GroupTruncated(g) => write!(f, "RXD group {g} runs past the end of the unit"),
            RxdError::BadRateMode(v) => write!(f, "P-RXV TX_MODE {v} is not an 11ax PHY mode"),
            RxdError::BadBandwidth(v) => write!(f, "P-RXV FRAME_MODE {v} is a reserved bandwidth"),
            RxdError::McsOutOfRange(v) => write!(f, "P-RXV MCS index {v} is out of range"),
            RxdError::NoPayload => write!(f, "RXD leaves no 802.11 frame in the unit"),
        }
    }
}

impl From<RxdError> for FaceError {
    fn from(e: RxdError) -> Self {
        FaceError::Io(std::io::Error::other(format!("connac2 RXD: {e}")))
    }
}

// ── The parser ──────────────────────────────────────────────────────────────

/// Read a little-endian dword at dword index `i`, or `None` past the end.
fn dw(buf: &[u8], i: usize) -> Option<u32> {
    let o = i * 4;
    buf.get(o..o + 4)
        .map(|s| u32::from_le_bytes([s[0], s[1], s[2], s[3]]))
}

/// Parse the connac2 RX descriptor at the start of `buf`.
///
/// `buf` is a bulk-IN transfer (or the remainder of one). Byte 0 is `rxd[0]`: `mt7921u`
/// declares `MT_DRV_RX_DMA_HDR` (`mt7921/usb.c:153`), so unlike the mt76x0/mt76x2 parts
/// there is no 4-byte DMA header in front of the descriptor — `mt76u_get_rx_entry_len`
/// (`usb.c:466-473`) reads the entry length straight out of `rxd[0]`'s low half and hands
/// the descriptor itself to the driver.
///
/// This is a faithful port of `mt7921_mac_fill_rx` (`mt7921/mac.c:168-445`) with three
/// deliberate differences, each because this driver is a promiscuous monitor and not a
/// mac80211 station:
///
///   1. **FCS errors are not dropped.** Upstream flags them and lets mac80211 discard;
///      here they are reported ([`Rxd::fcs_err`]) so the backend can count them as channel
///      information. Everything upstream returns `-EINVAL` for *is* rejected.
///   2. **No station lookup.** There is no WCID table in this driver, so the `msta` /
///      `mt76_wcid` plumbing (`:214-221`) has no analogue; [`Rxd::wcid`] is the raw index.
///   3. **The rate vector is decoded even when group 5 is absent.** Upstream does the same
///      for connac2 (`mt76_connac_mac.c:1055-1063` reads everything from P-RXV DW0); only
///      the mt7915 branch needs the C-RXV.
///
/// ★ Upstream over-reads in one place and this port does not follow it:
/// `mt76_connac2_mac_fill_rx_rate` unconditionally loads `v2 = le32_to_cpu(rxv[2])`
/// (`mt76_connac_mac.c:1050`), two dwords past the end of a 2-dword P-RXV, and then uses
/// `v2` only in the `is_mt7915` branch. On connac2 that dword is either the first dword of
/// the C-RXV or the start of the 802.11 frame. Harmless upstream (the value is discarded)
/// but it is a read past a structure, so it is not reproduced.
pub fn parse_rxd(buf: &[u8]) -> Result<Rxd, RxdError> {
    if buf.len() < RXD_FIXED_DWORDS * 4 {
        return Err(RxdError::TooShort);
    }
    let rxd0 = dw(buf, 0).ok_or(RxdError::TooShort)?;
    let rxd1 = dw(buf, 1).ok_or(RxdError::TooShort)?;
    let rxd2 = dw(buf, 2).ok_or(RxdError::TooShort)?;
    let rxd3 = dw(buf, 3).ok_or(RxdError::TooShort)?;
    let rxd4 = dw(buf, 4).ok_or(RxdError::TooShort)?;

    // Length validation. Upstream does none at all on this bus: `mt76u_get_rx_entry_len`
    // (`usb.c:466-480`) returns `dma_len` unchecked as soon as `MT_DRV_RX_DMA_HDR` is set
    // (`:472-473`), and only the *other* branch — the mt76x02 one — checks the value. That
    // is fine for a kernel that trusts its own URB accounting and is not fine for us, so the
    // two checks that still hold on this chip are made: the unit must contain at least the
    // fixed head, and it must fit in what we were handed. Everything below bounds against
    // this number, so it is the one field that must not be taken on trust.
    //
    // ⚠ The mt76x02 branch's **4-alignment** check (`usb.c:478`: `dma_len & 0x3`) is
    // deliberately *not* applied here, and this is not an oversight. It is not a connac2
    // rule: [`MT_RXD0_LENGTH`] counts the descriptor plus the MPDU, and an MPDU is any
    // number of bytes. Alignment on this part is arranged at the *front* instead — that is
    // what [`MT_RXD2_NORMAL_HDR_OFFSET`] is for, padding the header up to a 4-boundary.
    // Requiring a 4-aligned total rejects every odd-length frame, and the ones it rejects
    // are real ones.
    let unit_len = field_get(MT_RXD0_LENGTH, rxd0) as usize;
    if unit_len < RXD_FIXED_DWORDS * 4 || unit_len > buf.len() {
        return Err(RxdError::BadLength);
    }
    // From here on, bound every advance against the *unit*, not the transfer: a bulk-IN may
    // carry padding or a following unit, and walking into it would be a silent corruption.
    let unit = &buf[..unit_len];

    if rxd1 & MT_RXD1_NORMAL_BAND_IDX != 0 {
        return Err(RxdError::BandIdx);
    }
    if rxd2 & MT_RXD2_NORMAL_AMSDU_ERR != 0 {
        return Err(RxdError::AmsduErr);
    }
    let hdr_trans = rxd2 & MT_RXD2_NORMAL_HDR_TRANS != 0;
    if hdr_trans && rxd1 & MT_RXD1_NORMAL_CM != 0 {
        return Err(RxdError::HdrTransCipherMismatch);
    }
    if rxd2 & MT_RXD2_NORMAL_MAX_LEN_ERROR != 0 {
        return Err(RxdError::MaxLenError);
    }

    let remove_pad = field_get(MT_RXD2_NORMAL_HDR_OFFSET, rxd2) as usize;

    // ── the group walk ──────────────────────────────────────────────────────
    // Order is 4, 1, 2, 3, 5 — `mt7921/mac.c:263-364` uses one forward-only cursor, so the
    // order it reads them in is the order they lie in the descriptor.
    let mut cur = RXD_FIXED_DWORDS;

    // Upstream's bound check after every advance is `(u8 *)rxd - skb->data >= skb->len`
    // (e.g. `:272-273`) — note `>=`, not `>`: a descriptor that exactly fills the unit is
    // rejected, because then there is no frame behind it.
    let bound = |cur: usize, group: u8| -> Result<(), RxdError> {
        if cur * 4 >= unit_len {
            Err(RxdError::GroupTruncated(group))
        } else {
            Ok(())
        }
    };

    // Group 4 — frame-control / TA / seq-ctl / QoS-ctl (`:263-274`).
    let (mut frame_control, mut seq_ctrl, mut qos_ctl) = (None, None, None);
    if rxd1 & MT_RXD1_NORMAL_GROUP_4 != 0 {
        let v0 = dw(unit, cur).ok_or(RxdError::GroupTruncated(4))?;
        let v2 = dw(unit, cur + 2).ok_or(RxdError::GroupTruncated(4))?;
        frame_control = Some(field_get(MT_RXD6_FRAME_CONTROL, v0) as u16);
        seq_ctrl = Some(field_get(MT_RXD8_SEQ_CTRL, v2) as u16);
        qos_ctl = Some(field_get(MT_RXD8_QOS_CTL, v2) as u16);
        cur += RXD_GROUP4_DWORDS;
        bound(cur, 4)?;
    }

    // Group 1 — IV/EIV (`:276-305`). Walked, not decoded: see the module header.
    if rxd1 & MT_RXD1_NORMAL_GROUP_1 != 0 {
        cur += RXD_GROUP1_DWORDS;
        bound(cur, 1)?;
    }

    // ★ Group 2 — the MAC timestamp (`:307-327`).
    let mut timestamp = None;
    if rxd1 & MT_RXD1_NORMAL_GROUP_2 != 0 {
        timestamp = Some(dw(unit, cur).ok_or(RxdError::GroupTruncated(2))?);
        // Dword 1 of this group is read by nothing upstream. It is *not* assumed to be the
        // high half of the TSF: that is exactly the kind of guess this codebase spends its
        // time removing. If a capture ever shows it advancing once per 71 minutes, the
        // wrap problem in `Rxd::timestamp` disappears — until then it is two anonymous
        // bytes-worth of dword and stays undecoded.
        cur += RXD_GROUP2_DWORDS;
        bound(cur, 2)?;
    }

    // Group 3 — P-RXV, and group 5 nested inside it (`:329-380`).
    let mut rate = None;
    let mut rcpi = None;
    let mut rcpi_from_crxv = false;
    let mut crxv_offset = None;
    if rxd1 & MT_RXD1_NORMAL_GROUP_3 != 0 {
        let v0 = dw(unit, cur).ok_or(RxdError::GroupTruncated(3))?;
        let v1 = dw(unit, cur + 1).ok_or(RxdError::GroupTruncated(3))?;
        cur += RXD_GROUP3_DWORDS;
        bound(cur, 3)?;

        rate = Some(decode_prxv(v0)?);
        rcpi = Some(unpack_rcpi(v1));

        // ★ Group 5 is handled *inside* group 3, exactly as upstream does. If a descriptor
        // ever set GROUP_5 without GROUP_3 the cursor would not advance and every offset
        // after it would be wrong — upstream has the identical hole (`:350`), and the
        // hardware presumably never emits a C-RXV without the P-RXV that precedes it.
        // Ported faithfully rather than "fixed", because inventing an advance for a
        // combination that has never been seen is a guess with the same failure mode.
        if rxd1 & MT_RXD1_NORMAL_GROUP_5 != 0 {
            crxv_offset = Some(cur * 4);
            let rcpi_dw = dw(unit, cur + CRXV_RCPI_DWORD).ok_or(RxdError::GroupTruncated(5))?;
            rcpi = Some(unpack_rcpi(rcpi_dw));
            rcpi_from_crxv = true;
            cur += RXD_GROUP5_DWORDS;
            bound(cur, 5)?;
        }
    }

    let rxd_len = cur * 4;
    let hdr_gap = rxd_len + 2 * remove_pad;
    if hdr_gap >= unit_len {
        return Err(RxdError::NoPayload);
    }

    let amsdu = match field_get(MT_RXD4_NORMAL_PAYLOAD_FORMAT, rxd4) {
        0 => AmsduPos::None,
        v if v == MT_RXD4_FIRST_AMSDU_FRAME => AmsduPos::First,
        v if v == MT_RXD4_LAST_AMSDU_FRAME => AmsduPos::Last,
        _ => AmsduPos::Mid,
    };
    // `mt7921/mac.c:406-410`: a non-translated A-MSDU subframe carries two pad bytes between
    // the 802.11 header and the body, which upstream removes by sliding the header forward.
    let body_pad = if !hdr_trans && amsdu != AmsduPos::None {
        2
    } else {
        0
    };

    Ok(Rxd {
        unit_len,
        pkt_type: field_get(MT_RXD0_PKT_TYPE, rxd0) as u8,
        rxd_len,
        hdr_gap,
        body_pad,
        wcid: field_get(MT_RXD1_NORMAL_WLAN_IDX, rxd1) as u16,
        group_bits: rxd1 & 0x0000_f800,
        fcs_err: rxd1 & MT_RXD1_NORMAL_FCS_ERR != 0,
        icv_err: rxd1 & MT_RXD1_NORMAL_ICV_ERR != 0,
        tkip_mic_err: rxd1 & MT_RXD1_NORMAL_TKIP_MIC_ERR != 0,
        sec_mode: field_get(MT_RXD1_NORMAL_SEC_MODE, rxd1) as u8,
        key_id: field_get(MT_RXD1_NORMAL_KEY_ID, rxd1) as u8,
        hdr_trans,
        mac_hdr_len_raw: field_get(MT_RXD2_NORMAL_MAC_HDR_LEN, rxd2) as u8,
        tid: field_get(MT_RXD2_NORMAL_TID, rxd2) as u8,
        ampdu: rxd2 & MT_RXD2_NORMAL_NON_AMPDU == 0,
        amsdu,
        ch_idx: field_get(MT_RXD3_NORMAL_CH_FREQ, rxd3) as u8,
        unicast: field_get(MT_RXD3_NORMAL_ADDR_TYPE, rxd3) == MT_RXD3_NORMAL_U2M,
        frame_control,
        seq_ctrl,
        qos_ctl,
        timestamp,
        rate,
        rcpi,
        rcpi_from_crxv,
        crxv_offset,
    })
}

/// Split a P-RXV DW0 into a [`RxRate`] — the connac2 branch of
/// `mt76_connac2_mac_fill_rx_rate` (`mt76_connac_mac.c:1039-1145`).
fn decode_prxv(v0: u32) -> Result<RxRate, RxdError> {
    let raw_idx = field_get(MT_PRXV_TX_RATE, v0) as u8;
    let nsts = field_get(MT_PRXV_NSTS, v0) as u8;
    let stbc = field_get(MT_PRXV_HT_STBC, v0) as u8;
    let gi = field_get(MT_PRXV_HT_SGI, v0) as u8;
    let mode_code = field_get(MT_PRXV_TX_MODE, v0) as u8;
    let bw_code = field_get(MT_PRXV_FRAME_MODE, v0) as u8;
    let ldpc = v0 & MT_PRXV_HT_AD_CODE != 0;
    let txbf = v0 & MT_PRXV_TXBF != 0;
    // `:1059-1060` — on connac2 the DCM bit lives *inside* the rate index, not at
    // `MT_PRXV_DCM`.
    let dcm = u32::from(raw_idx) & MT_PRXV_TX_DCM != 0;
    let er_su_106t = u32::from(raw_idx) & MT_PRXV_TX_ER_SU_106T != 0;

    let mode = RateMode::from_code(mode_code).ok_or(RxdError::BadRateMode(mode_code))?;

    // `:1072-1074` — the hardware reports space-time streams; halve for STBC to get the
    // data (spatial) stream count.
    let mut nss = nsts + 1;
    if stbc != 0 && nss > 1 {
        nss >>= 1;
    }

    let (mcs, legacy_code, he_gi) = match mode {
        RateMode::Cck | RateMode::Ofdm => (None, Some(raw_idx), None),
        RateMode::Ht | RateMode::HtGf => {
            if raw_idx > 31 {
                return Err(RxdError::McsOutOfRange(raw_idx));
            }
            (Some(raw_idx), None, None)
        }
        RateMode::Vht => {
            if raw_idx > 11 {
                return Err(RxdError::McsOutOfRange(raw_idx));
            }
            (Some(raw_idx), None, None)
        }
        // `:1105` masks the HE MCS to 4 bits, because bits 4-5 of the same field are DCM
        // and the 106-tone marker.
        _ => {
            let he_gi = if gi <= 3 { Some(gi) } else { None };
            (Some(raw_idx & 0x0f), None, he_gi)
        }
    };

    // `:1117-1138`. The ER-SU / 106-tone special case is why this is not a plain match on
    // `bw_code`: the hardware says "40 MHz" for a PPDU that is in fact narrower than 20.
    let bw = match bw_code {
        0 => RxBw::Bw20,
        1 => {
            if mode == RateMode::HeExtSu && er_su_106t {
                RxBw::HeRu106
            } else {
                RxBw::Bw40
            }
        }
        2 => RxBw::Bw80,
        3 => RxBw::Bw160,
        v => return Err(RxdError::BadBandwidth(v)),
    };

    Ok(RxRate {
        mode,
        raw_idx,
        mcs,
        legacy_code,
        nss,
        bw,
        // `:1141-1142` — the short-GI flag is meaningful only below HE, where the same
        // two bits are a 4-valued GI code instead.
        short_gi: !mode.is_he() && gi != 0,
        he_gi,
        dcm,
        stbc,
        ldpc,
        txbf,
    })
}

/// Unpack the four packed RCPI bytes of a P-RXV DW1 or C-RXV RCPI dword.
const fn unpack_rcpi(v: u32) -> [u8; 4] {
    [
        field_get(MT_PRXV_RCPI0, v) as u8,
        field_get(MT_PRXV_RCPI1, v) as u8,
        field_get(MT_PRXV_RCPI2, v) as u8,
        field_get(MT_PRXV_RCPI3, v) as u8,
    ]
}

/// RCPI byte → dBm, i.e. `to_rssi` (`mt7921/mt7921.h:109`: `(FIELD_GET(field, rxv) - 220) / 2`).
///
/// RCPI is the 802.11 half-dBm scale anchored at −110 dBm, so this is `rcpi/2 - 110` and
/// upstream's spelling is the same line rearranged.
///
/// ⚠ The division must **floor**, not truncate toward zero. Upstream does the subtraction in
/// `u32`, so `rcpi < 220` wraps and the `/2` becomes a floor before the result is narrowed
/// to `s8` — accidental, but it is the behaviour the numbers were validated against. Plain
/// Rust `/` truncates toward zero and would return 0 dBm for `rcpi = 219` where upstream
/// returns −1. Hence `div_euclid`.
pub const fn rcpi_to_dbm(rcpi: u8) -> i8 {
    ((rcpi as i16 - 220).div_euclid(2)) as i8
}

/// SNR in dB from a **C-RXV that is at least 21 dwords long**, or `None`.
///
/// ★ **This returns `None` for every normal RX descriptor, by construction, and that is the
/// finding — not a stub.** `MT_CRXV_SNR` lives in C-RXV dword 20 (`mt7915/mac.c:585-586`
/// reads `v20 = rxv[20]`), but the C-RXV embedded in an RXD as group 5 is **18 dwords**
/// (`mt7915/mac.c:448`; `mt7921/mac.c:351,361` = 6 + 12). Dword 20 is past the end. The
/// only packet on this chip that carries a longer vector is the standalone
/// [`PKT_TYPE_TXRXV`] report, which upstream builds only under `CONFIG_NL80211_TESTMODE`.
///
/// So: **per-frame SNR and CFO are not available from a normal RXD on connac2.** Manufacturing
/// one out of an in-range dword would be the exact defect this codebase exists to remove, so
/// [`Rxd`] has no `snr` field at all and this helper is here for the day the TXRXV path is
/// implemented. The C-RXV base convention it assumes (dword 0 = the first dword after the
/// P-RXV) is the one `mt76_connac2_mac_fill_rx_rate` uses in its mt7915 branch
/// (`mt76_connac_mac.c:1065-1069` reads the `MT_CRXV_*` fields from `rxv[2]`, and the P-RXV
/// is 2 dwords).
pub fn crxv_snr_db(crxv: &[u8]) -> Option<i8> {
    let v20 = dw(crxv, 20)?;
    // `mt7915/mac.c:586`: the field is biased by 16.
    Some((field_get(MT_CRXV_SNR, v20) as i16 - 16) as i8)
}

/// Frequency offset estimate from a C-RXV of at least 22 dwords, in the hardware's own
/// units (`mt7915/mac.c:583-584` recombines a 13-bit low half with a 7-bit high half; it
/// names no unit and neither can this port). `None` for the same reason as [`crxv_snr_db`].
pub fn crxv_freq_offset(crxv: &[u8]) -> Option<i32> {
    let v20 = dw(crxv, 20)?;
    let v21 = dw(crxv, 21)?;
    let raw =
        field_get(MT_CRXV_FOE_LO, v20) | (field_get(MT_CRXV_FOE_HI, v21) << MT_CRXV_FOE_SHIFT);
    // 20 bits, two's complement — upstream assigns it to an `s32` without sign-extending,
    // which is a bug it never notices because the testmode print is only ever eyeballed.
    // Sign-extended here; flagged rather than silently "corrected".
    let signed = ((raw << 12) as i32) >> 12;
    Some(signed)
}

/// The bare 802.11 MPDU of this unit as a slice, when no A-MSDU pad has to be removed.
///
/// The common case by a wide margin — a non-aggregated frame — and it costs no copy.
/// `None` when a pad is present (use [`mpdu`]) or the unit is truncated.
pub fn mpdu_slice<'a>(unit: &'a [u8], d: &Rxd) -> Option<&'a [u8]> {
    if d.body_pad != 0 {
        return None;
    }
    unit.get(d.hdr_gap..d.unit_len)
}

/// The bare 802.11 MPDU of this unit, with the A-MSDU pad removed if there is one.
///
/// The pad sits *between* the header and the body, so removing it means moving the header,
/// not trimming an end: `mt7921/mac.c:407-409` does `memmove(data + 2, data, hdrlen)` and
/// then pulls 2. Same operation, done by copy because we do not own the transfer buffer.
pub fn mpdu(unit: &[u8], d: &Rxd) -> Option<Vec<u8>> {
    if d.body_pad == 0 {
        return mpdu_slice(unit, d).map(<[u8]>::to_vec);
    }
    let start = d.hdr_gap;
    let fc = unit.get(start..start + 2)?;
    let hdr_len = dot11_hdr_len(fc[0], fc[1]);
    let hdr = unit.get(start..start + hdr_len)?;
    let body_start = start + hdr_len + d.body_pad;
    let body = unit.get(body_start..d.unit_len)?;
    let mut out = Vec::with_capacity(hdr.len() + body.len());
    out.extend_from_slice(hdr);
    out.extend_from_slice(body);
    Some(out)
}

/// 802.11 header length from the frame-control octets — the shape of `ieee80211_hdrlen`.
///
/// Needed in two places where a wrong answer *misaligns* rather than merely mislabels: the
/// A-MSDU pad in [`mpdu`], and `MT_TXD1_HDR_INFO` in [`build_txd`] (which upstream fills
/// from `ieee80211_get_hdrlen_from_skb`, `mt76_connac_mac.c:440-441`). Base 24, plus
/// `addr4` when ToDS and FromDS are both set, plus QoS Control on a QoS-Data subtype, plus
/// HT Control on the Order/+HTC bit.
///
/// A near-duplicate of the private helper in `crate::mt76x0`; not shared, because that one
/// is a module-private item of a different silicon family's backend and reaching across for
/// it would couple two ports that have nothing else in common.
pub const fn dot11_hdr_len(fc0: u8, fc1: u8) -> usize {
    let ftype = (fc0 >> 2) & 0x3;
    let subtype = fc0 >> 4;
    match ftype {
        // Control: only the three long-header subtypes matter for alignment.
        1 => match subtype {
            0xc | 0xd => 10, // CTS, ACK
            _ => 16,         // PS-Poll, RTS, CF-End, BlockAck(Req)
        },
        // Data.
        2 => {
            let mut n = 24;
            if fc1 & 0x03 == 0x03 {
                n += 6; // addr4
            }
            if subtype & 0x08 != 0 {
                n += 2; // QoS Control
                if fc1 & 0x80 != 0 {
                    n += 4; // HT Control (+HTC)
                }
            }
            n
        }
        // Management (and the 802.11 "extension" type, which we do not decode).
        _ => {
            if fc1 & 0x80 != 0 {
                28
            } else {
                24
            }
        }
    }
}

// ════════════════════════════════════════════════════════════════════════════
// TX — the 8-dword TXD and its rate word
// ════════════════════════════════════════════════════════════════════════════

// ── TXD DW0-DW8 (mt76_connac2_mac.h:50-131) ─────────────────────────────────

/// Which LMAC queue the firmware should put this frame on — see [`MT_LMAC_AC01`].
pub const MT_TXD0_Q_IDX: u32 = 0xfe00_0000;
/// `enum tx_pkt_type` (`mt76_connac2_mac.h:14-19`).
pub const MT_TXD0_PKT_FMT: u32 = 0x0180_0000;
/// Total bytes the hardware should read: payload **plus the descriptor**
/// (`mt76_connac_mac.c:552`: `skb->len + sz_txd`, where `sz_txd` is
/// [`TXD_LEN`] on a USB part).
pub const MT_TXD0_TX_BYTES: u32 = 0x0000_ffff;

/// Long (8-dword) descriptor format. Always set by `mt76_connac2_mac_write_txwi`
/// (`:557`).
pub const MT_TXD1_LONG_FORMAT: u32 = 1 << 31;
/// Which of the hardware's own MAC addresses to transmit from.
pub const MT_TXD1_OWN_MAC: u32 = 0x3f00_0000;
/// QoS TID.
pub const MT_TXD1_TID: u32 = 0x0070_0000;
/// `enum tx_header_format` (`mt76_connac2_mac.h:7-12`) — see [`MT_HDR_FORMAT_802_11`].
pub const MT_TXD1_HDR_FORMAT: u32 = 0x0003_0000;
/// 802.11 header length **in units of 2 bytes** (`mt76_connac_mac.c:440-441`:
/// `ieee80211_get_hdrlen_from_skb(skb) / 2`).
pub const MT_TXD1_HDR_INFO: u32 = 0x0000_f800;
/// Vendor-tag insertion. `mt76_connac_mac.c:560-561` sets it only when **not** connac2, so
/// it stays clear on this chip (MEASURED chip id 0x7961).
pub const MT_TXD1_VTA: u32 = 1 << 10;
/// Station index. `0x3ff` is the "no station" index used for a keyless broadcast.
pub const MT_TXD1_WLAN_IDX: u32 = 0x0000_03ff;

/// ★ Use the rate in [`MT_TXD6_TX_RATE`] instead of letting the rate controller pick.
/// `mt76_connac_mac.c:459-461` sets it for any non-data, multicast, or min-rate frame — so
/// every broadcast this driver sends takes the fixed-rate path.
pub const MT_TXD2_FIX_RATE: u32 = 1 << 31;
/// A second fixed-rate bit that **no upstream code path ever sets**
/// (`mt76_connac2_mac.h:68` is its only occurrence in the tree). Recorded so a reader does
/// not mistake it for the one that works.
pub const MT_TXD2_FIXED_RATE: u32 = 1 << 30;
/// Per-frame TX power offset.
pub const MT_TXD2_POWER_OFFSET: u32 = 0x3f00_0000;
/// Cap on the airtime this frame may occupy.
pub const MT_TXD2_MAX_TX_TIME: u32 = 0x00ff_0000;
/// "HTC valid" — `mt76_connac_mac.c:604-605` sets it alongside every fixed rate, with the
/// comment *"hardware won't add HTC for mgmt/ctrl frame"*. The reason the two travel
/// together is not stated upstream and is not inferred here; it is ported because it is
/// what the working driver does.
pub const MT_TXD2_HTC_VLD: u32 = 1 << 13;
/// Address 1 is a group address (`mt76_connac_mac.c:451`).
pub const MT_TXD2_MULTICAST: u32 = 1 << 10;
/// Protect the frame with RTS/CTS.
pub const MT_TXD2_RTS: u32 = 1 << 9;
/// 802.11 frame type, bits 3:2 of frame-control (`mt76_connac_mac.c:446`).
pub const MT_TXD2_FRAME_TYPE: u32 = 0x0000_0030;
/// 802.11 frame subtype, bits 7:4 of frame-control.
pub const MT_TXD2_SUB_TYPE: u32 = 0x0000_000f;

/// The sequence number in [`MT_TXD3_SEQ`] is ours, not the hardware's. mac80211's injected
/// path sets it (`mt76_connac_mac.c:487-489`); the mt7915 testmode explicitly *clears* it
/// (`mt7915/mac.c:702`). Both are attested, so [`TxdConfig::seq`] makes it a choice.
pub const MT_TXD3_SN_VALID: u32 = 1 << 31;
/// Do not aggregate this frame into a Block-Ack session. Set with every fixed rate
/// (`mt76_connac_mac.c:609`).
pub const MT_TXD3_BA_DISABLE: u32 = 1 << 28;
/// The sequence number, when [`MT_TXD3_SN_VALID`] is set.
pub const MT_TXD3_SEQ: u32 = 0x0fff_0000;
/// Remaining transmit attempts. `mt76_connac2_mac_write_txwi` uses 15
/// (`mt76_connac_mac.c:568`); the mt7915 testmode uses 1 (`mt7915/mac.c:678`). For a
/// no-ACK broadcast the value is inert — there is nothing to retry on.
pub const MT_TXD3_REM_TX_COUNT: u32 = 0x0000_f800;
/// Transmit-attempt counter.
pub const MT_TXD3_TX_COUNT: u32 = 0x0000_07c0;
/// Software power management. Not set on connac2 (`mt76_connac_mac.c:569-570`).
pub const MT_TXD3_SW_POWER_MGMT: u32 = 1 << 29;
/// Encrypt with the WCID's key.
pub const MT_TXD3_PROTECT_FRAME: u32 = 1 << 1;
/// ★ Do not expect an ACK. mac80211 sets `IEEE80211_TX_CTL_NO_ACK` for group-addressed
/// frames, which `mt76_connac_mac.c:573-574` copies here.
pub const MT_TXD3_NO_ACK: u32 = 1 << 0;

/// Report the transmit status to the host (`mt76_connac_mac.c:580-581`, set whenever the
/// packet id is a real one).
pub const MT_TXD5_TX_STATUS_HOST: u32 = 1 << 10;
/// Report the transmit status to the MCU instead.
pub const MT_TXD5_TX_STATUS_MCU: u32 = 1 << 9;
/// Packet id, echoed back in the TXS report so a status can be matched to a frame.
pub const MT_TXD5_PID: u32 = 0x0000_00ff;

/// ★ The 14-bit fixed-rate word — see [`encode_rate`].
pub const MT_TXD6_TX_RATE: u32 = 0x3fff_0000;
/// Guard-interval / HE-GI code, 2 bits (`mt7915/mac.c:684` passes the testmode value
/// straight through).
pub const MT_TXD6_SGI: u32 = 0x0000_c000;
/// HE LTF size, 2 bits. Upstream sets it only for `mode >= MT_PHY_TYPE_HE_SU`
/// (`mt7915/mac.c:696-697`) and documents the legal GI/LTF pairs only in the comment at
/// `mt7915/mac.c:686-695`; there is no enum to port, so [`FixedRate::he_ltf`] is raw.
pub const MT_TXD6_HELTF: u32 = 0x0000_3000;
/// LDPC instead of BCC. Note `mt7915/mac.c:697` forces it on for any HE PPDU wider than
/// 20 MHz, whatever the caller asked for.
pub const MT_TXD6_LDPC: u32 = 1 << 11;
/// Antenna selection. **Never written by any connac2 path upstream** — the
/// non-connac2 branch uses `MT_TXD7_SPE_IDX` instead (`mt76_connac_mac.c:611-617`). Exposed
/// through [`FixedRate::ant_id`] and defaulted to 0; that it does anything on this chip is
/// unproven.
pub const MT_TXD6_ANT_ID: u32 = 0x0000_00f0;
/// Let the hardware narrow the bandwidth dynamically.
pub const MT_TXD6_DYN_BW: u32 = 1 << 3;
/// Transmit at exactly [`MT_TXD6_BW`]. Set with every fixed rate
/// (`mt76_connac_mac.c:602`).
pub const MT_TXD6_FIXED_BW: u32 = 1 << 2;
/// Bandwidth: 0/1/2/3 = 20/40/80/160 MHz (`mt7915/mac.c:650-664`).
pub const MT_TXD6_BW: u32 = 0x0000_0003;

/// Hardware A-MSDU aggregation. Cleared on every fixed-rate frame
/// (`mt76_connac_mac.c:619`).
pub const MT_TXD7_HW_AMSDU: u32 = 1 << 10;

/// Frame type, repeated in DW8 for USB/SDIO. `mt76_connac2_mac_write_txwi_80211` puts it in
/// DW7 for MMIO parts and DW8 for everything else (`mt76_connac_mac.c:493-501`) — the one
/// place in the descriptor where the bus changes the layout, and therefore the one place a
/// PCIe-derived port silently writes the wrong dword.
pub const MT_TXD8_L_TYPE: u32 = 0x0000_0030;
/// Frame subtype, DW8. See [`MT_TXD8_L_TYPE`].
pub const MT_TXD8_L_SUB_TYPE: u32 = 0x0000_000f;

// ── TX rate-word fields (mt76_connac2_mac.h:133-139) ────────────────────────

/// Space-time block coding.
pub const MT_TX_RATE_STBC: u32 = 1 << 13;
/// ★ Spatial streams **minus one**. `mt7915/mac.c:674` writes `nss - 1` and the TXS decoder
/// reads `FIELD_GET(...) + 1` (`mt76_connac_mac.c:662`), so the field is 0-based — see the
/// warning in [`encode_rate`] about upstream's own inconsistency here.
pub const MT_TX_RATE_NSS: u32 = 0x0000_1c00;
/// PHY mode — the same `enum mt76_phy_type` values [`RateMode`] carries.
pub const MT_TX_RATE_MODE: u32 = 0x0000_03c0;
/// HE ER-SU 106-tone variant. The RX side's [`MT_PRXV_TX_ER_SU_106T`], same bit.
pub const MT_TX_RATE_SU_EXT_TONE: u32 = 1 << 5;
/// HE dual-carrier modulation. The RX side's [`MT_PRXV_TX_DCM`], same bit.
pub const MT_TX_RATE_DCM: u32 = 1 << 4;
/// Rate index. 6 bits, but *"VHT/HE only use bits 0-3"* (`mt76_connac2_mac.h:138`) because
/// bits 4-5 are DCM and the 106-tone marker for those modes.
pub const MT_TX_RATE_IDX: u32 = 0x0000_003f;

// ── enum values ─────────────────────────────────────────────────────────────

/// `MT_HDR_FORMAT_802_11` (`mt76_connac2_mac.h:10`) — the payload is a bare 802.11 frame,
/// which is the only format this driver ever sends.
pub const MT_HDR_FORMAT_802_11: u32 = 2;
/// `MT_TX_TYPE_SF` (`mt76_connac2_mac.h:16`) — "short format". `mt76_connac_mac.c:543`
/// picks it for every non-MMIO data frame, i.e. for us.
pub const MT_TX_TYPE_SF: u32 = 1;
/// `MT_TX_TYPE_CT` — the MMIO counterpart of [`MT_TX_TYPE_SF`]; recorded so the bus-dependent
/// choice is visible.
pub const MT_TX_TYPE_CT: u32 = 0;
/// `MT_LMAC_AC01` (`mt76_connac2_mac.h:26`) — the LMAC queue a **best-effort** frame goes
/// to.
///
/// The derivation, because it is not obvious: `mt76_connac_mac.c:544-545` computes
/// `q_idx = wmm_idx * 4 + mt76_connac_lmac_mapping(ac)`, and `mt76_connac_lmac_mapping` is
/// `3 - ac` (`mt76_connac.h:399-403`, *"LMAC uses the reverse order of mac80211 AC
/// indexes"*). mac80211's BE is AC 2, so `3 - 2 = 1`. Note this is **independent** of the
/// bulk-OUT endpoint: `mt76u_ac_to_hwq`'s `case 0x7961` (`usb.c:967-971`) gives BE
/// `hw_idx = 0` and therefore `ep = 1` = `MT_EP_OUT_AC_BE` = `0x05`. Queue index 1, endpoint
/// 5, same frame.
pub const MT_LMAC_AC01: u32 = 1;
/// `MT_LMAC_ALTX0` (`mt76_connac2_mac.h:29`) — the "alternative TX" queue, used for PSD and
/// in-band discovery frames.
pub const MT_LMAC_ALTX0: u32 = 0x10;

/// `MT_PACKET_ID_FIRST` (`mt76_connac.h:498`): packet ids below this are reserved
/// (0 = no-ACK, 1 = no-skb, 2 = WED), and only an id at or above it turns on
/// [`MT_TXD5_TX_STATUS_HOST`] (`mt76_connac_mac.c:580-581`).
pub const MT_PACKET_ID_FIRST: u8 = 3;

// ── descriptor sizes ────────────────────────────────────────────────────────

/// The TXD as this driver writes it: **64 bytes**, 16 dwords.
///
/// Not 32. `MT_TXD_SIZE` is `8 * 4` (`mt76_connac.h:34`) and that is what an MMIO part
/// uses, but `mt76_connac2_mac_write_txwi` sizes the frame with
/// `sz_txd = mt76_is_mmio(dev) ? MT_TXD_SIZE : MT_SDIO_TXD_SIZE` (`mt76_connac_mac.c:514`)
/// and `mt7921/usb.c:152` declares `.txwi_size = MT_SDIO_TXD_SIZE` = `MT_TXD_SIZE + 8 * 4`
/// = 64 (`mt76_connac.h:40`). Dwords 0-8 are populated (DW8 only on this bus — see
/// [`MT_TXD8_L_TYPE`]); dwords 9-15 are zero and their meaning is undetermined.
pub const TXD_LEN: usize = 64;
/// Dwords of the TXD that carry fields. DW9-DW15 exist but nothing upstream writes them.
pub const TXD_USED_DWORDS: usize = 9;
/// `MT_USB_HDR_SIZE` (`mt76_connac.h:37`) — the 4-byte length header
/// `mt792x_skb_add_usb_sdio_hdr` (`mt792x.h:572-583`) pushes in front of the TXD.
pub const USB_HDR_LEN: usize = 4;
/// `MT_USB_TAIL_SIZE` (`mt76_connac.h:38`) — four zero bytes after the 4-aligned payload
/// (`mt7921/mac.c:811-813`: `pad = round_up(len,4) - len; if (usb) pad += 4`).
pub const USB_TAIL_LEN: usize = 4;
/// Bytes of the USB TX header (`MT792x_SDIO_HDR_TX_BYTES`, `mt792x.h:64`). On USB the count
/// **excludes** the header itself (`mt792x.h:578`: `len = mt76_is_usb ? skb->len :
/// skb->len + sizeof(hdr)`), so it equals `TXD_LEN + payload` — the same number as
/// [`MT_TXD0_TX_BYTES`].
pub const MT792X_SDIO_HDR_TX_BYTES: u32 = 0x0000_ffff;
/// Packet-type field of the USB/SDIO header (`mt792x.h:65`). Zero on USB
/// (`mt7921/mac.c:809`: `type = mt76_is_sdio(mdev) ? MT7921_SDIO_DATA : 0`).
pub const MT792X_SDIO_HDR_PKT_TYPE: u32 = 0x0003_0000;

// ── the rate word ───────────────────────────────────────────────────────────

/// A legacy (CCK / OFDM) rate, as a name rather than a magic number.
///
/// `McsDescriptor` has no way to *say* "legacy" — it carries an MCS index and mode flags —
/// so legacy rates come through this enum instead, exactly as they do in
/// `crate::mt76x0`'s backend (`mod mt76x0` is private there, so this is deliberately not a
/// link). [`encode_rate`] therefore covers HT/VHT/HE and
/// [`LegacyRate::rate_val`] covers CCK/OFDM; both emit the *same* 14-bit word, and
/// [`decode_rate`] reads all five modes back.
///
/// ★ The index values are **not** ordinals. They are the `hw_value` low bytes of
/// `mt76_rates[]` (`mac80211.c:160-173` with the macros at `mt76.h:1180-1192`), and the
/// OFDM ones are the 802.11a SIGNAL-field rate codes — 6 Mbps is 11, not 0. A port that
/// assumed ordinals here would transmit 54 Mbps when it asked for 6, which is precisely the
/// failure the a81a taught this lab to look for.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LegacyRate {
    /// DSSS 1 Mbps — 2.4 GHz only.
    Cck1,
    /// DSSS 2 Mbps.
    Cck2,
    /// CCK 5.5 Mbps.
    Cck5_5,
    /// CCK 11 Mbps.
    Cck11,
    /// OFDM 6 Mbps — the universally-decodable rate on both bands, and this workspace's
    /// default for an unmeasured link.
    Ofdm6,
    /// OFDM 9 Mbps.
    Ofdm9,
    /// OFDM 12 Mbps.
    Ofdm12,
    /// OFDM 18 Mbps.
    Ofdm18,
    /// OFDM 24 Mbps.
    Ofdm24,
    /// OFDM 36 Mbps.
    Ofdm36,
    /// OFDM 48 Mbps.
    Ofdm48,
    /// OFDM 54 Mbps.
    Ofdm54,
}

impl LegacyRate {
    /// `(mode, hardware rate code)` for this rate.
    const fn parts(self) -> (RateMode, u8) {
        match self {
            LegacyRate::Cck1 => (RateMode::Cck, 0),
            LegacyRate::Cck2 => (RateMode::Cck, 1),
            LegacyRate::Cck5_5 => (RateMode::Cck, 2),
            LegacyRate::Cck11 => (RateMode::Cck, 3),
            LegacyRate::Ofdm6 => (RateMode::Ofdm, 11),
            LegacyRate::Ofdm9 => (RateMode::Ofdm, 15),
            LegacyRate::Ofdm12 => (RateMode::Ofdm, 10),
            LegacyRate::Ofdm18 => (RateMode::Ofdm, 14),
            LegacyRate::Ofdm24 => (RateMode::Ofdm, 9),
            LegacyRate::Ofdm36 => (RateMode::Ofdm, 13),
            LegacyRate::Ofdm48 => (RateMode::Ofdm, 8),
            LegacyRate::Ofdm54 => (RateMode::Ofdm, 12),
        }
    }

    /// The 14-bit rate word for this rate, long preamble. `NSS` stays 0 (= one stream) and
    /// STBC/DCM/106-tone are all inapplicable to a legacy PPDU.
    pub const fn rate_val(self) -> u16 {
        let (mode, idx) = self.parts();
        (field_prep(MT_TX_RATE_IDX, idx as u32) | field_prep(MT_TX_RATE_MODE, mode.code() as u32))
            as u16
    }

    /// The rate word with the **short-preamble** CCK code (`hw_value_short`, `mt76.h:1184`:
    /// `4 + idx`). Identical to [`LegacyRate::rate_val`] for OFDM, which has no preamble
    /// variants.
    pub const fn rate_val_short_preamble(self) -> u16 {
        let (mode, idx) = self.parts();
        let idx = match mode {
            RateMode::Cck => idx + 4,
            _ => idx,
        };
        (field_prep(MT_TX_RATE_IDX, idx as u32) | field_prep(MT_TX_RATE_MODE, mode.code() as u32))
            as u16
    }

    /// Name a hardware rate code from a received frame's [`RxRate::legacy_code`], or `None`
    /// for a code this table does not contain (including the short-preamble CCK codes 4-7,
    /// which name the same four bit rates and are deliberately not aliased onto them —
    /// preamble length is a real difference in airtime).
    pub const fn from_code(mode: RateMode, code: u8) -> Option<Self> {
        match (mode, code) {
            (RateMode::Cck, 0) => Some(LegacyRate::Cck1),
            (RateMode::Cck, 1) => Some(LegacyRate::Cck2),
            (RateMode::Cck, 2) => Some(LegacyRate::Cck5_5),
            (RateMode::Cck, 3) => Some(LegacyRate::Cck11),
            (RateMode::Ofdm, 11) => Some(LegacyRate::Ofdm6),
            (RateMode::Ofdm, 15) => Some(LegacyRate::Ofdm9),
            (RateMode::Ofdm, 10) => Some(LegacyRate::Ofdm12),
            (RateMode::Ofdm, 14) => Some(LegacyRate::Ofdm18),
            (RateMode::Ofdm, 9) => Some(LegacyRate::Ofdm24),
            (RateMode::Ofdm, 13) => Some(LegacyRate::Ofdm36),
            (RateMode::Ofdm, 8) => Some(LegacyRate::Ofdm48),
            (RateMode::Ofdm, 12) => Some(LegacyRate::Ofdm54),
            _ => None,
        }
    }
}

/// ★ Build the 14-bit [`MT_TXD6_TX_RATE`] word for an [`McsDescriptor`] — HT, VHT and, for
/// the first time in this crate, **HE**.
///
/// Layout (`mt76_connac2_mac.h:133-139`):
/// `IDX[5:0] | DCM[4] | SU_EXT_TONE[5] | MODE[9:6] | NSS[12:10] | STBC[13]`. Bits 4 and 5
/// are shared between the index and the two HE flags, which is legal because VHT and HE
/// indices only reach 11.
///
/// # What is confident, and what is inferred
///
/// **Confident** — three independent attestations each:
///   * `MODE` carries `enum mt76_phy_type`, and `MT_PHY_TYPE_HE_SU = 8` /
///     `HE_EXT_SU = 9` (`mt76.h:344-345`). The RX decoder reads the *same* enum out of
///     `MT_PRXV_TX_MODE`, and the TXS decoder reads it back out of this very field
///     (`mt76_connac_mac.c:673`).
///   * `NSS` is **`nss - 1`**: `mt7915/mac.c:674` writes `nss - 1`, and
///     `mt76_connac_mac.c:662` reads `+ 1`.
///     ⚠ `mt76_connac2_mac_tx_rate_val` disagrees with both — it writes `nss = i + 1` for a
///     1-stream VHT/HE beacon rate (`mt76_connac_mac.c:298-299, 363`), i.e. one too many.
///     That path is unreachable on this chip (`:321-324` short-circuits `is_connac2` to
///     legacy before it), which is presumably why the bug has survived. This function
///     follows the testmode/TXS pair, not `tx_rate_val`.
///   * `STBC` pairs with an incremented stream count: `mt7915/mac.c:667-670` does
///     `if (stbc && nss == 1) { nss++; STBC }`, because an STBC PPDU spreads one *spatial*
///     stream over two *space-time* streams. Done here, and the corresponding RX halving is
///     in `decode_prxv` (private).
///   * `DCM` is bit 4 and `SU_EXT_TONE` bit 5 **within** the index field — the RX side reads
///     exactly those two bits out of `MT_PRXV_TX_RATE` (`mt76_connac_mac.c:1060`, `:1122`),
///     which is a second, independent statement of the same layout.
///
/// **Inferred, and unproven on silicon:**
///   * That a fixed HE rate is *accepted at all* on a 7961. Upstream never sends one:
///     `mt76_connac2_mac_tx_rate_val` returns a legacy rate for every connac2 frame
///     (`mt76_connac_mac.c:321-324`), and the only code in the tree that builds a fixed
///     HE rate word is the **mt7915** testmode path (`mt7915/mac.c:596-704`), a different
///     chip. Everything here is that path's structure with connac2's field offsets. The
///     firmware may simply reject it.
///   * That `er_su` should select `HE_EXT_SU` **with `SU_EXT_TONE` clear**. ER-SU has two
///     variants — 242-tone (the full 20 MHz) and 106-tone (half of it). `SU_EXT_TONE`
///     selects the 106-tone one. This function emits the 242-tone variant, because that is
///     the one whose MCS/NSS range is unrestricted and because a 106-tone PPDU is a
///     narrowband transmission whose interaction with our channel-occupancy sensing has
///     never been thought through. A caller that wants 106-tone can OR in
///     [`MT_TX_RATE_SU_EXT_TONE`]; nothing here does it silently.
///   * That `DCM` composes with any HE MCS. The 802.11ax spec allows DCM only for MCS 0/1/3/4
///     and NSS ≤ 2; the hardware's behaviour on an illegal pair is undetermined, so this
///     function passes the flag through **unchecked** rather than silently reshaping the
///     caller's request into a different rate. The clamp belongs in cognition, where the
///     decision is visible.
///
/// # Clamps that are applied
///
/// This part is **2×2**, so HT indices above 15 and `nss` above 2 are rejected by clamping
/// rather than truncation — `index & 0x3f` on an out-of-range MCS silently becomes a
/// *different, lower* rate, and a radio that quietly transmits something other than what it
/// was asked for is the hardest kind of bug to see from the far end.
pub fn encode_rate(m: &McsDescriptor) -> u16 {
    let (mode, idx, mut nss) = if m.he {
        // HE: 4-bit MCS; ER-SU is a *mode*, not a flag on HE-SU.
        let mode = if m.er_su {
            RateMode::HeExtSu
        } else {
            RateMode::HeSu
        };
        (
            mode,
            u32::from(m.index.min(11)),
            u32::from(m.nss.clamp(1, 2)),
        )
    } else if m.vht {
        (
            RateMode::Vht,
            u32::from(m.index.min(9)),
            u32::from(m.nss.clamp(1, 2)),
        )
    } else {
        // HT carries the stream count inside the index — `nss = 1 + (idx >> 3)`
        // (`mt7915/mac.c:598-600`) — so `McsDescriptor::nss` is ignored here, as its own
        // doc comment says it is for HT.
        let idx = u32::from(m.index.min(15));
        (RateMode::Ht, idx, 1 + (idx >> 3))
    };

    let mut v = field_prep(MT_TX_RATE_IDX, idx) | field_prep(MT_TX_RATE_MODE, mode.code() as u32);

    if m.he && m.dcm {
        v |= MT_TX_RATE_DCM;
    }

    // `mt7915/mac.c:667-670`, verbatim: `if (stbc && nss == 1) { nss++; STBC }`. STBC needs a
    // second space-*time* stream to spread the one spatial stream over, so the NSS field
    // goes up with it. The `nss == 1` guard is upstream's and is kept: a 2-stream rate has
    // nothing spare to Alamouti-encode across. For HT this reads the stream count that was
    // just derived from the index, so MCS 0-7 get STBC and MCS 8-15 do not — which is the
    // right answer for the same reason.
    if m.stbc && nss == 1 {
        nss += 1;
        v |= MT_TX_RATE_STBC;
    }

    v |= field_prep(MT_TX_RATE_NSS, nss.saturating_sub(1));
    v as u16
}

/// A rate word decoded back into its parts — the inverse of [`encode_rate`] and
/// [`LegacyRate::rate_val`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TxRateWord {
    /// PHY mode.
    pub mode: RateMode,
    /// Rate index: an MCS for HT/VHT/HE (masked to 4 bits for VHT/HE, where bits 4-5 are
    /// the two flags below), a hardware rate code for CCK/OFDM.
    pub idx: u8,
    /// Spatial streams (the field is 0-based; this is the field **+ 1**,
    /// `mt76_connac_mac.c:662`).
    pub nss: u8,
    /// STBC.
    pub stbc: bool,
    /// HE dual-carrier modulation.
    pub dcm: bool,
    /// HE ER-SU 106-tone variant.
    pub su_ext_tone: bool,
}

/// Decode a [`MT_TXD6_TX_RATE`] word. Follows `mt76_connac2_mac_fill_txs`
/// (`mt76_connac_mac.c:659-717`), which is the code that reads this same field back out of
/// a transmit-status report — so this is not a hand-written inverse but upstream's own.
pub fn decode_rate(word: u16) -> Option<TxRateWord> {
    let v = u32::from(word);
    let mode = RateMode::from_code(field_get(MT_TX_RATE_MODE, v) as u8)?;
    let raw_idx = field_get(MT_TX_RATE_IDX, v) as u8;
    let idx = if matches!(mode, RateMode::Vht) || mode.is_he() {
        raw_idx & 0x0f
    } else {
        raw_idx
    };
    Some(TxRateWord {
        mode,
        idx,
        nss: field_get(MT_TX_RATE_NSS, v) as u8 + 1,
        stbc: v & MT_TX_RATE_STBC != 0,
        dcm: v & MT_TX_RATE_DCM != 0,
        su_ext_tone: v & MT_TX_RATE_SU_EXT_TONE != 0,
    })
}

// ── the TXD builder ─────────────────────────────────────────────────────────

/// The fixed-rate half of a TXD: everything that goes into DW6.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FixedRate {
    /// The 14-bit rate word from [`encode_rate`] or [`LegacyRate::rate_val`].
    pub rate: u16,
    /// Bandwidth code, 0/1/2/3 = 20/40/80/160 MHz. Must match the tuned channel width — a
    /// rate word wider than the baseband is a malformed PPDU.
    pub bw: u8,
    /// [`MT_TXD6_SGI`], 2 bits. 1 = short GI for HT/VHT; for HE it is the GI code
    /// (0 = 0.8 µs, 1 = 1.6 µs, 2 = 3.2 µs) and must be paired with a legal
    /// [`FixedRate::he_ltf`] — the legal pairs are listed only in the comment at
    /// `mt7915/mac.c:686-695`.
    pub sgi: u8,
    /// [`MT_TXD6_HELTF`], 2 bits, raw. Ignored by the hardware below HE.
    pub he_ltf: u8,
    /// LDPC instead of BCC.
    pub ldpc: bool,
    /// [`MT_TXD6_ANT_ID`] — see that constant: unproven on this chip.
    pub ant_id: u8,
}

impl FixedRate {
    /// The workspace default: a rate at 20 MHz, long GI, BCC.
    pub const fn at(rate: u16) -> Self {
        FixedRate {
            rate,
            bw: 0,
            sgi: 0,
            he_ltf: 0,
            ldpc: false,
            ant_id: 0,
        }
    }

    /// Build DW6 from these settings (`mt76_connac_mac.c:602-608` plus the HE/LDPC bits
    /// `mt7915/mac.c:682-698`).
    fn dw6(&self) -> u32 {
        let mut v = MT_TXD6_FIXED_BW
            | field_prep(MT_TXD6_BW, u32::from(self.bw) & 3)
            | field_prep(MT_TXD6_TX_RATE, u32::from(self.rate))
            | field_prep(MT_TXD6_SGI, u32::from(self.sgi) & 3)
            | field_prep(MT_TXD6_ANT_ID, u32::from(self.ant_id) & 0xf);
        let he = decode_rate(self.rate)
            .map(|r| r.mode.is_he())
            .unwrap_or(false);
        if he {
            v |= field_prep(MT_TXD6_HELTF, u32::from(self.he_ltf) & 3);
        }
        // `mt7915/mac.c:697`: LDPC is forced for any HE PPDU above 20 MHz, whatever the
        // caller asked for. Ported, and stated here rather than left as a surprise.
        if self.ldpc || (he && self.bw > 0) {
            v |= MT_TXD6_LDPC;
        }
        v
    }
}

/// Everything [`build_txd`] needs that it cannot read out of the 802.11 frame itself.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TxdConfig {
    /// Station index for `MT_TXD1_WLAN_IDX`. A keyless broadcast uses the driver's global
    /// WCID; there is no station table in this driver.
    pub wcid: u16,
    /// Which hardware MAC address to transmit from (`MT_TXD1_OWN_MAC`).
    pub omac_idx: u8,
    /// LMAC queue — [`MT_LMAC_AC01`] for best-effort. See that constant for why the number
    /// is 1 and not 2.
    pub q_idx: u8,
    /// `enum tx_pkt_type`; [`MT_TX_TYPE_SF`] on USB.
    pub pkt_fmt: u8,
    /// QoS TID.
    pub tid: u8,
    /// Packet id echoed in the TXS report. Ids below [`MT_PACKET_ID_FIRST`] are reserved and
    /// suppress [`MT_TXD5_TX_STATUS_HOST`], which is how "I do not want a status for this
    /// frame" is expressed.
    pub pktid: u8,
    /// Force no-ACK regardless of the destination address. `None` = derive it from the
    /// frame (group-addressed ⇒ no ACK), which is what mac80211 does for us upstream.
    pub no_ack: Option<bool>,
    /// The fixed rate, or `None` to let the firmware's rate controller choose. `None` also
    /// clears [`MT_TXD2_FIX_RATE`], [`MT_TXD2_HTC_VLD`] and [`MT_TXD3_BA_DISABLE`], which
    /// upstream only ever sets together (`mt76_connac_mac.c:594-620`).
    pub fixed_rate: Option<FixedRate>,
    /// Use this sequence number rather than letting the hardware assign one — see
    /// [`MT_TXD3_SN_VALID`].
    pub seq: Option<u16>,
    /// Remaining transmit attempts (`MT_TXD3_REM_TX_COUNT`). Upstream's data path uses 15.
    pub rem_tx_count: u8,
}

impl Default for TxdConfig {
    /// The broadcast-monitor default: no station, best-effort queue, short format, no
    /// status report, hardware-assigned sequence numbers.
    fn default() -> Self {
        TxdConfig {
            wcid: 0,
            omac_idx: 0,
            q_idx: MT_LMAC_AC01 as u8,
            pkt_fmt: MT_TX_TYPE_SF as u8,
            tid: 0,
            pktid: 0,
            no_ack: None,
            fixed_rate: None,
            seq: None,
            rem_tx_count: 15,
        }
    }
}

/// Build the 64-byte TXD for one bare 802.11 frame.
///
/// A port of `mt76_connac2_mac_write_txwi` (`mt76_connac_mac.c:504-621`) plus its
/// `_80211` half (`:409-502`), taking the `is_connac2` branch everywhere (MEASURED chip id
/// 0x7961) and the non-MMIO branch everywhere (this is a USB part). Concretely that means:
/// [`MT_TXD1_VTA`] stays clear, [`MT_TXD3_SW_POWER_MGMT`] stays clear, the frame type goes
/// in **DW8** rather than DW7, and [`MT_TXD0_TX_BYTES`] counts [`TXD_LEN`] = 64 rather than
/// 32.
///
/// `dot11` is the bare 802.11 frame — no radiotap, no FCS. Frames shorter than 4 bytes
/// (frame-control + duration) cannot have a type read out of them and produce a descriptor
/// with the frame-type fields zeroed, which is a descriptor the hardware will not like; the
/// caller is expected not to do that.
pub fn build_txd(cfg: &TxdConfig, dot11: &[u8]) -> [u8; TXD_LEN] {
    let fc0 = dot11.first().copied().unwrap_or(0);
    let fc1 = dot11.get(1).copied().unwrap_or(0);
    let fc_type = u32::from((fc0 >> 2) & 0x3);
    let fc_stype = u32::from(fc0 >> 4);
    let hdr_len = dot11_hdr_len(fc0, fc1);
    // Address 1 starts at offset 4; bit 0 of its first octet is the group bit.
    let multicast = dot11.get(4).is_some_and(|b| b & 0x01 != 0);
    let is_data = fc_type == 2;
    let no_ack = cfg.no_ack.unwrap_or(multicast);

    let mut d = [0u32; 16];

    // DW0 (`mt76_connac_mac.c:552-555`).
    d[0] = field_prep(MT_TXD0_TX_BYTES, (dot11.len() + TXD_LEN) as u32)
        | field_prep(MT_TXD0_PKT_FMT, u32::from(cfg.pkt_fmt))
        | field_prep(MT_TXD0_Q_IDX, u32::from(cfg.q_idx));

    // DW1 (`:557-565` then `:439-444`). `MT_TXD1_VTA` is deliberately absent: `:560-561`
    // sets it only when `!is_connac2`.
    d[1] = MT_TXD1_LONG_FORMAT
        | field_prep(MT_TXD1_WLAN_IDX, u32::from(cfg.wcid))
        | field_prep(MT_TXD1_OWN_MAC, u32::from(cfg.omac_idx))
        | field_prep(MT_TXD1_HDR_FORMAT, MT_HDR_FORMAT_802_11)
        | field_prep(MT_TXD1_HDR_INFO, (hdr_len / 2) as u32)
        | field_prep(MT_TXD1_TID, u32::from(cfg.tid));

    // DW2 (`:566` then `:449-470`).
    d[2] = field_prep(MT_TXD2_FRAME_TYPE, fc_type)
        | field_prep(MT_TXD2_SUB_TYPE, fc_stype)
        | if multicast { MT_TXD2_MULTICAST } else { 0 };
    // `:459-461` — a fixed rate is mandatory for anything that is *not* unicast data, and
    // unavailable for anything that is. This driver's broadcast frames are data frames to a
    // group address, so they take that path; the condition is ported in full rather than
    // shortened to "multicast", because a unicast data frame silently pinned to a fixed rate
    // is a rate controller that has stopped working without saying so.
    let fixed = cfg.fixed_rate.filter(|_| !is_data || multicast);
    if fixed.is_some() {
        d[2] |= MT_TXD2_FIX_RATE;
    }

    // DW3 (`:568-576`). `MT_TXD3_SW_POWER_MGMT` is absent for the same reason as
    // `MT_TXD1_VTA`.
    d[3] = field_prep(MT_TXD3_REM_TX_COUNT, u32::from(cfg.rem_tx_count) & 0x1f);
    if no_ack {
        d[3] |= MT_TXD3_NO_ACK;
    }
    if let Some(sn) = cfg.seq {
        // `:487-489` — the injected-frame path.
        d[3] |= MT_TXD3_SN_VALID | field_prep(MT_TXD3_SEQ, u32::from(sn) & 0xfff);
    }

    // DW4 is the low half of the packet number; zero without a hardware key.
    // DW5 (`:579-585`).
    d[5] = field_prep(MT_TXD5_PID, u32::from(cfg.pktid));
    if cfg.pktid >= MT_PACKET_ID_FIRST {
        d[5] |= MT_TXD5_TX_STATUS_HOST;
    }

    // DW7 stays 0: `amsdu_en` is false for a driver with no A-MSDU offload, and the
    // fixed-rate tail clears `MT_TXD7_HW_AMSDU` anyway (`:619`).

    // DW8 — the frame type again, on the USB/SDIO side of the bus split (`:497-501`).
    d[8] = field_prep(MT_TXD8_L_TYPE, fc_type) | field_prep(MT_TXD8_L_SUB_TYPE, fc_stype);

    // The fixed-rate tail (`:594-620`). All four of these travel together upstream and are
    // kept together here.
    if let Some(fr) = fixed {
        d[2] |= MT_TXD2_HTC_VLD;
        d[6] |= fr.dw6();
        d[3] |= MT_TXD3_BA_DISABLE;
        d[7] &= !MT_TXD7_HW_AMSDU;
    }

    let mut out = [0u8; TXD_LEN];
    for (i, v) in d.iter().enumerate() {
        out[i * 4..i * 4 + 4].copy_from_slice(&v.to_le_bytes());
    }
    out
}

/// Build the complete bulk-OUT buffer for one 802.11 frame:
/// `[4 B USB header][64 B TXD][802.11][pad to 4][4 B zero tail]`.
///
/// From `mt7921_usb_sdio_tx_prepare_skb` (`mt7921/mac.c:776-821`):
/// `mt792x_skb_add_usb_sdio_hdr` pushes the header (`mt792x.h:572-583`), then
/// `pad = round_up(skb->len, 4) - skb->len` and, on USB, `pad += 4`
/// (`mt7921/mac.c:811-813`) — so the tail is always present and the total is always
/// 4-aligned.
///
/// The USB header's length field **excludes itself** on USB (`mt792x.h:578`) and therefore
/// equals [`MT_TXD0_TX_BYTES`]: two independent copies of the same number, which is worth
/// knowing when a capture disagrees with itself.
pub fn build_usb_tx(cfg: &TxdConfig, dot11: &[u8]) -> Vec<u8> {
    let txd = build_txd(cfg, dot11);
    let body_len = TXD_LEN + dot11.len();
    let hdr = field_prep(MT792X_SDIO_HDR_TX_BYTES, body_len as u32)
        | field_prep(MT792X_SDIO_HDR_PKT_TYPE, 0);

    let unpadded = USB_HDR_LEN + body_len;
    let pad = unpadded.next_multiple_of(4) - unpadded + USB_TAIL_LEN;

    let mut out = Vec::with_capacity(unpadded + pad);
    out.extend_from_slice(&hdr.to_le_bytes());
    out.extend_from_slice(&txd);
    out.extend_from_slice(dot11);
    out.resize(unpadded + pad, 0);
    out
}

// ════════════════════════════════════════════════════════════════════════════
// Tests — the whole file is pure, so all of this runs with the dongle unplugged.
// ════════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod tests {
    use super::*;

    /// Assemble a synthetic RX unit: `dwords` then `payload`, with `rxd[0]`'s length field
    /// patched to the true total so the parser's bound checks see a consistent descriptor.
    fn rx_unit(mut dwords: Vec<u32>, payload: &[u8]) -> Vec<u8> {
        let total = dwords.len() * 4 + payload.len();
        dwords[0] = (dwords[0] & !MT_RXD0_LENGTH) | field_prep(MT_RXD0_LENGTH, total as u32);
        let mut out = Vec::with_capacity(total);
        for v in &dwords {
            out.extend_from_slice(&v.to_le_bytes());
        }
        out.extend_from_slice(payload);
        out
    }

    /// A minimal 24-byte broadcast data frame plus `n` body bytes.
    fn bcast_frame(n: usize) -> Vec<u8> {
        let mut f = vec![0u8; 24 + n];
        f[0] = 0x08; // type 2 (data), subtype 0
        f[1] = 0x00;
        f[4..10].copy_from_slice(&[0xff; 6]); // addr1 = broadcast
        f[10..16].copy_from_slice(&[0x02, 0, 0, 0, 0, 1]); // addr2
        f
    }

    /// ★ The cursor arithmetic, which is the thing this whole file can get wrong in a way
    /// nothing else would notice. Every combination of group bits must land the 802.11
    /// header at head + Σ(present group sizes), and the parse must agree byte-for-byte with
    /// the frame we planted there.
    #[test]
    fn group_walk_cursor_arithmetic() {
        // (group bits, expected descriptor dwords) — the sizes come from
        // mt7921/mac.c:262-364 and are cross-checked against mt7915/mac.c:424-451.
        let cases: &[(u32, usize)] = &[
            (0, 6),
            (MT_RXD1_NORMAL_GROUP_4, 6 + 4),
            (MT_RXD1_NORMAL_GROUP_1, 6 + 4),
            (MT_RXD1_NORMAL_GROUP_2, 6 + 2),
            (MT_RXD1_NORMAL_GROUP_3, 6 + 2),
            (MT_RXD1_NORMAL_GROUP_3 | MT_RXD1_NORMAL_GROUP_5, 6 + 2 + 18),
            (
                MT_RXD1_NORMAL_GROUP_4 | MT_RXD1_NORMAL_GROUP_2 | MT_RXD1_NORMAL_GROUP_3,
                6 + 4 + 2 + 2,
            ),
            (
                MT_RXD1_NORMAL_GROUP_1
                    | MT_RXD1_NORMAL_GROUP_2
                    | MT_RXD1_NORMAL_GROUP_3
                    | MT_RXD1_NORMAL_GROUP_4
                    | MT_RXD1_NORMAL_GROUP_5,
                RXD_MAX_DWORDS,
            ),
        ];

        let frame = bcast_frame(8);
        for &(bits, want_dwords) in cases {
            let mut dw = vec![0u32; want_dwords];
            dw[1] = bits;
            // A valid P-RXV wherever group 3 is claimed, so decode_prxv does not reject.
            if bits & MT_RXD1_NORMAL_GROUP_3 != 0 {
                let p =
                    6 + if bits & MT_RXD1_NORMAL_GROUP_4 != 0 {
                        4
                    } else {
                        0
                    } + if bits & MT_RXD1_NORMAL_GROUP_1 != 0 {
                        4
                    } else {
                        0
                    } + if bits & MT_RXD1_NORMAL_GROUP_2 != 0 {
                        2
                    } else {
                        0
                    };
                dw[p] = field_prep(MT_PRXV_TX_MODE, RateMode::Ofdm.code() as u32)
                    | field_prep(MT_PRXV_TX_RATE, 11);
            }
            let unit = rx_unit(dw, &frame);
            let d = parse_rxd(&unit).expect("descriptor must parse");
            assert_eq!(
                d.rxd_len,
                want_dwords * 4,
                "group bits {bits:#x}: wrong descriptor length"
            );
            assert_eq!(d.hdr_gap, want_dwords * 4, "no remove_pad in this case");
            assert_eq!(
                mpdu_slice(&unit, &d).unwrap(),
                &frame[..],
                "group bits {bits:#x}: the MPDU landed at the wrong offset"
            );
        }
    }

    /// `MT_RXD2_NORMAL_HDR_OFFSET` shifts the header by 2 bytes per unit, on top of the
    /// group walk (`mt7921/mac.c:389`).
    ///
    /// ★ Also the regression test for the alignment trap in [`parse_rxd`]: an odd
    /// `pad_units` makes the *unit* length ≡ 2 (mod 4), and a parser that imported the
    /// mt76x02 4-alignment rule rejects exactly those frames — half of the padded ones —
    /// while looking perfectly healthy on the rest.
    #[test]
    fn remove_pad_shifts_the_header() {
        for body in [4usize, 5] {
            let frame = bcast_frame(body);
            for pad_units in 0u32..=3 {
                let mut dw = vec![0u32; 6];
                dw[2] = field_prep(MT_RXD2_NORMAL_HDR_OFFSET, pad_units);
                let mut padded = vec![0u8; 2 * pad_units as usize];
                padded.extend_from_slice(&frame);
                let unit = rx_unit(dw, &padded);
                let d = parse_rxd(&unit).unwrap_or_else(|e| {
                    panic!("body {body}, pad {pad_units}: {e}");
                });
                assert_eq!(d.hdr_gap, 24 + 2 * pad_units as usize);
                assert_eq!(mpdu_slice(&unit, &d).unwrap(), &frame[..]);
                assert_eq!(d.mpdu_len(), frame.len());
            }
        }
    }

    /// ★ The headline field: present when group 2 is set, `None` — never 0 — when it is not.
    #[test]
    fn timestamp_present_and_absent() {
        let frame = bcast_frame(4);

        // Absent: no group-2 bit, and the dword that *would* hold it is a plausible-looking
        // value sitting in the payload. A parser that reads by fixed offset would find it.
        let mut dw = vec![0u32; 6];
        dw[1] = 0;
        let unit = rx_unit(dw, &frame);
        let d = parse_rxd(&unit).unwrap();
        assert_eq!(d.timestamp, None, "no group 2 means no timestamp, not zero");

        // Present, including the value 0 — which must come back as `Some(0)`, because a
        // TSF that has just wrapped is a real reading and must not read as "absent".
        for stamp in [0u32, 1, 0xdead_beef, u32::MAX] {
            let mut dw = vec![0u32; 8];
            dw[1] = MT_RXD1_NORMAL_GROUP_2;
            dw[6] = stamp;
            dw[7] = 0x5555_5555; // the undecoded second dword of the group
            let unit = rx_unit(dw, &frame);
            let d = parse_rxd(&unit).unwrap();
            assert_eq!(d.timestamp, Some(stamp));
            assert_eq!(d.hdr_gap, 32, "group 2 is 2 dwords");
            assert_eq!(mpdu_slice(&unit, &d).unwrap(), &frame[..]);
        }

        // And it must survive being preceded by groups 4 and 1, which is where an off-by-one
        // in an earlier advance would show up as a garbage timestamp rather than a parse
        // failure.
        let mut dw = vec![0u32; 6 + 4 + 4 + 2];
        dw[1] = MT_RXD1_NORMAL_GROUP_4 | MT_RXD1_NORMAL_GROUP_1 | MT_RXD1_NORMAL_GROUP_2;
        dw[6 + 4 + 4] = 0x0123_4567;
        let unit = rx_unit(dw, &frame);
        assert_eq!(parse_rxd(&unit).unwrap().timestamp, Some(0x0123_4567));
    }

    /// The two RCPI sources must be distinguishable, and the C-RXV one must be read from
    /// dword 6 of the vector (`mt7921/mac.c:351-359`).
    #[test]
    fn rcpi_source_and_offset() {
        let frame = bcast_frame(4);
        let prxv = field_prep(MT_PRXV_TX_MODE, RateMode::Ht.code() as u32)
            | field_prep(MT_PRXV_TX_RATE, 5);

        // P-RXV only.
        let mut dw = vec![0u32; 6 + 2];
        dw[1] = MT_RXD1_NORMAL_GROUP_3;
        dw[6] = prxv;
        dw[7] = 0x0000_00c8; // RCPI0 = 200 -> -10 dBm
        let unit = rx_unit(dw, &frame);
        let d = parse_rxd(&unit).unwrap();
        assert!(!d.rcpi_from_crxv);
        assert_eq!(d.rcpi.unwrap()[0], 200);
        assert_eq!(d.signal_dbm(), Some(-10));
        assert_eq!(d.crxv_offset, None);

        // With group 5, the C-RXV copy wins and the P-RXV value must be discarded.
        let mut dw = vec![0u32; 6 + 2 + 18];
        dw[1] = MT_RXD1_NORMAL_GROUP_3 | MT_RXD1_NORMAL_GROUP_5;
        dw[6] = prxv;
        dw[7] = 0x0000_00c8;
        dw[8 + CRXV_RCPI_DWORD] = 0x0000_9090; // RCPI0 = RCPI1 = 144 -> -38 dBm
        let unit = rx_unit(dw, &frame);
        let d = parse_rxd(&unit).unwrap();
        assert!(d.rcpi_from_crxv);
        assert_eq!(d.rcpi.unwrap(), [144, 144, 0, 0]);
        assert_eq!(d.signal_dbm(), Some(-38));
        assert_eq!(d.crxv_offset, Some(8 * 4));
        assert_eq!(d.hdr_gap, (6 + 2 + 18) * 4);
    }

    /// `to_rssi` must floor, not truncate toward zero — see [`rcpi_to_dbm`].
    #[test]
    fn rcpi_conversion_matches_upstream() {
        assert_eq!(rcpi_to_dbm(220), 0);
        assert_eq!(rcpi_to_dbm(200), -10);
        assert_eq!(rcpi_to_dbm(144), -38);
        assert_eq!(rcpi_to_dbm(0), -110);
        // The odd values are where a truncating divide diverges.
        assert_eq!(rcpi_to_dbm(219), -1);
        assert_eq!(rcpi_to_dbm(1), -110);
    }

    /// The rate decoder is upstream's TXS decoder, so encoder→decoder must round-trip for
    /// every mode this part can transmit — including the HE ones, where the round trip is
    /// the only check available short of an SDR.
    #[test]
    fn rate_encoder_and_decoder_round_trip() {
        // Legacy, both preambles.
        for r in [
            LegacyRate::Cck1,
            LegacyRate::Cck11,
            LegacyRate::Ofdm6,
            LegacyRate::Ofdm24,
            LegacyRate::Ofdm54,
        ] {
            let (mode, code) = r.parts();
            let d = decode_rate(r.rate_val()).unwrap();
            assert_eq!(d.mode, mode);
            assert_eq!(d.idx, code);
            assert_eq!(d.nss, 1, "legacy is single-stream and the field is 0-based");
            assert_eq!(LegacyRate::from_code(d.mode, d.idx), Some(r));
        }
        // OFDM 6 Mbps is code 11, not 0 — the trap this table exists to avoid.
        assert_eq!(decode_rate(LegacyRate::Ofdm6.rate_val()).unwrap().idx, 11);
        // Short preamble moves CCK by 4 and leaves OFDM alone.
        assert_eq!(
            decode_rate(LegacyRate::Cck11.rate_val_short_preamble())
                .unwrap()
                .idx,
            7
        );
        assert_eq!(
            LegacyRate::Ofdm6.rate_val_short_preamble(),
            LegacyRate::Ofdm6.rate_val()
        );

        // HT: the stream count lives inside the index.
        for idx in 0u8..=15 {
            let d = decode_rate(encode_rate(&McsDescriptor::ht(idx))).unwrap();
            assert_eq!(d.mode, RateMode::Ht);
            assert_eq!(d.idx, idx);
            assert_eq!(d.nss, 1 + u8::from(idx >= 8), "HT nss = 1 + (idx >> 3)");
            assert!(!d.dcm && !d.su_ext_tone);
        }

        // VHT, one and two streams.
        let d = decode_rate(encode_rate(&McsDescriptor::vht(9))).unwrap();
        assert_eq!((d.mode, d.idx, d.nss), (RateMode::Vht, 9, 1));
        let d = decode_rate(encode_rate(&McsDescriptor::vht_2ss(7))).unwrap();
        assert_eq!((d.mode, d.idx, d.nss), (RateMode::Vht, 7, 2));

        // ★ HE-SU, HE-SU + DCM, and HE-EXT-SU.
        let he = McsDescriptor {
            he: true,
            index: 5,
            nss: 1,
            ..McsDescriptor::CONSERVATIVE
        };
        let d = decode_rate(encode_rate(&he)).unwrap();
        assert_eq!((d.mode, d.idx, d.nss), (RateMode::HeSu, 5, 1));
        assert!(!d.dcm && !d.su_ext_tone);

        let dcm = McsDescriptor { dcm: true, ..he };
        let d = decode_rate(encode_rate(&dcm)).unwrap();
        assert_eq!(d.mode, RateMode::HeSu);
        assert!(d.dcm, "DCM is bit 4 inside the index field");
        assert_eq!(d.idx, 5, "and must not corrupt the MCS");

        let er = McsDescriptor {
            er_su: true,
            index: 0,
            ..he
        };
        let d = decode_rate(encode_rate(&er)).unwrap();
        assert_eq!(
            d.mode,
            RateMode::HeExtSu,
            "er_su selects the mode, not a flag"
        );
        assert!(
            !d.su_ext_tone,
            "242-tone ER-SU: the 106-tone variant is opt-in, never silent"
        );

        // STBC adds a space-time stream (mt7915/mac.c:667-670) for a 1-stream rate only.
        let d = decode_rate(encode_rate(&McsDescriptor::vht(4).with_stbc())).unwrap();
        assert!(d.stbc);
        assert_eq!(d.nss, 2, "one spatial stream over two space-time streams");
        let d = decode_rate(encode_rate(&McsDescriptor::vht_2ss(4).with_stbc())).unwrap();
        assert!(
            !d.stbc,
            "2 spatial streams have nothing spare to encode across"
        );
    }

    /// The RX rate decoder must read back what the TX encoder wrote, in the fields the two
    /// genuinely share: the low six bits of the index and the mode enum.
    #[test]
    fn prxv_decode_agrees_with_the_tx_encoding() {
        let he_dcm = McsDescriptor {
            he: true,
            dcm: true,
            index: 3,
            nss: 2,
            ..McsDescriptor::CONSERVATIVE
        };
        let word = encode_rate(&he_dcm);
        let tx = decode_rate(word).unwrap();

        // Rebuild the same rate as a P-RXV DW0 and check the RX path agrees.
        let v0 = field_prep(MT_PRXV_TX_RATE, u32::from(word) & MT_TX_RATE_IDX)
            | field_prep(MT_PRXV_TX_MODE, tx.mode.code() as u32)
            | field_prep(MT_PRXV_NSTS, u32::from(tx.nss - 1));
        let rx = decode_prxv(v0).unwrap();
        assert_eq!(rx.mode, RateMode::HeSu);
        assert_eq!(rx.mcs, Some(3));
        assert_eq!(rx.nss, 2);
        assert!(rx.dcm, "DCM survives the trip through both encodings");
        assert_eq!(rx.legacy_code, None);

        // ER-SU at "40 MHz" with the 106-tone bit is a resource unit, not a bandwidth.
        let v0 = field_prep(MT_PRXV_TX_RATE, MT_PRXV_TX_ER_SU_106T)
            | field_prep(MT_PRXV_TX_MODE, RateMode::HeExtSu.code() as u32)
            | field_prep(MT_PRXV_FRAME_MODE, 1);
        assert_eq!(decode_prxv(v0).unwrap().bw, RxBw::HeRu106);
        // Same bandwidth code without the bit really is 40 MHz.
        let v0 = field_prep(MT_PRXV_TX_MODE, RateMode::HeSu.code() as u32)
            | field_prep(MT_PRXV_FRAME_MODE, 1);
        assert_eq!(decode_prxv(v0).unwrap().bw, RxBw::Bw40);

        // STBC halves the reported stream count (mt76_connac_mac.c:1072-1074).
        let v0 = field_prep(MT_PRXV_TX_MODE, RateMode::Vht.code() as u32)
            | field_prep(MT_PRXV_NSTS, 1)
            | field_prep(MT_PRXV_HT_STBC, 1);
        assert_eq!(decode_prxv(v0).unwrap().nss, 1);
    }

    /// ★ The TXD, byte for byte, for a broadcast data frame at a fixed legacy rate — the
    /// case this driver actually transmits. Every constant below is derived by hand from
    /// the file:lines in `build_txd`, so a change that "looks equivalent" has to justify
    /// itself against them.
    #[test]
    fn txd_layout_for_a_broadcast_at_a_fixed_legacy_rate() {
        let frame = bcast_frame(10); // 34 bytes, 24-byte header
        let cfg = TxdConfig {
            fixed_rate: Some(FixedRate::at(LegacyRate::Ofdm6.rate_val())),
            ..TxdConfig::default()
        };
        let txd = build_txd(&cfg, &frame);

        let d = |i: usize| u32::from_le_bytes(txd[i * 4..i * 4 + 4].try_into().unwrap());

        // DW0: TX_BYTES = 34 + 64 = 98, PKT_FMT = MT_TX_TYPE_SF (1), Q_IDX = MT_LMAC_AC01 (1).
        assert_eq!(d(0), 98 | (1 << 23) | (1 << 25), "DW0");
        // DW1: LONG_FORMAT | HDR_FORMAT_802_11 (2) | HDR_INFO = 24/2 = 12. WLAN_IDX, OWN_MAC,
        // TID all 0. VTA must be clear — this is connac2.
        assert_eq!(d(1), (1 << 31) | (2 << 16) | (12 << 11), "DW1");
        assert_eq!(d(1) & MT_TXD1_VTA, 0, "VTA is not set on connac2");
        // DW2: FIX_RATE | HTC_VLD | MULTICAST | FRAME_TYPE 2 | SUB_TYPE 0.
        assert_eq!(d(2), (1u32 << 31) | (1 << 13) | (1 << 10) | (2 << 4), "DW2");
        // DW3: BA_DISABLE | REM_TX_COUNT 15 | NO_ACK. SN_VALID clear (hardware sequence),
        // SW_POWER_MGMT clear (connac2).
        assert_eq!(d(3), (1u32 << 28) | (15 << 11) | 1, "DW3");
        assert_eq!(d(3) & MT_TXD3_SW_POWER_MGMT, 0);
        assert_eq!(d(3) & MT_TXD3_SN_VALID, 0);
        // DW4: no packet number without a key.
        assert_eq!(d(4), 0, "DW4");
        // DW5: pktid 0 is below MT_PACKET_ID_FIRST, so no host status report.
        assert_eq!(d(5), 0, "DW5");
        assert_eq!(d(5) & MT_TXD5_TX_STATUS_HOST, 0);
        // DW6: FIXED_BW | BW 0 | TX_RATE = OFDM code 11 with mode 1 = 0x4b.
        assert_eq!(d(6), (0x4b << 16) | MT_TXD6_FIXED_BW, "DW6");
        // DW7: no hardware A-MSDU.
        assert_eq!(d(7), 0, "DW7");
        // DW8: the frame type again — USB puts it here, not in DW7.
        assert_eq!(d(8), 2 << 4, "DW8");
        // DW9-15 are untouched.
        for i in 9..16 {
            assert_eq!(d(i), 0, "DW{i} must stay zero");
        }
    }

    /// The USB framing around that TXD: header length field, 4-alignment, and the mandatory
    /// zero tail (`mt7921/mac.c:809-815`).
    #[test]
    fn usb_tx_framing() {
        let frame = bcast_frame(10); // 34 bytes
        let cfg = TxdConfig {
            fixed_rate: Some(FixedRate::at(LegacyRate::Ofdm6.rate_val())),
            ..TxdConfig::default()
        };
        let buf = build_usb_tx(&cfg, &frame);

        // 4 + 64 + 34 = 102 -> round up to 104 -> +4 tail = 108.
        assert_eq!(buf.len(), 108);
        assert_eq!(buf.len() % 4, 0);
        let hdr = u32::from_le_bytes(buf[0..4].try_into().unwrap());
        assert_eq!(
            field_get(MT792X_SDIO_HDR_TX_BYTES, hdr),
            (TXD_LEN + frame.len()) as u32,
            "the USB length excludes the header itself"
        );
        assert_eq!(field_get(MT792X_SDIO_HDR_PKT_TYPE, hdr), 0, "USB, not SDIO");
        // The TXD's own byte count must agree with the USB header's.
        let dw0 = u32::from_le_bytes(buf[4..8].try_into().unwrap());
        assert_eq!(
            field_get(MT_TXD0_TX_BYTES, dw0),
            field_get(MT792X_SDIO_HDR_TX_BYTES, hdr)
        );
        assert_eq!(
            &buf[USB_HDR_LEN + TXD_LEN..USB_HDR_LEN + TXD_LEN + 34],
            &frame[..]
        );
        assert!(buf[102..].iter().all(|&b| b == 0), "pad and tail are zero");

        // A frame whose length is already 4-aligned still gets the 4-byte tail and nothing
        // more, because `round_up` contributes 0.
        let f2 = bcast_frame(8); // 32 bytes; 4 + 64 + 32 = 100, already aligned
        assert_eq!(build_usb_tx(&cfg, &f2).len(), 104);
    }

    /// A frame with no fixed rate must not carry any of the four bits upstream only ever
    /// sets together.
    #[test]
    fn no_fixed_rate_leaves_the_rate_bits_alone() {
        let txd = build_txd(&TxdConfig::default(), &bcast_frame(4));
        let d = |i: usize| u32::from_le_bytes(txd[i * 4..i * 4 + 4].try_into().unwrap());
        assert_eq!(d(2) & (MT_TXD2_FIX_RATE | MT_TXD2_HTC_VLD), 0);
        assert_eq!(d(3) & MT_TXD3_BA_DISABLE, 0);
        assert_eq!(d(6), 0);
    }

    /// Unicast data must **not** take the fixed-rate path even when one is supplied — that
    /// is `mt76_connac_mac.c:459-461`'s condition, and shortening it to "a rate was given"
    /// would put every unicast frame on a fixed rate.
    #[test]
    fn fixed_rate_applies_only_where_upstream_applies_it() {
        let mut ucast = bcast_frame(4);
        ucast[4..10].copy_from_slice(&[0x02, 0, 0, 0, 0, 2]); // unicast addr1
        let cfg = TxdConfig {
            fixed_rate: Some(FixedRate::at(LegacyRate::Ofdm6.rate_val())),
            ..TxdConfig::default()
        };
        let txd = build_txd(&cfg, &ucast);
        let dw2 = u32::from_le_bytes(txd[8..12].try_into().unwrap());
        assert_eq!(dw2 & MT_TXD2_FIX_RATE, 0, "unicast data keeps rate control");
        assert_eq!(dw2 & MT_TXD2_MULTICAST, 0);
        let dw3 = u32::from_le_bytes(txd[12..16].try_into().unwrap());
        assert_eq!(dw3 & MT_TXD3_NO_ACK, 0, "unicast wants an ACK");

        // A management frame is not data, so it does take the fixed-rate path even unicast.
        let mut mgmt = ucast.clone();
        mgmt[0] = 0x40; // type 0 (mgmt), subtype 4 = probe request
        let dw2 = u32::from_le_bytes(build_txd(&cfg, &mgmt)[8..12].try_into().unwrap());
        assert_ne!(dw2 & MT_TXD2_FIX_RATE, 0);
    }

    /// Sequence numbers: `None` leaves the hardware in charge, `Some` sets SN_VALID.
    #[test]
    fn explicit_sequence_number() {
        let cfg = TxdConfig {
            seq: Some(0x123),
            ..TxdConfig::default()
        };
        let dw3 = u32::from_le_bytes(build_txd(&cfg, &bcast_frame(4))[12..16].try_into().unwrap());
        assert_ne!(dw3 & MT_TXD3_SN_VALID, 0);
        assert_eq!(field_get(MT_TXD3_SEQ, dw3), 0x123);
    }

    /// A packet id at or above `MT_PACKET_ID_FIRST` turns on the host status report; one
    /// below it does not (`mt76_connac_mac.c:580-581`).
    #[test]
    fn pktid_gates_the_status_report() {
        for (pid, want) in [(0u8, false), (2, false), (3, true), (200, true)] {
            let cfg = TxdConfig {
                pktid: pid,
                ..TxdConfig::default()
            };
            let dw5 =
                u32::from_le_bytes(build_txd(&cfg, &bcast_frame(4))[20..24].try_into().unwrap());
            assert_eq!(dw5 & MT_TXD5_TX_STATUS_HOST != 0, want, "pktid {pid}");
            assert_eq!(field_get(MT_TXD5_PID, dw5), u32::from(pid));
        }
    }

    /// LDPC is forced on for an HE PPDU above 20 MHz whatever the caller asked
    /// (`mt7915/mac.c:697`), and left alone below it.
    #[test]
    fn he_wide_forces_ldpc() {
        let he = McsDescriptor {
            he: true,
            index: 4,
            nss: 1,
            ..McsDescriptor::CONSERVATIVE
        };
        let wide = FixedRate {
            bw: 2,
            ..FixedRate::at(encode_rate(&he))
        };
        assert_ne!(wide.dw6() & MT_TXD6_LDPC, 0);
        let narrow = FixedRate::at(encode_rate(&he));
        assert_eq!(narrow.dw6() & MT_TXD6_LDPC, 0);
        // And a legacy rate at 80 MHz is not silently given LDPC — the rule is HE-only.
        let legacy_wide = FixedRate {
            bw: 2,
            ..FixedRate::at(LegacyRate::Ofdm6.rate_val())
        };
        assert_eq!(legacy_wide.dw6() & MT_TXD6_LDPC, 0);
    }

    /// Every rejection upstream makes, this parser must make too.
    #[test]
    fn malformed_descriptors_are_rejected() {
        let frame = bcast_frame(4);
        let bad = |patch: &dyn Fn(&mut Vec<u32>)| {
            let mut dw = vec![0u32; 6];
            patch(&mut dw);
            parse_rxd(&rx_unit(dw, &frame)).unwrap_err()
        };
        assert_eq!(
            bad(&|d: &mut Vec<u32>| d[1] |= MT_RXD1_NORMAL_BAND_IDX),
            RxdError::BandIdx
        );
        assert_eq!(
            bad(&|d: &mut Vec<u32>| d[2] |= MT_RXD2_NORMAL_AMSDU_ERR),
            RxdError::AmsduErr
        );
        assert_eq!(
            bad(&|d: &mut Vec<u32>| d[2] |= MT_RXD2_NORMAL_MAX_LEN_ERROR),
            RxdError::MaxLenError
        );
        assert_eq!(
            bad(&|d: &mut Vec<u32>| {
                d[1] |= MT_RXD1_NORMAL_CM;
                d[2] |= MT_RXD2_NORMAL_HDR_TRANS;
            }),
            RxdError::HdrTransCipherMismatch
        );

        // Too short for the fixed head.
        assert_eq!(parse_rxd(&[0u8; 20]).unwrap_err(), RxdError::TooShort);

        // A length field that lies.
        let mut unit = rx_unit(vec![0u32; 6], &frame);
        unit[0] = 0xff;
        unit[1] = 0xff;
        assert_eq!(parse_rxd(&unit).unwrap_err(), RxdError::BadLength);

        // A group claimed but not present: the walk must run off the end and say which
        // group, rather than reading the frame as descriptor.
        let mut dw = vec![0u32; 6];
        dw[1] = MT_RXD1_NORMAL_GROUP_5 | MT_RXD1_NORMAL_GROUP_3;
        let unit = rx_unit(dw, &frame[..8]);
        assert!(matches!(
            parse_rxd(&unit).unwrap_err(),
            RxdError::GroupTruncated(_)
        ));

        // A PHY mode this part cannot produce.
        let mut dw = vec![0u32; 8];
        dw[1] = MT_RXD1_NORMAL_GROUP_3;
        dw[6] = field_prep(MT_PRXV_TX_MODE, 13); // EHT-SU: not on 11ax silicon
        assert_eq!(
            parse_rxd(&rx_unit(dw, &frame)).unwrap_err(),
            RxdError::BadRateMode(13)
        );
    }

    /// The A-MSDU pad sits between the header and the body, so removing it moves the
    /// header — a parser that trimmed an end instead would corrupt every subframe.
    #[test]
    fn amsdu_pad_is_removed_from_the_middle() {
        let mut frame = bcast_frame(6); // 24-byte header + 6 body bytes
        for (i, b) in frame[24..].iter_mut().enumerate() {
            *b = 0xa0 + i as u8;
        }
        // Plant the on-air layout: header, 2 pad bytes, body.
        let mut padded = Vec::new();
        padded.extend_from_slice(&frame[..24]);
        padded.extend_from_slice(&[0xee, 0xee]);
        padded.extend_from_slice(&frame[24..]);

        let mut dw = vec![0u32; 6];
        dw[4] = field_prep(MT_RXD4_NORMAL_PAYLOAD_FORMAT, MT_RXD4_FIRST_AMSDU_FRAME);
        let unit = rx_unit(dw, &padded);
        let d = parse_rxd(&unit).unwrap();
        assert_eq!(d.amsdu, AmsduPos::First);
        assert_eq!(d.body_pad, 2);
        assert_eq!(d.mpdu_len(), frame.len());
        assert_eq!(mpdu_slice(&unit, &d), None, "the slice path cannot do this");
        assert_eq!(
            mpdu(&unit, &d).unwrap(),
            frame,
            "pad removed from the middle"
        );

        // Header translation suppresses the pad (`mt7921/mac.c:406`).
        let mut dw = vec![0u32; 6];
        dw[2] = MT_RXD2_NORMAL_HDR_TRANS;
        dw[4] = field_prep(MT_RXD4_NORMAL_PAYLOAD_FORMAT, MT_RXD4_FIRST_AMSDU_FRAME);
        let unit = rx_unit(dw, &padded);
        assert_eq!(parse_rxd(&unit).unwrap().body_pad, 0);
    }

    /// The A-MPDU flag is stored inverted in the descriptor; the struct must not be.
    #[test]
    fn ampdu_flag_is_un_inverted() {
        let frame = bcast_frame(4);
        let mut dw = vec![0u32; 6];
        dw[2] = MT_RXD2_NORMAL_NON_AMPDU;
        assert!(!parse_rxd(&rx_unit(dw, &frame)).unwrap().ampdu);
        assert!(parse_rxd(&rx_unit(vec![0u32; 6], &frame)).unwrap().ampdu);
    }

    /// SNR is not reachable from a normal RXD's group 5, and the helper must say so rather
    /// than return a number read out of the frame body.
    #[test]
    fn snr_is_not_available_from_an_inline_crxv() {
        let inline = [0u8; RXD_GROUP5_DWORDS * 4];
        assert_eq!(
            crxv_snr_db(&inline),
            None,
            "18 dwords cannot contain dword 20"
        );
        assert_eq!(crxv_freq_offset(&inline), None);

        // Given a long-enough vector (a TXRXV report) it does decode, with the -16 bias.
        let mut long = vec![0u8; 22 * 4];
        long[80..84].copy_from_slice(&field_prep(MT_CRXV_SNR, 40).to_le_bytes());
        assert_eq!(crxv_snr_db(&long), Some(24));
    }

    /// The geometry constants must agree with each other; `RXD_MAX_DWORDS` is used to size
    /// bounds elsewhere and a drift between it and the parts would be silent.
    #[test]
    fn geometry_constants_agree() {
        assert_eq!(RXD_MAX_DWORDS, 36);
        assert_eq!(RXD_MAX_DWORDS * 4, 144);
        const { assert!(CRXV_RCPI_DWORD < RXD_GROUP5_DWORDS) };
        // mt7921 splits the group-5 advance as 6 + 12; mt7915 does it as one 18.
        assert_eq!(CRXV_RCPI_DWORD + 12, RXD_GROUP5_DWORDS);
        assert_eq!(TXD_LEN, 64);
        assert_eq!(TXD_USED_DWORDS * 4, 36);
    }
}
