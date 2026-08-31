//! The **connac2 MCU layer** for the MT7921AU (`0e8d:7961`): message framing,
//! the firmware download, and the small command subset a monitor-mode + raw
//! -inject driver actually needs.
//!
//! Ported from the mainline GPL `mt76` tree — `mt76_connac_mcu.c`,
//! `mt76_connac_mcu.h`, `mt76_connac2_mac.h`, `mt7921/mcu.c`, `mt7921/mcu.h`,
//! `mt7921/usb.c`, `mt7921/init.c`, `mt7921/main.c`, `mt792x_usb.c`,
//! `mt792x_core.c`, `mt792x.h`, `mt792x_regs.h`, `mcu.c`, `usb.c`. Every
//! non-obvious constant carries its upstream `file:line`.
//!
//! # MEASURED vs CODE-READ
//!
//! Reasoning about these radios has a ~0 % hit rate on this bench and
//! measurement ~100 %, so the split is kept in the code rather than in prose
//! somewhere else.
//!
//! **MEASURED on the target silicon** (mds-o5p-3, `141.225.165.147`,
//! 2026-08-27) — these constrain everything below:
//!   * The part enumerates at **USB 2.0 high speed**. The WLAN function is
//!     **interface 3**, class `ff/ff/ff`; interfaces 0–2 are `e0/01/01`
//!     Bluetooth and belong to `btusb`. Endpoints on if3: bulk IN `0x84`,
//!     `0x85`; bulk OUT `0x04`–`0x09`; interrupt IN `0x86`; all 512 B.
//!     In `enum mt76u_in_ep` / `enum mt76u_out_ep` order (`mt76.h:646-660`)
//!     that makes `0x84` = `MT_EP_IN_PKT_RX`, `0x85` = `MT_EP_IN_CMD_RESP`,
//!     `0x04` = `MT_EP_OUT_INBAND_CMD`, `0x05` = `MT_EP_OUT_AC_BE`.
//!   * Register access works from our own libusb code via `MT_VEND_READ_EXT`
//!     (`0x63`) / `MT_VEND_WRITE_EXT` (`0x66`) with `bmRequestType` `0xc0`/
//!     `0x40`, `wValue = addr >> 16`, `wIndex = addr & 0xffff`, 4-byte LE
//!     payload. The **full 32-bit address goes straight into wValue/wIndex**;
//!     no remap window is involved. Note upstream uses `0xdf`/`0x5f`
//!     (`MT_USB_TYPE_VENDOR = USB_TYPE_VENDOR | 0x1f`, `mt792x.h:555`) — the
//!     low recipient nibble is evidently decorative on this part, since `0xc0`
//!     read the right values.
//!   * `MT_HW_CHIPID (0x70010200) = 0x0000_7961`,
//!     `MT_HW_REV (0x70010204) = 0x0000_8a10`,
//!     `MT_CONN_ON_MISC (0x7c0600f0) = 0x0000_0000`,
//!     `MT_TOP_MISC (0x700000f0) = 0x0000_0000`.
//!     ★ `MT_CONN_ON_MISC = 0` means `FW_N9_RDY` is **clear**: firmware is not
//!     running on a cold plug, so [`power_up`] takes the *no-wfsys-reset*
//!     branch exactly as `mt7921/usb.c:218-222` does.
//!   * **EP0 round trip = 268 µs** on this USB2 bus (vs 92–151 µs on the rig's
//!     other dongles). Every `rr`/`wr`/`rmw` below costs that, so the poll
//!     budgets in this file are stated in wall-clock milliseconds, not in
//!     "tries".
//!
//! **MEASURED from the shipped blobs** (`fw/mt7961/`, parsed byte-for-byte;
//! every one of these is asserted in the unit tests at the bottom of this
//! file, so a re-vendored blob that does not match fails the test suite rather
//! than the bring-up):
//!
//! | | `WIFI_MT7961_patch_mcu_1_2_hdr.bin` | `WIFI_RAM_CODE_MT7961_1.bin` |
//! |---|---|---|
//! | size | 92 192 B | 791 588 B |
//! | build date | `20250625153620a\n` | `20250625153703` |
//! | identity | `hw_sw_ver = 0x8a10_8a10`, platform `ALPS` | `chip_id = 0x0d`, `eco = 0x01` |
//! | regions | 1 patch section | 5 regions, **4 downloadable + 1 `NON_DL`** |
//!
//! ★ The patch header's `hw_sw_ver` high half is **`0x8a10`, which is exactly
//! the MEASURED `MT_HW_REV`**. [`load_patch`] validates that and refuses a
//! mismatched blob — the one cheap guard against silently pushing a 7961 image
//! at some other connac2 part (or the reverse) and bricking the download.
//!
//! ★ The patch section's load address is **`0x0090_0000`**, which sends
//! [`init_download`] down the `MCU_CMD(PATCH_START_REQ)` branch, **not**
//! `TARGET_ADDRESS_LEN_REQ` (`mt76_connac_mcu.c:68-75`: `is_connac2(dev) &&
//! addr == 0x900000`, and `mt76_connac.h:281-285` puts chip `0x7961` in
//! `is_connac2`). Getting that wrong is a download that ACKs and does nothing.
//!
//! ★ The RAM blob's **region 4 is `NON_DL` (`feature_set = 0x40`) and
//! `type = FW_TYPE_CLC` (2)** — 88 416 B of country/regulatory tables that are
//! *skipped* by the downloader **but still advance the file offset**
//! (`mt76_connac_mcu.c:3059-3080`). A walker that skips the region *and* its
//! bytes mis-aligns every later region; there is no later region here (CLC is
//! last), but the loop is written the upstream way regardless. So: 4 regions
//! are pushed to the chip — `0x0091_5000` (363 536 B, `feature_set = 0x20` =
//! `FW_FEATURE_OVERRIDE_ADDR`, so it becomes the `FW_START_REQ` override
//! address), `0x0201_5c00` (272 400 B), `0x0040_4400` (15 376 B),
//! `0xe027_0000` (51 472 B) — 702 784 B in all.
//!
//! **CODE-READ, unvalidated on this silicon:** everything else. The framing,
//! the download sequence, the semaphore dance, the readiness polls and the
//! whole command subset are faithful transcriptions that have never been run
//! against the part. Where upstream's reason is undetermined this file says so
//! in those words instead of inventing one.
//!
//! # The wire format, exactly
//!
//! A command is one bulk-OUT transfer. Nothing here is a control transfer.
//!
//! ```text
//! ordinary command (MT_EP_OUT_INBAND_CMD = 0x04)
//! ┌──────────┬──────────────────────────────┬──────────┬─────┬──────┐
//! │ usb hdr  │ mt76_connac2_mcu_txd  64 B   │ payload  │ pad │ tail │
//! │   4 B    │  (or ..._uni_txd  48 B)      │   n B    │0..3 │ 4 B  │
//! └──────────┴──────────────────────────────┴──────────┴─────┴──────┘
//!  usb hdr = le32( TX_BYTES[15:0] | PKT_TYPE[17:16] )      mt792x.h:64-65,573-582
//!            TX_BYTES = txd_len + n      (USB: the header itself is NOT counted;
//!                                         SDIO adds 4 — mt792x.h:578)
//!            PKT_TYPE = 0 on USB         (mt7921/usb.c:52, mt7921/mac.c:810 —
//!                                         only SDIO ever passes MT7921_SDIO_DATA)
//!  pad+tail: pad = round_up(len,4) + 4 - len   mt7921/usb.c:53-54
//!            i.e. round the frame up to 4 and then append 4 zero bytes
//!            (MT_USB_TAIL_SIZE, mt76_connac.h:38). It is NOT a zero-length-
//!            packet substitute: mt76 never sets URB_ZERO_PACKET, the device
//!            takes its length from the header.
//!
//! firmware chunk (MCU_CMD(FW_SCATTER) → MT_EP_OUT_AC_BE = 0x05)
//! ┌──────────┬───────────────────────┬─────┬──────┐
//! │ usb hdr  │ up to 4096 B of image │ pad │ tail │
//! └──────────┴───────────────────────┴─────┴──────┘
//!  ★ no txd at all: mt76_connac2_mcu_fill_message goes straight to `exit`
//!    for FW_SCATTER (mt76_connac_mcu.c:3493-3494), and the frame leaves on
//!    AC_BE rather than the command pipe (mt7921/usb.c:47-50). Both halves of
//!    that are silent breakage if missed: a txd'd chunk is rejected, and a
//!    chunk on 0x04 simply never lands.
//!  ★ AC_BE only carries firmware because MT_UDMA_TX_QSEL's MT_FW_DL_EN bit is
//!    set for the duration of the download (mt7921/usb.c:76,82). See
//!    [`run_firmware`].
//! ```
//!
//! ## `mt76_connac2_mcu_txd`, 64 bytes (`mt76_connac_mcu.h:53-70`)
//!
//! ```text
//! off  size  field
//!   0    32  txd[8] — the hardware descriptor
//!            txd[0] = Q_IDX[31:25]=MT_TX_MCU_PORT_RX_Q0(0x20)
//!                   | PKT_FMT[24:23]=MT_TX_TYPE_CMD(2)
//!                   | TX_BYTES[15:0]=64+n                      = 0x4100_0000|len
//!            txd[1] = LONG_FORMAT(31) | HDR_FORMAT[17:16]=MT_HDR_FORMAT_CMD(1)
//!                                                              = 0x8001_0000
//!            txd[2..8] = 0
//!  32     2  le16 len   = 32 + n     (skb->len - sizeof(txd[8]))
//!  34     2  le16 pq_id = MCU_PQ_ID(MT_TX_PORT_IDX_MCU, MT_TX_MCU_PORT_RX_Q0)
//!                       = (1<<15) | (0x20<<10) = 0x8000   ← both terms are the
//!                         same bit; that is upstream's arithmetic, not a typo
//!  36     1  cid        = command id (low 8 bits of the packed cmd word)
//!  37     1  pkt_type   = MCU_PKT_ID = 0xa0
//!  38     1  set_query  = MCU_Q_QUERY(0) / MCU_Q_SET(1) / MCU_Q_NA(3)
//!  39     1  seq        = 1..15
//!  40     1  uc_d2b0_rev = 0
//!  41     1  ext_cid
//!  42     1  s2d_index  = MCU_S2D_H2N(0), or MCU_S2D_H2C(2) for a WA command
//!  43     1  ext_cid_ack = (ext_cid != 0)
//!  44    20  rsv[5] = 0
//! ```
//!
//! ## `mt76_connac2_mcu_uni_txd`, 48 bytes (`mt76_connac_mcu.h:101-121`)
//!
//! Same `txd[8]` (with `TX_BYTES = 48 + n`), then `le16 len = 16 + n`,
//! `le16 cid`, `rsv`, `pkt_type = 0xa0`, `frag_n = 0`, `seq`, `le16 checksum = 0`,
//! `s2d_index = MCU_S2D_H2N`, `option = MCU_CMD_UNI_EXT_ACK (0x7)`, `rsv1[4]`.
//! The option byte is unconditional upstream — a UNI *query* still sets the
//! `SET` bit. Ported as-is; the reason is undetermined.
//!
//! ## Response, `mt76_connac2_mcu_rxd`, 36-byte header (`mt76_connac_mcu.h:123-143`)
//!
//! ```text
//!   0    24  rxd[6] — rxd[0] bits 15:0 = total length, bits 31:27 = pkt_type
//!  24     2  le16 len
//!  26     2  le16 pkt_type_id
//!  28     1  eid          32     1  ext_eid   ← ★ the status byte, see below
//!  29     1  seq          33     2  rsv1[2]
//!  30     1  option       35     1  s2d_index
//!  31     1  rsv          36    ..  payload / TLVs
//! ```
//! ★ `mt7921_mcu_parse_response` (`mt7921/mcu.c:34-37`) reads the result of
//! `PATCH_SEM_CONTROL` and `PATCH_FINISH_REQ` as **the single byte at offset
//! 32** (`skb_pull(sizeof(*rxd) - 4)`), i.e. the byte the struct labels
//! `ext_eid`. That is not a typo in this port; the firmware puts the status in
//! the last dword of the header region. [`McuEvent::status_u8`] is that byte.
//!
//! # What this module deliberately does **not** do
//!
//! Named so the omissions are visible instead of lost:
//!   * **`mt7921_mcu_get_nic_capability`** (`mt7921/mcu.c:576`) — a TLV walk
//!     that fills in HE/6 GHz/EEPROM-mode capability for mac80211. Nothing in a
//!     monitor + raw-inject driver reads it, and mis-parsing a TLV stream we
//!     never consult is pure risk.
//!   * **`mt7921_load_clc`** (`mt7921/mcu.c:412`) — parses the `NON_DL` CLC
//!     region and pushes `MCU_CE_CMD(SET_CLC)` per country. This driver does
//!     not do regulatory; [`parse_ram`] identifies the region and skips it.
//!   * **`mt7921_mcu_fw_log_2_host(dev, 1)`** (`mt7921/mcu.c:644`,
//!     `mt7921/mcu.c:674`) — turns on firmware logging *into the event pipe*.
//!     We share that pipe with the command responses, so enabling it would add
//!     unbounded unsolicited traffic for [`Connac2Mcu::wait_event`] to filter.
//!     Left off. Turn it on deliberately if a download ever fails opaquely.
//!   * **`mt7921_mac_init`** (`mt7921/init.c:64`), `mt792x_mac_init_band`, the
//!     WTBL wipe, `MCU_EXT_CMD(SET_RTS_THRESH)` — MAC-block bring-up, which
//!     belongs to `super::mac` / the backend, not to the MCU transport.
//!   * **`mt792xu_dma_init` / `mt792xu_wfdma_init`** (`mt792x_usb.c:279-422`) —
//!     ★ **required before any of this works**, and assigned to no file in this
//!     port. `MT_UDMA_WLCFG_0`'s `MT_WL_RX_EN | MT_WL_TX_EN` are what make the
//!     bulk pipes carry anything at all, and `MT_WFDMA_HOST_CONFIG_USB_RXEVT_EP4_EN`
//!     decides which IN endpoint events land on. See [`Connac2Mcu`] for how
//!     this module survives not knowing which way that bit went.
//!   * **Suspend/resume, TX status, scan, station/BSS records, PM ownership**
//!     (`mt792xe_mcu_drv_pmctrl` and friends) — the PCIe/SDIO ownership dance
//!     has no USB counterpart in `mt792xu_*`; USB bring-up is
//!     [`power_up`] and nothing else.
#![allow(dead_code)]

use std::sync::atomic::{AtomicU8, Ordering};
use std::time::{Duration, Instant};

use crate::FaceError;
use crate::connac2::regs::{
    MT_CBTOP_RGU_WF_SUBSYS_RST, MT_CBTOP_RGU_WF_SUBSYS_RST_WF_WHOLE_PATH, MT_CONN_ON_MISC,
    MT_FW_DL_EN, MT_HW_CHIPID, MT_HW_REV, MT_SSUSB_EPCTL_CSR_EP_RST_OPT, MT_SWDEF_MODE,
    MT_SWDEF_NORMAL_MODE, MT_TOP_MISC_FW_STATE, MT_TOP_MISC2_FW_N9_RDY, MT_TOP_MISC2_FW_PWR_ON,
    MT_UDMA_CONN_INFRA_STATUS, MT_UDMA_CONN_INFRA_STATUS_SEL, MT_UDMA_CONN_WFSYS_INIT_DONE,
    MT_UDMA_TX_QSEL, MT_UDMA_WLCFG_0, MT_WL_RX_BUSY, MT_WL_RX_FLUSH, MT_WL_TX_BUSY,
};
use crate::connac2::usb::Connac2Usb;

// ─────────────────────────────────────────────────────────────────────────────
// Errors and bit helpers
// ─────────────────────────────────────────────────────────────────────────────

/// Every error out of this module carries the `mt7921u mcu:` prefix so a log
/// line says which of the rig's several USB radios produced it — the same
/// discipline as [`crate::mt76x0::mcu`]'s `mt7610u mcu:`.
fn mcu_err(msg: impl AsRef<str>) -> FaceError {
    FaceError::Io(std::io::Error::other(format!(
        "mt7921u mcu: {}",
        msg.as_ref()
    )))
}

/// `FIELD_PREP`: place `val` into the contiguous `mask`.
///
/// Local rather than borrowed from [`crate::mt76::regs`] because that helper
/// belongs to the mt76x02 family's register map and this part shares none of
/// it — the two headers only look alike.
const fn fp(mask: u32, val: u32) -> u32 {
    (val << mask.trailing_zeros()) & mask
}

/// `FIELD_GET`: extract the contiguous `mask` from `val`.
const fn fg(mask: u32, val: u32) -> u32 {
    (val & mask) >> mask.trailing_zeros()
}

/// `FIELD_GET` over a `u8` register/field byte (the firmware `feature_set`
/// nibbles are byte-wide).
const fn fg8(mask: u8, val: u8) -> u8 {
    (val & mask) >> mask.trailing_zeros()
}

// ─────────────────────────────────────────────────────────────────────────────
// Note on USB vendor requests
// ─────────────────────────────────────────────────────────────────────────────
//
// This module issues **no** vendor request itself. `MT_VEND_READ_EXT` /
// `MT_VEND_WRITE_EXT` (`mt76.h:641-642`) reach the normal register window
// through [`Connac2Usb::rr`] / [`Connac2Usb::wr`], `MT_VEND_DEV_MODE` /
// `MT_VEND_WRITE` (`:632-633`) reach the UHW window through
// [`Connac2Usb::uhw_rr`] / [`Connac2Usb::uhw_wr`], and `MT_VEND_POWER_ON`
// (`:634`) is [`Connac2Usb::power_on`]. That is deliberate: the request code
// and the `bmRequestType` together select an **address space** on this part
// (`MT_USB_TYPE_UHW_VENDOR` vs `MT_USB_TYPE_VENDOR`, `mt792x.h:555-556`), so
// the pairing belongs to the transport where it can be kept consistent, not to
// each caller.

// ─────────────────────────────────────────────────────────────────────────────
// Command encoding — the packed `__MCU_CMD_FIELD_*` word
// ─────────────────────────────────────────────────────────────────────────────

/// `__MCU_CMD_FIELD_ID` — the command id (`mt76_connac_mcu.h:1215`).
pub const MCU_CMD_FIELD_ID: u32 = 0x0000_00ff;
/// `__MCU_CMD_FIELD_EXT_ID` — the extended command id (`:1216`).
pub const MCU_CMD_FIELD_EXT_ID: u32 = 0x0000_ff00;
/// `__MCU_CMD_FIELD_QUERY` — read rather than write (`:1217`).
pub const MCU_CMD_FIELD_QUERY: u32 = 1 << 16;
/// `__MCU_CMD_FIELD_UNI` — use the 48-byte unified descriptor (`:1218`).
pub const MCU_CMD_FIELD_UNI: u32 = 1 << 17;
/// `__MCU_CMD_FIELD_CE` — a "connectivity engine" offload command (`:1219`).
pub const MCU_CMD_FIELD_CE: u32 = 1 << 18;
/// `__MCU_CMD_FIELD_WA` — addressed to the WA core, not WM (`:1220`).
pub const MCU_CMD_FIELD_WA: u32 = 1 << 19;
/// `__MCU_CMD_FIELD_WM` — addressed to the WM core (`:1221`). Never set by
/// anything this module sends; present for completeness of the decode.
pub const MCU_CMD_FIELD_WM: u32 = 1 << 20;

/// `MCU_CMD_EXT_CID` (`mt76_connac_mcu.h:1376`) — the id every `MCU_EXT_CMD`
/// rides on, with the real command in `EXT_ID`.
pub const MCU_CMD_EXT_CID: u8 = 0xed;

/// `MCU_PKT_ID` (`mt76_connac_mcu.h:51`) — the constant `pkt_type` byte in
/// both descriptor flavours. "cmd packet by long format".
pub const MCU_PKT_ID: u8 = 0xa0;

/// `MCU_PQ_ID(MT_TX_PORT_IDX_MCU, MT_TX_MCU_PORT_RX_Q0)`
/// (`mt76_connac_mcu.h:50`, `mt76_connac2_mac.h:345,354`) —
/// `(1 << 15) | (0x20 << 10)`. Both terms land on bit 15, so the field is
/// `0x8000`.
pub const MCU_PQ_ID_RX_Q0: u16 = 0x8000;

/// `enum { MCU_Q_QUERY, MCU_Q_SET, MCU_Q_RESERVED, MCU_Q_NA }`
/// (`mt76_connac_mcu.h:1113-1118`).
pub const MCU_Q_QUERY: u8 = 0;
/// See [`MCU_Q_QUERY`].
pub const MCU_Q_SET: u8 = 1;
/// See [`MCU_Q_QUERY`]. Used when a command is neither EXT nor CE — i.e. the
/// firmware-download commands.
pub const MCU_Q_NA: u8 = 3;

/// `MCU_S2D_H2N` — host to WM (`mt76_connac_mcu.h:1120-1125`).
pub const MCU_S2D_H2N: u8 = 0;
/// `MCU_S2D_H2C` — host to WA.
pub const MCU_S2D_H2C: u8 = 2;

/// `MCU_CMD_UNI_EXT_ACK = MCU_CMD_ACK | MCU_CMD_UNI | MCU_CMD_SET`
/// (`mt76_connac_mcu.h:1208-1213`) — the `option` byte of every UNI command.
pub const MCU_CMD_UNI_EXT_ACK: u8 = 0x07;

/// One MCU command, as the packed word upstream's `MCU_*_CMD()` macros build.
///
/// It is a single `u32` and not an enum because the *bits* are the protocol:
/// [`Connac2Mcu::send`] branches on `UNI`, `CE`, `QUERY` and `WA` to fill the
/// descriptor, exactly as `mt76_connac2_mcu_fill_message` does
/// (`mt76_connac_mcu.c:3496-3543`). Losing the packing would mean re-deriving
/// those decisions at every call site.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct McuCmd(pub u32);

impl McuCmd {
    /// `MCU_CMD(_t)` — a bare command id (`mt76_connac_mcu.h:1223`).
    pub const fn plain(id: u8) -> Self {
        Self(fp(MCU_CMD_FIELD_ID, id as u32))
    }

    /// `MCU_EXT_CMD(_t)` — `EXT_CID` in the id byte, the real command in
    /// `EXT_ID` (`mt76_connac_mcu.h:1225-1228`).
    pub const fn ext(ext_id: u8) -> Self {
        Self(fp(MCU_CMD_FIELD_ID, MCU_CMD_EXT_CID as u32) | fp(MCU_CMD_FIELD_EXT_ID, ext_id as u32))
    }

    /// `MCU_EXT_QUERY(_t)` (`mt76_connac_mcu.h:1228`).
    pub const fn ext_query(ext_id: u8) -> Self {
        Self(Self::ext(ext_id).0 | MCU_CMD_FIELD_QUERY)
    }

    /// `MCU_UNI_CMD(_t)` (`mt76_connac_mcu.h:1229-1232`).
    pub const fn uni(id: u8) -> Self {
        Self(MCU_CMD_FIELD_UNI | fp(MCU_CMD_FIELD_ID, id as u32))
    }

    /// `MCU_CE_CMD(_t)` (`mt76_connac_mcu.h:1237-1240`).
    pub const fn ce(id: u8) -> Self {
        Self(MCU_CMD_FIELD_CE | fp(MCU_CMD_FIELD_ID, id as u32))
    }

    /// `MCU_CE_QUERY(_t)` (`mt76_connac_mcu.h:1240`).
    pub const fn ce_query(id: u8) -> Self {
        Self(Self::ce(id).0 | MCU_CMD_FIELD_QUERY)
    }

    /// The `cid` byte that goes into the descriptor.
    pub const fn id(self) -> u8 {
        fg(MCU_CMD_FIELD_ID, self.0) as u8
    }

    /// The `ext_cid` byte; 0 for a non-EXT command.
    pub const fn ext_id(self) -> u8 {
        fg(MCU_CMD_FIELD_EXT_ID, self.0) as u8
    }

    /// Does this command use the 48-byte unified descriptor?
    pub const fn is_uni(self) -> bool {
        self.0 & MCU_CMD_FIELD_UNI != 0
    }

    /// Is this a CE (offload-core) command?
    pub const fn is_ce(self) -> bool {
        self.0 & MCU_CMD_FIELD_CE != 0
    }

    /// Is this a read rather than a write?
    pub const fn is_query(self) -> bool {
        self.0 & MCU_CMD_FIELD_QUERY != 0
    }

    /// Is this addressed to the WA core (`s2d_index = MCU_S2D_H2C`)?
    pub const fn is_wa(self) -> bool {
        self.0 & MCU_CMD_FIELD_WA != 0
    }
}

// -- the commands this module actually sends ---------------------------------

/// `MCU_CMD(TARGET_ADDRESS_LEN_REQ)` = `0x01` (`mt76_connac_mcu.h:1365`) — the
/// "here comes a region at this address" preamble for a **RAM** region.
pub const CMD_TARGET_ADDRESS_LEN_REQ: McuCmd = McuCmd::plain(0x01);
/// `MCU_CMD(FW_START_REQ)` = `0x02` (`:1366`) — run what was just downloaded.
pub const CMD_FW_START_REQ: McuCmd = McuCmd::plain(0x02);
/// `MCU_CMD(NIC_POWER_CTRL)` = `0x04` (`:1368`) — `mt76_connac_mcu_restart`.
pub const CMD_NIC_POWER_CTRL: McuCmd = McuCmd::plain(0x04);
/// `MCU_CMD(PATCH_START_REQ)` = `0x05` (`:1369`) — ★ the preamble used instead
/// of [`CMD_TARGET_ADDRESS_LEN_REQ`] when the region loads at `0x900000`,
/// which is exactly where our patch blob's one section goes.
pub const CMD_PATCH_START_REQ: McuCmd = McuCmd::plain(0x05);
/// `MCU_CMD(PATCH_FINISH_REQ)` = `0x07` (`:1370`) — `mt76_connac_mcu_start_patch`.
pub const CMD_PATCH_FINISH_REQ: McuCmd = McuCmd::plain(0x07);
/// `MCU_CMD(PATCH_SEM_CONTROL)` = `0x10` (`:1371`) — acquire/release the ROM
/// patch semaphore.
pub const CMD_PATCH_SEM_CONTROL: McuCmd = McuCmd::plain(0x10);
/// `MCU_CMD(FW_SCATTER)` = `0xee` (`:1377`) — one chunk of image bytes. ★ The
/// only command that carries **no descriptor** and leaves on
/// `MT_EP_OUT_AC_BE` rather than the command pipe (`mt7921/usb.c:47-50`).
pub const CMD_FW_SCATTER: McuCmd = McuCmd::plain(0xee);

/// `MCU_EXT_CMD(EFUSE_ACCESS)` = ext `0x01` (`mt76_connac_mcu.h:1260`), issued
/// as a query — reads one 16-byte efuse block.
pub const EXT_QUERY_EFUSE_ACCESS: McuCmd = McuCmd::ext_query(0x01);
/// `MCU_EXT_CMD(CHANNEL_SWITCH)` = ext `0x08` (`:1265`).
pub const EXT_CMD_CHANNEL_SWITCH: McuCmd = McuCmd::ext(0x08);
/// `MCU_EXT_CMD(EFUSE_BUFFER_MODE)` = ext `0x21` (`:1269`) —
/// `mt7921_mcu_set_eeprom`.
pub const EXT_CMD_EFUSE_BUFFER_MODE: McuCmd = McuCmd::ext(0x21);
/// `MCU_EXT_CMD(MAC_INIT_CTRL)` = ext `0x46` (`:1283`) —
/// `mt76_connac_mcu_set_mac_enable`.
pub const EXT_CMD_MAC_INIT_CTRL: McuCmd = McuCmd::ext(0x46);
/// `MCU_EXT_CMD(SET_RX_PATH)` = ext `0x4e` (`:1289`). Shares
/// [`ChannelReq`]'s payload with [`EXT_CMD_CHANNEL_SWITCH`] but wants
/// `rx_streams` as a **mask**, not a count — see [`ChannelReq::encode`].
pub const EXT_CMD_SET_RX_PATH: McuCmd = McuCmd::ext(0x4e);

/// `MCU_CE_CMD(SET_RX_FILTER)` = CE `0x0a` (`mt76_connac_mcu.h:1386`) —
/// `mt7921_mcu_set_rxfilter`.
pub const CE_CMD_SET_RX_FILTER: McuCmd = McuCmd::ce(0x0a);

/// `MCU_CE_CMD(SET_EDCA_PARMS)` = CE `0x1d` (`mt76_connac_mcu.h:1391`).
///
/// ★ **The contention knob, and on this part it is the throughput knob.** MEASURED: with the
/// firmware's defaults the fixed per-PPDU cost is ~185 µs at 80 MHz, and the default `cw_min`
/// **exponent of 5** (`mt7921/mcu.c:747-750` substitutes 5 when mac80211 passes 0) is a
/// contention window of 31 slots — an average backoff of 15.5 x 9 µs = **140 µs**, i.e. very
/// nearly the whole of it. `txop` is likewise 0 by default, so the MAC contends once per PPDU
/// instead of bursting several.
///
/// ⚠ This is NOT the register-level "aggressive EDCA" that wedged the MT7612U. That wrote CW
/// exponent 0 straight into `MT_WMM_*`/`MT_EDCA_CFG_AC(n)` and cost a physical replug. Here the
/// firmware owns the arbiter and validates the request, and the sane move is a *smaller*
/// exponent (2-4), not zero.
pub const CE_CMD_SET_EDCA_PARMS: McuCmd = McuCmd::ce(0x1d);

/// One access category's EDCA parameters — `struct edca` (`mt7921/mcu.c:694-701`), 10 bytes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EdcaAc {
    /// Contention-window minimum, as an **exponent**: CW = 2^cw_min - 1. Firmware default 5
    /// (31 slots, ~140 µs average backoff at a 9 µs slot).
    pub cw_min: u16,
    /// Contention-window maximum exponent. Firmware default 10.
    pub cw_max: u16,
    /// TXOP limit in **32 µs units**; 0 = one PPDU per contention.
    pub txop: u16,
    /// Arbitration inter-frame spacing, in slots.
    pub aifs: u16,
    /// Admission-control guard time.
    pub guardtime: u8,
    /// Admission control mandatory.
    pub acm: u8,
}

impl EdcaAc {
    /// The firmware's own defaults, for restoring.
    pub const DEFAULT: EdcaAc = EdcaAc {
        cw_min: 5,
        cw_max: 10,
        txop: 0,
        aifs: 2,
        guardtime: 0,
        acm: 0,
    };

    fn encode(&self, out: &mut Vec<u8>) {
        out.extend_from_slice(&self.cw_min.to_le_bytes());
        out.extend_from_slice(&self.cw_max.to_le_bytes());
        out.extend_from_slice(&self.txop.to_le_bytes());
        out.extend_from_slice(&self.aifs.to_le_bytes());
        out.push(self.guardtime);
        out.push(self.acm);
    }
}

/// Program all four access categories — `struct mt7921_mcu_tx` (`mt7921/mcu.c:703-712`):
/// four 10-byte `edca` records then `bss_idx`, `qos`, `wmm_idx`, pad. 44 bytes total.
///
/// ⚠ The array is indexed by **ACI**, not by mac80211's AC order; upstream maps through
/// `to_aci[] = {1, 0, 2, 3}` (`mt7921/mcu.c:736`). Callers here pass ACI order directly, since
/// this driver has no mac80211 queues to translate from.
pub fn set_edca(
    mcu: &Connac2Mcu,
    bus: &Connac2Usb,
    acs: &[EdcaAc; 4],
    bss_idx: u8,
    qos: bool,
    wmm_idx: u8,
) -> Result<(), FaceError> {
    let mut req = Vec::with_capacity(44);
    for ac in acs {
        ac.encode(&mut req);
    }
    req.push(bss_idx);
    req.push(u8::from(qos));
    req.push(wmm_idx);
    req.push(0);
    debug_assert_eq!(req.len(), 44);
    mcu.send(bus, CE_CMD_SET_EDCA_PARMS, &req, false)
        .map(|_| ())
}

/// `MCU_UNI_CMD(SNIFFER)` = UNI `0x24` (`mt76_connac_mcu.h:1336`) — the
/// monitor-mode enable and its channel description.
pub const UNI_CMD_SNIFFER: McuCmd = McuCmd::uni(0x24);

// ─────────────────────────────────────────────────────────────────────────────
// Framing
// ─────────────────────────────────────────────────────────────────────────────

/// `MT792x_SDIO_HDR_TX_BYTES` — bits 15:0 of the 4-byte USB/SDIO prefix
/// (`mt792x.h:64`).
pub const SDIO_HDR_TX_BYTES: u32 = 0x0000_ffff;
/// `MT792x_SDIO_HDR_PKT_TYPE` — bits 17:16 (`mt792x.h:65`). Always 0 on USB.
pub const SDIO_HDR_PKT_TYPE: u32 = 0x0003_0000;

/// `MT_TXD0_TX_BYTES` (`mt76_connac2_mac.h:53`).
const MT_TXD0_TX_BYTES: u32 = 0x0000_ffff;
/// `MT_TXD0_PKT_FMT` (`mt76_connac2_mac.h:51`).
const MT_TXD0_PKT_FMT: u32 = 0x0180_0000;
/// `MT_TXD0_Q_IDX` (`mt76_connac2_mac.h:50`).
const MT_TXD0_Q_IDX: u32 = 0xfe00_0000;
/// `MT_TXD1_LONG_FORMAT` (`mt76_connac2_mac.h:55`).
const MT_TXD1_LONG_FORMAT: u32 = 1 << 31;
/// `MT_TXD1_HDR_FORMAT` (`mt76_connac2_mac.h:61`).
const MT_TXD1_HDR_FORMAT: u32 = 0x0003_0000;
/// `MT_TX_TYPE_CMD` (`mt76_connac2_mac.h:14-19`).
const MT_TX_TYPE_CMD: u32 = 2;
/// `MT_HDR_FORMAT_CMD` (`mt76_connac2_mac.h:7-12`).
const MT_HDR_FORMAT_CMD: u32 = 1;
/// `MT_TX_MCU_PORT_RX_Q0` (`mt76_connac2_mac.h:345-346`).
const MT_TX_MCU_PORT_RX_Q0: u32 = 0x20;

/// `sizeof(struct mt76_connac2_mcu_txd)` (`mt76_connac_mcu.h:53-70`).
pub const MCU_TXD_LEN: usize = 64;
/// `sizeof(struct mt76_connac2_mcu_uni_txd)` (`mt76_connac_mcu.h:101-121`).
pub const MCU_UNI_TXD_LEN: usize = 48;
/// `MT_SDIO_HDR_SIZE` (`mt76_connac.h:42`) — the 4-byte USB/SDIO prefix.
pub const USB_HDR_LEN: usize = 4;
/// `MT_USB_TAIL_SIZE` (`mt76_connac.h:38`) — the 4 zero bytes appended after
/// the 4-byte round-up.
pub const USB_TAIL_LEN: usize = 4;
/// `sizeof(struct mt76_connac2_mcu_rxd)` (`mt76_connac_mcu.h:123-143`).
pub const MCU_RXD_LEN: usize = 36;

/// The 4-byte USB/SDIO prefix (`mt792x_skb_add_usb_sdio_hdr`,
/// `mt792x.h:573-582`).
///
/// `tx_bytes` is the length of what follows **before** padding and **not**
/// counting these four bytes. On SDIO the same helper adds 4; the USB branch
/// does not, which is why this takes the already-correct value rather than
/// computing it.
pub fn usb_sdio_hdr(tx_bytes: u16, pkt_type: u8) -> [u8; 4] {
    let v = fp(SDIO_HDR_TX_BYTES, tx_bytes as u32) | fp(SDIO_HDR_PKT_TYPE, pkt_type as u32);
    v.to_le_bytes()
}

/// The shared first two descriptor dwords (`mt76_connac_mcu.c:3499-3506`).
///
/// `total` is the descriptor length plus the payload — i.e. `skb->len` *after*
/// the descriptor has been pushed, which is what upstream feeds
/// `MT_TXD0_TX_BYTES`.
fn txd_head(total: usize) -> [u32; 2] {
    [
        fp(MT_TXD0_TX_BYTES, total as u32)
            | fp(MT_TXD0_PKT_FMT, MT_TX_TYPE_CMD)
            | fp(MT_TXD0_Q_IDX, MT_TX_MCU_PORT_RX_Q0),
        MT_TXD1_LONG_FORMAT | fp(MT_TXD1_HDR_FORMAT, MT_HDR_FORMAT_CMD),
    ]
}

/// Build the 64-byte legacy descriptor (`mt76_connac_mcu.c:3520-3542`).
///
/// The `set_query` / `ext_cid_ack` / `s2d_index` decisions are upstream's
/// verbatim: a command is "set or query" only if it is EXT or CE; a plain
/// command — which is every firmware-download command — is `MCU_Q_NA`.
pub fn mcu_txd(cmd: McuCmd, seq: u8, payload_len: usize) -> [u8; MCU_TXD_LEN] {
    let mut d = [0u8; MCU_TXD_LEN];
    let total = MCU_TXD_LEN + payload_len;
    let head = txd_head(total);
    d[0..4].copy_from_slice(&head[0].to_le_bytes());
    d[4..8].copy_from_slice(&head[1].to_le_bytes());
    // txd[2..8] stay zero.

    // len = skb->len - sizeof(mcu_txd->txd), and txd[8] is 32 bytes.
    d[32..34].copy_from_slice(&((total - 32) as u16).to_le_bytes());
    d[34..36].copy_from_slice(&MCU_PQ_ID_RX_Q0.to_le_bytes());
    d[36] = cmd.id();
    d[37] = MCU_PKT_ID;

    let ext_cid = cmd.ext_id();
    let (set_query, ext_cid_ack) = if ext_cid != 0 || cmd.is_ce() {
        (
            if cmd.is_query() {
                MCU_Q_QUERY
            } else {
                MCU_Q_SET
            },
            u8::from(ext_cid != 0),
        )
    } else {
        (MCU_Q_NA, 0)
    };
    d[38] = set_query;
    d[39] = seq;
    d[40] = 0; // uc_d2b0_rev
    d[41] = ext_cid;
    d[42] = if cmd.is_wa() {
        MCU_S2D_H2C
    } else {
        MCU_S2D_H2N
    };
    d[43] = ext_cid_ack;
    d
}

/// Build the 48-byte unified descriptor (`mt76_connac_mcu.c:3508-3517`).
pub fn mcu_uni_txd(cmd: McuCmd, seq: u8, payload_len: usize) -> [u8; MCU_UNI_TXD_LEN] {
    let mut d = [0u8; MCU_UNI_TXD_LEN];
    let total = MCU_UNI_TXD_LEN + payload_len;
    let head = txd_head(total);
    d[0..4].copy_from_slice(&head[0].to_le_bytes());
    d[4..8].copy_from_slice(&head[1].to_le_bytes());

    d[32..34].copy_from_slice(&((total - 32) as u16).to_le_bytes());
    d[34..36].copy_from_slice(&(cmd.id() as u16).to_le_bytes());
    d[36] = 0; // rsv
    d[37] = MCU_PKT_ID;
    d[38] = 0; // frag_n
    d[39] = seq;
    d[40..42].copy_from_slice(&0u16.to_le_bytes()); // checksum: 0 = none
    d[42] = MCU_S2D_H2N;
    d[43] = MCU_CMD_UNI_EXT_ACK;
    d
}

/// Round `len` up to a multiple of 4 and append [`USB_TAIL_LEN`] zero bytes,
/// in place (`mt7921/usb.c:53-54`).
fn pad_and_tail(buf: &mut Vec<u8>) {
    let pad = buf.len().next_multiple_of(4) + USB_TAIL_LEN - buf.len();
    buf.resize(buf.len() + pad, 0);
}

/// The complete bulk-OUT frame for one command: USB header, descriptor,
/// payload, pad, tail.
///
/// ★ [`CMD_FW_SCATTER`] is the exception — no descriptor at all
/// (`mt76_connac_mcu.c:3493-3494`). Use [`fw_scatter_frame`] for it; passing it
/// here would produce a frame the firmware rejects.
pub fn command_frame(cmd: McuCmd, seq: u8, payload: &[u8]) -> Vec<u8> {
    let txd_len = if cmd.is_uni() {
        MCU_UNI_TXD_LEN
    } else {
        MCU_TXD_LEN
    };
    let mut buf = Vec::with_capacity(USB_HDR_LEN + txd_len + payload.len() + 8);
    buf.extend_from_slice(&usb_sdio_hdr((txd_len + payload.len()) as u16, 0));
    if cmd.is_uni() {
        buf.extend_from_slice(&mcu_uni_txd(cmd, seq, payload.len()));
    } else {
        buf.extend_from_slice(&mcu_txd(cmd, seq, payload.len()));
    }
    buf.extend_from_slice(payload);
    pad_and_tail(&mut buf);
    buf
}

/// The complete bulk-OUT frame for one firmware chunk: USB header, raw image
/// bytes, pad, tail — and **no descriptor**.
pub fn fw_scatter_frame(chunk: &[u8]) -> Vec<u8> {
    let mut buf = Vec::with_capacity(USB_HDR_LEN + chunk.len() + 8);
    buf.extend_from_slice(&usb_sdio_hdr(chunk.len() as u16, 0));
    buf.extend_from_slice(chunk);
    pad_and_tail(&mut buf);
    buf
}

// ─────────────────────────────────────────────────────────────────────────────
// Response parsing
// ─────────────────────────────────────────────────────────────────────────────

/// `MT_RXD0_LENGTH` (`mt76_connac2_mac.h:185`) — bits 15:0 of the first RX
/// dword are the total transfer length. This is the whole of the "DMA header"
/// on connac2 USB: `MT_DRV_RX_DMA_HDR` makes `mt76u_get_rx_entry_len` read the
/// length straight out of `rxd[0]` with **no separate prefix**
/// (`usb.c:471-473`, `mt7921/usb.c:153`).
const MT_RXD0_LENGTH: u32 = 0x0000_ffff;
/// `MT_RXD0_PKT_TYPE` (`mt76_connac2_mac.h:187`).
const MT_RXD0_PKT_TYPE: u32 = 0xf800_0000;
/// `MT_RXD0_PKT_FLAG` (`mt76_connac2_mac.h:186`).
const MT_RXD0_PKT_FLAG: u32 = 0x000f_0000;

/// `PKT_TYPE_RX_EVENT` (`mt76_connac.h:9-21`) — an MCU response or an
/// unsolicited firmware event.
pub const PKT_TYPE_RX_EVENT: u8 = 7;
/// `PKT_TYPE_NORMAL` — a received 802.11 frame. Named here only so
/// [`Connac2Mcu::wait_event`] can say *why* it is discarding something when the
/// event pipe turns out to be the data pipe.
pub const PKT_TYPE_NORMAL: u8 = 2;

/// One parsed MCU event: the 36-byte `mt76_connac2_mcu_rxd` header plus
/// whatever followed it.
///
/// The raw bytes are kept because the interesting fields differ per command and
/// upstream reads several of them by offset rather than by name — see
/// [`Self::status_u8`].
#[derive(Clone, Debug)]
pub struct McuEvent {
    /// `rxd[0]` bits 31:27 — [`PKT_TYPE_RX_EVENT`] for anything this module
    /// cares about.
    pub pkt_type: u8,
    /// `rxd[0]` bits 19:16. `mt7921_queue_rx_skb` (`mt7921/mac.c:596-600`)
    /// re-labels an event with `flag == 0x1` as `PKT_TYPE_NORMAL_MCU`, i.e. a
    /// received frame that arrived on the MCU path — not a command response.
    pub pkt_flag: u8,
    /// Header `len` at offset 24.
    pub len: u16,
    /// Event id at offset 28.
    pub eid: u8,
    /// Sequence at offset 29 — matched against the one [`Connac2Mcu`] issued.
    pub seq: u8,
    /// Option byte at offset 30.
    pub option: u8,
    /// Extended event id at offset 32 — and, for the patch commands, the
    /// status byte. See [`Self::status_u8`].
    pub ext_eid: u8,
    /// The whole transfer as it came off the wire, header included.
    pub raw: Vec<u8>,
}

impl McuEvent {
    /// The single status byte upstream reads for `MCU_CMD(PATCH_SEM_CONTROL)`
    /// and `MCU_CMD(PATCH_FINISH_REQ)` — **offset 32**, which
    /// `mt7921_mcu_parse_response` reaches by `skb_pull(sizeof(*rxd) - 4)`
    /// (`mt7921/mcu.c:34-37`). Structurally that is the `ext_eid` byte; the
    /// firmware uses the header's last dword as the return value.
    pub fn status_u8(&self) -> u8 {
        self.ext_eid
    }

    /// Everything after the 36-byte header — the payload of an EXT/CE query
    /// (`mt7921/mcu.c:68`, the `else` branch's plain `skb_pull(sizeof(rxd))`).
    pub fn payload(&self) -> &[u8] {
        &self.raw[MCU_RXD_LEN.min(self.raw.len())..]
    }

    /// The `mt76_connac_mcu_uni_event` status (`mt76_connac_mcu.h:1872-1876`):
    /// `u8 cid; u8 pad[3]; __le32 status` at the head of the payload, 0 on
    /// success. `None` if the event is too short to hold one.
    pub fn uni_status(&self) -> Option<(u8, u32)> {
        let p = self.payload();
        if p.len() < 8 {
            return None;
        }
        Some((p[0], u32::from_le_bytes([p[4], p[5], p[6], p[7]])))
    }
}

/// Parse one bulk-IN transfer as an MCU event.
///
/// Rejects anything shorter than the 36-byte header rather than indexing into
/// it — a short read here means the pipe handed back a fragment, and treating
/// a fragment's bytes as a sequence number is exactly how a download appears to
/// succeed and then does nothing.
pub fn parse_event(buf: &[u8]) -> Result<McuEvent, FaceError> {
    if buf.len() < MCU_RXD_LEN {
        return Err(mcu_err(format!(
            "event too short: {} B, need {MCU_RXD_LEN}",
            buf.len()
        )));
    }
    let rxd0 = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
    let dma_len = fg(MT_RXD0_LENGTH, rxd0) as usize;
    // Trust the header's length over the transfer length when it is shorter:
    // the device may have padded the bulk transfer up to a packet boundary.
    let end = if dma_len >= MCU_RXD_LEN && dma_len <= buf.len() {
        dma_len
    } else {
        buf.len()
    };
    Ok(McuEvent {
        pkt_type: fg(MT_RXD0_PKT_TYPE, rxd0) as u8,
        pkt_flag: fg(MT_RXD0_PKT_FLAG, rxd0) as u8,
        len: u16::from_le_bytes([buf[24], buf[25]]),
        eid: buf[28],
        seq: buf[29],
        option: buf[30],
        ext_eid: buf[32],
        raw: buf[..end].to_vec(),
    })
}

// ─────────────────────────────────────────────────────────────────────────────
// The MCU transport
// ─────────────────────────────────────────────────────────────────────────────

/// Command timeout. `mt7921u_mcu_send_message` sets `mcu.timeout = 3 * HZ`
/// (`mt7921/usb.c:45`), overriding the 20 s that
/// `mt76_connac2_mcu_fill_message` had just written (`mt76_connac_mcu.c:3487`).
/// The USB value wins because it is set last; 3 s it is.
pub const MCU_TIMEOUT_MS: u64 = 3_000;

/// Buffer for one response read. `MCU_RESP_URB_SIZE` is 1024 upstream
/// (`mt76.h:674`); 2048 is used here because it is still a whole number of
/// 512-byte high-speed packets and leaves room for an event that arrives
/// concatenated with a firmware log line.
const RESP_BUF_LEN: usize = 2048;

/// Sentinel for "the response endpoint has not been determined yet".
const RESP_EP_UNKNOWN: u8 = 0;

/// The MCU command channel: a sequence counter and a latched response
/// endpoint.
///
/// # Why the response endpoint is discovered rather than assumed
///
/// ★ This is the one genuinely undetermined thing in the file.
/// `mt76u_alloc_mcu_queue` puts the MCU RX queue on `in_ep[MT_EP_IN_CMD_RESP]`
/// = **`0x85`** (`usb.c:721-724`, `mt76.h:646-650`). But
/// `mt792xu_dma_rx_evt_ep4` (`mt792x_usb.c:318-330`) sets
/// `MT_WFDMA_HOST_CONFIG_USB_RXEVT_EP4_EN` during `mt792xu_dma_init`, whose
/// name says RX **events** go to EP**4** = `0x84` — the data pipe — and
/// `mt76u_process_rx_entry` does hand every USB RX buffer to
/// `rx_skb(dev, MT_RXQ_MAIN, …)` (`usb.c:561`), where
/// `mt7921_queue_rx_skb` demuxes `PKT_TYPE_RX_EVENT` out by descriptor type
/// (`mt7921/mac.c:596-613`). Both readings are consistent with the tree and
/// nothing upstream states which pipe actually carries a command response. No
/// register comment, no commit message, no dev_dbg settles it.
///
/// So [`Self::wait_event`] **finds out**: the first wait tries
/// [`Connac2Usb::ep_in_resp`], falls back to [`Connac2Usb::ep_in_data`], and
/// latches whichever answered. Every later command uses the latched pipe, so
/// the cost is one wrong-pipe timeout, once, during bring-up — before any RX
/// pump is running and therefore with nobody to race for the data pipe.
///
/// If the latch lands on `0x84`, the backend **must not** start its RX pump
/// while MCU commands are still in flight: two readers on one bulk pipe split
/// the traffic and each sees half. That is the same hazard the mt76x0 channel
/// -time counters have, and it is just as silent.
#[derive(Debug)]
pub struct Connac2Mcu {
    /// `dev->mcu.msg_seq` (`mt76.h:665`), cycling 1..=15.
    seq: AtomicU8,
    /// The bulk IN endpoint responses were last seen on, or
    /// [`RESP_EP_UNKNOWN`].
    resp_ep: AtomicU8,
}

impl Default for Connac2Mcu {
    fn default() -> Self {
        Self::new()
    }
}

impl Connac2Mcu {
    /// A fresh command channel with nothing latched.
    pub const fn new() -> Self {
        Self {
            seq: AtomicU8::new(0),
            resp_ep: AtomicU8::new(RESP_EP_UNKNOWN),
        }
    }

    /// The next sequence number, exactly as `mt76_connac2_mcu_fill_message`
    /// allocates it (`mt76_connac_mcu.c:3489-3491`): increment, mask to 4 bits,
    /// and if that lands on 0 increment again. So the sequence walks 1..=15 and
    /// **never uses 0** — 0 is what an all-zero or truncated event decodes to,
    /// which is why skipping it matters.
    pub fn next_seq(&self) -> u8 {
        let mut s = self.seq.fetch_add(1, Ordering::SeqCst).wrapping_add(1) & 0xf;
        if s == 0 {
            s = self.seq.fetch_add(1, Ordering::SeqCst).wrapping_add(1) & 0xf;
        }
        s
    }

    /// Which bulk IN endpoint responses are coming from, if it has been
    /// determined. `None` before the first successful wait.
    pub fn response_ep(&self) -> Option<u8> {
        match self.resp_ep.load(Ordering::SeqCst) {
            RESP_EP_UNKNOWN => None,
            ep => Some(ep),
        }
    }

    /// Send one command and optionally wait for its response.
    ///
    /// Returns `Ok(None)` when `wait` is false. `wait` mirrors upstream's
    /// `wait_resp` argument to `mt76_mcu_send_msg` and is **not** a free
    /// choice: a command upstream waits on is one whose response the firmware
    /// actually produces, and waiting on one that produces none costs
    /// [`MCU_TIMEOUT_MS`] every call.
    ///
    /// Cost: one bulk OUT (plus one bulk IN when waiting). Not an EP0 round
    /// trip — nothing on this path is a control transfer.
    pub fn send(
        &self,
        bus: &Connac2Usb,
        cmd: McuCmd,
        payload: &[u8],
        wait: bool,
    ) -> Result<Option<McuEvent>, FaceError> {
        let seq = self.next_seq();
        let (frame, ep) = if cmd == CMD_FW_SCATTER {
            // ★ mt7921/usb.c:47-50 — the download pipe, not the command pipe.
            (fw_scatter_frame(payload), bus.ep_out_data())
        } else {
            (command_frame(cmd, seq, payload), bus.ep_out_cmd())
        };
        bus.bulk_out(ep, &frame)?;
        if !wait {
            return Ok(None);
        }
        self.wait_event(bus, cmd, seq).map(Some)
    }

    /// [`Self::send`] with `wait = true`, returning the event.
    pub fn send_and_get(
        &self,
        bus: &Connac2Usb,
        cmd: McuCmd,
        payload: &[u8],
    ) -> Result<McuEvent, FaceError> {
        self.send(bus, cmd, payload, true)?
            .ok_or_else(|| mcu_err("waited for a response and got none"))
    }

    /// Wait for the event whose `seq` matches, discarding anything else.
    ///
    /// Unsolicited firmware events and (if the latch landed on the data pipe)
    /// received frames both appear here; both are dropped. The loop is bounded
    /// by wall clock rather than by attempt count so a device that streams
    /// events cannot hold the caller past [`MCU_TIMEOUT_MS`].
    ///
    /// [`Connac2Usb::bulk_in`] reports a **timeout as `Ok(0)`** and reserves
    /// `Err` for real bus faults, which is what makes the two cases separable
    /// here: `Ok(0)` just means try again, while an `Err` on the *latched* pipe
    /// is propagated immediately rather than retried until the deadline. During
    /// the one-time endpoint probe an `Err` is instead remembered and the other
    /// candidate is tried, because reading the wrong pipe is allowed to fail.
    fn wait_event(&self, bus: &Connac2Usb, cmd: McuCmd, seq: u8) -> Result<McuEvent, FaceError> {
        let deadline = Instant::now() + Duration::from_millis(MCU_TIMEOUT_MS);
        let mut buf = vec![0u8; RESP_BUF_LEN];
        let mut skipped = 0usize;
        let mut probe_error: Option<FaceError> = None;

        // Endpoints to try, most likely first. Once latched, exactly one — and
        // then an error is a fault, not a wrong guess.
        let latched = self.response_ep();
        let candidates: Vec<u8> = match latched {
            Some(ep) => vec![ep],
            None => vec![bus.ep_in_resp(), bus.ep_in_data()],
        };

        while Instant::now() < deadline {
            for &ep in &candidates {
                let n = match bus.bulk_in(ep, &mut buf) {
                    Ok(n) => n,
                    Err(e) if latched.is_some() => return Err(e),
                    Err(e) => {
                        probe_error = Some(e);
                        continue;
                    }
                };
                // Ok(0) is Connac2Usb's timeout: nothing arrived on this pipe
                // within its own window. Try the next candidate, then round
                // again until our deadline.
                if n == 0 {
                    continue;
                }
                let ev = match parse_event(&buf[..n]) {
                    Ok(ev) => ev,
                    Err(e) => {
                        tracing::debug!(
                            target: "named_radio",
                            ep = format_args!("{ep:#04x}"),
                            n,
                            error = %e,
                            "mt7921u mcu: undecodable transfer while waiting for a response",
                        );
                        continue;
                    }
                };
                if ev.seq == seq {
                    if self.resp_ep.load(Ordering::SeqCst) == RESP_EP_UNKNOWN {
                        self.resp_ep.store(ep, Ordering::SeqCst);
                        tracing::info!(
                            target: "named_radio",
                            radio = "mt7921u",
                            response_ep = format_args!("{ep:#04x}"),
                            data_ep = format_args!("{:#04x}", bus.ep_in_data()),
                            "mt7921u mcu: latched the MCU response endpoint (see Connac2Mcu docs)",
                        );
                    }
                    return Ok(ev);
                }
                skipped += 1;
                tracing::debug!(
                    target: "named_radio",
                    want_seq = seq,
                    got_seq = ev.seq,
                    pkt_type = ev.pkt_type,
                    eid = ev.eid,
                    "mt7921u mcu: discarding an event that is not our response",
                );
            }
        }
        Err(mcu_err(match probe_error {
            Some(e) => format!(
                "command {:#x} (seq {seq}) timed out after {MCU_TIMEOUT_MS} ms; {skipped} other event(s) seen; last endpoint-probe error: {e}",
                cmd.0
            ),
            None => format!(
                "command {:#x} (seq {seq}) timed out after {MCU_TIMEOUT_MS} ms; {skipped} other event(s) seen",
                cmd.0
            ),
        }))
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Firmware blobs
// ─────────────────────────────────────────────────────────────────────────────

/// The ROM patch, `MT7921_ROM_PATCH` (`mt792x.h:55`).
///
/// MEASURED: 92 192 B, build date `20250625153620a\n`, platform `ALPS`,
/// `hw_sw_ver = 0x8a10_8a10`, one section at `0x0090_0000` of 92 032 B starting
/// at file offset 160. Vendored from the node's `/lib/firmware/mediatek/`.
pub const MT7961_PATCH: &[u8] = include_bytes!("../../fw/mt7961/WIFI_MT7961_patch_mcu_1_2_hdr.bin");

/// The WM RAM image, `MT7921_FIRMWARE_WM` (`mt792x.h:47`).
///
/// MEASURED: 791 588 B, `chip_id = 0x0d`, `eco = 0x01`, 5 regions of which 4
/// are downloadable and the fifth is the `NON_DL` CLC blob.
pub const MT7961_RAM: &[u8] = include_bytes!("../../fw/mt7961/WIFI_RAM_CODE_MT7961_1.bin");

/// `mt76_is_sdio(dev) ? 2048 : 4096` (`mt76_connac_mcu.c:3044`) — the largest
/// image chunk per [`CMD_FW_SCATTER`].
pub const FW_CHUNK_MAX: usize = 4096;

// -- download-mode bits ------------------------------------------------------

/// `DL_MODE_ENCRYPT` (`mt76_connac_mcu.h:15`).
const DL_MODE_ENCRYPT: u32 = 1 << 0;
/// `DL_MODE_KEY_IDX` (`:16`).
const DL_MODE_KEY_IDX: u32 = 0x0000_0006;
/// `DL_MODE_RESET_SEC_IV` (`:17`).
const DL_MODE_RESET_SEC_IV: u32 = 1 << 3;
/// `DL_MODE_WORKING_PDA_CR4` (`:18`) — only ever set for a WA image, which this
/// part does not have (`mt792x_core.c:1017` passes `fw_wa = NULL`).
const DL_MODE_WORKING_PDA_CR4: u32 = 1 << 4;
/// `DL_CONFIG_ENCRY_MODE_SEL` (`:20`).
const DL_CONFIG_ENCRY_MODE_SEL: u32 = 1 << 6;
/// `DL_MODE_NEED_RSP` (`:21`) — set on every download this port issues.
const DL_MODE_NEED_RSP: u32 = 1 << 31;

/// `FW_START_OVERRIDE` (`mt76_connac_mcu.h:23`) — start at the address carried
/// by the `FW_FEATURE_OVERRIDE_ADDR` region rather than at the image default.
const FW_START_OVERRIDE: u32 = 1 << 0;

/// `FW_FEATURE_SET_ENCRYPT` (`mt76_connac_mcu.h:9`).
const FW_FEATURE_SET_ENCRYPT: u8 = 1 << 0;
/// `FW_FEATURE_SET_KEY_IDX` (`:10`).
const FW_FEATURE_SET_KEY_IDX: u8 = 0x06;
/// `FW_FEATURE_ENCRY_MODE` (`:11`).
const FW_FEATURE_ENCRY_MODE: u8 = 1 << 4;
/// `FW_FEATURE_OVERRIDE_ADDR` (`:12`) — MEASURED set (`0x20`) on RAM region 0.
const FW_FEATURE_OVERRIDE_ADDR: u8 = 1 << 5;
/// `FW_FEATURE_NON_DL` (`:13`) — MEASURED set (`0x40`) on RAM region 4, the
/// CLC blob.
const FW_FEATURE_NON_DL: u8 = 1 << 6;

/// `FW_TYPE_CLC` (`mt76_connac_mcu.h:44-48`) — the country/regulatory region.
pub const FW_TYPE_CLC: u8 = 2;

/// `PATCH_SEC_TYPE_MASK` / `PATCH_SEC_TYPE_INFO` (`mt76_connac_mcu.h:28-29`).
const PATCH_SEC_TYPE_MASK: u32 = 0x0000_ffff;
/// See [`PATCH_SEC_TYPE_MASK`]. MEASURED: our one section's `type` is
/// `0x0004_0002`, so the low half is `INFO` and the subsystem nibble is 4.
const PATCH_SEC_TYPE_INFO: u32 = 0x2;
/// `PATCH_SEC_NOT_SUPPORT` (`mt76_connac_mcu.h:27`) — all ones in
/// `sec_key_idx` means "this firmware predates the encryption fields".
const PATCH_SEC_NOT_SUPPORT: u32 = 0xffff_ffff;
/// `PATCH_SEC_ENC_TYPE_MASK` (`:31`).
const PATCH_SEC_ENC_TYPE_MASK: u32 = 0xff00_0000;
/// `PATCH_SEC_ENC_TYPE_PLAIN` (`:32`). MEASURED: our section's `sec_key_idx` is
/// 0, so this is the branch taken.
const PATCH_SEC_ENC_TYPE_PLAIN: u32 = 0x00;
/// `PATCH_SEC_ENC_TYPE_AES` (`:33`).
const PATCH_SEC_ENC_TYPE_AES: u32 = 0x01;
/// `PATCH_SEC_ENC_TYPE_SCRAMBLE` (`:34`).
const PATCH_SEC_ENC_TYPE_SCRAMBLE: u32 = 0x02;
/// `PATCH_SEC_ENC_AES_KEY_MASK` (`:36`).
const PATCH_SEC_ENC_AES_KEY_MASK: u32 = 0x0000_00ff;

/// `enum { PATCH_NOT_DL_SEM_FAIL, PATCH_IS_DL, PATCH_NOT_DL_SEM_SUCCESS,
/// PATCH_REL_SEM_SUCCESS }` (`mt76_connac_mcu.h:1128-1133`).
pub const PATCH_NOT_DL_SEM_FAIL: u8 = 0;
/// See [`PATCH_NOT_DL_SEM_FAIL`] — the patch is already resident, so
/// [`load_patch`] returns without sending anything.
pub const PATCH_IS_DL: u8 = 1;
/// See [`PATCH_NOT_DL_SEM_FAIL`] — semaphore acquired, go ahead.
pub const PATCH_NOT_DL_SEM_SUCCESS: u8 = 2;
/// See [`PATCH_NOT_DL_SEM_FAIL`] — semaphore released.
pub const PATCH_REL_SEM_SUCCESS: u8 = 3;

/// `PATCH_SEM_RELEASE` / `PATCH_SEM_GET` (`mt76_connac_mcu.h:1409-1412`).
const PATCH_SEM_RELEASE: u32 = 0;
/// See [`PATCH_SEM_RELEASE`].
const PATCH_SEM_GET: u32 = 1;

/// `MCU_PATCH_ADDRESS` (`mt76_connac_mcu.c:52`) — the connac-v1 patch load
/// address, kept because [`init_download`] tests both it and the connac2 one.
const MCU_PATCH_ADDRESS: u32 = 0x0020_0000;
/// ★ The connac2 patch load address (`mt76_connac_mcu.c:69`). MEASURED to be
/// exactly where our patch section wants to go, which is what routes the
/// preamble to [`CMD_PATCH_START_REQ`].
pub const CONNAC2_PATCH_ADDRESS: u32 = 0x0090_0000;

/// `sizeof(struct mt76_connac2_patch_hdr)` (`mt76_connac_mcu.h:145-160`):
/// 16 + 4 + 4 + 4 + 2 + 2 + (4·5 + 44).
pub const PATCH_HDR_LEN: usize = 96;
/// `sizeof(struct mt76_connac2_patch_sec)` (`mt76_connac_mcu.h:162-176`):
/// 4 + 4 + 4 + 13·4.
pub const PATCH_SEC_LEN: usize = 64;
/// `sizeof(struct mt76_connac2_fw_trailer)` (`mt76_connac_mcu.h:178-188`):
/// 1+1+1+1+1+2 + 10 + 15 + 4.
pub const FW_TRAILER_LEN: usize = 36;
/// `sizeof(struct mt76_connac2_fw_region)` (`mt76_connac_mcu.h:190-200`):
/// 4+4+4+4 + 4+4 + 1+1+14.
pub const FW_REGION_LEN: usize = 40;

/// The ROM patch header. **All multi-byte fields are big endian** — the one
/// place in this driver where that is true, and the reason a naive LE parse
/// reads `n_region` as 16 777 216 and walks off the end.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PatchHeader {
    /// `build_date[16]`. MEASURED `20250625153620a\n`.
    pub build_date: [u8; 16],
    /// `platform[4]`. MEASURED `ALPS`.
    pub platform: [u8; 4],
    /// `hw_sw_ver`, big endian. MEASURED `0x8a10_8a10`; the **high half is the
    /// hardware version** and matches `MT_HW_REV`'s low 16 bits.
    pub hw_sw_ver: u32,
    /// `patch_ver`, big endian. MEASURED `0xffff_ffff`.
    pub patch_ver: u32,
    /// `checksum`, big endian. MEASURED 0.
    pub checksum: u16,
    /// `desc.patch_ver`, big endian. MEASURED `0x4433_2211` — the bytes
    /// `44 33 22 11`, i.e. the little-endian constant `0x1122_3344`. Upstream
    /// never reads it, so whether it is a magic or a version is undetermined;
    /// [`load_patch`] does not check it.
    pub desc_patch_ver: u32,
    /// `desc.subsys`, big endian. MEASURED 4.
    pub subsys: u32,
    /// `desc.feature`, big endian. MEASURED 0.
    pub feature: u32,
    /// `desc.n_region`, big endian. MEASURED 1.
    pub n_region: u32,
    /// `desc.crc`, big endian. MEASURED `0x0000_ffff`.
    pub crc: u32,
}

impl PatchHeader {
    /// The hardware version half of [`Self::hw_sw_ver`] — what is compared
    /// against `MT_HW_REV`.
    pub fn hw_ver(&self) -> u16 {
        (self.hw_sw_ver >> 16) as u16
    }

    /// The software version half of [`Self::hw_sw_ver`].
    pub fn sw_ver(&self) -> u16 {
        self.hw_sw_ver as u16
    }
}

/// One patch section (`mt76_connac2_patch_sec`), again big endian.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PatchSection {
    /// `type`. Low 16 bits must be [`PATCH_SEC_TYPE_INFO`]; upstream refuses
    /// the whole blob otherwise (`mt76_connac_mcu.c:3278-3282`).
    pub sec_type: u32,
    /// `offs` — where the section's bytes start in the file. MEASURED 160,
    /// i.e. immediately after the header and the one section descriptor.
    pub offs: u32,
    /// `size` — the section's byte count as recorded in the outer descriptor.
    pub size: u32,
    /// `info.addr` — the chip address to load at. MEASURED `0x0090_0000`.
    pub addr: u32,
    /// `info.len` — the byte count actually downloaded. MEASURED 92 032, equal
    /// to [`Self::size`].
    pub len: u32,
    /// `info.sec_key_idx` — the encryption descriptor. MEASURED 0 = plain.
    pub sec_key_idx: u32,
    /// `info.align_len`. MEASURED 0. Not used by the download path.
    pub align_len: u32,
}

/// Parse the ROM-patch header and its section table.
pub fn parse_patch(blob: &[u8]) -> Result<(PatchHeader, Vec<PatchSection>), FaceError> {
    if blob.len() < PATCH_HDR_LEN {
        return Err(mcu_err(format!(
            "patch blob is {} B, shorter than its {PATCH_HDR_LEN} B header",
            blob.len()
        )));
    }
    let be32 = |o: usize| u32::from_be_bytes([blob[o], blob[o + 1], blob[o + 2], blob[o + 3]]);
    let mut build_date = [0u8; 16];
    build_date.copy_from_slice(&blob[0..16]);
    let mut platform = [0u8; 4];
    platform.copy_from_slice(&blob[16..20]);

    let hdr = PatchHeader {
        build_date,
        platform,
        hw_sw_ver: be32(20),
        patch_ver: be32(24),
        checksum: u16::from_be_bytes([blob[28], blob[29]]),
        desc_patch_ver: be32(32),
        subsys: be32(36),
        feature: be32(40),
        n_region: be32(44),
        crc: be32(48),
    };

    // A corrupt or wrongly-endian n_region would otherwise index far past the
    // blob; refuse before allocating anything sized from it.
    let n = hdr.n_region as usize;
    let table_end = PATCH_HDR_LEN.saturating_add(n.saturating_mul(PATCH_SEC_LEN));
    if n == 0 || table_end > blob.len() {
        return Err(mcu_err(format!(
            "patch header claims {n} region(s), which does not fit in {} B",
            blob.len()
        )));
    }

    let mut secs = Vec::with_capacity(n);
    for i in 0..n {
        let b = PATCH_HDR_LEN + i * PATCH_SEC_LEN;
        let s = PatchSection {
            sec_type: be32(b),
            offs: be32(b + 4),
            size: be32(b + 8),
            addr: be32(b + 12),
            len: be32(b + 16),
            sec_key_idx: be32(b + 20),
            align_len: be32(b + 24),
        };
        if s.sec_type & PATCH_SEC_TYPE_MASK != PATCH_SEC_TYPE_INFO {
            return Err(mcu_err(format!(
                "patch section {i} type {:#010x} is not PATCH_SEC_TYPE_INFO",
                s.sec_type
            )));
        }
        let end = (s.offs as usize)
            .checked_add(s.len as usize)
            .ok_or_else(|| mcu_err(format!("patch section {i} offset+len overflows")))?;
        if end > blob.len() {
            return Err(mcu_err(format!(
                "patch section {i} spans {}..{end}, past the {} B blob",
                s.offs,
                blob.len()
            )));
        }
        secs.push(s);
    }
    Ok((hdr, secs))
}

/// The RAM image trailer (`mt76_connac2_fw_trailer`), little endian, sitting at
/// the **end** of the file.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FwTrailer {
    /// MEASURED `0x0d`.
    pub chip_id: u8,
    /// MEASURED `0x01`.
    pub eco_code: u8,
    /// MEASURED 5.
    pub n_region: u8,
    /// MEASURED 2.
    pub format_ver: u8,
    /// MEASURED 1.
    pub format_flag: u8,
    /// MEASURED `____010000`.
    pub fw_ver: [u8; 10],
    /// MEASURED `20250625153703\0`.
    pub build_date: [u8; 15],
    /// MEASURED `0x4dae_f689`. Not verified by this port; upstream does not
    /// verify it either, and `mt76_connac_mcu_start_patch` explicitly sends
    /// `check_crc = 0` (`mt76_connac_mcu.c:41-45`).
    pub crc: u32,
}

/// One RAM region, plus the file offset the region walk arrives at.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FwRegion {
    /// `decomp_crc`. MEASURED 0 on every region — this image is not
    /// compressed.
    pub decomp_crc: u32,
    /// `decomp_len`. MEASURED 0.
    pub decomp_len: u32,
    /// `decomp_blk_sz`. MEASURED 0.
    pub decomp_blk_sz: u32,
    /// Chip load address.
    pub addr: u32,
    /// Byte count.
    pub len: u32,
    /// `feature_set` — [`FW_FEATURE_NON_DL`] and [`FW_FEATURE_OVERRIDE_ADDR`]
    /// are the two bits that matter here.
    pub feature_set: u8,
    /// `type` — [`FW_TYPE_CLC`] on the one `NON_DL` region.
    pub kind: u8,
    /// Where this region's bytes start in the file. ★ Computed by summing the
    /// lengths of *all* previous regions, `NON_DL` ones included
    /// (`mt76_connac_mcu.c:3059-3080`: the `goto next` skips the download, not
    /// the `offset += len`).
    pub file_offset: usize,
}

impl FwRegion {
    /// Is this region pushed to the chip at all?
    pub fn downloadable(&self) -> bool {
        self.feature_set & FW_FEATURE_NON_DL == 0
    }

    /// Does this region's address become the `FW_START_REQ` override?
    pub fn is_override(&self) -> bool {
        self.feature_set & FW_FEATURE_OVERRIDE_ADDR != 0
    }
}

/// Parse the RAM image's trailer and walk its region table backwards from it.
///
/// The bounds check (`len > fw_data_len - offset`) is upstream's, but only
/// `mt7921_load_clc` performs it (`mt7921/mcu.c:455-466`);
/// `mt76_connac_mcu_send_ram_firmware` does not, and would happily hand a
/// past-the-end slice to the chunker. It is applied to every region here.
pub fn parse_ram(blob: &[u8]) -> Result<(FwTrailer, Vec<FwRegion>), FaceError> {
    if blob.len() < FW_TRAILER_LEN {
        return Err(mcu_err(format!(
            "ram blob is {} B, shorter than its {FW_TRAILER_LEN} B trailer",
            blob.len()
        )));
    }
    let base = blob.len() - FW_TRAILER_LEN;
    let t = &blob[base..];
    let mut fw_ver = [0u8; 10];
    fw_ver.copy_from_slice(&t[7..17]);
    let mut build_date = [0u8; 15];
    build_date.copy_from_slice(&t[17..32]);
    let trailer = FwTrailer {
        chip_id: t[0],
        eco_code: t[1],
        n_region: t[2],
        format_ver: t[3],
        format_flag: t[4],
        fw_ver,
        build_date,
        crc: u32::from_le_bytes([t[32], t[33], t[34], t[35]]),
    };

    let n = trailer.n_region as usize;
    let table_len = n * FW_REGION_LEN;
    if n == 0 || table_len > base {
        return Err(mcu_err(format!(
            "ram trailer claims {n} region(s), which does not fit in {} B",
            blob.len()
        )));
    }
    let data_len = base - table_len;

    let le32 = |o: usize| u32::from_le_bytes([blob[o], blob[o + 1], blob[o + 2], blob[o + 3]]);
    let mut regions = Vec::with_capacity(n);
    let mut offset = 0usize;
    for i in 0..n {
        // region = hdr - (n_region - i) * sizeof(region)  (mt76_connac_mcu.c:3052-3053)
        let b = base - (n - i) * FW_REGION_LEN;
        let len = le32(b + 20);
        if len as usize > data_len - offset {
            return Err(mcu_err(format!(
                "ram region {i} is {len} B but only {} B remain before the region table",
                data_len - offset
            )));
        }
        regions.push(FwRegion {
            decomp_crc: le32(b),
            decomp_len: le32(b + 4),
            decomp_blk_sz: le32(b + 8),
            addr: le32(b + 16),
            len,
            feature_set: blob[b + 24],
            kind: blob[b + 25],
            file_offset: offset,
        });
        offset += len as usize;
    }
    Ok((trailer, regions))
}

/// `mt76_connac_mcu_gen_dl_mode` (`mt76_connac_mcu.h:1916-1932`) for a
/// connac2 part.
///
/// `is_connac2(dev)` is unconditionally true here (chip `0x7961`,
/// `mt76_connac.h:281-285`), so the `DL_CONFIG_ENCRY_MODE_SEL` clause is not
/// gated. MEASURED: every RAM region has `feature_set` of `0x00`, `0x20` or
/// `0x40`, none of which sets an encryption bit, so this returns exactly
/// `DL_MODE_NEED_RSP` for all four downloadable regions.
pub fn gen_dl_mode(feature_set: u8, is_wa: bool) -> u32 {
    let mut mode = 0u32;
    if feature_set & FW_FEATURE_SET_ENCRYPT != 0 {
        mode |= DL_MODE_ENCRYPT | DL_MODE_RESET_SEC_IV;
    }
    if feature_set & FW_FEATURE_ENCRY_MODE != 0 {
        mode |= DL_CONFIG_ENCRY_MODE_SEL;
    }
    mode |= fp(
        DL_MODE_KEY_IDX,
        fg8(FW_FEATURE_SET_KEY_IDX, feature_set) as u32,
    );
    mode |= DL_MODE_NEED_RSP;
    if is_wa {
        mode |= DL_MODE_WORKING_PDA_CR4;
    }
    mode
}

/// `mt76_connac2_get_data_mode` (`mt76_connac_mcu.c:3242-3268`) — the patch
/// section's download mode, derived from its `sec_key_idx`.
///
/// MEASURED: our section's `sec_key_idx` is 0, i.e. `PLAIN`, so this returns
/// bare [`DL_MODE_NEED_RSP`]. The AES and scramble branches are ported because
/// a re-vendored blob could use them, and are untested for the same reason.
pub fn patch_data_mode(sec_key_idx: u32) -> Result<u32, FaceError> {
    let mut mode = DL_MODE_NEED_RSP;
    if sec_key_idx == PATCH_SEC_NOT_SUPPORT {
        return Ok(mode);
    }
    match fg(PATCH_SEC_ENC_TYPE_MASK, sec_key_idx) {
        PATCH_SEC_ENC_TYPE_PLAIN => {}
        PATCH_SEC_ENC_TYPE_AES => {
            mode |= DL_MODE_ENCRYPT;
            mode |= fp(DL_MODE_KEY_IDX, sec_key_idx & PATCH_SEC_ENC_AES_KEY_MASK) & DL_MODE_KEY_IDX;
            mode |= DL_MODE_RESET_SEC_IV;
        }
        PATCH_SEC_ENC_TYPE_SCRAMBLE => {
            mode |= DL_MODE_ENCRYPT | DL_CONFIG_ENCRY_MODE_SEL | DL_MODE_RESET_SEC_IV;
        }
        other => {
            return Err(mcu_err(format!(
                "patch section encryption type {other:#x} is not supported"
            )));
        }
    }
    Ok(mode)
}

/// Which preamble command a region at `addr` uses
/// (`mt76_connac_mcu.c:68-75`).
///
/// ★ For this part the answer is [`CMD_PATCH_START_REQ`] at
/// [`CONNAC2_PATCH_ADDRESS`] (`0x900000`) — MEASURED to be exactly the patch
/// blob's load address — and [`CMD_TARGET_ADDRESS_LEN_REQ`] everywhere else,
/// which covers all four RAM regions. `MCU_PATCH_ADDRESS` (`0x200000`) is in
/// the test too because the `!is_connac_v1` clause makes it apply here as
/// well, even though nothing in our blobs uses it.
pub fn init_download_cmd(addr: u32) -> McuCmd {
    if addr == MCU_PATCH_ADDRESS || addr == CONNAC2_PATCH_ADDRESS {
        CMD_PATCH_START_REQ
    } else {
        CMD_TARGET_ADDRESS_LEN_REQ
    }
}

/// `mt76_connac_mcu_init_download` (`mt76_connac_mcu.c:54-78`) — announce a
/// region: `{ le32 addr; le32 len; le32 mode; }`.
pub fn init_download(
    mcu: &Connac2Mcu,
    bus: &Connac2Usb,
    addr: u32,
    len: u32,
    mode: u32,
) -> Result<(), FaceError> {
    let mut req = [0u8; 12];
    req[0..4].copy_from_slice(&addr.to_le_bytes());
    req[4..8].copy_from_slice(&len.to_le_bytes());
    req[8..12].copy_from_slice(&mode.to_le_bytes());
    mcu.send_and_get(bus, init_download_cmd(addr), &req)?;
    Ok(())
}

/// `__mt76_mcu_send_firmware` (`mcu.c:142-164`) — chunk `data` into
/// [`FW_CHUNK_MAX`]-byte [`CMD_FW_SCATTER`] frames.
///
/// None of them is waited on (`wait_resp = false` at `mcu.c:150`), which is
/// what makes the download fast enough to be practical: 4 KiB per bulk transfer
/// with no round trip in between.
pub fn send_firmware(mcu: &Connac2Mcu, bus: &Connac2Usb, data: &[u8]) -> Result<(), FaceError> {
    for chunk in data.chunks(FW_CHUNK_MAX) {
        mcu.send(bus, CMD_FW_SCATTER, chunk, false)?;
    }
    Ok(())
}

/// `mt76_connac_mcu_patch_sem_ctrl` (`mt76_connac_mcu.c:24-35`) — take or drop
/// the ROM-patch semaphore. Returns the status byte (see
/// [`McuEvent::status_u8`]).
pub fn patch_sem_ctrl(mcu: &Connac2Mcu, bus: &Connac2Usb, get: bool) -> Result<u8, FaceError> {
    let op = if get {
        PATCH_SEM_GET
    } else {
        PATCH_SEM_RELEASE
    };
    let ev = mcu.send_and_get(bus, CMD_PATCH_SEM_CONTROL, &op.to_le_bytes())?;
    Ok(ev.status_u8())
}

/// `mt76_connac_mcu_start_patch` (`mt76_connac_mcu.c:38-49`) —
/// `{ u8 check_crc; u8 reserved[3]; }` with `check_crc = 0`.
///
/// Upstream disables the CRC check and does not say why; ported as-is.
pub fn start_patch(mcu: &Connac2Mcu, bus: &Connac2Usb) -> Result<u8, FaceError> {
    let ev = mcu.send_and_get(bus, CMD_PATCH_FINISH_REQ, &[0u8; 4])?;
    Ok(ev.status_u8())
}

/// `mt76_connac_mcu_start_firmware` (`mt76_connac_mcu.c:9-21`) —
/// `{ le32 option; le32 addr; }`.
pub fn start_firmware(
    mcu: &Connac2Mcu,
    bus: &Connac2Usb,
    addr: u32,
    option: u32,
) -> Result<(), FaceError> {
    let mut req = [0u8; 8];
    req[0..4].copy_from_slice(&option.to_le_bytes());
    req[4..8].copy_from_slice(&addr.to_le_bytes());
    mcu.send_and_get(bus, CMD_FW_START_REQ, &req)?;
    Ok(())
}

/// `mt76_connac_mcu_restart` (`mt76_connac_mcu.c:2976-2987`) —
/// `MCU_CMD(NIC_POWER_CTRL)` with `power_mode = 1`, **not waited on**.
///
/// This tells a *running* firmware to fall back into download mode. On a cold
/// part — which is the MEASURED state here, `MT_CONN_ON_MISC = 0` — there is
/// nothing listening and the bytes go nowhere. Upstream sends it
/// unconditionally at the head of `mt792x_load_firmware` (`mt792x_core.c:984`)
/// and so does [`run_firmware`].
pub fn restart(mcu: &Connac2Mcu, bus: &Connac2Usb) -> Result<(), FaceError> {
    mcu.send(bus, CMD_NIC_POWER_CTRL, &[1u8, 0, 0, 0], false)?;
    Ok(())
}

/// `mt76_connac2_load_patch` (`mt76_connac_mcu.c:3269-3352`).
///
/// The semaphore is acquired first and released in every exit path, including
/// the failure ones — upstream's `out:` label. A leaked semaphore means the
/// next attempt (and the kernel driver, if it is ever rebound) is told
/// `PATCH_NOT_DL_SEM_FAIL` and cannot patch at all until the chip is
/// power-cycled.
///
/// ★ `hw_rev` is checked against the blob's `hw_sw_ver` high half before
/// anything is sent. MEASURED: blob `0x8a10`, chip `MT_HW_REV = 0x0000_8a10`.
pub fn load_patch(
    mcu: &Connac2Mcu,
    bus: &Connac2Usb,
    blob: &[u8],
    hw_rev: u32,
) -> Result<(), FaceError> {
    let (hdr, secs) = parse_patch(blob)?;
    if hdr.hw_ver() != (hw_rev & 0xffff) as u16 {
        return Err(mcu_err(format!(
            "patch blob is for hw {:#06x} but this chip reports MT_HW_REV {:#010x} (hw {:#06x}) — refusing to download it",
            hdr.hw_ver(),
            hw_rev,
            hw_rev & 0xffff
        )));
    }
    let build_date = String::from_utf8_lossy(&hdr.build_date);
    let platform = String::from_utf8_lossy(&hdr.platform);
    tracing::info!(
        target: "named_radio",
        radio = "mt7921u",
        hw_sw_ver = format_args!("{:#010x}", hdr.hw_sw_ver),
        build_date = %build_date.trim_end(),
        platform = %platform,
        n_region = hdr.n_region,
        "mt7921u mcu: ROM patch",
    );

    let sem = patch_sem_ctrl(mcu, bus, true)?;
    match sem {
        PATCH_IS_DL => {
            tracing::info!(
                target: "named_radio",
                radio = "mt7921u",
                "mt7921u mcu: ROM patch already resident, skipping download",
            );
            return Ok(());
        }
        PATCH_NOT_DL_SEM_SUCCESS => {}
        other => {
            return Err(mcu_err(format!(
                "failed to take the patch semaphore: status {other}"
            )));
        }
    }

    let mut result: Result<(), FaceError> = Ok(());
    for (i, s) in secs.iter().enumerate() {
        let mode = match patch_data_mode(s.sec_key_idx) {
            Ok(m) => m,
            Err(e) => {
                result = Err(e);
                break;
            }
        };
        let body = &blob[s.offs as usize..s.offs as usize + s.len as usize];
        if let Err(e) = init_download(mcu, bus, s.addr, s.len, mode) {
            result = Err(mcu_err(format!(
                "patch section {i} download request failed: {e}"
            )));
            break;
        }
        if let Err(e) = send_firmware(mcu, bus, body) {
            result = Err(mcu_err(format!("patch section {i} transfer failed: {e}")));
            break;
        }
    }

    if result.is_ok() {
        match start_patch(mcu, bus) {
            Ok(_) => {}
            Err(e) => result = Err(mcu_err(format!("PATCH_FINISH_REQ failed: {e}"))),
        }
    }

    // Release in every path (mt76_connac_mcu.c:3338-3348).
    match patch_sem_ctrl(mcu, bus, false) {
        Ok(PATCH_REL_SEM_SUCCESS) => {}
        Ok(other) => {
            let e = mcu_err(format!(
                "failed to release the patch semaphore: status {other}"
            ));
            if result.is_ok() {
                result = Err(e);
            } else {
                tracing::warn!(target: "named_radio", error = %e, "mt7921u mcu");
            }
        }
        Err(e) => {
            if result.is_ok() {
                result = Err(e);
            } else {
                tracing::warn!(target: "named_radio", error = %e, "mt7921u mcu");
            }
        }
    }
    result
}

/// `mt76_connac2_load_ram` + `mt76_connac_mcu_send_ram_firmware`
/// (`mt76_connac_mcu.c:3040-3087,3141-3206`), for the WM image only —
/// `mt792x_core.c:1017` passes `fw_wa = NULL` on this part, so the `is_wa`
/// half of upstream never runs and is not ported.
///
/// ★ MEASURED on our blob: 5 regions, 4 downloaded (702 784 B in 173 chunks)
/// and region 4 skipped as `NON_DL`/CLC; region 0 carries
/// `FW_FEATURE_OVERRIDE_ADDR`, so `FW_START_REQ` goes out with
/// `option = FW_START_OVERRIDE` and `addr = 0x0091_5000`.
pub fn load_ram(mcu: &Connac2Mcu, bus: &Connac2Usb, blob: &[u8]) -> Result<(), FaceError> {
    let (trailer, regions) = parse_ram(blob)?;
    let fw_ver = String::from_utf8_lossy(&trailer.fw_ver);
    let build_date = String::from_utf8_lossy(&trailer.build_date);
    tracing::info!(
        target: "named_radio",
        radio = "mt7921u",
        chip_id = format_args!("{:#04x}", trailer.chip_id),
        eco = format_args!("{:#04x}", trailer.eco_code),
        fw_ver = %fw_ver,
        build_date = %build_date.trim_end_matches('\0'),
        n_region = trailer.n_region,
        "mt7921u mcu: WM RAM image",
    );

    let mut override_addr = 0u32;
    for (i, r) in regions.iter().enumerate() {
        if !r.downloadable() {
            tracing::info!(
                target: "named_radio",
                radio = "mt7921u",
                region = i,
                len = r.len,
                kind = r.kind,
                is_clc = r.kind == FW_TYPE_CLC,
                "mt7921u mcu: skipping NON_DL region (its bytes still advance the file offset)",
            );
            continue;
        }
        if r.is_override() {
            override_addr = r.addr;
        }
        let mode = gen_dl_mode(r.feature_set, false);
        let body = &blob[r.file_offset..r.file_offset + r.len as usize];
        init_download(mcu, bus, r.addr, r.len, mode)
            .map_err(|e| mcu_err(format!("ram region {i} download request failed: {e}")))?;
        send_firmware(mcu, bus, body)
            .map_err(|e| mcu_err(format!("ram region {i} transfer failed: {e}")))?;
    }

    let option = if override_addr != 0 {
        FW_START_OVERRIDE
    } else {
        0
    };
    start_firmware(mcu, bus, override_addr, option)
}

// ─────────────────────────────────────────────────────────────────────────────
// Bring-up
// ─────────────────────────────────────────────────────────────────────────────

/// `____mt76_poll_msec` (`util.c:27-42`) — read `addr` every 10 ms until
/// `(val & mask) == expect` or `timeout_ms` elapses.
///
/// Written here rather than using [`Connac2Usb::poll`] because these timeouts
/// are specified upstream in wall-clock milliseconds (500, 1000, 1500) and, at
/// the MEASURED 268 µs per EP0 round trip, a try-count is not the same thing.
fn poll_msec(
    bus: &Connac2Usb,
    addr: u32,
    mask: u32,
    expect: u32,
    timeout_ms: u64,
) -> Result<bool, FaceError> {
    let deadline = Instant::now() + Duration::from_millis(timeout_ms);
    loop {
        if (bus.rr(addr)? & mask) == expect {
            return Ok(true);
        }
        if Instant::now() >= deadline {
            return Ok(false);
        }
        std::thread::sleep(Duration::from_millis(10));
    }
}

/// `mt792xu_epctl_rst_opt` (`mt792x_usb.c:332-347`) — the USB endpoint
/// reset-option mask: out bulk EPs 4–9 (bits 9:4), in bulk EPs 4–5 (bits
/// 21:20), in interrupt EP 6 (bit 22).
///
/// Upstream calls this with `reset = false` on both the WFSYS-reset path and
/// the DMA-init path, i.e. it only ever *clears* these bits in practice. The
/// `true` direction is ported for completeness and never used here.
fn epctl_rst_opt(bus: &Connac2Usb, reset: bool) -> Result<(), FaceError> {
    const MASK: u32 = 0x0000_03f0 | 0x0070_0000;
    let (clear, set) = if reset { (0, MASK) } else { (MASK, 0) };
    bus.uhw_rmw(MT_SSUSB_EPCTL_CSR_EP_RST_OPT, clear, set)?;
    Ok(())
}

/// `mt792xu_wait_udma_idle` (`mt792x_usb.c:349-364`) — flush RX and wait for
/// both UDMA busy bits to clear, then warn (not fail) if they did not.
fn wait_udma_idle(bus: &Connac2Usb) -> Result<(), FaceError> {
    bus.rmw(MT_UDMA_WLCFG_0, 0, MT_WL_RX_FLUSH)?;
    let mask = MT_WL_RX_BUSY | MT_WL_TX_BUSY;
    if !poll_msec(bus, MT_UDMA_WLCFG_0, mask, 0, 1_000)? {
        let v = bus.rr(MT_UDMA_WLCFG_0)?;
        tracing::warn!(
            target: "named_radio",
            radio = "mt7921u",
            wlcfg0 = format_args!("{v:#010x}"),
            "mt7921u mcu: UDMA still busy before WFSYS reset",
        );
    }
    Ok(())
}

/// `mt792xu_wfsys_reset` (`mt792x_usb.c:425-467`), the `mt7921_wfsys_desc`
/// variant (`:375-382`).
///
/// ★ This is a **subsystem** reset through `MT_CBTOP_RGU_WF_SUBSYS_RST`, not a
/// USB reset. It never touches `libusb`'s `reset()`, which on this rig has
/// three times left a hub port marked `disable=1` and the dongle unrecoverable
/// without a physical replug.
///
/// It is also **not run on a cold plug**: [`power_up`] gates it on
/// `MT_TOP_MISC2_FW_N9_RDY` exactly as `mt7921/usb.c:218-222` does, and the
/// MEASURED `MT_CONN_ON_MISC = 0` means the gate is closed. It matters when
/// the kernel `mt7921u` had the part first, or when this driver is re-opened
/// with its own firmware still running.
pub fn wfsys_reset(bus: &Connac2Usb) -> Result<(), FaceError> {
    wait_udma_idle(bus)?;
    epctl_rst_opt(bus, false)?;

    bus.uhw_rmw(
        MT_CBTOP_RGU_WF_SUBSYS_RST,
        0,
        MT_CBTOP_RGU_WF_SUBSYS_RST_WF_WHOLE_PATH,
    )?;
    // usleep_range(10, 20) upstream; the sleep granularity here is coarser than
    // that and it does not matter — this is a reset assertion width, and longer
    // is safe.
    std::thread::sleep(Duration::from_micros(50));
    bus.uhw_rmw(
        MT_CBTOP_RGU_WF_SUBSYS_RST,
        MT_CBTOP_RGU_WF_SUBSYS_RST_WF_WHOLE_PATH,
        0,
    )?;

    // `need_status_sel` is true for the 7921 descriptor: select bank 0 of the
    // conn-infra status register before reading INIT_DONE out of it.
    bus.uhw_wr(MT_UDMA_CONN_INFRA_STATUS_SEL, 0)?;

    // MT792x_WFSYS_INIT_RETRY_COUNT = 2, msleep(100) between (mt792x.h:43).
    for _ in 0..2 {
        let v = bus.uhw_rr(MT_UDMA_CONN_INFRA_STATUS)?;
        if v & MT_UDMA_CONN_WFSYS_INIT_DONE == MT_UDMA_CONN_WFSYS_INIT_DONE {
            return Ok(());
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    Err(mcu_err("WFSYS reset did not reach INIT_DONE"))
}

/// `mt792xu_mcu_power_on` (`mt792x_usb.c:214-232`) — one `MT_VEND_POWER_ON`
/// vendor write with no data stage, then poll `MT_CONN_ON_MISC` for
/// `FW_PWR_ON` for 500 ms.
///
/// MEASURED before this runs: `MT_CONN_ON_MISC = 0x0000_0000`, i.e. `FW_PWR_ON`
/// clear. That is the observation that says the part is genuinely off and this
/// call is not a no-op.
pub fn power_on(bus: &Connac2Usb) -> Result<(), FaceError> {
    bus.power_on()?;
    if !poll_msec(
        bus,
        MT_CONN_ON_MISC,
        MT_TOP_MISC2_FW_PWR_ON,
        MT_TOP_MISC2_FW_PWR_ON,
        500,
    )? {
        return Err(mcu_err(
            "timeout waiting for MT_TOP_MISC2_FW_PWR_ON after MT_VEND_POWER_ON",
        ));
    }
    Ok(())
}

/// What [`power_up`] found on the chip, so a caller can log it rather than
/// re-reading two registers at 268 µs each.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ChipId {
    /// `MT_HW_CHIPID`. MEASURED `0x0000_7961`.
    pub chip_id: u32,
    /// `MT_HW_REV`. MEASURED `0x0000_8a10` — and its low 16 bits are what
    /// [`load_patch`] matches the blob against.
    pub hw_rev: u32,
    /// Was firmware already running when we arrived (`FW_N9_RDY` set)? If so,
    /// [`power_up`] ran [`wfsys_reset`] first.
    pub was_running: bool,
}

/// The MT7961 chip id (`mt7921/usb.c:16`, and the `is_connac2` test at
/// `mt76_connac.h:281`).
pub const MT7961_CHIP_ID: u32 = 0x7961;

/// Bring the chip to the point where firmware can be downloaded —
/// `mt7921u_probe`'s pre-registration half (`mt7921/usb.c:214-226`).
///
/// ★ **Ordering matters and is easy to get backwards.** Upstream does
/// `wfsys_reset` (conditionally) and *then* `power_on`. Reversing it drops
/// `FW_PWR_ON` — the reset clears it — with no second power-on to restore it,
/// and every later register access reads a chip that is off.
///
/// Cost: 3–4 EP0 round trips plus, on the reset path, a subsystem reset that
/// can take up to ~200 ms.
pub fn power_up(bus: &Connac2Usb) -> Result<ChipId, FaceError> {
    let chip_id = bus.rr(MT_HW_CHIPID)?;
    let hw_rev = bus.rr(MT_HW_REV)?;
    if chip_id & 0xffff != MT7961_CHIP_ID {
        return Err(mcu_err(format!(
            "MT_HW_CHIPID is {chip_id:#010x}, not an MT7961 ({MT7961_CHIP_ID:#06x})"
        )));
    }

    let misc = bus.rr(MT_CONN_ON_MISC)?;
    let was_running = misc & MT_TOP_MISC2_FW_N9_RDY != 0;
    tracing::info!(
        target: "named_radio",
        radio = "mt7921u",
        chip_id = format_args!("{chip_id:#010x}"),
        hw_rev = format_args!("{hw_rev:#010x}"),
        conn_on_misc = format_args!("{misc:#010x}"),
        was_running,
        "mt7921u mcu: chip identity as found",
    );

    if was_running {
        wfsys_reset(bus)?;
    }
    power_on(bus)?;

    Ok(ChipId {
        chip_id,
        hw_rev,
        was_running,
    })
}

/// Download and start the firmware — `mt7921u_mcu_init`
/// (`mt7921/usb.c:63-85`) wrapped around `mt792x_load_firmware`
/// (`mt792x_core.c:980-1036`), minus the parts this driver does not use.
///
/// The sequence, with upstream lines:
///
/// 1. `MT_SWDEF_MODE = MT_SWDEF_NORMAL_MODE` — *"force firmware operation mode
///    into normal state, which should be set before firmware download stage"*
///    (`mt7921/init.c:88-91`).
/// 2. `MT_UDMA_TX_QSEL |= MT_FW_DL_EN` (`mt7921/usb.c:76`). ★ This is what makes
///    `MT_EP_OUT_AC_BE` carry firmware chunks instead of frames. Without it the
///    [`CMD_FW_SCATTER`] transfers are accepted by USB and discarded by the
///    chip.
/// 3. [`restart`] — `NIC_POWER_CTRL`, unwaited (`mt792x_core.c:984`).
/// 4. Poll `MT_CONN_ON_MISC & MT_TOP_MISC_FW_STATE == MT_TOP_MISC2_FW_PWR_ON`
///    for 1 s (`mt792x_core.c:986-989`). ★ Note the mask and the value come
///    from two different register namespaces (`MISC` vs `MISC2`) — that is
///    upstream's expression, not a transcription slip, and it only warns.
/// 5. [`load_patch`], then [`load_ram`] (`:1000`, `:1017`).
/// 6. Poll `MT_TOP_MISC2_FW_N9_RDY` for 1.5 s; this one is a hard error
///    (`:1021-1025`).
/// 7. `MT_UDMA_TX_QSEL &= ~MT_FW_DL_EN` (`mt7921/usb.c:82`).
///
/// **Not done here**, and each is a deliberate omission with its reason on the
/// module header: `mt7921_mcu_get_nic_capability`, `mt7921_load_clc`,
/// `mt7921_mcu_fw_log_2_host`.
///
/// ⚠ `mt792xu_dma_init` (`mt792x_usb.c:393-422`) runs **between** [`power_up`]
/// and this call upstream, and is not implemented anywhere in this port. Until
/// it is, `MT_UDMA_WLCFG_0`'s RX/TX enables are whatever the last driver left,
/// and this download's outcome is not a clean experiment.
pub fn run_firmware(
    mcu: &Connac2Mcu,
    bus: &Connac2Usb,
    hw_rev: u32,
    patch: &[u8],
    ram: &[u8],
) -> Result<(), FaceError> {
    bus.wr(MT_SWDEF_MODE, MT_SWDEF_NORMAL_MODE)?;
    bus.rmw(MT_UDMA_TX_QSEL, 0, MT_FW_DL_EN)?;

    let result = (|| -> Result<(), FaceError> {
        restart(mcu, bus)?;
        if !poll_msec(
            bus,
            MT_CONN_ON_MISC,
            MT_TOP_MISC_FW_STATE,
            MT_TOP_MISC2_FW_PWR_ON,
            1_000,
        )? {
            tracing::warn!(
                target: "named_radio",
                radio = "mt7921u",
                "mt7921u mcu: MCU is not reporting ready for firmware download; continuing anyway (upstream does the same)",
            );
        }

        load_patch(mcu, bus, patch, hw_rev)?;
        load_ram(mcu, bus, ram)?;

        if !poll_msec(
            bus,
            MT_CONN_ON_MISC,
            MT_TOP_MISC2_FW_N9_RDY,
            MT_TOP_MISC2_FW_N9_RDY,
            1_500,
        )? {
            return Err(mcu_err(
                "timeout waiting for MT_TOP_MISC2_FW_N9_RDY after FW_START_REQ",
            ));
        }
        Ok(())
    })();

    // Leave the download path disabled either way: with MT_FW_DL_EN still set,
    // AC_BE stays wired to the firmware queue and no data frame can ever be
    // transmitted.
    let clear = bus.rmw(MT_UDMA_TX_QSEL, MT_FW_DL_EN, 0);
    result.and(clear.map(|_| ()))
}

// ─────────────────────────────────────────────────────────────────────────────
// The command subset for monitor RX + raw inject
// ─────────────────────────────────────────────────────────────────────────────

/// `mt76_connac_mcu_set_mac_enable` (`mt76_connac_mcu.c:216-231`) —
/// `MCU_EXT_CMD(MAC_INIT_CTRL)` with `{ u8 enable; u8 band; u8 rsv[2]; }`.
///
/// The `hdr_trans` argument upstream takes is **ignored by upstream itself**:
/// the parameter is accepted and never written into the request. Ported
/// without it rather than carrying a dead argument.
pub fn set_mac_enable(
    mcu: &Connac2Mcu,
    bus: &Connac2Usb,
    band: u8,
    enable: bool,
) -> Result<(), FaceError> {
    let req = [u8::from(enable), band, 0, 0];
    mcu.send_and_get(bus, EXT_CMD_MAC_INIT_CTRL, &req)?;
    Ok(())
}

/// `MT7921_FILTER_ENABLE` — bit 31 of the `fif` word
/// (`mt7921/main.c:671-674`). Without it the firmware ignores the word.
pub const RX_FILTER_ENABLE: u32 = 1 << 31;
/// `MT7921_FILTER_FCSFAIL` — pass frames whose FCS failed.
pub const RX_FILTER_FCSFAIL: u32 = 1 << 2;
/// `MT7921_FILTER_CONTROL` — pass control frames.
pub const RX_FILTER_CONTROL: u32 = 1 << 5;
/// `MT7921_FILTER_OTHER_BSS` — pass frames not addressed to our BSS. This is
/// the bit that makes the part a monitor rather than a station.
pub const RX_FILTER_OTHER_BSS: u32 = 1 << 6;

/// Everything through: the `fif` word for a **sensor**, FCS failures included.
///
/// ★ A CRC-failed frame is still evidence the medium was occupied, which is
/// exactly what an occupancy sensor wants — but it is a trap for a decoder.
/// The LR2021 testbed spent a whole campaign on results that turned out to be
/// CRC-failing frames. If the consumer parses payloads, use
/// [`RX_FILTER_PROMISCUOUS_VALID`].
pub const RX_FILTER_PROMISCUOUS: u32 =
    RX_FILTER_ENABLE | RX_FILTER_FCSFAIL | RX_FILTER_CONTROL | RX_FILTER_OTHER_BSS;

/// Promiscuous, but drop frames that failed FCS — the setting for a consumer
/// that will parse what it receives.
pub const RX_FILTER_PROMISCUOUS_VALID: u32 =
    RX_FILTER_ENABLE | RX_FILTER_CONTROL | RX_FILTER_OTHER_BSS;

/// `MT7921_FIF_BIT_SET` (`mt7921/mcu.c:1098`) — OR `bit_map` into the
/// firmware's `MT_WF_RFCR` shadow.
pub const FIF_BIT_SET: u8 = 1 << 0;
/// `MT7921_FIF_BIT_CLR` (`mt7921/mcu.c:1097`) — AND-NOT it out.
pub const FIF_BIT_CLR: u8 = 1 << 1;

/// `mt7921_mcu_set_rxfilter` (`mt7921/mcu.c:1477-1497`) — 68-byte request:
/// `{ u8 rsv[4]; u8 mode; u8 rsv2[3]; le32 fif; le32 bit_map; u8 bit_op;
/// u8 pad[51]; }`, `mode = fif ? 1 : 2`, sent **unwaited**.
///
/// # Why the MCU path and not the register path
///
/// `MT_WF_RFCR` (`mt792x_regs.h:223`) is a real register at `0x820e5000` and
/// `MT_VEND_WRITE_EXT` can reach it, so writing the drop bits directly looks
/// tempting — one EP0 write instead of a bulk command. It is not taken here
/// for one reason: **the firmware owns that register.** `mt7921_mcu_set_rxfilter`
/// exists precisely because the RFCR shadow lives in firmware, which rewrites
/// the register on every channel switch, sniffer enable and BSS update
/// (`mt7921/mcu.c:1106-1123` reaches for the same command just to flip
/// `DROP_OTHER_BEACON`). A host-side write would be silently reverted at the
/// next firmware event, and the failure mode — RX that works until you retune —
/// is the worst kind. The register path would be correct only on a part with no
/// firmware running, which is not this one.
///
/// Two shapes, both reachable here:
///   * `fif != 0` — mode 1, "here is the whole filter word".
///   * `fif == 0` — mode 2, "set or clear these `MT_WF_RFCR` bits", which is
///     how upstream toggles individual drops.
pub fn set_rx_filter(
    mcu: &Connac2Mcu,
    bus: &Connac2Usb,
    fif: u32,
    bit_op: u8,
    bit_map: u32,
) -> Result<(), FaceError> {
    let mut req = [0u8; 68];
    req[4] = if fif != 0 { 1 } else { 2 };
    req[8..12].copy_from_slice(&fif.to_le_bytes());
    req[12..16].copy_from_slice(&bit_map.to_le_bytes());
    req[16] = bit_op;
    // wait_resp = false upstream (mt7921/mcu.c:1496).
    mcu.send(bus, CE_CMD_SET_RX_FILTER, &req, false)?;
    Ok(())
}

/// `CMD_CBW_*` (`mt76_connac.h:56-64`), which are the `IEEE80211_STA_RX_BW_*`
/// values for the first four: 20 MHz.
pub const CMD_CBW_20MHZ: u8 = 0;
/// 40 MHz.
pub const CMD_CBW_40MHZ: u8 = 1;
/// 80 MHz.
pub const CMD_CBW_80MHZ: u8 = 2;
/// 160 MHz.
pub const CMD_CBW_160MHZ: u8 = 3;
/// 10 MHz.
pub const CMD_CBW_10MHZ: u8 = 4;
/// 5 MHz.
pub const CMD_CBW_5MHZ: u8 = 5;
/// 80+80 MHz.
pub const CMD_CBW_8080MHZ: u8 = 6;

/// `CH_SWITCH_NORMAL` (`mt76_connac_mcu.h:1150-1158`) — the reason a monitor
/// vif always uses (`mt7921/mcu.c:919-921`).
pub const CH_SWITCH_NORMAL: u8 = 0;
/// `CH_SWITCH_SCAN_BYPASS_DPD` — off-channel, skip digital pre-distortion.
pub const CH_SWITCH_SCAN_BYPASS_DPD: u8 = 9;

/// Band selector for [`ChannelReq::channel_band`]: 2.4 GHz.
pub const CHANNEL_BAND_2G: u8 = 0;
/// 5 GHz.
pub const CHANNEL_BAND_5G: u8 = 1;
/// 6 GHz. ★ `mt7921_mcu_set_chan_info` maps `NL80211_BAND_6GHZ` (which is 3)
/// to **2** here (`mt7921/mcu.c:914-917`); the firmware's numbering is not
/// nl80211's.
pub const CHANNEL_BAND_6G: u8 = 2;

/// The 76-byte `MCU_EXT_CMD(CHANNEL_SWITCH)` / `MCU_EXT_CMD(SET_RX_PATH)`
/// request (`mt7921_mcu_set_chan_info`, `mt7921/mcu.c:883-940`).
///
/// Field order, since the trailing reserve makes the size easy to get wrong:
/// `control_ch, center_ch, bw, tx_streams_num, rx_streams, switch_reason,
/// band_idx, center_ch2` (8 B), `le16 cac_case`, `channel_band, rsv0`,
/// `le32 outband_freq`, `txpower_drop, ap_bw, ap_center_ch`, `rsv1[57]`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ChannelReq {
    /// The control channel number (`chandef->chan->hw_value`).
    pub control_ch: u8,
    /// The centre channel number of the operating width.
    pub center_ch: u8,
    /// One of the `CMD_CBW_*` constants.
    pub bw: u8,
    /// Number of TX chains. 2 on this 2×2 part when both are wanted.
    pub tx_streams: u8,
    /// The RX antenna **mask** — [`Self::encode`] converts it to a count for
    /// `CHANNEL_SWITCH`.
    pub rx_streams_mask: u8,
    /// One of the `CH_SWITCH_*` constants.
    pub switch_reason: u8,
    /// Band index; 0 for the only band on this part.
    pub band_idx: u8,
    /// Second centre channel, 80+80 only.
    pub center_ch2: u8,
    /// One of the `CHANNEL_BAND_*` constants.
    pub channel_band: u8,
}

impl ChannelReq {
    /// Encode for `cmd`.
    ///
    /// ★ The one command-dependent field: `MCU_EXT_CMD(CHANNEL_SWITCH)` wants
    /// `rx_streams` as a **count**, `MCU_EXT_CMD(SET_RX_PATH)` wants it as a
    /// **mask** (`mt7921/mcu.c:930-931` replaces the mask with its popcount for
    /// `CHANNEL_SWITCH` only). Two commands, one struct, one field with two
    /// meanings — exactly the kind of thing that produces a radio that tunes
    /// but only receives on one chain.
    pub fn encode(&self, cmd: McuCmd) -> [u8; 76] {
        let mut r = [0u8; 76];
        r[0] = self.control_ch;
        r[1] = self.center_ch;
        r[2] = self.bw;
        r[3] = self.tx_streams;
        r[4] = if cmd == EXT_CMD_CHANNEL_SWITCH {
            self.rx_streams_mask.count_ones() as u8
        } else {
            self.rx_streams_mask
        };
        r[5] = self.switch_reason;
        r[6] = self.band_idx;
        r[7] = self.center_ch2;
        // r[8..10]  cac_case      = 0
        r[10] = self.channel_band;
        // r[11]     rsv0          = 0
        // r[12..16] outband_freq  = 0
        // r[16]     txpower_drop  = 0
        // r[17]     ap_bw         = 0
        // r[18]     ap_center_ch  = 0
        // r[19..76] rsv1[57]      = 0
        r
    }
}

/// Send a [`ChannelReq`] as `MCU_EXT_CMD(CHANNEL_SWITCH)`
/// (`mt7921_set_channel`, `mt7921/main.c:477-499`).
///
/// This is the **PHY** retune and it is issued for every vif type, monitor
/// included. It is not sufficient on its own for monitor mode: the MAC also
/// needs [`config_sniffer`], which carries its own copy of the channel.
pub fn set_channel(mcu: &Connac2Mcu, bus: &Connac2Usb, req: &ChannelReq) -> Result<(), FaceError> {
    mcu.send_and_get(
        bus,
        EXT_CMD_CHANNEL_SWITCH,
        &req.encode(EXT_CMD_CHANNEL_SWITCH),
    )?;
    Ok(())
}

/// `mt7921_mcu_set_sniffer` (`mt7921/mcu.c:1151-1178`) — `MCU_UNI_CMD(SNIFFER)`
/// with `{ u8 band_idx; u8 pad[3]; }` then TLV tag 0
/// `{ le16 tag; le16 len; u8 enable; u8 pad[3]; }`.
///
/// This is the actual monitor-mode switch: mac80211 calls it from
/// `mt7921_config` on `IEEE80211_CONF_CHANGE_MONITOR` (`mt7921/main.c:610`).
pub fn set_sniffer(
    mcu: &Connac2Mcu,
    bus: &Connac2Usb,
    band_idx: u8,
    enable: bool,
) -> Result<(), FaceError> {
    let mut req = [0u8; 12];
    req[0] = band_idx;
    req[4..6].copy_from_slice(&0u16.to_le_bytes()); // tag 0
    req[6..8].copy_from_slice(&8u16.to_le_bytes()); // sizeof(sniffer_enable_tlv)
    req[8] = u8::from(enable);
    mcu.send_and_get(bus, UNI_CMD_SNIFFER, &req)?;
    Ok(())
}

/// The channel description a sniffer needs — the `ch_width` and `ch_band`
/// encodings here are **not** the `CMD_CBW_*` ones.
///
/// `mt7921_mcu_config_sniffer` (`mt7921/mcu.c:1181-1247`) maps
/// `NL80211_CHAN_WIDTH_{20_NOHT,20,40} → 0`, `80 → 1`, `160 → 2`,
/// `80P80 → 3`, `5 → 4`, `10 → 5`, `320 → 6`, and the bands
/// `2GHZ → 1, 5GHZ → 2, 6GHZ → 3`. Both differ from every other width/band
/// encoding in this file; that is upstream's, and the reason is undetermined.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SnifferChan {
    /// Band index; 0 here.
    pub band_idx: u8,
    /// 1 = 2.4 GHz, 2 = 5 GHz, 3 = 6 GHz.
    pub ch_band: u8,
    /// 0 = 20/40 MHz, 1 = 80, 2 = 160, 3 = 80+80, 4 = 5, 5 = 10, 6 = 320.
    pub bw: u8,
    /// Control channel number.
    pub control_ch: u8,
    /// Centre channel number.
    pub center_ch: u8,
    /// Second centre channel, 80+80 only; 0 otherwise.
    pub center_ch2: u8,
    /// ★ Upstream hardcodes `drop_err = 1` (`mt7921/mcu.c:1255`), i.e. the
    /// sniffer drops errored frames. Exposed rather than hardcoded because a
    /// *sensor* wants the opposite — see [`RX_FILTER_PROMISCUOUS`] for the same
    /// argument on the filter word. Setting it to 0 is a departure from
    /// upstream and is UNVALIDATED.
    pub drop_err: u8,
}

impl SnifferChan {
    /// Encode the 20-byte request: 4-byte header then TLV tag 1 of 16 bytes.
    ///
    /// `sco` (secondary-channel offset) is derived, not supplied: 1 (SCA) when
    /// the control channel is below the centre, 3 (SCB) when above, 0 when they
    /// are equal (`mt7921/mcu.c:1266-1269`).
    pub fn encode(&self) -> [u8; 20] {
        let mut r = [0u8; 20];
        r[0] = self.band_idx;
        r[4..6].copy_from_slice(&1u16.to_le_bytes()); // tag 1
        r[6..8].copy_from_slice(&16u16.to_le_bytes()); // sizeof(config_tlv)
        // r[8..10] aid = 0
        r[10] = self.ch_band;
        r[11] = self.bw;
        r[12] = self.control_ch;
        r[13] = match self.control_ch.cmp(&self.center_ch) {
            std::cmp::Ordering::Less => 1,    // SCA
            std::cmp::Ordering::Greater => 3, // SCB
            std::cmp::Ordering::Equal => 0,
        };
        r[14] = self.center_ch;
        r[15] = self.center_ch2;
        r[16] = self.drop_err;
        // r[17..20] pad
        r
    }
}

/// `mt7921_mcu_config_sniffer` (`mt7921/mcu.c:1181-1247`).
pub fn config_sniffer(
    mcu: &Connac2Mcu,
    bus: &Connac2Usb,
    chan: &SnifferChan,
) -> Result<(), FaceError> {
    mcu.send_and_get(bus, UNI_CMD_SNIFFER, &chan.encode())?;
    Ok(())
}

/// `EE_MODE_EFUSE` / `EE_MODE_BUFFER` (`mt76_connac_mcu.h:1189-1192`).
const EE_MODE_EFUSE: u8 = 0;
/// `EE_FORMAT_WHOLE` (`mt76_connac_mcu.h:1194-1198`).
const EE_FORMAT_WHOLE: u8 = 1;

/// `mt7921_mcu_set_eeprom` (`mt7921/mcu.c:942-956`) —
/// `MCU_EXT_CMD(EFUSE_BUFFER_MODE)` with
/// `{ u8 buffer_mode; u8 format; le16 len; }` = `{ EFUSE, WHOLE, 0 }`.
///
/// **This one is needed.** It tells the firmware to take its calibration and
/// per-channel power tables from the on-chip efuse rather than waiting for the
/// host to push an EEPROM image. `mt7921u_mac_reset` sends it immediately after
/// every firmware load (`mt7921/usb.c:131`), and `__mt7921_init_hardware`
/// sends it before `mt7921_mac_init` (`mt7921/init.c:100`). Skip it and the
/// PHY runs uncalibrated — which on the neighbouring Realtek parts is exactly
/// the "TX works, decode is marginal" failure that cost this bench weeks.
pub fn set_eeprom_efuse_mode(mcu: &Connac2Mcu, bus: &Connac2Usb) -> Result<(), FaceError> {
    let req = [EE_MODE_EFUSE, EE_FORMAT_WHOLE, 0, 0];
    mcu.send_and_get(bus, EXT_CMD_EFUSE_BUFFER_MODE, &req)?;
    Ok(())
}

/// `MT7921_EEPROM_BLOCK_SIZE` (`mt7921/mt7921.h:26`).
pub const EEPROM_BLOCK_SIZE: usize = 16;
/// `MT_EE_MAC_ADDR` (`mt7921/mt7921.h:162`) — where the 6-byte MAC lives in
/// the efuse image.
pub const MT_EE_MAC_ADDR: u32 = 0x004;

/// `mt7921_mcu_read_eeprom` (`mt7921/mcu.c:75-94`) — read the 16-byte efuse
/// block containing `offset`, via `MCU_EXT_QUERY(EFUSE_ACCESS)`.
///
/// Request is `{ le32 addr; }` with `addr` rounded **down** to a block
/// boundary; the response payload is
/// `struct mt7921_mcu_eeprom_info { le32 addr; le32 valid; u8 data[16]; }`
/// (`mt7921/mcu.h:41-45`) starting at [`MCU_RXD_LEN`].
///
/// Needed only if the backend wants the part's MAC address — which it does, to
/// stamp an injected frame's address 2 with something the world will not
/// mistake for another node. Everything else the efuse holds is consumed by
/// firmware, not by us.
pub fn read_efuse_block(
    mcu: &Connac2Mcu,
    bus: &Connac2Usb,
    offset: u32,
) -> Result<(u32, [u8; EEPROM_BLOCK_SIZE]), FaceError> {
    let base = offset & !(EEPROM_BLOCK_SIZE as u32 - 1);
    let ev = mcu.send_and_get(bus, EXT_QUERY_EFUSE_ACCESS, &base.to_le_bytes())?;
    let p = ev.payload();
    if p.len() < 8 + EEPROM_BLOCK_SIZE {
        return Err(mcu_err(format!(
            "efuse response payload is {} B, need {}",
            p.len(),
            8 + EEPROM_BLOCK_SIZE
        )));
    }
    let valid = u32::from_le_bytes([p[4], p[5], p[6], p[7]]);
    let mut data = [0u8; EEPROM_BLOCK_SIZE];
    data.copy_from_slice(&p[8..8 + EEPROM_BLOCK_SIZE]);
    Ok((valid, data))
}

/// Read the part's MAC address out of the efuse ([`MT_EE_MAC_ADDR`]).
pub fn read_mac_addr(mcu: &Connac2Mcu, bus: &Connac2Usb) -> Result<[u8; 6], FaceError> {
    let (_, block) = read_efuse_block(mcu, bus, MT_EE_MAC_ADDR)?;
    let base = (MT_EE_MAC_ADDR as usize) % EEPROM_BLOCK_SIZE;
    let mut mac = [0u8; 6];
    mac.copy_from_slice(&block[base..base + 6]);
    Ok(mac)
}

#[cfg(test)]
mod tests {
    use super::*;

    // ── command word encoding ────────────────────────────────────────────

    /// The packed command word is the protocol; if these drift, every
    /// descriptor built from them is wrong in a way that only shows up as a
    /// firmware that ignores us.
    #[test]
    fn command_words_match_the_upstream_macros() {
        // MCU_CMD(FW_SCATTER) = FIELD_PREP(ID, 0xee)
        assert_eq!(CMD_FW_SCATTER.0, 0x0000_00ee);
        assert_eq!(CMD_PATCH_START_REQ.0, 0x0000_0005);
        assert_eq!(CMD_TARGET_ADDRESS_LEN_REQ.0, 0x0000_0001);
        // MCU_EXT_CMD(CHANNEL_SWITCH) = MCU_CMD(EXT_CID) | FIELD_PREP(EXT_ID, 0x08)
        assert_eq!(EXT_CMD_CHANNEL_SWITCH.0, 0x0000_08ed);
        assert_eq!(EXT_CMD_CHANNEL_SWITCH.id(), 0xed);
        assert_eq!(EXT_CMD_CHANNEL_SWITCH.ext_id(), 0x08);
        // MCU_EXT_QUERY(EFUSE_ACCESS) adds BIT(16)
        assert_eq!(EXT_QUERY_EFUSE_ACCESS.0, 0x0001_01ed);
        assert!(EXT_QUERY_EFUSE_ACCESS.is_query());
        // MCU_CE_CMD(SET_RX_FILTER) = BIT(18) | 0x0a
        assert_eq!(CE_CMD_SET_RX_FILTER.0, 0x0004_000a);
        assert!(CE_CMD_SET_RX_FILTER.is_ce());
        // MCU_UNI_CMD(SNIFFER) = BIT(17) | 0x24
        assert_eq!(UNI_CMD_SNIFFER.0, 0x0002_0024);
        assert!(UNI_CMD_SNIFFER.is_uni());
        assert!(!UNI_CMD_SNIFFER.is_ce());
    }

    // ── descriptor layout ────────────────────────────────────────────────

    /// The 64-byte legacy descriptor, byte for byte, for an EXT command.
    #[test]
    fn mcu_txd_layout_for_an_ext_command() {
        let d = mcu_txd(EXT_CMD_CHANNEL_SWITCH, 7, 76);
        assert_eq!(d.len(), 64);
        // txd[0]: Q_IDX 0x20 << 25 | PKT_FMT 2 << 23 | TX_BYTES (64+76)
        assert_eq!(
            u32::from_le_bytes([d[0], d[1], d[2], d[3]]),
            0x4000_0000 | 0x0100_0000 | 140
        );
        // txd[1]: LONG_FORMAT | HDR_FORMAT_CMD
        assert_eq!(u32::from_le_bytes([d[4], d[5], d[6], d[7]]), 0x8001_0000);
        assert!(d[8..32].iter().all(|&b| b == 0), "txd[2..8] must be zero");
        // len = skb->len - sizeof(txd[8]) = 140 - 32
        assert_eq!(u16::from_le_bytes([d[32], d[33]]), 108);
        assert_eq!(u16::from_le_bytes([d[34], d[35]]), 0x8000, "pq_id");
        assert_eq!(d[36], 0xed, "cid = EXT_CID");
        assert_eq!(d[37], MCU_PKT_ID);
        assert_eq!(d[38], MCU_Q_SET, "EXT + !query => MCU_Q_SET");
        assert_eq!(d[39], 7, "seq");
        assert_eq!(d[40], 0);
        assert_eq!(d[41], 0x08, "ext_cid");
        assert_eq!(d[42], MCU_S2D_H2N);
        assert_eq!(d[43], 1, "ext_cid_ack is set because ext_cid != 0");
        assert!(d[44..].iter().all(|&b| b == 0), "rsv[5] must be zero");
    }

    /// A plain (non-EXT, non-CE) command — every firmware-download command —
    /// is `MCU_Q_NA` with no ack byte. Getting this wrong makes the download
    /// commands look like queries.
    #[test]
    fn mcu_txd_plain_command_is_q_na() {
        let d = mcu_txd(CMD_TARGET_ADDRESS_LEN_REQ, 1, 12);
        assert_eq!(d[36], 0x01, "cid");
        assert_eq!(d[38], MCU_Q_NA);
        assert_eq!(d[41], 0, "ext_cid");
        assert_eq!(d[43], 0, "ext_cid_ack");
        assert_eq!(u16::from_le_bytes([d[32], d[33]]), (64 + 12 - 32) as u16);
    }

    /// A CE command is set/query even with no ext_cid, and does NOT set
    /// ext_cid_ack (`mt76_connac_mcu.c:3529-3537`).
    #[test]
    fn mcu_txd_ce_command_is_set_without_ext_ack() {
        let d = mcu_txd(CE_CMD_SET_RX_FILTER, 3, 68);
        assert_eq!(d[36], 0x0a);
        assert_eq!(d[38], MCU_Q_SET);
        assert_eq!(d[41], 0);
        assert_eq!(d[43], 0);
        let q = mcu_txd(McuCmd::ce_query(0xc0), 3, 4);
        assert_eq!(q[38], MCU_Q_QUERY);
    }

    /// The 48-byte unified descriptor.
    #[test]
    fn uni_txd_layout() {
        let d = mcu_uni_txd(UNI_CMD_SNIFFER, 9, 12);
        assert_eq!(d.len(), 48);
        assert_eq!(
            u32::from_le_bytes([d[0], d[1], d[2], d[3]]),
            0x4000_0000 | 0x0100_0000 | (48 + 12)
        );
        assert_eq!(u16::from_le_bytes([d[32], d[33]]), (48 + 12 - 32) as u16);
        assert_eq!(u16::from_le_bytes([d[34], d[35]]), 0x24, "cid");
        assert_eq!(d[37], MCU_PKT_ID);
        assert_eq!(d[39], 9, "seq");
        assert_eq!(d[42], MCU_S2D_H2N);
        assert_eq!(d[43], MCU_CMD_UNI_EXT_ACK);
    }

    // ── USB framing ──────────────────────────────────────────────────────

    /// The USB prefix counts the descriptor and payload but **not itself**,
    /// and the frame is rounded to 4 and then given 4 more zero bytes.
    #[test]
    fn command_frame_header_pad_and_tail() {
        // payload 3 B => not 4-aligned, so the round-up is exercised.
        let f = command_frame(EXT_CMD_MAC_INIT_CTRL, 5, &[1, 2, 3]);
        assert_eq!(
            u32::from_le_bytes([f[0], f[1], f[2], f[3]]) & SDIO_HDR_TX_BYTES,
            (64 + 3) as u32,
            "TX_BYTES excludes the 4-byte header itself on USB",
        );
        assert_eq!(
            u32::from_le_bytes([f[0], f[1], f[2], f[3]]) & SDIO_HDR_PKT_TYPE,
            0,
            "PKT_TYPE is 0 on USB (only SDIO passes MT7921_SDIO_DATA)",
        );
        // 4 + 64 + 3 = 71 -> round_up(71,4) = 72 -> +4 tail = 76
        assert_eq!(f.len(), 76);
        assert_eq!(&f[68..71], &[1, 2, 3]);
        assert!(f[71..].iter().all(|&b| b == 0), "pad and tail are zero");

        // An already-4-aligned frame still gets the 4-byte tail.
        let g = command_frame(EXT_CMD_MAC_INIT_CTRL, 5, &[1, 2, 3, 4]);
        assert_eq!(g.len(), 4 + 64 + 4 + 4);
    }

    /// ★ FW_SCATTER carries no descriptor at all.
    #[test]
    fn fw_scatter_frame_has_no_descriptor() {
        let chunk = vec![0xabu8; FW_CHUNK_MAX];
        let f = fw_scatter_frame(&chunk);
        assert_eq!(
            u32::from_le_bytes([f[0], f[1], f[2], f[3]]) & SDIO_HDR_TX_BYTES,
            FW_CHUNK_MAX as u32
        );
        assert_eq!(
            f[4], 0xab,
            "image bytes start right after the 4-byte header"
        );
        assert_eq!(f.len(), 4 + FW_CHUNK_MAX + 4);

        // A ragged last chunk pads to 4 and then adds the tail.
        let f = fw_scatter_frame(&[0x11, 0x22, 0x33, 0x44, 0x55]);
        assert_eq!(f.len(), 4 + 8 + 4);
        assert_eq!(&f[4..9], &[0x11, 0x22, 0x33, 0x44, 0x55]);
        assert!(f[9..].iter().all(|&b| b == 0));
    }

    // ── sequence numbers ─────────────────────────────────────────────────

    /// Sequence walks 1..=15 and never issues 0 — 0 is what a truncated or
    /// all-zero event decodes to, so a matcher that accepts it would take
    /// garbage for a response.
    #[test]
    fn seq_never_hands_out_zero() {
        let mcu = Connac2Mcu::new();
        let mut seen = [false; 16];
        for _ in 0..64 {
            let s = mcu.next_seq();
            assert_ne!(s, 0);
            assert!(s < 16);
            seen[s as usize] = true;
        }
        assert!(!seen[0]);
        assert!(seen[1..].iter().all(|&b| b), "all of 1..=15 are used");
    }

    // ── event parsing ────────────────────────────────────────────────────

    fn synth_event(pkt_type: u8, seq: u8, status: u8, payload: &[u8]) -> Vec<u8> {
        let total = MCU_RXD_LEN + payload.len();
        let mut b = vec![0u8; total];
        let rxd0 = fp(MT_RXD0_LENGTH, total as u32) | fp(MT_RXD0_PKT_TYPE, pkt_type as u32);
        b[0..4].copy_from_slice(&rxd0.to_le_bytes());
        b[24..26].copy_from_slice(&(payload.len() as u16).to_le_bytes());
        b[28] = 0x01; // eid
        b[29] = seq;
        b[32] = status; // ★ where the patch commands' status lives
        b[MCU_RXD_LEN..].copy_from_slice(payload);
        b
    }

    /// ★ The patch semaphore status is the byte at offset 32 — the one
    /// `skb_pull(sizeof(*rxd) - 4)` lands on (`mt7921/mcu.c:36-37`).
    #[test]
    fn event_status_byte_is_at_offset_32() {
        let raw = synth_event(PKT_TYPE_RX_EVENT, 4, PATCH_NOT_DL_SEM_SUCCESS, &[]);
        let ev = parse_event(&raw).unwrap();
        assert_eq!(ev.pkt_type, PKT_TYPE_RX_EVENT);
        assert_eq!(ev.seq, 4);
        assert_eq!(ev.status_u8(), PATCH_NOT_DL_SEM_SUCCESS);
        assert!(ev.payload().is_empty());
    }

    /// A short transfer is refused rather than indexed into: a fragment's
    /// bytes must never be read as a sequence number.
    #[test]
    fn event_shorter_than_the_header_is_refused() {
        assert!(parse_event(&[0u8; 35]).is_err());
        assert!(parse_event(&[]).is_err());
        assert!(parse_event(&synth_event(PKT_TYPE_RX_EVENT, 1, 0, &[])).is_ok());
    }

    /// The header's own length wins over a bulk transfer padded up to a
    /// packet boundary.
    #[test]
    fn event_trims_to_the_header_length() {
        let mut raw = synth_event(PKT_TYPE_RX_EVENT, 2, 0, &[9, 9, 9, 9]);
        raw.resize(512, 0); // as if the device padded the bulk transfer
        let ev = parse_event(&raw).unwrap();
        assert_eq!(ev.raw.len(), MCU_RXD_LEN + 4);
        assert_eq!(ev.payload(), &[9, 9, 9, 9]);
    }

    /// The UNI status word lives at payload offset 4 after a `cid` byte and 3
    /// pad (`mt76_connac_mcu.h:1872-1876`).
    #[test]
    fn uni_status_decodes() {
        let mut p = vec![0u8; 8];
        p[0] = 0x24; // cid
        p[4..8].copy_from_slice(&0u32.to_le_bytes());
        let ev = parse_event(&synth_event(PKT_TYPE_RX_EVENT, 3, 0, &p)).unwrap();
        assert_eq!(ev.uni_status(), Some((0x24, 0)));
        let short = parse_event(&synth_event(PKT_TYPE_RX_EVENT, 3, 0, &[1, 2])).unwrap();
        assert_eq!(short.uni_status(), None);
    }

    // ── the shipped blobs ────────────────────────────────────────────────

    /// ★ MEASURED patch header. If a re-vendored blob does not match this, the
    /// bring-up assumptions in this file no longer hold and the test says so
    /// before the hardware does.
    #[test]
    fn patch_header_matches_the_measured_blob() {
        assert_eq!(MT7961_PATCH.len(), 92_192);
        let (hdr, secs) = parse_patch(MT7961_PATCH).unwrap();
        assert_eq!(&hdr.build_date, b"20250625153620a\n");
        assert_eq!(&hdr.platform, b"ALPS");
        assert_eq!(hdr.hw_sw_ver, 0x8a10_8a10);
        assert_eq!(hdr.hw_ver(), 0x8a10);
        assert_eq!(hdr.sw_ver(), 0x8a10);
        assert_eq!(hdr.patch_ver, 0xffff_ffff);
        assert_eq!(hdr.checksum, 0);
        // The `44 33 22 11` at offset 32 that the brief calls the patch magic:
        // big-endian it reads 0x44332211, i.e. LE 0x11223344.
        assert_eq!(hdr.desc_patch_ver, 0x4433_2211);
        assert_eq!(hdr.subsys, 4);
        assert_eq!(hdr.feature, 0);
        assert_eq!(hdr.n_region, 1);
        assert_eq!(hdr.crc, 0x0000_ffff);

        assert_eq!(secs.len(), 1);
        let s = secs[0];
        assert_eq!(s.sec_type & PATCH_SEC_TYPE_MASK, PATCH_SEC_TYPE_INFO);
        assert_eq!(s.offs, 160, "immediately after hdr(96) + one sec(64)");
        assert_eq!(s.size, 92_032);
        assert_eq!(s.len, 92_032);
        assert_eq!(s.sec_key_idx, 0, "plain, so mode is bare NEED_RSP");
        // The section's bytes account for the file exactly.
        assert_eq!(s.offs as usize + s.len as usize, MT7961_PATCH.len());
        // ★ the address that routes the preamble to PATCH_START_REQ
        assert_eq!(s.addr, CONNAC2_PATCH_ADDRESS);
        assert_eq!(init_download_cmd(s.addr), CMD_PATCH_START_REQ);
        assert_eq!(patch_data_mode(s.sec_key_idx).unwrap(), DL_MODE_NEED_RSP);
    }

    /// ★ The blob's hardware version is exactly the MEASURED `MT_HW_REV`, and
    /// a mismatch is refused before a single byte is sent.
    #[test]
    fn patch_hw_version_matches_the_measured_chip() {
        let (hdr, _) = parse_patch(MT7961_PATCH).unwrap();
        const MEASURED_MT_HW_REV: u32 = 0x0000_8a10;
        assert_eq!(hdr.hw_ver(), (MEASURED_MT_HW_REV & 0xffff) as u16);
        assert_ne!(hdr.hw_ver(), 0x8a11, "a neighbouring rev must not match");
    }

    /// A corrupt region count must be refused, not used to index.
    #[test]
    fn patch_parser_rejects_impossible_region_counts() {
        let mut bad = MT7961_PATCH[..512].to_vec();
        bad[44..48].copy_from_slice(&0x0000_1000u32.to_be_bytes());
        assert!(parse_patch(&bad).is_err());
        assert!(parse_patch(&MT7961_PATCH[..64]).is_err());
    }

    /// ★ MEASURED RAM trailer.
    #[test]
    fn ram_trailer_matches_the_measured_blob() {
        assert_eq!(MT7961_RAM.len(), 791_588);
        let (t, _) = parse_ram(MT7961_RAM).unwrap();
        assert_eq!(t.chip_id, 0x0d);
        assert_eq!(t.eco_code, 0x01);
        assert_eq!(t.n_region, 5);
        assert_eq!(t.format_ver, 2);
        assert_eq!(t.format_flag, 1);
        assert_eq!(&t.fw_ver, b"____010000");
        assert_eq!(&t.build_date, b"20250625153703\0");
        assert_eq!(t.crc, 0x4dae_f689);
    }

    /// ★ MEASURED region table: 5 regions, 4 downloadable, one `NON_DL` CLC
    /// blob whose bytes still advance the offset, and region 0 carrying the
    /// `FW_START_REQ` override address.
    #[test]
    fn ram_regions_match_the_measured_blob() {
        let (_, r) = parse_ram(MT7961_RAM).unwrap();
        assert_eq!(r.len(), 5);

        let want = [
            //  addr,        len,     feature, kind, file_offset
            (0x0091_5000u32, 363_536u32, 0x20u8, 0u8, 0usize),
            (0x0201_5c00, 272_400, 0x00, 0, 363_536),
            (0x0040_4400, 15_376, 0x00, 0, 635_936),
            (0xe027_0000, 51_472, 0x00, 0, 651_312),
            (0x0000_0000, 88_416, 0x40, FW_TYPE_CLC, 702_784),
        ];
        for (i, (addr, len, feature, kind, off)) in want.into_iter().enumerate() {
            assert_eq!(r[i].addr, addr, "region {i} addr");
            assert_eq!(r[i].len, len, "region {i} len");
            assert_eq!(r[i].feature_set, feature, "region {i} feature_set");
            assert_eq!(r[i].kind, kind, "region {i} type");
            assert_eq!(r[i].file_offset, off, "region {i} file offset");
        }

        // 4 downloadable, 1 skipped — and the skipped one is the CLC blob.
        let dl: Vec<_> = r.iter().filter(|x| x.downloadable()).collect();
        assert_eq!(dl.len(), 4);
        assert!(!r[4].downloadable());
        assert_eq!(r[4].kind, FW_TYPE_CLC);
        assert_eq!(dl.iter().map(|x| x.len as usize).sum::<usize>(), 702_784);

        // Exactly one region supplies the FW_START_REQ override.
        let ov: Vec<_> = r.iter().filter(|x| x.is_override()).collect();
        assert_eq!(ov.len(), 1);
        assert_eq!(ov[0].addr, 0x0091_5000);

        // Every RAM region takes the non-patch preamble.
        for x in &r {
            assert_eq!(init_download_cmd(x.addr), CMD_TARGET_ADDRESS_LEN_REQ);
        }

        // Every region's bytes lie inside the image data area.
        let data_end = MT7961_RAM.len() - FW_TRAILER_LEN - 5 * FW_REGION_LEN;
        for x in &r {
            assert!(x.file_offset + x.len as usize <= data_end);
        }
        // ★ 152 B between the last region's bytes and the region table are
        // unaccounted for in the format; upstream ignores them and so does
        // this port. Recorded so a future reader does not "fix" it.
        assert_eq!(data_end - (r[4].file_offset + r[4].len as usize), 152);
    }

    /// ★ The `NON_DL` region must be skipped for download but must still
    /// advance the offset. A walker that skips the bytes too mis-aligns
    /// everything after it.
    #[test]
    fn non_dl_region_still_advances_the_file_offset() {
        let (_, r) = parse_ram(MT7961_RAM).unwrap();
        // The last downloadable region ends where the NON_DL one begins.
        assert_eq!(r[3].file_offset + r[3].len as usize, r[4].file_offset);
    }

    /// The download mode word for our actual feature bytes: none of `0x00`,
    /// `0x20`, `0x40` sets an encryption bit, so all four downloads go out as
    /// bare `DL_MODE_NEED_RSP`.
    #[test]
    fn gen_dl_mode_for_the_measured_feature_bytes() {
        assert_eq!(gen_dl_mode(0x00, false), DL_MODE_NEED_RSP);
        assert_eq!(gen_dl_mode(0x20, false), DL_MODE_NEED_RSP);
        assert_eq!(gen_dl_mode(0x40, false), DL_MODE_NEED_RSP);
        // And the encrypted branches, for the day a blob uses them.
        assert_eq!(
            gen_dl_mode(FW_FEATURE_SET_ENCRYPT, false),
            DL_MODE_NEED_RSP | DL_MODE_ENCRYPT | DL_MODE_RESET_SEC_IV
        );
        assert_eq!(
            gen_dl_mode(FW_FEATURE_ENCRY_MODE, false),
            DL_MODE_NEED_RSP | DL_CONFIG_ENCRY_MODE_SEL
        );
        // key idx 3 (bits 2:1 of feature_set) lands in bits 2:1 of the mode.
        assert_eq!(
            gen_dl_mode(0x06, false),
            DL_MODE_NEED_RSP | fp(DL_MODE_KEY_IDX, 3)
        );
        assert_eq!(
            gen_dl_mode(0x00, true),
            DL_MODE_NEED_RSP | DL_MODE_WORKING_PDA_CR4
        );
    }

    /// `PATCH_SEC_NOT_SUPPORT` short-circuits to bare NEED_RSP, and the two
    /// encrypted branches set what upstream sets.
    #[test]
    fn patch_data_mode_branches() {
        assert_eq!(
            patch_data_mode(PATCH_SEC_NOT_SUPPORT).unwrap(),
            DL_MODE_NEED_RSP
        );
        assert_eq!(patch_data_mode(0).unwrap(), DL_MODE_NEED_RSP);
        // ★ AES: the key index is the raw low byte, FIELD_PREP'd (i.e. shifted
        // left by 1) and *then* masked to bits 2:1 — so low byte 2 lands as the
        // field value 2, and anything >= 4 is silently truncated away. That is
        // `mode |= FIELD_PREP(DL_MODE_KEY_IDX, info & 0xff) & DL_MODE_KEY_IDX`
        // at mt76_connac_mcu.c:3224-3225, quirk included.
        assert_eq!(
            patch_data_mode(0x0100_0002).unwrap(),
            DL_MODE_NEED_RSP | DL_MODE_ENCRYPT | fp(DL_MODE_KEY_IDX, 2) | DL_MODE_RESET_SEC_IV,
        );
        assert_eq!(
            patch_data_mode(0x0100_0004).unwrap() & DL_MODE_KEY_IDX,
            0,
            "a low byte of 4 shifts clean out of the 2-bit field",
        );
        assert_eq!(
            patch_data_mode(0x0200_0000).unwrap(),
            DL_MODE_NEED_RSP | DL_MODE_ENCRYPT | DL_CONFIG_ENCRY_MODE_SEL | DL_MODE_RESET_SEC_IV,
        );
        assert!(patch_data_mode(0x0300_0000).is_err());
    }

    /// The chunker splits at 4096 and the last chunk is whatever is left.
    #[test]
    fn chunking_is_4096_with_a_ragged_tail() {
        let (_, r) = parse_ram(MT7961_RAM).unwrap();
        let n: usize = r
            .iter()
            .filter(|x| x.downloadable())
            .map(|x| (x.len as usize).div_ceil(FW_CHUNK_MAX))
            .sum();
        assert_eq!(n, 173, "173 FW_SCATTER transfers for the whole RAM image");
        assert_eq!((363_536usize).div_ceil(FW_CHUNK_MAX), 89);
    }

    // ── request payloads ─────────────────────────────────────────────────

    /// ★ `rx_streams` is a count for CHANNEL_SWITCH and a mask for
    /// SET_RX_PATH — one field, two meanings (`mt7921/mcu.c:930-931`).
    #[test]
    fn channel_req_rx_streams_is_a_count_only_for_channel_switch() {
        let req = ChannelReq {
            control_ch: 36,
            center_ch: 42,
            bw: CMD_CBW_80MHZ,
            tx_streams: 2,
            rx_streams_mask: 0b11,
            switch_reason: CH_SWITCH_NORMAL,
            band_idx: 0,
            center_ch2: 0,
            channel_band: CHANNEL_BAND_5G,
        };
        let cs = req.encode(EXT_CMD_CHANNEL_SWITCH);
        assert_eq!(cs.len(), 76);
        assert_eq!(cs[0], 36);
        assert_eq!(cs[1], 42);
        assert_eq!(cs[2], CMD_CBW_80MHZ);
        assert_eq!(cs[3], 2);
        assert_eq!(cs[4], 2, "CHANNEL_SWITCH wants the popcount");
        assert_eq!(cs[5], CH_SWITCH_NORMAL);
        assert_eq!(cs[10], CHANNEL_BAND_5G);
        assert!(cs[19..].iter().all(|&b| b == 0), "rsv1[57]");

        let rp = req.encode(EXT_CMD_SET_RX_PATH);
        assert_eq!(rp[4], 0b11, "SET_RX_PATH wants the mask");
    }

    /// The RX filter request is 68 bytes with `mode` at 4, `fif` at 8,
    /// `bit_map` at 12 and `bit_op` at 16.
    #[test]
    fn rx_filter_payload_layout() {
        let mut req = [0u8; 68];
        req[4] = 1;
        req[8..12].copy_from_slice(&RX_FILTER_PROMISCUOUS.to_le_bytes());
        assert_eq!(req.len(), 68);
        // The promiscuous words themselves.
        assert_eq!(RX_FILTER_PROMISCUOUS, 0x8000_0064);
        assert_eq!(RX_FILTER_PROMISCUOUS_VALID, 0x8000_0060);
        assert_eq!(
            RX_FILTER_PROMISCUOUS & !RX_FILTER_PROMISCUOUS_VALID,
            RX_FILTER_FCSFAIL,
            "the decoder-safe variant differs only by the FCS-fail pass",
        );
    }

    /// The sniffer enable request is a 4-byte header plus an 8-byte TLV.
    #[test]
    fn sniffer_enable_payload_layout() {
        let mut req = [0u8; 12];
        req[0] = 0;
        req[4..6].copy_from_slice(&0u16.to_le_bytes());
        req[6..8].copy_from_slice(&8u16.to_le_bytes());
        req[8] = 1;
        assert_eq!(req.len(), 12);
        assert_eq!(u16::from_le_bytes([req[6], req[7]]), 8);
    }

    /// The sniffer config request is 20 bytes, and `sco` is derived from the
    /// control/centre relationship rather than supplied.
    #[test]
    fn sniffer_config_derives_sco_from_the_channel_pair() {
        let below = SnifferChan {
            band_idx: 0,
            ch_band: 2,
            bw: 1,
            control_ch: 36,
            center_ch: 42,
            center_ch2: 0,
            drop_err: 1,
        }
        .encode();
        assert_eq!(below.len(), 20);
        assert_eq!(u16::from_le_bytes([below[4], below[5]]), 1, "tag 1");
        assert_eq!(u16::from_le_bytes([below[6], below[7]]), 16, "tlv len");
        assert_eq!(below[10], 2, "ch_band 5 GHz in the sniffer's own numbering");
        assert_eq!(below[12], 36);
        assert_eq!(below[13], 1, "SCA when control < centre");
        assert_eq!(below[14], 42);
        assert_eq!(below[16], 1, "drop_err");

        let mut above_chan = SnifferChan {
            band_idx: 0,
            ch_band: 2,
            bw: 1,
            control_ch: 36,
            center_ch: 42,
            center_ch2: 0,
            drop_err: 1,
        };
        above_chan.control_ch = 48;
        let above = above_chan.encode();
        assert_eq!(above[13], 3, "SCB when control > centre");

        let same = SnifferChan {
            band_idx: 0,
            ch_band: 1,
            bw: 0,
            control_ch: 6,
            center_ch: 6,
            center_ch2: 0,
            drop_err: 0,
        }
        .encode();
        assert_eq!(same[13], 0, "no offset at 20 MHz");
        assert_eq!(same[16], 0, "drop_err is exposed, not hardcoded");
    }

    /// The efuse read rounds down to a 16-byte block boundary, and the MAC
    /// address lands at block offset 4.
    #[test]
    fn efuse_block_alignment_and_mac_offset() {
        assert_eq!(MT_EE_MAC_ADDR & !(EEPROM_BLOCK_SIZE as u32 - 1), 0);
        assert_eq!((MT_EE_MAC_ADDR as usize) % EEPROM_BLOCK_SIZE, 4);
        // 6 bytes of MAC starting at 4 fit inside the 16-byte block.
        const { assert!(4 + 6 <= EEPROM_BLOCK_SIZE) };
    }

    // ── struct sizes, which upstream expresses only as C layout ──────────

    /// The descriptor and header sizes every offset in this file depends on.
    #[test]
    fn wire_struct_sizes() {
        assert_eq!(MCU_TXD_LEN, 64, "mt76_connac2_mcu_txd");
        assert_eq!(MCU_UNI_TXD_LEN, 48, "mt76_connac2_mcu_uni_txd");
        assert_eq!(MCU_RXD_LEN, 36, "mt76_connac2_mcu_rxd");
        assert_eq!(PATCH_HDR_LEN, 96, "mt76_connac2_patch_hdr");
        assert_eq!(PATCH_SEC_LEN, 64, "mt76_connac2_patch_sec");
        assert_eq!(FW_TRAILER_LEN, 36, "mt76_connac2_fw_trailer");
        assert_eq!(FW_REGION_LEN, 40, "mt76_connac2_fw_region");
        // headroom = MT_SDIO_HDR_SIZE + sizeof(mt76_connac2_mcu_txd) = 68
        assert_eq!(USB_HDR_LEN + MCU_TXD_LEN, 68);
    }
}
