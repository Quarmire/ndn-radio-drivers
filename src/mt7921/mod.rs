//! Userspace libusb backend for the **MT7921AU** (`0e8d:7961`, MediaTek `connac2`, 2×2
//! **802.11ax**) — the third MediaTek part in this crate beside the 2×2 11ac
//! [`MT7612U`](crate::Mt7612uBackend) and the 1×1 11ac [`MT7610U`](crate::Mt7610uBackend),
//! and the only one that shares no silicon with them: `connac2` is a different MAC, a
//! different descriptor format and a different firmware model, which is why it gets its own
//! four-layer stack in [`crate::connac2`] rather than joining [`crate::mt76`].
//!
//! This file is the thin part: it owns bring-up ordering, the monitor configuration, the RX
//! and TX hot paths and the four HAL traits. Everything it stands on —
//! [`usb`](crate::connac2::usb) (EP0 + endpoints), [`regs`](crate::connac2::regs) (the
//! numeric map), [`mcu`](crate::connac2::mcu) (framing + firmware + commands) and
//! [`mac`](crate::connac2::mac) (RXD/TXD) — is ported and documented there.
//!
//! # ★ Why this part earns its place — two things nothing else in this crate has
//!
//! **1. A per-frame hardware RX timestamp, i.e. common view.** `mt7921/mac.c:307-309`
//! latches `status->timestamp = le32_to_cpu(rxd[0]); status->flag |= RX_FLAG_MACTIME_START`
//! out of **RXD group 2**. `struct mt76x02_rxwi` (`mt76x02_mac.h:97-108`) — the descriptor
//! both other MediaTek backends parse — is `rxinfo/ctl/tid_sn/rate/rssi[4]/bbp_rxinfo[4]`,
//! and **not one of those fields is a time**. That is the whole reason the MT7612U and
//! MT7610U can only ever offer a *read-now* [`RadioClockKind::PortTsf`], costing a 151 µs
//! EP0 round trip per read, and why `FaceTimeProfile::can_common_view` is correctly `false`
//! for both. This part stamps the frame **in the MAC** and ships the stamp inside the
//! descriptor, so [`RadioTime::time_sources`] here declares a genuine
//! [`RadioClockKind::FreeRunRxStamp`] and `can_common_view` comes out **true** — putting it
//! in the same class as the Realtek a81a/8733b (`RXTSFL`) and the AR9271 (`rs_tstamp`).
//! See [`RadioTime::time_sources`] for the tick, the wrap, the domain choice, and exactly
//! how much of it is measured (none of it yet).
//!
//! **2. 802.11ax.** [`McsDescriptor`]'s `he` / `dcm` / `er_su` levers have existed in the
//! HAL with **no Wi-Fi actuator anywhere in this crate**. [`mac::encode_rate`] is that
//! actuator and this backend is what reaches it, so [`declared_capability`] is the first —
//! and today the only — Wi-Fi capability in the crate with `he_cap: true`. Read
//! [`mac::encode_rate`]'s own doc before believing the HE encoding: upstream never transmits
//! a fixed HT/VHT/HE rate on this chip (`mt76_connac_mac.c:321-324` short-circuits
//! `is_connac2` to the legacy branch), so the HE bits are assembled from field masks plus
//! the mt7915 testmode path, not from a code path that has ever run on a 7961.
//!
//! [`RadioClockKind::PortTsf`]: ndn_time::RadioClockKind::PortTsf
//! [`RadioClockKind::FreeRunRxStamp`]: ndn_time::RadioClockKind::FreeRunRxStamp
//!
//! # MEASURED vs CODE-READ
//!
//! Reasoning about these radios has a ~0 % hit rate on this bench and measurement ~100 %, so
//! the split is kept in the code rather than in prose somewhere else.
//!
//! **MEASURED on mds-o5p-3's MT7921AU (2026-08-27) — this file depends on all of it:**
//!   * The part is on **USB 2.0** (high speed, 480). It probes cleanly under the kernel
//!     `mt7921u`.
//!   * ★ **The WLAN function is interface 3**, class `ff/ff/ff`. Interfaces 0-2 are
//!     `e0/01/01` Bluetooth and belong to `btusb` — **never touched**; the transport claims
//!     interface 3 only ([`crate::connac2::usb::select_wlan_interface`]).
//!   * Endpoints on if3: bulk IN `0x84` (`MT_EP_IN_PKT_RX`), `0x85` (`MT_EP_IN_CMD_RESP`);
//!     bulk OUT `0x04`-`0x09` (`INBAND_CMD`, `AC_BE`, `AC_BK`, `AC_VI`, `AC_VO`, `HCCA`);
//!     interrupt IN `0x86`. All 512 B. **TX goes out on `0x05` (AC_BE)**, not the command
//!     pipe.
//!   * Register access works over `MT_VEND_READ_EXT` (`0x63`) / `MT_VEND_WRITE_EXT`
//!     (`0x66`), `bmRequestType` `0xc0`/`0x40`, `wValue = addr >> 16`,
//!     `wIndex = addr & 0xffff`, 4-byte LE. The **full 32-bit address** goes straight in; no
//!     remap window.
//!   * `MT_HW_CHIPID = 0x7961`, `MT_HW_REV = 0x8a10` (= the patch header's `hw_sw_ver`),
//!     `MT_CONN_ON_MISC = 0x0000_0000`. ★ That last one means `FW_PWR_ON` and `FW_N9_RDY`
//!     are **clear on a cold plug** — firmware is not running — which is what makes
//!     [`firmware_running`](Mt7921uBackend::firmware_running) a meaningful guard rather than
//!     a coin flip.
//!   * ★ **EP0 round trip = 268 µs** ([`regs::measured::EP0_ROUND_TRIP_US`]), nearly double
//!     the MT7610U's 151 µs and triple the 8733b's ~92 µs. Nothing on a per-frame path may
//!     read a register. That is why the RX timestamp *must* come out of the descriptor, why
//!     [`rx_health`](Mt7921uBackend::rx_health) is a diagnostic and not a poll, and why
//!     [`dma_init`](Mt7921uBackend::dma_init)'s ~45 register operations cost ~12 ms.
//!
//! **CODE-READ, unvalidated on this silicon — everything else**, in particular: the bring-up
//! ordering, the firmware download, the monitor configuration below, that RXD group 2 is
//! present on a monitor-mode receive at all, the 1 µs tick of that timestamp, the TXD, and
//! every HE bit. The gate that turns these into measurements is a capture on the target:
//! bring up, dump the first 144 B of a bulk-IN transfer, check `rxd1` bits 11-15 against the
//! offset the 802.11 header actually starts at, and difference the group-2 dword against
//! `MT_LPON_UTTR0` read immediately after.
//!
//! # The two traps this file is built around
//!
//! **★ The warm-reopen guard must not be a mailbox.** The MT7610U/MT7612U guard is
//! `MT_MCU_COM_REG0 == 1` — and `MT_MCU_COM_REG0` is a **mailbox the firmware reuses**, so
//! the guard goes stale within seconds of a successful load and the next `open()`
//! re-downloads firmware into a live MCU. That took the device off the USB bus and needed a
//! physical replug **three times this week**. On connac2 the evidence is *persistent state*,
//! not a message: see [`firmware_running`](Mt7921uBackend::firmware_running).
//!
//! **★ Never `handle.reset()`, and never a blind chip reset.** A failed USB reset leaves the
//! hub port `disable=1` and the part is unrecoverable without a physical replug (three
//! MT7612U casualties, same week). There is no `reset()` anywhere in this stack. Upstream's
//! probe *does* call `usb_reset_device` (`mt7921/usb.c:206`); we deliberately do not follow
//! it. The only reset used here is the *subsystem* reset inside
//! [`mcu::power_up`](crate::connac2::mcu::power_up), which goes through
//! `MT_CBTOP_RGU_WF_SUBSYS_RST` and never touches the bus.
//!
//! # Deliberate omissions, so they are visible rather than lost
//!
//!   * **TX power.** No `set_tx_power` / `set_tx_power_dbm`. Power on connac2 is
//!     firmware-owned: `mt7921_set_tx_sar_pwr` pushes a whole SAR table through
//!     `MCU_CE_CMD(SET_RATE_TX_POWER)` / the per-band SAR TLVs, and there is no host
//!     register the way the mt76x0's `MT_TX_ALC_CFG_0` is one. Half-porting it would give a
//!     knob whose effect nobody could state. `max_tx_power` is declared as an **inherited,
//!     unverified** value and says so.
//!   * **40 / 80 / 160 MHz.** [`RadioKnobs::set_channel`] refuses anything but 20 MHz and
//!     [`declared_capability`] reports `max_bw: 2` to match — the seam carries only
//!     `(channel, bw)` and a wider width needs the *centre* channel
//!     ([`mcu::ChannelReq::center_ch`](crate::connac2::mcu::ChannelReq)), which cannot be
//!     derived from the control channel alone. Declaration and actuator agree.
//!   * **A-MPDU / A-MSDU.** [`FrameIo::inject_batch`] sends one MPDU per NDN packet.
//!     Hardware de-aggregation is enabled on RX ([`regs::MT_MDP_DCR0_DAMSDU_EN`]); host-built
//!     aggregation on monitor injection is firmware-gated on the sibling MT7612U (verified
//!     0/200 on air there) and untested here.
//!   * **TXS (transmit status).** `MT_TXS4_TIMESTAMP` (`mt76_connac2_mac.h:174`) is a
//!     32-bit stamp in the *same domain* as the RX stamp, which is what would make a TX/RX
//!     common view possible on one part — the AR9271 lesson, where the enabler was a
//!     firmware TX counter and not the RX path. It needs a TXS ring and a `pktid` map; not
//!     here, but this is the first part in the crate where it is even possible.
//!   * **`PhyMetrics`.** Left `None`, and that is a finding rather than a gap:
//!     [`mac::crxv_snr_db`] returns `None` for every normal RXD **by construction** — the
//!     C-RXV embedded as group 5 is 18 dwords (`mt7915/mac.c:448`) and `MT_CRXV_SNR` lives
//!     in dword 20. Per-frame SNR/CFO on connac2 need the standalone
//!     [`PKT_TYPE_TXRXV`](crate::connac2::mac::PKT_TYPE_TXRXV) report, which upstream builds
//!     only under `CONFIG_NL80211_TESTMODE`.
#![allow(dead_code)]

use std::sync::Arc;
use std::sync::Mutex;
use std::sync::atomic::{AtomicBool, AtomicU8, AtomicU16, AtomicU64, Ordering};
use std::time::{Duration, Instant};

use async_trait::async_trait;
use rusb::{Context, DeviceHandle};

use ndn_radio_hal::{
    Band, Bandwidth, CsiSupport, RadioCapability, RadioKind, RadioKnobs, RadioProfile, RadioTime,
    RateCapability, TxDiscipline,
};
use ndn_time::{ClockDomainId, LatchPoint, LinkStamp, RadioTimeSource};

use crate::connac2::{mac, mcu, regs, usb::Connac2Usb};
use crate::usb_select::DeviceSelect;
use crate::{CapturedFrame, FaceError, FrameFormat, FrameIo, InjectFrame, McsDescriptor};

// ── Identity ────────────────────────────────────────────────────────────────

/// MediaTek's USB vendor id. MEASURED: the reference MT7921AU on mds-o5p-3 is `0e8d:7961`.
pub const MEDIATEK_VID: u16 = 0x0e8d;

/// MT7921AU under MediaTek's own vendor id.
pub const MT7921AU_PID: u16 = 0x7961;

/// The product ids this backend claims **under [`MEDIATEK_VID`]**, and nothing else.
///
/// ★ Deliberately a *vid-scoped* list, because the flat `&[u16]` shape the rest of this
/// crate uses for PID tables **cannot express this part's device table**. Four of the five
/// boards upstream claims sit behind *other* vendor ids (`mt7921/usb.c:15-30`), so a bare
/// PID list is either a lie (it implies `0e8d:6211` exists) or an under-claim. The honest
/// model is [`MT7921U_BOARDS`]; this constant is the MediaTek-VID subset, kept because the
/// crate's PID-dispatch and the [`crate::coverage`] table both want a `&'static [u16]`.
///
/// A dispatcher that wants the rebadges must match on `(vid, pid)` — which is exactly what
/// [`Connac2Usb::open_selected`] already does, walking
/// [`crate::connac2::usb::MT7921U_VID_PIDS`].
pub const MT7921U_PIDS: &[u16] = &[MT7921AU_PID];

/// Largest payload this backend will hand the hardware in one MPDU.
///
/// MEASURED: 7935 B transmits at 2742 f/s (174 Mbit/s at VHT MCS9 2SS/80 MHz/SGI); 11000 B
/// collapses to 3 f/s. 11454 is the 802.11 VHT A-MPDU length limit and is the ceiling used here;
/// [`FrameIo::inject`] refuses anything above it because an oversized MPDU resets these radios
/// rather than being dropped.
pub const MAX_MPDU_PAYLOAD: usize = 11_454;

/// One board upstream's `mt7921u_device_table` claims (`mt7921/usb.c:15-30`).
///
/// Carried as a struct rather than a tuple so `measured` can travel with the ids: a PID
/// table in this codebase is a claim about **what has been held in this lab**, and exactly
/// one of these has.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Mt7921uBoard {
    /// USB vendor id.
    pub vid: u16,
    /// USB product id.
    pub pid: u16,
    /// The board name upstream gives it, or the silicon name for the reference design.
    pub name: &'static str,
    /// True only for a board that has physically been on this bench.
    pub measured: bool,
}

/// Every board upstream claims, with its vendor id — **four different VIDs**, which is why
/// [`MT7921U_PIDS`] cannot stand alone.
///
/// All five are matched by upstream on the interface triple `ff/ff/ff` *as well as* on the
/// ids (`mt7921/usb.c:16`), the same rule [`crate::connac2::usb::select_wlan_interface`]
/// applies — so a composite stick whose Bluetooth function shares the ids still resolves to
/// the WLAN interface and `btusb` keeps its own.
pub const MT7921U_BOARDS: &[Mt7921uBoard] = &[
    Mt7921uBoard {
        vid: MEDIATEK_VID,
        pid: MT7921AU_PID,
        name: "MediaTek MT7921AU reference",
        measured: true, // mds-o5p-3, 2026-08-27
    },
    Mt7921uBoard {
        vid: 0x3574,
        pid: 0x6211,
        name: "Comfast CF-952AX",
        measured: false, // mt7921/usb.c:18-20
    },
    Mt7921uBoard {
        vid: 0x0846,
        pid: 0x9060,
        name: "Netgear A8000 / AXE3000",
        measured: false, // mt7921/usb.c:21-23
    },
    Mt7921uBoard {
        vid: 0x0846,
        pid: 0x9065,
        name: "Netgear A7500",
        measured: false, // mt7921/usb.c:24-26
    },
    Mt7921uBoard {
        vid: 0x35bc,
        pid: 0x0107,
        name: "TP-Link TXE50UH",
        measured: false, // mt7921/usb.c:27-29
    },
];

/// The channels [`RadioKnobs::set_channel`] will actually accept, and therefore the only
/// ones [`declared_capability`] declares — `mt76_channels_2ghz` / `mt76_channels_5ghz`
/// (`mac80211.c:30-79`), minus the 6 GHz table this port does not reach (see
/// [`RadioKnobs::set_channel`] on why 6 GHz is refused).
///
/// 5 GHz stops at 165: `mt76_channels_5ghz` continues 169/173/177, but those are U-NII-4
/// channels whose availability is regulatory-domain-dependent and whose acceptance by this
/// firmware's CLC tables has not been tried. A channel that is declared and then refused by
/// the MCU is worse than one that is never offered.
const CHANNELS_2GHZ: &[u8] = &[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14];
const CHANNELS_5GHZ: &[u8] = &[
    36, 40, 44, 48, // U-NII-1
    52, 56, 60, 64, // U-NII-2A (DFS)
    100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144, // U-NII-2C (DFS)
    149, 153, 157, 161, 165, // U-NII-3
];

// ── Timeouts ────────────────────────────────────────────────────────────────

/// TX bulk timeout. Generous: a stalled AC queue is a real condition, not a fast failure.
const BULK_TX_TIMEOUT: Duration = Duration::from_secs(1);
/// One-shot RX read window when no pump is running ([`FrameIo::recv_frame`]'s slow path).
const BULK_RX_TIMEOUT: Duration = Duration::from_millis(200);

/// Buffer for one bulk-IN transfer. One RX unit per transfer (see `parse_transfer`), but
/// an A-MSDU MPDU from a foreign network reaches ~7935 B and the descriptor can add 144 B,
/// so a short buffer would truncate a real frame into an undecodable descriptor rather than
/// merely dropping it.
const RX_BUF_LEN: usize = 16384;

fn io_err(what: String) -> FaceError {
    FaceError::Io(std::io::Error::other(what))
}

fn usb_err(e: rusb::Error) -> FaceError {
    FaceError::Io(std::io::Error::other(format!("mt7921u usb: {e}")))
}

// ── Software RX bookkeeping ─────────────────────────────────────────────────

/// A window of the RX pipeline's own counters, from
/// [`rx_stats_reset`](Mt7921uBackend::rx_stats_reset).
///
/// These are **software** counts of what came off USB. They are not the hardware MIB
/// counters [`RadioKnobs::read_ofdm_counters`] reads, and both are needed: the MIB says what
/// the PHY saw, these say what actually reached the host.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RxStats {
    /// RX units pulled off bulk-IN whose descriptor parsed.
    pub units: u64,
    /// ★ Of those, units that carried an **RXD group 2 timestamp**. This is the headline
    /// feature's health check: if `units` climbs and `stamped` does not, the per-frame clock
    /// is not there and every common-view claim built on it is void. Nothing upstream gates
    /// group 2 (see [`mac::Rxd::timestamp`]), so a zero here is a discovery, not a setting.
    pub stamped: u64,
    /// Units the hardware flagged `MT_RXD1_NORMAL_FCS_ERR` — a collision or a marginal link.
    /// Counted (it is real information about the channel) and then dropped.
    pub fcs_errors: u64,
    /// Units whose payload the hardware had rewritten into an 802.3 header
    /// ([`mac::Rxd::hdr_trans`]). Must stay **0**: it means
    /// [`regs::MT_MDP_DCR0_RX_HDR_TRANS_EN`] is set and there is no 802.11 header left to parse.
    pub hdr_translated: u64,
    /// Units carrying the 2-byte A-MSDU subframe pad, removed by [`mac::mpdu`].
    pub amsdu_pad: u64,
    /// ★ Transfers that decoded as a **firmware event or TXS report** rather than a frame
    /// (`pkt_type` outside `NORMAL`/`NORMAL_MCU`). A non-zero count is load-bearing: it
    /// means `MT_WFDMA_HOST_CONFIG_USB_RXEVT_EP4_EN` really does route MCU events onto the
    /// **data** pipe, so the RX pump and the MCU are sharing `0x84` and an MCU command
    /// issued while the pump runs can have its response eaten. See [`mcu::Connac2Mcu`].
    pub mcu_events: u64,
    /// Transfers whose descriptor did not decode. A climbing count here means the framing
    /// assumption is wrong, not that the air is quiet.
    pub undecodable: u64,
    /// Units that parsed all the way to a [`CapturedFrame`] in our wire format.
    pub accepted: u64,
}

// ── The backend ─────────────────────────────────────────────────────────────

/// A claimed MT7921AU: the connac2 USB transport, its MCU command channel, the current tune,
/// the RX pipeline, and this device's clock domain.

/// The 802.11 **centre channel** for a control channel at a given width, and the `CMD_CBW_*`
/// code that goes with it.
///
/// ★ This is what unblocked wider bandwidth, and the blocker was a seam, not the silicon. The
/// MCU has accepted `CMD_CBW_40MHZ` / `_80MHZ` / `_160MHZ` all along and `ChannelReq` has always
/// carried `center_ch`; what was missing is that `RadioKnobs::set_channel(channel, bw)` passes
/// only a *control* channel, and a wide PPDU needs its centre. The centre is not extra
/// information — 802.11 fixes it — so deriving it here costs nothing and turns a refused width
/// into a tuned one.
///
/// Why it was worth doing: MEASURED at 20 MHz the part tops out near 37 Mbit/s at a 1500 B MTU,
/// and every doubling of width is a doubling of the per-byte rate against a per-frame cost that
/// does not move. The sibling MT7612U's notes put VHT80 2x2 at ~142 Mbit/s where 20 MHz gives
/// ~37 — the same 37 this port measured before this function existed.
///
/// The blocks are the standard channelisation, written as tables rather than arithmetic because
/// the 5 GHz grid is not uniform (149-161 does not continue 132-144's spacing) and a clever
/// formula would be wrong at exactly the edges nobody tests.
fn centre_and_cbw(control: u8, bw: Bandwidth) -> Option<(u8, u8)> {
    /// 5 GHz 40 MHz pairs; the centre is the lower member + 2.
    const PAIRS_5G: [u8; 12] = [36, 44, 52, 60, 100, 108, 116, 124, 132, 140, 149, 157];
    /// 5 GHz 80 MHz blocks: (lowest control channel, centre).
    const BLOCKS80: [(u8, u8); 6] = [
        (36, 42),
        (52, 58),
        (100, 106),
        (116, 122),
        (132, 138),
        (149, 155),
    ];
    /// 5 GHz 160 MHz blocks. Only two exist, and 149+ has none.
    const BLOCKS160: [(u8, u8); 2] = [(36, 50), (100, 114)];

    let is_2ghz = CHANNELS_2GHZ.contains(&control);
    match bw {
        Bandwidth::Bw20 => Some((control, mcu::CMD_CBW_20MHZ)),
        Bandwidth::Bw40 => {
            if is_2ghz {
                // 2.4 GHz has no fixed grid: the secondary sits above for low control channels
                // and below for high ones, which is the only way to stay inside 1..13.
                let centre = if control <= 7 {
                    control + 2
                } else {
                    control - 2
                };
                (1..=13)
                    .contains(&centre)
                    .then_some((centre, mcu::CMD_CBW_40MHZ))
            } else {
                PAIRS_5G
                    .iter()
                    .find(|&&lo| control == lo || control == lo + 4)
                    .map(|&lo| (lo + 2, mcu::CMD_CBW_40MHZ))
            }
        }
        Bandwidth::Bw80 => {
            if is_2ghz {
                return None; // no 80 MHz in 2.4 GHz, ever
            }
            BLOCKS80
                .iter()
                .find(|&&(lo, _)| {
                    (lo..lo + 16).contains(&control) && CHANNELS_5GHZ.contains(&control)
                })
                .map(|&(_, c)| (c, mcu::CMD_CBW_80MHZ))
        }
        // 5 and 10 MHz are narrowband modes this part's channel-switch does not expose the way
        // the Realtek ports do; refuse rather than silently tune 20.
        Bandwidth::Nb5 | Bandwidth::Nb10 => None,
    }
    .or_else(|| {
        // 160 MHz has no `Bandwidth` variant in the HAL, so it is unreachable through this seam
        // today. Left as a named table above rather than deleted: the MCU takes CMD_CBW_160MHZ,
        // and when the HAL grows the variant this is the two-line change.
        let _ = (&BLOCKS160, mcu::CMD_CBW_160MHZ);
        None
    })
}

pub struct Mt7921uBackend {
    /// EP0 register access, the endpoint map, and the bulk pipes.
    usb: Connac2Usb,
    /// The MCU command channel — sequence numbers plus the **latched response endpoint**.
    ///
    /// ⚠ That latch is the one genuinely undetermined thing in this stack (see
    /// [`mcu::Connac2Mcu`]): `mt76u_alloc_mcu_queue` puts the MCU RX queue on `0x85`, but
    /// `mt792xu_dma_rx_evt_ep4` sets `USB_RXEVT_EP4_EN`, whose name says events go to `0x84`
    /// — the data pipe. Nothing upstream settles it, so the first waited command finds out.
    /// Everything in this file that reads `0x84` (the bring-up drain, the RX pump) is
    /// arranged around the possibility that it lands there; see
    /// [`Mt7921uBackend::bring_up`] and [`Mt7921uBackend::spawn_rx_pump`].
    mcu: mcu::Connac2Mcu,
    /// The part's factory MAC address once the efuse has been read (needs firmware). `None`
    /// before [`Mt7921uBackend::bring_up`].
    ///
    /// Note this is *not* used to address frames — the named-radio doctrine forbids a host
    /// identity on air, and `addr2` carries a name-derived value or an ephemeral nonce. It
    /// is kept for logging and for the one place a hardware address is genuinely wanted:
    /// telling two identical dongles apart in a run header.
    mac_addr: Mutex<Option<[u8; 6]>>,
    /// Currently tuned channel (0 = never tuned).
    channel: AtomicU8,
    /// Currently tuned bandwidth as [`Bandwidth::code`]. Always 0 — see the module header.
    bw: AtomicU8,
    /// Whether [`Mt7921uBackend::setup_monitor_rx`] has run, so
    /// [`RadioKnobs::set_channel`] knows to re-send the sniffer's own copy of the channel.
    monitor: AtomicBool,
    /// The fixed rate every subsequent [`FrameIo::inject`] transmits at, or `None` to let
    /// the firmware's rate controller choose. Rate is bearer *state*, not a per-frame
    /// argument.
    cur_rate: Mutex<Option<mac::FixedRate>>,
    /// Wire frame format (the NDN ethertype by default).
    format: FrameFormat,
    /// 12-bit 802.11 sequence counter, for a future path that sets `MT_TXD3_SN_VALID`.
    seq: AtomicU16,
    /// The shared RX pipeline (queue + wake + pumped flag).
    rx: crate::rx_pump::RxPumpState,
    /// TX pump: when [`spawn_tx_pump`](Mt7921uBackend::spawn_tx_pump) has run, [`inject`] hands
    /// pre-built USB bulks to dedicated writer threads instead of awaiting one round trip per
    /// frame. See that method for why it exists and what it is worth.
    tx_sender: Mutex<Option<std::sync::mpsc::SyncSender<Vec<u8>>>>,
    /// Bulks and bytes the TX-pump threads have actually written — offered load, measured at
    /// the USB boundary rather than at the call site.
    tx_count: std::sync::atomic::AtomicU64,
    tx_bytes: std::sync::atomic::AtomicU64,
    /// The domain this device's LPON counter lives in — **one domain for both the per-frame
    /// RXD stamp and the read-now port TSF**, because they are the same physical counter.
    /// See [`RadioTime::time_sources`]. Per *device*, not per driver: two MT7921AUs on one
    /// host have unrelated counters and must never be differenced.
    tsf_domain: ClockDomainId,
    /// Wall-clock instant of the previous [`RadioKnobs::read_channel_activity`], because the
    /// MIB busy counter is read-and-clear and has no companion idle counter to normalise
    /// against — see that method.
    last_activity: Mutex<Instant>,
    // Software RX bookkeeping — see [`RxStats`].
    rx_units: AtomicU64,
    rx_stamped: AtomicU64,
    rx_fcs_errors: AtomicU64,
    rx_hdr_translated: AtomicU64,
    rx_amsdu_pad: AtomicU64,
    rx_mcu_events: AtomicU64,
    rx_undecodable: AtomicU64,
    rx_accepted: AtomicU64,
    /// The same accepted-unit count on its own window, so
    /// [`RadioKnobs::read_ofdm_counters`] and [`Mt7921uBackend::rx_stats_reset`] do not
    /// drain each other's counter — two consumers sampling at different cadences would
    /// otherwise each see a fraction of the traffic and neither would look wrong.
    rx_ok_window: AtomicU64,
}

/// Stops the bring-up bulk-IN drain on **every** exit path, including the error ones.
///
/// The same shape as the MT7610U's guard. A drain thread that outlives a failed bring-up
/// keeps reading `0x84` forever and silently competes with whatever runs next.
struct DrainGuard(Arc<AtomicBool>);

impl Drop for DrainGuard {
    fn drop(&mut self) {
        self.0.store(true, Ordering::Relaxed);
    }
}

impl Mt7921uBackend {
    // ── Open ────────────────────────────────────────────────────────────────

    /// Claim the first MT7921AU on the bus.
    ///
    /// **Never resets.** See the module header: a blind `handle.reset()` is what wedges
    /// these parts, and it is not needed — the warm-reopen guard in
    /// [`bring_up`](Self::bring_up), not a bus reset, is what keeps a second run from
    /// re-downloading firmware over a live MCU.
    pub fn open() -> Result<Self, FaceError> {
        Self::open_selected(DeviceSelect::from_env())
    }

    /// Claim a specific MT7921AU — by USB bus:port (`"1-1.4"`, stable across reboots) or by
    /// enumeration index (`"#1"`). The selector is how a node with two identical dongles
    /// pins the spare instead of stealing the one carrying a live kernel link; the transport
    /// runs [`crate::usb_select::check_live_link`] before claiming, and claims **interface 3
    /// only** so the Bluetooth function keeps `btusb`.
    pub fn open_selected(sel: DeviceSelect) -> Result<Self, FaceError> {
        let usb = Connac2Usb::open_selected(&sel)?;

        // Identity first, one EP0 round trip: everything downstream — the firmware blobs,
        // the descriptor layout, the capability — is specific to this die, and a stack that
        // proceeds past a wrong chip id fails later somewhere far less legible.
        // `mt792xu_check_bus` (`mt792x_usb.c:116-129`) does the same check.
        let chip = usb.check_bus()?;

        // One domain per physical device, matching the scheme the Realtek and ath9k backends
        // use so a mixed-radio node's domains cannot collide.
        let dev = usb.handle().device();
        let tsf_domain =
            ClockDomainId((u32::from(dev.bus_number()) << 8) | u32::from(dev.address()));

        tracing::info!(
            target: "named_radio",
            chip = "MT7921AU",
            usb_addr = usb.usb_addr(),
            chip_id = format_args!("{chip:#010x}"),
            domain = tsf_domain.0,
            "mt7921u: claimed (802.11ax, 2x2, per-frame RX timestamp)",
        );

        Ok(Self {
            usb,
            mcu: mcu::Connac2Mcu::new(),
            mac_addr: Mutex::new(None),
            channel: AtomicU8::new(0),
            bw: AtomicU8::new(0),
            monitor: AtomicBool::new(false),
            cur_rate: Mutex::new(None),
            format: FrameFormat::default(),
            seq: AtomicU16::new(0),
            rx: crate::rx_pump::RxPumpState::new(),
            tx_sender: Mutex::new(None),
            tx_count: std::sync::atomic::AtomicU64::new(0),
            tx_bytes: std::sync::atomic::AtomicU64::new(0),
            tsf_domain,
            last_activity: Mutex::new(Instant::now()),
            rx_units: AtomicU64::new(0),
            rx_stamped: AtomicU64::new(0),
            rx_fcs_errors: AtomicU64::new(0),
            rx_hdr_translated: AtomicU64::new(0),
            rx_amsdu_pad: AtomicU64::new(0),
            rx_mcu_events: AtomicU64::new(0),
            rx_undecodable: AtomicU64::new(0),
            rx_accepted: AtomicU64::new(0),
            rx_ok_window: AtomicU64::new(0),
        })
    }

    /// Set the wire frame format for [`FrameIo`] (defaults to the NDN ethertype).
    pub fn with_format(mut self, format: FrameFormat) -> Self {
        self.format = format;
        self
    }

    // ── Register access ─────────────────────────────────────────────────────

    /// Read a 32-bit register. ⚠ **268 µs** per call (MEASURED) — affordable in a channel
    /// switch or a sensing window, never on a per-frame path.
    pub fn rr(&self, addr: u32) -> Result<u32, FaceError> {
        self.usb.rr(addr)
    }

    /// Write a 32-bit register.
    pub fn wr(&self, addr: u32, val: u32) -> Result<(), FaceError> {
        self.usb.wr(addr, val)
    }

    /// Read-modify-write: clear `clear`, set `set`. Two round trips (~536 µs).
    pub fn rmw(&self, addr: u32, clear: u32, set: u32) -> Result<u32, FaceError> {
        self.usb.rmw(addr, clear, set)
    }

    /// The transport, for a caller that wants the endpoint map or a raw vendor request.
    pub fn usb(&self) -> &Connac2Usb {
        &self.usb
    }

    /// Which bulk IN endpoint the MCU's responses were last seen on, once determined.
    ///
    /// ★ Worth checking before starting the RX pump: if this is
    /// [`Connac2Usb::ep_in_data`] the pump and the MCU share `0x84`, and any later command
    /// (a channel switch, say) can have its response consumed by a pump thread.
    pub fn mcu_response_ep(&self) -> Option<u8> {
        self.mcu.response_ep()
    }

    /// The part's factory MAC address, if [`bring_up`](Self::bring_up) has read the efuse.
    pub fn mac_address(&self) -> Option<[u8; 6]> {
        *self.mac_addr.lock().unwrap_or_else(|e| e.into_inner())
    }

    // ── The warm-reopen guard ───────────────────────────────────────────────

    /// True when the MCU firmware is already up.
    ///
    /// ★ **This is the guard, and the register it reads is the whole point.**
    ///
    /// The MT7610U/MT7612U equivalent is `MT_MCU_COM_REG0 == 1`
    /// (`mt76x0/mcu.h:41-44`) — and `MT_MCU_COM_REG0` is a **mailbox**: the firmware writes
    /// other values into it in normal operation, so the guard reads `false` within seconds
    /// of a successful load. The next `open()` then takes the cold path and re-downloads
    /// firmware into a running MCU, which took the device off the USB bus and needed a
    /// physical replug three times this week.
    ///
    /// The connac2 answer is [`regs::MT_CONN_ON_MISC`]'s
    /// [`FW_N9_RDY`](regs::MT_TOP_MISC2_FW_N9_RDY) = `GENMASK(1,0)` = `FW_PWR_ON | FW_N9_ON`
    /// (`mt792x_regs.h:505-508`). That is **latched hardware state, not a message**:
    ///   * `FW_PWR_ON` is set by the `MT_VEND_POWER_ON` vendor request and cleared by a
    ///     subsystem reset (`mt792x_usb.c:214-231`);
    ///   * `FW_N9_ON` is set by the N9 core coming up and is what
    ///     [`mcu::run_firmware`](crate::connac2::mcu::run_firmware) polls for after
    ///     `FW_START_REQ` (`mt792x_core.c:1021-1025`);
    ///   * neither is a destination the firmware ever writes a *message* into. Nothing
    ///     clears them but a reset or a power cycle.
    ///
    /// So it cannot go stale in the direction that matters: it can only read `true` while
    /// firmware genuinely holds the chip. It is exactly the register upstream's own probe
    /// tests to decide whether to run a WFSYS reset (`mt7921/usb.c:218-222`), and MEASURED
    /// `0x0000_0000` on a cold plug here — so the guard has a known false state to compare
    /// against, which the mailbox never had.
    ///
    /// [`regs::MT_TOP_MISC`]'s `FW_STATE` field (`mt792x_regs.h:411-412`) is the second
    /// piece of evidence the plan called for, and it is read by
    /// [`firmware_state`](Self::firmware_state) — but it is **not** decisive here, and the
    /// reason is recorded in [`regs`]'s own Flag 4: `MT_TOP_MISC` is `0x1806_00f0` and has
    /// **never been read on this part**. The bench note that says "MT_TOP_MISC = 0" was
    /// actually a read of `0x7000_00f0`, an unnamed CB-TOP word. Gating a replug-risking
    /// decision on an address nobody has confirmed answers would be exactly the kind of
    /// inference this file exists to avoid.
    pub fn firmware_running(&self) -> bool {
        match self.usb.rr(regs::MT_CONN_ON_MISC) {
            Ok(v) => v & regs::MT_TOP_MISC2_FW_N9_RDY == regs::MT_TOP_MISC2_FW_N9_RDY,
            Err(_) => false,
        }
    }

    /// `(MT_CONN_ON_MISC, MT_TOP_MISC's FW_STATE)` — the corroborating read.
    ///
    /// ⚠ `MT_TOP_MISC` has never been read on this silicon (see
    /// [`firmware_running`](Self::firmware_running)); this exists so the *first* bring-up on
    /// the target can log it beside the register that is trusted, and settle whether it is a
    /// usable second witness. It decides nothing today.
    pub fn firmware_state(&self) -> Result<(u32, u32), FaceError> {
        let misc = self.usb.rr(regs::MT_CONN_ON_MISC)?;
        let top = self.usb.rr(regs::MT_TOP_MISC)?;
        Ok((misc, regs::field_get(regs::MT_TOP_MISC_FW_STATE, top)))
    }

    // ── Bring-up ────────────────────────────────────────────────────────────

    /// Full bring-up: power the chip, program the USB DMA engine, download the ROM patch and
    /// the RAM firmware, start it, and do the post-firmware MAC programming.
    ///
    /// Cold sequence, with upstream lines:
    /// 1. [`mcu::power_up`](crate::connac2::mcu::power_up) — chip-id check, then
    ///    `mt792xu_wfsys_reset` **only if firmware was already running**, then
    ///    `mt792xu_mcu_power_on` (`mt7921/usb.c:214-226`). ★ That order is easy to get
    ///    backwards: the reset clears `FW_PWR_ON`, so a power-on that ran first would be
    ///    undone and every later register access would read a chip that is off.
    /// 2. [`dma_init`](Self::dma_init) — `mt792xu_dma_init` (`mt792x_usb.c:393-422`).
    ///    Upstream runs this *between* power-on and `mcu_init`, and
    ///    [`mcu::run_firmware`](crate::connac2::mcu::run_firmware)'s own doc flags its
    ///    absence as making the download "not a clean experiment". It is implemented here
    ///    because it is bus programming, not MCU protocol.
    /// 3. [`mcu::run_firmware`](crate::connac2::mcu::run_firmware) — `SWDEF_NORMAL_MODE`,
    ///    `MT_FW_DL_EN`, `NIC_POWER_CTRL`, patch, RAM, `FW_START_REQ`, the `FW_N9_RDY` poll,
    ///    `MT_FW_DL_EN` off (`mt7921/usb.c:63-85` + `mt792x_core.c:980-1036`).
    /// 4. `mt7921_mcu_set_eeprom` (`mt7921/init.c:100`) — take calibration from the efuse.
    ///    Skip it and the PHY runs uncalibrated, which on the neighbouring Realtek parts is
    ///    exactly the "TX works, decode is marginal" failure that cost this bench weeks.
    /// 5. `mac_init` — `mt7921_mac_init` + `mt792x_mac_init_band`.
    /// 6. `mt76_connac_mcu_set_mac_enable` (`mt76_connac_mcu.c:216-231`).
    /// 7. Read the factory MAC out of the efuse (needs firmware).
    ///
    /// **Warm re-open.** If [`firmware_running`](Self::firmware_running) says the MCU already
    /// holds the chip, steps 1 and 3 are skipped and only the idempotent register/command
    /// programming re-runs. `NDN_RADIO_FORCE_FW=1` forces the cold path on a device you are
    /// willing to replug. Note upstream does **not** do this — `mt7921u_probe` resets and
    /// reloads unconditionally (`mt7921/usb.c:218-226`) — and the divergence is deliberate;
    /// see [`firmware_running`](Self::firmware_running).
    ///
    /// # ★ The bulk-IN drain, and why it does not cover the whole of bring-up
    ///
    /// On both older mt76 parts, if nobody reads the data bulk-IN the device's USB DMA backs
    /// up and **the MCU stops consuming inband commands** — they are accepted into the FIFO
    /// and never processed, so the *next* command's bulk-out times out rather than the
    /// offending one (MEASURED on the MT7610U as `command 0x0c seq 0 bulk-out (200 B):
    /// Operation timed out` partway through `init_hardware`, on a device whose registers
    /// were all answering). The MT7610U and MT7612U backends both carry a background drain
    /// for the duration of bring-up, with a `DrainGuard` that stops it on every error path.
    /// That pattern is copied here — with one **deliberate difference, stated loudly**:
    ///
    /// The drain starts **after** the firmware download, not before it. On the mt76x0 the
    /// drain is unconditionally safe because the command-response pipe (`0x85`) is a
    /// different pipe from the data pipe (`0x84`). On connac2 that is *undetermined*: the
    /// MCU response endpoint is discovered by [`mcu::Connac2Mcu`] on the first waited
    /// command, and if it lands on `0x84` a drain reading `0x84` eats responses. So:
    ///
    ///   * **Phase A — power-up, DMA init, firmware download: no drain.** Nothing is being
    ///     received here (no channel, no MAC enable, and `MT_FW_DL_EN` has AC_BE wired to
    ///     the firmware queue), so the pipe has nothing to back up *with*; and this is
    ///     precisely the phase whose first waited command determines the latch. Upstream
    ///     submits no RX URBs during download either.
    ///   * **Phase B — eeprom, MAC init, mac-enable and everything after: drain, but only
    ///     if the latch came out on `0x85`.** That is the mt76x0-equivalent situation and
    ///     the phase where the older parts actually stalled. If the latch is `0x84` the
    ///     drain is skipped and a warning is logged, because reading that pipe would break
    ///     every subsequent command instead of protecting it.
    ///
    /// The MEASURED failure mode is a *stalled command*, which is loud. Silently eating an
    /// MCU response is quiet. Given a choice between the two, the loud one wins.
    pub fn bring_up(&self) -> Result<(), FaceError> {
        let force = std::env::var_os("NDN_RADIO_FORCE_FW").is_some();
        let warm = !force && self.firmware_running();

        if warm {
            tracing::info!(
                target: "named_radio",
                chip = "MT7921AU",
                "firmware already running (MT_CONN_ON_MISC & FW_N9_RDY) — warm re-open, \
                 skipping the power-up and the firmware download",
            );
            // ★ `resume = true`, which is upstream's own parameter and not a shortcut:
            // `mt792xu_dma_init(dev, true)` re-applies the WFDMA/UDMA register values (they
            // are values, not a sequence, so this is idempotent) and **stops before**
            // `mt792xu_dma_rx_evt_ep4` and `mt792xu_epctl_rst_opt` (`mt792x_usb.c:411-421`).
            // That matters here: `dma_rx_evt_ep4` drops and re-raises `RX_DMA_EN` on a chip
            // whose firmware is live and may be mid-transfer, which is exactly the class of
            // disturbance a warm re-open exists to avoid.
            self.dma_init(true)?;
        } else {
            let chip = mcu::power_up(&self.usb)?;
            self.dma_init(false)?;
            mcu::run_firmware(
                &self.mcu,
                &self.usb,
                chip.hw_rev,
                mcu::MT7961_PATCH,
                mcu::MT7961_RAM,
            )?;
            tracing::info!(
                target: "named_radio",
                chip = "MT7921AU",
                hw_rev = format_args!("{:#010x}", chip.hw_rev),
                was_running = chip.was_running,
                mcu_response_ep = ?self.mcu.response_ep().map(|e| format!("{e:#04x}")),
                "mt7921u: firmware up",
            );
        }

        // ── Phase B: the drain may run from here (see the doc above) ────────
        let _drain = self.spawn_bringup_drain();

        mcu::set_eeprom_efuse_mode(&self.mcu, &self.usb)?;
        self.mac_init()?;
        mcu::set_mac_enable(&self.mcu, &self.usb, 0, true)?;

        // The efuse read needs firmware, so it lives here and not in `open`. Not fatal:
        // this address is never put on air (the named-radio doctrine forbids a host identity
        // in the source field), so a failure costs a log line, not a radio.
        match mcu::read_mac_addr(&self.mcu, &self.usb) {
            Ok(mac) => {
                *self.mac_addr.lock().unwrap_or_else(|e| e.into_inner()) = Some(mac);
                tracing::info!(
                    target: "named_radio",
                    chip = "MT7921AU",
                    mac = format_args!(
                        "{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
                        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]
                    ),
                    "mt7921u: factory MAC (diagnostic only — never transmitted)",
                );
            }
            Err(e) => tracing::warn!(
                target: "named_radio",
                chip = "MT7921AU",
                error = %e,
                "mt7921u: could not read the factory MAC from the efuse; continuing",
            ),
        }
        Ok(())
    }

    /// Start the bring-up bulk-IN drain, if it is safe to (see [`bring_up`](Self::bring_up)).
    ///
    /// Returns a guard whose `Drop` stops the thread — on the success path and on every `?`
    /// in the caller alike. A drain thread that outlives a failed bring-up would keep reading
    /// `0x84` for the life of the process and compete with whatever ran next.
    ///
    /// [`Connac2Usb::spawn_rx_drain`] exists and does almost this, but its thread runs for
    /// the life of the process and is only *pausable*; a bring-up wants one that genuinely
    /// stops, so this is the mt76x0's local-thread-plus-stop-flag shape instead.
    fn spawn_bringup_drain(&self) -> Option<DrainGuard> {
        let data_ep = self.usb.ep_in_data();
        match self.mcu.response_ep() {
            Some(ep) if ep == data_ep => {
                tracing::warn!(
                    target: "named_radio",
                    chip = "MT7921AU",
                    ep = format_args!("{ep:#04x}"),
                    "mt7921u: the MCU latched its response endpoint onto the DATA pipe \
                     (USB_RXEVT_EP4_EN) — skipping the bring-up RX drain, because reading \
                     that pipe would consume MCU responses. Do not start the RX pump while \
                     commands are in flight on this device.",
                );
                return None;
            }
            None => {
                // Not yet determined — only reachable on a warm re-open, where no command has
                // been waited on yet. Draining could eat the very response that determines
                // the latch, and the commands that follow are few.
                tracing::debug!(
                    target: "named_radio",
                    chip = "MT7921AU",
                    "mt7921u: MCU response endpoint not yet latched — no bring-up drain",
                );
                return None;
            }
            Some(_) => {}
        }

        let stop = Arc::new(AtomicBool::new(false));
        let flag = stop.clone();
        let handle = self.usb.handle();
        std::thread::spawn(move || {
            let mut buf = vec![0u8; RX_BUF_LEN];
            while !flag.load(Ordering::Relaxed) {
                let _ = handle.read_bulk(data_ep, &mut buf, Duration::from_millis(50));
            }
        });
        Some(DrainGuard(stop))
    }

    /// `mt792xu_dma_init` (`mt792x_usb.c:393-422`) — the USB DMA engine.
    ///
    /// Not in [`mcu`](crate::connac2::mcu) because it is bus programming rather than MCU
    /// protocol; that module's [`run_firmware`](crate::connac2::mcu::run_firmware) carries a
    /// ⚠ noting its absence, and this is what closes it. ~45 register operations at 268 µs
    /// each ≈ **12 ms**.
    ///
    /// In order:
    /// 1. `mt792xu_wfdma_init` (`:279-315`) — prefetch config, `MT_UWFDMA0_GLO_CFG`, and the
    ///    DMA scheduler tables.
    /// 2. `MT_UDMA_WLCFG_0`: clear `RX_FLUSH`, then set `RX_EN | TX_EN | RX_MPSZ_PAD0 |
    ///    TICK_1US_EN` (`:398-403`). ★ **`MT_WL_RX_AGG_EN` (bit 21) is not set** — see
    ///    `parse_transfer`.
    /// 3. TX timeout limit + `TX_TMOUT_FUNC_EN`, then clear every RX-aggregation knob
    ///    (`:404-410`).
    /// 4. `mt792xu_dma_rx_evt_ep4` (`:318-330`) — quiesce RX DMA, set
    ///    `USB_RXEVT_EP4_EN`, re-enable. ★ This is the bit that (by its name) routes firmware
    ///    **events** onto the data pipe; see [`mcu::Connac2Mcu`].
    /// 5. `mt792xu_epctl_rst_opt(false)` (`:333-348`) — clear the endpoint-reset options for
    ///    bulk OUT 4-9 and bulk IN 4-6.
    ///
    /// `resume` is upstream's own parameter (`mt792xu_dma_init(dev, bool resume)`,
    /// `:411-421`): when true, steps 4 and 5 are **skipped**. Pass `true` on a warm re-open,
    /// where the chip is live and dropping `RX_DMA_EN` under running firmware is a
    /// disturbance with nothing to gain — see [`bring_up`](Self::bring_up).
    pub fn dma_init(&self, resume: bool) -> Result<(), FaceError> {
        self.wfdma_init()?;

        self.usb
            .clear_bits(regs::MT_UDMA_WLCFG_0, regs::MT_WL_RX_FLUSH)?;
        self.usb.set_bits(
            regs::MT_UDMA_WLCFG_0,
            regs::MT_WL_RX_EN | regs::MT_WL_TX_EN | regs::MT_WL_RX_MPSZ_PAD0 | regs::MT_TICK_1US_EN,
        )?;
        self.usb.rmw(
            regs::MT_UDMA_WLCFG_1,
            regs::MT_WL_TX_TMOUT_LMT,
            regs::field_prep(regs::MT_WL_TX_TMOUT_LMT, regs::MT792X_USB_TX_TIMEOUT_LIMIT),
        )?;
        self.usb
            .set_bits(regs::MT_UDMA_WLCFG_0, regs::MT_WL_TX_TMOUT_FUNC_EN)?;
        self.usb.clear_bits(
            regs::MT_UDMA_WLCFG_0,
            regs::MT_WL_RX_AGG_TO | regs::MT_WL_RX_AGG_LMT,
        )?;
        self.usb
            .clear_bits(regs::MT_UDMA_WLCFG_1, regs::MT_WL_RX_AGG_PKT_LMT)?;

        if resume {
            return Ok(());
        }
        self.dma_rx_evt_ep4()?;
        self.epctl_rst_opt(false)?;
        Ok(())
    }

    /// `mt792xu_wfdma_init` (`mt792x_usb.c:279-315`), including its `mt792xu_dma_prefetch`
    /// (`:261-277`).
    ///
    /// Every value here is upstream's and upstream states a reason for none of them — the
    /// prefetch depths, the group quotas, the queue maps and the two scheduler words are
    /// magic constants in the tree. Ported faithfully; meanings undetermined.
    fn wfdma_init(&self) -> Result<(), FaceError> {
        // DMA_PREFETCH_CONF(idx, cnt, base) — `mt792x_usb.c:262-266`, applied to rings
        // 0-4, 16 and 17 (`:268-275`).
        for (idx, base) in [
            (0u32, 0x080u32),
            (1, 0x0c0),
            (2, 0x100),
            (3, 0x140),
            (4, 0x180),
            (16, 0x280),
            (17, 0x2c0),
        ] {
            self.usb.rmw(
                regs::mt_uwfdma0_tx_ring_ext_ctrl(idx),
                regs::MT_WPDMA0_MAX_CNT_MASK | regs::MT_WPDMA0_BASE_PTR_MASK,
                regs::field_prep(regs::MT_WPDMA0_MAX_CNT_MASK, 4)
                    | regs::field_prep(regs::MT_WPDMA0_BASE_PTR_MASK, base),
            )?;
        }

        self.usb.clear_bits(
            regs::MT_UWFDMA0_GLO_CFG,
            regs::MT_WFDMA0_GLO_CFG_OMIT_RX_INFO,
        )?;
        self.usb.set_bits(
            regs::MT_UWFDMA0_GLO_CFG,
            regs::MT_WFDMA0_GLO_CFG_OMIT_TX_INFO
                | regs::MT_WFDMA0_GLO_CFG_OMIT_RX_INFO_PFET2
                | regs::MT_WFDMA0_GLO_CFG_FW_DWLD_BYPASS_DMASHDL
                | regs::MT_WFDMA0_GLO_CFG_TX_DMA_EN
                | regs::MT_WFDMA0_GLO_CFG_RX_DMA_EN,
        )?;

        self.usb.rmw(
            regs::MT_DMASHDL_REFILL,
            regs::MT_DMASHDL_REFILL_MASK,
            0xffe0_0000,
        )?;
        self.usb
            .clear_bits(regs::MT_DMASHDL_PAGE, regs::MT_DMASHDL_GROUP_SEQ_ORDER)?;
        self.usb.rmw(
            regs::MT_DMASHDL_PKT_MAX_SIZE,
            regs::MT_DMASHDL_PKT_MAX_SIZE_PLE | regs::MT_DMASHDL_PKT_MAX_SIZE_PSE,
            regs::field_prep(regs::MT_DMASHDL_PKT_MAX_SIZE_PLE, 1)
                | regs::field_prep(regs::MT_DMASHDL_PKT_MAX_SIZE_PSE, 0),
        )?;
        // Groups 0-4 get a real quota; 5-15 get none (`:298-303`).
        for i in 0..5u32 {
            self.usb.wr(
                regs::mt_dmashdl_group_quota(i),
                regs::field_prep(regs::MT_DMASHDL_GROUP_QUOTA_MIN, 0x3)
                    | regs::field_prep(regs::MT_DMASHDL_GROUP_QUOTA_MAX, 0xfff),
            )?;
        }
        for i in 5..16u32 {
            self.usb.wr(regs::mt_dmashdl_group_quota(i), 0)?;
        }
        self.usb.wr(regs::mt_dmashdl_q_map(0), 0x3201_3201)?;
        self.usb.wr(regs::mt_dmashdl_q_map(1), 0x3201_3201)?;
        self.usb.wr(regs::mt_dmashdl_q_map(2), 0x5555_5444)?;
        self.usb.wr(regs::mt_dmashdl_q_map(3), 0x5555_5444)?;
        self.usb.wr(regs::mt_dmashdl_sched_set(0), 0x7654_0132)?;
        self.usb.wr(regs::mt_dmashdl_sched_set(1), 0xFEDC_BA98)?;

        self.usb
            .set_bits(regs::MT_WFDMA_DUMMY_CR, regs::MT_WFDMA_NEED_REINIT)?;
        Ok(())
    }

    /// `mt792xu_dma_rx_evt_ep4` (`mt792x_usb.c:318-330`): wait for RX DMA to go idle, drop
    /// `RX_DMA_EN`, set `MT_WFDMA_HOST_CONFIG_USB_RXEVT_EP4_EN`, re-enable.
    ///
    /// ★ The consequence of that bit is the one open question in this stack — see
    /// [`mcu::Connac2Mcu`]. Its name says RX **events** go to EP**4** (`0x84`, the data
    /// pipe); upstream's own MCU queue is allocated on `0x85`; nothing in the tree resolves
    /// the contradiction. Set anyway, because that is what upstream does and a driver that
    /// silently diverges from the reference on a DMA-routing bit is not a port.
    fn dma_rx_evt_ep4(&self) -> Result<(), FaceError> {
        self.usb.poll(
            regs::MT_UWFDMA0_GLO_CFG,
            regs::MT_WFDMA0_GLO_CFG_RX_DMA_BUSY,
            0,
            1_000,
        )?;
        self.usb
            .clear_bits(regs::MT_UWFDMA0_GLO_CFG, regs::MT_WFDMA0_GLO_CFG_RX_DMA_EN)?;
        self.usb.set_bits(
            regs::MT_WFDMA_HOST_CONFIG,
            regs::MT_WFDMA_HOST_CONFIG_USB_RXEVT_EP4_EN,
        )?;
        self.usb
            .set_bits(regs::MT_UWFDMA0_GLO_CFG, regs::MT_WFDMA0_GLO_CFG_RX_DMA_EN)?;
        Ok(())
    }

    /// `mt792xu_epctl_rst_opt` (`mt792x_usb.c:333-348`) — the USB endpoint reset options for
    /// bulk OUT 4-9, bulk IN 4-5 and interrupt IN 6.
    ///
    /// ⚠ **UHW path**, not the ordinary register window: `MT_SSUSB_EPCTL_CSR_EP_RST_OPT`
    /// lives in the SSUSB block and is only reachable through `MT_VEND_DEV_MODE` /
    /// `MT_VEND_WRITE` with the `0x5e`-family `bmRequestType`
    /// ([`Connac2Usb::uhw_rr`]/[`Connac2Usb::uhw_wr`]). Reaching it with the normal
    /// `READ_EXT`/`WRITE_EXT` pair reads a different address space.
    ///
    /// This is **not** a USB reset and touches no `libusb` reset path: it only clears the
    /// chip's *option* bits saying which endpoints a future reset would take with it.
    /// `mt792xu_dma_init` calls it with `reset = false` (`:420`).
    fn epctl_rst_opt(&self, reset: bool) -> Result<(), FaceError> {
        let bits = regs::MT_SSUSB_EPCTL_RST_OPT_OUT_EP | regs::MT_SSUSB_EPCTL_RST_OPT_IN_EP;
        let v = self.usb.uhw_rr(regs::MT_SSUSB_EPCTL_CSR_EP_RST_OPT)?;
        let v = if reset { v | bits } else { v & !bits };
        self.usb.uhw_wr(regs::MT_SSUSB_EPCTL_CSR_EP_RST_OPT, v)
    }

    /// `mt7921_mac_init` (`mt7921/init.c:64-81`) + `mt792x_mac_init_band(0)`
    /// (`mt792x_mac.c:284-311`), with the **three monitor-mode departures** below.
    ///
    /// ★ **1. `MT_MDP_DCR0_RX_HDR_TRANS_EN` is CLEARED, not set.** `mt7921/init.c:72` sets
    /// it ("enable hardware rx header translation") and with it set the hardware rewrites
    /// every data frame's 802.11 header into an 802.3 one before the host sees it:
    /// [`mac::Rxd::hdr_trans`] goes true and there is no 802.11 header left to parse.
    /// mt7915 clears it for monitor mode for exactly this reason (`mt7915/main.c:509`). A
    /// named-radio face that leaves it set receives Ethernet frames and reports **zero** NDN
    /// frames, which reads identically to a dead receiver.
    ///
    /// ★ **2. `MT_DMA_DCR0_RXD_G5_EN` is SET, not cleared.** `mt792x_mac.c:302-304` clears
    /// it — *"disable rx rate report by default due to hw issues"* — and with it clear there
    /// is no RXD group 5, so per-chain RCPI comes from P-RXV DW1 instead of the C-RXV.
    /// Upstream's own comment at `mt7921/mac.c:356-358` says monitor mode wants the group-5
    /// copy. Both paths work ([`mac::Rxd::rcpi_from_crxv`] says which produced the numbers);
    /// this port takes the one upstream recommends for monitors. The "hw issues" upstream
    /// cites are unstated, so if RSSI ever looks wrong this is the first bit to flip back.
    ///
    /// ★ **3. The 20-entry WTBL admission-count wipe (`init.c:76-78`) is skipped.** ~20 MCU
    /// register cycles for a station table this driver never reads: TX uses the global WCID
    /// 0 and RX is promiscuous. Skipped and said so, rather than silently dropped.
    ///
    /// Everything else is upstream's, including `mt792x_mac_init_band`'s MIB arming (which
    /// is what makes [`RadioKnobs::read_channel_activity`] return a real number instead of a
    /// confident zero) and its `RCPI_MODE`/`RCPI_PARAM` write, whose comment upstream is
    /// *"filter out non-resp frames and get instantaneous signal reporting"*.
    ///
    /// Band 1 is not initialised: `mt7921_mac_init` loops `for (i = 0; i < 2; i++)` but this
    /// part has one PHY — [`mac::parse_rxd`] rejects any descriptor with `BAND_IDX` set — so
    /// the second pass programs a block that does not exist.
    fn mac_init(&self) -> Result<(), FaceError> {
        // `mt7921_mac_init` (`init.c:68-73`).
        self.usb.rmw(
            regs::MT_MDP_DCR1,
            regs::MT_MDP_DCR1_MAX_RX_LEN,
            regs::field_prep(regs::MT_MDP_DCR1_MAX_RX_LEN, 1536),
        )?;
        self.usb
            .set_bits(regs::MT_MDP_DCR0, regs::MT_MDP_DCR0_DAMSDU_EN)?;
        // ★ Departure 1.
        self.usb
            .clear_bits(regs::MT_MDP_DCR0, regs::MT_MDP_DCR0_RX_HDR_TRANS_EN)?;

        self.mac_init_band(0)?;
        Ok(())
    }

    /// `mt792x_mac_init_band` (`mt792x_mac.c:284-311`), with departure 2 above.
    fn mac_init_band(&self, band: u32) -> Result<(), FaceError> {
        self.usb.rmw(
            regs::mt_tmac_ctcr0(band),
            regs::MT_TMAC_CTCR0_INS_DDLMT_REFTIME,
            regs::field_prep(regs::MT_TMAC_CTCR0_INS_DDLMT_REFTIME, 0x3f),
        )?;
        self.usb.set_bits(
            regs::mt_tmac_ctcr0(band),
            regs::MT_TMAC_CTCR0_INS_DDLMT_VHT_SMPDU_EN | regs::MT_TMAC_CTCR0_INS_DDLMT_EN,
        )?;

        // Arm the RX-time MIB. Free (two register writes) and the radio's only frame-free
        // occupancy sense; leaving it unarmed makes `read_channel_activity` return a
        // confident 0 per mille, which reads as "quiet channel" and is not. The MT7610U was
        // MEASURED in exactly that state before its equivalent call was added.
        self.usb.set_bits(
            regs::mt_wf_rmac_mib_time0(band),
            regs::MT_WF_RMAC_MIB_RXTIME_EN,
        )?;
        self.usb.set_bits(
            regs::mt_wf_rmac_mib_airtime0(band),
            regs::MT_WF_RMAC_MIB_RXTIME_EN,
        )?;
        self.usb
            .set_bits(regs::mt_mib_scr1(band), regs::MT_MIB_TXDUR_EN)?;
        self.usb
            .set_bits(regs::mt_mib_scr1(band), regs::MT_MIB_RXDUR_EN)?;

        self.usb.rmw(
            regs::mt_dma_dcr0(band),
            regs::MT_DMA_DCR0_MAX_RX_LEN,
            regs::field_prep(regs::MT_DMA_DCR0_MAX_RX_LEN, 1536),
        )?;
        // ★ Departure 2: SET, where upstream clears.
        self.usb
            .set_bits(regs::mt_dma_dcr0(band), regs::MT_DMA_DCR0_RXD_G5_EN)?;

        self.usb.rmw(
            regs::mt_wtbloff_top_rscr(band),
            regs::MT_WTBLOFF_TOP_RSCR_RCPI_MODE | regs::MT_WTBLOFF_TOP_RSCR_RCPI_PARAM,
            regs::field_prep(regs::MT_WTBLOFF_TOP_RSCR_RCPI_MODE, 0)
                | regs::field_prep(regs::MT_WTBLOFF_TOP_RSCR_RCPI_PARAM, 0x3),
        )?;
        Ok(())
    }

    /// `mt792x_mac_set_timeing` (`mt792x_mac.c:35-75`; the spelling is upstream's) — the
    /// CCK/OFDM detection timeouts and the IFS block, both of which are **band-dependent**:
    /// SIFS is 10 µs on 2.4 GHz and 16 µs on 5 GHz.
    ///
    /// Called from the channel switch because that is where upstream calls it
    /// (`mt7921/main.c:485`), and it matters for TX rather than RX: an IFS block left at the
    /// wrong band's values makes this transmitter's inter-frame spacing disagree with
    /// everyone else's on the channel.
    ///
    /// `coverage_class = 0` (no long-distance offset) and `slottime = 9` (short slot). The
    /// slot value is mac80211's default rather than upstream's own — `phy->slottime` is set
    /// from `bss_conf.use_short_slot` and a monitor has no BSS to read it from, so 9 is a
    /// choice this port makes and states rather than a constant it inherited.
    fn set_mac_timing(&self, is_2ghz: bool) -> Result<(), FaceError> {
        const SLOTTIME: u32 = 9;
        let band = 0u32;
        let cck = regs::field_prep(regs::MT_TIMEOUT_VAL_PLCP, 231)
            | regs::field_prep(regs::MT_TIMEOUT_VAL_CCA, 48);
        let ofdm = regs::field_prep(regs::MT_TIMEOUT_VAL_PLCP, 60)
            | regs::field_prep(regs::MT_TIMEOUT_VAL_CCA, 28);
        let sifs = if is_2ghz { 10 } else { 16 };

        // Quiesce the arbiter across the update, exactly as upstream does (`:50-52`): these
        // are live timing registers and a partially-applied IFS block is a transmitter
        // nobody can hear cleanly.
        self.usb.set_bits(
            regs::mt_arb_scr(band),
            regs::MT_ARB_SCR_TX_DISABLE | regs::MT_ARB_SCR_RX_DISABLE,
        )?;

        let result = (|| -> Result<(), FaceError> {
            self.usb.wr(regs::mt_tmac_cdtr(band), cck)?;
            self.usb.wr(regs::mt_tmac_odtr(band), ofdm)?;
            self.usb.wr(
                regs::mt_tmac_icr0(band),
                regs::field_prep(regs::MT_IFS_EIFS, 360)
                    | regs::field_prep(regs::MT_IFS_RIFS, 2)
                    | regs::field_prep(regs::MT_IFS_SIFS, sifs)
                    | regs::field_prep(regs::MT_IFS_SLOT, SLOTTIME),
            )?;
            // `:66-71`: the CF-End rate goes to the 11b value only on a long-slot 2.4 GHz
            // channel. With SLOTTIME 9 that is never, but the branch is ported rather than
            // folded away, because the constant above is this port's choice and not a fact.
            let cfend = if SLOTTIME < 20 || !is_2ghz {
                regs::MT792X_CFEND_RATE_DEFAULT
            } else {
                regs::MT792X_CFEND_RATE_11B
            };
            self.usb.rmw(
                regs::mt_agg_acr0(band),
                regs::MT_AGG_ACR_CFEND_RATE,
                regs::field_prep(regs::MT_AGG_ACR_CFEND_RATE, cfend),
            )?;
            Ok(())
        })();

        // Release the arbiter on both paths: leaving TX_DISABLE|RX_DISABLE set turns a
        // failed tune into a radio that is silently deaf and mute.
        let release = self.usb.clear_bits(
            regs::mt_arb_scr(band),
            regs::MT_ARB_SCR_TX_DISABLE | regs::MT_ARB_SCR_RX_DISABLE,
        );
        result.and(release.map(|_| ()))
    }

    // ── Monitor RX ──────────────────────────────────────────────────────────

    /// Put the part into promiscuous monitor receive.
    ///
    /// In order:
    /// 1. `mt7921_mcu_set_sniffer(enable)` (`mt7921/mcu.c:1151-1178`) — the actual
    ///    monitor-mode switch; mac80211 reaches it from `mt7921_config` on
    ///    `IEEE80211_CONF_CHANGE_MONITOR` (`mt7921/main.c:610`).
    /// 2. `mt7921_mcu_config_sniffer` (`:1181-1247`) — the sniffer's **own copy** of the
    ///    channel. The PHY retune ([`RadioKnobs::set_channel`]) is not sufficient on its own.
    /// 3. `mt7921_mcu_set_rxfilter` (`:1477-1497`) with everything through.
    ///
    /// # Why the RX filter goes through the MCU and not `MT_WF_RFCR`
    ///
    /// `MT_WF_RFCR` is a real register at `0x820e5000` and `MT_VEND_WRITE_EXT` reaches it,
    /// so writing the drop bits directly looks like one EP0 write instead of a bulk command.
    /// It is wrong here for one reason: **the firmware owns that register** and rewrites it
    /// on every channel switch, sniffer enable and BSS update (`mt7921/mcu.c:1106-1123`
    /// reaches for the same command just to flip `DROP_OTHER_BEACON`). A host-side write
    /// would be silently reverted at the next firmware event — RX that works until you
    /// retune, which is the worst failure shape there is.
    ///
    /// # ★ FCS-failed frames come through, and are dropped in software
    ///
    /// Default is [`mcu::RX_FILTER_PROMISCUOUS`] (`ENABLE | FCSFAIL | CONTROL | OTHER_BSS`)
    /// with the sniffer's `drop_err = 0`. Dropping bad-FCS frames *in hardware* makes a
    /// marginal link and a quiet channel look identical, which is the exact ambiguity the
    /// occupancy counters exist to resolve; carrying the verdict per frame
    /// ([`mac::Rxd::fcs_err`]) lets `parse_transfer` **count** them and then drop them.
    ///
    /// ⚠ And that is precisely the trap the LR2021 testbed spent a campaign inside — every
    /// on-air result there turned out to be CRC-failing frames. The rule here: the *counter*
    /// is evidence about the channel, the *payload path* never sees one.
    /// `NDN_RADIO_RX_STRICT=1` switches to [`mcu::RX_FILTER_PROMISCUOUS_VALID`] and
    /// `drop_err = 1` (upstream's own setting) for a run where that must be true in hardware
    /// too.
    ///
    /// # ★ What makes RXD group 2 — the timestamp — present
    ///
    /// **Nothing does, and that is the honest answer.** The *only* RXD-group gate anywhere
    /// in the upstream tree is `MT_DMA_DCR0_RXD_G5_EN` for group 5 (`mt792x_mac.c:304`,
    /// `mt7915/main.c:507`); groups 1-4 have no enable register. The evidence points to a
    /// per-frame, content-dependent bitmap — group 1 tracks `SEC_MODE`, group 4 tracks
    /// header translation — with group 2 apparently always on for a normal frame.
    ///
    /// So this method does the two things that are actually in its power, and then *watches*:
    ///   * it leaves header translation **off** (`mac_init` departure 1),
    ///     since group 4's presence tracks it and a translated frame is a different
    ///     descriptor shape;
    ///   * it turns group 5 **on**, which is the one group bit that is settable at all, so a
    ///     later "group N is missing" question can be asked against a known configuration;
    ///   * and `parse_transfer` counts [`RxStats::stamped`] separately from
    ///     [`RxStats::units`], so a missing timestamp is **visible in one number** instead of
    ///     silently absent. [`rx_health`](Self::rx_health) prints both.
    ///
    /// If a capture ever shows group 2 missing, the place to look is the firmware
    /// `CHIP_CONFIG` / RX-header-translation MCU commands, not a register in this file.
    pub fn setup_monitor_rx(&self) -> Result<(), FaceError> {
        let ch = self.channel.load(Ordering::Relaxed);
        if ch == 0 {
            return Err(io_err(
                "mt7921u: setup_monitor_rx before set_channel — the sniffer carries its own \
                 copy of the channel and has nothing to be told"
                    .into(),
            ));
        }
        let strict = std::env::var_os("NDN_RADIO_RX_STRICT").is_some();

        mcu::set_sniffer(&self.mcu, &self.usb, 0, true)?;
        // Carry the width the radio is actually tuned to, not a guess — see config_sniffer.
        self.config_sniffer(
            ch,
            Bandwidth::from_code(self.bw.load(Ordering::Relaxed)),
            strict,
        )?;
        let fif = if strict {
            mcu::RX_FILTER_PROMISCUOUS_VALID
        } else {
            mcu::RX_FILTER_PROMISCUOUS
        };
        mcu::set_rx_filter(&self.mcu, &self.usb, fif, 0, 0)?;
        self.monitor.store(true, Ordering::Relaxed);

        if self.mcu.response_ep() == Some(self.usb.ep_in_data()) {
            tracing::warn!(
                target: "named_radio",
                chip = "MT7921AU",
                "mt7921u: MCU responses share the RX data pipe (0x84). Any MCU command \
                 issued while the RX pump runs — a channel switch, a filter change — can \
                 have its response consumed by a pump thread. Check RxStats::mcu_events.",
            );
        }
        Ok(())
    }

    /// Send the sniffer's copy of the channel — `mt7921_mcu_config_sniffer`
    /// (`mt7921/mcu.c:1181-1247`).
    ///
    /// ⚠ The `ch_band` and `bw` encodings here are **not** the `CMD_CBW_*` / `CHANNEL_BAND_*`
    /// ones used by the PHY retune: this command wants `2GHZ → 1, 5GHZ → 2` and
    /// `20/40 MHz → 0`. Two commands, two encodings, one channel — see
    /// [`mcu::SnifferChan`]. Getting them crossed tunes the PHY correctly and points the MAC
    /// somewhere else.
    ///
    /// ★★ **This command runs AFTER the PHY retune and silently overrides it.** MEASURED
    /// 2026-08-27: with `set_channel(36, Bw80)` sending `CMD_CBW_80MHZ` and a derived centre of
    /// 42, and with the TXD carrying `MT_TXD6_BW = 2`, a witness radiotap still reported every
    /// PPDU as **`MCS 8 ... 20 MHz`** — because this function hard-coded `bw: 0` and
    /// `center_ch: control_ch`, and being the last writer it won. The register writes were all
    /// correct and the air disagreed; only the witness settled it.
    ///
    /// The same shape of bug as the MT7612U's channel replay overwriting its own RX filter:
    /// a later configuration step quietly undoing an earlier one, invisible from the host side.
    /// Take the width from the caller so the two cannot disagree.
    fn config_sniffer(&self, channel: u8, bw: Bandwidth, strict: bool) -> Result<(), FaceError> {
        // This command's `bw` is NOT the CMD_CBW_* encoding — see the note above. Upstream's
        // `mt7921_mcu_config_sniffer` uses its own small enum where 0 = 20/40 MHz, 1 = 80,
        // 2 = 160, 3 = 80+80 (`mt7921/mcu.c:1181-1247`).
        let (center_ch, sniffer_bw) = match bw {
            Bandwidth::Bw80 => (centre_and_cbw(channel, bw).map_or(channel, |(c, _)| c), 1),
            Bandwidth::Bw40 => (centre_and_cbw(channel, bw).map_or(channel, |(c, _)| c), 0),
            _ => (channel, 0),
        };
        let chan = mcu::SnifferChan {
            band_idx: 0,
            ch_band: if channel <= 14 { 1 } else { 2 },
            bw: sniffer_bw,
            control_ch: channel,
            center_ch,
            center_ch2: 0,
            // Upstream hardcodes 1 (`mt7921/mcu.c:1255`); see the FCS note on
            // `setup_monitor_rx`. 0 is a departure and is UNVALIDATED.
            drop_err: u8::from(strict),
        };
        mcu::config_sniffer(&self.mcu, &self.usb, &chan)
    }

    /// A one-line summary of the registers and counters that decide whether this radio can
    /// hear anything — printed by the bring-up gate, and cheap enough (9 EP0 reads ≈ 2.4 ms)
    /// to call after any state change.
    ///
    /// A silently-off receiver looks exactly like a quiet channel, and on this part there
    /// are five independent ways to be off. In order of how often they bite:
    ///   * `MDP_DCR0.HDR_TRANS` set → frames arrive as **802.3** and every NDN parse fails;
    ///   * `WLCFG_0` without `RX_EN` → the USB DMA never delivers;
    ///   * `UWFDMA0_GLO_CFG` without `RX_DMA_EN` → the MAC never reaches the USB DMA;
    ///   * `ARB_SCR.RX_DISABLE` set → the arbiter is holding the receiver down (a
    ///     `set_mac_timing` that failed halfway leaves it here);
    ///   * `CONN_ON_MISC` without `FW_N9_RDY` → there is no firmware and nothing is running.
    ///
    /// ★ It also prints `stamped/units`, which is the health of the *headline feature*: if
    /// units climb and stamped does not, RXD group 2 is absent and this radio is not a
    /// common-view participant however loudly [`RadioTime::time_sources`] says it is.
    /// Program the EDCA parameters for all four access categories.
    ///
    /// ★ **This is the throughput knob on this part.** MEASURED at 80 MHz: ~185 µs of fixed
    /// per-PPDU cost, of which the firmware-default `cw_min` exponent 5 (CW = 31 slots, average
    /// backoff 15.5 x 9 µs = ~140 µs) is almost all. `txop = 0` likewise forces one PPDU per
    /// contention. Lowering the exponent and granting a TXOP are the two levers that attack the
    /// cost directly rather than amortising it.
    ///
    /// ⚠ Unlike the MT7612U's register-level equivalent — which took CW to 0 and cost a physical
    /// replug — this goes through the firmware, which owns the arbiter and validates the request.
    /// Use a *smaller* exponent (2-4), never 0.
    pub fn set_edca(&self, acs: &[mcu::EdcaAc; 4]) -> Result<(), FaceError> {
        mcu::set_edca(&self.mcu, &self.usb, acs, 0, true, 0)
    }

    /// Restore the firmware's default EDCA (cw_min 5, cw_max 10, txop 0, aifs 2).
    pub fn restore_edca(&self) -> Result<(), FaceError> {
        self.set_edca(&[mcu::EdcaAc::DEFAULT; 4])
    }

    /// The MAC's own account of how long it has spent **transmitting**, in microseconds
    /// (`MT_MIB_SDR36`, 24-bit, free-running).
    ///
    /// ★ **This is the instrument that separates "the radio accepted the frame" from "the frame
    /// went out".** A TX-pump throughput figure counts successful USB writes, and MEASURED
    /// 2026-08-28 those can be entirely fictitious: an HE MCS11 2SS/80 MHz flood reported
    /// **444 Mbit/s offered while a witness saw 173 frames and zero markers** — nothing
    /// radiated. A receiver is not a reliable check either, because a saturated witness pins at
    /// its own RX ceiling (~3900 f/s here) and reads like on-air loss. The transmitter's own
    /// airtime counter answers the question directly: multiply the offered frame rate by the
    /// PPDU airtime the chosen MCS implies, and compare.
    ///
    /// ⚠ Needs `MT_MIB_TXDUR_EN`, which [`enable_mib_airtime`](Self::enable_mib_airtime) sets;
    /// without it this reads 0. ⚠ 24 bits of microseconds **wraps every 16.7 s**, so sample
    /// windows must be shorter than that and differenced.
    pub fn tx_airtime_us(&self) -> Result<u32, FaceError> {
        Ok(self.rr(regs::mt_mib_sdr36(0))? & regs::MT_MIB_SDR36_TXTIME_MASK)
    }

    /// Arm the MIB TX/RX duration counters (`MT_MIB_TXDUR_EN` | `MT_MIB_RXDUR_EN` in
    /// `MT_MIB_SCR1`) — `mt792x_mac_init_band`, `mt792x_mac.c:298-300`.
    pub fn enable_mib_airtime(&self) -> Result<(), FaceError> {
        self.rmw(
            regs::mt_mib_scr1(0),
            0,
            regs::MT_MIB_TXDUR_EN | regs::MT_MIB_RXDUR_EN,
        )?;
        Ok(())
    }

    pub fn rx_health(&self) -> Result<String, FaceError> {
        let misc = self.usb.rr(regs::MT_CONN_ON_MISC)?;
        let wlcfg = self.usb.rr(regs::MT_UDMA_WLCFG_0)?;
        let glo = self.usb.rr(regs::MT_UWFDMA0_GLO_CFG)?;
        let mdp = self.usb.rr(regs::MT_MDP_DCR0)?;
        let dcr0 = self.usb.rr(regs::mt_dma_dcr0(0))?;
        let rfcr = self.usb.rr(regs::mt_wf_rfcr(0))?;
        let arb = self.usb.rr(regs::mt_arb_scr(0))?;
        let units = self.rx_units.load(Ordering::Relaxed);
        let stamped = self.rx_stamped.load(Ordering::Relaxed);
        Ok(format!(
            "CONN_ON_MISC={misc:#010x}(fw_rdy={}) WLCFG0={wlcfg:#010x}(rx_en={} tx_en={}) \
             UWFDMA0_GLO={glo:#010x}(rx_dma={}) MDP_DCR0={mdp:#010x}(hdr_trans={}) \
             DMA_DCR0={dcr0:#010x}(g5={} maxrx={}) RFCR={rfcr:#010x} \
             ARB_SCR={arb:#010x}(rx_dis={} tx_dis={}) ch={} rx units={units} stamped={stamped}",
            misc & regs::MT_TOP_MISC2_FW_N9_RDY == regs::MT_TOP_MISC2_FW_N9_RDY,
            wlcfg & regs::MT_WL_RX_EN != 0,
            wlcfg & regs::MT_WL_TX_EN != 0,
            glo & regs::MT_WFDMA0_GLO_CFG_RX_DMA_EN != 0,
            mdp & regs::MT_MDP_DCR0_RX_HDR_TRANS_EN != 0,
            dcr0 & regs::MT_DMA_DCR0_RXD_G5_EN != 0,
            regs::field_get(regs::MT_DMA_DCR0_MAX_RX_LEN, dcr0),
            arb & regs::MT_ARB_SCR_RX_DISABLE != 0,
            arb & regs::MT_ARB_SCR_TX_DISABLE != 0,
            self.channel.load(Ordering::Relaxed),
        ))
    }

    /// Take and clear the software RX counters — see [`RxStats`].
    pub fn rx_stats_reset(&self) -> RxStats {
        RxStats {
            units: self.rx_units.swap(0, Ordering::Relaxed),
            stamped: self.rx_stamped.swap(0, Ordering::Relaxed),
            fcs_errors: self.rx_fcs_errors.swap(0, Ordering::Relaxed),
            hdr_translated: self.rx_hdr_translated.swap(0, Ordering::Relaxed),
            amsdu_pad: self.rx_amsdu_pad.swap(0, Ordering::Relaxed),
            mcu_events: self.rx_mcu_events.swap(0, Ordering::Relaxed),
            undecodable: self.rx_undecodable.swap(0, Ordering::Relaxed),
            accepted: self.rx_accepted.swap(0, Ordering::Relaxed),
        }
    }

    /// Keep `depth` bulk-IN transfers in flight so a busy channel is not dropped between
    /// userspace reads; [`FrameIo::recv_frame`] then drains the shared queue.
    ///
    /// ⚠ **Do not issue MCU commands while this runs if
    /// [`mcu_response_ep`](Self::mcu_response_ep) is the data pipe.** Two readers on one
    /// bulk pipe split the traffic and each sees half — the same hazard the MT7610U's
    /// channel-time counters have, and just as silent. [`RxStats::mcu_events`] is the
    /// detector: a non-zero count means firmware events really are arriving on `0x84`.
    pub fn spawn_rx_pump(self: &Arc<Self>, depth: usize) -> Vec<std::thread::JoinHandle<()>> {
        crate::rx_pump::spawn_rx_pump(self, depth)
    }

    /// Read one raw bulk-IN transfer (an RX unit: descriptor + pad + MPDU). Returns 0 on
    /// timeout. For the first "is anything arriving at all" check.
    /// Start the pipelined TX path: `depth` writer threads draining a queue that
    /// [`FrameIo::inject`] feeds, instead of one awaited USB round trip per frame.
    ///
    /// ★ **MEASURED 2026-08-27, and this is the binding constraint on this radio's throughput.**
    /// A rate sweep at 1400 B through the synchronous path, on the USB 2.0 host:
    ///
    /// | rate | offered | throughput | airtime the PHY needs | we spend | non-PHY overhead |
    /// |---|---|---|---|---|---|
    /// | legacy OFDM 6M | 505 f/s | 5.66 Mbit/s | 1867 µs | 1979 µs | 112 µs |
    /// | HT MCS7 1SS | 2452 f/s | 27.5 Mbit/s | 172 µs | 408 µs | 236 µs |
    /// | VHT MCS7 2SS | 3281 f/s | 36.8 Mbit/s | 86 µs | 305 µs | 219 µs |
    /// | VHT MCS8 2SS | 3362 f/s | 37.7 Mbit/s | 72 µs | 297 µs | 226 µs |
    /// | HE MCS7 1SS | 2638 f/s | 29.6 Mbit/s | 165 µs | 379 µs | 214 µs |
    ///
    /// Read the last column: it converges to **~220 µs of per-frame cost that is not airtime**,
    /// and above about HT MCS7 that cost is most of the frame. Throughput saturates near
    /// **3300 f/s / 37 Mbit/s no matter what the PHY can do** — VHT MCS8 2SS has a 156 Mbit/s
    /// PHY and delivers 37.
    ///
    /// ★★ **RETRACTION, measured in the same session.** The paragraph that used to stand here
    /// said the ~220 µs was "one blocking `write_bulk` per frame" and that this pump was the
    /// fix, by analogy with the 8812au going 246 → 12913 f/s when its per-frame dispatch was
    /// replaced. **The A/B refutes it.** Same sweep, eight writer threads, backpressure only at
    /// the queue:
    ///
    /// | rate | synchronous | pumped |
    /// |---|---|---|
    /// | VHT MCS7 2SS | 3281 f/s / 36.75 Mbit/s | 3284 f/s / 36.78 Mbit/s |
    /// | HT MCS7 1SS | 2452 f/s / 27.5 | 2425 f/s / 27.2 |
    /// | HE MCS7 1SS | 2638 f/s / 29.6 | 2853 f/s / 32.0 |
    ///
    /// Pipelining buys **nothing** here. So the ~220 µs is not USB dispatch — it is the
    /// **medium**: at VHT MCS7 2SS with a 1400 B payload the airtime is 86 µs, and a VHT
    /// preamble (~40 µs) + DIFS (34 µs) + an average CW-15 backoff (~67 µs at 9 µs slots) is
    /// ~141 µs on top, which lands within a few µs of the 305 µs actually spent. Channel 6 was
    /// carrying ~172 frames/s of ambient traffic throughout, so the backoff term is real.
    ///
    /// ⇒ **The ceiling is per-frame 802.11 medium access, and the lever is aggregation, not
    /// concurrency.** One A-MPDU carrying many MPDUs pays that ~220 µs once instead of per
    /// frame. `MT_TXD7_HW_AMSDU` and `MT_DRV_AMSDU_OFFLOAD` exist on this part and are the
    /// place to look; a quieter channel would also move it, which is a separate experiment
    /// (the 8812au work showed contention dominating in exactly this way).
    ///
    /// ★★★ **AND THE RETRACTION WAS ITSELF TOO BROAD.** The sentence that stood here — "the pump
    /// is not the throughput fix" — was true of the 20 MHz / 1400 B corner it was measured in and
    /// false in general. Re-measured at ch36, 80 MHz, VHT MCS9 2SS short-GI:
    ///
    /// | payload | no pump | pump (8 threads) |
    /// |---|---|---|
    /// | 7000 B | 2009 f/s / 112.5 Mbit/s | 3082 f/s / **172.6 Mbit/s** |
    /// | 7935 B | 1888 f/s / 119.9 Mbit/s | 2742 f/s / **174.0 Mbit/s** |
    ///
    /// **+54%.** Both measurements are correct; the mistake was generalising from one corner.
    /// The two costs trade places: at 20 MHz with a small frame the medium dominates and
    /// pipelining has nothing to hide, while at 80 MHz with an 8 KB frame the airtime is ~65 µs
    /// and pushing 8 KB across USB 2.0 is the serial step — which is exactly what several URBs
    /// in flight overlap. Neither the pump nor aggregation is "the" fix; **width, MPDU size and
    /// pipelining are three multiplicative levers**, and 37 → 174 Mbit/s is what pulling all
    /// three is worth on this part.
    ///
    /// Recorded at this length because the general lesson cost two wrong claims in one session:
    /// a throughput number is meaningless without the corner it was taken in.
    ///
    /// ⚠ Fire-and-forget by construction: a write that fails is counted and dropped, not
    /// retried and not reported to the caller. That is the right trade for a broadcast,
    /// un-ACKed medium — there is no receiver to disappoint and a retry would only reorder —
    /// but it means [`FrameIo::inject`] returning `Ok` after this is a *queue* acceptance, not
    /// a transmission. Read [`tx_written`](Mt7921uBackend::tx_written) for what actually left.
    ///
    /// ⚠ The queue is **bounded** and `inject` blocks when it is full — see the comment on the
    /// channel below for the measurement that made that mandatory.
    pub fn spawn_tx_pump(self: &Arc<Self>, depth: usize) -> Vec<std::thread::JoinHandle<()>> {
        use std::sync::atomic::Ordering;
        // ★ BOUNDED, and that is not a detail. An unbounded queue turns `inject` into a
        // non-blocking enqueue with no backpressure whatsoever: MEASURED, a 3-second flood
        // enqueued hundreds of thousands of frames that then took minutes to drain, so the
        // "throughput" being measured was the speed of a channel send and the radio was still
        // transmitting long after the test believed it had finished. A bounded queue makes
        // `inject` block once the pipeline is full, which is exactly the backpressure the
        // synchronous path had for free — the point of the pump is to keep several URBs in
        // flight, not to accept work the radio cannot do.
        //
        // Depth x 4: enough to keep every writer thread fed across a scheduling hiccup, small
        // enough that a caller's next `inject` is delayed by the radio rather than by a queue.
        let (tx, rx) = std::sync::mpsc::sync_channel::<Vec<u8>>(depth.max(1) * 4);
        *self.tx_sender.lock().unwrap_or_else(|e| e.into_inner()) = Some(tx);
        let rx = Arc::new(Mutex::new(rx));
        (0..depth.max(1))
            .map(|_| {
                let me = self.clone();
                let rx = rx.clone();
                std::thread::spawn(move || {
                    let handle = me.usb.handle();
                    let ep = me.usb.ep_out_data();
                    loop {
                        // `recv` blocks, so an idle pump costs nothing — no polling sleep, which
                        // is what bounds the sibling implementation's latency at low rates.
                        let buf = match rx.lock().unwrap_or_else(|e| e.into_inner()).recv() {
                            Ok(b) => b,
                            Err(_) => break, // sender dropped
                        };
                        if let Ok(n) = handle.write_bulk(ep, &buf, BULK_TX_TIMEOUT) {
                            me.tx_bytes.fetch_add(n as u64, Ordering::Relaxed);
                            me.tx_count.fetch_add(1, Ordering::Relaxed);
                        }
                    }
                })
            })
            .collect()
    }

    /// `(frames, bytes)` the TX pump has written to USB. The honest denominator for any
    /// offered-load figure once [`spawn_tx_pump`](Mt7921uBackend::spawn_tx_pump) is running,
    /// because `inject` then returns as soon as the queue accepts.
    pub fn tx_written(&self) -> (u64, u64) {
        use std::sync::atomic::Ordering;
        (
            self.tx_count.load(Ordering::Relaxed),
            self.tx_bytes.load(Ordering::Relaxed),
        )
    }

    pub fn read_rx(&self, buf: &mut [u8]) -> Result<usize, FaceError> {
        self.usb
            .bulk_in_timeout(self.usb.ep_in_data(), buf, BULK_RX_TIMEOUT)
    }

    // ── TX ──────────────────────────────────────────────────────────────────

    /// The fixed rate to transmit `frame` at, or `None` to let the firmware's rate
    /// controller choose.
    ///
    /// * A [`Reliability::MostRobust`](ndn_radio_hal::Reliability::MostRobust) intent goes
    ///   out **legacy OFDM 6 Mbps**, whatever the control plane last set. That intent means
    ///   "the worst receiver in earshot must decode this", and an HT/VHT/HE PPDU excludes
    ///   every receiver without that decoder by construction. Same rule as every other
    ///   backend here.
    ///
    ///   ★ Note this deliberately does **not** use
    ///   [`McsDescriptor::for_intent`]'s HE branch, which turns `MostRobust` on an
    ///   `he_cap` radio into `he(0).with_er_su().with_dcm()`. ER-SU is the strongest
    ///   single-frame reach mode an HE PHY offers — ~2-4 dB of receiver sensitivity — but
    ///   **only an HE receiver can decode it**, exactly as only 11ac decodes VHT. On a
    ///   broadcast bearer with no ACKs there is no way to discover that the neighbour could
    ///   not hear it. So ER-SU is a lever cognition *opts into* for a reach it already knows
    ///   is HE-capable, and it arrives here through [`FrameIo::set_rate`] — never as the
    ///   default for a frame whose whole point is being universally decodable.
    /// * Otherwise the rate stored by [`FrameIo::set_rate`] /
    ///   [`set_legacy_rate`](Self::set_legacy_rate), if any.
    /// * Otherwise legacy OFDM 6 Mbps — the workspace default for an unmeasured link.
    ///
    /// `NDN_RADIO_TX_RATE` overrides everything with a **raw 14-bit connac2 rate word**
    /// (decimal, or `0x`-prefixed hex), the [`mac::MT_TXD6_TX_RATE`] encoding:
    /// `0x02cb` legacy OFDM-6M, `0x0080` HT-MCS0, `0x0087` HT-MCS7, `0x0200` HE-SU-MCS0.
    /// The variable is per-driver by design — a Realtek DESC code, an mt76x02 rate word and
    /// a connac2 rate word are three different encodings and must not be confused.
    fn resolved_rate(&self, frame: &InjectFrame) -> Option<mac::FixedRate> {
        if let Some(v) = std::env::var("NDN_RADIO_TX_RATE").ok().and_then(|s| {
            let s = s.trim().to_ascii_lowercase();
            match s.strip_prefix("0x") {
                Some(hex) => u16::from_str_radix(hex, 16).ok(),
                None => s.parse::<u16>().ok(),
            }
        }) {
            return Some(mac::FixedRate::at(v));
        }
        if frame.tx.reliability == ndn_radio_hal::Reliability::MostRobust {
            return Some(mac::FixedRate::at(mac::LegacyRate::Ofdm6.rate_val()));
        }
        Some(
            self.cur_rate
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .unwrap_or_else(|| mac::FixedRate::at(mac::LegacyRate::Ofdm6.rate_val())),
        )
    }

    /// Transmit at a fixed legacy rate from here on — the worst-receiver lever as bearer
    /// state, the legacy sibling of [`FrameIo::set_rate`].
    pub fn set_legacy_rate(&self, rate: mac::LegacyRate) {
        *self.cur_rate.lock().unwrap_or_else(|e| e.into_inner()) =
            Some(mac::FixedRate::at(rate.rate_val()));
    }

    /// The [`mac::TxdConfig`] for one broadcast frame at `rate`.
    ///
    /// ★ **`no_ack: Some(true)`, not the derived default.** [`mac::build_txd`] otherwise
    /// derives it from address 1's group bit, which is right for mac80211 and wrong here:
    /// under the named-radio addressing doctrine `addr1` is a **name-derived prefix-set
    /// filter half**, not a MAC address, so its low bit is whatever the name hashed to. A
    /// filter whose first octet happens to be even would look like a unicast address, and
    /// the hardware would wait for an ACK that nobody on a connectionless broadcast bearer
    /// will ever send — burning a retry ladder and delaying every frame behind it. The
    /// bearer has no ACKs by construction, so this is stated rather than derived.
    ///
    /// `wcid: 0` is upstream's global WCID: *"Beacon and mgmt frames should occupy wcid 0"*
    /// (`mt792x_core.c:849-855`), which is the one entry a driver with no station table can
    /// legitimately transmit from.
    ///
    /// `seq: None` — the hardware assigns the sequence number
    /// ([`MT_TXD3_SN_VALID`](mac::MT_TXD3_SN_VALID) stays clear), which is upstream's data
    /// path. `crate::frame::build_dot11` fills a SeqCtrl of its own and the hardware
    /// overwrites it; nothing on this bearer reads it.
    fn txd_config(&self, rate: Option<mac::FixedRate>) -> mac::TxdConfig {
        mac::TxdConfig {
            no_ack: Some(true),
            fixed_rate: rate,
            ..mac::TxdConfig::default()
        }
    }

    /// Build the complete USB bulk-OUT for one bare 802.11 frame:
    /// `[4 B USB header][64 B TXD][802.11][pad to 4][4 B zero tail]` — [`mac::build_usb_tx`].
    pub fn build_tx_bulk(&self, dot11: &[u8], rate: Option<mac::FixedRate>) -> Vec<u8> {
        mac::build_usb_tx(&self.txd_config(rate), dot11)
    }

    /// Write a pre-built TX bulk to the **AC_BE** bulk-OUT pipe, blocking.
    ///
    /// AC_BE (`0x05`), not the inband-command pipe: `0x04` carries MCU messages, and a data
    /// frame pushed into it is not transmitted. (AC_BE also carries firmware chunks during
    /// download — but only while `MT_UDMA_TX_QSEL`'s `MT_FW_DL_EN` is set, which
    /// [`mcu::run_firmware`](crate::connac2::mcu::run_firmware) clears on every exit path
    /// precisely so this call works afterwards.)
    pub fn tx_raw(&self, bulk: &[u8]) -> Result<(), FaceError> {
        self.usb
            .bulk_out_timeout(self.usb.ep_out_data(), bulk, BULK_TX_TIMEOUT)
    }

    /// Transmit one bare 802.11 frame at `rate`. Synchronous; the async path is
    /// [`FrameIo::inject`].
    pub fn transmit(&self, dot11: &[u8], rate: Option<mac::FixedRate>) -> Result<(), FaceError> {
        self.tx_raw(&self.build_tx_bulk(dot11, rate))
    }

    /// The next 12-bit 802.11 sequence number. Reserved for a future path that sets
    /// [`MT_TXD3_SN_VALID`](mac::MT_TXD3_SN_VALID); the base frame builder fills SeqCtrl
    /// itself and the hardware overwrites it.
    fn next_seq(&self) -> u16 {
        self.seq.fetch_add(1, Ordering::Relaxed) & 0x0fff
    }

    // ── Clocks ──────────────────────────────────────────────────────────────

    /// Read the 64-bit LPON TSF — `mt792x_get_tsf` (`mt792x_core.c:243-265`).
    ///
    /// Three EP0 round trips ≈ **804 µs**: set `MT_LPON_TCR`'s `SW_MODE` (`GENMASK(1,0)`,
    /// which `mt7915/regs.h:299` names `SW_READ` — a software-read latch), then read
    /// `MT_LPON_UTTR0` (low) and `MT_LPON_UTTR1` (high).
    ///
    /// ★ This is the **same counter** the per-frame RXD group-2 stamp comes from (see
    /// [`mac::Rxd::timestamp`]), which is what makes it useful rather than redundant: the
    /// descriptor gives 32 bits at 1 µs — wrapping every 71 min 35 s — and this read
    /// supplies the high word and the epoch to unwrap them against. Note the direction of
    /// the dependency: the register read is the **coarse anchor**, the descriptor is the
    /// measurement.
    ///
    /// The `n` index is the OMAC slot; a monitor has none, so `HW_BSSID_0` = 0
    /// (`mt792x_core.c:256`).
    pub fn read_tsf(&self) -> Result<u64, FaceError> {
        self.usb
            .set_bits(regs::mt_lpon_tcr(0, 0), regs::MT_LPON_TCR_SW_MODE)?;
        let lo = self.usb.rr(regs::mt_lpon_uttr0(0))?;
        let hi = self.usb.rr(regs::mt_lpon_uttr1(0))?;
        Ok((u64::from(hi) << 32) | u64::from(lo))
    }
}

// ── RX pump ─────────────────────────────────────────────────────────────────

/// The connac2 side of the shared RX pipeline.
///
/// **One RX unit per bulk-IN transfer, and here is the citation.** Two independent facts:
///   * `mt792xu_dma_init` sets `MT_WL_RX_EN | MT_WL_TX_EN | MT_WL_RX_MPSZ_PAD0 |
///     MT_TICK_1US_EN` in `MT_UDMA_WLCFG_0` (`mt792x_usb.c:398-403`) and **never sets
///     `MT_WL_RX_AGG_EN`** (bit 21, `mt792x_regs.h:478`) — the only tree-wide setter of that
///     bit is `mt7615/usb_sdio.c:269`, a different family. It then *clears* every
///     aggregation knob it can reach: `MT_WL_RX_AGG_TO`, `MT_WL_RX_AGG_LMT`
///     (`:407-408`) and `MT_WL_RX_AGG_PKT_LMT` (`:409-410`). **That is the register that
///     decides it, and on this part it is off.**
///   * `mt76u_process_rx_entry` (`usb.c:522-560`) builds exactly **one** skb per URB: with
///     `MT_DRV_RX_DMA_HDR` set (`mt7921/usb.c:153`) it takes `mt76u_get_rx_entry_len`'s
///     value as the whole entry and never loops. A kernel that expected aggregation here
///     would drop everything after the first unit.
///
/// The loop below still walks [`mac::Rxd::unit_len`] rather than treating the transfer as
/// one unit, so a device that ever *did* aggregate would be de-aggregated instead of
/// silently truncated — but it does not loop twice today, and that is a fact with a register
/// behind it rather than the open question the MT7612U backend still carries.
impl crate::rx_pump::Pumpable for Mt7921uBackend {
    fn pump_handle(&self) -> Arc<DeviceHandle<Context>> {
        self.usb.handle()
    }

    fn pump_bulk_in(&self) -> u8 {
        self.usb.ep_in_data()
    }

    fn pump_state(&self) -> &crate::rx_pump::RxPumpState {
        &self.rx
    }

    /// Split one bulk-IN transfer into the frames it carried.
    ///
    /// Byte 0 of the transfer **is** `rxd[0]` — unlike the mt76x0/mt76x2 there is no 4-byte
    /// DMA header in front of the descriptor, because `mt7921u` declares
    /// `MT_DRV_RX_DMA_HDR` (`mt7921/usb.c:153`).
    fn parse_transfer(&self, buf: &[u8]) -> Vec<CapturedFrame> {
        let mut out = Vec::new();
        let mut off = 0usize;

        while off + mac::RXD_FIXED_DWORDS * 4 <= buf.len() {
            let unit = &buf[off..];
            let d = match mac::parse_rxd(unit) {
                Ok(d) => d,
                Err(e) => {
                    // At the very start this is a short or malformed transfer worth counting;
                    // past the first unit it is just the tail of a padded transfer.
                    if off == 0 {
                        self.rx_undecodable.fetch_add(1, Ordering::Relaxed);
                        tracing::debug!(
                            target: "named_radio",
                            chip = "MT7921AU",
                            len = buf.len(),
                            error = ?e,
                            "mt7921u: undecodable RX descriptor",
                        );
                    }
                    break;
                }
            };

            // ★ Every RX unit pulled off USB, counted BEFORE the pkt-type, FCS and format
            // filters, so `rx_raw_frames()` is directly comparable to a kernel monitor's
            // `rx_packets`. A backend that joins the pump and forgets this reports a
            // structural 0, which reads exactly like a dead receiver — see the warning on
            // `crate::RX_RAW_FRAMES`, which records that trap costing a real debugging
            // session on 2026-08-24.
            crate::RX_RAW_FRAMES.fetch_add(1, Ordering::Relaxed);
            self.rx_units.fetch_add(1, Ordering::Relaxed);

            // Advance before any `continue`. A hypothetical second unit would start
            // 4-aligned (`MT_RXD0_LENGTH` counts descriptor + MPDU and an MPDU may be odd),
            // so the round-up is the only defensible guess — and it is unreachable today,
            // see this impl's doc.
            off += d.unit_len.next_multiple_of(4).max(4);

            // ★ Not every transfer on this pipe is a frame. With
            // `MT_WFDMA_HOST_CONFIG_USB_RXEVT_EP4_EN` set, firmware events and TXS reports
            // arrive here too and are demuxed by descriptor type
            // (`mt7921_queue_rx_skb`, `mt7921/mac.c:596-613`). Counted separately, because a
            // non-zero count is what proves the MCU shares this pipe.
            if d.pkt_type != mac::PKT_TYPE_NORMAL && d.pkt_type != mac::PKT_TYPE_NORMAL_MCU {
                self.rx_mcu_events.fetch_add(1, Ordering::Relaxed);
                continue;
            }

            // ★ The headline feature's health counter — see `RxStats::stamped`.
            if d.timestamp.is_some() {
                self.rx_stamped.fetch_add(1, Ordering::Relaxed);
            }
            if d.body_pad != 0 {
                self.rx_amsdu_pad.fetch_add(1, Ordering::Relaxed);
            }
            if d.hdr_trans {
                // The hardware rewrote the 802.11 header into an 802.3 one: there is no
                // 802.11 frame here to parse. Must be 0 — `mac_init` clears
                // MT_MDP_DCR0_RX_HDR_TRANS_EN precisely so it is.
                self.rx_hdr_translated.fetch_add(1, Ordering::Relaxed);
                continue;
            }
            if d.fcs_err {
                // The hardware demodulated a PPDU and its FCS failed — a collision or a
                // marginal link. Counted (it is signal about the channel), never delivered.
                self.rx_fcs_errors.fetch_add(1, Ordering::Relaxed);
                continue;
            }

            let Some(mpdu) = mac::mpdu(&unit[..d.unit_len.min(unit.len())], &d) else {
                self.rx_undecodable.fetch_add(1, Ordering::Relaxed);
                continue;
            };

            // ★ The per-frame hardware stamp: RXD group 2 dword 0, 32 bits at (CODE-READ)
            // 1 µs, latched at PPDU start (`RX_FLAG_MACTIME_START`, `mt7921/mac.c:309`).
            // Carried raw and 32-bit — the wrap is the consumer's problem and
            // `read_clock` on the same domain is how it is resolved; see
            // `RadioTime::time_sources`. `None` when the descriptor did not carry group 2,
            // never a fabricated 0: a timestamp of 0 and "no timestamp" are the difference
            // between a working common-view clock and a silently broken one.
            let stamp = d.timestamp.map(|ts| {
                LinkStamp::new(
                    u64::from(ts),
                    self.tsf_domain,
                    LatchPoint::MacDone.precision_floor_ns(),
                    LatchPoint::MacDone,
                )
            });
            // Strongest per-chain RCPI over the two chains that physically exist, converted
            // by `rcpi/2 - 110` (`mt7921/mac.c:371-379`). `None` when there was no P-RXV or
            // every chain reported an implausible (>= 0 dBm) level.
            let rssi = d.signal_dbm();
            let mcs = d.rate.and_then(|r| r.mcs);

            // `phy` stays None — and that is a finding, not a gap. See the module header:
            // `mac::crxv_snr_db` returns None for every normal RXD because the embedded
            // C-RXV is 18 dwords and MT_CRXV_SNR lives in dword 20. Manufacturing a metric
            // out of an in-range dword is exactly the defect this codebase removes.
            if let Some(f) = crate::frame::parse_dot11(self.format, &mpdu, rssi, mcs, stamp) {
                self.rx_accepted.fetch_add(1, Ordering::Relaxed);
                self.rx_ok_window.fetch_add(1, Ordering::Relaxed);
                out.push(f);
            }
        }
        out
    }
}

// ── FrameIo ─────────────────────────────────────────────────────────────────

#[async_trait]
impl FrameIo for Mt7921uBackend {
    async fn inject(&self, frame: InjectFrame) -> Result<(), FaceError> {
        // ★ Same hazard as the sibling MT7612U, whose limit is lower: an MPDU past what the part
        // accepts does not get dropped, it **resets the radio**. MEASURED here: 7935 B sustains
        // 2742 f/s / 174 Mbit/s, while 11000 B collapses to 3 f/s. The 802.11 VHT A-MPDU limit
        // is 11454 B, so cap at that and let the measurement stand where it is until a larger
        // size is shown to be safe on silicon.
        if frame.payload.len() > MAX_MPDU_PAYLOAD {
            return Err(io_err(format!(
                "mt7921u: payload {} B exceeds MAX_MPDU_PAYLOAD {MAX_MPDU_PAYLOAD} — an \
                 oversized MPDU has been MEASURED to reset this family's radios rather than be \
                 dropped. Fragment above this seam.",
                frame.payload.len()
            )));
        }
        let dot11 = crate::frame::build_dot11(self.format, &frame)?;
        let rate = self.resolved_rate(&frame);
        let buf = self.build_tx_bulk(&dot11, rate);
        // Pumped path: hand the built bulk to a writer thread and return. Costs one channel
        // send instead of a USB round trip — see `spawn_tx_pump` for the measurement that
        // makes this the difference between 37 Mbit/s and the PHY's actual capability.
        if let Some(tx) = self
            .tx_sender
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .as_ref()
        {
            return tx
                .send(buf)
                .map_err(|_| io_err("mt7921u TX: pump threads have exited".into()));
        }
        let handle = self.usb.handle();
        let ep = self.usb.ep_out_data();
        tokio::task::spawn_blocking(move || {
            handle
                .write_bulk(ep, &buf, BULK_TX_TIMEOUT)
                .map_err(usb_err)
                .and_then(|n| {
                    (n == buf.len())
                        .then_some(())
                        .ok_or_else(|| io_err(format!("mt7921u TX: short write {n}/{}", buf.len())))
                })
        })
        .await
        .map_err(|e| io_err(format!("mt7921u TX: join {e}")))?
    }

    /// Rate as bearer state: the exact TXD DW6 every subsequent [`inject`](FrameIo::inject)
    /// transmits at.
    ///
    /// ★ **This is the crate's only path to an 802.11ax transmit.** [`mac::encode_rate`]
    /// honours `he`, `dcm` and `er_su` here and nowhere else — an `McsDescriptor` with
    /// `he: true` reaches a real `MT_TX_RATE_MODE` of `HE_SU` (8) or `HE_EXT_SU` (9), and
    /// `dcm` / `er_su` reach `MT_TX_RATE_DCM` / `MT_TX_RATE_SU_EXT_TONE`. On every other
    /// Wi-Fi backend in this crate those three fields are silently ignored, which is why
    /// [`declared_capability`] setting `he_cap` is load-bearing rather than cosmetic:
    /// cognition only builds an HE descriptor for a radio that claims one.
    ///
    /// ⚠ Bandwidth stays 20 MHz ([`mac::FixedRate::at`]) because that is the only width
    /// [`RadioKnobs::set_channel`] tunes. A rate word wider than the tuned baseband is a
    /// malformed PPDU, so the two must not be allowed to disagree.
    ///
    /// A `MostRobust` frame still overrides this with legacy OFDM 6 Mbps — see
    /// `resolved_rate` for why HE ER-SU is not the
    /// broadcast default even though this part can transmit it.
    fn set_rate(&self, mcs: McsDescriptor) -> Result<(), FaceError> {
        // ★ The rate word's BW field must equal the tuned width. `FixedRate::at` pins 20 MHz,
        // which was correct while `set_channel` only tuned 20 — it is now a bug waiting to
        // happen, because a 20 MHz rate word on an 80 MHz baseband wastes three quarters of the
        // channel silently (and the reverse is a malformed PPDU). Take it from the radio's own
        // state so the two cannot disagree.
        let mut r = mac::FixedRate::at(mac::encode_rate(&mcs));
        // ★ **Clamp the width to what the PHY MODE allows, not just to what is tuned.**
        //
        // Taking the width straight from the tuned channel was wrong and MEASURED harmful: an
        // **HT rate word carrying BW = 80 MHz is malformed** — 802.11n stops at 40 — and this
        // MAC does not reject it, it stops transmitting. The rate sweep that found it went
        // legacy 6M (514 f/s, fine, because `MostRobust` builds its own descriptor at BW 20) ->
        // HT MCS3 @ 80 (60 f/s) -> HT MCS7 @ 80 (**0 f/s**), and never recovered.
        //
        // Legacy CCK/OFDM has no bandwidth concept and must be 20; HT tops out at 40; only
        // VHT and HE reach 80. Narrowing is always safe, so clamp rather than error: a rate the
        // caller can have at a lesser width beats a silent transmitter.
        let tuned = self.bw.load(Ordering::Relaxed).min(3);
        let mode_max = if mcs.he || mcs.vht {
            3 // VHT/HE: up to 160 as far as the rate word is concerned
        } else if mcs.index > 0 || mcs.nss > 1 {
            1 // HT: 20 or 40 only
        } else {
            1
        };
        r.bw = tuned.min(mode_max);
        r.sgi = u8::from(mcs.short_gi);
        r.ldpc = mcs.ldpc;
        *self.cur_rate.lock().unwrap_or_else(|e| e.into_inner()) = Some(r);
        Ok(())
    }

    /// One plain MPDU per NDN packet.
    ///
    /// No host-built A-MSDU bundling: it is firmware-gated on monitor injection on the
    /// sibling MT7612U (verified 0/200 on air there) and nothing has tested it here. Rather
    /// than inherit a neighbouring chip's unverified claim in either direction, this sends
    /// what is known to work. (`MT_TXD7_HW_AMSDU` exists and `MT_DRV_AMSDU_OFFLOAD` is in
    /// this part's driver flags, so the question is worth a measurement.)
    async fn inject_batch(&self, frames: Vec<InjectFrame>) -> Result<(), FaceError> {
        for f in frames {
            self.inject(f).await?;
        }
        Ok(())
    }

    async fn recv_frame(&self) -> Result<CapturedFrame, FaceError> {
        // Pumped mode: background threads fill the shared queue; just drain it.
        if self.rx.is_pumped() {
            return Ok(self.rx.recv().await);
        }
        loop {
            if let Some(f) = self.rx.try_pop() {
                return Ok(f);
            }
            let handle = self.usb.handle();
            let ep = self.usb.ep_in_data();
            let got = tokio::task::spawn_blocking(move || {
                let mut b = vec![0u8; RX_BUF_LEN];
                match handle.read_bulk(ep, &mut b, BULK_RX_TIMEOUT) {
                    Ok(n) if n > 0 => {
                        b.truncate(n);
                        Ok(Some(b))
                    }
                    Ok(_) | Err(rusb::Error::Timeout) => Ok(None),
                    Err(e) => Err(usb_err(e)),
                }
            })
            .await
            .map_err(|e| io_err(format!("mt7921u recv_frame: join {e}")))??;
            if let Some(b) = got {
                self.rx
                    .push(crate::rx_pump::Pumpable::parse_transfer(self, &b));
            }
        }
    }
}

// ── RadioKnobs ──────────────────────────────────────────────────────────────

impl RadioKnobs for Mt7921uBackend {
    /// Tune to `channel` at 20 MHz, via the firmware's channel switch.
    ///
    /// Three things happen, and all three are needed:
    /// 1. `MCU_EXT_CMD(CHANNEL_SWITCH)` — the PHY retune (`mt7921_set_channel`,
    ///    `mt7921/main.c:477-499`).
    /// 2. `set_mac_timing` — SIFS is **band-dependent**
    ///    (10 µs on 2.4 GHz, 16 µs on 5 GHz), so a cross-band hop that skipped this would
    ///    leave the transmitter spacing its frames by the other band's rules.
    /// 3. The sniffer's own copy of the channel, if monitor is up
    ///    ([`mcu::config_sniffer`](crate::connac2::mcu::config_sniffer)) — the PHY retune is
    ///    **not** sufficient for monitor mode. Two commands, two different width/band
    ///    encodings; see [`mcu::SnifferChan`].
    ///
    /// Tunes 20/40/80 MHz, deriving the 802.11 centre channel from the control channel (see
    /// [`centre_and_cbw`]), and [`declared_capability`] reports `max_bw: 2` to
    /// match. 40/80/160 MHz exist in [`mcu::ChannelReq`] and in the firmware, but selecting
    /// one needs the *centre* channel, which cannot be derived from a control channel alone
    /// (channel 36 at 80 MHz centres on 42; at 40 MHz it centres on 38 or 34 depending on
    /// the secondary's side). Guessing would put the secondary channel in the wrong place —
    /// silently, on air.
    ///
    /// **Rejects a channel not in `CHANNELS_2GHZ`/`CHANNELS_5GHZ`** rather than tuning
    /// something adjacent: a synthesiser pointed somewhere unintended looks exactly like a
    /// quiet channel. 6 GHz is refused for the same reason — [`mcu::CHANNEL_BAND_6G`] exists
    /// and the firmware's own numbering for it is **2, not nl80211's 3**
    /// (`mt7921/mcu.c:914-917`), but no 6 GHz-capable antenna or regulatory table has been
    /// tried here and this part's CLC data decides what it will accept.
    ///
    /// ⚠ If [`mcu_response_ep`](Mt7921uBackend::mcu_response_ep) is the data pipe, do not
    /// call this while the RX pump runs — the pump can eat the switch's response and this
    /// returns a timeout for a command the firmware executed.
    fn set_channel(&self, channel: u8, bw: Bandwidth) -> Result<(), FaceError> {
        let is_2ghz = CHANNELS_2GHZ.contains(&channel);
        if !is_2ghz && !CHANNELS_5GHZ.contains(&channel) {
            return Err(io_err(format!(
                "mt7921u: channel {channel} is not one this port tunes (2.4 GHz 1-14, \
                 5 GHz 36-165; 6 GHz is deliberately refused)"
            )));
        }
        let (center_ch, cbw) = centre_and_cbw(channel, bw).ok_or_else(|| {
            io_err(format!(
                "mt7921u: ch{channel} has no {bw:?} block in the 802.11 channelisation \
                 (2.4 GHz has no 80 MHz; 5 GHz 40/80 blocks are fixed and this channel is not \
                 a member of one)"
            ))
        })?;

        let req = mcu::ChannelReq {
            control_ch: channel,
            center_ch,
            bw: cbw,
            // 2x2: both chains transmit and both receive. `rx_streams_mask` is a MASK here
            // and `ChannelReq::encode` turns it into a count for CHANNEL_SWITCH — see that
            // method, it is one field with two meanings across two commands.
            tx_streams: 2,
            rx_streams_mask: 0x3,
            switch_reason: mcu::CH_SWITCH_NORMAL,
            band_idx: 0,
            center_ch2: 0,
            channel_band: if is_2ghz {
                mcu::CHANNEL_BAND_2G
            } else {
                mcu::CHANNEL_BAND_5G
            },
        };
        mcu::set_channel(&self.mcu, &self.usb, &req)?;
        self.set_mac_timing(is_2ghz)?;

        self.channel.store(channel, Ordering::Relaxed);
        self.bw.store(bw.code(), Ordering::Relaxed);
        if self.monitor.load(Ordering::Relaxed) {
            let strict = std::env::var_os("NDN_RADIO_RX_STRICT").is_some();
            self.config_sniffer(channel, bw, strict)?;
        }

        // The MIB airtime counters accumulated on the *previous* channel and are
        // read-and-clear (`mt792x_mac_reset_counters`, `mt792x_mac.c:205-211`), so one
        // discarded read keeps the first occupancy window after a hop from being charged to
        // the wrong channel.
        let _ = self.usb.rr(regs::mt_mib_sdr9(0));
        let _ = self.usb.rr(regs::mt_mib_sdr36(0));
        let _ = self.usb.rr(regs::mt_mib_sdr37(0));
        let _ = self.usb.set_bits(
            regs::mt_wf_rmac_mib_time0(0),
            regs::MT_WF_RMAC_MIB_RXTIME_CLR,
        );
        *self.last_activity.lock().unwrap_or_else(|e| e.into_inner()) = Instant::now();
        Ok(())
    }

    /// Contention window as a posture — the actuator for the slot decision, and on this part the
    /// **largest single throughput lever there is**.
    ///
    /// ★ MEASURED 2026-08-28 (ch36, VHT MCS9 2SS / 80 MHz / SGI, 9000 B, SuperSpeed): the
    /// firmware substitutes `cw_min` **exponent 5** when nothing else is programmed — CW = 31
    /// slots, an average backoff of 15.5 x 9 us = **~140 us**, which was most of the per-PPDU
    /// fixed cost. Moving to exponent 2 took offered throughput from **382 to 416 Mbit/s**, and a
    /// witness confirmed the frames radiate. That is the difference between contending for a
    /// medium and using one you have already been granted.
    ///
    /// Goes through `MCU_CE_CMD(SET_EDCA_PARMS)`, so the firmware owns the arbiter and validates
    /// the request — unlike the mt76x02 register path, where the equivalent write cost a replug.
    fn set_contention(
        &self,
        posture: ndn_radio_hal::ContentionPosture,
    ) -> Result<ndn_radio_hal::ContentionApplied, FaceError> {
        use ndn_radio_hal::{ContentionApplied, ContentionPosture};
        // (cw_min, cw_max, aifs, txop) exponents. `Shared` reproduces the firmware's own
        // defaults (`mt7921/mcu.c:747-752`) so "back to normal" is a real restore.
        let (cw_min, cw_max, aifs, txop) = match posture {
            ContentionPosture::Owned => (2u16, 5u16, 1u16, 0u16),
            ContentionPosture::Shared => (5, 10, 2, 0),
            ContentionPosture::Yielding => (7, 10, 3, 0),
        };
        let ac = mcu::EdcaAc {
            cw_min,
            cw_max,
            txop,
            aifs,
            guardtime: 0,
            acm: 0,
        };
        self.set_edca(&[ac; 4])?;
        // Read the slot back rather than assume the 802.11a 9 us: on this family it lives in
        // TMAC ICR0, and it is what every term of the DCF budget is counted in. (The sibling
        // MT7612U boots at 20 us, which is precisely why this is measured and not assumed.)
        let slot_us = self
            .usb
            .rr(regs::mt_tmac_icr0(0))
            .map(|v| regs::field_get(regs::MT_IFS_SLOT, v) as u8)
            .ok()
            .filter(|s| (9..=20).contains(s))
            .unwrap_or(9);
        Ok(ContentionApplied {
            cw_min: cw_min as u8,
            cw_max: cw_max as u8,
            aifs: aifs as u8,
            txop,
            slot_us,
            avg_backoff_us: ContentionApplied::avg_backoff_us_at(cw_min as u8, slot_us),
        })
    }

    /// **Frame-free channel occupancy**, as busy per mille of the window since the last call.
    ///
    /// `MT_MIB_SDR9`'s `BUSY_MASK` (`GENMASK(23,0)`, `mt792x_regs.h:109-110`) is the MAC's
    /// channel-busy time in microseconds — no frame is decoded, so an interferer this PHY
    /// cannot demodulate still shows up. That is what makes it a real duty cycle rather than
    /// a frame-rate proxy. `mt792x_mac_reset_counters` (`mt792x_mac.c:205-208`) resets these
    /// by *reading* them, so the value is the level for the elapsed window.
    ///
    /// ⚠ **Two departures from the MT7610U's version of this method, both forced:**
    ///   * **There is no hardware idle counter here.** The mt76x0 has `MT_CH_IDLE` beside
    ///     `MT_CH_BUSY` and MEASURED `(idle + busy) / elapsed = 1.00`, so its denominator is
    ///     the hardware's. This part reports busy, TX time (`SDR36`) and RX time (`SDR37`)
    ///     and nothing that counts idle, so the denominator here is the **host clock**
    ///     between calls. That is honest but it is a different measurement: it charges USB
    ///     scheduling delay and any host stall to "idle".
    ///   * **The counter is 24 bits at 1 µs = 16.777 s.** Call this at least every ~16
    ///     seconds or the window saturates silently. The result is clamped to 1000 rather
    ///     than allowed to exceed it, so a saturated window reads "fully busy" instead of
    ///     nonsense.
    ///
    /// ⚠ Do not difference two of these. The hardware clears on read, so the value returned
    /// is already the level for the elapsed window; the per-mille normalisation is what makes
    /// that safe, since a differenced per mille is visibly nonsense where a differenced raw
    /// microsecond count would look plausible.
    fn read_channel_activity(&self) -> Result<Option<u16>, FaceError> {
        let raw = self.usb.rr(regs::mt_mib_sdr9(0))?;
        let busy_us = u64::from(regs::field_get(regs::MT_MIB_SDR9_BUSY_MASK, raw));
        let mut last = self.last_activity.lock().unwrap_or_else(|e| e.into_inner());
        let elapsed_us = last.elapsed().as_micros() as u64;
        *last = Instant::now();
        drop(last);
        if elapsed_us == 0 {
            return Ok(None);
        }
        Ok(Some(
            (busy_us.saturating_mul(1000) / elapsed_us).min(1000) as u16
        ))
    }

    /// `(ok, err)` PPDU counts for the window since the last call.
    ///
    /// `err` is the hardware's: `MT_MIB_SDR3`'s `FCS_ERR_MASK` (`GENMASK(31,16)`,
    /// `mt792x_regs.h:104-105`), read by `mt792x_mac_update_mib_stats`
    /// (`mt792x_mac.c:84-85`) — PPDUs the baseband began to demodulate and whose FCS failed,
    /// which is exactly the collision / marginal-decode signature the trait is after.
    ///
    /// ⚠ `ok` is **not** a hardware counter, because this part reports no
    /// good-PPDU counter in the same block. It is this driver's own count of RX units
    /// accepted off USB in the same window. Reporting a structural `0` would have been
    /// "honest" and useless — a permanent zero on the good half reads exactly like a dead
    /// receiver, which is the trap `crate::RX_RAW_FRAMES` documents having actually cost a
    /// debugging session. Both halves cover the same window because both are cleared here.
    fn read_ofdm_counters(&self) -> Result<Option<(u16, u16)>, FaceError> {
        let raw = self.usb.rr(regs::mt_mib_sdr3(0))?;
        let err =
            regs::field_get(regs::MT_MIB_SDR3_FCS_ERR_MASK, raw).min(u32::from(u16::MAX)) as u16;
        let ok = self
            .rx_ok_window
            .swap(0, Ordering::Relaxed)
            .min(u64::from(u16::MAX)) as u16;
        Ok(Some((ok, err)))
    }

    /// `BestEffort`.
    ///
    /// This part has the best hardware clock in the crate (see
    /// [`RadioTime::time_sources`]) and it changes nothing here: there is no ported path
    /// from that clock to a **gated transmit**. Nothing in this file arms a timer that keys
    /// the PA, and `MT_ARB_SCR`'s `TX_DISABLE` — the closest mechanism — is a software gate
    /// held across a register write, not a scheduled instant.
    ///
    /// Promising `PromptBounded` on the strength of a good receive clock would be a Cut-2
    /// capability the scheduler reads and acts on, backed by no mechanism. When a scheduled
    /// transmit does land here it will have a real granularity to declare, because the TXS
    /// timestamp (`MT_TXS4_TIMESTAMP`) shares the RX stamp's domain and could measure it.
    fn tx_discipline(&self) -> TxDiscipline {
        TxDiscipline::BestEffort
    }

    // `set_tx_power` / `set_tx_power_dbm` are deliberately the trait defaults — see the
    // module header. Power on connac2 is firmware-owned (a SAR table pushed through
    // `MCU_CE_CMD(SET_RATE_TX_POWER)` and the per-band SAR TLVs); there is no host register
    // to write, and a knob whose effect nobody can state is worse than none.
    //
    // `set_edcca_ignore` likewise: connac2 exposes no host-visible energy-detect CCA
    // threshold the way the mt76x02's `MT_TXOP_CTRL_CFG.ED_CCA_EN` does. And the knob would
    // be less useful than it sounds — MEASURED on the 8812au, defeating carrier sense on a
    // saturated channel trades collision loss for TX starvation and delivered frames fell
    // 237 -> 26/s. It is a knob for a channel you own, not a contention remedy.
}

// ── RadioTime — the payoff ──────────────────────────────────────────────────

impl RadioTime for Mt7921uBackend {
    /// ★ **The first MediaTek part in this crate that can source common view.**
    ///
    /// Two sources, and they share **one domain**, because they are one counter.
    ///
    /// # 1. The per-frame RXD group-2 stamp — [`RadioClockKind::FreeRunRxStamp`]
    ///
    /// `mt7921/mac.c:307-309` latches `status->timestamp = le32_to_cpu(rxd[0])` out of RXD
    /// group 2 and sets `RX_FLAG_MACTIME_START`. This is what the MT7612U and MT7610U
    /// **cannot** do: `struct mt76x02_rxwi` (`mt76x02_mac.h:97-108`) is
    /// `rxinfo/ctl/tid_sn/rate/rssi[4]/bbp_rxinfo[4]` and carries no time at all, so those
    /// two can only offer a read-now port TSF at 151 µs per read — and a clock you cannot
    /// read more precisely than 151 µs cannot resolve the microsecond-scale offset between
    /// two receivers. Hence `can_common_view` is `false` for both and **`true` here**: two
    /// nodes that hear the same frame get two stamps of one event, on stable free-running
    /// counters, and the difference is the offset. That is the same mechanism that MEASURED
    /// 0.0034 ppm two-node common view on the 8733b and 1.05 µs on the AR9271.
    ///
    /// * `tick_ns = 1_000` — ★ **MEASURED 2026-08-27, and it closes exactly the way the note
    ///   that used to sit here asked for.** `examples/mt7921_bringup.rs` on mds-o5p-3, channel
    ///   6, 15 s: **2401 frames, 2401 of them stamped (100%)**, and the first-to-last stamp
    ///   difference was **15 007 757 ticks across 15 007 907 host microseconds = 1.0000 MHz**.
    ///   Five significant figures against the host clock; the stamp is a microsecond TSF.
    ///
    ///   This mattered because the inference was not safe. The 8733b declared 1 000 here on the
    ///   same reasoning (RX_FLAG_MACTIME_START implies µs; 802.11 mandates a 1 MHz TSF) and
    ///   MEASURED **4 000** — every duration derived from its stamps was 4x short for months.
    ///   The reasoning was sound and the silicon disagreed. Here it happens to agree, and now
    ///   that is a fact rather than a hope.
    /// * `precision_ns = 1_000` ([`LatchPoint::MacDone`]'s floor) — the stamp is latched in
    ///   hardware, so unlike the MT7610U's port TSF the EP0 round trip is **not** in the
    ///   path and does not belong in this number. It is the tick that limits it.
    /// * `latch = MacDone`, not `PhyPreamble`, and that is deliberate. Upstream's flag says
    ///   the value is referenced to the PPDU *start*, which sounds like `PhyPreamble` — but
    ///   `PhyPreamble`'s precision floor is 1 ns and this counter ticks at 1 µs, so
    ///   declaring it would let the source claim a precision the hardware cannot deliver.
    ///   `MacDone`'s 1 µs floor is the honest description of a 1 µs counter, and it is what
    ///   [`RadioTimeSource::free_run_rx_stamp`] uses for every other part here.
    /// * `monotonic = true` — earned: this driver never associates, never beacons and never
    ///   writes `MT_LPON_TCR`'s `SW_WRITE`/`SW_ADJUST`, so nothing resynchronises the
    ///   counter. A beacon-resynced TSF would not qualify.
    /// * ⚠ **32 bits.** The descriptor carries only the low half of a 64-bit counter, so at
    ///   1 µs it wraps every 2³² µs = **4294.967 s ≈ 71 min 35 s**. Differences are valid
    ///   modulo 2³²; a session must unwrap in software or re-anchor periodically, and a pair
    ///   of nodes more than ~35 minutes apart cannot be disambiguated from this field alone.
    ///   Which is exactly what the second source is for.
    ///
    /// # 2. The LPON port TSF — read-now, **same domain**
    ///
    /// ★ **This is a deliberate departure from how the 8733b models its two clocks**, and
    /// from [`RadioTimeSource::port_tsf`]'s own advice to give it a separate domain. That
    /// advice is right when the two are different physical counters, which on the Realtek
    /// part they are — its RXTSFL and its port TSF are separate registers that MEASURED
    /// *different* 4 µs scale errors and had to be corrected independently.
    ///
    /// Here they are **the same counter**: `MT_LPON_UTTR0/1` is the 64-bit whole whose low
    /// half the descriptor ships (see [`mac::Rxd::timestamp`]). Splitting it into two
    /// domains would assert that two readings are incomparable when in fact one is the
    /// unwrapped form of the other — and that comparability is the entire value: the
    /// register read supplies the high word and epoch that resolve the 71-minute wrap, and
    /// it lets a frame's **age** be computed against the clock that stamped it, which
    /// `read_now: false` alone cannot do.
    ///
    /// * `read_now = true`, at ~**804 µs** per read (three EP0 round trips at the MEASURED
    ///   268 µs). That is the number in `precision_ns` for *this* source, because for a
    ///   read-now clock the read is the measurement — the same reasoning that made the
    ///   MT7610U declare 151 000 rather than the 1 000 its tick would suggest.
    /// * `monotonic = true`, overriding [`RadioTimeSource::port_tsf`]'s `false`: that default
    ///   describes a TSF that a received beacon can slam, and this one has no BSS to be
    ///   slammed by.
    ///
    /// Order matters: [`ndn_radio_hal::FaceTimeProfile::derive`] takes `best_clock` from the
    /// **head** of this list, so the per-frame stamp is first.
    ///
    /// [`RadioClockKind::FreeRunRxStamp`]: ndn_time::RadioClockKind::FreeRunRxStamp
    fn time_sources(&self) -> Vec<RadioTimeSource> {
        // Reference: **crystal**, INFERRED FROM THIS PART'S OWN MEASUREMENT, and the inference is
        // stated so a reader can check it. No `xtal`/`crystal` string exists anywhere in
        // `src/connac2/` or `src/mt7921/` — this port never reads a crystal cap — so the kind is not
        // witnessed by a register. What witnesses it is the rate: 15_007_757 ticks across
        // 15_007_907 host microseconds is **-10.0 ppm over 15 s**, five significant figures against
        // the host clock. An RC reference is percent-class (MEASURED in this tree: the LR2021's
        // internal source at +2253 ppm, the Waveshare's at ~-3100 ppm), so a counter three orders of
        // magnitude tighter than that is not sitting on one. The measurement rides along below so
        // the inference is auditable rather than asserted.
        let xtal = ndn_radio_hal::ClockReference::crystal().measured(
            ndn_radio_hal::RateMeasurement::new(-10.0, 15.0, ndn_radio_hal::RateWitness::HostClock),
        );
        vec![
            // ★ The per-frame stamp — what makes this radio a common-view participant.
            RadioTimeSource {
                read_now: true,
                reference: xtal,
                ..RadioTimeSource::free_run_rx_stamp(self.tsf_domain, 1_000)
            },
            // The same counter, read on demand: the high word, the epoch, and frame age. Same
            // counter therefore same reference, necessarily.
            RadioTimeSource {
                precision_ns: regs::measured::EP0_ROUND_TRIP_US * 3 * 1_000,
                tick_ns: 1_000,
                monotonic: true,
                reference: xtal,
                ..RadioTimeSource::port_tsf(self.tsf_domain)
            },
        ]
    }

    fn read_clock(&self, domain: ClockDomainId) -> Result<Option<u64>, FaceError> {
        if domain != self.tsf_domain {
            return Ok(None);
        }
        // The 64-bit whole, so a caller can unwrap a 32-bit RX stamp against it.
        self.read_tsf().map(Some)
    }

    /// `None` — this radio steers nothing, **yet**.
    ///
    /// `MT_LPON_TCR`'s `SW_ADJUST` (`BIT(1)`, `mt7915/regs.h:298`) is a TSF *phase* nudge,
    /// not a rate trim, so it is the wrong actuator: correcting phase without correcting
    /// rate means the correction re-accumulates, which is the exact distinction
    /// [`ndn_radio_hal::ClockSteering`] draws. A real crystal trim on connac2, if one is
    /// host-reachable at all, has not been located in the tree.
    ///
    /// And the field would demand MEASURED `range_ppm` and `resolution_ppm` regardless: a
    /// discipline loop believes both, and a datasheet-width range is exactly the fabricated
    /// number that field warns about. Closing this needs the two-node common-view sweep that
    /// characterised the 8733b's curve — which this part can now *run*, since it has the
    /// sensor. Sensor first, actuator later; the 8733b campaign found itself actuator-limited
    /// at 0.0034 ppm, and here we do not yet have an actuator at all.
    fn clock_steering(&self) -> Option<ndn_radio_hal::ClockSteering> {
        None
    }
}

// ── RadioProfile ────────────────────────────────────────────────────────────

impl RadioProfile for Mt7921uBackend {
    fn capability(&self) -> RadioCapability {
        declared_capability()
    }
}

/// **What this radio is**, as a free function — a static fact about the silicon, not about
/// any open handle, so it is checkable with no dongle plugged in. A capability that can only
/// be asserted on hardware is one nothing verifies.
///
/// * ★ **`he_cap: true` — the only Wi-Fi capability in this crate that may say so.** The
///   MT7921 is 802.11ax silicon and [`mac::encode_rate`] is a real actuator for
///   [`McsDescriptor`]'s `he` / `dcm` / `er_su` (`MT_TX_RATE_MODE` = `HE_SU`/`HE_EXT_SU`,
///   `MT_TX_RATE_DCM`, `MT_TX_RATE_SU_EXT_TONE`). This flag is what unlocks
///   [`McsDescriptor::for_intent`]'s HE reach branch, so setting it on a part that could not
///   transmit HE would aim undecodable ER-SU frames at the world. ⚠ The HE *encoding* is
///   CODE-READ: upstream never transmits a fixed HE rate on this chip
///   (`mt76_connac_mac.c:321-324`), so the first on-air HE frame from this driver will also
///   be the first test of those bits.
/// * **`rate: Wifi { max_mcs: 11, max_nss: 2, max_bw: 2 }`.** Two chains (MEASURED silicon
///   spec, and [`mac::Rxd::CHAINS`] agrees). `max_mcs: 11` is the **HE** ceiling — HE-MCS
///   0-11, which is what `he_cap: true` makes reachable and what
///   [`mac::encode_rate`] clamps to (`m.index.min(11)`). `max_bw: 2` because
///   [`RadioKnobs::set_channel`] tunes 20/40/80 MHz via [`centre_and_cbw`]; declaration and
///   actuator agree.
///
///   ⚠ **11 is mode-dependent and no ordinary path will reach it**, which is worth knowing
///   before reading it as "this radio is 1.6× the mt7612". `McsDescriptor::for_intent`'s
///   `Throughput` branch clamps to the *structural* mode ceiling — 7 for HT, 8 for
///   single-stream VHT — and [`ndn_radio_hal::mcs_for_rssi`] is an 11n heuristic that never
///   returns above 7. So MCS 8-11 are reachable only through an explicit HE
///   [`FrameIo::set_rate`], i.e. when cognition has decided the reach is HE-capable. The
///   number is declared because it is what the hardware and the encoder will accept; it is
///   not a promise that anything currently asks for it.
/// * **`channels`.** Exactly `CHANNELS_2GHZ` + `CHANNELS_5GHZ`, i.e. exactly what
///   `set_channel` accepts — nothing declared that the actuator would reject, and nothing
///   accepted that is not declared.
/// * **`retune_us: None` — not measured.** The MT7610U declares 135 000 because someone
///   timed it three times; this is an MCU command rather than ~150 RF register writes, so it
///   is probably far faster, and "probably" is not a number a hop scheduler may believe.
///   `can_hop` will correctly answer "I cannot say" until someone times
///   `set_channel(149, Bw20)` on the target. Per this field's own rule, an invented figure is
///   worse than `None`.
/// * **`tx_power_dbm: None`** — there is no dBm knob here at all; power is firmware-owned
///   (see the module header). `max_tx_power: 63` is ⚠ **INHERITED from the generic Wi-Fi
///   preset and unverified on this part**, declared knowingly per that preset's own
///   instruction: no ported knob consumes it, [`RadioKnobs::set_tx_power`] is the trait's
///   no-op default, and it is here only so the field is not a degenerate 0 that would read
///   as "this radio cannot transmit".
/// * **`csi: None`.** No host-visible channel state beyond per-frame RSSI/MCS — and on this
///   part not even per-frame SNR, for the structural reason in the module header.
pub fn declared_capability() -> RadioCapability {
    let mut channels = Vec::with_capacity(CHANNELS_2GHZ.len() + CHANNELS_5GHZ.len());
    channels.extend_from_slice(CHANNELS_2GHZ);
    channels.extend_from_slice(CHANNELS_5GHZ);
    RadioCapability {
        // ── PHY-mode / hop fields, added to `RadioCapability` for the sub-GHz family ──
        // `PhyMode` is a LoRa/FSK/BLE-family table with no Wi-Fi modulation in it, so the honest
        // answer for this part is "I cannot say" — NOT a set of one. Per the field docs, an empty
        // `PhyModeSet` is exactly that statement, and a planner must not read it as a single mode.
        // `hop` is autonomous frequency hopping: a Wi-Fi radio has no sequencer of its own, and
        // host-commanded retunes are already priced by `retune_us`.
        phy_modes: ndn_radio_hal::PhyModeSet::empty(),
        phy_current: None,
        hop: None,
        kind: RadioKind::WifiMonitor,
        he_cap: true,
        bands: vec![Band::Band2_4GHz, Band::Band5GHz],
        rate: RateCapability::Wifi {
            max_mcs: 11,
            max_nss: 2,
            // MEASURED-reachable widths: `centre_and_cbw` derives the 802.11 centre for 20/40/80,
            // and `set_channel` sends the matching CMD_CBW_*. 160 MHz has no `Bandwidth` variant in
            // the HAL to ask for, so it is not claimed.
            max_bw: 2,
        },
        channels,
        max_tx_power: 63,
        // ★ This part has NO power actuator. `set_tx_power` is not implemented and `MT_TXD2_POWER_OFFSET`
        // (`connac2/mac.rs`) is defined and never written, so `max_tx_power` below is decorative. Saying
        // so here is the whole point: before this field existed the MT7921AU accepted every back-off
        // cognition asked for and the bandit was rewarded for spatial reuse that never physically
        // happened.
        min_tx_power: None,
        db_per_power_idx: None,
        power_actuated: false,
        tx_power_dbm: None,
        retune_us: None,
        rx_only: false,
        duty_cycle_max: 1.0,
        max_payload: 1500,
        half_duplex: true,
        csi: CsiSupport::None,
    }
}

// ── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {

    /// The 802.11 channelisation is a table, not a formula, and the places a formula goes wrong
    /// are exactly the ones nobody tunes by hand. Pin the boundaries.
    #[test]
    fn centre_channel_derivation() {
        use ndn_radio_hal::Bandwidth::*;
        // 20 MHz: centre is the control channel, on both bands.
        assert_eq!(centre_and_cbw(6, Bw20), Some((6, mcu::CMD_CBW_20MHZ)));
        assert_eq!(centre_and_cbw(149, Bw20), Some((149, mcu::CMD_CBW_20MHZ)));
        // 5 GHz 40 MHz: both members of a pair give the same centre.
        assert_eq!(centre_and_cbw(36, Bw40), Some((38, mcu::CMD_CBW_40MHZ)));
        assert_eq!(centre_and_cbw(40, Bw40), Some((38, mcu::CMD_CBW_40MHZ)));
        assert_eq!(centre_and_cbw(157, Bw40), Some((159, mcu::CMD_CBW_40MHZ)));
        // ...and a channel that is not in any pair has no 40 MHz block.
        assert_eq!(
            centre_and_cbw(165, Bw40),
            None,
            "165 is a lone 20 MHz channel"
        );
        // 5 GHz 80 MHz blocks, at both edges of one block.
        assert_eq!(centre_and_cbw(36, Bw80), Some((42, mcu::CMD_CBW_80MHZ)));
        assert_eq!(centre_and_cbw(48, Bw80), Some((42, mcu::CMD_CBW_80MHZ)));
        assert_eq!(centre_and_cbw(149, Bw80), Some((155, mcu::CMD_CBW_80MHZ)));
        assert_eq!(centre_and_cbw(161, Bw80), Some((155, mcu::CMD_CBW_80MHZ)));
        assert_eq!(centre_and_cbw(165, Bw80), None);
        // 2.4 GHz: 40 MHz flips the secondary to stay inside 1..13; 80 MHz never exists.
        assert_eq!(centre_and_cbw(1, Bw40), Some((3, mcu::CMD_CBW_40MHZ)));
        assert_eq!(centre_and_cbw(11, Bw40), Some((9, mcu::CMD_CBW_40MHZ)));
        assert_eq!(centre_and_cbw(6, Bw80), None, "no 80 MHz in 2.4 GHz");
        // Narrowband modes are refused rather than silently tuned as 20.
        assert_eq!(centre_and_cbw(6, Nb5), None);
        assert_eq!(centre_and_cbw(6, Nb10), None);
    }

    use super::*;

    /// The capability must describe *this* part and no other.
    ///
    /// Three failure modes are worth catching mechanically: a degenerate declaration (empty
    /// channels, a zero rate ceiling — a radio nothing would ever select); a declaration
    /// copy-pasted from one of the two sibling MediaTek backends, which would misroute rate
    /// selection; and — the one unique to this part — losing `he_cap`, which would silently
    /// disconnect the crate's only 802.11ax actuator from the only thing that turns it on.
    #[test]
    fn declared_capability_is_non_degenerate_and_is_not_a_sibling_mt76s() {
        let c = declared_capability();

        assert!(
            !c.channels.is_empty(),
            "a radio with no channels is unusable"
        );
        assert!(!c.bands.is_empty());
        assert!(!c.rx_only, "this part transmits");
        assert!(c.max_payload > 0);
        assert!(c.rate_rank() > 0.0, "a zero rate rank is never selected");

        // ★ The whole point of this backend existing.
        assert!(
            c.he_cap(),
            "the MT7921AU is 802.11ax and mac::encode_rate actuates he/dcm/er_su — this is \
             the ONLY Wi-Fi part in the crate for which he_cap may be true, and cognition \
             only builds an HE descriptor for a radio that claims one"
        );
        assert_eq!(c.max_nss(), 2, "MT7921AU is 2x2");
        assert_eq!(
            c.max_mcs(),
            11,
            "HE-MCS 0-11, the ceiling mac::encode_rate clamps to"
        );
        assert_eq!(
            c.max_bw(),
            2,
            "set_channel now derives the 802.11 centre channel and tunes 20/40/80 MHz"
        );

        // Distinct from BOTH mt76 siblings, on the axes that would misroute traffic.
        let mt7612 = crate::Mt7612uBackend::declared_capability();
        let mt7610 = crate::mt76x0::declared_capability();
        assert_ne!(c, mt7612, "must not declare the MT7612U's radio");
        assert_ne!(c, mt7610, "must not declare the MT7610U's radio");
        assert!(
            !mt7612.he_cap() && !mt7610.he_cap(),
            "if a sibling ever gains he_cap, this test's premise — and the module header's \
             claim to be the crate's only HE actuator — must be revisited"
        );
        // The MT7610U is 1x1; this is 2x2. The MT7612U is 2x2 11ac and stops at MCS9.
        assert!(c.max_nss() > mt7610.max_nss());
        assert!(c.max_mcs() > mt7612.max_mcs());

        // Nothing declared that `set_channel` would reject, and nothing accepted that is not
        // declared: the two lists are the same lists.
        for ch in &c.channels {
            assert!(
                CHANNELS_2GHZ.contains(ch) || CHANNELS_5GHZ.contains(ch),
                "channel {ch} is declared but set_channel would refuse it"
            );
        }
        assert_eq!(
            c.channels.len(),
            CHANNELS_2GHZ.len() + CHANNELS_5GHZ.len(),
            "every channel set_channel accepts must be declared"
        );

        // Unmeasured fields must stay absent rather than plausible.
        assert!(
            c.retune_us.is_none(),
            "the retune has not been timed on this part"
        );
        assert!(
            c.tx_power_dbm.is_none(),
            "there is no dBm knob on this part"
        );
        assert_eq!(
            c.can_hop(20_000),
            None,
            "with retune_us unmeasured, can_hop must answer 'I cannot say' rather than guess"
        );
    }

    /// The board table is the claim about what this backend matches, and it must not drift
    /// from the transport's — which is the list that actually drives enumeration.
    #[test]
    fn board_table_agrees_with_the_transport_and_is_honest_about_vendors() {
        use crate::connac2::usb::MT7921U_VID_PIDS;

        assert_eq!(
            MT7921U_BOARDS.len(),
            MT7921U_VID_PIDS.len(),
            "the documented board table and the table the transport enumerates must match"
        );
        for b in MT7921U_BOARDS {
            assert!(
                MT7921U_VID_PIDS.contains(&(b.vid, b.pid)),
                "{} ({:#06x}:{:#06x}) is documented but the transport would never open it",
                b.name,
                b.vid,
                b.pid
            );
        }

        // ★ The reason MT7921U_PIDS cannot stand alone: four vendor ids, not one.
        let vids: std::collections::BTreeSet<u16> = MT7921U_BOARDS.iter().map(|b| b.vid).collect();
        assert!(
            vids.len() > 1,
            "a flat PID list would imply one vendor; this device table has several"
        );
        assert_eq!(
            MT7921U_PIDS,
            &[MT7921AU_PID],
            "MT7921U_PIDS is the MediaTek-VID subset ONLY — widening it would claim that \
             e.g. 0e8d:6211 exists, which it does not"
        );

        // Exactly one board has been held on this bench; the rest are CODE-READ.
        assert_eq!(
            MT7921U_BOARDS.iter().filter(|b| b.measured).count(),
            1,
            "a PID table in this crate is a claim about what has been measured"
        );
        assert!(MT7921U_BOARDS[0].measured && MT7921U_BOARDS[0].vid == MEDIATEK_VID);
    }

    /// The two clocks must share one domain, the per-frame stamp must come first, and the
    /// list must be the shape `FaceTimeProfile::derive` reads.
    ///
    /// This is checked without a device by constructing the sources the way
    /// [`RadioTime::time_sources`] does — the domain is the only per-device input, and the
    /// property under test is about the *shape* of the declaration, not about any handle.
    #[test]
    fn time_sources_declare_one_counter_and_source_common_view() {
        use ndn_time::RadioClockKind;

        let domain = ClockDomainId(0x0123);
        // Mirrors `time_sources` exactly, reference included — the reference is now half of the
        // capability this test is named after, so leaving it off would test the wrong shape.
        let xtal = ndn_radio_hal::ClockReference::crystal().measured(
            ndn_radio_hal::RateMeasurement::new(-10.0, 15.0, ndn_radio_hal::RateWitness::HostClock),
        );
        let sources = vec![
            RadioTimeSource {
                read_now: true,
                reference: xtal,
                ..RadioTimeSource::free_run_rx_stamp(domain, 1_000)
            },
            RadioTimeSource {
                precision_ns: regs::measured::EP0_ROUND_TRIP_US * 3 * 1_000,
                tick_ns: 1_000,
                monotonic: true,
                reference: xtal,
                ..RadioTimeSource::port_tsf(domain)
            },
        ];

        // ★ The headline: a per-frame RX stamp is what makes common view possible, and it
        // must be the HEAD, because `derive` takes `best_clock` from the head.
        assert_eq!(sources[0].kind, RadioClockKind::FreeRunRxStamp);
        assert!(
            sources
                .iter()
                .any(|s| s.kind == RadioClockKind::FreeRunRxStamp)
        );
        assert_eq!(sources[1].kind, RadioClockKind::PortTsf);

        // ★ ONE domain: MT_LPON_UTTR0/1 and the RXD group-2 stamp are the same counter, so
        // splitting them would assert that two readings of one register are incomparable.
        assert_eq!(sources[0].domain, sources[1].domain);

        // The stamp is latched in hardware, so the EP0 round trip is NOT in its precision;
        // the read-now source is where the 268 us x 3 lives.
        assert_eq!(sources[0].precision_ns, 1_000);
        assert_eq!(sources[1].precision_ns, 804_000);
        assert!(sources[1].precision_ns > sources[0].precision_ns);
        assert!(sources[0].monotonic && sources[1].monotonic);
        assert!(sources[1].read_now);

        // ★ And the second half of the name: the verdict, through the real predicate. A per-frame
        // stamp alone no longer earns it — the crystal reference, MEASURED at -10.0 ppm over 15 s
        // against the host clock, is what makes this radio a common-view participant.
        struct Fixed(Vec<RadioTimeSource>);
        impl ndn_radio_hal::RadioTime for Fixed {
            fn time_sources(&self) -> Vec<RadioTimeSource> {
                self.0.clone()
            }
        }
        let profile = ndn_radio_hal::FaceTimeProfile::derive(
            &Fixed(sources.clone()),
            ndn_radio_hal::TxDiscipline::BestEffort,
        );
        assert!(profile.hw_rx_stamp);
        assert!(profile.can_common_view);
        assert_eq!(profile.clock_reference, Some(xtal));

        // ...and it really is conditional on the reference, not a dressed-up latch test.
        let unwitnessed: Vec<RadioTimeSource> = sources
            .iter()
            .map(|s| s.with_reference(ndn_radio_hal::ClockReference::unknown()))
            .collect();
        let profile = ndn_radio_hal::FaceTimeProfile::derive(
            &Fixed(unwitnessed),
            ndn_radio_hal::TxDiscipline::BestEffort,
        );
        assert!(profile.hw_rx_stamp);
        assert!(!profile.can_common_view);
    }

    /// `resolved_rate`'s worst-receiver rule, checked at the encoding level: the rate a
    /// `MostRobust` frame goes out at must be LEGACY, not HE — even though this is the one
    /// part in the crate that could transmit HE ER-SU.
    ///
    /// A regression here would be silent and expensive: every legacy-only neighbour would
    /// stop hearing discovery and reception reports, and a broadcast bearer has no ACK to
    /// tell anyone.
    #[test]
    fn most_robust_stays_legacy_even_on_the_he_part() {
        let robust = mac::LegacyRate::Ofdm6.rate_val();
        let decoded = mac::decode_rate(robust).expect("OFDM 6M must decode");
        assert_eq!(decoded.mode, mac::RateMode::Ofdm);
        assert!(
            !decoded.mode.is_he(),
            "the universally-decodable rate cannot be HE"
        );

        // And the HE path really is reachable — the levers exist, they are just not the
        // broadcast default. This is the assertion that would fail if `mac::encode_rate`
        // ever stopped honouring the HE flags.
        let er_su = McsDescriptor::he(0).with_er_su().with_dcm();
        let word = mac::encode_rate(&er_su);
        let d = mac::decode_rate(word).expect("HE ER-SU must decode");
        assert_eq!(d.mode, mac::RateMode::HeExtSu);
        assert!(d.mode.is_he());
        assert!(
            d.dcm,
            "DCM is one of the two HE reach levers this part unlocks"
        );
    }
}
