//! Layer: driver — the **802.11ah (Wi-Fi HaLow / S1G) data plane** for both sub-GHz radios in this
//! rig: the Newracom NRC7292 and the Morse Micro MM6108.
//!
//! The control planes live next door ([`crate::nrc7292`], [`crate::morse`]); this module is the
//! `FrameIo` half — injection, capture, and the per-frame metadata that comes back with a captured
//! frame. It exists as one module because the two radios share far more than they differ: the same
//! `FrameFormat::RawNdnS1g` body, the same radiotap TX header, and — verified from both vendors'
//! sources — a **byte-identical S1G radiotap TLV** on receive, decoded once in
//! [`ndn_frame_io::radiotap`] rather than twice here.
//!
//! What they do *not* share is the shape of the data plane itself, and getting that wrong is silent
//! on both parts. That is what this module encodes.
//!
//! # NRC7292 — one netdev, and three claims this module used to make that are FALSE on air
//!
//! The vif is a **separate** `mon0`, added with `iw phy nrc80211 interface add mon0 type monitor`
//! (the phy is *named* `nrc80211`, so `iw phy phyN` fails) — not `halow0` retyped. Three things
//! this module asserted were measured on the associated `halow_demo` pair (`mds-o5p-0` STA /
//! `mds-o5p-3` AP, driver 1.5.2, kernel 7.0.11) on 2026-08-31, and all three were wrong:
//!
//! * ☠ **"A monitor vif puts the whole chip in promiscuous mode, so the managed/AP data path
//!   receives nothing while this face exists."** It does not. With `mon0` up on both ends the
//!   association held and IP ping ran 0% loss the whole session. The driver's `nw->promisc`
//!   short-circuit is never reached, because mac80211 only calls `nrc_mac_add_interface` for a
//!   monitor vif when the hw asks for one (`WANT_MONITOR_VIF`) and this driver does not — the vif
//!   is a **mac80211 software monitor**.
//! * ☠ **"RX arrives with a 34-byte `struct nrc_radiotap_hdr` carrying TSFT, FLAGS, CHANNEL and the
//!   S1G TLV."** It arrives with mac80211's own **18-byte** header: `it_present = 0x0000482e` =
//!   FLAGS|RATE|CHANNEL|DBM_ANTSIGNAL|ANTENNA|RX_FLAGS. **No TSFT bit and no S1G TLV**, on
//!   224/224 frames. That is the driver's monitor path never being entered, so `nrc_radiotap_hdr`
//!   is not what reaches userspace at all.
//! * ★ Consequence: `linux::Nrc7292FrameIo` returns `stamp: None` and `mcs_index: None` on every
//!   frame (MEASURED 0/120 and 0/3), so composing [`crate::nrc7292::Nrc7292Clock`] via
//!   `with_clock` relates a read-now counter to per-frame stamps **that do not exist** through this
//!   vif. `rssi_dbm` is the one metadata field that survives, and only on genuine receives — the
//!   same socket also returns this host's own transmissions (radiotap TX_FLAGS set, `rssi_dbm:
//!   None`), which nothing in `recv_frame` filters.
//!
//! ## ☠ Injection is structurally dead on the stock driver — MEASURED, three instruments
//!
//! 500 frames injected on `mon0` through `AfPacketBackend`: 500 `sendto` successes, 0 errors, and
//!
//! | instrument | reading |
//! |---|---|
//! | sender chip `cli_app show mac tx stats` OK count | **+0** (a 20-ping control on the same chip moved it **+20**) |
//! | peer `mon0` (tcpdump, filtered on our addr2) | **0** |
//! | peer chip `cli_app show mac rx stats` OK count | **+0** |
//!
//! …while the sender's *own* `mon0` showed all 100/100 as radiotap TX_FLAGS echoes. So mac80211
//! accepted the frame and the chip never saw it. Vendor source says why:
//! `nrc_skb_append_tx_info()` sets `p->inject = !!(frame_injection || wlantest)`, so without the
//! out-of-tree `nrc7292/inject_monitor.patch` the descriptor's inject bit is never set for
//! radiotap monitor TX. The only lever reachable without rebuilding the module is the runtime
//! module parameter `wlantest` (`/sys/module/nrc/parameters/wlantest`, mode 0600, currently `N`) —
//! **unverified**, and it sets the inject bit on *every* frame including the AP's beacons, so it
//! is a deliberate, reversible experiment and not a default.
//!
//! Until then the only path that reaches the air on this radio is the **operating vif's data
//! path** (`halow_onair txdata halow0 …`), which under `RawNdnS1g` puts the *same bytes* on the
//! air — `AA AA 03 00 00 00 <ethertype>` inside a QoS-Data MPDU. MEASURED end to end: AP → STA,
//! `ours=3/3`, seq span 497..=499 100% delivered, decoded by `Nrc7292FrameIo::recv_frame`.
//!
//! `Nrc7292FrameIo` adds the two things a plain `AfPacketBackend` on `halow0` cannot do:
//! it composes the radio's **read-now clock** ([`crate::nrc7292::Nrc7292Clock`]) with the
//! backend's per-frame RX stamps — a measured capability that was unreachable through the face,
//! because `AfPacketBackend`'s `RadioTime` reports `read_clock() = None` — and it harvests
//! [`ndn_frame_io::FrameIo::mesh_common_view`] from S1G beacons, which the data plane's own `recv_frame` throws
//! away as "not a data frame".
//!
//! # MM6108 — ☠ two netdevs, and one of them cannot transmit
//!
//! The Morse data plane is **split, by driver construction, not by preference**:
//!
//! * **TX on a mac80211 monitor vif** (`mon0`, created with `iw phy <phy> interface add mon0 type
//!   monitor` — there is no `change_interface`, and `iw dev … set type monitor` returns −95).
//!   Requires the out-of-tree `monitor_inject.patch`, without which
//!   `morse_rc_sta_fill_tx_rates()` dereferences a NULL `info->control.vif` and **hard-locks the
//!   board**, costing a physical power cycle. MEASURED once patched: 9.6 Mbit/s at 8 MHz, and
//!   cross-vendor Morse → NRC7292 71/80.
//! * **RX on `morse0`**, the driver's own global sniffer netdev (`alloc_netdev(0, "morse%d", …)`,
//!   `monitor.c:440`). Not `mon0`, and not the managed vif: `morse_mac_skb_recv` (`mac.c:6539`)
//!   does `if (mors->monitor_mode) { morse_mon_rx(...); goto exit; }` — `ieee80211_rx()` is never
//!   called, so mac80211 delivers nothing anywhere else. MEASURED 488/500 at 1 MHz, 499/500 at
//!   8 MHz.
//! * ☠ **`morse0` cannot transmit.** `morse_mon_xmit` is literally
//!   `/* TODO: allow packet injection */ dev_kfree_skb(skb); return NETDEV_TX_OK;` — a `sendto()`
//!   there succeeds at the syscall and radiates nothing. That is the exact "returned `Ok(())` and
//!   actuated nothing" failure this crate is scarred by, so `MorseFrameIo::new` **refuses it at
//!   construction** rather than letting it be discovered on air.
//!
//! ## ★ The vif roles, and the four ways they fail SILENTLY
//!
//! | interface | role | what it is | what it does NOT do |
//! |---|---|---|---|
//! | `mon0` | **TX** | mac80211 monitor vif (`type` 803, has `phy80211`) | receiving: a capture here shows only this host's own **TX echo** |
//! | `morse0` | **RX** | the driver's global sniffer netdev (`type` 803, **no** `phy80211`) | transmitting: `morse_mon_xmit` frees the skb and returns `NETDEV_TX_OK` |
//! | `wlan0` | operating vif | managed/mesh/AP, must be UP for TX to key up | delivering radiotap — it *structurally cannot*, so a capture there is not evidence |
//!
//! 1. ★★ **`morse0` delivers ZERO frames unless a mac80211 monitor vif exists on the phy.**
//!    `morse_mac_skb_recv` calls `morse_mon_rx` only when `mors->monitor_mode` is set, and that
//!    flag is set only when mac80211 reports a monitor vif (`IEEE80211_CONF_CHANGE_MONITOR`,
//!    `mac.c:3901`; `morse_wiphy_add_iface`, `wiphy.c:1328`). MEASURED A/B, same node, same
//!    second, same channel: **no `mon0` → 0 packets; create `mon0` → 1880.** No error, no log
//!    line, no counter — capture simply blocks forever. This once bought a wrong "RX is broken"
//!    verdict. [`check_morse_vif_roles`] now refuses it at construction, and the TX vif this face
//!    already requires is the very thing that satisfies it.
//! 2. **`mon0` is the TX vif and shows only TX echo.** `morse_mac_skb_recv` (`mac.c:6540`) does
//!    `if (mors->monitor_mode) { morse_mon_rx(...); goto exit; }` — `ieee80211_rx()` is never
//!    reached, so no received frame is ever delivered to a mac80211 vif. The signature of this
//!    mistake is a capture full of "our own frames" and nothing from the air.
//! 3. **A managed vif cannot deliver radiotap.** `wlan0`/`halow0` in managed mode is
//!    `ARPHRD_ETHER`: no TSFT, no S1G TLV, no per-frame RSSI or MCS. Capturing there proves
//!    nothing, and it is not a weaker measurement — it is a different one.
//! 4. ☠ **Netdev packet counters lie on these interfaces.** MEASURED:
//!    `/sys/class/net/morse0/statistics/rx_packets` read **0** while `tcpdump` pulled **324**
//!    frames off that same interface in that same interval; `/sys/class/net/mon0/statistics/`
//!    `rx_packets` reads 0 while tcpdump captures hundreds; and a beacon TX never increments
//!    `wlan0 tx_packets` (26123 while the AP beaconed 3.96M times). Use `morse_cli stats -m`,
//!    `cli_app show mac rx stats`, or the capture itself — never a netdev counter.
//!
//! One further constraint the constructor cannot check but a deployment must honour: a supported
//! *operating* vif (mesh/AP/managed) must be UP or TX produces `Data Tx` delta = 0 even with the
//! patch.
//!
//! # What both radios cannot do, stated rather than faked
//!
//! * **No scheduled TX.** Neither part exposes a host-reachable "transmit at time T" seam, so
//!   [`ndn_frame_io::FrameIo::schedules_tx`] stays `false` on both and `inject_after`/`inject_at_clock` keep the
//!   default (inject now). Morse's MAC core demonstrably *has* an internal scheduled-TX timestamp
//!   (`n_cross_tx_ts`, "AGG crosses scheduled TX ts") but `morse_skb_tx_info` carries neither a
//!   timestamp nor a non-contend flag: the mechanism is in firmware and the host ABI is missing.
//!   Declaring `ScheduledAt` without the seam is worse than silence — the scheduler would skip its
//!   own software gate and the frame would go out ungated.
//! * ★ **No per-frame PHY quality — `CapturedFrame.phy` is `None`, and the reason is the vendor
//!   driver, not the radio.** This was chased down after an on-air run populated `rssi_dbm`,
//!   `mcs_index` and `stamp` 1489/1489 and `phy` **0**/1489. Three candidate causes; the answer is
//!   the third.
//!   * *Our parser never sets it?* True but not the cause: [`ndn_frame_io::frame::parse`] writes
//!     `phy: None` on every arm because there is nothing on the wire to fill it from.
//!   * *Absent from the S1G radiotap TLV?* Yes. The TLV is 6 bytes — format, response indication,
//!     GI, bandwidth, MCS, colour, uplink, signal — and carries no SNR, EVM or CFO field. Neither
//!     driver sets `IEEE80211_RADIOTAP_DBM_ANTNOISE` either: Morse's `it_present` is
//!     `FLAGS|CHANNEL|TSFT|DBM_ANTSIGNAL` (+`RATE`, `TLVS`, `AMPDU_STATUS`) and Newracom's is
//!     `TSFT|FLAGS|CHANNEL|TLV` (+`AMPDU_STATUS`).
//!   * ★ *The chip measures it and the driver drops it.* **Both parts report a per-frame quality
//!     figure to the host and neither puts it in radiotap.** Morse's `struct morse_skb_rx_status`
//!     carries `s8 noise_dbm` ("the most recent noise level measured by the PHY") on **every**
//!     received frame — and at driver release 1.17.9 that field is *never read anywhere in the
//!     driver*, only declared. Newracom's `struct frame_hdr` carries a 6-bit `snr` per frame
//!     (`flags.rx.snr`), which `nrc_mac_rx_h_status` folds into a per-*peer* moving average keyed
//!     on `addr2` (and only when the `signal_monitor` parameter is on), never into
//!     `nrc_add_rx_s1g_radiotap_header`; the monitor path's own `struct rxInfo` has `rcpi` and no
//!     SNR at all.
//!
//!   So `phy: None` is honest and stays. It is **not** faked from a per-peer average (which is a
//!   different quantity, and keyed on a host MAC the named-radio doctrine forbids), and not from
//!   the channel-survey noise floor either — Morse does surface one, per *channel*, through
//!   `survey->noise` (`mac.c:4249`, readable with `iw dev wlan0 survey dump`), which would give a
//!   link-level SNR estimate but never a per-frame one. Closing this properly is a one-field
//!   driver patch on either part: add `BIT(IEEE80211_RADIOTAP_DBM_ANTNOISE)` and the byte, and
//!   `phy.snr_db` follows from `rssi − noise` in the existing parser. Until someone writes it,
//!   the field stays empty and says so.

use ndn_frame_io::{FaceError, RadioCapability, radiotap};
use ndn_radio_hal::bringup::{
    AppliedPower, Assert, BringUp, BringUpFailure, BringUpReport, Ctx, Guards, Plan, PlanId,
    PlanRun, PowerRequest, PumpPolicy, RadioState, Role, Severity, Stage, Step, StepClass, StepId,
    StepOutcome,
};
use ndn_radio_hal::{Band, Bandwidth, CsiSupport, MeshCv, PhyModeSet, RadioKind, RateCapability};
use std::sync::Arc;

// ─────────────────────────────────────────────────────────────────────────────────────────────
// Measured on-air limits
// ─────────────────────────────────────────────────────────────────────────────────────────────

/// ★ MEASURED payload ceiling for one MM6108 MPDU: **1546 bytes**, byte-exact — 1546 delivers,
/// 1547 never arrives.
///
/// The chip discards an oversize frame with no error on any interface, which is the same
/// silent-drop shape as the 2296 → 2272 MTU bug `ndn-frame-io` already documents. Both
/// [`ndn_frame_io::MONITOR_MTU`] (2272) and [`ndn_frame_io::DEFAULT_AMSDU_BODY`] (3839) overshoot
/// it, so `MorseFrameIo` clamps and refuses rather than letting a frame
/// vanish.
pub const MM6108_MAX_PAYLOAD: usize = 1546;

/// MEASURED on-air MPDU cutoff on the MM6108: 1584 bytes (the [`MM6108_MAX_PAYLOAD`] plus the
/// 802.11 + LLC/SNAP headers this face puts in front of it).
pub const MM6108_MAX_MPDU: usize = 1584;

/// A-MSDU body budget for the MM6108 — one MPDU's worth, since the aggregate *is* the MPDU.
/// Passed to `AfPacketBackend::with_amsdu_cap`.
pub const MM6108_AMSDU_BODY: usize = MM6108_MAX_PAYLOAD;

/// The Morse driver's global sniffer netdev name prefix (`alloc_netdev(0, "morse%d", …)`).
/// Receive-only: see `MorseFrameIo::new`.
pub const MORSE_SNIFFER_PREFIX: &str = "morse";

/// Default sysfs path of the patched driver's injection MCS module parameter.
///
/// ⚠ **UNVERIFIED spelling.** `inject_mcs` does **not** exist in the vendor driver at either
/// release on this machine (`git grep inject_mcs` is empty at 1.16.4 and 1.17.9); it comes from the
/// out-of-tree `monitor_inject.patch` that lives on the opi5pro deployment and is not in any repo
/// here. The *effect* is measured (walking MCS 0 → 7 took delivered throughput 2.15 → 7.06 Mbit/s,
/// the single biggest lever found on this radio); the path string is a bench note. Override it with
/// `MorseFrameIo::with_inject_mcs_param` if the patch names it differently.
pub const MORSE_INJECT_MCS_PARAM: &str = "/sys/module/morse/parameters/inject_mcs";

/// Default sysfs path of the patched driver's injection-bandwidth module parameter (0/1/2/3 =
/// 1/2/4/8 MHz). Same UNVERIFIED-spelling caveat as [`MORSE_INJECT_MCS_PARAM`]; the effect is
/// MEASURED twice on air (the RX-side S1G TLV bandwidth field tracked a 0..3 sweep as 1/2/4/8 MHz,
/// and a receiver parked at 4 MHz decoded `inject_bw` 0 and 2 at 300/300 while `inject_bw` 3 gave
/// 0/300). The emitted width is `min(inject_bw, operating_bw)`.
pub const MORSE_INJECT_BW_PARAM: &str = "/sys/module/morse/parameters/inject_bw";

// ─────────────────────────────────────────────────────────────────────────────────────────────
// Interface-pairing rules (platform-neutral, so they are unit-tested on every host)
// ─────────────────────────────────────────────────────────────────────────────────────────────

/// Is `iface` one of the Morse driver's sniffer netdevs (`morse0`, `morse1`, …)?
///
/// Those are the *only* interfaces that receive on this radio, and `morse_mon_xmit` frees every
/// skb handed to them, so they are also the only ones that must never be used for TX.
pub fn is_morse_sniffer(iface: &str) -> bool {
    iface
        .strip_prefix(MORSE_SNIFFER_PREFIX)
        .is_some_and(|n| !n.is_empty() && n.chars().all(|c| c.is_ascii_digit()))
}

/// Validate an MM6108 (tx, rx) interface pair against the two configurations that fail **silently**
/// on this radio. Pure and total — no I/O — so it is checked on every host, not only on the node.
///
/// * `tx == rx` — whichever name it is, one of the two directions is dead: on `morse0` TX is a
///   no-op, on `mon0` RX never fires.
/// * `tx` is a sniffer netdev — `sendto()` returns success and nothing radiates.
///
/// The receive name is deliberately **not** constrained: it must be the driver's sniffer netdev,
/// but a netdev can be renamed and a wrong guess here would block a working deployment, whereas the
/// two rules above are properties of the driver's code and hold under any name it is given.
pub fn check_morse_ifaces(tx: &str, rx: &str) -> Result<(), FaceError> {
    if is_morse_sniffer(tx) {
        return Err(FaceError::Io(std::io::Error::new(
            std::io::ErrorKind::InvalidInput,
            format!(
                "{tx} cannot transmit: morse_mon_xmit() frees the skb and returns NETDEV_TX_OK, so \
                 sendto() succeeds and nothing radiates. Inject on a mac80211 monitor vif (mon0) \
                 and receive on {rx}."
            ),
        )));
    }
    if tx == rx {
        return Err(FaceError::Io(std::io::Error::new(
            std::io::ErrorKind::InvalidInput,
            format!(
                "the MM6108 data plane is split and cannot use one interface for both directions \
                 (got {tx} twice): TX must be a mac80211 monitor vif, RX must be the driver's \
                 morseN sniffer netdev — monitor mode diverts all receive there and mac80211 sees \
                 nothing"
            ),
        )));
    }
    Ok(())
}

// ─────────────────────────────────────────────────────────────────────────────────────────────
// Interface-NATURE rules — the preconditions whose failure mode is a SILENT ZERO
// ─────────────────────────────────────────────────────────────────────────────────────────────

/// `ARPHRD_IEEE80211_RADIOTAP` (`include/uapi/linux/if_arp.h`), the ARP hardware type a netdev
/// reports in `/sys/class/net/<iface>/type` when its frames carry a radiotap header.
///
/// A managed/AP/mesh netdev is `ARPHRD_ETHER` (1) and **structurally cannot deliver radiotap** —
/// no TSFT, no S1G TLV, no per-frame RSSI or MCS — so capturing there proves nothing about the
/// air. Both HaLow radios' receive netdevs must be this value: the Morse sniffer sets it in
/// `morse_mon_setup` (`dev->type = ARPHRD_IEEE80211_RADIOTAP`) and mac80211 sets it for every
/// monitor vif.
pub const ARPHRD_IEEE80211_RADIOTAP: u32 = 803;

/// The prefix every Morse bus driver's name shares: `morse_spi` (`spi.c`), `morse_sdio`
/// (`sdio.c`), `morse_usb` (`usb.c`) at driver release 1.17.9. Used to tell a monitor vif on the
/// *Morse* phy from one on some other radio's phy — a distinction that is invisible in the
/// interface name and fatal to receive (see [`check_morse_vif_roles`]).
pub const MORSE_DRIVER_PREFIX: &str = "morse";

/// What `/sys/class/net/<iface>` says about an interface's **nature** — not its name, and not
/// whether it is up, but *what kind of thing it is*.
///
/// Split out from the I/O that gathers it (`halow/linux.rs`) so the rules below are decided by a
/// pure function and unit-tested on every host, including the ones with no radio in them.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct IfaceNature {
    /// `/sys/class/net/<iface>/type`, the ARP hardware type. [`ARPHRD_IEEE80211_RADIOTAP`] (803)
    /// for a monitor netdev, 1 (`ARPHRD_ETHER`) for a managed/AP vif.
    pub arphrd: u32,
    /// The mac80211 phy this interface belongs to (`readlink /sys/class/net/<iface>/phy80211`),
    /// or `None` when the interface is **not a mac80211 vif at all**.
    ///
    /// ★ That `None` is load-bearing rather than incidental: the Morse driver's sniffer netdev is
    /// a bare `alloc_netdev(0, "morse%d", …)` with no `wireless_dev`, so it never has this link,
    /// while `mon0` always does. It is the only sysfs-visible way to tell the two apart, and they
    /// must never be swapped.
    pub mac80211_phy: Option<String>,
    /// The kernel driver behind that phy (basename of `phy80211/device/driver`), when the link
    /// resolves. `None` means "sysfs did not say", and the rules that read it are then skipped —
    /// this is the least certain of the three, so it fails open where the others fail closed.
    pub phy_driver: Option<String>,
}

fn refuse(msg: String) -> FaceError {
    FaceError::Io(std::io::Error::new(std::io::ErrorKind::InvalidInput, msg))
}

/// ★★ **The monitor-vif precondition, as one error message with one explanation.**
///
/// On the MM6108 the TX interface is not only the transmitter: it is what turns RECEIVE on.
/// `morse_mac_skb_recv` routes a received frame to the `morseN` sniffer netdev only while
/// `mors->monitor_mode` is set, and that flag is set only when mac80211 reports an **open** monitor
/// vif (`IEEE80211_CONF_CHANGE_MONITOR`, `mac.c:3901`; `morse_wiphy_add_iface`, `wiphy.c:1328`).
/// So all three of "there is no `mon0`", "`mon0` is down" and "`mon0` is not a mac80211 vif" have
/// the *same* consequence — `morse0` delivers **zero** frames, silently, with no error, no log line
/// and no counter — and they get the same message, `why` naming which one it was.
///
/// MEASURED A/B, one node, one second, one channel: **no monitor vif → 0 packets; monitor vif →
/// 1880.** That silence once bought a wrong "RX is broken" verdict, which is the whole reason this
/// is an error rather than a comment. `MorseFrameIo::new` raises it for the two cases that are
/// invisible to [`check_morse_vif_roles`] (the interface is missing, or present but down) before it
/// ever opens a socket.
pub fn morse_monitor_vif_error(tx: &str, rx: &str, why: &str) -> FaceError {
    refuse(format!(
        "{tx} is not a usable mac80211 monitor vif ({why}) — and on this radio that breaks \
         RECEIVE, not just transmit. The driver hands frames to {rx} only while \
         mors->monitor_mode is set, and that flag is set only when mac80211 reports an OPEN \
         monitor vif (IEEE80211_CONF_CHANGE_MONITOR). Without one, {rx} delivers ZERO frames and \
         reports no error at all. MEASURED A/B, same node/second/channel: no monitor vif -> 0 \
         packets; monitor vif -> 1880. Fix: `iw phy <morse phy> interface add {tx} type monitor && \
         ip link set {tx} up` — find the phy with `readlink /sys/class/net/<morse managed \
         iface>/phy80211`, whose driver is {MORSE_DRIVER_PREFIX}_spi / {MORSE_DRIVER_PREFIX}_sdio \
         / {MORSE_DRIVER_PREFIX}_usb."
    ))
}

/// ★ Validate the MM6108's **vif roles** — the preconditions that, when violated, produce zero
/// frames and no error whatsoever.
///
/// [`check_morse_ifaces`] rules on the *names*; this rules on what the two interfaces **are**, and
/// it is the check that would have saved a bench week. Every rule here is a MEASURED failure, not
/// a defensive guess:
///
/// 1. **The RX netdev must carry radiotap** (`arphrd == `[`ARPHRD_IEEE80211_RADIOTAP`]). A managed
///    vif (`wlan0`, `halow0`) cannot, so a capture there yields no S1G TLV and no HaLow frame.
/// 2. **The RX netdev must NOT be a mac80211 vif.** ☠ MEASURED: capturing on `mon0` shows only
///    this host's own **TX echo** — "N frames, all our own" is the signature — because
///    `morse_mac_skb_recv` (`mac.c:6540`) hands every received frame to `morse_mon_rx` and
///    `goto exit`s, so `ieee80211_rx()` is never called and mac80211 delivers nothing anywhere.
///    Receive is `morse0` only. A renamed sniffer netdev is still accepted: it is recognised by
///    having no `phy80211`, not by its name.
/// 3. ★★ **The TX interface must be a mac80211 monitor vif, and that is ALSO the receive
///    precondition.** The driver routes frames to `morse0` only while `mors->monitor_mode` is
///    set, and that flag is set *only* when mac80211 reports a monitor vif
///    (`IEEE80211_CONF_CHANGE_MONITOR`, `mac.c:3901`; `morse_wiphy_add_iface`, `wiphy.c:1328`).
///    With no monitor vif on the phy, `morse0` delivers **ZERO** frames, silently, forever.
///    MEASURED A/B on one node in one second on one channel: no `mon0` → **0** packets;
///    `iw phy … interface add mon0 type monitor` → **1880**. That measurement once bought a wrong
///    "RX is broken" verdict, which is why it is an error here and not a comment.
/// 4. **That monitor vif must be on the *Morse* phy.** A monitor vif on some other radio's phy
///    satisfies mac80211 and leaves `mors->monitor_mode` false — the same silent zero, one step
///    further from suspicion. Checked only when the driver name resolves.
pub fn check_morse_vif_roles(
    tx: &str,
    rx: &str,
    tx_nature: &IfaceNature,
    rx_nature: &IfaceNature,
) -> Result<(), FaceError> {
    // ── RX: the driver's sniffer netdev, and nothing else ────────────────────────────────────
    if rx_nature.arphrd != ARPHRD_IEEE80211_RADIOTAP {
        return Err(refuse(format!(
            "{rx} cannot deliver radiotap: /sys/class/net/{rx}/type is {}, not \
             {ARPHRD_IEEE80211_RADIOTAP} (ARPHRD_IEEE80211_RADIOTAP). A managed/AP netdev has no \
             radiotap header at all — no TSFT, no S1G TLV, no per-frame RSSI or MCS — so a capture \
             there is not evidence about the air. Receive on the Morse driver's sniffer netdev \
             (morse0).",
            rx_nature.arphrd
        )));
    }
    if let Some(phy) = rx_nature.mac80211_phy.as_deref() {
        return Err(refuse(format!(
            "{rx} is a mac80211 monitor vif (on {phy}), not the Morse driver's sniffer netdev. \
             MEASURED: capture there shows only this host's own TX ECHO and zero frames from the \
             air — morse_mac_skb_recv() gives every received frame to morse_mon_rx() and returns, \
             so ieee80211_rx() is never called and mac80211 delivers nothing. Receive on morse0 \
             (any name works: the sniffer netdev is the one with no phy80211 link)."
        )));
    }

    // ── TX: a mac80211 monitor vif on the Morse phy — which is ALSO what turns RX on ─────────
    if tx_nature.arphrd != ARPHRD_IEEE80211_RADIOTAP {
        return Err(refuse(format!(
            "{tx} cannot inject: /sys/class/net/{tx}/type is {}, not \
             {ARPHRD_IEEE80211_RADIOTAP} (ARPHRD_IEEE80211_RADIOTAP), so it is not a monitor \
             netdev and a radiotap frame written to it is not a transmission. Create one: \
             `iw phy <morse phy> interface add {tx} type monitor && ip link set {tx} up` (there is \
             no change_interface on this driver: `iw dev … set type monitor` returns -95).",
            tx_nature.arphrd
        )));
    }
    let Some(phy) = tx_nature.mac80211_phy.as_deref() else {
        return Err(morse_monitor_vif_error(
            tx,
            rx,
            &format!("no /sys/class/net/{tx}/phy80211, so it is not a mac80211 vif at all"),
        ));
    };
    if let Some(drv) = tx_nature.phy_driver.as_deref()
        && !drv.to_ascii_lowercase().starts_with(MORSE_DRIVER_PREFIX)
    {
        return Err(refuse(format!(
            "{tx} is a monitor vif on {phy}, whose driver is {drv:?} — not a Morse radio \
             ({MORSE_DRIVER_PREFIX}_spi / {MORSE_DRIVER_PREFIX}_sdio / {MORSE_DRIVER_PREFIX}_usb). \
             A monitor vif on the WRONG phy satisfies mac80211 and leaves the Morse chip's \
             mors->monitor_mode false, so {rx} stays silent with no error. Find the Morse phy: \
             `readlink /sys/class/net/*/phy80211` and check \
             `readlink /sys/class/net/<if>/device/driver` for {MORSE_DRIVER_PREFIX}_*."
        )));
    }
    Ok(())
}

/// Validate the NRC7292's single interface: it must be in **monitor** mode.
///
/// One netdev carries both directions on this part, so there is no role to confuse — but the
/// managed-vif trap is identical and just as silent. `nrc_mac_rx` only takes the
/// `nrc_mac_s1g_monitor_rx` path when `nw->promisc`, which `add_interface` sets for a MONITOR vif;
/// on a managed `halow0` the frames go to mac80211 with no radiotap header, so an `AF_PACKET`
/// capture sees an ordinary Ethernet-shaped feed with none of the S1G metadata this face reads.
///
/// The driver name is deliberately **not** checked here (unlike the Morse case, where a monitor
/// vif on the wrong phy silences a *different* netdev): here a wrong-phy interface simply is not
/// this radio, and the caller named it explicitly.
pub fn check_nrc_iface(iface: &str, nature: &IfaceNature) -> Result<(), FaceError> {
    if nature.arphrd != ARPHRD_IEEE80211_RADIOTAP {
        return Err(refuse(format!(
            "{iface} is not in monitor mode: /sys/class/net/{iface}/type is {}, not \
             {ARPHRD_IEEE80211_RADIOTAP} (ARPHRD_IEEE80211_RADIOTAP). A managed vif cannot deliver \
             a radiotap header, so this face would capture no TSFT, no S1G TLV, no RSSI and no MCS \
             — and injection would not radiate. Fix: `ip link set {iface} down && iw dev {iface} \
             set type monitor && ip link set {iface} up`.",
            nature.arphrd
        )));
    }
    Ok(())
}

// ─────────────────────────────────────────────────────────────────────────────────────────────
// Capability descriptors
// ─────────────────────────────────────────────────────────────────────────────────────────────

/// Read a HAL [`Bandwidth`] as an S1G width request, **identically on both HaLow radios**.
///
/// ⚠ [`Bandwidth`] enumerates 20/40/80/10/5 MHz and **cannot express S1G's 1/2/4/8 MHz at all**.
/// That is a HAL gap (see the crate docs for what the enum would need); until it is closed, every
/// HaLow backend has to decide what an inexpressible request means, and the one thing they must not
/// do is decide it *differently* — the whole point of this bearer is that one wireless face covers
/// both radios, and a caller must not get a refusal from one and a shrug from the other.
///
/// The reading, and why each case is what it is:
///
/// * [`Bandwidth::Bw20`] → `Ok(None)`, **"no width preference"**. This is the enum's `Default` and
///   what `Bandwidth::from_code(0)` produces, which is what every Wi-Fi-shaped plan passes. Reading
///   it as a literal 20 MHz demand would refuse every tune ever issued.
/// * [`Bandwidth::Nb5`] → 1 MHz, [`Bandwidth::Nb10`] → 2 MHz. The narrowband members are the only
///   two that carry a "narrower than 20" intent at all, so they are the least-wrong carriers for
///   the two narrow S1G widths. ⚠ **This is a convention of this crate, not a HAL statement**: 5 and
///   10 MHz are not 1 and 2 MHz. It is honoured only when the channel already has that width.
/// * [`Bandwidth::Bw40`] / [`Bandwidth::Bw80`] → **`Err`**. No S1G channel is that wide, so there is
///   no width they could plausibly mean and no way to serve them. Accepting them and tuning at
///   1 MHz anyway would be a silent lie about the width actually transmitted.
///
/// The caller then checks the returned `Some(w)` against the width its channel number carries —
/// which on S1G is where width really lives — and refuses a mismatch.
pub fn s1g_width_request(bw: Bandwidth) -> Result<Option<u8>, FaceError> {
    match bw {
        Bandwidth::Bw20 => Ok(None),
        Bandwidth::Nb5 => Ok(Some(1)),
        Bandwidth::Nb10 => Ok(Some(2)),
        Bandwidth::Bw40 | Bandwidth::Bw80 => Err(FaceError::Io(std::io::Error::new(
            std::io::ErrorKind::InvalidInput,
            format!(
                "{bw:?} has no S1G meaning — HaLow channels are 1/2/4/8 MHz and the width is a \
                 property of the channel number, so pick the channel that has the width"
            ),
        ))),
    }
}

/// The shared 802.11ah capability skeleton for both parts, with the corrections the
/// `RadioCapability::wifi_halow_s1g` preset gets wrong.
///
/// ★ **`max_mcs` is 7, not 10, and that is a bug fix rather than a conservatism.** S1G MCS10 is a
/// **1 MHz-only, repetition-coded BPSK** mode: it is the most *robust* rate and half the speed of
/// MCS0 — so the index is not a monotone ladder at its top, and `max_mcs` would not be a *maximum
/// rate*. [`ndn_radio_hal::s1g_phy_rate_bps`] carries the arithmetic
/// (`s1g_phy_rate_bps(10, 1, ..) * 2 == s1g_phy_rate_bps(0, 1, ..)`).
///
/// ⚠ **Which consumers this actually matters to — corrected, because the first version of this
/// note named the wrong one.** It claimed `RadioCapability::mcs_for_rssi` would hand the strongest
/// link the slowest rate. It would not: that method is
/// `mcs_for_rssi(rssi).min(max_mcs)` and the free `mcs_for_rssi` independently caps at
/// `MAX_RELIABLE_MCS = 7`, so `.min(10)` is a no-op and MCS10 is unreachable through it.
/// `McsDescriptor::for_intent` likewise clamps to a structural `mode_ceiling` of 7 (non-VHT).
/// The consumers that *do* read `max_mcs` unclamped are:
///
/// * **the contextual bandit** (`ndn-radio-cognition`, `contextual.rs`):
///   `w.mcs = (m + arm.mcs_delta).clamp(0, max_mcs)`. Declaring 10 puts index 10 inside the
///   exploration range, where the arm's reward model reads a higher index as a more aggressive
///   rate and would be learning from the exact opposite outcome.
/// * **`RadioCapability::rate_rank`**: `(max_mcs / 9.0 + (max_nss - 1) / 3.0) / 2.0`. With
///   `max_mcs = 10` the numerator exceeds full scale, so a 1×1 sub-GHz radio would out-rank a
///   2-stream VHT part on the axis used to *choose* a radio.
///
/// (Corroborated on both parts: the NRC7292's `show autotxgain` key list enumerates "MCS 0".."MCS 7"
/// and then a *separate* "MCS 10"; the Morse driver puts MCS10 behind a `mcs10_mode` module
/// parameter that defaults to disabled.) MCS10 is reachable by name — `cli_app test mcs 10`, or
/// Morse's `mcs10_mode` — never through the rate ladder.
///
/// `kind` is [`RadioKind::WifiHaLow`], not the preset's `WifiMonitor`, so that nothing downstream
/// has to guess a HaLow radio from its channel numbers.
///
/// `max_bw: 0` is a placeholder that means "20 MHz" in that field's encoding and is meaningless
/// here — `Bandwidth`/`RateCapability` have no S1G widths at all. It is left as the preset has it
/// because there is no truthful value to put there; the real width travels with the channel (see
/// `crate::morse::us_s1g_channel`) and comes back per frame in
/// [`radiotap::S1gInfo::bandwidth_mhz`].
fn halow_base(channels: Vec<u8>, max_payload: usize) -> RadioCapability {
    RadioCapability {
        kind: RadioKind::WifiHaLow,
        he_cap: false,
        bands: vec![Band::Sub1GHz],
        rate: RateCapability::Wifi {
            max_mcs: 7,
            max_nss: 1,
            max_bw: 0,
        },
        channels,
        max_tx_power: 63,
        min_tx_power: None,
        db_per_power_idx: None,
        // Set per part below: on Morse this is mode-dependent (the driver drops a TX-power set when
        // IEEE80211_CONF_MONITOR is on), which is exactly what this flag exists to prevent lying
        // about.
        power_actuated: false,
        tx_power_dbm: None,
        retune_us: None,
        rx_only: false,
        // 802.11ah is licence-exempt sub-GHz but, unlike LoRa, uses CSMA/LBT rather than a hard
        // duty fraction. On the Morse the real ceiling is readable (`morse_cli duty_cycle`) and
        // regdomain-derived; on the NRC7292 `set duty` is MEASURED DEAD (silently refuses,
        // regdomain-gated) so nothing could enforce a fraction anyway.
        duty_cycle_max: 1.0,
        max_payload,
        half_duplex: true,
        // The per-frame SNR either part reports is a station statistic, not a channel estimate;
        // `CsiSupport::Coarse` would claim a phystatus export neither has.
        csi: CsiSupport::None,
        // `PhyMode` is a sub-GHz-modem enum (Lora/Fsk/Ble/Flrc/…) with no S1G/OFDM member, so
        // "I cannot say" is the only true answer — never a set of one.
        phy_modes: PhyModeSet::empty(),
        phy_current: None,
        hop: None,
    }
}

/// Capability of an NRC7292 monitor face. `channels` are the driver's **alias** (non-S1G shadow)
/// numbers, because that is what mac80211 and `iw` speak on this driver — e.g. alias 161 →
/// shadow 5805 MHz → S1G 925.0 MHz at 2 MHz wide (cross-checked against `nrc-bd.c`'s
/// `g_bd_ch_table` joined with `nrc-s1g.c`'s `s1g_ch_table_us`).
///
/// `max_payload` is left at the conventional 1500: UNVERIFIED for S1G on this part. `show uinfo`
/// reports a per-peer `max mpdu_len` that would settle it.
pub fn nrc7292_capability(channels: Vec<u8>) -> RadioCapability {
    halow_base(channels, 1500)
}

/// Capability of an MM6108 monitor face, with the one number this radio has actually had measured:
/// [`MM6108_MAX_PAYLOAD`].
///
/// `power_actuated` stays **false**, deliberately. The MM6108 does have a genuine dBm axis — SDR-
/// validated at 0.986 dB/dB with 0.22 dB rms residual over a 21.5 dB span, through the driver's
/// debugfs row — but `morse_mac_ops_config` gates the whole TX-power block on
/// `!(conf->flags & IEEE80211_CONF_MONITOR)`, and this face *is* a monitor configuration. Declaring
/// `true` here would report a knob that does not act in the mode we run in, which is the precise
/// thing the flag was added to prevent. A caller that has verified actuation on its own vif can
/// raise it.
pub fn mm6108_capability(channels: Vec<u8>) -> RadioCapability {
    halow_base(channels, MM6108_MAX_PAYLOAD)
}

/// Decode the S1G per-frame metadata from a captured buffer, for callers that want the fields
/// [`ndn_frame_io::CapturedFrame`] has nowhere to put — received bandwidth, PPDU format, BSS
/// colour, and (Morse only) the exact frequency in kHz.
///
/// `CapturedFrame` carries `rssi_dbm`, `mcs_index` and `stamp`, and that is all; the width in
/// particular is the established on-air oracle for this bearer *and* the missing half of turning an
/// S1G MCS into a bit rate, so it is worth reaching for directly. Returns `None` when the buffer is
/// not valid radiotap or carried no S1G TLV.
pub fn s1g_metadata(buf: &[u8]) -> Option<(radiotap::S1gInfo, Option<u32>)> {
    let info = radiotap::parse(buf)?;
    Some((info.s1g?, info.freq_khz))
}

/// Harvests **mesh common-view observations from S1G beacons** out of a raw monitor capture.
///
/// [`ndn_frame_io::frame::parse`] correctly refuses an S1G beacon — it is an 802.11 Extension
/// frame, not a data frame — so the data plane's `recv_frame` never sees one, and these are exactly
/// the frames a common-view estimator wants. This type sits on the raw bytes before that decode.
/// It is deliberately platform-neutral and byte-driven so that its two load-bearing filters are
/// unit-tested rather than reasoned about.
///
/// ⚠ **Filter 1: group by transmitter AND Frame Control variant.** The S1G Beacon's `fc1` bits
/// select which of Next-TBTT / Compressed-SSID / ANO are present, which moves everything after the
/// 4-octet partial TSF. Mixing variants from one transmitter measured **sd = 1004 µs, span
/// 3366 µs**; splitting the *same capture* by `fc1` gave **sd = 5.4 µs** (n=12, fc1=0x08) and
/// **sd = 5.8 µs** (n=108, fc1=0x0b), which independently agree on drift to 0.01 ppm. A 170× error
/// that looks entirely plausible. [`MeshCv`] carries one observation at a time with no room to say
/// which variant produced it, so the first variant seen from a transmitter is pinned and the others
/// are dropped.
///
/// ⚠ **Filter 2: mesh transmitters only, by default.** [`ndn_frame_io::FrameIo::mesh_common_view`]'s contract is
/// beacons from *our* mesh — locally-administered BSSIDs, i.e. the ephemeral nonces the named-radio
/// doctrine uses — not infrastructure APs. Note what that costs on this bench: the NRC7292 AP the
/// 5.8 µs common view was measured against beacons from `00:c0:ca:b4:65:e2`, a **universally**
/// administered Newracom address, so it is filtered out by default. A caller measuring against an
/// AP must opt in with [`new(false)`](Self::new) and know that it is stepping outside the trait's
/// stated scope.
#[derive(Debug, Default)]
pub struct MeshCvHarvester {
    mesh_only: bool,
    count: u64,
    latest: Option<MeshCv>,
    fc1_by_sa: std::collections::HashMap<[u8; 6], u8>,
}

impl MeshCvHarvester {
    /// `mesh_only = true` honours [`ndn_frame_io::FrameIo::mesh_common_view`]'s contract (locally-administered
    /// transmitters only). Pass `false` to also accept infrastructure beacons — useful for bench
    /// measurement against an AP, and a departure from the trait's documented scope.
    pub fn new(mesh_only: bool) -> Self {
        Self {
            mesh_only,
            ..Self::default()
        }
    }

    /// Offer one raw capture (`radiotap ++ 802.11 ++ …`). Returns `true` when it was an acceptable
    /// S1G beacon and a new observation was recorded.
    pub fn observe(&mut self, buf: &[u8]) -> bool {
        let Some(info) = radiotap::parse(buf) else {
            return false;
        };
        if info.bad_fcs() {
            return false;
        }
        let Some(tsft) = info.tsft else {
            return false; // no local stamp ⇒ nothing to compare against
        };
        let Some(mut body) = buf.get(info.header_len..) else {
            return false;
        };
        if info.fcs_included() {
            match body.len().checked_sub(4).and_then(|n| body.get(..n)) {
                Some(b) => body = b,
                None => return false,
            }
        }
        let Some(off) = crate::nrc7292::s1g_beacon_offset(body, tsft) else {
            return false;
        };
        // Bit 1 of the first address octet is the universal/local bit.
        if self.mesh_only && off.sa[0] & 0x02 == 0 {
            return false;
        }
        let pinned = *self.fc1_by_sa.entry(off.sa).or_insert(off.fc1);
        if pinned != off.fc1 {
            return false;
        }
        self.count += 1;
        self.latest = Some(MeshCv {
            // ★ Both values are in the beacon's own 32-bit microsecond modulus. The beacon carries
            // only a 4-octet partial TSF, so our 64-bit stamp is truncated to match; that is what
            // keeps their difference meaningful across the ~71.6 minute wrap. Widening one without
            // widening the other silently produces a ~4.3e9 µs offset.
            peer_tsf: u64::from(off.peer_tsf_us),
            our_rxtsfl: u64::from(tsft as u32),
            count: self.count,
            bssid: off.sa,
            // A bare S1G beacon advertises no network-time belief, so the receiver treats this
            // transmitter as a stratum-0 reference (#75).
            belief: None,
        });
        true
    }

    /// The most recent accepted observation.
    pub fn latest(&self) -> Option<MeshCv> {
        self.latest
    }
}

// ─────────────────────────────────────────────────────────────────────────────────────────────
// M6 · §1.4 — THE PLAN.  802.11ah / HaLow: Newracom NRC7292 and Morse Micro MM6108.
// ─────────────────────────────────────────────────────────────────────────────────────────────
//
// Specification: `docs/bringup-contract.md` §1.4/§1.5/§5-M6:
//
//   > HaLow's plan is `OutOfBand` steps + asserts — the report names `modprobe`/`iw`/
//   > `hostapd_s1g`/`morse_cli` as **unverified provenance**, which is better than shell history
//   > and worse than a real plan; the spec says so rather than pretending.
//
// **So this document says so too, in the code, once, plainly.** Every rung below is
// [`StepClass::OutOfBand`]. Not one of them brings this radio up. `modprobe nrc` / `modprobe
// morse_spi`, `iw phy … interface add mon0 type monitor`, `hostapd_s1g`, `morse_cli channel …` —
// all of it happens in a shell, before this process starts, with no record anywhere that a later
// measurement can be read against. This plan **cannot fix that**. What it can do, and does, is:
//
//   * name each out-of-band actor in `established_by`, so the report says WHO established the
//     state rather than leaving it implied;
//   * VALIDATE what can be validated from `/sys/class/net` — which is exactly the set of rules
//     that, when violated on these two radios, produce **zero frames and no error whatsoever**;
//   * mark the rest `unverified` in the report, in writing, so a number taken on this bearer is
//     never quoted as if the channel and width were known.
//
// ★ Everything here is deliberately **platform-independent** and lives beside the pure rules in
// this module, not in `halow/linux.rs`. The reason is the one already given for `IfaceNature`:
// the rules are decided by pure functions so they are unit-testable on every host, including the
// ones with no radio in them — and the plan is a rule, not I/O. The Linux data plane gathers the
// `IfaceNature`s and hands them here.
//
// ⚠ **What is NOT validated, and would silently be wrong**:
//   * the CHANNEL and the WIDTH. Both are set by `morse_cli` / `hostapd_s1g` / `iw` and neither is
//     reliably readable back through this path; `morse_cli` is the authoritative reader on the
//     MM6108 and this crate does not shell out to it. The channel in the report is *what the
//     caller said*, not what the chip is on.
//   * whether the out-of-tree injection patch is loaded. On the MM6108 the unpatched driver
//     **hard-locks the board** (a NULL `info->control.vif` in `morse_rc_sta_fill_tx_rates`) and
//     costs a physical power cycle; on the NRC7292 the unpatched driver accepts every injected
//     frame and transmits none (MEASURED: 500 `sendto` successes, sender chip TX-OK +0, peer
//     RX-OK +0). Neither is detectable from a socket.

/// The synthetic "register" ids the HaLow asserts read. There are no registers on this bearer —
/// the state lives in `/sys/class/net` — so these are stable identifiers for the report's `reg`
/// column, chosen to be obviously not addresses.
const HALOW_REG_RX_ARPHRD: u32 = 0x5F5F_0001;
const HALOW_REG_TX_ARPHRD: u32 = 0x5F5F_0002;

/// **The out-of-band state a HaLow plan validates.** Gathered by the Linux data plane
/// (`halow/linux.rs`), decided here.
///
/// It is a plain value rather than a live handle for the reason the whole module is arranged this
/// way: `run_plan` needs an `Arc<B>`, and the thing being brought up is not an object this process
/// owns — it is an interface some shell command created. This type IS that state, snapshotted.
#[derive(Clone, Debug)]
pub struct HalowIfaces {
    /// The interface frames are injected on. On the NRC7292 this is the same netdev as `rx`.
    pub tx: String,
    /// The interface frames are captured on. On the MM6108 this is the driver's sniffer netdev.
    pub rx: String,
    pub tx_nature: IfaceNature,
    pub rx_nature: IfaceNature,
}

/// The NRC7292's out-of-band state — one netdev, both directions.
pub struct Nrc7292BringUp(pub HalowIfaces);
/// The MM6108's out-of-band state — TX on a mac80211 monitor vif, RX on the driver's sniffer
/// netdev, and they are NOT interchangeable.
pub struct Mm6108BringUp(pub HalowIfaces);

fn s_halow_driver_loaded(
    ifaces: &HalowIfaces,
    c: &mut Ctx<'_>,
    want_prefix: Option<&str>,
) -> Result<StepOutcome, FaceError> {
    match (ifaces.tx_nature.phy_driver.as_deref(), want_prefix) {
        (Some(drv), Some(want)) if drv.to_ascii_lowercase().starts_with(want) => {
            Ok(StepOutcome::Branch("driver matches the expected family"))
        }
        (Some(drv), Some(want)) => {
            c.warn(format!(
                "{} sits on a phy whose driver is {drv:?}, not {want}* — this is NOT the radio you \
                 think it is. On the MM6108 a monitor vif on the WRONG phy satisfies mac80211 and \
                 leaves the Morse chip's mors->monitor_mode false, so the sniffer netdev stays \
                 silent with no error",
                ifaces.tx
            ));
            Ok(StepOutcome::Branch("driver mismatch"))
        }
        (Some(drv), None) => {
            c.warn(format!(
                "phy driver reported as {drv:?} (not checked on this part)"
            ));
            Ok(StepOutcome::Branch("driver reported"))
        }
        (None, _) => {
            c.warn(format!(
                "sysfs did not name the driver behind {}'s phy, so WHICH module established this \
                 interface is unknown to this process. `modprobe` provenance is unverified either \
                 way; here it is not even reported",
                ifaces.tx
            ));
            Ok(StepOutcome::Branch("driver unknown"))
        }
    }
}

fn s_nrc_driver_loaded(b: &Arc<Nrc7292BringUp>, c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
    s_halow_driver_loaded(&b.0, c, None)
}

fn s_morse_driver_loaded(
    b: &Arc<Mm6108BringUp>,
    c: &mut Ctx<'_>,
) -> Result<StepOutcome, FaceError> {
    s_halow_driver_loaded(&b.0, c, Some(MORSE_DRIVER_PREFIX))
}

fn s_nrc_monitor_vif(b: &Arc<Nrc7292BringUp>, _c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
    check_nrc_iface(&b.0.rx, &b.0.rx_nature)?;
    Ok(StepOutcome::Done)
}

fn s_morse_vif_roles(b: &Arc<Mm6108BringUp>, _c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
    check_morse_ifaces(&b.0.tx, &b.0.rx)?;
    check_morse_vif_roles(&b.0.tx, &b.0.rx, &b.0.tx_nature, &b.0.rx_nature)?;
    Ok(StepOutcome::Done)
}

fn s_halow_channel_unverified(c: &mut Ctx<'_>, tool: &str) -> Result<StepOutcome, FaceError> {
    c.warn(format!(
        "channel {} and width are UNVERIFIED PROVENANCE: they were set out of band by {tool}, and \
         this process neither issued that command nor reads the result back. The channel in this \
         report is WHAT THE CALLER SAID. ⚠ On S1G the width travels WITH the channel number (US \
         8 MHz = ch 12/28/44, op class 4; 904.5 MHz can never be 8 MHz), so a channel number \
         quoted without its width is not a statement about the air",
        c.state_ref().channel
    ));
    Ok(StepOutcome::Branch("unverified"))
}

fn s_nrc_channel(_b: &Arc<Nrc7292BringUp>, c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
    s_halow_channel_unverified(c, "`iw`/`hostapd_s1g`/the vendor `cli_app`")
}

fn s_morse_channel(_b: &Arc<Mm6108BringUp>, c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
    s_halow_channel_unverified(c, "`morse_cli channel` / `hostapd_s1g`")
}

fn s_nrc_inject_patch(_b: &Arc<Nrc7292BringUp>, c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
    c.warn(
        "☠ INJECTION IS STRUCTURALLY DEAD ON THE STOCK DRIVER, and this process cannot tell \
         whether the fix is loaded. `nrc_skb_append_tx_info()` sets the descriptor's inject bit \
         only when `frame_injection || wlantest`, so without the out-of-tree \
         nrc7292/inject_monitor.patch every injected frame is accepted by mac80211 and never \
         reaches the chip. MEASURED, three instruments: 500 sendto successes, sender chip TX-OK \
         +0 (a 20-ping control on the same chip moved it +20), peer mon0 0, peer chip RX-OK +0 — \
         while our OWN mon0 showed 100/100 as radiotap TX_FLAGS echoes. The socket succeeds \
         either way, so this is a deployment precondition, not a check",
    );
    Ok(StepOutcome::Branch("unverifiable"))
}

fn s_morse_inject_patch(
    _b: &Arc<Mm6108BringUp>,
    c: &mut Ctx<'_>,
) -> Result<StepOutcome, FaceError> {
    c.warn(
        "☠ Monitor injection needs the out-of-tree morse `monitor_inject.patch`, and this process \
         cannot tell whether it is loaded. WITHOUT it, `morse_rc_sta_fill_tx_rates()` dereferences \
         a NULL `info->control.vif` and HARD-LOCKS THE BOARD — a physical power cycle, not an \
         error. With it: MEASURED 9.6 Mbit/s at 8 MHz and cross-vendor Morse -> NRC7292 71/80. \
         Nothing readable from a socket distinguishes the two states before the first inject",
    );
    Ok(StepOutcome::Branch("unverifiable"))
}

// ── the rungs ────────────────────────────────────────────────────────────────
//
// The `why` strings are shared between the two parts wherever the reason is the same, so the two
// plans cannot drift into disagreeing about a bearer they share.

const WHY_DRIVER: &str = "★ UNVERIFIED PROVENANCE, named. The kernel module was loaded by \
    `modprobe` in a shell this process never saw; all it can do is read back which driver sysfs \
    says is behind the phy. §5-M6 is explicit that this is better than shell history and worse \
    than owning the sequence, and that the report should say so rather than pretend.";

const WHY_CHANNEL: &str = "★ UNVERIFIED PROVENANCE, named, and the one that most often makes a \
    HaLow number wrong. The channel and the S1G width are established by `morse_cli` / \
    `hostapd_s1g` / `iw`, and this crate neither issues those commands nor reads them back — \
    `morse_cli` is the authoritative reader on the MM6108 and shelling out to it is not something \
    a driver should do. So the report carries the caller's claim, flagged as a claim.";

const NRC_R_DRIVER: Step<Nrc7292BringUp> = Step {
    id: StepId("driver_loaded"),
    stage: Stage::Attach,
    class: StepClass::OutOfBand {
        established_by: "`modprobe nrc` (out of band, before this process started)",
    },
    why: WHY_DRIVER,
    must_follow: &[],
    must_precede: &[],
    run: s_nrc_driver_loaded,
};

const NRC_R_MONITOR_VIF: Step<Nrc7292BringUp> = Step {
    id: StepId("monitor_vif"),
    stage: Stage::RxEnable,
    class: StepClass::OutOfBand {
        established_by: "`iw phy nrc80211 interface add mon0 type monitor && ip link set mon0 up` \
                         — note the phy is NAMED `nrc80211`, so `iw phy phyN` fails",
    },
    why: "★ The one HaLow precondition that IS checkable, and its violation is silent. \
          `nrc_mac_rx` takes the `nrc_mac_s1g_monitor_rx` path only when `nw->promisc`, which \
          `add_interface` sets for a MONITOR vif; on a managed `halow0` the frames go to mac80211 \
          with NO radiotap header at all — no TSFT, no S1G TLV, no RSSI, no MCS — so the face \
          captures an Ethernet-shaped feed and reads as a broken radio rather than a misconfigured \
          interface. Validated from `/sys/class/net/<if>/type`, which is where the answer is.",
    must_follow: &[StepId("driver_loaded")],
    must_precede: &[],
    run: s_nrc_monitor_vif,
};

const NRC_R_CHANNEL: Step<Nrc7292BringUp> = Step {
    id: StepId("channel_out_of_band"),
    stage: Stage::Tune,
    class: StepClass::OutOfBand {
        established_by: "`iw` / `hostapd_s1g` / the vendor `cli_app` (out of band, unverified)",
    },
    why: WHY_CHANNEL,
    must_follow: &[],
    must_precede: &[],
    run: s_nrc_channel,
};

const NRC_R_INJECT_PATCH: Step<Nrc7292BringUp> = Step {
    id: StepId("inject_patch"),
    stage: Stage::TxEnable,
    class: StepClass::OutOfBand {
        established_by: "the out-of-tree `nrc7292/inject_monitor.patch` at module build time, or \
                         the `wlantest` module parameter — neither readable from here",
    },
    why: "★ The transmit half of this bearer's unverifiable provenance, and it is not a small \
          caveat: on the stock driver injection reaches the socket and never reaches the air, with \
          every layer reporting success. Exactly the shape the bring-up contract exists for, on a \
          bearer where the contract's own instruments cannot reach.",
    must_follow: &[],
    must_precede: &[],
    run: s_nrc_inject_patch,
};

const MORSE_R_DRIVER: Step<Mm6108BringUp> = Step {
    id: StepId("driver_loaded"),
    stage: Stage::Attach,
    class: StepClass::OutOfBand {
        established_by: "`modprobe morse_spi` / `morse_sdio` / `morse_usb` (out of band, before \
                         this process started)",
    },
    why: WHY_DRIVER,
    must_follow: &[],
    must_precede: &[],
    run: s_morse_driver_loaded,
};

const MORSE_R_VIF_ROLES: Step<Mm6108BringUp> = Step {
    id: StepId("vif_roles"),
    stage: Stage::RxEnable,
    class: StepClass::OutOfBand {
        established_by: "`iw phy <morse phy> interface add mon0 type monitor && ip link set mon0 \
                         up` (there is no `change_interface` on this driver: `iw dev … set type \
                         monitor` returns -95)",
    },
    why: "★★ The check that would have saved a bench week, and the reason this bearer gets a plan \
          at all. The MM6108's data plane is SPLIT by driver construction: TX on a mac80211 \
          monitor vif, RX on the driver's own sniffer netdev — and the TX vif is what turns RECEIVE \
          ON. `morse_mac_skb_recv` routes a frame to the sniffer only while `mors->monitor_mode` is \
          set, and that flag is set only when mac80211 reports an OPEN monitor vif. MEASURED A/B, \
          one node, one second, one channel: no monitor vif -> 0 packets; monitor vif -> 1880. \
          Swapping the two interfaces is equally silent: a capture on a mac80211 monitor vif shows \
          only this host's own TX echo, because `morse_mac_skb_recv` hands the frame to \
          `morse_mon_rx()` and returns without ever calling `ieee80211_rx()`.",
    must_follow: &[StepId("driver_loaded")],
    must_precede: &[],
    run: s_morse_vif_roles,
};

const MORSE_R_CHANNEL: Step<Mm6108BringUp> = Step {
    id: StepId("channel_out_of_band"),
    stage: Stage::Tune,
    class: StepClass::OutOfBand {
        established_by: "`morse_cli channel` / `hostapd_s1g` (out of band, unverified)",
    },
    why: WHY_CHANNEL,
    must_follow: &[],
    must_precede: &[],
    run: s_morse_channel,
};

const MORSE_R_INJECT_PATCH: Step<Mm6108BringUp> = Step {
    id: StepId("inject_patch"),
    stage: Stage::TxEnable,
    class: StepClass::OutOfBand {
        established_by: "the out-of-tree morse `monitor_inject.patch` at module build time — not \
                         readable from here",
    },
    why: "☠ The most expensive unverifiable precondition in the fleet: without the patch the first \
          injected frame HARD-LOCKS the board and costs a physical power cycle. Named in the \
          report because a bring-up that cannot check something this consequential should at least \
          say that it cannot.",
    must_follow: &[],
    must_precede: &[],
    run: s_morse_inject_patch,
};

const NRC_STEPS: &[Step<Nrc7292BringUp>] = &[
    NRC_R_DRIVER,
    NRC_R_MONITOR_VIF,
    NRC_R_CHANNEL,
    NRC_R_INJECT_PATCH,
];

const MORSE_STEPS: &[Step<Mm6108BringUp>] = &[
    MORSE_R_DRIVER,
    MORSE_R_VIF_ROLES,
    MORSE_R_CHANNEL,
    MORSE_R_INJECT_PATCH,
];

/// The exclusions every HaLow plan shares. A blank cell is a written decision, not an absence.
const HALOW_EXCLUDED: &[(Stage, &str)] = &[
    (
        Stage::Firmware,
        "the chip's firmware is loaded by the kernel module at probe, out of band. On the MM6108 \
         this crate CAN patch and boot `.mac_imem` (verified in chip memory), but that is a \
         workbench operation, not a bring-up rung.",
    ),
    (
        Stage::PowerOn,
        "power sequencing is the bus driver's (`morse_spi` / the NRC SPI driver). ⚠ Neither is \
         recoverable from here: a wedged NRC7292 is NOT reset by a modprobe reload \
         (ENABLE_HW_RESET is compiled out — the fix is an SPI unbind/bind on BOTH ends), and a \
         Morse GPIO reset line must never be pulsed by hand (it left the line stuck as an output \
         and faked a hardware fault).",
    ),
    (
        Stage::Power,
        "TX power on this bearer is a real absolute-dBm axis reached through nl80211 / the vendor \
         tools, and it is NOT set at open. Declaring a power this plan did not set would be the \
         exact defect the contract exists to remove.",
    ),
    (
        Stage::Calibrate,
        "neither part exposes a host-driven calibration: on both the NRC7292 and the MM6108 the \
         RF calibration is the firmware's, applied before this process can see the chip. There is \
         nothing here to run, skip or deviate from, and a rung that reported `Done` for work the \
         firmware did would be the report claiming credit for a sequence it did not execute.",
    ),
];

const NRC_PLAN: Plan<Nrc7292BringUp> = Plan {
    id: PlanId {
        part: "nrc7292",
        name: "monitor-out-of-band",
        ver: 1,
    },
    role: Role::TransmitAndReceive,
    steps: NRC_STEPS,
    excluded: HALOW_EXCLUDED,
};

const MORSE_PLAN: Plan<Mm6108BringUp> = Plan {
    id: PlanId {
        part: "mm6108",
        name: "split-monitor-out-of-band",
        ver: 1,
    },
    role: Role::TransmitAndReceive,
    steps: MORSE_STEPS,
    excluded: HALOW_EXCLUDED,
};

const _: () = NRC_PLAN.check_or_panic();
const _: () = MORSE_PLAN.check_or_panic();

/// The NRC7292's plan — four `OutOfBand` rungs, one of which is genuinely checkable.
pub static PLAN_NRC7292: Plan<Nrc7292BringUp> = NRC_PLAN;
/// The MM6108's plan — the same shape, plus the split-vif rule that produces zero frames and no
/// error when it is violated.
pub static PLAN_MM6108: Plan<Mm6108BringUp> = MORSE_PLAN;

/// §1.5 — the one thing on this bearer that can be READ BACK rather than asserted in prose.
///
/// `/sys/class/net/<if>/type` is the gate the out-of-band `iw` command was supposed to have set,
/// and reading it is the HaLow analogue of reading `REG_TXPAUSE`. ⚠ `Warn` on introduction, per
/// §5/M-hazards.
const ASSERTS_NRC7292: &[Assert<Nrc7292BringUp>] = &[Assert {
    id: StepId("rx_is_radiotap"),
    reg: HALOW_REG_RX_ARPHRD,
    read: |b: &Nrc7292BringUp| Ok(b.0.rx_nature.arphrd),
    want: ARPHRD_IEEE80211_RADIOTAP,
    mask: u32::MAX,
    why: "the interface must be ARPHRD_IEEE80211_RADIOTAP (803). A managed vif is ARPHRD_ETHER (1) \
          and carries no radiotap at all, so every per-frame fact this face reads — TSFT, the S1G \
          TLV, RSSI, MCS — is absent, and injection does not radiate. The `monitor_vif` rung \
          refuses that case; this reads the same gate back at the end, because the interface could \
          have been retyped underneath a running process.",
    severity: Severity::Warn,
}];

/// §1.5 for the MM6108 — **both** interfaces, because the split is the whole hazard.
const ASSERTS_MM6108: &[Assert<Mm6108BringUp>] = &[
    Assert {
        id: StepId("rx_is_radiotap"),
        reg: HALOW_REG_RX_ARPHRD,
        read: |b: &Mm6108BringUp| Ok(b.0.rx_nature.arphrd),
        want: ARPHRD_IEEE80211_RADIOTAP,
        mask: u32::MAX,
        why: "the sniffer netdev must be ARPHRD_IEEE80211_RADIOTAP (803) or a capture on it is not \
              evidence about the air.",
        severity: Severity::Warn,
    },
    Assert {
        id: StepId("tx_is_radiotap"),
        reg: HALOW_REG_TX_ARPHRD,
        read: |b: &Mm6108BringUp| Ok(b.0.tx_nature.arphrd),
        want: ARPHRD_IEEE80211_RADIOTAP,
        mask: u32::MAX,
        why: "the TX monitor vif must be ARPHRD_IEEE80211_RADIOTAP (803) — and on this part that \
              gate controls RECEIVE as well: `mors->monitor_mode` is set only while mac80211 \
              reports an open monitor vif. A vif that went down after this face was built takes \
              the receiver with it, silently.",
        severity: Severity::Warn,
    },
];

/// Why neither HaLow part can answer §4's transmit question (A) from its own state.
const HALOW_TX_UNPROVABLE: &str = "an AF_PACKET `sendto` returning Ok proves a socket write completed and nothing more. On BOTH \
     of these radios that is MEASURED insufficient: on the stock NRC7292 driver 500 sends moved \
     the sender chip's own TX-OK counter by +0, and on the MM6108 an unpatched driver locks the \
     board instead of transmitting. The chip counters that would answer (A) are behind the vendor \
     CLIs (`cli_app show mac tx stats`, `morse_cli`), which are out of band — the same boundary \
     every rung in this plan is about. Prove transmission with a witness receiver.";

impl BringUp for Nrc7292BringUp {
    fn plan(role: Role) -> Option<&'static Plan<Self>> {
        (role == Role::TransmitAndReceive).then_some(&PLAN_NRC7292)
    }
    fn asserts() -> &'static [Assert<Self>] {
        ASSERTS_NRC7292
    }
    fn tx_unprovable_reason() -> Option<&'static str> {
        Some(HALOW_TX_UNPROVABLE)
    }
}

impl BringUp for Mm6108BringUp {
    fn plan(role: Role) -> Option<&'static Plan<Self>> {
        (role == Role::TransmitAndReceive).then_some(&PLAN_MM6108)
    }
    fn asserts() -> &'static [Assert<Self>] {
        ASSERTS_MM6108
    }
    fn tx_unprovable_reason() -> Option<&'static str> {
        Some(HALOW_TX_UNPROVABLE)
    }
}

/// The regime a HaLow plan starts from — the caller's claim, which the rungs then flag.
fn halow_state(channel: u8, bw: Bandwidth) -> RadioState {
    RadioState {
        channel,
        bw,
        format: "RawNdnS1g(0x8624)",
        role: Role::TransmitAndReceive,
        // ⚠ NOT `NoActuator`: both parts have a real absolute-dBm power axis (nl80211 / the vendor
        // tools). This says the BRING-UP did not set power, which is the truth — and is a
        // different statement from "this radio has no power knob".
        power: AppliedPower::no_actuator(PowerRequest::NoActuator),
        rate: ndn_radio_hal::RateState::unreported(),
        warm: None,
        contention: None,
        pump: PumpPolicy::CallerOwns,
        facts: Vec::new(),
    }
}

/// Run [`PLAN_NRC7292`] over an already-configured interface and return the report.
///
/// ⚠ It brings nothing up. See the M6 block above: every rung is `OutOfBand`, and the value is
/// that the report NAMES who established each piece of state and marks the rest unverified.
pub fn nrc7292_bringup(
    ifaces: HalowIfaces,
    channel: u8,
    bw: Bandwidth,
    capability: RadioCapability,
) -> Result<BringUpReport, FaceError> {
    let dev = Arc::new(Nrc7292BringUp(ifaces));
    let run = PlanRun::new(
        "NRC7292",
        ndn_radio_hal::DeviceAddress::NetDev(dev.0.rx.clone()),
        halow_state(channel, bw),
    );
    run_halow(
        <Nrc7292BringUp as BringUp>::bring_up(&dev, &run),
        capability,
    )
}

/// Run [`PLAN_MM6108`] over an already-configured pair of interfaces and return the report.
pub fn mm6108_bringup(
    ifaces: HalowIfaces,
    channel: u8,
    bw: Bandwidth,
    capability: RadioCapability,
) -> Result<BringUpReport, FaceError> {
    let dev = Arc::new(Mm6108BringUp(ifaces));
    let run = PlanRun::new(
        "MM6108",
        ndn_radio_hal::DeviceAddress::NetDev(format!("{} / {}", dev.0.tx, dev.0.rx)),
        halow_state(channel, bw),
    );
    run_halow(<Mm6108BringUp as BringUp>::bring_up(&dev, &run), capability)
}

#[allow(clippy::result_large_err)]
fn run_halow(
    out: Result<(BringUpReport, Guards), BringUpFailure>,
    capability: RadioCapability,
) -> Result<BringUpReport, FaceError> {
    let (report, guards) = out.map_err(|f| {
        eprintln!(
            "HaLow bring-up VALIDATION FAILED at `{}` — the partial report:\n{}",
            f.failed_at,
            f.report.render()
        );
        f.source
    })?;
    debug_assert!(guards.is_empty(), "the HaLow plans produce no guards");
    Ok(report.with_capability(capability))
}

// Explicit path so the module resolves identically whether this file is compiled in place or
// pulled in by a `#[path]`-mounted compile-check harness (which is how the Linux-only data plane is
// verified from a macOS bench — the root crate cannot cross-compile, its libusb dependency needs a
// C toolchain, but this module needs no C at all).
#[cfg(target_os = "linux")]
#[path = "halow/linux.rs"]
mod linux;
#[cfg(target_os = "linux")]
pub use linux::{MorseFrameIo, Nrc7292FrameIo};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sniffer_netdevs_are_recognised_by_shape_not_by_one_name() {
        assert!(is_morse_sniffer("morse0"));
        assert!(is_morse_sniffer("morse1"));
        assert!(is_morse_sniffer("morse12"));
        assert!(!is_morse_sniffer("morse"), "the driver always appends %d");
        assert!(!is_morse_sniffer("mon0"));
        assert!(!is_morse_sniffer("wlan0"));
        assert!(!is_morse_sniffer("morsex"));
    }

    /// ☠ The single most expensive misconfiguration on this radio: injecting on `morse0`. The
    /// syscall succeeds, the counters move, and nothing goes on air. It must fail at construction.
    #[test]
    fn injecting_on_the_sniffer_netdev_is_refused() {
        let e = check_morse_ifaces("morse0", "morse0").expect_err("must be refused");
        let msg = format!("{e}");
        assert!(msg.contains("cannot transmit"), "got {msg:?}");
        assert!(msg.contains("morse_mon_xmit"), "say WHY: {msg:?}");
    }

    /// One interface for both directions is always wrong on this part, whichever name it is.
    #[test]
    fn a_single_interface_is_refused() {
        assert!(check_morse_ifaces("mon0", "mon0").is_err());
        assert!(check_morse_ifaces("wlan0", "wlan0").is_err());
    }

    // ─────────────────────────────────────────────────────────────────────────────────────────
    // Interface-nature rules — one test per MEASURED silent-zero
    // ─────────────────────────────────────────────────────────────────────────────────────────

    /// The deployment that works: `mon0` a monitor vif on a Morse phy, `morse0` the sniffer.
    fn good_tx() -> IfaceNature {
        IfaceNature {
            arphrd: ARPHRD_IEEE80211_RADIOTAP,
            mac80211_phy: Some("phy0".into()),
            phy_driver: Some("morse_spi".into()),
        }
    }
    fn good_rx() -> IfaceNature {
        IfaceNature {
            arphrd: ARPHRD_IEEE80211_RADIOTAP,
            mac80211_phy: None, // the sniffer netdev is not a mac80211 vif — this is the marker
            phy_driver: None,
        }
    }

    #[test]
    fn the_working_deployment_is_accepted() {
        check_morse_vif_roles("mon0", "morse0", &good_tx(), &good_rx()).expect("the measured rig");
        // A renamed sniffer is still a sniffer: recognised by having no phy80211, not by name.
        check_morse_vif_roles("mon0", "halowmon", &good_tx(), &good_rx()).expect("rename is fine");
        // And the driver name is checked case-insensitively, on the prefix only.
        let mut tx = good_tx();
        for d in ["morse_spi", "morse_sdio", "morse_usb", "MORSE_SDIO"] {
            tx.phy_driver = Some(d.into());
            check_morse_vif_roles("mon0", "morse0", &tx, &good_rx()).expect(d);
        }
    }

    /// ★★ THE expensive one. `morse0` delivers ZERO frames unless a mac80211 monitor vif exists on
    /// the phy — `morse_mac_skb_recv` only calls `morse_mon_rx` while `mors->monitor_mode` is set,
    /// and only mac80211 reporting a monitor vif sets it. MEASURED A/B on one node in one second:
    /// no monitor vif → 0 packets, monitor vif → 1880. It fails with no error, no log and no
    /// counter, and it once bought a wrong "RX is broken" verdict. So: a TX interface that is not a
    /// mac80211 vif must be refused, and the message must name the fix.
    #[test]
    fn a_tx_interface_that_is_not_a_mac80211_vif_is_refused_because_rx_dies_silently() {
        let tx = IfaceNature {
            arphrd: ARPHRD_IEEE80211_RADIOTAP, // looks like a monitor netdev...
            mac80211_phy: None, // ...but is not a mac80211 vif, so monitor_mode is off
            phy_driver: None,
        };
        let e = check_morse_vif_roles("mon0", "morse0", &tx, &good_rx()).expect_err("must refuse");
        let m = format!("{e}");
        assert!(m.contains("phy80211"), "name what is missing: {m:?}");
        assert!(m.contains("monitor_mode"), "say WHY receive dies: {m:?}");
        assert!(
            m.contains("ZERO"),
            "say that the failure is a silent zero: {m:?}"
        );
        assert!(m.contains("1880"), "carry the A/B measurement: {m:?}");
        assert!(
            m.contains("interface add") && m.contains("type monitor"),
            "name the FIX: {m:?}"
        );
    }

    /// The three shapes of "there is no usable monitor vif" — absent, down, not a mac80211 vif —
    /// are one consequence and get one message. `MorseFrameIo::new` raises the first two before it
    /// opens a socket, because those are the shapes an operator who never created `mon0` actually
    /// hits; a bare `ENOENT` there would say nothing about why receive is silent.
    #[test]
    fn every_missing_monitor_vif_reports_the_receive_consequence() {
        for why in [
            "no such interface",
            "the interface exists but is DOWN, and a closed monitor vif does not count",
            "no /sys/class/net/mon0/phy80211, so it is not a mac80211 vif at all",
        ] {
            let m = format!("{}", morse_monitor_vif_error("mon0", "morse0", why));
            assert!(m.contains(why), "carry which case it was: {m:?}");
            assert!(
                m.contains("breaks RECEIVE"),
                "lead with the surprise: {m:?}"
            );
            assert!(
                m.contains("morse0"),
                "name the netdev that goes silent: {m:?}"
            );
            assert!(m.contains("monitor_mode"), "say the mechanism: {m:?}");
            assert!(m.contains("ZERO") && m.contains("no error"), "{m:?}");
            assert!(
                m.contains("0 packets") && m.contains("1880"),
                "carry the A/B measurement: {m:?}"
            );
            assert!(
                m.contains("interface add") && m.contains("type monitor"),
                "name the FIX: {m:?}"
            );
        }
    }

    /// A monitor vif on the wrong phy is the same silent zero, one step further from suspicion: it
    /// satisfies mac80211 and leaves the *Morse* chip's `monitor_mode` false. Checked only when the
    /// driver name resolves — `None` there means "sysfs did not say", and the least certain input
    /// is the one allowed to fail open.
    #[test]
    fn a_monitor_vif_on_someone_elses_phy_is_refused() {
        let mut tx = good_tx();
        for wrong in ["brcmfmac", "mt76x2u", "ath9k_htc", "nrc80211"] {
            tx.phy_driver = Some(wrong.into());
            let e =
                check_morse_vif_roles("mon0", "morse0", &tx, &good_rx()).expect_err("wrong phy");
            let m = format!("{e}");
            assert!(m.contains(wrong), "name the driver found: {m:?}");
            assert!(
                m.contains("monitor_mode"),
                "say why morse0 stays silent: {m:?}"
            );
        }
        // Unknown driver ⇒ the rule is skipped, not guessed at.
        tx.phy_driver = None;
        check_morse_vif_roles("mon0", "morse0", &tx, &good_rx())
            .expect("an unreadable driver link must not block a working deployment");
    }

    /// ☠ Trap #1, as an error instead of a bench week: `mon0` is the TX vif and a capture there
    /// shows only this host's own TX ECHO. The signature is "N frames, all our own". Recognised by
    /// nature (it has a `phy80211`), so a renamed sniffer is unaffected.
    #[test]
    fn receiving_on_the_mac80211_monitor_vif_is_refused_it_only_echoes_our_tx() {
        let e = check_morse_vif_roles("mon0", "mon1", &good_tx(), &good_tx()).expect_err("echo");
        let m = format!("{e}");
        assert!(m.contains("TX ECHO"), "name the symptom: {m:?}");
        assert!(m.contains("morse_mon_rx"), "say WHY: {m:?}");
        assert!(
            m.contains("morse0"),
            "name the interface that does work: {m:?}"
        );
    }

    /// ☠ Trap #2: a managed vif is `ARPHRD_ETHER` and structurally cannot deliver radiotap, so a
    /// capture there is not a weaker measurement — it is a different one, with no TSFT, no S1G TLV,
    /// no RSSI and no MCS. Refused on either side of the split.
    #[test]
    fn a_managed_vif_cannot_deliver_radiotap_and_is_refused_on_both_sides() {
        let managed = IfaceNature {
            arphrd: 1, // ARPHRD_ETHER
            mac80211_phy: Some("phy0".into()),
            phy_driver: Some("morse_spi".into()),
        };
        let e = check_morse_vif_roles("mon0", "wlan0", &good_tx(), &managed).expect_err("rx");
        let m = format!("{e}");
        assert!(m.contains("radiotap"), "name what is missing: {m:?}");
        assert!(m.contains("803"), "name the type it must be: {m:?}");

        let e = check_morse_vif_roles("wlan0", "morse0", &managed, &good_rx()).expect_err("tx");
        let m = format!("{e}");
        assert!(m.contains("cannot inject"), "say what breaks: {m:?}");
        assert!(m.contains("type monitor"), "name the fix: {m:?}");
    }

    /// The NRC7292 has one netdev and no roles to confuse, but the managed-vif trap is identical
    /// and just as silent: `nrc_mac_rx` only takes the monitor path when `nw->promisc`.
    #[test]
    fn the_nrc_face_refuses_a_managed_interface() {
        check_nrc_iface(
            "halow0",
            &IfaceNature {
                arphrd: ARPHRD_IEEE80211_RADIOTAP,
                mac80211_phy: Some("phy0".into()),
                phy_driver: Some("nrc80211".into()),
            },
        )
        .expect("a monitor vif is the working case");

        let e = check_nrc_iface(
            "halow0",
            &IfaceNature {
                arphrd: 1,
                mac80211_phy: Some("phy0".into()),
                phy_driver: Some("nrc80211".into()),
            },
        )
        .expect_err("managed must be refused");
        let m = format!("{e}");
        assert!(
            m.contains("not in monitor mode"),
            "say what is wrong: {m:?}"
        );
        assert!(m.contains("set type monitor"), "name the fix: {m:?}");
    }

    /// The two radios must agree on what a radiotap netdev *is*, or a node running both would
    /// judge the same sysfs value two ways. One constant, one meaning.
    #[test]
    fn both_radios_read_the_same_arphrd_constant() {
        assert_eq!(ARPHRD_IEEE80211_RADIOTAP, 803, "ARPHRD_IEEE80211_RADIOTAP");
        let ether = IfaceNature {
            arphrd: 1,
            mac80211_phy: None,
            phy_driver: None,
        };
        assert!(check_nrc_iface("halow0", &ether).is_err());
        assert!(check_morse_vif_roles("mon0", "morse0", &good_tx(), &ether).is_err());
    }

    /// The one correct pairing, and the reason the RX name is not constrained: a renamed sniffer
    /// netdev is still a legitimate deployment.
    #[test]
    fn the_split_pairing_is_accepted() {
        assert!(check_morse_ifaces("mon0", "morse0").is_ok());
        assert!(check_morse_ifaces("mon0", "halowmon").is_ok());
    }

    /// ★ The rate-ladder correction, asserted rather than commented: the shared `wifi_halow_s1g`
    /// preset declares `max_mcs: 10`, but S1G MCS10 is 1 MHz-only repetition-coded BPSK at *half*
    /// MCS0, so the index is not a rate ceiling. The consumers that read it unclamped are the
    /// contextual bandit's `clamp(0, max_mcs)` and `rate_rank`; see [`halow_base`] for why
    /// `mcs_for_rssi` — the function this note used to blame — is not one of them.
    /// The shared width reading, pinned — including the two cases that are conventions rather
    /// than facts, so a future change to either is a deliberate one.
    #[test]
    fn the_width_request_reading_is_the_one_both_radios_share() {
        // `Default` / `from_code(0)`: no preference, must be accepted or every tune is refused.
        assert_eq!(Bandwidth::default(), Bandwidth::Bw20);
        assert_eq!(s1g_width_request(Bandwidth::Bw20).unwrap(), None);
        // The convention: the two narrowband members carry the two narrow S1G widths.
        assert_eq!(s1g_width_request(Bandwidth::Nb5).unwrap(), Some(1));
        assert_eq!(s1g_width_request(Bandwidth::Nb10).unwrap(), Some(2));
        // No S1G channel is 40 or 80 MHz wide: refused, never served at some other width.
        for bw in [Bandwidth::Bw40, Bandwidth::Bw80] {
            assert!(
                s1g_width_request(bw).is_err(),
                "{bw:?} must be refused, not silently served at the channel's width"
            );
        }
    }

    /// ★ **The two S1G channel tables in this crate must not drift apart.**
    ///
    /// The same physical band is described twice, because each backend needs a different key:
    /// [`crate::nrc7292::US_S1G_CHANNELS`] is a 45-row const table keyed on the driver's *alias*
    /// (2.4/5 GHz shadow) number, since that is what `iw` speaks on the NRC7292; and
    /// [`crate::morse::us_s1g_channel`] is a closed-form rule keyed on the *real* S1G channel
    /// number, which is what `morse_cli` speaks. Neither can be deleted in favour of the other —
    /// the alias join genuinely carries information the formula cannot (`nrc-bd.c`'s shadow map),
    /// and the formula covers 8 MHz rows the NRC7292's US table does not have.
    ///
    /// What they *must* agree on is the physics: an S1G channel number has one centre frequency
    /// and one width, and two radios told "channel 8" must land on the same air. This test is the
    /// join. It is what makes the duplication safe rather than merely tolerated — a lesson already
    /// paid for once here, when `set_channel` computed `902_500 + ch × 1000` and put channel 8 at
    /// 910.5 MHz instead of 906.0.
    #[test]
    fn both_s1g_channel_tables_describe_the_same_air() {
        let mut checked = 0;
        for row in crate::nrc7292::US_S1G_CHANNELS {
            let (khz, width) = crate::morse::us_s1g_channel(row.s1g_channel).unwrap_or_else(|| {
                panic!(
                    "S1G ch {} is in the NRC7292 table but the Morse rule rejects it",
                    row.s1g_channel
                )
            });
            assert_eq!(
                khz,
                u32::from(row.s1g_freq_100khz) * 100,
                "S1G ch {} centre frequency disagrees",
                row.s1g_channel
            );
            assert_eq!(
                width, row.bw_mhz,
                "S1G ch {} width disagrees",
                row.s1g_channel
            );
            checked += 1;
        }
        assert_eq!(checked, 45, "the NRC7292 table lost rows");
    }

    #[test]
    fn the_declared_mcs_ceiling_excludes_the_off_ladder_reach_rate() {
        for cap in [nrc7292_capability(vec![161]), mm6108_capability(vec![36])] {
            match cap.rate {
                RateCapability::Wifi {
                    max_mcs, max_nss, ..
                } => {
                    assert_eq!(max_mcs, 7, "MCS10 is below MCS0, not above MCS7");
                    assert_eq!(max_nss, 1, "both parts are single-chain");
                }
                other => panic!("HaLow must declare a Wi-Fi rate model, got {other:?}"),
            }
            assert_eq!(cap.kind, RadioKind::WifiHaLow);
            assert_eq!(cap.bands, vec![Band::Sub1GHz]);
            assert!(cap.half_duplex);
            assert_eq!(cap.csi, CsiSupport::None);
            assert!(cap.phy_modes.is_empty(), "PhyMode cannot name S1G");
            assert!(
                cap.hop.is_none(),
                "no frequency-hop sequencer on either part"
            );
            assert!(
                cap.tx_power_dbm.is_none(),
                "a dBm range must be attached from a measurement, never asserted by a preset"
            );
        }
    }

    /// The MM6108's payload cap is the MEASURED one, not an 802.11n inheritance — 1546, where the
    /// generic monitor MTU is 2272 and the default A-MSDU budget 3839. Both of those overshoot into
    /// the region where the chip discards silently.
    #[test]
    fn the_morse_payload_cap_is_the_measured_one() {
        assert_eq!(mm6108_capability(vec![]).max_payload, 1546);
        assert!(MM6108_MAX_PAYLOAD < ndn_frame_io::MONITOR_MTU);
        assert!(MM6108_AMSDU_BODY < ndn_frame_io::DEFAULT_AMSDU_BODY);
        assert_eq!(
            MM6108_MAX_MPDU - MM6108_MAX_PAYLOAD,
            38,
            "802.11 24 + LLC 8 + FCS 4 + 2"
        );
    }

    /// `power_actuated` must not claim a knob that the driver drops in monitor mode.
    #[test]
    fn morse_does_not_claim_power_actuation_on_a_monitor_vif() {
        assert!(!mm6108_capability(vec![]).power_actuated);
    }

    /// The metadata helper reaches the fields `CapturedFrame` has no room for. Bytes are the
    /// NRC7292's `struct nrc_radiotap_hdr` shape (34 B, S1G TLV at offset 24).
    #[test]
    fn s1g_metadata_surfaces_the_received_width() {
        let mut w = vec![0u8, 0];
        w.extend_from_slice(&34u16.to_le_bytes());
        w.extend_from_slice(&(1u32 | (1 << 1) | (1 << 3) | (1 << 28)).to_le_bytes());
        w.extend_from_slice(&99u64.to_le_bytes());
        w.push(0x10);
        w.push(0);
        w.extend_from_slice(&925u16.to_le_bytes());
        w.extend_from_slice(&0x0140u16.to_le_bytes());
        w.extend_from_slice(&[0, 0]);
        w.extend_from_slice(&32u16.to_le_bytes());
        w.extend_from_slice(&6u16.to_le_bytes());
        w.extend_from_slice(&0x007fu16.to_le_bytes());
        w.extend_from_slice(&(1u16 | (2 << 8) | (3 << 12)).to_le_bytes()); // bw 4 MHz, MCS 3
        w.extend_from_slice(&0u16.to_le_bytes());
        let (s1g, freq) = s1g_metadata(&w).expect("S1G TLV present");
        assert_eq!(s1g.bandwidth_mhz, Some(4));
        assert_eq!(s1g.mcs, Some(3));
        assert_eq!(freq, None, "Newracom emits no vendor frequency TLV");
        // And the rate that width + MCS actually implies — not the 11n table's answer.
        // 4 MHz MCS3 long GI = 108 data subcarriers x 4 bits x 1/2 / 40 us = 5.4 Mbit/s, which is
        // 802.11ac's 40 MHz MCS3 (54 Mbit/s) downclocked by 10.
        assert_eq!(
            ndn_radio_hal::s1g_phy_rate_bps(3, 4, false),
            Some(5_400_000)
        );
        assert_eq!(
            ndn_frame_io::mcs_phy_rate_bps(3),
            26_000_000,
            "the 11n table would overstate this same frame's rate by ~4.8x"
        );
    }

    // ── Mesh common view from S1G beacons ─────────────────────────────────────────────────────

    /// Wrap a bare 802.11 frame in the exact 34-byte `struct nrc_radiotap_hdr` this radio emits
    /// (TSFT + FLAGS + CHANNEL + S1G TLV, `F_FCS` set) and append the 4-byte FCS the driver adds
    /// with `skb_put(skb, 4)` — so the harvester is exercised through the same strip the data plane
    /// uses.
    fn nrc_wrap(body: &[u8], tsft: u64) -> Vec<u8> {
        let mut w = vec![0u8, 0];
        w.extend_from_slice(&34u16.to_le_bytes());
        w.extend_from_slice(&(1u32 | (1 << 1) | (1 << 3) | (1 << 28)).to_le_bytes());
        w.extend_from_slice(&tsft.to_le_bytes());
        w.push(0x10); // rt_flags = IEEE80211_RADIOTAP_F_FCS
        w.push(0);
        w.extend_from_slice(&925u16.to_le_bytes());
        w.extend_from_slice(&0x0140u16.to_le_bytes());
        w.extend_from_slice(&[0, 0]);
        w.extend_from_slice(&32u16.to_le_bytes());
        w.extend_from_slice(&6u16.to_le_bytes());
        w.extend_from_slice(&0x007fu16.to_le_bytes());
        w.extend_from_slice(&0u16.to_le_bytes());
        w.extend_from_slice(&0u16.to_le_bytes());
        assert_eq!(w.len(), 34);
        w.extend_from_slice(body);
        w.extend_from_slice(&[0xde, 0xad, 0xbe, 0xef]); // FCS
        w
    }

    /// Two consecutive S1G beacons captured off-air from mds-o5p-3, with the radiotap TSFT
    /// mds-o5p-0 stamped them with — the same bytes `nrc7292`'s own test uses, here driven through
    /// the whole radiotap + FCS path rather than handed straight to the parser.
    const BEACON_A: [u8; 14] = [
        0x1c, 0x0b, 0x00, 0x00, 0x00, 0xc0, 0xca, 0xb4, 0x65, 0xe2, 0xac, 0xf0, 0x67, 0x83,
    ];
    const TSFT_A: u64 = 1_308_937_865;

    #[test]
    fn a_real_beacon_becomes_a_common_view_observation() {
        // These beacons come from an NRC7292 *AP* (00:c0:ca is Newracom — universally
        // administered), so the mesh filter has to be off to see them; that is the point of the
        // knob, and of the warning on `MeshCvHarvester`.
        let mut h = MeshCvHarvester::new(false);
        assert!(h.observe(&nrc_wrap(&BEACON_A, TSFT_A)));
        let cv = h.latest().expect("recorded");
        assert_eq!(cv.bssid, [0x00, 0xc0, 0xca, 0xb4, 0x65, 0xe2]);
        assert_eq!(cv.peer_tsf, 0x8367_f0ac);
        assert_eq!(cv.our_rxtsfl, TSFT_A);
        assert_eq!(cv.count, 1);
        assert!(
            cv.belief.is_none(),
            "a bare beacon advertises no network-time belief"
        );
    }

    /// The default honours the trait's contract: infrastructure beacons are not mesh peers.
    #[test]
    fn the_mesh_filter_rejects_a_universally_administered_transmitter() {
        let mut h = MeshCvHarvester::new(true);
        assert!(!h.observe(&nrc_wrap(&BEACON_A, TSFT_A)));
        assert!(h.latest().is_none());

        // Flip the universal/local bit and the same beacon is one of ours.
        let mut mesh = BEACON_A;
        mesh[4] |= 0x02;
        assert!(h.observe(&nrc_wrap(&mesh, TSFT_A)));
        assert_eq!(h.latest().unwrap().bssid[0], 0x02);
    }

    /// ★ The 170× error, guarded. The S1G Beacon's `fc1` selects which optional fields follow the
    /// partial TSF, so two variants from one transmitter are two different field layouts. Mixing
    /// them measured sd = 1004 µs where splitting by `fc1` gave 5.4 / 5.8 µs.
    #[test]
    fn a_second_frame_control_variant_from_the_same_transmitter_is_dropped() {
        let mut h = MeshCvHarvester::new(false);
        assert!(h.observe(&nrc_wrap(&BEACON_A, TSFT_A)));

        let mut other_variant = BEACON_A;
        other_variant[1] = 0x08; // fc1 = 0x08 instead of 0x0b
        other_variant[10] = 0xff; // a visibly different TSF, so a leak would be obvious
        assert!(
            !h.observe(&nrc_wrap(&other_variant, TSFT_A + 1000)),
            "fc1 0x08 must not join a series pinned to 0x0b"
        );
        assert_eq!(
            h.latest().unwrap().peer_tsf,
            0x8367_f0ac,
            "series unchanged"
        );
        assert_eq!(h.latest().unwrap().count, 1);

        // A *different* transmitter pins its own variant independently.
        let mut other_sa = other_variant;
        other_sa[9] = 0xe3;
        assert!(h.observe(&nrc_wrap(&other_sa, TSFT_A + 2000)));
        assert_eq!(h.latest().unwrap().count, 2);
    }

    /// ★ Both halves of the pair are reported in the beacon's own 32-bit microsecond modulus,
    /// because that is all the beacon carries. Widening one without the other turns a few-µs offset
    /// into a ~4.3e9 µs one that still looks like a number.
    #[test]
    fn the_local_stamp_is_truncated_into_the_beacons_modulus() {
        let mut h = MeshCvHarvester::new(false);
        let big: u64 = 0x0000_0007_1234_5678; // well past 2^32
        assert!(h.observe(&nrc_wrap(&BEACON_A, big)));
        let cv = h.latest().unwrap();
        assert_eq!(cv.our_rxtsfl, 0x1234_5678);
        assert!(cv.our_rxtsfl <= u64::from(u32::MAX));
        assert!(cv.peer_tsf <= u64::from(u32::MAX));
    }

    /// Everything that is not an acceptable beacon is silently not an observation — a data frame,
    /// a frame the PHY flagged as corrupt, a header with no TSFT to compare against, and garbage.
    #[test]
    fn non_beacons_are_never_observations() {
        let mut h = MeshCvHarvester::new(false);
        // An ordinary data frame (FC 0x08).
        let data = [0x08u8, 0x00, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        assert!(!h.observe(&nrc_wrap(&data, TSFT_A)));
        // A beacon the PHY says failed its FCS.
        let mut bad = nrc_wrap(&BEACON_A, TSFT_A);
        bad[16] |= 0x40; // rt_flags |= F_BADFCS
        assert!(!h.observe(&bad));
        // A header with no TSFT: nothing to pair the peer's clock with.
        let mut no_tsft = vec![0u8, 0];
        no_tsft.extend_from_slice(&9u16.to_le_bytes());
        no_tsft.extend_from_slice(&(1u32 << 1).to_le_bytes());
        no_tsft.push(0);
        no_tsft.extend_from_slice(&BEACON_A);
        assert!(!h.observe(&no_tsft));
        // Not radiotap at all.
        assert!(!h.observe(&[]));
        assert!(!h.observe(&[1, 0, 8, 0, 0, 0, 0, 0]));
        assert!(h.latest().is_none());
    }
}
