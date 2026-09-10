//! Userspace USB Wi-Fi monitor-mode driver backends over the `ndn-radio-hal` contract.
//!
//! Split out of `ndn-face-monitor-wifi` so drivers have a dedicated home; each
//! backend implements `FrameIo` against the HAL and does no NDN forwarding.

// Re-export the contract surface the backend modules reference as `crate::…`
// (they were written as modules of ndn-face-monitor-wifi, which re-exported these).
pub use ndn_frame_io::{
    BROADCAST, CapturedFrame, DEFAULT_SRC, FaceError, FaceId, FrameFormat, FrameIo, InjectFrame,
    MAX_RELIABLE_MCS, McsDescriptor, McsPolicy, Reach, Reliability, TxIntent, frame, mcs_for_rssi,
    mcs_phy_rate_bps, radiotap,
};
// #78: `OpenRadio` and the capability traits it hands out live in the HAL, beside the traits they
// aggregate — a driver crate builds one and a face crate consumes one, so neither should need a
// dependency on the other to name it. Re-exported here so a caller of `open_named_radio` needs
// exactly one import to use everything the opener returns.
pub use ndn_radio_hal::{OpenRadio, RadioKnobs, RadioProfile, RadioTime};
// ★ M8: the bring-up contract's own vocabulary, re-exported.
//
// `bring_up_planned` — the one entry point on every part — takes a `Role`, a `PowerRequest`, a
// `Deviation` and a `ProofRequirement`, and `open_radio` takes a `BringUpRequest` built from them.
// A caller that cannot NAME those types cannot use the API, so a crate depending on the drivers
// had to add a second dependency on the HAL to say `Role::ReceiveOnly`. The types still LIVE in
// the HAL (a face crate consumes a report without knowing this crate exists); this is a
// re-export, not a second home.
pub use ndn_radio_hal::bringup;
pub use ndn_radio_hal::bringup::{
    AppliedPower, BringUpFailure, BringUpReport, Deviation, Guards, PowerReference, PowerRequest,
    ProofRequirement, PumpPolicy, RateGroupPolicy, RfAuthority, Role, WitnessId, WitnessOracle,
};

// ─────────────────────────────────────────────────────────────────────────────
// §5-M8 — the ladder rungs are not public API
// ─────────────────────────────────────────────────────────────────────────────

/// **Declare one bring-up rung.** `pub` under `feature = "bench"`, `pub(crate)` without it.
///
/// ★ The contract's §5-M8: *"sub-steps drop `pub` → `pub(crate)`, re-exported only under
/// `feature = "bench"`."* The sequence belongs to the part's [`Plan`](ndn_radio_hal::Plan); a
/// consumer composes a bring-up by naming a [`Role`], not by calling rungs in an order it
/// remembers. Sixteen private ladders assembled from these very methods is how a silent power-regime
/// regime stayed invisible for months — see `docs/bringup-root-cause-2026-09-03.md`.
///
/// After this, `tests/no_hand_rolled_ladder.rs` becomes a **belt over a compiler brace**: a
/// production crate that tries to compose its own ladder does not build, and the source-level test
/// remains only to catch a rung re-exported under `bench` being used outside a bench example.
///
/// ⚠ It does not hide anything from an operator. `bench` is enabled for this crate's own
/// `examples/` and `tests/` (by a dev-dependency on ourselves) and for `ndn-phy-wifi`'s
/// `dev-examples`, because a bench instrument that pokes exactly one rung is a legitimate and
/// necessary thing. What it stops is a *production* path composing a bring-up.
#[cfg(feature = "bench")]
macro_rules! rung {
    ($(#[$m:meta])* fn $($rest:tt)*) => {
        $(#[$m])*
        pub fn $($rest)*
    };
}
#[cfg(not(feature = "bench"))]
macro_rules! rung {
    ($(#[$m:meta])* fn $($rest:tt)*) => {
        $(#[$m])*
        pub(crate) fn $($rest)*
    };
}
// ⚠ NO `pub(crate) use rung;` — the macro reaches the backend modules by macro_rules' TEXTUAL
// scoping, which is why it is declared HERE, immediately above the `mod` declarations. Moving it
// below them silently puts every rung back to `pub`, so keep it first.

pub mod mt76;
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
/// **The caller-boundary reader for the a81a's `NDN_RADIO_MINIMAL` / `NDN_RADIO_SKIP_CAL` /
/// `NDN_RADIO_NO_EFEM`**, assembled into a self-labelling `Deviation` by the module that owns the
/// rung ids it names. `BringUpRequest::from_env` calls it; a bench instrument that drives
/// `bring_up_planned` directly should too, or those three knobs silently stop working for it.
pub use libusb_rtl88xx::a81a_env_deviation;
pub use libusb_rtl88xx::{
    CHIP_ID_8822E, ChannelBw, FwVersion, LibUsbRtl88xxBackend, PLAN_A81A, REALTEK_VID, REG_SYS_CFG,
    RTL88XX_PIDS, RfPath,
};
// AR9271 (ath9k_htc) — the one Wi-Fi part whose FIRMWARE is ours, so Tier-0 can reject a frame
// before it crosses USB (design §8.2) and TX can be scheduled off the hardware TSF (§8.5).
// L1: USB transport + firmware download + HTC handshake + WMI. Does not yet replace ath9k_htc.
mod ath9k_htc;
/// **The bring-up coverage table** (contract §6.3): one cell per part × [`Role`], `Provided` or
/// `Excluded` in writing. The sibling of [`coverage`] one layer down — that table makes a missing
/// trait impl a visible row, this one makes a missing *sequence* a visible row.
pub mod bringup_coverage;
pub mod coverage;
// PHY-init data for the M1 bring-up port, transcribed verbatim from mainline ath9k v6.12.33:
// AR9271 initval tables (ar9002_initvals.h), the register offsets/bits the reset+cal path writes
// (reg.h / ar9002_phy.h / mac.h), and the HTC wire structs (htc.h). Consumed by ath9k_htc.rs.
mod ath9k_htc_structs;
mod ath9k_initvals;
mod ath9k_reg;
pub use ath9k_htc::{
    AR9271_FIRMWARE, AR9271_FIRMWARE_TEXT, AR9271_IDS, AR9271_PID, ATHEROS_VID, Ath9kBringUpOpts,
    Ath9kCalPolicy, Ath9kHtcBackend, BoardValues, CalStatus, FW_NAME, GainTableChoice,
    HTC_RX_STATUS_LEN, HtcService, IEEE80211_MODE_11NG, IniVerify, LegacyRate, NDR_MEM_MAX_TUPLES,
    PLAN_AR9271_MONITOR, PLAN_AR9271_RX, REG_WRITE_MAX_PAIRS,
    ResetStatus, RxFrame, WmiCmd, ath9k_channel_to_mhz, ath9k_mhz_to_channel,
};
mod rtl8821c;
pub use rtl8821c::{
    PLAN_8821CU_FW_STA, PLAN_8821CU_IBSS, PLAN_8821CU_MONITOR, PLAN_8821CU_NO_TXEN,
    PLAN_8821CU_STATION_REGS, RTL8821CU_PIDS, Rtl8821cVariant, Rtl8821cuBackend,
};
mod mt7612;
pub use mt7612::{MT7612U_PIDS, Mt7612uBackend, PLAN_MT7612U};
// MT7610U (mt76x0u, 1x1 dual-band 802.11ac) — the sibling port. Shares `mt76x02_regs.h` with the
// MT7612U above, so the two share `crate::mt76`'s register map, transport and knob layer; what
// differs is the firmware (no ROM patch), the RF programming model (host-programmable via
// MT_RF_CSR_CFG, MEASURED working over USB) and the 1x1 chain configuration.
mod mt76x0;
pub use mt76x0::{MT7610U_PIDS, Mt7610uBackend, PLAN_MT7610U};
/// Shared MediaTek **connac2** layer (MT7921/MT792x): a different architecture from `mt76`, not a
/// newer revision of one — extended vendor requests, a composite BT+WLAN device, patch+RAM
/// firmware, and a variable-length RX descriptor whose group 2 carries a per-frame hardware
/// timestamp.
pub mod connac2;
// MT7921AU (connac2, 2x2 802.11ax) — the only Wi-Fi part in this crate that can actuate the HAL's
// HE levers, and the only MediaTek one that can source common view.
mod mt7921;
pub use mt7921::{MT7921U_PIDS, Mt7921uBackend, PLAN_MT7921AU};
mod rtl8812au;
pub use rtl8812au::{
    ChipInfo, IqkResult, PLAN_8812AU_MONITOR, PhySense, RTL8812AU_PID, RTL8812AU_PIDS,
    Rtl8812auBackend,
};
// RTL8731BU / RTL8733BU (halmac_87xx, 1x1 802.11n, dual-band, 20/40) — ground-up port, complete
// against the HAL: power-on,
// firmware download, MAC/BB/RF init, 1x1 calibration (IQK/TXGAPK/DPK), channel/power, RX capture and
// on-air TX, impl'ing FrameIo + RadioKnobs + RadioTime + RadioProfile. It is the *reference* RadioTime
// implementation (both link clocks: free-run RX stamp + port TSF). Open it through `open_named_radio`.
// REALTEK_VID is already re-exported above.
mod libusb_rtl8733b;
/// **The caller-boundary reader for the 8733b's `NDN_8733B_NO_TSSI`.** See
/// [`a81a_env_deviation`].
pub use libusb_rtl8733b::rtl8733b_env_deviation;
pub use libusb_rtl8733b::{
    ChipVersion, FW_NIC_8733B, FwHeader, PLAN_8733B_MONITOR, PLAN_8733B_TX, PowerTracker,
    RTL8733B_PIDS, Rtl8733buBackend,
};

// Serial-bridged 802.11 backend — a raw injector/capturer driven over USB-serial (the ND wire protocol),
// backing a BW16 (RTL8720DN, firmware/bw16-ndn-bridge) or an ESP32-C5 (firmware/esp32c5-ndn), implementing
// the same FrameIo/RadioKnobs contract as the USB drivers.
#[cfg(feature = "serial-radio")]
mod serial_radio;
#[cfg(feature = "serial-radio")]
pub use serial_radio::{
    Bw16SerialBackend, ChannelProfile, Esp32SerialBackend, PLAN_SERIAL_BRIDGE, SERIAL_RADIO_BAUD,
    SerialRadioBackend, bw16_clock_domain,
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
    NdnStats, NodeProfile, PLAN_LORA_NODE, PROTO_VER, RadioKindHint, StampKind, lora_clock_domain,
    name_hash,
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

// The named airtime lease's bearer-side geometry: name -> owned slot -> absolute target instant,
// sized to the MEASURED placement floor of this bearer (p99.9 = 896 us => 1 ms guard, 2 ms minimum
// slot) and refusing any geometry that floor cannot hold. No association, no AP, no host identity,
// no AID: the grant is computed from the name and a shared clock by everyone who holds the name.
// It hashes with `ndn_frame_io::prefix_hash` — the control plane's canonical name key, moved down to
// that crate so driver and decider share ONE implementation. See src/lease.rs.
pub mod lease;

/// **§1.1 + §1.7 of the bring-up contract: `BringUpRequest` and `open_radio` — the one door.**
/// M8. See `src/open_radio.rs`; in particular its list of the behaviour changes a deployed node
/// will see, which the contract's §5-M8 asked to be said out loud rather than discovered.
pub mod open_radio;
pub use lease::{GridError, LeaseGrid, MIN_GUARD_US, MIN_SLOT_US};
pub use open_radio::{BringUpRequest, PartOpts, open_radio};

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

/// **The `tracing::warn!` the bring-up contract requires (§2.5.3)** whenever the resolved power
/// reference is the raw/chip-max axis, plus the INFO block every open prints.
///
/// The two lines are the whole point of M2: an operator reading a run's own output can now tell a
/// calibrated transmitter from one running on the raw axis, which on 2026-09-03 took a day of
/// bisection and a witness receiver.
pub(crate) fn emit_bringup(report: &ndn_radio_hal::BringUpReport) {
    if report.state.power.reference.is_off_scale() {
        tracing::warn!(
            target: "named_radio",
            part = report.part,
            device = %report.device,
            plan_digest = format_args!("{:#018x}", report.plan_digest),
            power = %report.state.power.render(),
            "RF OFF THE REGULATORY SCALE — this radio is transmitting on the raw chip TXAGC axis \
             (calibration bypassed); it may exceed licensed EIRP, and no measurement taken here is \
             comparable with one taken on the calibrated scale"
        );
    }
    // Also `emit()` — the HAL's own off-scale stderr line. It duplicates the warn above ONLY in
    // the dangerous case, and deliberately: a bench run started without a `tracing` subscriber
    // would otherwise see nothing at all about a transmitter that is off the regulatory
    // scale.
    report.emit();
    tracing::info!(
        target: "named_radio",
        part = report.part,
        plan = %report.plan,
        plan_digest = format_args!("{:#018x}", report.plan_digest),
        power_reference = report.state.power.reference.tag(),
        channel = report.state.channel,
        "\n{}",
        report.render()
    );
}

// The control-plane `RadioKnobs` impls for the driver backends. These live with
// the driver types (the trait is from `ndn-radio-hal`, the types are declared
// here) — the orphan rule requires the impl travel with the local type. The
// data-plane `FrameIo` impls live in each backend module.
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
        fn set_tx_power(
            &self,
            req: ndn_radio_hal::PowerRequest,
        ) -> Result<ndn_radio_hal::AppliedPower, FaceError> {
            crate::LibUsbRtl88xxBackend::set_tx_power(self, req)
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
        /// Per-rate TXAGC index — the devourer jaguar1 power knob, validated monotone on air
        /// (#38). This is the actuator behind the cognition policy's reciprocity `decide_power`
        /// backoff, and the one whose two meanings the bring-up contract exists to separate: see
        /// [`Rtl8812auBackend::set_tx_power`](crate::Rtl8812auBackend::set_tx_power).
        fn set_tx_power(
            &self,
            req: ndn_radio_hal::PowerRequest,
        ) -> Result<ndn_radio_hal::AppliedPower, FaceError> {
            crate::Rtl8812auBackend::set_tx_power(self, req)
        }
        /// ★ **Hold or release transmissions at the MAC** — the hardware half of a slot MAC.
        ///
        /// This was previously the HAL default, which returns **`Ok(())`**. So
        /// `FaceScheduler`'s `TxHoldGuard` was told its airtime lease was enforced while nothing
        /// was ever written: frames already queued in the MAC went out inside somebody else's
        /// slot, charged to a name that did not cause them. That default's own doc says it — "the
        /// default must never be 'pretend it worked'".
        ///
        /// See `Rtl8812auBackend::set_tx_pause` for the MEASURED hold-not-drop semantics a
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
        /// `floor_dBm = IGI - 110`. See `Rtl8812auBackend::set_rx_floor_dbm`, which cognition
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
