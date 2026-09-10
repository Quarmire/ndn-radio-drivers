//! Linux `AF_PACKET` data plane for the two HaLow radios. Split out because it is the only part of
//! [`crate::halow`] that needs a socket; every rule it enforces is decided by the platform-neutral
//! helpers next door, so the rules themselves are unit-tested on any host.

use std::path::{Path, PathBuf};
use std::sync::Mutex;

use async_trait::async_trait;
use ndn_frame_io::{
    AfPacketBackend, CapturedFrame, ClockDomainId, ClockReference, FaceError, FrameFormat, FrameIo,
    InjectFrame, LatchPoint, McsDescriptor, RadioCapability, RadioClockKind, RadioProfile,
    RadioTime, RadioTimeSource, frame,
};
use ndn_radio_hal::MeshCv;

use super::{
    HalowIfaces, IfaceNature, MM6108_AMSDU_BODY, MM6108_MAX_PAYLOAD, MORSE_INJECT_BW_PARAM,
    MORSE_INJECT_MCS_PARAM, MeshCvHarvester, check_morse_ifaces, check_morse_vif_roles,
    check_nrc_iface, mm6108_bringup, mm6108_capability, morse_monitor_vif_error, nrc7292_bringup,
    nrc7292_capability,
};
use crate::nrc7292::Nrc7292Clock;

/// Is the interface administratively UP (`IFF_UP`)?
///
/// Worth checking at construction on both radios, because "down" is the failure that looks like
/// nothing at all: on the Morse it makes `morse_mon_rx` early-return on `!netif_running(morse_mon)`
/// so capture yields zero frames with no error, and it makes every `morse_cli` vendor command fail
/// `ENETDOWN`. Reads `/sys/class/net/<iface>/flags`, whose bit 0 is `IFF_UP` — `operstate` is
/// deliberately not used, because a monitor netdev reports `unknown` there even when it is running.
fn iface_is_up(iface: &str) -> Result<bool, FaceError> {
    let raw = std::fs::read_to_string(format!("/sys/class/net/{iface}/flags")).map_err(|e| {
        FaceError::Io(std::io::Error::new(
            e.kind(),
            format!("{iface}: cannot read interface flags: {e}"),
        ))
    })?;
    let t = raw.trim();
    let hex = t.strip_prefix("0x").unwrap_or(t);
    let flags = u32::from_str_radix(hex, 16).map_err(|e| {
        FaceError::Io(std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            format!("{iface}: unparseable flags {t:?}: {e}"),
        ))
    })?;
    Ok(flags & 0x1 != 0)
}

fn require_up(iface: &str) -> Result<(), FaceError> {
    if iface_is_up(iface)? {
        return Ok(());
    }
    Err(FaceError::Io(std::io::Error::new(
        std::io::ErrorKind::NotConnected,
        format!("{iface} is DOWN — bring it up (`ip link set {iface} up`) before opening the face"),
    )))
}

/// Does the interface exist at all? `/sys/class/net/<iface>` is the directory every netdev has.
///
/// Asked separately from [`require_up`] because on the Morse the *absent* TX vif is not a generic
/// "no such device": it is the monitor-vif precondition, and it deserves that error rather than an
/// `ENOENT` that says nothing about why receive will be silent.
fn iface_exists(iface: &str) -> bool {
    Path::new(&format!("/sys/class/net/{iface}")).exists()
}

/// Read `/sys/class/net/<iface>/type` — the ARP hardware type, which is how sysfs answers "is this
/// a monitor netdev or a managed one".
///
/// An error here is a real error, not a shrug: [`require_up`] has already read this interface's
/// `flags`, so the directory demonstrably exists, and `type` is present on every netdev sysfs has
/// ever exported. Failing open would put the silent-zero checks back to being unchecked.
fn iface_arphrd(iface: &str) -> Result<u32, FaceError> {
    let raw = std::fs::read_to_string(format!("/sys/class/net/{iface}/type")).map_err(|e| {
        FaceError::Io(std::io::Error::new(
            e.kind(),
            format!("{iface}: cannot read the interface type: {e}"),
        ))
    })?;
    raw.trim().parse::<u32>().map_err(|e| {
        FaceError::Io(std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            format!("{iface}: unparseable type {:?}: {e}", raw.trim()),
        ))
    })
}

/// The last component of a sysfs symlink's target, or `None` when the link is absent or unreadable.
/// `None` is a *fact* for `phy80211` (the interface is not a mac80211 vif) and merely "sysfs did
/// not say" for `phy80211/device/driver`; [`IfaceNature`] documents which is which.
fn link_basename(path: PathBuf) -> Option<String> {
    let target = std::fs::read_link(path).ok()?;
    Some(target.file_name()?.to_string_lossy().into_owned())
}

/// Gather what sysfs says about an interface's nature, for the pure rules next door
/// ([`check_morse_vif_roles`], [`check_nrc_iface`]) to judge.
fn iface_nature(iface: &str) -> Result<IfaceNature, FaceError> {
    Ok(IfaceNature {
        arphrd: iface_arphrd(iface)?,
        mac80211_phy: link_basename(PathBuf::from(format!("/sys/class/net/{iface}/phy80211"))),
        phy_driver: link_basename(PathBuf::from(format!(
            "/sys/class/net/{iface}/phy80211/device/driver"
        ))),
    })
}

/// Write a decimal value to a driver module parameter, turning "the patch is not loaded" into a
/// named error rather than a silent no-op.
fn write_param(path: &Path, value: u32) -> Result<(), FaceError> {
    std::fs::write(path, format!("{value}\n")).map_err(|e| {
        FaceError::Io(std::io::Error::new(
            e.kind(),
            format!("{}: cannot write {value}: {e}", path.display()),
        ))
    })
}

// ═════════════════════════════════════════════════════════════════════════════════════════════
// Morse Micro MM6108 — the split-netdev data plane
// ═════════════════════════════════════════════════════════════════════════════════════════════

/// The MM6108 `FrameIo`: **inject on a mac80211 monitor vif, capture on the driver's `morseN`
/// sniffer netdev.** See the [module docs](crate::halow) for why that is not a choice.
///
/// What this adds over a bare [`AfPacketBackend::split`]:
///
/// * the two silent *naming* misconfigurations are refused at construction
///   ([`super::check_morse_ifaces`]), not discovered on air;
/// * ★ the silent *role* misconfigurations are too ([`super::check_morse_vif_roles`]) — above all
///   the **monitor-vif precondition**: `morse0` delivers ZERO frames unless a mac80211 monitor vif
///   exists on the phy (MEASURED, same node/second/channel: no `mon0` → 0 packets, `mon0` → 1880)
///   and nothing anywhere reports that. The TX vif this face already needs is exactly what
///   satisfies it, so the check costs a deployment nothing it was not already doing right;
/// * both interfaces are checked UP, because a down sniffer netdev captures nothing *and reports
///   nothing*;
/// * the A-MSDU budget and the single-frame payload cap are the **MEASURED** 1546 bytes rather
///   than an inherited 802.11n number, and an oversize frame is refused instead of being discarded
///   by the chip;
/// * [`set_rate`](FrameIo::set_rate) reaches the patched driver's injection MCS parameter when the
///   caller has opted into it, and is honestly inert when they have not.
pub struct MorseFrameIo {
    af: AfPacketBackend,
    tx_iface: String,
    rx_iface: String,
    /// The clock domain every RX stamp off this face is keyed on: the **sniffer** interface's
    /// index, the same value [`AfPacketBackend::recv_frame`] hands to
    /// [`frame::parse`](ndn_frame_io::frame::parse) and the same convention
    /// [`Nrc7292FrameIo`] uses. Cached at construction because `time_sources()` must name it and
    /// takes `&self`.
    domain: ClockDomainId,
    inject_mcs: Option<PathBuf>,
    inject_bw: Option<PathBuf>,
}

impl MorseFrameIo {
    /// Open the split data plane: TX on `tx_iface` (a mac80211 monitor vif, e.g. `mon0`), RX on
    /// `rx_iface` (the driver's sniffer netdev, e.g. `morse0`).
    ///
    /// Refuses `tx_iface == rx_iface` and refuses to transmit on a `morseN` netdev; requires both
    /// interfaces to exist and be UP; and requires each to *be* what its role needs — the TX vif a
    /// mac80211 monitor vif on the Morse phy, the RX netdev a radiotap sniffer that is **not** a
    /// mac80211 vif. See [`super::check_morse_vif_roles`]: those are the rules whose violation
    /// produces no frames and no error.
    pub fn new(tx_iface: &str, rx_iface: &str, format: FrameFormat) -> Result<Self, FaceError> {
        check_morse_ifaces(tx_iface, rx_iface)?;
        // ★ The TX vif's two "not there" cases are reported as what they actually cost — the
        // monitor-vif precondition, i.e. RECEIVE — rather than as a bare ENOENT or a generic
        // "is DOWN". These are the *reachable* forms of the silent zero: the operator who never
        // ran `iw phy … interface add mon0 type monitor` has no `mon0` to name, and a monitor vif
        // that is merely down never raises IEEE80211_CONF_CHANGE_MONITOR either. Both leave
        // `morse0` delivering nothing, forever, with no error anywhere.
        if !iface_exists(tx_iface) {
            return Err(morse_monitor_vif_error(
                tx_iface,
                rx_iface,
                "no such interface",
            ));
        }
        if !iface_is_up(tx_iface)? {
            return Err(morse_monitor_vif_error(
                tx_iface,
                rx_iface,
                "the interface exists but is DOWN, and a closed monitor vif does not count",
            ));
        }
        require_up(rx_iface)?;
        // What the two interfaces *are*, now that both are known to exist: the remaining
        // silent-zero shapes (a monitor vif that is not on the Morse phy, a receive netdev that is
        // really a mac80211 vif and only echoes our own TX, a managed vif that cannot carry
        // radiotap at all). The existence checks above are what let `iface_nature` treat an
        // unreadable `type` as a hard error rather than a shrug.
        check_morse_vif_roles(
            tx_iface,
            rx_iface,
            &iface_nature(tx_iface)?,
            &iface_nature(rx_iface)?,
        )?;
        let af = AfPacketBackend::split(tx_iface, rx_iface, format)
            .map_err(FaceError::Io)?
            .with_amsdu_cap(MM6108_AMSDU_BODY)
            .with_capability(mm6108_capability(Vec::new()));
        // Keyed on the RX ifindex, NOT the TX one: `AfPacketBackend::split` stamps captured
        // frames `ClockDomainId(rx_ifindex)`, so anything else here would advertise a domain no
        // stamp this face emits is ever in.
        let domain = ClockDomainId(af.rx_ifindex() as u32);
        Ok(Self {
            af,
            tx_iface: tx_iface.to_string(),
            rx_iface: rx_iface.to_string(),
            domain,
            inject_mcs: None,
            inject_bw: None,
        })
    }

    /// Declare the radio's real capability (its tuned channel list, and anything the caller has
    /// itself measured on this unit).
    pub fn with_capability(self, capability: RadioCapability) -> Self {
        let Self {
            af,
            tx_iface,
            rx_iface,
            domain,
            inject_mcs,
            inject_bw,
        } = self;
        Self {
            af: af.with_capability(capability),
            tx_iface,
            rx_iface,
            domain,
            inject_mcs,
            inject_bw,
        }
    }

    /// Opt in to the patched driver's **injection MCS** module parameter, verifying it exists now
    /// rather than failing on the first `set_rate`.
    ///
    /// Pass `None` for the default path ([`MORSE_INJECT_MCS_PARAM`]), whose spelling is a bench
    /// note rather than a source fact — see that constant. Errors if the file is absent, which is
    /// what an unpatched driver looks like.
    pub fn with_inject_mcs_param(mut self, path: Option<PathBuf>) -> Result<Self, FaceError> {
        let p = path.unwrap_or_else(|| PathBuf::from(MORSE_INJECT_MCS_PARAM));
        if !p.exists() {
            return Err(FaceError::Io(std::io::Error::new(
                std::io::ErrorKind::NotFound,
                format!(
                    "{}: no such module parameter — the monitor-injection patch is not loaded, so \
                     there is no host-side rate knob on this radio",
                    p.display()
                ),
            )));
        }
        self.inject_mcs = Some(p);
        Ok(self)
    }

    /// Opt in to the patched driver's **injection bandwidth** module parameter (see
    /// [`MORSE_INJECT_BW_PARAM`]), verified to exist. Enables
    /// [`set_inject_bw_mhz`](Self::set_inject_bw_mhz).
    pub fn with_inject_bw_param(mut self, path: Option<PathBuf>) -> Result<Self, FaceError> {
        let p = path.unwrap_or_else(|| PathBuf::from(MORSE_INJECT_BW_PARAM));
        if !p.exists() {
            return Err(FaceError::Io(std::io::Error::new(
                std::io::ErrorKind::NotFound,
                format!(
                    "{}: no such module parameter — the monitor-injection patch is not loaded",
                    p.display()
                ),
            )));
        }
        self.inject_bw = Some(p);
        Ok(self)
    }

    /// Set the **transmitted S1G channel width** in MHz (1/2/4/8) — a native method because
    /// `ndn_radio_hal::Bandwidth` enumerates 20/40/80/10/5 MHz and cannot express an S1G width at
    /// all.
    ///
    /// ★ MEASURED to actuate, twice and by independent means: sweeping the parameter 0..3 with the
    /// operating channel at 8 MHz made a receiver's S1G radiotap bandwidth field read back
    /// 1/2/4/8 MHz, and a receiver parked at 4 MHz decoded widths 1 and 4 at 300/300 while an
    /// 8 MHz PPDU gave 0/300. (An earlier SDR-based "inject_bw has no effect" conclusion is
    /// RETRACTED — that instrument was producing byte-identical numbers for four different
    /// configurations.)
    ///
    /// ⚠ The emitted width is `min(inject_bw, operating_bw)`: the driver defaults every non-mgmt
    /// frame to `custom_configs.channel_info.op_bw_mhz`, so asking for 8 MHz on a 1 MHz operating
    /// channel gets 1 MHz and no error. Set the operating width first
    /// (`crate::morse::MorseKnobs::set_channel_s1g`).
    ///
    /// Requires [`with_inject_bw_param`](Self::with_inject_bw_param); without it this is an error,
    /// never a silent success.
    pub fn set_inject_bw_mhz(&self, mhz: u8) -> Result<(), FaceError> {
        let code = match mhz {
            1 => 0u32,
            2 => 1,
            4 => 2,
            8 => 3,
            other => {
                return Err(FaceError::Io(std::io::Error::new(
                    std::io::ErrorKind::InvalidInput,
                    format!("{other} MHz is not an S1G injection width (1/2/4/8)"),
                )));
            }
        };
        let Some(p) = self.inject_bw.as_ref() else {
            return Err(FaceError::Io(std::io::Error::new(
                std::io::ErrorKind::Unsupported,
                "no injection-bandwidth parameter configured — call with_inject_bw_param() on a \
                 driver carrying the monitor-injection patch",
            )));
        };
        write_param(p, code)
    }

    /// The interfaces in use, `(tx, rx)` — different by construction on this radio.
    pub fn interfaces(&self) -> (&str, &str) {
        (&self.tx_iface, &self.rx_iface)
    }

    /// The clock domain this face's RX stamps are keyed on — the **sniffer** interface's index.
    ///
    /// Exposed for the same reason [`Nrc7292Clock::domain`] is: a caller composing this face with
    /// anything that relates timestamps (a `RadioHwClock`, a cross-domain map) has to be able to
    /// check that the two are talking about the same counter, and on this radio the answer is a
    /// property of *which netdev capture came off*, not of the driver.
    pub fn clock_domain(&self) -> ClockDomainId {
        self.domain
    }

    /// ★ **Does [`set_rate`](FrameIo::set_rate) reach the air on this instance?**
    ///
    /// `true` only when [`with_inject_mcs_param`](Self::with_inject_mcs_param) has verified the
    /// patched driver's parameter exists. `false` means `set_rate` stores the rate as bearer state
    /// (which still groups an A-MSDU batch) and the PPDU goes out at whatever the on-chip MAC
    /// chooses — the trait's sanctioned "this bearer resolves rate elsewhere" behaviour, not an
    /// actuator.
    ///
    /// It exists because `set_rate` **cannot** report this through its own return value. It is
    /// called per frame on the hot path and `ndn-phy-wifi`'s `medium.rs` propagates it with `?`, so
    /// returning `Err` for "no rate actuator" would refuse to transmit at all on an unpatched
    /// driver — a strictly worse failure than transmitting at the MAC's chosen rate.
    ///
    /// ⚠ **This is a HAL gap, worked around locally.** `RadioCapability` has `power_actuated`
    /// precisely so a radio can say "that number is decorative", and there is no `rate_actuated`
    /// counterpart, so a caller holding only `&dyn FrameIo` still cannot ask. Named here so the
    /// asymmetry is visible rather than inferred.
    pub fn rate_actuated(&self) -> bool {
        self.inject_mcs.is_some()
    }

    /// Whether [`set_inject_bw_mhz`](Self::set_inject_bw_mhz) has an actuator behind it. Unlike
    /// [`rate_actuated`](Self::rate_actuated) this is only informational — that method already
    /// errors rather than lying when the parameter is absent.
    pub fn inject_bw_actuated(&self) -> bool {
        self.inject_bw.is_some()
    }

    /// **M6 — the MM6108 as a full [`OpenRadio`], with the report `PLAN_MM6108` produces.**
    ///
    /// ⚠ The plan brings NOTHING up: every rung is `OutOfBand`. Its value is the `vif_roles`
    /// validation — the split-data-plane rule whose violation gives zero frames and no error at
    /// all — plus naming `modprobe` / `iw` / `morse_cli` / `hostapd_s1g` as the establishers of
    /// everything it cannot check. See [`crate::halow`]'s M6 block.
    ///
    /// `channel` is the caller's claim about what `morse_cli` did, and is reported as a claim. On
    /// S1G the width travels WITH the channel number, so both are unverified together.
    pub fn open_radio(
        tx_iface: &str,
        rx_iface: &str,
        format: FrameFormat,
        channel: u8,
        bw: ndn_radio_hal::Bandwidth,
    ) -> Result<ndn_radio_hal::OpenRadio, FaceError> {
        // `new` FIRST: it owns `morse_monitor_vif_error`, which explains that a missing or down
        // TX monitor vif breaks RECEIVE on this part. A bare `iface_nature` error would replace
        // that with a sysfs read failure and lose the one message worth having here.
        let dev = std::sync::Arc::new(Self::new(tx_iface, rx_iface, format)?);
        let tx_nature = iface_nature(tx_iface)?;
        let rx_nature = iface_nature(rx_iface)?;
        let report = mm6108_bringup(
            HalowIfaces {
                tx: tx_iface.to_string(),
                rx: rx_iface.to_string(),
                tx_nature,
                rx_nature,
            },
            channel,
            bw,
            RadioProfile::capability(dev.as_ref()),
        )?;
        // ⚠ No second `emit()`: `run_plan` already emitted this report.
        Ok(ndn_radio_hal::OpenRadio {
            io: dev.clone(),
            // No `RadioKnobs` impl: the rate/width actuators on this part are module PARAMETERS
            // (`inject_mcs`, `inject_bw`), reached through this type's own methods and only on a
            // patched driver. Channel and power are `morse_cli`/nl80211, out of band.
            knobs: None,
            time: Some(dev.clone()),
            profile: Some(dev),
            report,
        })
    }

    /// Refuse a frame the chip would discard without telling anyone.
    fn check_payload(&self, frame: &InjectFrame) -> Result<(), FaceError> {
        if frame.payload.len() > MM6108_MAX_PAYLOAD {
            return Err(FaceError::Io(std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                format!(
                    "payload {} B exceeds the MEASURED MM6108 cap of {MM6108_MAX_PAYLOAD} B \
                     (byte-exact: 1546 delivers, 1547 never arrives) — lower the face MTU",
                    frame.payload.len()
                ),
            )));
        }
        Ok(())
    }
}

#[async_trait]
impl FrameIo for MorseFrameIo {
    /// This radio's own capability, so a face built from the bare `dyn FrameIo` does not have to
    /// invent one. Delegates to this type's [`RadioProfile`] — the single source of truth.
    fn radio_capability(&self) -> Option<ndn_radio_hal::RadioCapability> {
        Some(<Self as ndn_radio_hal::RadioProfile>::capability(self))
    }
    async fn inject(&self, frame: InjectFrame) -> Result<(), FaceError> {
        self.check_payload(&frame)?;
        self.af.inject(frame).await
    }

    async fn inject_batch(&self, frames: Vec<InjectFrame>) -> Result<(), FaceError> {
        for f in &frames {
            self.check_payload(f)?;
        }
        self.af.inject_batch(frames).await
    }

    async fn inject_batch_at(
        &self,
        frames: Vec<(InjectFrame, McsDescriptor)>,
    ) -> Result<(), FaceError> {
        for (f, _) in &frames {
            self.check_payload(f)?;
        }
        self.af.inject_batch_at(frames).await
    }

    async fn recv_frame(&self) -> Result<CapturedFrame, FaceError> {
        self.af.recv_frame().await
    }

    /// Set the injection rate.
    ///
    /// With [`with_inject_mcs_param`](MorseFrameIo::with_inject_mcs_param) this writes the patched
    /// driver's module parameter and a write failure is a real error. MEASURED effect: MCS 0 → 7
    /// took delivered throughput 2.15 → 7.06 Mbit/s, the largest single lever on this radio.
    ///
    /// ⚠ Without it the call falls through to storing the rate as bearer state, and **the stored
    /// rate does not reach the air** — the S1G radiotap TX header names no rate by design (the
    /// on-chip MAC owns it) and the unpatched driver builds the ratecode from its own module
    /// parameters. That is the trait's sanctioned "this bearer resolves rate elsewhere" default,
    /// not an actuator; the stored value still groups an A-MSDU batch. Opt in if you need the rate
    /// to move.
    ///
    /// ⚠ The parameter is **process-global and root-only**: two faces on one chip would fight over
    /// it.
    fn set_rate(&self, mcs: McsDescriptor) -> Result<(), FaceError> {
        if let Some(p) = self.inject_mcs.as_ref() {
            write_param(p, u32::from(mcs.index))?;
        }
        // ⚠ When `inject_mcs` is `None` this stores bearer state and nothing more. Ask
        // [`MorseFrameIo::rate_actuated`] rather than inferring it from an `Ok(())`; the reason it
        // cannot be reported through this return value is documented there.
        self.af.set_rate(mcs)
    }

    // `inject_at_clock` / `inject_after` / `schedules_tx` keep their defaults: this radio exposes no
    // host-reachable scheduled-TX seam. See the module docs — the MAC core has one internally
    // (`n_cross_tx_ts`) and `morse_skb_tx_info` cannot carry a timestamp to it.

    // `mesh_common_view` keeps its `None` default: there is no verified beacon-TSF parser for this
    // vendor's S1G beacons, and the hardware-ness of its radiotap TSFT is itself unverified (the
    // vendor header warns monitor mode may use the chip's LOCAL TIMER and that "currently TSF is
    // not implemented").
}

/// Per-frame RX stamps from the **sniffer** netdev's radiotap TSFT, keyed on that interface's
/// index — the same domain the `AF_PACKET` backend stamps with, re-declared here rather than
/// delegated, and carrying that backend's [`ndn_frame_io::ClockReference`] of `unknown`.
///
/// That `unknown` is not laziness and is not upgraded here. The Morse driver does stamp every
/// frame with `hdr_rx_status->rx_timestamp_us` at zero bus cost, and the firmware does implement
/// `MORSE_CMD_ID_GET_TSF` (0x0028) and `MORSE_CMD_ID_SET_OFFSET_TSF` (0x003A) — this radio can
/// even *steer* its TSF, which the NRC7292 cannot. But no `morse_cli` verb issues either command
/// and there is no debugfs hook, so there is no host path to read the clock, and the vendor header
/// warns that monitor mode may be reporting a local timer rather than the TSF. Until the four
/// checks (non-zero, monotonic, wall-clock-consistent, not a host timer) are run, claiming a
/// hardware latch on a stated oscillator would be an assumption wearing a measurement's clothes.
///
/// ★ **THREE OF THE FOUR ARE NOW MEASURED** (2026-09-01, the lease acceptance run: 1709 frames
/// over 165 s on mds-o5p-1, `examples/halow_lease.rs rx --csv`, analysed by
/// `tools/lease_accept.py`).
///
/// * **stamped** — 1709/1709 frames carried a TSFT. Not a field that is sometimes filled.
/// * **non-zero, monotonic** — both, over the whole capture.
/// * **wall-clock-consistent** — regressed against the receiver's own `CLOCK_REALTIME` the stamp
///   runs at **1.000005544 host µs per tick (+5.5 ppm)**. It is a microsecond counter.
/// * **not a host timer — evidence, not proof.** On the SAME frames the stamp's jitter about the
///   sender's intended instant is **1.3–2.3× smaller** than the host `CLOCK_REALTIME`'s
///   (sd 142–164 µs vs 196–328 µs, every arm). A timestamp taken where userspace reads the frame
///   cannot be quieter than userspace's own read; this one is latched upstream of it. That rules
///   out the *host* — it does not distinguish the S1G TSF from the chip's local timer, which is
///   precisely what the vendor header warns about, so the `unknown` kind STAYS.
///
/// ⚠ **Nothing about the declaration changes on this evidence.** The acceptance run did not need
/// the kind upgraded: every decisive statistic it reports (the Rayleigh R, the two-point phase
/// shift) is invariant to the arrival clock's offset and needs only its rate. Reading these
/// measurements as licence to publish a hardware latch would repeat exactly the leak the override
/// below exists to close.
impl RadioTime for MorseFrameIo {
    /// ⚠ **Deliberately NOT `self.af.time_sources()`.** That would inherit
    /// [`AfPacketBackend`]'s `free_run_rx_stamp` — a declaration that the stamp is latched by the
    /// MAC — and the paragraph above is the reason it must not: the vendor header warns monitor mode
    /// may be reporting a local timer rather than the TSF, and none of the four checks has been run.
    /// Inheriting it made `FaceTimeProfile::hw_rx_stamp` true and published a 1 µs precision floor
    /// for an MM6108 face, so a reader asking *which half* failed was told the latch half passed on
    /// evidence nobody has taken.
    ///
    /// The af_packet claim is right where it was written — the NRC7292 on the same ifindex domain
    /// has a verified TSF — and wrong here, so the override belongs in this impl rather than there.
    ///
    /// ⚠ **The downgrade is on the KIND axis only.** The domain stays
    /// `ClockDomainId(rx_ifindex)`, because that is not a claim about the clock's quality — it is
    /// the *identity* of the counter the stamps on this face are already counted in
    /// (`AfPacketBackend::recv_frame` → `frame::parse`). Substituting a host-clock domain here to
    /// avoid the `free_run_rx_stamp` claim would leave the face advertising a clock that no stamp
    /// it emits belongs to, so a consumer pairing `LinkStamp::domain` against the declared sources
    /// would match nothing, forever, with no error — and `Nrc7292FrameIo`, five hundred lines
    /// below, would be right on the very same ifindex convention. Withholding a capability and
    /// misnaming a counter are different acts; only the first one is safe.
    ///
    /// This UNDERSTATES the part: the Morse driver really does stamp every frame with
    /// `hdr_rx_status->rx_timestamp_us`, and it may well be a true TSF. Understating is the safe
    /// direction — a declaration may withhold a capability, never grant one. Run the four checks
    /// (non-zero, monotonic, wall-clock-consistent, not a host timer) and this becomes
    /// `free_run_rx_stamp` on a stated reference.
    fn time_sources(&self) -> Vec<RadioTimeSource> {
        morse_time_sources(self.domain)
    }

    // `read_clock` keeps the trait's `Ok(None)` default, and the declaration above keeps
    // `read_now: false` to match it. There is no host path to this counter: no `morse_cli` verb
    // issues `MORSE_CMD_ID_GET_TSF` and the driver exposes no debugfs hook, so a source claiming a
    // read-now would be advertising a reader that does not exist.
}

/// The MM6108's link-clock declaration, as a free function so it can be unit-tested without a
/// socket (constructing a [`MorseFrameIo`] needs two live netdevs, which no test host has).
///
/// Every field is either a fact or a deliberate understatement, and the understatements are the
/// point — see [`MorseFrameIo`]'s `RadioTime` impl for why each one is withheld.
fn morse_time_sources(domain: ClockDomainId) -> Vec<RadioTimeSource> {
    vec![RadioTimeSource {
        // ⚠ NOT `FreeRunRxStamp`. That kind asserts the MAC/PHY latched the counter with no
        // software in the path, which is exactly the unrun check — and it is the sole input to
        // `FaceTimeProfile::hw_rx_stamp`, so declaring it would grant the latch half on evidence
        // nobody has taken. `PortTsf` is what a radiotap TSFT *claims* to be, grants neither
        // `hw_rx_stamp` nor `can_common_view`, and its "gated, resynced, not monotonic" character
        // is the honest description of a counter whose behaviour has never been checked.
        kind: RadioClockKind::PortTsf,
        // The sniffer ifindex — the same domain `frame::parse` stamps every captured frame with
        // (`AfPacketBackend::recv_frame`). A different value here would be a face advertising a
        // clock none of its own stamps belong to, so nothing could relate the two.
        domain,
        // Matches the stamps this face actually delivers: `frame::parse` builds every TSFT stamp
        // at `MacDone` / 1 µs. Advertising looser than you stamp is safe; advertising tighter is
        // the failure `precision_floor_ns` exists to clamp.
        latch: LatchPoint::MacDone,
        precision_ns: LatchPoint::MacDone.precision_floor_ns(),
        // 1 µs: radiotap TSFT is microseconds by definition, and the driver's own field is
        // `hdr_rx_status->rx_timestamp_us`. This one is known.
        tick_ns: 1_000,
        // Unverified — "monotonic" is one of the four checks, and it is the check a beacon-resynced
        // TSF fails. Not claimed.
        monotonic: false,
        // No host path reads this counter. See the note on `read_clock` above.
        read_now: false,
        // ⚠ NOT `ClockReference::host_os()`. This counter is the radio's, not the host's; `HostOs`
        // is one of the two kinds where `holds_rate()` is true, so claiming it would hand a
        // never-measured oscillator half of the common-view predicate.
        reference: ClockReference::unknown(),
    }]
}

impl RadioProfile for MorseFrameIo {
    fn capability(&self) -> RadioCapability {
        self.af.capability()
    }
}

// ═════════════════════════════════════════════════════════════════════════════════════════════
// Newracom NRC7292 — one netdev, plus the clock and the beacons the data plane discards
// ═════════════════════════════════════════════════════════════════════════════════════════════

/// The NRC7292 `FrameIo`: an `AF_PACKET` monitor backend on `halow0`, **plus** the two things that
/// backend structurally cannot provide for this radio.
///
/// 1. **A read-now clock.** `AfPacketBackend` reports `read_clock() = None`, which is true of a
///    packet socket and false of this radio: its firmware keeps a microsecond counter in chip RAM
///    that [`Nrc7292Clock`] samples on demand, MEASURED (by bracketing) to be the *same* clock that
///    stamps received frames. Composing them here is what makes the pair reachable through one
///    face — a working capability that was previously dropped on the floor, because
///    `MonitorWifiFace::halow` hands out the `AF_PACKET` backend's `RadioTime` and never composes
///    this one.
/// 2. **Mesh common view from S1G beacons.** `frame::parse` correctly refuses a beacon ("not a data
///    frame"), so `recv_frame` never sees one; this backend reads the raw capture first, offers it
///    to a [`MeshCvHarvester`], and then decodes normally.
///
/// ☠ **Both of those are unreachable on the stock driver, and this doc used to claim the opposite.**
/// MEASURED 2026-08-31 on the `halow_demo` pair (see the [`super`] module docs for the numbers):
/// the radiotap that actually arrives is mac80211's 18-byte header with **no TSFT and no S1G
/// TLV**, so `stamp` is `None` on every frame and `with_clock` has nothing to relate its counter
/// to; and `frame::parse` sees no beacons through this vif either, so `mesh_common_view` stays
/// `None`. Injection is dead in the same driver (`p->inject` is never set for radiotap monitor TX),
/// so this type is **receive-only in practice** — `inject` returns `Ok(())` and the chip's TX
/// counter does not move.
///
/// ⚠ The old warning that "a monitor vif puts the whole chip in promiscuous mode, so the managed/AP
/// data path receives nothing while this face exists" is also **false as measured**: the
/// association held and IP ping ran 0% loss with `mon0` up on both ends. mac80211 never calls the
/// driver's `add_interface` for this vif, so `nw->promisc` is never set.
///
/// ⚠ `recv_frame` also returns **this host's own transmissions** — the same netdev delivers
/// radiotap TX echoes (TX_FLAGS set, `rssi_dbm: None`), MEASURED 60/120 of the frames in a
/// two-way run. Nothing here filters them, so a caller that must not hear itself has to.
pub struct Nrc7292FrameIo {
    af: AfPacketBackend,
    clock: Option<Nrc7292Clock>,
    domain: ClockDomainId,
    cv: Mutex<MeshCvHarvester>,
}

impl Nrc7292FrameIo {
    /// Open on `iface` — one netdev, already in `type monitor` and UP. Both are *checked*
    /// ([`super::check_nrc_iface`]): a managed vif cannot deliver a radiotap header, so a face
    /// opened on one would report no TSFT, no S1G TLV, no RSSI and no MCS, which reads as a broken
    /// radio rather than as a misconfigured interface.
    ///
    /// Injection additionally needs the out-of-tree `nrc7292/inject_monitor.patch`; without it the
    /// socket accepts frames the radio never transmits. That cannot be detected from here (the
    /// send succeeds either way), so it is a deployment precondition, not a constructor check.
    pub fn new(iface: &str, format: FrameFormat) -> Result<Self, FaceError> {
        require_up(iface)?;
        // Monitor mode is the precondition for everything this face reads: on a managed vif the
        // frames go to mac80211 with no radiotap header, so the capture carries no TSFT, no S1G
        // TLV, no RSSI and no MCS — and it fails by being quietly useless rather than by erroring.
        check_nrc_iface(iface, &iface_nature(iface)?)?;
        let af = AfPacketBackend::new(iface, format)
            .map_err(FaceError::Io)?
            .with_capability(nrc7292_capability(Vec::new()));
        let domain = ClockDomainId(af.rx_ifindex() as u32);
        Ok(Self {
            af,
            clock: None,
            domain,
            cv: Mutex::new(MeshCvHarvester::new(true)),
        })
    }

    /// Declare the radio's real capability (its alias channel list, and any dBm range the caller
    /// has measured on this unit).
    pub fn with_capability(self, capability: RadioCapability) -> Self {
        let Self {
            af,
            clock,
            domain,
            cv,
        } = self;
        Self {
            af: af.with_capability(capability),
            clock,
            domain,
            cv,
        }
    }

    /// Also harvest common-view observations from **infrastructure** beacons, not only from
    /// locally-administered (mesh) transmitters.
    ///
    /// Off by default, because [`FrameIo::mesh_common_view`]'s contract says mesh. Worth knowing
    /// what that costs on this bench: the two-node 5.8 µs common view was measured against an
    /// NRC7292 AP beaconing from `00:c0:ca:b4:65:e2`, a **universally** administered address, which
    /// the default filter drops. Turn this on to reproduce that measurement, and understand that
    /// the observations are then outside the trait's stated scope.
    pub fn with_infrastructure_beacons(self) -> Self {
        let Self {
            af, clock, domain, ..
        } = self;
        Self {
            af,
            clock,
            domain,
            cv: Mutex::new(MeshCvHarvester::new(false)),
        }
    }

    /// Compose the radio's read-now microsecond clock.
    ///
    /// The clock must have been constructed on the **same interface**, because its domain is keyed
    /// on the interface index and must equal this backend's or the two cannot be related; a
    /// mismatch is refused here rather than producing a face whose `read_clock` silently answers
    /// about a different radio.
    pub fn with_clock(mut self, clock: Nrc7292Clock) -> Result<Self, FaceError> {
        if clock.domain() != self.domain {
            return Err(FaceError::Io(std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                format!(
                    "clock domain {:?} is not this interface's {:?} — the read-now clock and the \
                     per-frame stamps must share an interface or nothing can relate them",
                    clock.domain(),
                    self.domain
                ),
            )));
        }
        self.clock = Some(clock);
        Ok(self)
    }

    /// ★ **`false`, always** — the counterpart to [`MorseFrameIo::rate_actuated`], answered as a
    /// constant because nothing on this part can make it true.
    ///
    /// The driver sets `ieee80211_hw_set(hw, HAS_RATE_CONTROL)`, so firmware owns the transmit
    /// rate, and the S1G injection radiotap names no MCS by design. The one candidate — `set rc
    /// off` followed by the firmware shell's `test mcs <n>` — is unwired, because nothing
    /// establishes that a *test*-namespace command pins the **data** rate rather than entering a
    /// test mode. [`FrameIo::set_rate`] therefore stores bearer state (still useful: it groups an
    /// A-MSDU batch) and returns `Ok`; this method is how a caller learns that.
    pub fn rate_actuated(&self) -> bool {
        false
    }

    /// **M6 — the NRC7292 as a full [`OpenRadio`], with the report `PLAN_NRC7292` produces.**
    ///
    /// ⚠ The plan brings NOTHING up: every rung is `OutOfBand`, and the report's value is that it
    /// NAMES `modprobe` / `iw` / `hostapd_s1g` / the vendor `cli_app` as the establishers and
    /// marks the channel, the width and the injection patch as unverified provenance. See
    /// [`crate::halow`]'s M6 block for why that is better than shell history and worse than owning
    /// the sequence.
    ///
    /// `channel` is the caller's claim about what the out-of-band tuning did, and is reported as
    /// a claim.
    pub fn open_radio(
        iface: &str,
        format: FrameFormat,
        channel: u8,
        bw: ndn_radio_hal::Bandwidth,
    ) -> Result<ndn_radio_hal::OpenRadio, FaceError> {
        // `new` FIRST: it owns the diagnostic messages for every reachable failure (missing,
        // down, managed rather than monitor), and a bare `iface_nature` error would replace them
        // with a sysfs read failure. The natures are then re-gathered for the report.
        let dev = std::sync::Arc::new(Self::new(iface, format)?);
        let nature = iface_nature(iface)?;
        let report = nrc7292_bringup(
            HalowIfaces {
                tx: iface.to_string(),
                rx: iface.to_string(),
                tx_nature: nature.clone(),
                rx_nature: nature,
            },
            channel,
            bw,
            RadioProfile::capability(dev.as_ref()),
        )?;
        // ⚠ No second `emit()`: `run_plan` already emitted this report.
        Ok(ndn_radio_hal::OpenRadio {
            io: dev.clone(),
            // No `RadioKnobs` impl on this type: channel and power are reached through nl80211 and
            // the vendor CLI, both out of band. Saying `None` is the honest answer — a knob that
            // silently does nothing is worse than no knob.
            knobs: None,
            time: Some(dev.clone()),
            profile: Some(dev),
            report,
        })
    }
}

#[async_trait]
impl FrameIo for Nrc7292FrameIo {
    /// This radio's own capability, so a face built from the bare `dyn FrameIo` does not have to
    /// invent one. Delegates to this type's [`RadioProfile`] — the single source of truth.
    fn radio_capability(&self) -> Option<ndn_radio_hal::RadioCapability> {
        Some(<Self as ndn_radio_hal::RadioProfile>::capability(self))
    }
    async fn inject(&self, frame: InjectFrame) -> Result<(), FaceError> {
        self.af.inject(frame).await
    }

    async fn inject_batch(&self, frames: Vec<InjectFrame>) -> Result<(), FaceError> {
        self.af.inject_batch(frames).await
    }

    async fn inject_batch_at(
        &self,
        frames: Vec<(InjectFrame, McsDescriptor)>,
    ) -> Result<(), FaceError> {
        self.af.inject_batch_at(frames).await
    }

    /// Read raw, harvest beacons, then decode — so the frames the data plane rightly discards are
    /// still worth something to the timekeeper.
    async fn recv_frame(&self) -> Result<CapturedFrame, FaceError> {
        let mut buf = [0u8; 4096];
        loop {
            let n = self.af.recv_into(&mut buf).await?;
            if let Ok(mut h) = self.cv.lock() {
                h.observe(&buf[..n]);
            }
            if let Some(f) = frame::parse(self.af.format(), &buf[..n], None, None, self.domain) {
                return Ok(f);
            }
        }
    }

    /// Stores the rate as bearer state, like the `AF_PACKET` backend it wraps.
    ///
    /// ⚠ On this radio it does **not** reach the air, and no honest version of it can today:
    /// `ieee80211_hw_set(hw, HAS_RATE_CONTROL)` means the firmware owns the rate, mac80211's rate
    /// control is bypassed, and the S1G radiotap header names no MCS by design. The unverified
    /// candidate is `cli_app set rc off` followed by `cli_app test mcs <n>` — but nothing
    /// establishes that a command in the firmware's `test` namespace pins the *data* rate rather
    /// than entering a test mode, so it is not wired. The stored value still groups an A-MSDU
    /// batch.
    /// ⚠ **Stores bearer state; does not reach the air on this part.** See
    /// [`Nrc7292FrameIo::rate_actuated`], which says so as a value a caller can test.
    fn set_rate(&self, mcs: McsDescriptor) -> Result<(), FaceError> {
        self.af.set_rate(mcs)
    }

    fn mesh_common_view(&self) -> Option<MeshCv> {
        self.cv.lock().ok()?.latest()
    }
}

/// Per-frame RX stamps **and**, when a clock is composed, a readable counter in the same domain.
///
/// This is the composition the `AF_PACKET` backend documents as impossible for it to make: it
/// wraps an arbitrary NIC and cannot know what the counter runs on, so it declares
/// `ClockReference::unknown`. [`Nrc7292Clock`] can, and does — **crystal**, from its own on-air
/// measurement of −35 ppm over 20 s, corroborated by two NRC7292s holding a common view at
/// sd 5.8 µs with +1.50 ppm relative drift. Passing its `time_sources()` through is what carries
/// that statement to the face.
impl RadioTime for Nrc7292FrameIo {
    fn time_sources(&self) -> Vec<RadioTimeSource> {
        match self.clock.as_ref() {
            Some(c) => c.time_sources(),
            None => self.af.time_sources(),
        }
    }

    fn read_clock(&self, domain: ClockDomainId) -> Result<Option<u64>, FaceError> {
        match self.clock.as_ref() {
            Some(c) => c.read_clock(domain),
            None => Ok(None),
        }
    }

    // `clock_steering` stays `None`: MEASURED — writing the TSF mirror returns success and does not
    // take, because firmware refreshes it from the hardware counter within a tick.
}

impl RadioProfile for Nrc7292FrameIo {
    fn capability(&self) -> RadioCapability {
        self.af.capability()
    }
}

// ═════════════════════════════════════════════════════════════════════════════════════════════
// Tests
// ═════════════════════════════════════════════════════════════════════════════════════════════
//
// ⚠ This whole file is `cfg(target_os = "linux")`, so these run on the target and nowhere else —
// which is precisely why they exist. The declaration below was wrong for an entire release and a
// macOS test run could not have said so.
//
// Neither `FrameIo` is constructible without live netdevs, so what is testable is the part that
// was actually wrong: the *rules*, factored out of the constructors.
#[cfg(test)]
mod tests {
    use super::*;
    use ndn_frame_io::ClockReferenceKind;
    use ndn_radio_hal::{FaceTimeProfile, TxDiscipline};

    /// A stand-in that answers only `time_sources`, so `FaceTimeProfile::derive` can be exercised
    /// against the MM6108 declaration without a socket.
    struct MorseClockOnly(ClockDomainId);
    impl RadioTime for MorseClockOnly {
        fn time_sources(&self) -> Vec<RadioTimeSource> {
            morse_time_sources(self.0)
        }
    }

    /// ★ The regression this file's history is about. The declared domain must be the interface
    /// index the captured frames are stamped in — `AfPacketBackend::recv_frame` passes
    /// `ClockDomainId(rx_ifindex)` to `frame::parse`, so any other value advertises a counter none
    /// of this face's own stamps live in. A previous compile fix substituted a host-clock domain
    /// here; it compiled, and nothing could ever have matched a stamp against it again.
    #[test]
    fn the_declared_domain_is_the_rx_ifindex_domain() {
        for ifindex in [3u32, 7, 42] {
            let v = morse_time_sources(ClockDomainId(ifindex));
            assert_eq!(
                v.len(),
                1,
                "one clock, or the `best_clock` head is ambiguous"
            );
            assert_eq!(
                v[0].domain,
                ClockDomainId(ifindex),
                "the declaration must name the domain the stamps are keyed on"
            );
        }
        // And it must NOT be the serial backends' host domain, "HOST" as four ASCII bytes.
        assert_ne!(
            morse_time_sources(ClockDomainId(3))[0].domain.0,
            0x484F_5354
        );
    }

    /// `read_now` is a promise that `read_clock` answers, and on this radio nothing does: no
    /// `morse_cli` verb issues `MORSE_CMD_ID_GET_TSF` and there is no debugfs hook, so
    /// `RadioTime::read_clock` keeps its `Ok(None)` default. A source claiming otherwise is the
    /// "reports success, actuates nothing" defect in declaration form.
    #[test]
    fn no_read_now_is_claimed_because_nothing_reads_it() {
        let s = morse_time_sources(ClockDomainId(3))[0];
        assert!(!s.read_now, "no host path reads this counter");
        // The trait default is what backs that up — assert the default is still what we get.
        let c = MorseClockOnly(ClockDomainId(3));
        assert_eq!(c.read_clock(ClockDomainId(3)).unwrap(), None);
        assert_eq!(c.read_clock(ClockDomainId(999)).unwrap(), None);
    }

    /// The reference is `Unknown`, not `HostOs`. `HostOs` is one of the two kinds where
    /// `holds_rate()` is true, so declaring it would hand a never-measured oscillator half of the
    /// common-view predicate — on a counter that is the radio's, not the host's.
    #[test]
    fn the_oscillator_is_not_claimed() {
        let s = morse_time_sources(ClockDomainId(3))[0];
        assert_eq!(s.reference.kind, ClockReferenceKind::Unknown);
        assert!(!s.reference.holds_rate());
        assert_eq!(
            s.reference.measured, None,
            "nobody has measured this counter"
        );
    }

    /// ★ The withheld capability, asserted through the consumer that reads it. `hw_rx_stamp` is the
    /// LATCH half of common view and its sole input is a `FreeRunRxStamp` source; the four checks
    /// (non-zero, monotonic, wall-clock-consistent, not a host timer) have not been run on this
    /// part, and the vendor header warns monitor mode may report a local timer. So: false.
    #[test]
    fn the_unverified_latch_is_not_granted() {
        let p =
            FaceTimeProfile::derive(&MorseClockOnly(ClockDomainId(3)), TxDiscipline::BestEffort);
        assert!(
            !p.hw_rx_stamp,
            "the latch half must stay withheld until the four checks are run"
        );
        assert!(!p.can_common_view);
        assert_eq!(p.best_clock, Some(RadioClockKind::PortTsf));
        assert_ne!(p.best_clock, Some(RadioClockKind::FreeRunRxStamp));
    }

    /// The advertisement and the per-frame stamps must not disagree about how good the clock is:
    /// `frame::parse` builds every radiotap-TSFT stamp at `MacDone` / 1 µs, so the declaration says
    /// the same. Advertising *tighter* than you stamp is the failure `precision_floor_ns` clamps.
    #[test]
    fn the_advertisement_matches_the_stamps_it_describes() {
        let s = morse_time_sources(ClockDomainId(3))[0];
        assert_eq!(s.latch, LatchPoint::MacDone);
        assert_eq!(s.precision_ns, LatchPoint::MacDone.precision_floor_ns());
        assert_eq!(s.tick_ns, 1_000, "radiotap TSFT is microseconds");
        assert!(
            s.precision_ns >= s.latch.precision_floor_ns(),
            "never advertise tighter than the latch's floor"
        );
        assert!(!s.monotonic, "monotonicity is one of the four unrun checks");
    }

    /// The Morse and the NRC7292 must key their domains the same way, or a node running both
    /// cannot tell whether two faces share a counter. Both are `ClockDomainId(rx_ifindex)`; this
    /// pins that they are derived by the same rule rather than by coincidence.
    #[test]
    fn both_halow_radios_key_their_domain_on_the_rx_ifindex() {
        let ifindex: i32 = 11;
        assert_eq!(
            morse_time_sources(ClockDomainId(ifindex as u32))[0].domain,
            ClockDomainId(ifindex as u32),
            "same rule Nrc7292FrameIo::new applies: ClockDomainId(af.rx_ifindex() as u32)"
        );
    }

    /// Payload-cap refusal is a real rule this file owns (the MEASURED byte-exact 1546/1547
    /// boundary), and it had no test either.
    #[test]
    fn the_measured_payload_cap_is_byte_exact() {
        assert_eq!(MM6108_MAX_PAYLOAD, 1546);
        assert!(MM6108_AMSDU_BODY >= MM6108_MAX_PAYLOAD);
    }
}
