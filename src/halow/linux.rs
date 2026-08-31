//! Linux `AF_PACKET` data plane for the two HaLow radios. Split out because it is the only part of
//! [`crate::halow`] that needs a socket; every rule it enforces is decided by the platform-neutral
//! helpers next door, so the rules themselves are unit-tested on any host.

use std::path::{Path, PathBuf};
use std::sync::Mutex;

use async_trait::async_trait;
use ndn_frame_io::{
    AfPacketBackend, CapturedFrame, ClockDomainId, FaceError, FrameFormat, FrameIo, InjectFrame,
    McsDescriptor, RadioCapability, RadioProfile, RadioTime, RadioTimeSource, frame,
};
use ndn_radio_hal::MeshCv;

use super::{
    MM6108_AMSDU_BODY, MM6108_MAX_PAYLOAD, MORSE_INJECT_BW_PARAM, MORSE_INJECT_MCS_PARAM,
    MeshCvHarvester, check_morse_ifaces, mm6108_capability, nrc7292_capability,
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
/// * the two silent misconfigurations are refused at construction ([`super::check_morse_ifaces`]),
///   not discovered on air;
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
    inject_mcs: Option<PathBuf>,
    inject_bw: Option<PathBuf>,
}

impl MorseFrameIo {
    /// Open the split data plane: TX on `tx_iface` (a mac80211 monitor vif, e.g. `mon0`), RX on
    /// `rx_iface` (the driver's sniffer netdev, e.g. `morse0`).
    ///
    /// Refuses `tx_iface == rx_iface` and refuses to transmit on a `morseN` netdev; requires both
    /// interfaces to exist and be UP.
    pub fn new(tx_iface: &str, rx_iface: &str, format: FrameFormat) -> Result<Self, FaceError> {
        check_morse_ifaces(tx_iface, rx_iface)?;
        require_up(tx_iface)?;
        require_up(rx_iface)?;
        let af = AfPacketBackend::split(tx_iface, rx_iface, format)
            .map_err(FaceError::Io)?
            .with_amsdu_cap(MM6108_AMSDU_BODY)
            .with_capability(mm6108_capability(Vec::new()));
        Ok(Self {
            af,
            tx_iface: tx_iface.to_string(),
            rx_iface: rx_iface.to_string(),
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
            inject_mcs,
            inject_bw,
        } = self;
        Self {
            af: af.with_capability(capability),
            tx_iface,
            rx_iface,
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
/// index — delegated to the `AF_PACKET` backend, including its [`ndn_frame_io::ClockReference`]
/// of `unknown`.
///
/// That `unknown` is not laziness and is not upgraded here. The Morse driver does stamp every
/// frame with `hdr_rx_status->rx_timestamp_us` at zero bus cost, and the firmware does implement
/// `MORSE_CMD_ID_GET_TSF` (0x0028) and `MORSE_CMD_ID_SET_OFFSET_TSF` (0x003A) — this radio can
/// even *steer* its TSF, which the NRC7292 cannot. But no `morse_cli` verb issues either command
/// and there is no debugfs hook, so there is no host path to read the clock, and the vendor header
/// warns that monitor mode may be reporting a local timer rather than the TSF. Until the four
/// checks (non-zero, monotonic, wall-clock-consistent, not a host timer) are run, claiming a
/// hardware latch on a stated oscillator would be an assumption wearing a measurement's clothes.
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
    /// This UNDERSTATES the part: the Morse driver really does stamp every frame with
    /// `hdr_rx_status->rx_timestamp_us`, and it may well be a true TSF. Understating is the safe
    /// direction — a declaration may withhold a capability, never grant one. Run the four checks
    /// (non-zero, monotonic, wall-clock-consistent, not a host timer) and this becomes
    /// `free_run_rx_stamp` on a stated reference.
    fn time_sources(&self) -> Vec<RadioTimeSource> {
        // (compile fix: `self.domain` does not exist on this struct. Every other `host_recv`
        // caller in the crate passes HOST_CLOCK_DOMAIN, and that is the right domain by
        // definition — a HOST-received timestamp is in the host's clock, not the NIC's.)
        // The host clock domain, "HOST" as four ASCII bytes — the same constant the serial
        // backends use. Defined locally rather than reaching across modules: it is a wire-level
        // identity, not shared state.
        const HOST_CLOCK_DOMAIN: ndn_frame_io::ClockDomainId =
            ndn_frame_io::ClockDomainId(0x484F_5354);
        vec![RadioTimeSource::host_recv(HOST_CLOCK_DOMAIN)]
    }
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
/// ⚠ A monitor vif puts the whole chip in promiscuous mode (`nrc_mac_rx` diverts *all* receive to
/// the monitor path when `nw->promisc`), so the managed/AP data path receives nothing while this
/// face exists. Named-radio operation and a concurrent managed path are mutually exclusive on this
/// radio, as they are on the Morse.
pub struct Nrc7292FrameIo {
    af: AfPacketBackend,
    clock: Option<Nrc7292Clock>,
    domain: ClockDomainId,
    cv: Mutex<MeshCvHarvester>,
}

impl Nrc7292FrameIo {
    /// Open on `iface` — one netdev, already in `type monitor` and UP.
    ///
    /// Injection additionally needs the out-of-tree `nrc7292/inject_monitor.patch`; without it the
    /// socket accepts frames the radio never transmits. That cannot be detected from here (the
    /// send succeeds either way), so it is a deployment precondition, not a constructor check.
    pub fn new(iface: &str, format: FrameFormat) -> Result<Self, FaceError> {
        require_up(iface)?;
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
}

#[async_trait]
impl FrameIo for Nrc7292FrameIo {
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
