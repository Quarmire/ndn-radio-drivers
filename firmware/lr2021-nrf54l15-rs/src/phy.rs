//! **Which modulation this node is running — as a KNOB, not as an identity.**
//!
//! Pure logic: no MCU, no radio, no driver enums, so every per-PHY number that reaches the wire is
//! testable on the host. The bring-up that programs the chip lives in [`crate::phy_link`]; the
//! per-PHY modem blocks live in [`crate::flrc_link`], [`crate::lora_link`] and
//! [`crate::lrfhss_link`].
//!
//! ## The design error this module exists to undo
//!
//! `SetPacketType` (datasheet Table 8-1) is a **runtime command with 14 modes**. The firmware called
//! it once at bring-up, and the wire protocol then encoded that one-time choice as identity —
//! `EVT_CAP.radio_kind = 2` meaning "LR2021-FLRC", with "LR2021-LoRa" as a *separate kind*. That is
//! backwards. Modulation is a knob cognition actuates, exactly like MCS or spreading factor, and it
//! is fleet-wide rather than an LR2021 quirk: the SX1262 does LoRa and GFSK, the SX1276 does LoRa,
//! FSK and OOK.
//!
//! ## Everything per-PHY moves together
//!
//! `max_payload`, `sf_min`/`sf_max`, the airtime model, the band and `sched_gran_ns` are properties
//! of the **current mode**, not of the part. This chip in FLRC carries 47 bytes and has no spreading
//! factor; the same chip in LoRa carries 247 and has SF7..SF12. A stale field after a PHY switch is
//! the exact class of bug this pass exists to remove, which is why they are gathered here as
//! functions **of a [`Phy`]** rather than as free constants that any call site can forget to update.

use crate::serial::phy_code;

/// The PHYs this firmware actually brings up.
///
/// Deliberately **not** all fourteen `SetPacketType` values. A variant here is a promise that
/// [`crate::phy_link::apply`] has a real, ordered bring-up sequence for it — advertising a mode the
/// firmware cannot configure would put a host into the one state this protocol works hardest to
/// avoid: a knob that reports success and does nothing.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Phy {
    /// `SetPacketType` 0x0. The fleet's actual modulation — what lets this node talk to the
    /// SX1262/SX1276 nodes, and the prerequisite for intra-packet hopping.
    Lora,
    /// `SetPacketType` 0x5. Semtech's GMSK proprietary mode, up to 2.6 Mbit/s: the reason this board
    /// can host a slot MAC at a realistic timescale.
    Flrc,
    /// `SetPacketType` 0x7. Transmit-only per §17.1, receive-capable per §17.2.2 — the chip settles
    /// it, not the PDF. See [`crate::lrfhss_link`].
    LrFhss,
}

impl Phy {
    /// The chip's own `SetPacketType` value — the number that travels on the wire.
    pub const fn code(self) -> u8 {
        match self {
            Phy::Lora => phy_code::LORA,
            Phy::Flrc => phy_code::FLRC,
            Phy::LrFhss => phy_code::LR_FHSS,
        }
    }

    /// A wire byte back to a PHY this build brings up. `None` for a value that is either not a
    /// packet type at all or one this firmware does not configure — the caller answers
    /// `EVT_UNSUPPORTED`, never a silent fallback to the current mode.
    pub const fn from_code(code: u8) -> Option<Phy> {
        match code {
            phy_code::LORA => Some(Phy::Lora),
            phy_code::FLRC => Some(Phy::Flrc),
            phy_code::LR_FHSS => Some(Phy::LrFhss),
            _ => None,
        }
    }
}

/// Was this firmware built for the 2.4 GHz HF port? Mirrors the `PHY_HF` build switch that
/// [`BAND_MIN_HZ`] and the front-end calibration already key on.
pub const IS_HF: bool = matches!(option_env!("PHY_HF"), Some(_));

// ── Band ────────────────────────────────────────────────────────────────────────────────────────

/// The band this firmware was **built** for, and the only range a retune will accept.
///
/// LF: the US 902-928 MHz ISM band the rest of this bench uses. HF: the 2.4 GHz ISM band.
///
/// The bound exists because a `SetRfFrequency` outside the calibrated front end does not fail — it
/// returns success and the receiver goes deaf with `RXFREQ_NO_FE_CAL` set, which reads exactly like
/// a dead link. Answering `EVT_UNSUPPORTED` instead is the difference between "the host asked for
/// something this build cannot do" and an hour of bisecting a radio that is fine.
///
/// Lives here rather than in `flrc_link` because it is a property of the **front-end calibration**,
/// which every PHY shares — not of FLRC.
pub const BAND_MIN_HZ: u32 = if IS_HF { 2_400_000_000 } else { 902_000_000 };
/// See [`BAND_MIN_HZ`].
pub const BAND_MAX_HZ: u32 = if IS_HF { 2_483_500_000 } else { 928_000_000 };

/// Is `hz` inside the band this firmware was built for?
pub const fn in_band(hz: u32) -> bool {
    hz >= BAND_MIN_HZ && hz <= BAND_MAX_HZ
}

// ── PA range ────────────────────────────────────────────────────────────────────────────────────

/// The PA's register range on the band this firmware was built for, in the chip's **half-dB** unit.
///
/// From the driver's own contract on `set_tx_params`: *"TX Power in given in half-dB unit. Range is
/// -19..44 for LF Path and -39..24 for HF path"* ⇒ **LF −9.5..+22 dBm, HF −19.5..+12 dBm**. These
/// are the numbers `EVT_CAP` reports, so they come from that source constant and not from a
/// recollection of what the board "does".
pub const PWR_REG_MIN: i8 = if IS_HF { -39 } else { -19 };
/// See [`PWR_REG_MIN`].
pub const PWR_REG_MAX: i8 = if IS_HF { 24 } else { 44 };

/// Lowest whole dBm this PA can be asked for (`PWR_REG_MIN/2`, truncated toward zero so the value is
/// always *inside* the register range: LF's true floor is −9.5 dBm, and −9 is reachable).
pub const PWR_MIN_DBM: i8 = PWR_REG_MIN / 2;
/// Highest whole dBm this PA can be asked for.
pub const PWR_MAX_DBM: i8 = PWR_REG_MAX / 2;

/// Convert a host's real dBm into the chip's half-dB register unit, clamped to the PA range.
///
/// Clamping rather than rejecting: a host asking for more power than the part has should get the
/// most it has (and see the applied value echoed in `EVT_INFO`), not a failed command — but it must
/// never get a register write outside the authorised range, which §7.4.1 lists as a way to damage
/// the PA.
pub const fn dbm_to_half_db(dbm: i8) -> i8 {
    let reg = (dbm as i16) * 2;
    if reg < PWR_REG_MIN as i16 {
        PWR_REG_MIN
    } else if reg > PWR_REG_MAX as i16 {
        PWR_REG_MAX
    } else {
        reg as i8
    }
}

// ── Payload caps, per PHY ───────────────────────────────────────────────────────────────────────

/// **The serial link's own ceiling on a delivered frame.** `EVT_RX` carries
/// `[rssi(2), snr(2), ts(4), frame…]` inside a 7E-A5 body whose length field is one byte, so the
/// frame can never exceed `255 − 8`.
///
/// This is a real end-to-end bound and it binds before the chip does on every PHY except FLRC, whose
/// own fixed PDU is far smaller. `EVT_CAP.max_payload` is documented fleet-wide as
/// `min(TX accept, RX buffer, on-air PDU)`, and this is the "RX buffer" term.
pub const SERIAL_RX_PAYLOAD_MAX: usize = 255 - 8;

/// **Fixed FLRC frame size, both roles**, overridable at build time via `PHY_LEN`.
///
/// Variable-length framing plus CRC cannot work with one `pld_len` register serving both roles (the
/// receiver would have to know each frame's size before it arrives), and constant airtime per slot
/// is what the lease design wants anyway. See [`crate::flrc_link::build_frame`].
pub const FLRC_FRAME_LEN: u16 = match option_env!("PHY_LEN") {
    Some(s) if matches!(s.as_bytes(), b"8") => 8,
    Some(s) if matches!(s.as_bytes(), b"16") => 16,
    Some(s) if matches!(s.as_bytes(), b"24") => 24,
    Some(s) if matches!(s.as_bytes(), b"96") => 96,
    _ => 48,
};

/// One FLRC on-air frame minus the in-frame length byte — the real end-to-end payload cap in FLRC.
/// Small (47 bytes by default), and that is the truth of this bearer.
pub const FLRC_PAYLOAD_MAX: usize = FLRC_FRAME_LEN as usize - 1;

/// LoRa's end-to-end payload cap here: **247 bytes**, and every term is a real bound.
///
/// ```text
///   chip     LoRa `payload_len` is one register byte                    255
///   TX side  a 7E-A5 CMD_TX body                                        255
///   RX side  an EVT_RX body, minus its 8-byte rssi/snr/ts header        247   <- binds
/// ```
///
/// Reported as the smaller side, always: a node that accepts more on TX than it can deliver on RX
/// reports what it can deliver.
pub const LORA_PAYLOAD_MAX: usize = SERIAL_RX_PAYLOAD_MAX;

/// LR-FHSS's payload cap here, by the same three terms as [`LORA_PAYLOAD_MAX`].
///
/// **No PHY-level maximum is claimed**, because none is sourced: the vendor command takes the
/// payload as a variable-length write into a 256-byte command buffer, and the LoRa Alliance's
/// per-data-rate limits are regulatory (dwell time), not modem limits. At this cap the airtime is
/// *many seconds* (see [`crate::airtime::lrfhss_airtime_us`]) and a regional duty-cycle rule will
/// bind long before the PHY does — which is a fact about the band, not a number this firmware may
/// invent.
pub const LRFHSS_PAYLOAD_MAX: usize = SERIAL_RX_PAYLOAD_MAX;

/// The largest payload **any** PHY on this node can carry — the size every shared frame buffer in
/// the bridge must be, so a PHY switch cannot silently truncate.
pub const MAX_PAYLOAD_ANY: usize = if LORA_PAYLOAD_MAX > LRFHSS_PAYLOAD_MAX {
    LORA_PAYLOAD_MAX
} else {
    LRFHSS_PAYLOAD_MAX
};

/// `EVT_CAP.max_payload` for a given PHY.
pub const fn max_payload(p: Phy) -> usize {
    match p {
        Phy::Lora => LORA_PAYLOAD_MAX,
        Phy::Flrc => FLRC_PAYLOAD_MAX,
        Phy::LrFhss => LRFHSS_PAYLOAD_MAX,
    }
}

// ── Spreading factor, per PHY ───────────────────────────────────────────────────────────────────

/// **SF7..SF12 — what this node actually configures in LoRa**, not the part's full Sf5..Sf12 range.
///
/// The chip reaches SF5 and SF6, and they are excluded deliberately rather than forgotten:
///
/// * **SF5 does not exist on an SX127x at all**, and the Heltec node is the intended peer.
/// * **SF6 on an SX127x needs `comp_sx127x_sf6_sw` plus implicit-header framing** — a different
///   packet format, not a different number — so advertising it as an ordinary rung would hand a host
///   a setting that configures locally and never interoperates.
///
/// This is also exactly the span the Waveshare node advertises (`sx1262::SF_MIN`/`SF_MAX` = 7/12),
/// which is what makes a fleet-wide `CMD_SET_MOD` sweep mean the same thing on every node.
pub const LORA_SF_MIN: u8 = 7;
/// See [`LORA_SF_MIN`].
pub const LORA_SF_MAX: u8 = 12;

/// `EVT_CAP.sf_min` for a given PHY. **0 means the PHY has no spreading factor**, which is the
/// field's documented "none" — not an unknown.
pub const fn sf_min(p: Phy) -> u8 {
    match p {
        Phy::Lora => LORA_SF_MIN,
        // FLRC has no spreading factor; its rate is a bitrate rung. LR-FHSS has none either — its
        // rate is a coding rate over a fixed 488.28125 Hz modulation.
        Phy::Flrc | Phy::LrFhss => 0,
    }
}

/// See [`sf_min`].
pub const fn sf_max(p: Phy) -> u8 {
    match p {
        Phy::Lora => LORA_SF_MAX,
        Phy::Flrc | Phy::LrFhss => 0,
    }
}

// ── Scheduling granularity, per PHY ─────────────────────────────────────────────────────────────

/// `EVT_CAP.sched_gran_ns` for a given PHY.
///
/// **All three coincide today, and that is a derivation rather than a coincidence.** The four terms
/// of [`crate::airtime::SCHED_GRAN_NS`] — the timer tick, `SetTx` over SPI, the PA ramp and the MCU
/// reaction — are every one of them PHY-independent: the frame is *staged* before the deadline
/// (FIFO write, or `LrFhssBuildFrame`, plus the PLL settle), and the only thing that happens after
/// it is the same five-byte `SetTx`.
///
/// It is exposed as a function of [`Phy`] anyway, because the alternative is a free constant that a
/// future PHY with a different fire path silently inherits — which is the stale-field bug this whole
/// module is built to prevent.
pub const fn sched_gran_ns(p: Phy) -> u32 {
    match p {
        Phy::Lora | Phy::Flrc | Phy::LrFhss => crate::airtime::SCHED_GRAN_NS,
    }
}

// ── Intra-packet hopping ────────────────────────────────────────────────────────────────────────

/// Chip table depth for both hopping mechanisms: `SetLoraHopping` takes up to 40 frequencies and
/// `WriteLrFhssHoppingTable` up to 40 `(freq, nb_symbols)` couples.
pub const MAX_HOPS: usize = 40;

/// Does this PHY have **intra-packet** frequency hopping — the carrier moving inside one frame?
///
/// FLRC does not. There is no FLRC hopping command on this part, and refusing is the only honest
/// answer: silently accepting a hop table in FLRC would leave a host believing its frames were
/// spread when they were sitting on one carrier.
pub const fn has_intra_packet_hopping(p: Phy) -> bool {
    match p {
        Phy::Lora | Phy::LrFhss => true,
        Phy::Flrc => false,
    }
}

/// Everything a `CMD_SET_HOP` request has to satisfy before any of it reaches the chip.
///
/// Validated as a unit and *before* the first register write, for the reason the retune path already
/// learned: a half-applied RF configuration is worse than a refused one, and a hop table that is
/// accepted for its first 20 entries and rejected for its 21st leaves the modem in a state no host
/// command describes.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum HopReject {
    /// The current PHY has no intra-packet hopping.
    WrongPhy,
    /// More than [`MAX_HOPS`] frequencies.
    TooManyHops,
    /// Hopping enabled with a zero dwell, or with no frequencies to hop between.
    EmptyPlan,
    /// A frequency outside the band this build is calibrated for.
    OutOfBand,
    /// A reserved bit of `hop_ctrl` was set.
    ReservedBits,
}

/// `hop_ctrl` bit 0 — enable. Everything else is reserved and must be zero.
pub const HOP_CTRL_ENABLE: u8 = 0x01;

/// Validate a `CMD_SET_HOP` request. `Ok(true)` = enable with this table, `Ok(false)` = disable.
///
/// Disabling is accepted in any PHY that *has* hopping and ignores the rest of the payload: "turn it
/// off" must never fail on the contents of a table that is about to be discarded.
pub fn check_hop(phy: Phy, ctrl: u8, period: u16, freqs: &[u32]) -> Result<bool, HopReject> {
    if !has_intra_packet_hopping(phy) {
        return Err(HopReject::WrongPhy);
    }
    if ctrl & !HOP_CTRL_ENABLE != 0 {
        return Err(HopReject::ReservedBits);
    }
    let enable = ctrl & HOP_CTRL_ENABLE != 0;
    if freqs.len() > MAX_HOPS {
        return Err(HopReject::TooManyHops);
    }
    if !enable {
        return Ok(false);
    }
    if period == 0 || freqs.is_empty() {
        return Err(HopReject::EmptyPlan);
    }
    let mut i = 0;
    while i < freqs.len() {
        if !in_band(freqs[i]) {
            return Err(HopReject::OutOfBand);
        }
        i += 1;
    }
    Ok(true)
}

// ── LoRa bandwidth codes: the FLEET's space, not the chip's ─────────────────────────────────────

/// `CMD_SET_MOD`'s `bw` byte → bandwidth in Hz, **accepting both code spaces the fleet uses**.
///
/// The Heltec node had to learn this the hard way (its C1 defect): the canonical space is
/// `0/1/2 = 125/250/500 kHz` and the shipped host also emits the legacy SX1262 register values
/// `0x04/0x05/0x06` for the same three widths. The two are disjoint, so decoding both costs nothing
/// and means a host that has not been updated gets the bandwidth it asked for instead of silently
/// getting 125 kHz.
///
/// Unlike the Heltec's version this returns `None` for an unknown code rather than falling back to
/// 125 kHz. The fallback is right for a node whose host may be older than its firmware; here the
/// node advertises `CMD_SET_PHY` and a v3 host is by construction current, so an unrecognised
/// bandwidth is a bug to surface (`REASON_OUT_OF_RANGE`) rather than a value to guess at.
pub const fn lora_bw_hz_of_code(code: u8) -> Option<u32> {
    match code {
        0 | 0x04 => Some(125_000),
        1 | 0x05 => Some(250_000),
        2 | 0x06 => Some(500_000),
        _ => None,
    }
}

/// Inverse of [`lora_bw_hz_of_code`] in the **canonical** space — what `EVT_INFO.bw` reports, so a
/// host can see which width was actually applied whichever space it asked in.
pub const fn lora_bw_code_of_hz(hz: u32) -> u8 {
    match hz {
        250_000 => 1,
        500_000 => 2,
        _ => 0,
    }
}

// ── What this build advertises ──────────────────────────────────────────────────────────────────

/// **`EVT_CAP.phy_bitmap` for this build**: bit N set == `SetPacketType` value N is usable here.
///
/// * **LoRa (0x0)** and **FLRC (0x5)** on every build. Both MEASURED on air 2026-08-28 — a LoRa link
///   delivered 3/3 at −31 dBm / SNR 15, FLRC 100/100.
/// * **LR-FHSS (0x7): LF build AND opt-in `PHY_LRFHSS=1`.** See the trap below.
///
/// ## ☠ Why LR-FHSS is off by default: entering it TRAPS the node (MEASURED)
///
/// The question the mode was made reachable to settle is now ANSWERED on hardware:
/// `SetRxContinuous` in LR-FHSS returns **`CMD_FAIL` from StandbyRc**, so §17.1's "transmit-only" is
/// true and §17.2.2's "detection on the receiver side" is misleading wording. That was worth the trip.
///
/// But the refusal leaves the part unusable: every later `CMD_SET_PHY` fails, `CMD_TX` returns
/// `ok=0`, and **only a full MCU reset recovers it**. Three recoveries were tried and MEASURED not to
/// work — `ClearErrors` (§6.7.3, opcode `0x01 0x11`), an in-firmware `NRESET` + re-apply, and
/// `NRESET` + a `get_version()` wake + re-apply. After the in-firmware reset the part reports
/// `chip_mode = Sleep` and still refuses everything, so whatever the MCU boot path does is not
/// reproduced by resetting the radio alone, and it has **not been identified**.
///
/// A bit here is a promise that the mode is *usable*. LR-FHSS is not, so the honest state is
/// unadvertised — otherwise a host that probes every advertised PHY bricks this node on contact.
/// `PHY_LRFHSS=1` re-enables it for investigation; the bring-up, the hopping-table commands and
/// `EVT_PHY_ERR` all stay in place behind it.
pub const PHY_BITMAP: u32 = (1 << phy_code::LORA)
    | (1 << phy_code::FLRC)
    | if IS_HF || option_env!("PHY_LRFHSS").is_none() {
        0
    } else {
        1 << phy_code::LR_FHSS
    };

/// Is `p` advertised by this build?
pub const fn advertised(p: Phy) -> bool {
    PHY_BITMAP & (1u32 << p.code()) != 0
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The wire code space **is** the chip's, and the round trip is exact. A re-mapping here would
    /// have to be identical in five firmwares and one host with nothing on the wire to say which
    /// numbering a byte was in.
    #[test]
    fn phy_codes_are_the_chips_own() {
        assert_eq!(Phy::Lora.code(), 0x0);
        assert_eq!(Phy::Flrc.code(), 0x5);
        assert_eq!(Phy::LrFhss.code(), 0x7);
        for p in [Phy::Lora, Phy::Flrc, Phy::LrFhss] {
            assert_eq!(Phy::from_code(p.code()), Some(p));
        }
        // Packet types the part has and this firmware does not bring up must NOT decode — the
        // bridge answers EVT_UNSUPPORTED, it does not fall through to the current mode.
        for c in [
            phy_code::FSK_GENERIC,
            phy_code::BLE,
            phy_code::BPSK,
            phy_code::WM_BUS,
            phy_code::OOK,
            phy_code::RAW,
            phy_code::ZWAVE,
            0xFF,
        ] {
            assert_eq!(Phy::from_code(c), None, "code {c:#04x} must not decode");
        }
    }

    /// Every bit the bitmap sets must be a PHY the firmware can name, and every PHY it can name on
    /// this build must be in the bitmap. A set bit is a promise `CMD_SET_PHY` has to keep.
    #[test]
    fn bitmap_and_variants_agree() {
        for bit in 0..32u8 {
            if PHY_BITMAP & (1u32 << bit) != 0 {
                assert!(
                    Phy::from_code(bit).is_some(),
                    "bitmap claims packet type {bit:#04x} with no bring-up"
                );
            }
        }
        assert!(advertised(Phy::Lora));
        assert!(advertised(Phy::Flrc));
        // ☠ LR-FHSS is NAMABLE but not ADVERTISED unless opted into: entering it traps the part
        // (MEASURED — see `PHY_BITMAP`). `from_code` still decodes it, which is exactly the
        // distinction this test exists to keep: the bring-up exists, the promise does not.
        assert!(Phy::from_code(phy_code::LR_FHSS).is_some());
        assert_eq!(
            advertised(Phy::LrFhss),
            !IS_HF && option_env!("PHY_LRFHSS").is_some()
        );
        // The default (LF, no opt-in) build, pinned: LoRa | FLRC.
        if !IS_HF && option_env!("PHY_LRFHSS").is_none() {
            assert_eq!(PHY_BITMAP, 0x0000_0021);
        }
    }

    /// **The stale-field test.** Everything `EVT_CAP` reports per-PHY must actually differ between
    /// PHYs where the hardware differs — if these all returned the same number the switch would be
    /// reporting a lie and no other test would notice.
    #[test]
    fn per_phy_capabilities_really_are_per_phy() {
        assert_eq!(max_payload(Phy::Flrc), 47);
        assert_eq!(max_payload(Phy::Lora), 247);
        assert_ne!(max_payload(Phy::Flrc), max_payload(Phy::Lora));

        assert_eq!((sf_min(Phy::Flrc), sf_max(Phy::Flrc)), (0, 0));
        assert_eq!((sf_min(Phy::Lora), sf_max(Phy::Lora)), (7, 12));
        assert_eq!((sf_min(Phy::LrFhss), sf_max(Phy::LrFhss)), (0, 0));

        assert!(has_intra_packet_hopping(Phy::Lora));
        assert!(has_intra_packet_hopping(Phy::LrFhss));
        assert!(!has_intra_packet_hopping(Phy::Flrc));

        // Buffers sized for the biggest, so a PHY switch cannot truncate.
        for p in [Phy::Lora, Phy::Flrc, Phy::LrFhss] {
            assert!(max_payload(p) <= MAX_PAYLOAD_ANY);
        }
    }

    /// **Every per-PHY size has to fit the 7E-A5 frame that carries it**, and one byte of length
    /// field is all there is. A PHY whose payload cap exceeded this would deliver frames the wire
    /// silently truncates — the failure mode `max_payload` exists to prevent.
    #[test]
    fn every_phy_payload_fits_the_serial_frame() {
        for p in [Phy::Lora, Phy::Flrc, Phy::LrFhss] {
            // EVT_RX is [rssi(2), snr(2), ts(4), frame…].
            assert!(8 + max_payload(p) <= crate::serial::MAX_PAYLOAD);
        }
        // The largest CMD_SET_HOP: [ctrl][period u16][n] + 40 frequencies.
        assert!(4 + 4 * MAX_HOPS <= crate::serial::MAX_PAYLOAD);
        // And the deepest table the chip will take is exactly the deepest the wire will carry, so
        // neither side is the silent limit.
        assert_eq!(MAX_HOPS, 40);
    }

    /// The granularity is the same on every PHY **because its terms are**. Pinned so that a future
    /// PHY with a different fire path has to change this test rather than inherit a wrong number.
    #[test]
    fn scheduling_granularity_is_phy_independent_for_these_three() {
        for p in [Phy::Lora, Phy::Flrc, Phy::LrFhss] {
            assert_eq!(sched_gran_ns(p), crate::airtime::SCHED_GRAN_NS);
            assert!(sched_gran_ns(p) > 0);
        }
    }

    #[test]
    fn hop_requests_are_validated_as_a_unit() {
        // Derived from the band this build was compiled for, not written as 915 MHz literals: the
        // same test has to mean the same thing on an HF build.
        let ok = [
            BAND_MIN_HZ + 1_000_000,
            BAND_MIN_HZ + 3_000_000,
            BAND_MIN_HZ + 5_000_000,
        ];
        assert_eq!(check_hop(Phy::Lora, 1, 8, &ok), Ok(true));
        assert_eq!(check_hop(Phy::LrFhss, 1, 8, &ok), Ok(true));
        // FLRC has no intra-packet hopping: refused, never ignored.
        assert_eq!(check_hop(Phy::Flrc, 1, 8, &ok), Err(HopReject::WrongPhy));
        // Disable is accepted in a hopping PHY whatever the rest of the payload says.
        assert_eq!(check_hop(Phy::Lora, 0, 0, &[]), Ok(false));
        assert_eq!(check_hop(Phy::Lora, 0, 0, &ok), Ok(false));
        // …but not in one that never had hopping to turn off.
        assert_eq!(check_hop(Phy::Flrc, 0, 0, &[]), Err(HopReject::WrongPhy));
        // Enabled with nothing to hop between, or a zero dwell, is a plan that cannot be actuated.
        assert_eq!(check_hop(Phy::Lora, 1, 8, &[]), Err(HopReject::EmptyPlan));
        assert_eq!(check_hop(Phy::Lora, 1, 0, &ok), Err(HopReject::EmptyPlan));
        // The chip's table depth.
        let deep = [BAND_MIN_HZ + 3_000_000; MAX_HOPS + 1];
        assert_eq!(check_hop(Phy::Lora, 1, 8, &deep), Err(HopReject::TooManyHops));
        assert!(check_hop(Phy::Lora, 1, 8, &deep[..MAX_HOPS]).is_ok());
        // Out of the calibrated band: refusing beats a receiver that goes deaf with
        // RXFREQ_NO_FE_CAL set and looks exactly like a dead link.
        assert_eq!(
            check_hop(Phy::Lora, 1, 8, &[BAND_MIN_HZ - 1]),
            Err(HopReject::OutOfBand)
        );
        assert_eq!(
            check_hop(Phy::Lora, 1, 8, &[BAND_MAX_HZ + 1]),
            Err(HopReject::OutOfBand)
        );
        assert!(check_hop(Phy::Lora, 1, 8, &[BAND_MIN_HZ, BAND_MAX_HZ]).is_ok());
        // Reserved bits are refused rather than masked off: a host setting one means something we
        // do not implement, and quietly doing the subset is how a knob comes to mean two things.
        assert_eq!(check_hop(Phy::Lora, 0x02, 8, &ok), Err(HopReject::ReservedBits));
        assert_eq!(check_hop(Phy::Lora, 0x80, 8, &ok), Err(HopReject::ReservedBits));
    }

    /// Both LoRa bandwidth code spaces decode, and `EVT_INFO` answers in the canonical one.
    #[test]
    fn lora_bandwidth_codes_decode_both_spaces() {
        assert_eq!(lora_bw_hz_of_code(0), Some(125_000));
        assert_eq!(lora_bw_hz_of_code(1), Some(250_000));
        assert_eq!(lora_bw_hz_of_code(2), Some(500_000));
        assert_eq!(lora_bw_hz_of_code(0x04), Some(125_000));
        assert_eq!(lora_bw_hz_of_code(0x05), Some(250_000));
        assert_eq!(lora_bw_hz_of_code(0x06), Some(500_000));
        // Unknown is refused here, not defaulted — see the doc comment.
        assert_eq!(lora_bw_hz_of_code(3), None);
        assert_eq!(lora_bw_hz_of_code(0x7F), None);
        for code in [0u8, 1, 2, 0x04, 0x05, 0x06] {
            let hz = lora_bw_hz_of_code(code).unwrap();
            assert_eq!(lora_bw_hz_of_code(lora_bw_code_of_hz(hz)), Some(hz));
        }
    }

    /// The PA clamp, in the direction that matters: never a register write outside the authorised
    /// range, and the applied value is what `EVT_INFO` echoes.
    #[test]
    fn power_clamps_into_the_authorised_register_range() {
        assert_eq!(dbm_to_half_db(0), 0);
        assert_eq!(dbm_to_half_db(6), 12);
        assert_eq!(dbm_to_half_db(127), PWR_REG_MAX);
        assert_eq!(dbm_to_half_db(-128), PWR_REG_MIN);
        for dbm in -128i16..=127 {
            let reg = dbm_to_half_db(dbm as i8);
            assert!(reg >= PWR_REG_MIN && reg <= PWR_REG_MAX);
        }
        // The advertised dBm span is inside the register range, never outside it.
        assert!(dbm_to_half_db(PWR_MIN_DBM) >= PWR_REG_MIN);
        assert!(dbm_to_half_db(PWR_MAX_DBM) <= PWR_REG_MAX);
    }
}
