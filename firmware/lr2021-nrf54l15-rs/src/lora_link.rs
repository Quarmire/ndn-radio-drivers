//! **The LoRa PHY** — `SetPacketType 0x0`, its parameters, its bring-up block, and its framing.
//!
//! This is the mode that puts the LR2021 on the fleet's *actual* modulation. The Waveshare (SX1262)
//! and Heltec (SX1276) nodes are LoRa nodes; until `CMD_SET_PHY` existed this board could only ever
//! talk to the other LR2021, because FLRC is a Semtech-proprietary GMSK mode nothing else in the rig
//! demodulates. It is also the prerequisite for intra-packet hopping — `SetLoraHopping` exists only
//! in LoRa mode.
//!
//! ## What is deliberately NOT copied from [`crate::flrc_link`]
//!
//! FLRC framing carries an in-frame length byte inside a fixed-size, software-whitened PDU. Both
//! exist for reasons that are FLRC's alone:
//!
//! * **the length byte** because one `pld_len` register serves both roles, so a variable TX length
//!   would need the receiver to know each frame's size before it arrives. LoRa has an **explicit
//!   header** that carries the length on air, so the receiver learns it from the frame;
//! * **the whitening** because FLRC has no whitening command on this part and a transition-starved
//!   payload slips GMSK polarity mid-frame. LoRa's chirp spread spectrum has no such failure mode,
//!   and it interleaves and whitens in the modem.
//!
//! Carrying either into LoRa would be strictly worse: an extra byte off the payload, and a frame
//! byte-incompatible with every other LoRa node in the fleet — which is the entire point of the
//! mode. So a LoRa frame is the payload, unmodified.

use embedded_hal::digital::OutputPin;
use embedded_hal_async::spi::SpiBus;

use lr2021::lora::{HeaderType, Ldro, LoraBw, LoraCr, LoraModulationParams, LoraPacketParams, Sf};
use lr2021::{BusyPin, Lr2021, Lr2021Error};

use crate::phy;

/// **Private-network syncword, 0x12** — the SX127x/SX126x legacy one-byte notation, which
/// `set_lora_syncword` takes directly.
///
/// `0x34` is the LoRaWAN *public* network value and would make this bench's traffic visible to (and
/// visible from) any LoRaWAN gateway in range. `0x12` is what the Waveshare and Heltec nodes use, and
/// two nodes that disagree about a syncword do not error — they simply never hear each other.
pub const SYNCWORD: u8 = 0x12;

/// Default spreading factor: **SF7**, the fastest rung this node advertises
/// ([`phy::LORA_SF_MIN`]).
///
/// LoRa is slow — a 48-byte frame at SF7/125 kHz is ~100 ms against FLRC's 230 µs — so the default
/// is the fast end of the ladder, and `CMD_SET_MOD` walks down it when a link needs reach. Chosen as
/// the boot value rather than a mid-ladder compromise because a slot MAC measured at SF12 would be
/// measuring the modulation, not the MAC.
pub const DEFAULT_SF: u8 = phy::LORA_SF_MIN;

/// Default bandwidth: **125 kHz**, the fleet's canonical `bw_code = 0` and what both LoRa nodes boot
/// with.
pub const DEFAULT_BW_HZ: u32 = 125_000;

/// Default coding rate: **4/5** (`cr_code = 1`), matching the fleet.
pub const DEFAULT_CR: u8 = 1;

/// Default preamble: **8 symbols**, the LoRa convention and what the SX127x/SX126x nodes use.
///
/// Note this is the field `CMD_SET_PREAMBLE` carries fleet-wide, and in LoRa it finally means what
/// the fleet says it means — *symbols*. In FLRC the same command has to be reinterpreted as AGC
/// preamble **bits**, because FLRC has no symbols; see [`crate::flrc_link::preamble_from_bits`].
/// That reinterpretation is a cost of running FLRC, not a property of the protocol.
pub const DEFAULT_PREAMBLE_SYMS: u16 = 8;

/// **The LoRa modem parameters.** Everything `CMD_SET_MOD` / `CMD_SET_PREAMBLE` move, in the
/// **fleet's** units (SF 7..12, bandwidth in Hz, CR 1..4 = 4/5..4/8, preamble in symbols) rather than
/// the chip's enums, so the wire values and the stored state cannot drift apart.
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct LoraParams {
    /// Spreading factor, [`phy::LORA_SF_MIN`]..=[`phy::LORA_SF_MAX`].
    pub sf: u8,
    /// Bandwidth in Hz — 125 000 / 250 000 / 500 000. Stored in Hz, not as a code, because the fleet
    /// has *two* code spaces for the same three widths (see [`phy::lora_bw_hz_of_code`]) and Hz is
    /// the only representation both agree on.
    pub bw_hz: u32,
    /// Coding rate 1..4 = 4/5..4/8 — the fleet's numbering, which happens to be the LR2021's own
    /// `LoraCr` 1..4 for the short-interleaved codes as well.
    pub cr: u8,
    /// Preamble length in **symbols**.
    pub preamble_syms: u16,
}

impl Default for LoraParams {
    fn default() -> Self {
        Self {
            sf: DEFAULT_SF,
            bw_hz: DEFAULT_BW_HZ,
            cr: DEFAULT_CR,
            preamble_syms: DEFAULT_PREAMBLE_SYMS,
        }
    }
}

impl LoraParams {
    /// Airtime of a `payload_len`-byte frame at these settings, µs — the standard Semtech formula,
    /// shared by arithmetic with the other two LoRa nodes. See [`crate::airtime::lora_airtime_us`].
    ///
    /// Unlike FLRC, LoRa airtime **depends on the payload**: the header is explicit and the frame is
    /// as long as it needs to be. So `EVT_TX_STARTED` has to be computed per transmit here, where in
    /// FLRC it is a constant of the link.
    pub fn airtime_us(&self, payload_len: u16) -> u32 {
        crate::airtime::lora_airtime_us(self.sf, self.bw_hz, self.cr, payload_len, self.preamble_syms)
    }

    /// The chip's spreading-factor enum. `None` outside the advertised span — the caller answers
    /// `REASON_OUT_OF_RANGE` rather than clamping, so `EVT_CAP.sf_min`/`sf_max` stays true.
    pub fn sf_enum(&self) -> Option<Sf> {
        Some(match self.sf {
            7 => Sf::Sf7,
            8 => Sf::Sf8,
            9 => Sf::Sf9,
            10 => Sf::Sf10,
            11 => Sf::Sf11,
            12 => Sf::Sf12,
            // Sf5/Sf6 exist on the part and are excluded on purpose — see `phy::LORA_SF_MIN`.
            _ => return None,
        })
    }

    /// The chip's bandwidth enum for the three widths the fleet uses.
    pub fn bw_enum(&self) -> Option<LoraBw> {
        Some(match self.bw_hz {
            125_000 => LoraBw::Bw125,
            250_000 => LoraBw::Bw250,
            500_000 => LoraBw::Bw500,
            _ => return None,
        })
    }

    /// The chip's coding-rate enum. **The fleet's 1..4 and the LR2021's `LoraCr` 1..4 coincide** for
    /// the short-interleaved Hamming codes, so this is an identity mapping and not a re-numbering —
    /// worth stating, because the long-interleaved codes 5..9 sit right above them and picking one
    /// by accident would produce a link no SX127x can decode.
    pub fn cr_enum(&self) -> Option<LoraCr> {
        Some(match self.cr {
            1 => LoraCr::Cr1Ham45Si,
            2 => LoraCr::Cr2Ham23Si,
            3 => LoraCr::Cr3Ham47Si,
            4 => LoraCr::Cr4Ham12Si,
            _ => return None,
        })
    }

    /// Low-data-rate optimisation, on exactly where the other two nodes turn it on: **SF ≥ 11 at
    /// 125 kHz**.
    ///
    /// The driver's own `LoraModulationParams::basic` is *wider* (it also enables at SF11/250 kHz),
    /// and this deliberately follows the fleet instead: LDRO changes the symbol mapping, so the two
    /// ends must agree, and the SX1262/SX1276 nodes are the ones we have to agree with.
    pub fn ldro(&self) -> Ldro {
        if self.sf >= 11 && self.bw_hz == 125_000 {
            Ldro::On
        } else {
            Ldro::Off
        }
    }

    /// The full modulation parameter block, or `None` if any field is outside what this node
    /// advertises.
    pub fn modulation(&self) -> Option<LoraModulationParams> {
        Some(LoraModulationParams::new(
            self.sf_enum()?,
            self.bw_enum()?,
            self.cr_enum()?,
            self.ldro(),
        ))
    }

    /// Packet parameters for a given payload length.
    ///
    /// **Explicit header, CRC on, standard IQ** — the fleet's configuration. Explicit header is what
    /// makes variable-length frames work at all here (see the module note), and it is what the
    /// SX127x nodes transmit.
    pub fn packet(&self, payload_len: u8) -> LoraPacketParams {
        LoraPacketParams::new(
            self.preamble_syms,
            payload_len,
            HeaderType::Explicit,
            true,
            false,
        )
    }
}

/// **The LoRa modem block of the shared bring-up sequence** — modulation, packet params, syncword,
/// in that order, called by [`crate::phy_link::apply`] between `SetRfFrequency` and the PA.
///
/// `payload_len` is programmed to the node's maximum here rather than to a frame size: with an
/// explicit header the receiver takes the length off the air, and the transmitter overwrites this
/// register per frame in [`crate::phy_link::tx_stage`]. That is the opposite of FLRC, where one
/// `pld_len` serves both roles and the payload length has to travel inside the frame.
///
/// `hop` is applied last, after the modulation it depends on — `SetLoraHopping` programs the
/// modem's TX hop table and the SX127x compatibility bit lives in `lora_modem_main_tx_cfg1`, both of
/// which a `SetLoraModulationParams` can reset.
pub async fn apply_modem<O, SPI, M>(
    radio: &mut Lr2021<O, SPI, M>,
    p: &LoraParams,
    hop: &crate::phy_link::HopPlan,
) -> Result<(), Lr2021Error>
where
    O: OutputPin,
    SPI: SpiBus<u8>,
    M: BusyPin,
{
    // A `LoraParams` that cannot be expressed is a firmware bug, not a host error: the bridge
    // validates every `CMD_SET_MOD` field against the same tables before storing it. `CmdErr` is the
    // chip's own "invalid parameters" status, which is what this would have become one command later
    // anyway — reported here instead of half-programming the modem.
    let modulation = p.modulation().ok_or(Lr2021Error::CmdErr)?;
    radio.set_lora_modulation(&modulation).await?;
    crate::phy_link::dcdc_workaround(radio).await;
    radio.set_lora_packet(&p.packet(phy::LORA_PAYLOAD_MAX as u8)).await?;
    radio.set_lora_syncword(SYNCWORD).await?;
    apply_hopping(radio, hop).await
}

/// **Intra-packet frequency hopping** (`CMD_SET_HOP`) — `SetLoraHopping`, plus the SX127x
/// compatibility mode that makes the hop sequence agree with the Heltec node.
///
/// ⚠ **§9.8: the LR20xx and the SX127x hop INCOMPATIBLY by default.** "The internal timing, frequency
/// switching mechanisms, and control logic evolved between the chip generations, making them unable
/// to properly synchronize their hopping sequences." There is an explicit compatibility mode for it,
/// and this enables it **whenever hopping is on**, because the SX1276 (Heltec) is the intended peer
/// and a hop sequence only one end can follow is worse than no hopping at all — the link fails
/// mid-frame, which reads as a marginal channel rather than as a configuration error.
///
/// The mode is one bit of the internal register `lora_modem_main_tx_cfg1` at **0xF30A24**, wrapped by
/// the driver as `comp_sx127x_hopping`. It is **not** put into sleep retention (`ret_en = None`):
/// this node never sleeps, and a retention slot is a scarce resource that should be claimed by the
/// firmware that actually needs it rather than by whichever code touched the register first. Turning
/// it off with hopping is deliberate too — the bit only affects hopping, but leaving it set would
/// make "hopping off" a different modem configuration depending on what ran before.
///
/// ## Writing the hop table is the point
///
/// The frequencies come from the host, up to [`phy::MAX_HOPS`] of them, and the chip walks them in
/// order for `period` symbols each. That is the substrate a **name-derived** dwell schedule needs:
/// the sequence is ours to write, not Semtech's to compute from a seed.
///
/// ★ The vendor crate's `set_lora_hopping` has an **off-by-one** and is not used: it fills
/// `buffer[0..4 + 4n]` (opcode 0x02 0x2C, then the enable/period pair, then four bytes per
/// frequency) and then transmits `3 + 4n` of them, truncating the last frequency's low byte. With
/// `n = 0` the length is right by accident, which is why the disable path would have looked fine.
/// The command is built here instead, over the same public `buffer_mut()`/`cmd_buf_wr()` pair the
/// crate uses, so the fix does not fork the vendored driver.
pub async fn apply_hopping<O, SPI, M>(
    radio: &mut Lr2021<O, SPI, M>,
    hop: &crate::phy_link::HopPlan,
) -> Result<(), Lr2021Error>
where
    O: OutputPin,
    SPI: SpiBus<u8>,
    M: BusyPin,
{
    radio.comp_sx127x_hopping(hop.enabled, None).await?;
    let freqs = hop.freqs();
    let buf = radio.buffer_mut();
    buf[0] = 0x02;
    buf[1] = 0x2C;
    // Bit 6 enables hopping; the low 5 bits of byte 2 are the high bits of the 13-bit symbol period.
    buf[2] = if freqs.is_empty() {
        0
    } else {
        0x40 | ((hop.period >> 8) as u8 & 0x1F)
    };
    buf[3] = (hop.period & 0xFF) as u8;
    for (i, f) in freqs.iter().enumerate() {
        buf[4 + 4 * i..8 + 4 * i].copy_from_slice(&f.to_be_bytes());
    }
    // 4 header bytes + 4 per frequency — the length the vendor helper gets wrong.
    let len = if freqs.is_empty() { 3 } else { 4 + 4 * freqs.len() };
    radio.cmd_buf_wr(len).await
}
