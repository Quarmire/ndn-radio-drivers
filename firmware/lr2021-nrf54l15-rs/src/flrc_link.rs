//! The FLRC link both test nodes agree on — **one definition, shared by every binary**.
//!
//! FLRC (Fast Long Range Communication) is Semtech's GMSK-based proprietary mode. It is the reason
//! this board can host a slot MAC at all: at [`BITRATE`] a full frame is *sub-millisecond*, whereas
//! plain LoRa at SF7/125 kHz is ~5.5 kbit/s, making a 256-byte frame ~370 ms — a timescale on which
//! slot structure simply cannot be exercised.
//!
//! ## Why 2.4 GHz (the HF port) and not sub-GHz
//!
//! Both SMA ports are populated on these kits, so this is a real choice, and it is made *against*
//! sub-GHz on measurement grounds:
//!
//! - **The 902–928 MHz band on this bench already carries LoRa *and* HaLow**, and they have been
//!   measured interfering with each other there (mid-band collapse; the fix was to move to the band
//!   edge). This board exists to measure microsecond timing — putting it in the one band with known
//!   self-interference would pollute exactly the numbers it is here to produce.
//! - FLRC at 2.6 Mbit/s occupies roughly 2.4 MHz. That is comfortable at 2.4 GHz and awkward in a
//!   26 MHz ISM band that is already shared three ways.
//! - The Wi-Fi rig works mostly on 5 GHz, so 2.4 GHz collides with neither.
//!
//! [`FREQ_HZ`] then picks the quiet corner of 2.4 GHz the same way `ch14` was the clean room for the
//! Wi-Fi contention work: **above US Wi-Fi channel 11** (which ends ~2473 MHz — channels 12–14 are
//! not permitted in the US, so the top of the band is comparatively empty) and **below the BLE
//! advertising channel at 2480 MHz**. A 2.4 MHz-wide signal centred at 2477 MHz spans ~2475.8–2478.2
//! MHz and clears both.
//!
//! **This is a reasoned starting point, not a measured one.** Confirm it with a spectrum look before
//! trusting any timing result taken here — the standing lesson on this rig is that reasoning about
//! occupancy has a poor hit rate and measuring has a good one.

use embedded_hal::digital::OutputPin;
use embedded_hal_async::spi::SpiBus;

use lr2021::flrc::{AgcPblLen, Crc, FlrcBitrate, FlrcCr, FlrcPacketParams, PktFormat, SwLen, SwMatch, SwTx};
use lr2021::radio::{PacketType, RampTime, RxBoost, RxPath};
use lr2021::system::DioNum;
use lr2021::status::Intr;
use lr2021::{BusyPin, Lr2021, Lr2021Error, PulseShape};

/// Centre frequency, Hz — the quiet corner above US Wi-Fi ch11 and below BLE 2480. See module docs.
pub const FREQ_HZ: u32 = 2_477_000_000;

/// 2.6 Mbit/s — the whole point of using FLRC. See module docs.
pub const BITRATE: FlrcBitrate = FlrcBitrate::Br2600;

/// **CR 3/4 — matching Semtech's own reference**, not the `None` this started with.
///
/// The original rationale was "the MAC experiments want to see the raw link, so loss is not
/// silently repaired underneath the measurement". That reasoning is sound and the choice was still
/// wrong: Semtech's own FLRC packet-error-rate example
/// (`examples/main_examples/packet_error_rate_flrc_example` in Lora-net/usp) ships `FLRC_CR
/// RAL_FLRC_CR_3_4`, and a configuration the vendor does not exercise is not a baseline — it is an
/// untested corner. Get the link working against the reference first; revisit coding as a
/// *measured* variable afterwards.
pub const CODING: FlrcCr = FlrcCr::Cr34;

/// BT 0.5 — Semtech's reference uses `RAL_FLRC_PULSE_SHAPE_BT_05`; this was BT 1.0.
pub const PULSE_SHAPE: PulseShape = PulseShape::Bt0p5;

/// 32-bit AGC preamble — Semtech's reference uses `FLRC_PREAMBLE_BITS 32`; this was 16.
pub const PREAMBLE: AgcPblLen = AgcPblLen::Len32Bits;

/// 32-bit syncword — `0x8624_4E44` = the NDN ethertype `0x8624` followed by ASCII `"ND"`. Chosen to
/// be recognisable in a capture and unlikely to collide with stock FLRC/BLE traffic.
pub const SYNCWORD: u32 = 0x8624_4E44;

/// TX power, dBm. The HF PA table in the shield devicetree tops out at **12 dBm**; the two kits sit
/// on a bench, so start low — a strong link is not the goal and an overloaded receiver produces
/// garbage timing. Raise deliberately, never by default.
pub const TX_POWER_DBM: i8 = 0;

/// **Fixed frame size, both roles.**
///
/// Variable-length (`PktFormat::Dynamic`) was tried first and framing became correct — `pkt_len`
/// tracked the real payload — but **every packet still failed CRC** while `len_error` stayed 0 and
/// the signal sat at −33 dBm. The remaining explanation is that the receiver validates CRC over its
/// own `pld_len`, not over the length carried in the header, so a receiver configured for a maximum
/// can never check a shorter frame. With one `pld_len` register serving both roles, variable-length
/// + CRC needs the receiver to already know each frame's size — which it cannot.
///
/// Fixed size is also what a slot MAC actually wants: **constant airtime per slot** makes the base
/// slot a constant rather than something re-derived per frame, which is exactly the property the
/// lease design (#93) assumes.
pub const FRAME_LEN: u16 = 48;

/// Kept as the name other modules use; now the fixed frame size.
pub const MAX_PAYLOAD: u16 = FRAME_LEN;

/// FLRC packet parameters, rebuilt with a given payload length.
///
/// Split out because **`pld_len` means different things in the two roles**, which is the single
/// thing this driver's flat API hides and which cost the most here. Semtech's own stack keeps them
/// as separate fields (`radio_params.flrc.tx_size` vs `.max_rx_size`) precisely because one register
/// serves both:
///
/// - **TX: `pld_len` is the number of bytes actually transmitted.** Leave it at a large "maximum"
///   and the radio transmits that many bytes, underrunning a FIFO that holds fewer — so the CRC it
///   appends does not describe the frame, and **every** packet fails CRC at the receiver.
/// - **RX: `pld_len` is the maximum accepted length.** In variable-length mode an over-long packet
///   raises `LEN_ERROR` and the device stays in RX.
///
/// The symptom of getting this wrong is brutal to diagnose from the outside: strong signal
/// (−33 dBm), syncword matched, `len_error = 0`, and 100% `crc_error`, with the first bytes of each
/// frame intact — so a receiver that does not check CRC sees a working link.
fn pkt_params(pld_len: u16) -> FlrcPacketParams {
    FlrcPacketParams::new(
        PREAMBLE,
        SwLen::Sw32b,
        SwTx::Sw1,
        SwMatch::Match1,
        PktFormat::Fixed,
        Crc::Crc24,
        pld_len,
    )
}

/// Set the payload length. With [`PktFormat::Fixed`] both roles use [`FRAME_LEN`], so this is
/// normally only called by `configure`; it stays public for length experiments (#108).
pub async fn set_payload_len<O, SPI, M>(
    radio: &mut Lr2021<O, SPI, M>,
    len: u16,
) -> Result<(), Lr2021Error>
where
    O: OutputPin,
    SPI: SpiBus<u8>,
    M: BusyPin,
{
    // Spec: valid range is [6..511]; below 6 the command is rejected and the frame never goes out.
    radio.set_flrc_packet(&pkt_params(len.max(6))).await
}

/// Bring a reset LR2021 up as an FLRC node on [`FREQ_HZ`].
///
/// Ordering matters and follows the driver's documented sequence: packet type first (it selects
/// which modulation/packet registers exist), then modulation, syncword, packet params, then the
/// front end. Both nodes call this, so TX and RX cannot drift apart.
pub async fn configure<O, SPI, M>(radio: &mut Lr2021<O, SPI, M>) -> Result<(), Lr2021Error>
where
    O: OutputPin,
    SPI: SpiBus<u8>,
    M: BusyPin,
{
    radio.set_packet_type(PacketType::Flrc).await?;
    radio.set_flrc_modulation(BITRATE, CODING, PULSE_SHAPE).await?;

    // One syncword, and RX matches only that one: this is a two-node experiment, and accepting
    // other syncwords would let stray traffic masquerade as our packets.
    radio.set_flrc_syncword(1, SYNCWORD, true).await?;

    // Defaults to the RX role; a transmitter MUST call `set_payload_len` with the real frame size.
    radio.set_flrc_packet(&pkt_params(MAX_PAYLOAD)).await?;

    // HF front end for the 2.4 GHz port: PA on the HF path, RX on the HF path with full boost
    // (rx-boost-cfg = 7 in the shield devicetree).
    radio.set_pa_hf().await?;
    // **Ramp2u, not Ramp16u** (#104 lever 3). The PA ramp sits between the TX trigger and the first
    // on-air symbol, so its duration is pure transmit-instant offset — and any variation in it is
    // transmit-instant jitter, which is exactly the residual M5 left unexplained. 16 µs was an
    // arbitrary bring-up default; the shortest ramp is the right default for a slot MAC.
    radio.set_tx_params(TX_POWER_DBM, RampTime::Ramp2u).await?;
    radio.set_rx_path(RxPath::HfPath, RxBoost::Max).await?;
    radio.set_rf(FREQ_HZ).await?;

    // Route interrupts out on **DIO8**, which the shield wires to the MCU's P1.04.
    //
    // Easy to miss and expensive when missed: M3 polled the IRQ over SPI and never needed this pin,
    // so DIO8 sat idle and undriven. M4 then armed a DPPI capture on its edge and measured nothing —
    // a silent zero-sample result that looks exactly like "hardware timestamping does not work"
    // rather than "the interrupt was never routed to the pin". Configured here, in the shared setup,
    // so no binary can forget it.
    radio.set_dio_irq(DioNum::Dio8, Intr::new_txrx()).await?;

    Ok(())
}
