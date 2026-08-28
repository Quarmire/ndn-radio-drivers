//! **The FLRC PHY** — its parameters, its bring-up block, and its framing.
//!
//! ★ **FLRC is one of three modes this node runs, not what this node *is*.** `SetPacketType` is a
//! runtime command; the mode is a knob the host turns with `CMD_SET_PHY`. The dispatcher and the
//! front-end sequence every PHY shares live in [`crate::phy_link`], the per-PHY *numbers* that reach
//! `EVT_CAP` live in [`crate::phy`], and this file is what is genuinely FLRC-specific:
//! [`FlrcParams`], [`apply_modem`], and the fixed-frame packing.
//!
//! Everything below still holds for FLRC and is kept verbatim — it is the record of a long
//! bring-up, and every constant in it was paid for.
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
use lr2021::system::{ChipMode, DioNum};
use lr2021::status::Intr;
use lr2021::{BusyPin, Lr2021, Lr2021Error, PulseShape};

/// Centre frequency, Hz — the quiet corner above US Wi-Fi ch11 and below BLE 2480. See module docs.
/// **#108 ROOT CAUSE, MEASURED 2026-08-06: the 2.4 GHz HF path is the fault. LF works perfectly.**
///
/// Same firmware, same modem settings, same two boards, one line changed:
///
/// | path | 48-byte frame | 96-byte frame, CRC on |
/// |---|---|---|
/// | HF, 2477 MHz | first ~8-15 bytes correct, rest polarity-inverted runs | every packet fails CRC |
/// | **LF, 915 MHz** | **48/48, every frame** | **96/96, every frame** |
///
/// Eight mechanisms were proposed and disproved before this — DC imbalance, polarity-ambiguous
/// syncword, front-end overload, FIFO transfer, missing TCXO, carrier frequency offset, demod
/// margin, CDR starvation — and a fix was built for each. The reason none of them moved the result
/// is that **the modem was never the problem**: the failure was invariant under bitrate (8x range),
/// coding rate, carrier offset (+/-60 kHz), packet format, CRC mode, syncword and payload whitening
/// precisely because every one of those is a modem parameter and the fault is in the RF path.
///
/// The lead came from reading Semtech's own PER example rather than theorising again: it validates
/// FLRC at `RF_FREQ_IN_HZ 866500000` — the LF path. The board has two SMA ports (LF 150-960 MHz,
/// HF 2.4 GHz), and the most likely physical cause is simply that **the antenna is on the LF port**,
/// so HF was transmitting into an unterminated output and the receiver was hearing near-field
/// leakage: strong enough to sync at bench range (-46 dBm), far too distorted to decode. Worth
/// confirming by eye before trusting HF again.
///
/// **Band-sharing caveat:** 915 MHz co-bands with the LoRa dongles and the HaLow radios on this same
/// bench, and FLRC at Br2600 occupies ~2.7 MHz. Expect mutual interference in a way 2.4 GHz avoided;
/// schedule against them or move to a band edge, as the LoRa/HaLow work already had to.

pub const FREQ_HZ: u32 = match option_env!("PHY_HF") {
    // **`PHY_HF=1` selects the 2.4 GHz HF path, which is BROKEN on this board — see below.**, which is where Semtech's own PER example
    // validates it (`RF_FREQ_IN_HZ 866500000`). 915 MHz keeps us in the US ISM band the rest of this
    // bench already uses. #108 is invariant under every HF-side parameter we can reach — bitrate,
    // coding rate, carrier offset, packet format, CRC, syncword, payload whitening — and the LF/HF
    // path is the one axis the vendor's validated configuration differs from ours on.
    // `PHY_MHZ` picks a coarse HF centre frequency. The +/-60 kHz sweep (m110) only ever probed
    // tuning error; it could not distinguish a broken path from a CONGESTED one. This bench's
    // 2.4 GHz is already documented as contended (the 8812au "loss at 3 ft" was contention, and
    // ch14 was the clean room), and 2477 MHz sits on Wi-Fi ch13.
    Some(_) => match option_env!("PHY_MHZ") {
        Some(m) if matches!(m.as_bytes(), b"2405") => 2_405_000_000,
        Some(m) if matches!(m.as_bytes(), b"2425") => 2_425_000_000,
        Some(m) if matches!(m.as_bytes(), b"2445") => 2_445_000_000,
        Some(m) if matches!(m.as_bytes(), b"2465") => 2_465_000_000,
        Some(m) if matches!(m.as_bytes(), b"2480") => 2_480_000_000,
        Some(m) if matches!(m.as_bytes(), b"2464") => 2_464_000_000,
        _ => 2_477_000_000,
    },
    None => 915_000_000,
};

/// 2.6 Mbit/s — the whole point of using FLRC. See module docs.
///
/// **Overridable at build time** via `PHY_BR` (`2600`/`1300`/`0650`/`0325`) so the rate ladder can
/// be swept as a *measured variable* across both roles at once. #108's XOR-mask readout
/// (`m111_xor_mask_rx`) showed polarity slips at random positions, ragged at the edges the way a
/// Viterbi decoder smears a transition — a demod **margin** signature rather than a configuration
/// one. Margin is tested by walking the rate down, not by argument.
pub const BITRATE: FlrcBitrate = match option_env!("PHY_BR") {
    Some(s) if matches!(s.as_bytes(), b"1300") => FlrcBitrate::Br1300,
    Some(s) if matches!(s.as_bytes(), b"0650") => FlrcBitrate::Br0650,
    Some(s) if matches!(s.as_bytes(), b"0325") => FlrcBitrate::Br0325,
    _ => FlrcBitrate::Br2600,
};

/// **CR 3/4 — matching Semtech's own reference**, not the `None` this started with.
///
/// The original rationale was "the MAC experiments want to see the raw link, so loss is not
/// silently repaired underneath the measurement". That reasoning is sound and the choice was still
/// wrong: Semtech's own FLRC packet-error-rate example
/// (`examples/main_examples/packet_error_rate_flrc_example` in Lora-net/usp) ships `FLRC_CR
/// RAL_FLRC_CR_3_4`, and a configuration the vendor does not exercise is not a baseline — it is an
/// untested corner. Get the link working against the reference first; revisit coding as a
/// *measured* variable afterwards.
/// Overridable at build time via `PHY_CR` (`12`/`23`/`34`) — see [`BITRATE`] on why the PHY is
/// swept rather than reasoned about.
pub const CODING: FlrcCr = match option_env!("PHY_CR") {
    Some(s) if matches!(s.as_bytes(), b"12") => FlrcCr::Cr12,
    Some(s) if matches!(s.as_bytes(), b"23") => FlrcCr::Cr23,
    // **`none` = FEC OFF, and it is the sharpest discriminator left for #108.** The HF dumps show
    // runs of correctly-decoded bytes alternating with runs of the same bytes BIT-INVERTED, at
    // constant length in *bytes* across an 8x rate change. That is what a Viterbi decoder does when
    // it slides between the true trellis path and its complement — convolutional codes are
    // transparent, so an unresolved polarity survives decoding, and the run length is the traceback
    // depth (a fixed number of bits ~ 4-8 bytes).
    //
    // With CR = None there is no trellis and no complementary path. So:
    //   clean (or uniformly inverted) => the raw demod is fine and the FEC decoder is the problem
    //   still garbage                 => the demodulator itself is wrong, and FEC was never involved
    Some(s) if matches!(s.as_bytes(), b"none") => FlrcCr::None,
    _ => FlrcCr::Cr34,
};

/// BT 0.5 — Semtech's reference uses `RAL_FLRC_PULSE_SHAPE_BT_05`; this was BT 1.0.
pub const PULSE_SHAPE: PulseShape = PulseShape::Bt0p5;

/// 32-bit AGC preamble — Semtech's reference uses `FLRC_PREAMBLE_BITS 32`; this was 16.
///
/// The **build-time default**; `CMD_SET_PREAMBLE` moves [`LinkState::preamble`] at runtime.
pub const PREAMBLE: AgcPblLen = AgcPblLen::Len32Bits;

/// Length in **bits** of an [`AgcPblLen`]. Thin wrapper over
/// [`crate::airtime::preamble_bits_of_code`] — the arithmetic lives there because it is host-testable
/// there. Used by [`LinkState::airtime_us`], which must not be handed a remembered constant that a
/// runtime `CMD_SET_PREAMBLE` has since moved.
pub const fn preamble_bits(p: AgcPblLen) -> u32 {
    crate::airtime::preamble_bits_of_code(p as u8)
}

/// The FLRC preamble **at least** `bits` long, or `None` if the register cannot reach it — the
/// enum-typed half of [`crate::airtime::preamble_code_for_bits`], which owns the rounding rule and
/// its tests.
pub const fn preamble_from_bits(bits: u16) -> Option<AgcPblLen> {
    match crate::airtime::preamble_code_for_bits(bits) {
        Some(0) => Some(AgcPblLen::Len4Bits),
        Some(1) => Some(AgcPblLen::Len8Bits),
        Some(2) => Some(AgcPblLen::Len12Bits),
        Some(3) => Some(AgcPblLen::Len16Bits),
        Some(4) => Some(AgcPblLen::Len20Bits),
        Some(5) => Some(AgcPblLen::Len24Bits),
        Some(6) => Some(AgcPblLen::Len28Bits),
        Some(_) => Some(AgcPblLen::Len32Bits),
        None => None,
    }
}

/// Syncword length on air, in bits — [`SwLen::Sw32b`], as programmed in [`pkt_params_pbl`]. Like the
/// preamble it is sent uncoded, so [`crate::airtime::airtime_us`] needs it separately from the
/// FEC-expanded payload.
pub const SYNC_BITS: u32 = 32;

/// Bytes the PHY appends for a given CRC width.
pub const fn crc_bytes(c: Crc) -> u32 {
    match c {
        Crc::CrcOff => 0,
        Crc::Crc16 => 2,
        Crc::Crc24 => 3,
        Crc::Crc32 => 4,
    }
}

/// [`CRC_MODE`] as a byte count, for the airtime arithmetic.
pub const CRC_BYTES: u32 = crc_bytes(CRC_MODE);

/// 32-bit syncword — **Semtech's reference value**, not the aesthetic one this started with.
///
/// It was `0x8624_4E44` (the NDN ethertype followed by ASCII `"ND"`): recognisable in a capture, and
/// chosen with no thought at all to its correlation properties. That is very likely to matter here.
/// FLRC is GMSK with a **convolutional** code, and convolutional codes are typically *transparent* —
/// an inverted input decodes to an inverted output — so GMSK's inherent 180° phase ambiguity is not
/// removed by the FEC. **The syncword is what resolves polarity**, and a syncword that correlates
/// well against its own complement leaves it unresolved.
///
/// That mechanism matches the observed failure exactly: frames sync (`sw_num = 1`) and are correctly
/// delimited, then the payload arrives as cleanly BIT-INVERTED bytes, at −33 dBm with every chip
/// error flag clear.
///
/// A syncword for this modem is an RF parameter with correlation requirements, not a branding
/// opportunity. If a recognisable value is wanted later, pick one and *verify* its autocorrelation
/// and complement-correlation rather than assuming.
pub const SYNCWORD: u32 = match option_env!("PHY_VENDOR") {
    // Semtech's own PER example ships `default_syncword[4] = {0x90,0x56,0x34,0x12}`. Selected by
    // `PHY_VENDOR=1` together with the rest of that example's packet config, so the vendor-validated
    // combination can be tested as a unit rather than one guessed parameter at a time.
    Some(_) => 0x1234_5690,
    None => 0xCD05_CAFE,
};

/// TX power in the chip's **HALF-dB register unit** — the unit `set_tx_params` actually takes.
///
/// Renamed from `TX_POWER_DBM`, which was a half-truth its own doc comment admitted to and which
/// the host bridge then believed: `CMD_SET_PWR` passed the host's i8 dBm straight into
/// `set_tx_params`, so every host power request came out **2× too low**. The value here is
/// unchanged — the default is register 12, i.e. **+6 dBm** — only the name now says what it is.
/// Convert at the boundary with [`dbm_to_half_db`]; nothing above this module should ever see the
/// register unit.
///
/// `PHY_PWR` sweeps it. The label sets differ per band because they always did: the LF sweep names
/// register values (`n30`/`p24`) and the HF sweep names Table 7-18 *rows* (`0`/`6`), and the HF rows
/// are a **matched pair** with [`PA_HF_DUTY`] (§1.5.2 + Table 7-18) rather than two free knobs.
pub const TX_POWER_HALF_DB: i8 = match option_env!("PHY_HF") {
    // HF rows from Table 7-18 (2445 MHz Semtech reference design):
    //   +12 dBm -> tx_power reg 24, duty 16   |   +6 dBm -> reg 16, duty 30   |   0 dBm -> reg 4, duty 30
    Some(_) => match option_env!("PHY_PWR") {
        Some(v) if matches!(v.as_bytes(), b"0") => 4,
        Some(v) if matches!(v.as_bytes(), b"6") => 16,
        _ => 24,
    },
    None => match option_env!("PHY_PWR") {
        Some(v) if matches!(v.as_bytes(), b"n30") => -30,
        Some(v) if matches!(v.as_bytes(), b"n16") => -16,
        Some(v) if matches!(v.as_bytes(), b"p12") => 12,
        Some(v) if matches!(v.as_bytes(), b"p24") => 24,
        // Default +6 dBm (register 12), raised from 0. The LF power sweep showed the link is CLEAN
        // at +6 and +12 dBm and receives NOTHING at −8 dBm, so 0 dBm was sitting far closer to the
        // cliff than intended.
        _ => 12,
    },
};

/// The HF PA duty-cycle half of the power pair. **Now in [`crate::phy_link`]** with the rest of the
/// front-end sequence it belongs to.
pub use crate::phy_link::PA_HF_DUTY;

/// The PA's register range and the dBm→half-dB conversion. **Now in [`crate::phy`]**, where they
/// are host-testable — the PA is the board's, not FLRC's, and `EVT_CAP` reports its range on every
/// PHY. Re-exported because several binaries name them here.
pub use crate::phy::{dbm_to_half_db, PWR_MAX_DBM, PWR_MIN_DBM, PWR_REG_MAX, PWR_REG_MIN};

/// The band this firmware was built for. **Now [`crate::phy::BAND_MIN_HZ`]** — the front-end
/// calibration is shared by every PHY, so the bound is not FLRC's to own. Re-exported because
/// several binaries name it here.
///
/// **HF caveat:** [`CAL_POINTS`](crate::phy_link::CAL_POINTS) brackets only 2448-2480 MHz, so an HF
/// build retuned below 2448 MHz is extrapolating its front-end calibration. Recorded, not silently
/// accepted; the HF path is broken for other reasons anyway (see [`FREQ_HZ`]).
pub use crate::phy::{in_band, BAND_MAX_HZ, BAND_MIN_HZ};

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
///
/// **Overridable at build time via `PHY_LEN`** so the break point can be mapped against frame
/// length. #108's role-swap test showed the corruption is byte-for-byte REPRODUCIBLE and symmetric
/// between the two boards — which a channel cannot do — so the remaining question is what structural
/// boundary the first ~8-14 good bytes end at. A length sweep answers that directly: a break at a
/// FIXED index means a boundary, one that SCALES means a proportional (coding/interleaver) fault.
/// **Defined in [`crate::phy::FLRC_FRAME_LEN`]**, so `EVT_CAP.max_payload` and the framing here
/// cannot disagree about the size of a frame.
pub const FRAME_LEN: u16 = crate::phy::FLRC_FRAME_LEN;

/// Kept as the name other modules use; now the fixed frame size.
pub const MAX_PAYLOAD: u16 = FRAME_LEN;

/// [`FRAME_LEN`] as a `usize`, for array sizes.
pub const FRAME_BYTES: usize = FRAME_LEN as usize;

/// **The real end-to-end payload cap**: one on-air frame minus the in-frame length byte.
///
/// This is the number `EVT_CAP.max_payload` carries. It is small (47 bytes at the default
/// `FRAME_LEN` = 48) and that is the truth of this bearer — reporting the serial link's 255 would
/// be a lie the host would size packets against.
pub const PAYLOAD_MAX: usize = crate::phy::FLRC_PAYLOAD_MAX;

/// Pack a variable-length payload into the **fixed** on-air frame, whitened, ready for the FIFO.
///
/// ```text
///   [0]        payload length n
///   [1..1+n]   payload
///   [1+n..]    zero padding to FRAME_LEN
/// ```
///
/// ★ **This is the fix for the measured "6 bytes then 44 bytes of stale FIFO" bug.** With
/// [`PktFormat::Fixed`] the chip transmits exactly `pld_len` bytes whatever the FIFO holds, so a
/// short FIFO write puts whatever the previous frame left behind on the air. Two ways out:
///
/// 1. re-program `pld_len` per frame, or
/// 2. keep the on-air PDU constant and carry the real length **inside** it.
///
/// (2) is what this does, because `pld_len` is **one register serving both roles** — on TX it is
/// the transmit length, on RX the accepted length — so per-frame TX lengths would need the receiver
/// to know each frame's size before it arrives, which it cannot. Constant airtime per frame is also
/// what the slot MAC wants (#93: the base slot is a constant, not something re-derived per frame),
/// and it keeps the PHY framing byte-identical to the configuration that is verified live on air.
///
/// Whitening covers the length byte and the padding, which is the point: the padding is exactly the
/// transition-free run that starves the demodulator's clock recovery (see [`whiten`]).
///
/// Returns `None` if the payload does not fit — never a truncated frame, because a silently
/// truncated NDN packet is worse than a refused transmit.
pub fn build_frame(payload: &[u8]) -> Option<[u8; FRAME_BYTES]> {
    if payload.len() > PAYLOAD_MAX {
        return None;
    }
    let mut f = [0u8; FRAME_BYTES];
    f[0] = payload.len() as u8;
    f[1..1 + payload.len()].copy_from_slice(payload);
    whiten(&mut f);
    Some(f)
}

/// Un-whiten a received on-air frame **in place** and return the real payload length, so the
/// payload is `frame[1..1 + n]`.
///
/// `None` means the length byte is impossible for this frame size — a frame that got through the
/// PHY CRC with a corrupt header, or a peer built with a different `PHY_LEN`. Dropping it is right
/// either way: delivering 200 bytes of padding to the host as "payload" is how a link problem gets
/// mistaken for an application one.
pub fn unpack_frame(frame: &mut [u8]) -> Option<usize> {
    whiten(frame); // self-inverse
    let n = *frame.first()? as usize;
    if n > PAYLOAD_MAX || 1 + n > frame.len() {
        return None;
    }
    Some(n)
}

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
/// Front-end calibration point: 4 MHz steps, MSB set to select the **HF** path.
/// `0x8000 | (2477 MHz / 4)`.
/// Three front-end calibration points, all on the path in use — see the call site in [`configure`].
///
/// HF brackets the 2.4 GHz ISM band (2400 / 2440 / 2480 MHz); LF brackets the 902-928 ISM band.
/// The shield devicetree's own calibration list (470 / 897.5 / 2441 MHz) is the same idea: several
/// points, spanning the band actually used.
/// Front-end calibration points. **Now in [`crate::phy_link`]** — a property of the board's band,
/// applied identically whichever PHY is running.
pub use crate::phy_link::CAL_POINTS;

pub const FE_CAL: u16 = match option_env!("PHY_HF") {
    // The MSB selects the HF path; on LF it must be clear, and the step is still 4 MHz.
    Some(_) => 0x8000 | ((FREQ_HZ / 4_000_000) as u16),
    None => (FREQ_HZ / 4_000_000) as u16,
};

/// TCXO start-up timeout, **in 32 MHz clock periods** — 5 ms.
///
/// This was `0`, copied from the shield devicetree's `tcxo-wakeup-time = <0>` as though that were
/// the chip's units. It is not, and the datasheet is explicit (§6.11.3): *"start_time indicates the
/// maximum duration for the 32MHz oscillator to start and stabilize … measured in 32MHz clock
/// periods. **0: (Default) disables TCXO mode**"*.
///
/// So the earlier "TCXO enabled" change was a **no-op** — it disabled the very mode it claimed to
/// turn on, and was committed as a fix. The radio has been running on the plain XOSC path
/// throughout, which is consistent with the measured symptom: the received ramp arrives intact at
/// the correct byte offsets but with **runs of bit-inverted bytes**, alternating every ~5–10 bytes.
/// At 1.95 Mbit/s effective that is a polarity flip roughly every ~33 µs ⇒ ~15 kHz flip rate ⇒ a
/// frequency offset of order **7.5 kHz (~3 ppm)** between the two boards. That is ordinary crystal
/// tolerance, and it sits *inside* the ±150 kHz figure in Table 18-3 — because that number is the
/// acquisition tolerance, not a promise of phase coherence across a packet.
///
/// 5 ms is a conservative settling allowance for a TCXO; it is a *timeout*, not a fixed delay, so
/// over-provisioning costs nothing once the oscillator is detected. Failure to detect raises
/// `HF_XOSC_START_ERR`, which `m108_flrc_diag` already prints.
pub const TCXO_STARTUP: u32 = 160_000;

/// **PHY CRC: 16-bit, matching Semtech's reference (`FLRC_CRC RAL_FLRC_CRC_2_BYTES`).**
///
/// This was `Crc24` for the whole bring-up, chosen because the datasheet lists 0/2/3/4-byte CRCs as
/// valid and 24 bits seemed a reasonable middle. The vendor's own working packet-error-rate example
/// uses **2 bytes**, and after aligning every other parameter to that example this was the last
/// remaining difference.
///
/// Datasheet §18.2.1 is the reason it matters that TX and RX agree exactly: *"The CRC calculation is
/// performed on the entire preceding packet, excluding the preamble"* — so the CRC covers the
/// syncword and header too, and its width changes where the payload ends.
///
/// Historical note kept because the technique generalises:
///
/// With `Crc24` the receiver reported `crc_error` on 100% of frames while everything else looked
/// right: strong signal (−33 dBm), syncword matched (`sw_num = 1`), `len_error = 0`, and `pkt_len`
/// tracking the real payload. Knob-by-knob guessing (length semantics, fixed vs variable framing,
/// Semtech's exact modulation, syncword width) moved none of it.
///
/// Turning the CRC off asks the one question that partitions the problem: **do the payload bytes
/// arrive intact?**
///   - bytes intact ⇒ the modulation/framing path is sound and only the CRC block is at fault
///   - bytes corrupt ⇒ it is alignment or modulation, and CRC was merely the messenger
///
/// Note a named-data MAC does not actually need the PHY's CRC: integrity is decided by the NDN
/// signature, and Tier-0 wants an integrity check it controls anyway. So `CrcOff` plus our own
/// checksum is a legitimate destination, not only a diagnostic — but that should be a decision made
/// on evidence, which is what this setting is for.
pub const CRC_MODE: Crc = Crc::Crc16;

/// **Software whitening — XOR the payload with a PRBS so it is DC-balanced on air.**
///
/// FLRC has **no whitening command** on this chip: `SetFskWhiteningParams`, `SetOokWhiteningParams`
/// and BLE's whitening init all exist, and there is no FLRC equivalent. So a DC-balanced payload is
/// the caller's responsibility, and ours was the opposite of balanced: the Tier-0 filter is sparse
/// (~29 of 94 bits set, so mostly `0x00` bytes) and frames are zero-padded to a fixed size.
///
/// The evidence that this is the fault, obtained by turning the PHY CRC off and looking at the
/// bytes: frames arrive **correctly delimited** — our forced `0x03` group bits, a plausible name
/// length, a leading `/` — and then degrade into **systematically BIT-INVERTED ASCII**
/// (`0xd0` = `~'/'`, `0xcf` = `~'0'`). Inversion rather than noise is a GMSK **polarity slip**: with
/// no transitions to track, clock and polarity recovery drift mid-frame. Random errors would look
/// random; these do not.
///
/// The LFSR is the classic 9-bit `x⁹ + x⁵ + 1` used by the SX12xx family, reset per frame, so the
/// function is **self-inverse**: apply on TX before writing the FIFO, apply again on RX after
/// reading, and payload content can no longer starve the demodulator of transitions.
pub fn whiten(buf: &mut [u8]) {
    let mut lfsr: u16 = 0x01FF; // all-ones seed, as in the SX12xx whitening sequence
    for b in buf.iter_mut() {
        let mut mask = 0u8;
        for bit in 0..8 {
            mask |= ((lfsr & 1) as u8) << bit;
            // x^9 + x^5 + 1: taps at bit 0 and bit 4 of the 9-bit register.
            let fb = ((lfsr ^ (lfsr >> 4)) & 1) << 8;
            lfsr = (lfsr >> 1) | fb;
        }
        *b ^= mask;
    }
}

fn pkt_params(pld_len: u16) -> FlrcPacketParams {
    pkt_params_crc(PREAMBLE, pld_len, CRC_MODE)
}

/// [`pkt_params`] with a runtime preamble — what [`apply`] uses, so `CMD_SET_PREAMBLE` reaches the
/// chip rather than being remembered in a struct nothing reads.
fn pkt_params_pbl(pbl: AgcPblLen, pld_len: u16) -> FlrcPacketParams {
    pkt_params_crc(pbl, pld_len, CRC_MODE)
}

fn pkt_params_crc(pbl: AgcPblLen, pld_len: u16, crc: Crc) -> FlrcPacketParams {
    // `PHY_VENDOR=1` reproduces Semtech's PER example exactly: `FLRC_PLD_IS_FIX false` (DYNAMIC) and
    // `FLRC_CRC RAL_FLRC_CRC_2_BYTES`.
    //
    // We moved to FIXED + CRC-off because DYNAMIC + CRC failed every packet — but that was diagnosed
    // with the receiver front end overloaded (RxBoost Max, RSSI −33 dBm), the one condition since
    // shown to actually matter (leading run 7 → 15 once it was turned off). A conclusion drawn under
    // a condition later proven wrong has to be re-tested, not inherited.
    let (fmt, crc) = match option_env!("PHY_VENDOR") {
        Some(_) => (PktFormat::Dynamic, Crc::Crc16),
        None => (PktFormat::Fixed, crc),
    };
    FlrcPacketParams::new(pbl, SwLen::Sw32b, SwTx::Sw1, SwMatch::Match1, fmt, crc, pld_len)
}

/// Disable the PHY CRC for a raw byte-in/byte-out experiment (#108).
///
/// Every diagnostic so far has read the received bytes through at least two layers of our own
/// encoding — a sparse Bloom filter, an ASCII name, a whitening LFSR — and the interpretation of
/// "corrupt" kept shifting as those layers changed. With CRC off and a known constant payload there
/// is exactly one question left: which bytes went in, which came out.
pub async fn set_crc_off<O, SPI, M>(radio: &mut Lr2021<O, SPI, M>) -> Result<(), Lr2021Error>
where
    O: OutputPin,
    SPI: SpiBus<u8>,
    M: BusyPin,
{
    radio.set_flrc_packet(&pkt_params_crc(PREAMBLE, FRAME_LEN, Crc::CrcOff)).await
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

/// The **runtime-mutable half of the link** — everything a host command can change without a
/// reflash, gathered into one value so the whole RF chain can be re-programmed from it atomically.
///
/// It exists because of a measured failure: `CMD_SET_FREQ` used to call `set_rf` while the chip sat
/// in RX-continuous, and after **any** retune — even to the 915 MHz it was already on — every
/// subsequent transmit returned `ok = 0` until the board was reset, and a later `CMD_SET_PWR` did
/// not restore it. A frequency change invalidates the front-end/PA/tx-params state that was
/// programmed *for the old frequency*, and the Semtech ordering
/// (`pkt_type → rf_freq → tx_cfg → mod_params → pkt_params → syncword → PA → rx_path → calibrate`)
/// is not a suggestion: half of it must be re-issued after the retune.
///
/// The obvious fix — a second copy of that sequence inside the retune path — is how the two copies
/// drift apart three commits later. So there is exactly one copy, [`apply`], and both `configure`
/// and [`retune`] call it.
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct LinkState {
    /// Carrier, Hz. Must satisfy [`in_band`].
    pub freq_hz: u32,
    /// TX power in the chip's **half-dB register unit** — see [`TX_POWER_HALF_DB`] and
    /// [`dbm_to_half_db`]. Deliberately stored in the register unit, at the one layer that is
    /// allowed to know it.
    pub tx_power_half_db: i8,
    /// FLRC bitrate rung. Runtime-settable (`CMD_SET_MOD`); a rate change used to need a reflash.
    pub bitrate: FlrcBitrate,
    /// FLRC coding rate.
    pub coding: FlrcCr,
    /// AGC preamble length. Runtime-settable (`CMD_SET_PREAMBLE`); see [`preamble_from_bits`] for
    /// how the fleet's symbol-count field is mapped onto this register's 4-bit steps.
    ///
    /// **Both ends must agree**, the same way they must agree about the syncword: the receiver's AGC
    /// is sized by this, and a node whose peer shortened its preamble does not report an error, it
    /// simply stops hearing it. A host that moves this must move it on every node.
    pub preamble: AgcPblLen,
}

impl Default for LinkState {
    /// The build-time link both nodes agree on with no host in the loop.
    fn default() -> Self {
        Self {
            freq_hz: FREQ_HZ,
            tx_power_half_db: TX_POWER_HALF_DB,
            bitrate: BITRATE,
            coding: CODING,
            preamble: PREAMBLE,
        }
    }
}

impl LinkState {
    /// TX power in real dBm — what `EVT_INFO`/`EVT_CAP` report, never the register unit.
    pub fn tx_power_dbm(&self) -> i8 {
        self.tx_power_half_db / 2
    }

    /// **Airtime of one on-air frame at this link's settings, in µs.** The whole fixed PDU, because
    /// `PktFormat::Fixed` puts exactly [`FRAME_LEN`] bytes on air whatever the payload was.
    ///
    /// Computed from the live [`LinkState`], never from the build-time constants: `CMD_SET_MOD` and
    /// `CMD_SET_PREAMBLE` both move terms of this sum, and an `EVT_TX_STARTED` deadline derived from
    /// stale constants would be wrong by up to 10x (the rung ladder spans 2600..260 kbit/s).
    pub fn airtime_us(&self) -> u32 {
        self.flrc_params().airtime_us()
    }

    /// [`airtime_us`](Self::airtime_us) as the two big-endian bytes `EVT_TX_STARTED` carries —
    /// whole milliseconds, rounded UP and never 0. See [`crate::airtime::airtime_ms_ceil`].
    pub fn airtime_ms_be(&self) -> [u8; 2] {
        crate::airtime::airtime_ms_ceil(self.airtime_us()).to_be_bytes()
    }

    /// The FLRC-only half of this link, for [`crate::phy_link::PhyState`].
    pub fn flrc_params(&self) -> FlrcParams {
        FlrcParams { bitrate: self.bitrate, coding: self.coding, preamble: self.preamble }
    }
}

/// **The FLRC modem parameters** — the part of the link that is genuinely FLRC's, split out from
/// [`LinkState`] so [`crate::phy_link::PhyState`] can hold one PHY's parameters without dragging in
/// the frequency and power every PHY shares.
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct FlrcParams {
    /// FLRC bitrate rung. Runtime-settable (`CMD_SET_MOD`); a rate change used to need a reflash.
    pub bitrate: FlrcBitrate,
    /// FLRC coding rate.
    pub coding: FlrcCr,
    /// AGC preamble length. Runtime-settable (`CMD_SET_PREAMBLE`); see [`preamble_from_bits`] for
    /// how the fleet's symbol-count field is mapped onto this register's 4-bit steps.
    pub preamble: AgcPblLen,
}

impl Default for FlrcParams {
    fn default() -> Self {
        Self { bitrate: BITRATE, coding: CODING, preamble: PREAMBLE }
    }
}

impl FlrcParams {
    /// **Airtime of one on-air frame at these settings, in µs.** The whole fixed PDU, because
    /// `PktFormat::Fixed` puts exactly [`FRAME_LEN`] bytes on air whatever the payload was.
    ///
    /// Computed from the live parameters, never from the build-time constants: `CMD_SET_MOD` and
    /// `CMD_SET_PREAMBLE` both move terms of this sum, and an `EVT_TX_STARTED` deadline derived from
    /// stale constants would be wrong by up to 10x (the rung ladder spans 2600..260 kbit/s).
    pub fn airtime_us(&self) -> u32 {
        crate::airtime::airtime_us(
            self.bitrate as u8,
            self.coding as u8,
            preamble_bits(self.preamble),
            SYNC_BITS,
            FRAME_LEN as u32,
            CRC_BYTES,
        )
    }
}

/// **The FLRC modem block of the shared bring-up sequence.**
///
/// Called by [`crate::phy_link::apply`] between `SetRfFrequency` and the PA/RX front end, i.e. at
/// exactly the point Semtech's own `ralf_lr20xx_setup_flrc()` puts it:
///
/// ```text
///   set_pkt_type -> set_rf_freq -> [ set_flrc_mod_params -> set_flrc_pkt_params
///                                    -> set_flrc_sync_word ] -> tx_cfg -> rx_path -> calibrate
/// ```
///
/// Two differences from what this firmware originally had: the FREQUENCY is set second (not last),
/// and the SYNCWORD is set LAST (not before the packet params). Order is not obviously load-bearing
/// for every one of these, but the reference implementation is the only ordering anyone has
/// validated.
///
/// (`set_flrc_crc_params` is deliberately absent: `ral_lr20xx_set_flrc_crc_params` returns early
/// when seed and polynomial are both 0 — "keep the default CRC params as is" — which is exactly what
/// the PER example passes. So it is a no-op for us, not a missing call.)
pub async fn apply_modem<O, SPI, M>(
    radio: &mut Lr2021<O, SPI, M>,
    p: &FlrcParams,
) -> Result<(), Lr2021Error>
where
    O: OutputPin,
    SPI: SpiBus<u8>,
    M: BusyPin,
{
    radio.set_flrc_modulation(p.bitrate, p.coding, PULSE_SHAPE).await?;
    crate::phy_link::dcdc_workaround(radio).await;
    // `pld_len` = the fixed on-air PDU, identical in both roles. TX transmits exactly this many
    // bytes and RX accepts exactly this many, which is why the payload length has to travel INSIDE
    // the frame — see [`build_frame`].
    radio.set_flrc_packet(&pkt_params_pbl(p.preamble, MAX_PAYLOAD)).await?;
    // One syncword, and RX matches only that one: this is a two-node experiment, and accepting
    // other syncwords would let stray traffic masquerade as our packets.
    //
    // `is_16b = FALSE`. This is a footgun and the crate's own doc example gets it wrong
    // (`set_flrc_syncword(1, 0xCD05CAFE, true)` alongside `SwLen::Sw32b`), which is where the bug
    // was copied from. With `true` the driver does `syncword << 16` — DISCARDING the high half —
    // and truncates the command to two syncword bytes, so a 32-bit value silently becomes a 16-bit
    // one while the packet params still declare `Sw32b`. Both ends misconfigure identically, so they
    // still sync (`sw_num = 1`) and the length still decodes — and every frame fails CRC, because
    // the frame is delimited differently from the region the CRC covers.
    radio.set_flrc_syncword(1, SYNCWORD, false).await
}

/// Program the **entire** RF chain from `link`, in Semtech's validated order, ending in Standby RC.
///
/// **Now a thin wrapper over [`crate::phy_link::apply`]**, which owns the one copy of the shared
/// sequence — regulator, packet type, frequency, per-PHY modem block, PA, TX params, RX path,
/// calibration. Kept because twenty binaries call it and every one of them is an FLRC binary.
///
/// Safe to call at any time and from any chip mode: it drops to Standby RC first, which is what
/// `Calibrate` and `CalibFe` require ("does not work if device is in Rx or Tx mode") and what makes
/// a retune out of RX-continuous legal. It does **not** re-arm RX and does not touch DIO routing —
/// the caller owns both, because `m5_tx` deliberately repurposes DIO8 after `configure` and an
/// `apply` that reset it would silently undo that.
pub async fn apply<O, SPI, M>(radio: &mut Lr2021<O, SPI, M>, link: &LinkState) -> Result<(), Lr2021Error>
where
    O: OutputPin,
    SPI: SpiBus<u8>,
    M: BusyPin,
{
    crate::phy_link::apply(radio, &crate::phy_link::PhyState::from_link(link)).await
}

/// Bring a reset LR2021 up as an FLRC node on the build-time link ([`LinkState::default`]).
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
    configure_with(radio, &LinkState::default()).await
}

/// [`configure`] with a caller-supplied starting [`LinkState`] — for a binary that boots on
/// something other than the build-time defaults.
pub async fn configure_with<O, SPI, M>(
    radio: &mut Lr2021<O, SPI, M>,
    link: &LinkState,
) -> Result<(), Lr2021Error>
where
    O: OutputPin,
    SPI: SpiBus<u8>,
    M: BusyPin,
{
    apply(radio, link).await?;

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

/// **Re-program the link and go back to listening.** The one supported way to change frequency,
/// power or rate at runtime.
///
/// Takes the whole [`LinkState`], not just a frequency, precisely so it cannot half-apply: a
/// retune that re-issued only `set_rf` would silently revert power and rate to the build defaults
/// on every channel change, which is the same class of bug as the one it fixes.
///
/// The caller is responsible for checking [`in_band`] first and answering the host `EVT_UNSUPPORTED`
/// for an out-of-band request — bricking the transmit path is not an acceptable way to say no.
pub async fn retune<O, SPI, M>(
    radio: &mut Lr2021<O, SPI, M>,
    link: &LinkState,
) -> Result<(), Lr2021Error>
where
    O: OutputPin,
    SPI: SpiBus<u8>,
    M: BusyPin,
{
    apply(radio, link).await?;
    radio.set_rx_continous().await
}

/// The DCDC switcher workaround has moved to [`crate::phy_link::dcdc_workaround`] — it is required
/// after every `set_*_modulation_params` and after `set_rx_path` on **any** PHY, so it belongs with
/// the shared sequence rather than with FLRC.

/// **Settle the synthesizer before keying the PA. Call this before every `set_tx`.**
///
/// #108's root cause, measured with a B210 at 2477 MHz: going straight from Standby to TX left the
/// PLL still converging while data was already on the air. Per-eighth mean carrier frequency across
/// a 194 µs burst, all-zeros payload:
///
/// ```text
///   straight to TX:   +104  +42  +31  +24  +21  +18  +17  +15  kHz   (27 kHz of drift under the payload)
///   FS settle first:   +66  +11  +10   +8   +9   +9  +10   +9  kHz   ( 2 kHz — flat after the preamble)
/// ```
///
/// The receiver locks phase off the syncword at the start of the frame; a carrier that then slides
/// 27 kHz out from under it produces a progressive constellation rotation, which is exactly the
/// payload-independent `00 -> 55 -> ff -> aa -> 00` walk the HF dumps showed. On the LF path the
/// same transient is small enough for the tracking loop to hold, which is why 915 MHz was always
/// clean and 2.4 GHz never was — and why every modem-side parameter (bitrate, coding rate, packet
/// format, CRC, syncword, whitening, RX boost, TX power, static LO offset) was irrelevant.
///
/// A longer `RampTime` fixes it too, but this way keeps `Ramp2u`: the ramp sits between the TX
/// trigger and the first symbol, so it is pure transmit-instant offset and the slot MAC (#93, and
/// M5's guard band) wants it as short as possible. Settling in FS costs time *before* the trigger,
/// where it is free.
///
/// Build with `PHY_NO_FS=1` to skip it — for reproducing the fault, not for operation.
///
/// **PHY-independent**: the synthesizer does not know which modem is downstream of it, so
/// [`crate::phy_link`] calls this on every transmit path. It lives here only because the binaries
/// that measured it name it here.
pub async fn settle_before_tx<O, SPI, M>(radio: &mut Lr2021<O, SPI, M>)
where
    O: OutputPin,
    SPI: SpiBus<u8>,
    M: BusyPin,
{
    if option_env!("PHY_NO_FS").is_some() {
        return;
    }
    let _ = radio.set_chip_mode(ChipMode::Fs).await;
    // 500 µs is generous: the capture shows the carrier flat by the end of the first eighth of a
    // 194 µs burst (~24 µs). Measured margin, not a guess — tighten only against another capture.
    embassy_time::Timer::after(embassy_time::Duration::from_micros(500)).await;
}
