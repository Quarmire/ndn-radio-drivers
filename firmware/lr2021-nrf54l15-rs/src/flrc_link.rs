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
use lr2021::radio::{PacketType, PaLfMode, RampTime, RxBoost, RxPath};
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
    _ => FlrcCr::Cr34,
};

/// BT 0.5 — Semtech's reference uses `RAL_FLRC_PULSE_SHAPE_BT_05`; this was BT 1.0.
pub const PULSE_SHAPE: PulseShape = PulseShape::Bt0p5;

/// 32-bit AGC preamble — Semtech's reference uses `FLRC_PREAMBLE_BITS 32`; this was 16.
pub const PREAMBLE: AgcPblLen = AgcPblLen::Len32Bits;

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

/// TX power, dBm. The HF PA table in the shield devicetree tops out at **12 dBm**; the two kits sit
/// on a bench, so start low — a strong link is not the goal and an overloaded receiver produces
/// garbage timing. Raise deliberately, never by default.
/// TX power. **Note the unit: `set_tx_params` takes HALF-dB steps** (HF range −39..24 ⇒ −19.5 to
/// +12 dBm), so this constant's name is a half-truth inherited from the first draft. `PHY_PWR`
/// sweeps it — power is the one HF transmit knob #108 never varied (the PA *duty cycle* was swept,
/// which is a different register).
pub const TX_POWER_DBM: i8 = match option_env!("PHY_PWR") {
    Some(v) if matches!(v.as_bytes(), b"n30") => -30,
    Some(v) if matches!(v.as_bytes(), b"n16") => -16,
    Some(v) if matches!(v.as_bytes(), b"p12") => 12,
    Some(v) if matches!(v.as_bytes(), b"p24") => 24,
    // Default +6 dBm, raised from 0. The LF power sweep showed the link is CLEAN at +6 and +12 dBm
    // and receives NOTHING at −8 dBm, so 0 dBm was sitting far closer to the cliff than intended.
    _ => 12,
};

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
pub const FRAME_LEN: u16 = match option_env!("PHY_LEN") {
    Some(s) if matches!(s.as_bytes(), b"8") => 8,
    Some(s) if matches!(s.as_bytes(), b"16") => 16,
    Some(s) if matches!(s.as_bytes(), b"24") => 24,
    Some(s) if matches!(s.as_bytes(), b"96") => 96,
    _ => 48,
};

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
/// Front-end calibration point: 4 MHz steps, MSB set to select the **HF** path.
/// `0x8000 | (2477 MHz / 4)`.
/// Three front-end calibration points, all on the path in use — see the call site in [`configure`].
///
/// HF brackets the 2.4 GHz ISM band (2400 / 2440 / 2480 MHz); LF brackets the 902-928 ISM band.
/// The shield devicetree's own calibration list (470 / 897.5 / 2441 MHz) is the same idea: several
/// points, spanning the band actually used.
pub const CAL_POINTS: [u16; 3] = match option_env!("PHY_HF") {
    // **4 MHz grid.** `CalibFe` takes frequencies in 4 MHz steps, so a calibration point can only
    // ever land on a multiple of 4 MHz. `GetErrors` reports `RXFREQ_NO_FE_CAL` ("front end
    // calibration was not available for Rx operation with specified RF frequency") on BOTH paths at
    // our original 2477 / 915 MHz — neither of which is a multiple of 4. LF survived it only because
    // 915 MHz is exactly where §6.4 says the chip self-calibrates at boot; HF had no such luck.
    Some(_) => [0x8000 | 612, 0x8000 | 616, 0x8000 | 620], // 2448 / 2464 / 2480 MHz
    None => [225, 228, 232],
};

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
    pkt_params_crc(pld_len, CRC_MODE)
}

fn pkt_params_crc(pld_len: u16, crc: Crc) -> FlrcPacketParams {
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
    FlrcPacketParams::new(PREAMBLE, SwLen::Sw32b, SwTx::Sw1, SwMatch::Match1, fmt, crc, pld_len)
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
    radio.set_flrc_packet(&pkt_params_crc(FRAME_LEN, Crc::CrcOff)).await
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
    // **Enable the TCXO first.** The shield devicetree declares `tcxo-voltage = 1.8 V` with
    // `tcxo-wakeup-time = 0`, i.e. this board has a TCXO rather than a plain crystal — and nothing
    // in this firmware ever enabled it. The chip's own error description says so plainly:
    // "lf_xosc did not start correctly ... or there is a TCXO instead which must be enabled through
    // SetTcxoMode command."
    //
    // Running the radio off an un-started reference is consistent with the whole failure signature:
    // syncword matches (wide capture), length decodes, signal is strong at −33 dBm, and the payload
    // arrives cleanly BIT-INVERTED partway through — a mid-frame polarity/clock slip, which is what
    // a frequency-erroneous reference produces. Must precede any modulation or RF configuration.
    // **NO TCXO ON THIS BOARD — do not enable TCXO mode.** Proven, not assumed:
    // `SetTcxoMode` with a real (non-zero) start_time returns **CmdFail**, even from Standby RC,
    // which §6.11.3 names as the command's only valid mode. A non-zero start_time is a *timeout* for
    // detecting the 32 MHz oscillator, so failing it means no TCXO clock appeared.
    //
    // The shield devicetree lists `tcxo-voltage = 1.8V` alongside `tcxo-wakeup-time = <0>`, and the
    // zero is the operative half: it means "do not enable TCXO mode", exactly as the chip's own
    // `start_time = 0` does. Reading the voltage as evidence of a fitted TCXO was the error.
    //
    // Kept as a comment rather than a disabled call because a `set_tcxo(.., 0)` reads like it
    // enables something and does the opposite — that silent no-op was committed once already as a
    // fix, and it should not be re-introduced.

    // **All THREE calibration points must be supplied, and each carries its own path bit.**
    //
    // `CalibFe` declares freq1/freq2/freq3 as `optional: false`, and the MSB of each selects the
    // path (0 = LF, 1 = HF). Passing a single frequency leaves the crate to fill freq2/freq3 with
    // ZERO — which does not mean "unused", it encodes *LF path, 0 MHz*. On the LF path that is
    // merely redundant. On HF it hands the front end one HF point and two LF-path zeros, i.e. a
    // MIXED-PATH calibration of the ADC offset, PPF and image blocks.
    //
    // That is HF-specific, uniform across the band, and invisible to every modem parameter — which
    // is the exact signature #108 has: identical failure at 2405/2425/2445/2465/2477 MHz, unchanged
    // by bitrate, coding rate, packet format, CRC, syncword, whitening, the RF switch, and every
    // pa_hf_duty_cycle from 4 to 31.
    // **§6.3.18: `SetRegMode` defaults to SIMO_OFF (LDO) and must be issued in Standby RC.**
    // The shield devicetree specifies `reg-mode = DCDC`, and every PA figure in the datasheet is
    // characterised at "3.3 V SIMO". We had never called this at all.
    radio.set_chip_mode(ChipMode::StandbyRc).await?;
    // `set_regulator_mode(true)` emits `SimoUsage::Auto` == 2 == the datasheet's **SIMO_NORMAL**.
    // The crate's enum names do not match Table 6-26 — it calls 1 `All` and 3 `Vdcc` where the
    // datasheet marks both **RFU** — so this is right by value, not by name.
    let _ = radio.set_regulator_mode(true).await;

    // ── Semtech's own order, from `ralf_lr20xx_setup_flrc()` in Lora-net/usp ────────────────────
    //   set_pkt_type -> set_rf_freq -> set_tx_cfg(power, freq) -> set_flrc_mod_params
    //   -> set_flrc_pkt_params -> set_flrc_crc_params -> set_flrc_sync_word
    // Two differences from what we had: the FREQUENCY is set second (not last), and the SYNCWORD is
    // set LAST (not before the packet params). Order is not obviously load-bearing for every one of
    // these, but the reference implementation is the only ordering anyone has validated.
    //
    // (`set_flrc_crc_params` is deliberately absent: `ral_lr20xx_set_flrc_crc_params` returns early
    // when seed and polynomial are both 0 — "keep the default CRC params as is" — which is exactly
    // what the PER example passes. So it is a no-op for us, not a missing call.)
    radio.set_packet_type(PacketType::Flrc).await?;
    radio.set_rf(FREQ_HZ).await?;

    // One syncword, and RX matches only that one: this is a two-node experiment, and accepting
    // other syncwords would let stray traffic masquerade as our packets.
    // `is_16b = FALSE`. This is a footgun and the crate's own doc example gets it wrong
    // (`set_flrc_syncword(1, 0xCD05CAFE, true)` alongside `SwLen::Sw32b`), which is where the bug
    // was copied from. With `true` the driver does `syncword << 16` — DISCARDING the high half —
    // and truncates the command to two syncword bytes, so a 32-bit value silently becomes a 16-bit
    // one while the packet params still declare `Sw32b`. Both ends misconfigure identically, so they
    // still sync (`sw_num = 1`) and the length still decodes — and every frame fails CRC, because
    // the frame is delimited differently from the region the CRC covers.

    // Defaults to the RX role; a transmitter MUST call `set_payload_len` with the real frame size.
    radio.set_flrc_modulation(BITRATE, CODING, PULSE_SHAPE).await?;
    dcdc_workaround(radio).await;
    radio.set_flrc_packet(&pkt_params(MAX_PAYLOAD)).await?;
    radio.set_flrc_syncword(1, SYNCWORD, false).await?;

    // HF front end for the 2.4 GHz port: PA on the HF path, RX on the HF path with full boost
    // (rx-boost-cfg = 7 in the shield devicetree).
    if option_env!("PHY_HF").is_none() {
        // LF PA: the crate's HF helper hard-codes the HF PA selection, so the sub-GHz path needs the
        // explicit form. Duty cycle / slices copied from the HF helper's own defaults (6, 7).
        radio.set_pa_lf(PaLfMode::LfPaFsm, 6, 7).await?;
    } else {
        // **The crate's `set_pa_hf()` never sets `pa_hf_duty_cycle`.** It calls the 4-byte
        // `set_pa_config_cmd(HfPa, LfPaFsm, 6, 7)` — all three of those are the *LF* fields — while
        // the spec defines a 5th byte, `pa_hf_duty_cycle` (5 bits, default 16), described as
        // controlling "the duty cycle and maximum output power of the PA in HF mode". The crate even
        // has `set_pa_config_adv_cmd` for the 5-byte form and simply does not use it here.
        //
        // **Configured per datasheet Table 7-18, not by sweeping.** Three things it makes clear
        // that #108 had wrong:
        //
        // 1. §7.4.1: "**Only values from 16-31 are authorized** to avoid risk of aging the power
        //    amplifier", and non-optimized PA parameters risk "incorrect output power, excessive
        //    current consumption, **PA damage**, regulatory non-compliance". The earlier sweep of
        //    this field went down to 4 and 8 — outside that range. Do not repeat that.
        // 2. §1.5.2 + Table 7-18: `tx_power` and `pa_hf_duty_cycle` are a **matched pair**, not two
        //    independent knobs. Sweeping them separately never lands on a valid combination, which
        //    is why every earlier HF experiment was configured wrongly no matter what it varied.
        // 3. §7.4.3: `tx_power` is in **0.5 dB steps** (PA_HF −39..24 ⇒ −19.5..+12 dBm), so the
        //    register is 2x the dBm figure printed in Table 7-18.
        //
        // Rows used, from Table 7-18 (2445 MHz Semtech reference design):
        //   +12 dBm -> tx_power 12 dBm (reg 24), duty 16
        //    +6 dBm -> tx_power  8 dBm (reg 16), duty 30
        //     0 dBm -> tx_power  2 dBm (reg  4), duty 30
        let (hf_tx_pow, hf_duty): (i8, u8) = match option_env!("PHY_PWR") {
            Some(v) if matches!(v.as_bytes(), b"0") => (4, 30),
            Some(v) if matches!(v.as_bytes(), b"6") => (16, 30),
            _ => (24, 16),
        };
        let mut cmd = [0u8; 5];
        cmd[0] = 0x02;
        cmd[1] = 0x02;
        // pa_sel = 1 (HF), pa_lf_mode = 0, pa_lf_duty_cycle = 6, pa_lf_slices = 7 — the datasheet's
        // stated values for "LF PA not used".
        cmd[2] = (1 << 7) | (6 << 4);
        cmd[3] = 7;
        cmd[4] = hf_duty & 0x1f;
        radio.cmd_wr(&cmd).await?;
        radio.set_tx_params(hf_tx_pow, RampTime::Ramp2u).await?;
    }
    // **Ramp2u, not Ramp16u** (#104 lever 3). The PA ramp sits between the TX trigger and the first
    // on-air symbol, so its duration is pure transmit-instant offset — and any variation in it is
    // transmit-instant jitter, which is exactly the residual M5 left unexplained. 16 µs was an
    // arbitrary bring-up default; the shortest ramp is the right default for a slot MAC.
    if option_env!("PHY_HF").is_none() {
        radio.set_tx_params(TX_POWER_DBM, RampTime::Ramp2u).await?;
    }
    // **RX boost OFF, not Max.**
    //
    // The shield devicetree says `rx-boost-cfg = 7` and that was copied without asking what it is
    // for. The datasheet uses `rx_boost=7` for its SENSITIVITY figures — weak-signal conditions,
    // e.g. −100.5 dBm at BR 2600/CR 3/4. Our two boards sit two feet apart at **−33 dBm**, roughly
    // 67 dB above that, and running maximum LNA boost into a signal that strong overloads the front
    // end. The receiver then syncs fine (plenty of energy) and the demodulator distorts *during the
    // packet*, which is exactly the observed failure: alternating runs of correct and bit-inverted
    // bytes inside a single frame.
    //
    // This is the same trap the Wi-Fi work already documented — a point-blank link producing garbage
    // RX that looks like everything except what it is — and it explains why a dozen RF parameter
    // changes moved nothing: none of them touched the receiver's GAIN.
    //
    // Boost is a link-budget decision, not a constant. Raise it when the link is weak.
    let path = if option_env!("PHY_HF").is_some() { RxPath::HfPath } else { RxPath::LfPath };
    // `PHY_BOOST` sweeps the LNA boost. RxBoost::Off is the only change that ever improved #108
    // (leading run 7 → 15, RSSI −33 → −46), so its neighbourhood is worth mapping rather than
    // assuming the endpoint is optimal.
    let boost = match option_env!("PHY_BOOST") {
        Some(v) if matches!(v.as_bytes(), b"0") => RxBoost::Off,
        Some(v) if matches!(v.as_bytes(), b"4") => RxBoost::B4,
        Some(v) if matches!(v.as_bytes(), b"3") => RxBoost::B3,
        Some(v) if matches!(v.as_bytes(), b"5") => RxBoost::B5,
        Some(v) if matches!(v.as_bytes(), b"7") => RxBoost::Max,
        // **§7.3.2: "The recommended default values are: 0: In LF, 4: In HF."**
        // We had `Off` on both. That is right for LF and wrong for HF — and when the original
        // RxBoost::Max was found to be overloading the front end, the fix went straight to 0,
        // sailing past the recommended 4 without ever landing on it.
        _ if option_env!("PHY_HF").is_some() => RxBoost::B4,
        _ => RxBoost::Off,
    };
    radio.set_rx_path(path, boost).await?;
    dcdc_workaround(radio).await;

    // ── Calibration, LAST and from Standby RC ───────────────────────────────────────────────────
    //
    // §6.4: the chip boots with image rejection calibrated **at 915 MHz**, and "if operating at
    // another frequency the image calibration procedure must be restarted using command Calibrate
    // ... necessary if there is a frequency change > 10MHz". §6.4.1 adds PLL and AAF for changes
    // > 50MHz. We move 1562 MHz on HF.
    //
    // Both commands take no frequency and calibrate for the *current* configuration, so they run
    // last — after the frequency, the modulation params (the AAF is sized by bandwidth) and the RX
    // path. `Calibrate` cannot be issued in Rx or Tx, and `CalibFe` "does not work if device is in
    // Rx or Tx mode", so drop to Standby RC first rather than assuming which mode we are in.
    //
    // **Verified by `m113_errors`:** with this sequence `GetErrors` is clean through configure, TX
    // and RX entry. Without it the chip reports `RXFREQ_NO_FE_CAL` — "front end calibration was not
    // available for Rx operation with specified RF frequency" — on entering RX.
    radio.set_chip_mode(ChipMode::StandbyRc).await?;
    radio.calibrate(true, true, true, true, false, false).await?; // PA_OFF, MU, AAF, PLL
    radio.calib_fe(&CAL_POINTS).await?;

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

/// **DCDC switcher workaround — `lr20xx_workarounds_dcdc_configure()` from Semtech's driver.**
///
/// Not in the datasheet; it exists only in `lr20xx_workarounds.c` in Lora-net/usp, and the driver
/// calls it automatically at the end of **every** `set_*_modulation_params` and after
/// `set_rx_path`, via `LR20XX_WORKAROUNDS_CONDITIONAL_APPLY_AUTOMATIC_DCDC_CONFIGURE`.
///
/// It is required whenever the DCDC regulator is in use — which, since we now enable SIMO_NORMAL,
/// includes us. Enabling DCDC without it would have been a regression on the LF path, which works.
///
/// The header's prose says "sub GHz operations are intended", but the implementation branches on
/// `is_rx_hf` and programs both cases, so it applies to the HF path too:
///
/// ```text
///   ana_dec  = (reg 0x00F40200 >> 8) & 0x7
///   is_rx_hf = (reg 0x00F40430 & 0x3) == 1
///   if !is_rx_hf && (ana_dec == 1 || ana_dec == 2)  -> switcher rise 11, fall 13
///   else                                           -> switcher rise 15, fall 15
/// ```
///
/// Errors are swallowed: this is a best-effort register poke on an undocumented address, and a
/// failure here must not take down a link that otherwise works.
async fn dcdc_workaround<O, SPI, M>(radio: &mut Lr2021<O, SPI, M>)
where
    O: OutputPin,
    SPI: SpiBus<u8>,
    M: BusyPin,
{
    const ADC_CTRL: u32 = 0x00F4_0200;
    const RX_PATH: u32 = 0x00F4_0430;
    const SWITCHER: u32 = 0x00F2_0024;
    const RISE_MASK: u32 = 0xF << 20;
    const FALL_MASK: u32 = 0xF << 16;

    let ana_dec = match radio.rd_reg(ADC_CTRL).await {
        Ok(v) => (v >> 8) & 0x7,
        Err(_) => return,
    };
    let is_rx_hf = match radio.rd_reg(RX_PATH).await {
        Ok(v) => (v & 0x3) == 1,
        Err(_) => return,
    };

    let (rise, fall) = if !is_rx_hf && (ana_dec == 1 || ana_dec == 2) { (11u32, 13u32) } else { (15u32, 15u32) };
    let _ = radio.wr_reg_mask(SWITCHER, RISE_MASK, rise << 20).await;
    let _ = radio.wr_reg_mask(SWITCHER, FALL_MASK, fall << 16).await;
}
