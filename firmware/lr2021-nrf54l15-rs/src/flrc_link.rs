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
use lr2021::system::{ChipMode, DioNum, TcxoVoltage};
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
    Some(_) => 2_477_000_000,
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

    radio.calib_fe(&[FE_CAL]).await?;

    radio.set_packet_type(PacketType::Flrc).await?;
    radio.set_flrc_modulation(BITRATE, CODING, PULSE_SHAPE).await?;

    // One syncword, and RX matches only that one: this is a two-node experiment, and accepting
    // other syncwords would let stray traffic masquerade as our packets.
    // `is_16b = FALSE`. This is a footgun and the crate's own doc example gets it wrong
    // (`set_flrc_syncword(1, 0xCD05CAFE, true)` alongside `SwLen::Sw32b`), which is where the bug
    // was copied from. With `true` the driver does `syncword << 16` — DISCARDING the high half —
    // and truncates the command to two syncword bytes, so a 32-bit value silently becomes a 16-bit
    // one while the packet params still declare `Sw32b`. Both ends misconfigure identically, so they
    // still sync (`sw_num = 1`) and the length still decodes — and every frame fails CRC, because
    // the frame is delimited differently from the region the CRC covers.
    radio.set_flrc_syncword(1, SYNCWORD, false).await?;

    // Defaults to the RX role; a transmitter MUST call `set_payload_len` with the real frame size.
    radio.set_flrc_packet(&pkt_params(MAX_PAYLOAD)).await?;

    // HF front end for the 2.4 GHz port: PA on the HF path, RX on the HF path with full boost
    // (rx-boost-cfg = 7 in the shield devicetree).
    if option_env!("PHY_HF").is_none() {
        // LF PA: the crate's HF helper hard-codes the HF PA selection, so the sub-GHz path needs the
        // explicit form. Duty cycle / slices copied from the HF helper's own defaults (6, 7).
        radio.set_pa_lf(PaLfMode::LfPaFsm, 6, 7).await?;
    } else {
        radio.set_pa_hf().await?;
    }
    // **Ramp2u, not Ramp16u** (#104 lever 3). The PA ramp sits between the TX trigger and the first
    // on-air symbol, so its duration is pure transmit-instant offset — and any variation in it is
    // transmit-instant jitter, which is exactly the residual M5 left unexplained. 16 µs was an
    // arbitrary bring-up default; the shortest ramp is the right default for a slot MAC.
    radio.set_tx_params(TX_POWER_DBM, RampTime::Ramp2u).await?;
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
    radio.set_rx_path(path, RxBoost::Off).await?;
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
