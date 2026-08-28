//! **FLRC airtime, and the granularity this node can place a transmit at.** Pure integer
//! arithmetic, no MCU and no radio, so both numbers are testable on the host.
//!
//! Two wire fields are computed here and nowhere else:
//!
//! * [`EVT_TX_STARTED`](crate::serial::EVT_TX_STARTED)`[airtime_ms]` — the host turns it into a
//!   transmit deadline, so it must never be *under*-stated;
//! * [`EVT_CAP`](crate::serial::EVT_CAP)`.sched_gran_ns` — a slot scheduler sizes its guard band
//!   from it, so it must never be under-stated either.
//!
//! Both are therefore computed with **ceiling** arithmetic and, where a term is not measured, an
//! explicitly-declared margin. Over-stating either is safe; under-stating either is a lie the
//! planner acts on.
//!
//! Kept out of [`crate::flrc_link`] deliberately: that module pulls in `embedded-hal` and the
//! `lr2021` driver, so it only builds for `thumbv8m.main-none-eabihf` and cannot be tested. This
//! module takes the FLRC parameters as the **wire's own codes** (the `[rate_code, _, cr_code]` of
//! `CMD_SET_MOD`), which is what the bridge already has in hand, so nothing here needs the driver's
//! enums.

/// Ticks per microsecond of the MAC clock — **mirrored** from [`crate::timing::TICKS_PER_US`], which
/// cannot be used here because that module is device-only. `timing` carries a `const` assertion that
/// the two agree, so this cannot silently drift; changing the timer frequency in one place breaks
/// the build rather than quietly halving a reported granularity.
pub const MAC_TICKS_PER_US: u32 = 16;

/// Nanoseconds per MAC-clock tick, **rounded up** (16 MHz ⇒ 62.5 ns ⇒ 63).
///
/// [`crate::timing::TICK_NS`] truncates to 62 because it is a display figure; a granularity must
/// round the other way.
pub const SCHED_TICK_NS: u32 = (1000 + MAC_TICKS_PER_US - 1) / MAC_TICKS_PER_US;

/// The `SetTx` command on the wire to the radio: 5 bytes (`set_tx_adv_cmd`) at the 8 MHz SPI clock
/// [`crate::board::SPI_FREQ_HZ`] ⇒ 40 bits × 125 ns.
pub const SCHED_SPI_NS: u32 = 5 * 8 * 1000 / (crate::board::SPI_FREQ_HZ / 1_000_000);

/// PA ramp, `RampTime::Ramp2u` — the shortest the part offers, and what
/// [`crate::flrc_link`]`::ramp_time` defaults to. It sits between key-up and the first on-air symbol.
pub const SCHED_RAMP_NS: u32 = 2_000;

/// **The one measured MCU-side latency this board has**: M4's software path, DIO edge → the CPU's
/// own capture, 494 ticks = 30.9 µs, constant to within one 62.5 ns tick over 500 frames.
///
/// Used here as the stand-in for the parts of a CPU-mediated key-up that have *not* been measured —
/// EasyDMA start, the SPIM completion interrupt, the executor wake, and the driver's BUSY handshake.
/// They are the same class of latency (peripheral IRQ → embassy task resumes) on the same executor,
/// so borrowing the measured figure is a bound with a provenance rather than a number invented to
/// fill a field.
pub const SCHED_MCU_NS: u32 = 30_900;

/// **`EVT_CAP.sched_gran_ns` for this node: 50 µs.**
///
/// ```text
///   timer tick, ceil(1e9 / 16 MHz)                            SCHED_TICK_NS  =     63 ns
///   SetTx over SPI, 5 bytes @ 8 MHz                            SCHED_SPI_NS  =  5_000 ns
///   PA ramp, RampTime::Ramp2u                                 SCHED_RAMP_NS  =  2_000 ns
///   MCU reaction, M4-measured software path (30.9 µs)          SCHED_MCU_NS  = 30_900 ns
///                                                                              ---------
///                                                             sum            = 37_963 ns
///   declared                                                                 = 50_000 ns
/// ```
///
/// The declared figure is the sum rounded **up** to a round 50 µs, ~32% of margin. That direction is
/// deliberate: this is the CPU-mediated path (a TIMER compare polled by the CPU, then `SetTx` over
/// SPI), not the DPPI path `m5_tx` demonstrates, and the difference between them is exactly the four
/// terms above. Under-stating it would tell a slot scheduler it can pack slots this node cannot hit.
///
/// ★ **This describes the ABSOLUTE path, [`CMD_TX_AT_ABS`](crate::serial::CMD_TX_AT_ABS) — and only
/// that path can hit it.** Every term above is measured or bounded on the *node* side of the serial
/// link, which is exactly what an absolute target isolates: the host names an instant on the same
/// free-running counter `CMD_READ_CLOCK` reports, the firmware waits for that instant, and the
/// host→device latency lands *before* the deadline where it costs nothing.
///
/// The **relative** opcode [`CMD_TX_AT`](crate::serial::CMD_TX_AT) cannot, and the difference is
/// measured. Its `delay_us` is counted from the moment the firmware *processes the arm*, so the
/// serial round trip is inside the placement:
///
/// ```text
///   absolute-boundary slot train, 45/45 fired
///     accuracy   mean gap 2,399,818 ticks vs 2,400,000 nominal  -> 11 µs over 44 slots
///     jitter     sd 553 µs, p2p 1875 µs                         -> vs 50 µs declared here
///   the same node's CMD_GET_INFO round trip                      p2p 550 µs
/// ```
///
/// The two 550 µs figures are the same number, and that is the finding: as exercised, host-armed
/// *relative* scheduling is **worse** than the software path (sd 553 µs vs 155 µs) because it pays an
/// extra round trip for nothing. So the achievable placement of the relative path is bounded by the
/// serial round-trip spread — **p2p 550 µs measured on this node** — not by the granularity below,
/// and a host that needs the granularity must use the absolute opcode. `CMD_TX_AT` is kept because
/// it is still the right primitive for a delay the *firmware* computes, where no serial hop exists.
///
/// ★ **MEASURED 2026-08-28 — and the derived 50 µs was ~15× conservative.** The run this doc asked
/// for now exists: a `CMD_TX_AT_ABS` train, 45 slots of 150 ms, o5p-0 → o5p-1, with the placement
/// residual taken off the hardware RX stamp as `delta − round(delta/slot)·slot`, so a dropped frame
/// contributes the same residual as a kept one:
///
/// ```text
///   fired 45/45 · mean residual −0.2 µs · sd 0.7 µs · p2p 3.4 µs
/// ```
///
/// The terms above are a fixed LEAD, not jitter, and this firmware already compensates for them —
/// which is why the mean lands at −0.2 µs and not at −38 µs. What a slot guard band is built from is
/// the SPREAD, and that is 3.4 µs peak-to-peak. Declared at **10 µs**: ~3× the measured p2p, so it
/// still absorbs a worse day, without costing a scheduler the order of magnitude the derivation did.
///
/// (The inter-node clock difference — +16.7 ppm, i.e. 2.5 µs over a 150 ms slot — lands in the MEAN,
/// not the spread, so it does not inflate this. The receiver's own stamp precision is 62.5 ns.)
///
/// ⚠ This is the ABSOLUTE path's figure and only that. The relative `CMD_TX_AT` cannot reach it from
/// a host, for the reason in the note above.
pub const SCHED_GRAN_NS: u32 = 10_000;

/// **The relative path's achievable placement, MEASURED: p2p 550 µs.**
///
/// Not a granularity and deliberately not reported as one — [`SCHED_GRAN_NS`] is what `EVT_CAP`
/// carries, and it is true of [`CMD_TX_AT_ABS`](crate::serial::CMD_TX_AT_ABS). This constant exists
/// so the number that bounds the *relative* opcode is written down next to the one it is 11× larger
/// than, rather than living only in a commit message.
pub const RELATIVE_SCHED_SPREAD_NS: u32 = 550_000;

// ── FLRC AGC preamble ───────────────────────────────────────────────────────────────────────────
//
// The mapping lives here, in codes rather than in the driver's `AgcPblLen`, for one reason: it is
// what `CMD_SET_PREAMBLE` does to a number a host chose, so it is a wire contract, and a wire
// contract that only compiles for `thumbv8m` is a wire contract with no tests.

/// Length in **bits** of an FLRC AGC-preamble code. The register counts 4-bit steps: code 0 is
/// `AgcPblLen::Len4Bits`, code 7 is `Len32Bits`.
pub const fn preamble_bits_of_code(code: u8) -> u32 {
    (code as u32 + 1) * 4
}

/// The lowest preamble code that is **at least** `bits` long, or `None` if the register cannot
/// reach it.
///
/// Rounds **up** to the 4-bit step: a caller asking for more AGC settling than a step boundary gets
/// the next step up, never the one below — under-running the AGC is what makes a receiver miss
/// frames, and it does so silently.
///
/// Outside 4..32 the answer is `None` rather than a clamp. A host sending LoRa's typical 8-*symbol*
/// preamble is asking for something this radio does not have; 8 bits is a different thing, and
/// answering `EVT_UNSUPPORTED` is how it finds that out. Clamping would leave both ends configured
/// differently with nothing on the wire to say so.
pub const fn preamble_code_for_bits(bits: u16) -> Option<u8> {
    if bits < 4 || bits > 32 {
        return None;
    }
    Some(((bits as u32 + 3) / 4 - 1) as u8)
}

/// Raw channel bitrate, **kbit/s**, for the FLRC rung codes 0..7 that `CMD_SET_MOD` carries —
/// `FlrcBitrate::{Br2600, Br2080, Br1300, Br1040, Br0650, Br0520, Br0325, Br0260}`, which are the
/// chip's own encoding (`vendor/lr2021/src/cmd/cmd_flrc.rs`). The names *are* the bitrates.
pub const BITRATE_KBPS: [u32; 8] = [2600, 2080, 1300, 1040, 650, 520, 325, 260];

/// Channel bits per data bit for the FLRC coding-rate codes 0..3 —
/// `FlrcCr::{Cr12, Cr34, None, Cr23}` — as an exact `(numerator, denominator)` so the arithmetic
/// stays integral: rate 1/2 sends 2 channel bits per data bit, 3/4 sends 4/3, `None` sends 1, and
/// 2/3 sends 3/2.
///
/// Note the ordering: code `1` is 3/4 and code `2` is **FEC off**. Counter-intuitive, and it is what
/// the silicon uses, so it is what the wire uses.
pub const CR_EXPANSION: [(u32, u32); 4] = [(2, 1), (4, 3), (1, 1), (3, 2)];

/// **Airtime of one on-air FLRC frame, in microseconds, rounded up.**
///
/// ```text
///   [ AGC preamble ][ syncword ][ payload ][ CRC ]
///     \___ raw bitrate ______/   \__ FEC-expanded, then raw bitrate __/
/// ```
///
/// The preamble and syncword are not coded — they are what the receiver's AGC and correlator run on,
/// so they go out at the raw channel rate. Everything after the syncword passes through the
/// convolutional encoder first, which is why `cr_code` belongs in the second term only.
///
/// `pld_bytes` is the **whole fixed on-air PDU** (`flrc_link::FRAME_LEN`), not the caller's payload:
/// with `PktFormat::Fixed` the chip transmits exactly `pld_len` bytes whatever the FIFO holds, so a
/// 3-byte NDN Interest occupies precisely as much air as a 47-byte one. Constant airtime per frame is
/// the property the slot MAC wants, and computing it from the caller's length would under-state every
/// short frame's deadline.
///
/// With `PktFormat::Fixed` there is no length header on air, so no header term appears here. Returns
/// 0 for an unknown code rather than a fabricated duration — the caller has already validated both
/// codes against the same tables.
pub const fn airtime_us(rate_code: u8, cr_code: u8, preamble_bits: u32, sync_bits: u32, pld_bytes: u32, crc_bytes: u32) -> u32 {
    if rate_code as usize >= BITRATE_KBPS.len() || cr_code as usize >= CR_EXPANSION.len() {
        return 0;
    }
    let kbps = BITRATE_KBPS[rate_code as usize];
    let (num, den) = CR_EXPANSION[cr_code as usize];
    let data_bits = (pld_bytes + crc_bytes) * 8;
    // Ceiling on the expansion too: a 3/4 code cannot emit a fractional channel bit.
    let coded_bits = (data_bits * num + den - 1) / den;
    let bits = preamble_bits + sync_bits + coded_bits;
    // bits / (kbit/s) = milliseconds; ×1000 for µs. Ceiling.
    (bits * 1000 + kbps - 1) / kbps
}

// ── LoRa airtime ────────────────────────────────────────────────────────────────────────────────

/// **Airtime of one LoRa frame, in microseconds** — the standard Semtech time-on-air formula,
/// explicit header + CRC on, in the same integer arithmetic the Waveshare (`sx1262::airtime_ms`) and
/// Heltec (`airtime_ms`) nodes use.
///
/// ```text
///   payloadSymbNb = 8 + ceil( (8*PL - 4*SF + 28 + 16) / (4*(SF - 2*DE)) ) * (CR + 4)
///   Tsym          = 2^SF / BW
///   t             = Tsym * (4*preamble + 17)/4        <- preamble, including the 4.25-symbol sync
///                 + Tsym * payloadSymbNb
/// ```
///
/// Shared arithmetic with the other two nodes on purpose: `EVT_TX_STARTED` is how a host sizes its
/// reply deadline, and three nodes that computed airtime three ways would make a cross-node slot
/// comparison meaningless. `DE` is the low-data-rate optimisation, which the LR2021 driver turns on
/// under exactly the condition used here (`SF >= 11` at 125 kHz, and `LoraModulationParams::basic`
/// widens it to SF11 at 250 kHz too — the narrower rule over-states the symbol count slightly, and
/// over-stating is the safe direction for a deadline).
///
/// `cr` is the fleet's 1..4 (= 4/5..4/8), which is also the LR2021's own `LoraCr` 1..4 for the
/// short-interleaved codes. Returns 0 for arguments outside the ranges the caller has already
/// validated, rather than a fabricated duration.
pub const fn lora_airtime_us(sf: u8, bw_hz: u32, cr: u8, payload_len: u16, preamble_syms: u16) -> u32 {
    if sf < 5 || sf > 12 || cr < 1 || cr > 4 || bw_hz == 0 {
        return 0;
    }
    let sf_i = sf as i64;
    let de: i64 = if sf >= 11 && bw_hz == 125_000 { 1 } else { 0 };
    let pl = payload_len as i64;
    let num = 8 * pl - 4 * sf_i + 28 + 16;
    let den = 4 * (sf_i - 2 * de);
    let steps = if num <= 0 || den <= 0 {
        0
    } else {
        (num + den - 1) / den
    };
    let payload_sym = 8 + steps * (cr as i64 + 4);
    // Tsym in µs. The 2^SF numerator is exact for SF<=12 and the division truncates, which
    // under-states Tsym by <1 µs — recovered many times over by the ceiling on the total below.
    let tsym_us = ((1u64 << sf) * 1_000_000) / bw_hz as u64;
    let preamble_us = tsym_us * (4 * preamble_syms as u64 + 17) / 4;
    let payload_us = tsym_us * payload_sym as u64;
    let total = preamble_us + payload_us + 1; // +1 µs: never report 0 for a real frame
    if total > u32::MAX as u64 {
        u32::MAX
    } else {
        total as u32
    }
}

// ── LR-FHSS airtime ─────────────────────────────────────────────────────────────────────────────

/// LR-FHSS physical modulation rate: **488.28125 bit/s**, i.e. exactly `1e6 / 2048` — so one
/// physical bit is **2048 µs**. Expressed as the period rather than the rate because the period is
/// the exact integer and the rate is not.
///
/// This number is also the mechanism behind §17.1's "transmit-only" claim: Table 11-2 gives the
/// generic (G)FSK modem a **bitrate minimum of 500 bps**, and LR-FHSS sits 2.4% under it. See
/// [`crate::lrfhss_link`].
pub const LRFHSS_BIT_US: u32 = 2048;

/// Bits in one LR-FHSS **sync header**, and the bits of a hopping block: the frame is a run of
/// header replicas followed by the coded payload cut into fragments, each fragment carrying a
/// 2-bit block preamble.
///
/// ⚠ **Sourced from Semtech's `lr_fhss_mac.c` frame construction, and NOT verified against this
/// chip.** They are here because the alternative is worse: `EVT_TX_STARTED` and the TX watchdog both
/// need an airtime, an LR-FHSS frame is *seconds* long, and reporting the FLRC-scale default would
/// abort every transmit before it finished. The direction of any error is bounded on the safe side
/// by the caller — the TX watchdog uses **2× this figure plus a fixed floor** — so a wrong constant
/// degrades into a longer wait, never into a truncated transmit. Verify by timing TxDone on air.
pub const LRFHSS_HEADER_BITS: u32 = 114;
/// See [`LRFHSS_HEADER_BITS`]. Payload fragment length, in bits.
pub const LRFHSS_FRAG_BITS: u32 = 48;
/// See [`LRFHSS_HEADER_BITS`]. Block preamble prepended to each fragment, in bits.
pub const LRFHSS_BLOCK_PREAMBLE_BITS: u32 = 2;

/// **How many hopping blocks an LR-FHSS frame is cut into** — the `nb_hopping_blocks` argument of
/// `WriteLrFhssHoppingTable`, derived from the same block arithmetic as
/// [`lrfhss_airtime_us`] so the table the host writes and the duration the host is told cannot
/// describe different frames.
///
/// `cr` is the chip's `LrfhssCr` code. Returns 0 for an unknown coding rate.
pub const fn lrfhss_block_count(cr: u8, payload_len: u16) -> u16 {
    let length_bits = (payload_len as u32 + 2) * 8 + 6;
    let coded_bits = match cr {
        0 => (length_bits * 6 + 4) / 5,
        1 => (length_bits * 3 + 1) / 2,
        2 => length_bits * 2,
        3 => length_bits * 3,
        _ => return 0,
    };
    let whole = coded_bits / LRFHSS_FRAG_BITS;
    let rest = coded_bits % LRFHSS_FRAG_BITS;
    let n = whole + if rest != 0 { 1 } else { 0 };
    if n > u16::MAX as u32 {
        u16::MAX
    } else {
        n as u16
    }
}

/// **Airtime of one LR-FHSS frame, in microseconds.** `cr` is the chip's own `LrfhssCr` code
/// (0 = 5/6, 1 = 2/3, 2 = 1/2, 3 = 1/3) and `sync_headers` is the frame's header replica count 1..4.
///
/// ```text
///   length_bits = (payload + 2 CRC) * 8 + 6            physical-layer trailer
///   coded_bits  = length_bits expanded by the coding rate
///   payload_bits= whole fragments * (48 + 2) + any remainder * (remainder + 2)
///   t           = (sync_headers * 114 + payload_bits) * 2048 µs
/// ```
///
/// See [`LRFHSS_HEADER_BITS`] for the provenance of the constants and why an unverified model is
/// nonetheless better than none here. Returns 0 for an out-of-range coding rate.
pub const fn lrfhss_airtime_us(cr: u8, sync_headers: u8, payload_len: u16) -> u32 {
    let length_bits = (payload_len as u32 + 2) * 8 + 6;
    let coded_bits = match cr {
        0 => (length_bits * 6 + 4) / 5, // 5/6
        1 => (length_bits * 3 + 1) / 2, // 2/3
        2 => length_bits * 2,           // 1/2
        3 => length_bits * 3,           // 1/3
        _ => return 0,
    };
    let whole = coded_bits / LRFHSS_FRAG_BITS;
    let rest = coded_bits % LRFHSS_FRAG_BITS;
    let mut payload_bits = whole * (LRFHSS_FRAG_BITS + LRFHSS_BLOCK_PREAMBLE_BITS);
    if rest != 0 {
        payload_bits += rest + LRFHSS_BLOCK_PREAMBLE_BITS;
    }
    let hdrs = if sync_headers == 0 { 1 } else { sync_headers as u32 };
    let bits = hdrs * LRFHSS_HEADER_BITS + payload_bits;
    // At 2048 µs/bit a 247-byte payload at CR 1/3 is ~13 s, which still fits a u32 of µs (~71 min).
    bits.saturating_mul(LRFHSS_BIT_US)
}

/// The same airtime as whole **milliseconds, rounded up, never zero** — the unit
/// [`EVT_TX_STARTED`](crate::serial::EVT_TX_STARTED) carries fleet-wide.
///
/// Ceiling and the non-zero floor are both load-bearing. An FLRC frame is *sub-millisecond* at every
/// rung above 260 kbit/s (a 48-byte PDU at 2.6 Mbit/s is ~213 µs), so truncating would report **0 ms**
/// for the node's entire normal operating range, and a host deriving a deadline from 0 gets a
/// deadline that has already passed. The field's resolution simply cannot express this bearer; the
/// only honest way to lose that precision is upwards.
pub const fn airtime_ms_ceil(us: u32) -> u16 {
    let ms = (us + 999) / 1000;
    let ms = if ms == 0 { 1 } else { ms };
    if ms > u16::MAX as u32 { u16::MAX } else { ms as u16 }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// ★ **This invariant was RETIRED by measurement, and the reason matters more than the number.**
    ///
    /// It used to assert `SCHED_GRAN_NS >= SCHED_TICK + SPI + RAMP + MCU`, on the theory that the
    /// declared granularity must cover its components. That theory was wrong: those four terms are a
    /// fixed **LEAD** before key-up, not jitter, and the firmware compensates for them — the measured
    /// mean placement residual is −0.2 µs, not −38 µs. Summing a lead into a granularity over-states
    /// it, and here it did so by ~15×.
    ///
    /// A guard band is built from the SPREAD. MEASURED on the absolute path (45 slots of 150 ms,
    /// residual off the hardware RX stamp): sd 0.7 µs, **p2p 3.4 µs**. So the declared value is now
    /// pinned against the measurement with headroom, and the component sum is kept only as
    /// documentation of the lead.
    #[test]
    fn sched_granularity_is_pinned_to_the_measurement_not_the_lead() {
        let lead = SCHED_TICK_NS + SCHED_SPI_NS + SCHED_RAMP_NS + SCHED_MCU_NS;
        assert_eq!(lead, 37_963, "the arithmetic in the SCHED_GRAN_NS doc comment");
        // MEASURED p2p 3.4 µs. The declaration must cover it with margin, and must NOT drift back up
        // to the lead — that is the regression this now guards.
        const MEASURED_P2P_NS: u32 = 3_400;
        assert!(
            SCHED_GRAN_NS >= MEASURED_P2P_NS,
            "declared granularity is under the MEASURED spread"
        );
        assert!(
            SCHED_GRAN_NS < lead,
            "granularity has drifted back to the fixed lead; the lead is compensated, not jitter"
        );
        // Pin the components so a change to any one of them shows up here rather than silently
        // rebalancing the margin.
        assert_eq!(SCHED_TICK_NS, 63); // 62.5 ns rounded UP, unlike timing::TICK_NS
        assert_eq!(SCHED_SPI_NS, 5_000);
        assert_eq!(SCHED_GRAN_NS, 10_000);
    }

    /// The default link: 48-byte fixed PDU, 32-bit AGC preamble, 32-bit syncword, 2-byte CRC.
    ///
    /// At Br2600/Cr34: data = (48+2)×8 = 400 bits → ×4/3 = 534 coded bits → +64 = 598 bits
    /// → 598 000 / 2600 = 230.0 µs.
    #[test]
    fn airtime_of_the_default_frame() {
        assert_eq!(airtime_us(0, 1, 32, 32, 48, 2), 230);
        // …and the fleet's millisecond field can only round it up.
        assert_eq!(airtime_ms_ceil(230), 1);
    }

    /// Rung and coding both move it, in the direction and roughly the ratio they should.
    #[test]
    fn airtime_tracks_rung_and_coding() {
        let fast = airtime_us(0, 2, 32, 32, 48, 2); // 2.6 Mbit/s, FEC off
        let slow = airtime_us(7, 2, 32, 32, 48, 2); // 260 kbit/s, FEC off
        assert_eq!(fast, 179); // (400 + 64) bits / 2600 kbit/s
        assert_eq!(slow, 1785); // exactly 10× the rate ⇒ ~10× the time
        assert!(slow > fast * 9);

        // FEC off vs rate 1/2 at the same rung: the payload term doubles, the preamble does not.
        let half = airtime_us(0, 0, 32, 32, 48, 2);
        assert_eq!(half, 333); // (800 + 64) / 2600
        assert!(half > fast && half < fast * 2);

        // The slowest rung with the heaviest coding is the only combination that exceeds 1 ms and so
        // the only one the fleet's millisecond field can express with any resolution at all.
        assert_eq!(airtime_ms_ceil(airtime_us(7, 0, 32, 32, 48, 2)), 4);
    }

    /// An unknown code returns 0 — the caller validates first, and a fabricated duration would be
    /// worse than an obviously-absent one. `airtime_ms_ceil` still refuses to hand a host a zero
    /// deadline.
    #[test]
    fn unknown_codes_do_not_invent_a_duration() {
        assert_eq!(airtime_us(8, 1, 32, 32, 48, 2), 0);
        assert_eq!(airtime_us(0, 4, 32, 32, 48, 2), 0);
        assert_eq!(airtime_ms_ceil(0), 1);
    }

    /// The preamble mapping is round-trip exact on the step boundaries and rounds **up** between
    /// them. The direction is the whole point: a preamble one step short of what was asked for makes
    /// a receiver miss frames and says nothing about why.
    #[test]
    fn preamble_maps_on_steps_and_rounds_up() {
        for code in 0u8..8 {
            let bits = preamble_bits_of_code(code);
            assert_eq!(preamble_code_for_bits(bits as u16), Some(code));
        }
        assert_eq!(preamble_bits_of_code(0), 4);
        assert_eq!(preamble_bits_of_code(7), 32);
        // Between steps: 5..8 all land on the 8-bit step.
        for bits in 5u16..=8 {
            assert_eq!(preamble_code_for_bits(bits), Some(1));
        }
        assert_eq!(preamble_code_for_bits(29), Some(7));
    }

    /// Out of the register's reach is refused, not clamped — including LoRa's typical 8-**symbol**
    /// preamble expressed as a bigger number, and the 0 a zeroed struct would produce.
    #[test]
    fn preamble_outside_the_register_is_refused() {
        assert_eq!(preamble_code_for_bits(0), None);
        assert_eq!(preamble_code_for_bits(3), None);
        assert_eq!(preamble_code_for_bits(33), None);
        assert_eq!(preamble_code_for_bits(1024), None);
    }

    /// **LoRa airtime must agree with the other two nodes' arithmetic**, because a host compares
    /// them. Reference values recomputed from the Semtech formula by hand:
    ///
    /// SF7/125 kHz/CR 4/5, 8-symbol preamble, 20-byte payload:
    ///   num = 8*20 − 4*7 + 44 = 176; den = 4*7 = 28; steps = ceil(176/28) = 7
    ///   payloadSymbNb = 8 + 7*5 = 43; Tsym = 128*1e6/125000 = 1024 µs
    ///   preamble = 1024*(32+17)/4 = 12544; payload = 1024*43 = 44032 ⇒ 56 576 µs (+1)
    #[test]
    fn lora_airtime_matches_the_semtech_formula() {
        assert_eq!(lora_airtime_us(7, 125_000, 1, 20, 8), 56_577);
        assert_eq!(airtime_ms_ceil(lora_airtime_us(7, 125_000, 1, 20, 8)), 57);
        // Doubling the bandwidth halves the time; raising SF by one roughly doubles it.
        let base = lora_airtime_us(7, 125_000, 1, 20, 8);
        let wide = lora_airtime_us(7, 250_000, 1, 20, 8);
        let slow = lora_airtime_us(8, 125_000, 1, 20, 8);
        assert!(wide * 2 >= base - 2 && wide * 2 <= base + 2);
        assert!(slow > base * 3 / 2 && slow < base * 5 / 2);
        // Heavier coding lengthens it; a longer preamble lengthens it.
        assert!(lora_airtime_us(7, 125_000, 4, 20, 8) > base);
        assert!(lora_airtime_us(7, 125_000, 1, 20, 16) > base);
        // The LDRO condition is SF>=11 at 125 kHz, and it costs symbols.
        assert!(lora_airtime_us(11, 125_000, 1, 20, 8) > lora_airtime_us(11, 250_000, 1, 20, 8));
    }

    /// **This bearer is why `airtime_ms_ceil` refuses to return 0, and why it needs a u16.** LoRa at
    /// SF12/125 kHz with a full 247-byte payload is over ten seconds, three orders of magnitude from
    /// FLRC — the same wire field has to carry both.
    #[test]
    fn lora_airtime_spans_three_orders_of_magnitude_from_flrc() {
        let flrc = airtime_us(0, 1, 32, 32, 48, 2);
        let lora_max = lora_airtime_us(12, 125_000, 4, 247, 8);
        assert!(lora_max > 10_000_000, "{lora_max}");
        assert!(lora_max / flrc > 1000);
        // …and it still fits the fleet's u16 millisecond field.
        assert!(airtime_ms_ceil(lora_max) < u16::MAX);
    }

    /// Arguments outside what the caller has validated give 0, never an invented duration.
    #[test]
    fn lora_airtime_refuses_impossible_arguments() {
        assert_eq!(lora_airtime_us(4, 125_000, 1, 20, 8), 0);
        assert_eq!(lora_airtime_us(13, 125_000, 1, 20, 8), 0);
        assert_eq!(lora_airtime_us(7, 125_000, 0, 20, 8), 0);
        assert_eq!(lora_airtime_us(7, 125_000, 5, 20, 8), 0);
        assert_eq!(lora_airtime_us(7, 0, 1, 20, 8), 0);
        // …and the millisecond field still refuses to hand a host a zero deadline.
        assert_eq!(airtime_ms_ceil(0), 1);
    }

    /// LR-FHSS is a **seconds-scale** bearer, which is the whole reason its airtime cannot be left
    /// at the FLRC default: a 20 ms TX watchdog would abort every frame.
    ///
    /// Worked by hand at CR 1/3, 3 sync headers, 50-byte payload:
    ///   length_bits = 52*8 + 6 = 422 → ×3 = 1266 coded bits
    ///   whole = 1266/48 = 26, rest = 1266 − 26*48 = 18
    ///   payload_bits = 26*50 + (18+2) = 1320; +3*114 = 1662 bits ⇒ ×2048 µs = 3 403 776 µs
    #[test]
    fn lrfhss_airtime_is_seconds_not_milliseconds() {
        assert_eq!(lrfhss_airtime_us(3, 3, 50), 3_403_776);
        assert_eq!(airtime_ms_ceil(lrfhss_airtime_us(3, 3, 50)), 3404);
        // Lighter coding is faster; more header replicas cost exactly 114 bits each.
        assert!(lrfhss_airtime_us(0, 3, 50) < lrfhss_airtime_us(3, 3, 50));
        assert_eq!(
            lrfhss_airtime_us(3, 4, 50) - lrfhss_airtime_us(3, 3, 50),
            LRFHSS_HEADER_BITS * LRFHSS_BIT_US
        );
        // Even an empty payload is ~1 s: the trailer and the headers alone dominate.
        assert!(lrfhss_airtime_us(3, 3, 0) > 500_000);
        // An unknown coding rate invents nothing.
        assert_eq!(lrfhss_airtime_us(4, 3, 50), 0);
        // The block count is the same cut of the same frame the duration is computed from.
        assert_eq!(lrfhss_block_count(3, 50), 27); // 26 whole 48-bit fragments + an 18-bit tail
        assert_eq!(lrfhss_block_count(4, 50), 0);
        assert!(lrfhss_block_count(0, 50) < lrfhss_block_count(3, 50));
        // The full payload is many seconds and still inside a u32 of microseconds.
        assert!(lrfhss_airtime_us(3, 4, 247) > 10_000_000);
        assert!(lrfhss_airtime_us(3, 4, 247) < u32::MAX);
    }

    /// One physical bit is exactly 2048 µs, and that period is the same number §17.1's "transmit
    /// only" claim rests on: 1e6/2048 = 488.28125 bit/s, which is 2.4% **under** the (G)FSK modem's
    /// 500 bps floor in Table 11-2.
    #[test]
    fn lrfhss_bit_period_is_the_datasheets_own_rate() {
        assert_eq!(LRFHSS_BIT_US, 2048);
        // 1e6 µs/s ÷ 2048 µs/bit = 488.28125 bit/s, checked in the only integer form that is exact.
        assert_eq!(100_000_000u64 / LRFHSS_BIT_US as u64, 48_828);
        assert!(1_000_000 / LRFHSS_BIT_US < 500, "LR-FHSS sits under the 500 bps GFSK floor");
    }
}
