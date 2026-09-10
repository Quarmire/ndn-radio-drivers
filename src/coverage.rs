//! **The backend coverage table** (#79 / plan P3) — every radio shows a full row or a written
//! exclusion; there is no third state.
//!
//! The defect this closes: trait coverage across backends was ragged and *silent* — the rig's
//! highest-throughput radio (mt7612) declared no clock and was invisible to the time plane, and
//! nothing named that gap. A missing impl must be a visible row with a reason, not an absent
//! `None` discovered mid-campaign.
//!
//! Two enforcement mechanisms, split by what each can honestly check:
//! * **`Provided` cells are compile-time-verified**: the test instantiates a trait-bound assertion
//!   for every claimed impl, so this table cannot claim a seam the code does not have. (It CAN
//!   still miss a seam the code has — Rust cannot prove a negative — which is why exclusions are
//!   prose with a reason, reviewed, not derived.)
//! * **`Excluded` cells carry the ruling**: why, decided when, and what would lift it. An
//!   exclusion with an empty reason fails the test.
//!
//! ⚠ **The gate is ONE-DIRECTIONAL, and knowing that is part of using this table.** It prevents
//! *over*-claiming (a `Provided` with no witness fails to compile) but cannot catch *under*-claiming:
//! an `Excluded` cell whose impl quietly lands later adds to neither the `Provided` count nor the
//! witness count, so the equality assertion stays satisfied and the stale exclusion survives. That
//! is not hypothetical — the AR9271 `knobs` cell sat `Excluded("no &self RadioKnobs yet")` for the
//! whole life of a seven-method `impl RadioKnobs for Ath9kHtcBackend`. When you add an impl, come
//! here and flip its cell; nothing else will.
//!
//! ⚠ **`Provided` carries no notion of DEPTH.** One method that errors on almost every input and
//! seven hardware-validated actuators both render as `Provided`. Read the row's backend before
//! reading its cells as a capability claim.
//!
//! **P3.12 ruling — rates for MAC campaigns are PINNED, not adaptive.** The plan offered
//! "RateCalibrator wired or rates pinned per-experiment". Pinned wins for every MAC experiment:
//! an adaptive rate mid-run is a confounder (the arm's airtime changes under it, and airtime is
//! the quantity every MAC property is stated in). The mechanism is `NDN_RADIO_TX_RATE` — a
//! classified Config variable printed in the run header by `ndn-env`, so the pin is part of the
//! run's self-description. `RateCalibrator` stays a cognition-loop concern for non-campaign
//! operation; wiring it into campaign tooling is deferred *with this paragraph as the reason*.

/// One seam's status for one backend: implemented, or excluded in writing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Seam {
    Provided,
    /// The written exclusion: why this seam is deliberately absent, and (where known) what would
    /// lift it. Empty reasons fail the coverage test.
    Excluded(&'static str),
}

/// One backend's row. `FrameIo` is the price of admission — a backend without it is not a radio
/// this crate can drive and appears with `frame_io: Excluded` and the reason.
pub struct Row {
    pub backend: &'static str,
    /// USB PID(s) this backend claims — referencing each backend's exported PID const where one
    /// exists, so this table cannot drift from the dispatch (a hand-typed copy here would be the
    /// exact silent divergence it polices).
    pub pids: &'static [u16],
    /// In the pre-registered campaign set (plan P5): a campaign radio must show a FULL row —
    /// every seam `Provided` — or be dropped from the campaign, not carried half-described.
    pub campaign: bool,
    pub frame_io: Seam,
    pub knobs: Seam,
    pub time: Seam,
    pub profile: Seam,
}

use Seam::{Excluded, Provided};

/// ★ **Does each backend honour `TxIntent::needs_basic_rate`?** (added 2026-09-01)
///
/// `MostRobust` means "the worst receiver in earshot must decode this" — cooperative reports,
/// discovery, control. An HT/VHT/HE PPDU excludes every receiver without that decoder *by
/// construction*, and this stack has MEASURED that as a real one-way link (drone→GCS perfect,
/// GCS→drone nothing but legacy 6M, because the peer transmitted 2-stream MCS9 in good faith).
///
/// The doctrine was documented four times, in four backends, one of which says "Same rule as every
/// other backend here" — and it was **absent from ten of fifteen**. `Rtl8733buBackend::inject` read
/// only its stored rate/flags and never looked at `frame.tx` at all, so on that radio the traffic
/// whose entire purpose is universal decodability went out at the last throughput rate. That is the
/// worst-receiver failure reintroduced by omission, in exactly the frames it exists to protect.
///
/// The ENCODING is necessarily per-backend (a Realtek DESC code, an mt76x02 TXWI word and a connac2
/// rate word are three different things); the DECISION is now one predicate, and this table records
/// who applies it.
pub enum TxIntentSupport {
    /// Forces the basic rate for a `MostRobust` frame. The string names the mechanism.
    Honoured(&'static str),
    /// Not applicable to this PHY, with the reason.
    NotApplicable(&'static str),
    /// A KNOWN GAP: this radio could honour it and does not. The reason must say what would lift it.
    Gap(&'static str),
}

/// One row per backend that transmits. Adding a backend without an entry fails the gate below.
pub const TX_INTENT: &[(&str, TxIntentSupport)] = &[
    (
        "LibUsbRtl88xxBackend",
        TxIntentSupport::Honoured(
            "inject forces DESC 0x04 (legacy OFDM 6M) and suppresses HT-only SGI/LDPC/STBC",
        ),
    ),
    (
        "Rtl8812auBackend",
        TxIntentSupport::Honoured(
            "desc_rate_for returns DESC_RATE_6M ahead of the stored descriptor",
        ),
    ),
    (
        "Rtl8733buBackend",
        TxIntentSupport::Honoured(
            "inject picks DESC_RATE_6M with flags cleared, instead of the stored tx_rate/tx_flags. ★ VERIFIED ON AIR 2026-09-01, THREE arms so the delivery reading is controlled: A MostRobust 354 @ 6.0 Mb/s legacy, B Throughput 2 @ 65.0 Mb/s MCS7, C Throughput 354 @ 6.5 Mb/s MCS0 (405 sent each). C is the discriminator: the witness decodes this DUT HT fine at MCS0, so B is real rate/margin loss and not a dead HT decoder — a two-arm run could not tell those apart",
        ),
    ),
    (
        "Rtl8821cuBackend",
        TxIntentSupport::Honoured(
            "build_tx forces DESC_RATE_OFDM6M where the DESC code is chosen, so a stored cur_mcs cannot bypass it. ⚠ NOT verifiable on air yet, and NOT for want of trying: after a replug the dongle mode-switched to 0bda:c820 and our bring-up ran, but 440+439 injected frames produced ZERO frames at a witness on the same host. That is this row knobs/time exclusion showing up on the TX side too — bring-up is validated RX-side only and no control write has been hardware-verified. Lifted by the same bench session that lifts those",
        ),
    ),
    (
        "Mt7610uBackend",
        TxIntentSupport::Honoured(
            "resolved_rate returns the legacy OFDM-6M TXWI word ahead of cur_rate",
        ),
    ),
    (
        "Mt7612uBackend",
        TxIntentSupport::Honoured(
            "inject builds at MT76_RATE_OFDM6M via build_data_bulk_at, with the width field clamped to 0 (legacy PPDUs carry no wide format). ★ VERIFIED ON AIR 2026-09-01: 391 @ 6.0 Mb/s vs 348 @ 65.0 Mb/s MCS7",
        ),
    ),
    (
        "Mt7921uBackend",
        TxIntentSupport::Honoured(
            "legacy OFDM 6M rate word, deliberately not for_intent's HE ER-SU branch",
        ),
    ),
    (
        "Ath9kHtcBackend",
        TxIntentSupport::Honoured(
            "build_tx_frame returns early at LegacyRate::Ofdm6 with rate_flags 0, ahead of both cur_legacy and cur_mcs. ★ VERIFIED ON AIR 2026-09-01: 65 @ 6.0 Mb/s vs 64 @ 65.0 Mb/s MCS7",
        ),
    ),
    (
        "SerialRadioBackend",
        TxIntentSupport::NotApplicable(
            "rate is device state set by a command over the serial link, not a per-frame field; honouring intent per frame would cost a round trip per frame",
        ),
    ),
    (
        "Bw16SerialBackend",
        TxIntentSupport::NotApplicable(
            "as SerialRadioBackend — rate is a command, not a per-frame field",
        ),
    ),
    (
        "Esp32SerialBackend",
        TxIntentSupport::NotApplicable(
            "as SerialRadioBackend — rate is a command, not a per-frame field",
        ),
    ),
    (
        "LoraSerialBackend",
        TxIntentSupport::NotApplicable(
            "LoRa has no basic rate; robustness is the spreading factor, which is channel state adapted by the LoRa phy, not a per-frame choice",
        ),
    ),
    (
        "MorseFrameIo",
        TxIntentSupport::Gap(
            "HaLow injects through mac80211 and does not name a rate; lifting this means writing the S1G MCS field into the injected radiotap header",
        ),
    ),
    (
        "Nrc7292FrameIo",
        TxIntentSupport::Gap(
            "as MorseFrameIo — would need an explicit rate in the injected radiotap header",
        ),
    ),
];

/// ★ **How each backend makes its contention posture DETERMINISTIC** (added 2026-08-31).
///
/// USB never power-cycles a dongle between processes, so any MAC state a bring-up does not
/// establish is simply **inherited from whatever ran before**. MEASURED on the MT7610U across five
/// consecutive processes: a run that set no posture returned **2724 or 6706 f/s — a 2.5x swing
/// decided purely by run order**. Every unpinned A/B on that radio had been comparing history.
///
/// The bug was fixed in the driver file where it was found. It then turned out the MT7921AU and
/// MT7612U had the identical hole — each with a purpose-built restore function that nothing called
/// (`restore_edca` had zero callers workspace-wide; `restore_edca_defaults` was used only by an
/// example). That is the recurring shape this table exists to break: a fix that lands in one
/// backend instead of in a checklist every backend answers.
///
/// A backend is clean by **either** mechanism — writing the posture, or power-cycling the MAC so
/// the registers return to reset defaults. Both are recorded, because which one it is determines
/// whether a future refactor (e.g. removing a redundant-looking power cycle) can reopen the hole.
pub enum Contention {
    /// Writes a known posture at bring-up. The string names the call that does it.
    Pinned(&'static str),
    /// Power-cycles the MAC at bring-up, so contention returns to hardware defaults.
    PowerCycled(&'static str),
    /// Cannot leak: this backend does not own the MAC arbiter.
    NotApplicable(&'static str),
    /// Not yet established. Non-empty reason required — an unreviewed backend is a known unknown,
    /// not a silent assumption.
    Unreviewed(&'static str),
}

/// One row per backend that owns a MAC. Adding a backend without an entry fails the gate below.
pub const CONTENTION: &[(&str, Contention)] = &[
    (
        "Mt7610uBackend",
        Contention::Pinned(
            "bring_up -> mt76::knobs::set_contention(Shared); the radio the 2.5x swing was measured on",
        ),
    ),
    (
        "Mt7921uBackend",
        Contention::Pinned(
            "bring_up -> restore_edca(); EDCA is MCU_CE_CMD firmware state no register read reveals. ⚠ 2026-09-01: pin is non-fatal and CONSISTENT with working (no-posture runs 2187/2334/2277 f/s across interleaved aggressive runs) but NOT proven — the contention lever measures only 5.6% on this part (cw_min exp 2 vs 5: 3619 vs 3426 f/s) against a 3.3% run spread, so a leak would look like noise. Underpowered, not passing. Needs a firmware EDCA read-back the MCU does not expose",
        ),
    ),
    (
        "Mt7612uBackend",
        Contention::Pinned(
            "bring_up -> restore_edca_defaults(); writes the boot window, never below it (mt76x2 window_floor hazard). ★ PROVEN AT THE REGISTERS 2026-09-01, both halves in one run: a prior process left NDN_POSTURE=yielding (AIFSN 0x3333 / CWMIN 0x6666 / AC0 0x000a6300), a fresh process READ THOSE BACK as-found (the leak is real), and bring_up restored 0x2222 / 0x4444 / 0x000a4200 (the pin works). ☠ The first version of this pin was INERT: it sat at the tail of the COLD path while the warm re-open returns early — and the warm path is exactly where a chip keeps the previous state. Both paths now call one pin_edca() helper. Note Owned CANNOT test this on mt76x2 (window_floor clamps it to the boot window, so Owned == Shared == boot); Yielding is the only posture that moves these registers",
        ),
    ),
    (
        "LibUsbRtl88xxBackend",
        Contention::Pinned("mac_init -> init_edca_cfg(); slot + all four AC params by name"),
    ),
    (
        "Rtl8812auBackend",
        Contention::Pinned(
            "mac_config -> config_table(MAC_REG); the writes are DATA in an include_bytes! blob, invisible to a source grep — MEASURED identical across five processes",
        ),
    ),
    (
        "Rtl8733buBackend",
        Contention::PowerCycled(
            "PLAN_8812AU_MONITOR's `power_on` rung -> power_off() then power_on(); since M8 every \
             entry point routes through that plan",
        ),
    ),
    (
        "Rtl8821cuBackend",
        Contention::PowerCycled(
            "bring-up card-disables first when REG_CR != 0xea, then CARD_ENABLE. Its own EDCA write is gated behind NDN_RADIO_IBSS and does NOT run by default",
        ),
    ),
    (
        "Ath9kHtcBackend",
        Contention::Pinned(
            "init_queues resets all four TX queues on every bring-up via the ath9k_hw_resettxqueue sequence, writing AR_DLCL_IFS/AR_DRETRY_LIMIT/AR_QMISC/AR_DMISC with the USEDEFAULT DCF parameters (cwmin 15, cwmax 1023, aifs 2). Contention is therefore re-established per open, not inherited. ★ Reviewed 2026-09-01; the earlier Unreviewed reason named ath9k_hw_set_txq_props, which was never ported and so was the wrong thing to look for",
        ),
    ),
    (
        "MorseFrameIo",
        Contention::NotApplicable("HaLow via mac80211 — the kernel driver owns the EDCA arbiter"),
    ),
    (
        "Nrc7292FrameIo",
        Contention::NotApplicable("HaLow via mac80211 — the kernel driver owns the EDCA arbiter"),
    ),
    (
        "SerialRadioBackend",
        Contention::NotApplicable("the MAC lives in device firmware across a serial link"),
    ),
    (
        "Bw16SerialBackend",
        Contention::NotApplicable("the MAC lives in device firmware across a serial link"),
    ),
    (
        "Esp32SerialBackend",
        Contention::NotApplicable("the MAC lives in device firmware across a serial link"),
    ),
    (
        "LoraSerialBackend",
        Contention::NotApplicable("LoRa has no EDCA; contention is the firmware's CSMA/LBT"),
    ),
];

/// ★ **The declaration-vs-actuator gate for the payload ceiling** (added 2026-08-31).
///
/// `RadioCapability::max_payload` is what a backend ADVERTISES; the length check at the top of its
/// `inject` is what it ACTUALLY ACCEPTS. Nothing related the two, and they drifted in both
/// directions: the MT7921AU advertised 1500 against a 11454 guard (7.6x) and the MT7612U advertised
/// 1500 against 5650 (3.8x) — both shipped *after* the MT7610U found, measured and documented the
/// same bug, because the fix lived in one backend's `mod tests` instead of in a gate every backend
/// passes through. (The sibling's copy of that test had the load-bearing assertion replaced with
/// `assert!(c.max_payload > 0)`.)
///
/// The rule is deliberately **`declared <= guard`**, not equality: they are two honest and
/// different numbers. The guard defends the silicon — past it an oversized MPDU resets the radio
/// rather than being dropped. The declaration says what is *usable*, which measurement may put
/// lower (the MT7921AU sustains 7935 B but collapses to 3 f/s at 11000).
///
/// Each entry READS the backend's own declaration through a function pointer rather than restating
/// it — a hand-typed copy here would be the exact silent divergence this file exists to police.
pub const PAYLOAD: &[(&str, fn() -> ndn_radio_hal::RadioCapability, usize)] = &[
    (
        "Mt7610uBackend",
        crate::mt76x0::declared_capability,
        crate::mt76x0::MAX_MPDU_PAYLOAD,
    ),
    (
        "Mt7921uBackend",
        crate::mt7921::declared_capability,
        crate::mt7921::MAX_MPDU_PAYLOAD,
    ),
    (
        "Mt7612uBackend",
        crate::Mt7612uBackend::declared_capability,
        crate::Mt7612uBackend::MAX_MPDU_PAYLOAD,
    ),
];

/// The unmeasured payload default that `RadioCapability::wifi_monitor_5ghz` supplies. A backend
/// that has measured its own MPDU ceiling and still declares this has inherited the preset by
/// accident — which is precisely how both live bugs above happened.
pub const UNMEASURED_PRESET_PAYLOAD: usize = 1500;

/// The table. Order: campaign radios first.
pub const COVERAGE: &[Row] = &[
    Row {
        backend: "LibUsbRtl88xxBackend (RTL8822E \"a81a\")",
        pids: &[0xa81a, 0xa811, 0x8814],
        campaign: true, // the reliable 5 GHz TX (o5p-0)
        frame_io: Provided,
        knobs: Provided,
        time: Provided,
        profile: Provided,
    },
    Row {
        backend: "Rtl8812auBackend",
        pids: &[0x8812, 0x881a],
        campaign: true, // o5p-2 + the 881a on o5p-1 (RX/light roles; brownout under sustained TX)
        frame_io: Provided,
        knobs: Provided,
        // Provided, and SHALLOW in a way this cell cannot show (see the DEPTH warning above): a real
        // per-frame RXTSFL stamp, but its clock REFERENCE is `Unknown` and so `can_common_view` is
        // false. Unlike its two siblings this port never reads a crystal cap and nobody has
        // regressed its TSF against a host or a peer, so nothing in-tree witnesses what the counter
        // runs on. One run of `examples/rxtick.rs` would settle it. See `impl RadioTime for
        // Rtl8812auBackend`.
        //
        // ⚠ **And this radio's common-view observations are LIVE anyway** — the one open gap this
        // table should not hide. `FrameIo::mesh_common_view` (#74/#75) keeps pairing a neighbour's
        // beacon TSF with our RXTSFL, and `ndn-phy-wifi`'s `medium.rs` feeds that straight into
        // `FaceScheduler::ingest_common_view` without consulting `FaceTimeProfile` at all. So the
        // profile's `false` is a declaration nothing enforces, and on an 8812au face slot epochs are
        // being disciplined from a counter whose oscillator nobody has established. Two exits, in
        // order: settle the reference (then there is no contradiction), or gate the consumer (which
        // switches the #75 leaf path off on this radio — a deliberate cost, not a side effect).
        // Written up on `FrameIo::mesh_common_view for Rtl8812auBackend`.
        time: Provided,
        profile: Provided,
    },
    // ── The 7E-A5 serial sub-GHz fleet ─────────────────────────────────────────────────────────
    // ONE backend (`LoraSerialBackend`) drives all three, so the Rust type is the same in every row
    // and the witnesses below repeat. The rows are per NODE on purpose: what a row claims is what
    // that node's `EVT_CAP` (or its pinned `RadioKindHint` profile) makes reachable, and those
    // differ sharply — the LR2021 has no `CMD_SET_FREQ`, the Heltec has neither a hardware stamp nor
    // most of the knob set, the Waveshare has the full knob set. Reading one row as a statement
    // about the type would lose exactly the distinction the capability rewrite exists to carry.
    //
    // ⚠ CORRECTION 2026-08-31: this note used to end "…and the Waveshare has the full knob set and
    // only a software counter". That is false — its TIM3 input capture landed (95/95 frames
    // hardware-stamped, 0 fallbacks) and it declares a `FreeRunRxStamp` like the LR2021's.
    Row {
        backend: "LoraSerialBackend (Waveshare SX1262)",
        pids: &[0x55d3],
        campaign: true, // campaign (c) reports Wi-Fi and LoRa separately
        frame_io: Provided,
        knobs: Provided,
        time: Provided,
        profile: Provided,
    },
    Row {
        backend: "LoraSerialBackend (XIAO nRF54L15 + LR2021)",
        // No PID recorded: this board is reached by path (/dev/ttyACM*), and its USB ids have not
        // been read off the rig. An empty list is the honest entry — a plausible-looking PID here
        // would silently claim a device the dispatch never matches.
        pids: &[],
        // Not in the pre-registered campaign set: the link is live (o5p-0 -> o5p-1 decodes on air
        // at 915 MHz FLRC), but the node has not yet been exercised through this backend end to
        // end, and its 7E-A5 v2 self-description is landing concurrently. Admit it to the campaign
        // when a run has actually driven it through FrameIo, not on the strength of a live link.
        campaign: false,
        frame_io: Provided,
        knobs: Provided,
        // ⚠ **RETRACTED 2026-08-31**, both halves of it. This cell used to read: "the only node in
        // this fleet whose clock cell means what the Realtek ones mean: a free-running per-frame
        // HARDWARE stamp (16 MHz DPPI capture, 62.5 ns MEASURED), so
        // `FaceTimeProfile::can_common_view` is true here and false on every other sub-GHz node."
        //
        // * **"the only node"** — false since the Waveshare's TIM3 input capture landed. It stamps
        //   95/95 frames in silicon at the same `LatchPoint::RadioCapture`, so on the latch axis the
        //   two nodes are now indistinguishable.
        // * **"can_common_view is true here"** — no longer true, and the reason is the point.
        //   MEASURED across two receivers of the same frames: two LR2021s give 0.81-1.86 us and the
        //   residual is FLAT against the fit span (1.11 -> 1.55 us from 1.4 s to 10.2 s); two
        //   Waveshares gave 10.5-20.4 us and GROWING (16 -> 130 us). Same latch, different
        //   references — a crystal versus an 8 MHz internal RC — so the latch point alone stopped
        //   being the test and `RadioTimeSource::reference` became the other half of it.
        //
        // Where that leaves this node — ⚠ and the middle of this cell was itself wrong for a day,
        // so read the correction with the retraction. It said: "this firmware does not implement
        // `CMD_GET_CLOCK_REF` (0x21) … and the host will not promote a reference the node has not
        // stated, because NOTHING ON THE WIRE SEPARATES this build from the pre-2026-08-28 one that
        // MEASURED +2253 ppm on the nRF's internal RC. So `can_common_view` is false here today."
        //
        // That premise is false and `git` is the witness: `EVT_CAP` and `HfclkSource::ExternalXtal`
        // entered `m6_bridge` in the SAME commit (1283b7e — which added the `ExternalXtal` LINE to the older `hw.rs`, and
        // `git log -S hfclk_source -- firmware/` returns that commit alone), and the older RC build
        // speaks 7E-A5 **v1**, which cannot emit an `EVT_CAP` at all. An LR2021 that ANSWERS
        // `CMD_GET_CAP` is therefore necessarily a crystal build, `NodeProfile.learned` already
        // records whether the record came from the device, and `assumed_clock_reference` now reads
        // it (see its doc). So this node — the fleet's only part with a this-session-measured
        // common view, 0.81/1.55/1.86 us FLAT across the fit span — keeps `can_common_view` **true**
        // whenever it has spoken for itself, and loses it only when the host had to pin a profile.
        //
        // And since 4d01eb1 it no longer relies on that inference at all: `m6_bridge` answers 0x21
        // with `[crystal][accuracy unknown]`, MEASURED on both bench boards (`02 ff ff`), which is
        // bit-for-bit the same `ClockReference` the fallback assumes. The Waveshare answers it too.
        // The Heltec SX1276 replies `EVT_UNSUPPORTED` — an honest unknown, and it has no hardware
        // latch either way.
        time: Provided,
        profile: Provided,
    },
    Row {
        backend: "LoraSerialBackend (Heltec LoRa32 V2, SX1276)",
        pids: &[],       // CP2102 bridge; ids not read off the rig (see the LR2021 row)
        campaign: false, // NEVER exercised on air — mds-o5p-2's sshd is down, so the board has not
        // been flashed or measured. Its profile is pinned from firmware source + the SX1276
        // datasheet and must be replaced by the board's own EVT_CAP.
        frame_io: Provided,
        knobs: Provided,
        time: Provided,
        profile: Provided,
    },
    Row {
        backend: "Rtl8733buBackend",
        pids: crate::RTL8733B_PIDS,
        campaign: false,
        frame_io: Provided,
        knobs: Provided,
        time: Provided,
        profile: Provided,
    },
    Row {
        backend: "Mt7921uBackend (MT7921AU, connac2 802.11ax)",
        pids: crate::MT7921U_PIDS,
        campaign: false, // new port; firmware download and RX are the on-air gate
        frame_io: Provided,
        knobs: Provided,
        // ★ The only MediaTek part here whose clock cell means what the Realtek ones mean: a
        // per-frame FreeRunRxStamp from RXD group 2, not a read-now PortTsf. `FaceTimeProfile`
        // derives `can_common_view = true` from it — the first time that is true for a
        // MediaTek radio in this crate.
        //
        // Still true after the reference axis landed (2026-08-31), and now for a stated reason
        // rather than by default: no register in this port names the oscillator, but the counter
        // MEASURED 15_007_757 ticks across 15_007_907 host microseconds = **-10.0 ppm over 15 s**,
        // three orders of magnitude tighter than an RC reference (+2253 ppm on the LR2021's,
        // ~-3100 ppm on the Waveshare's), so it declares `ClockReference::crystal()` carrying that
        // measurement. See `impl RadioTime for Mt7921uBackend`.
        time: Provided,
        profile: Provided,
    },
    Row {
        backend: "Mt7610uBackend (MT7610U, mt76x0)",
        pids: crate::MT7610U_PIDS,
        campaign: false, // new port; on-air validation is the gate, not the trait matrix
        frame_io: Provided,
        knobs: Provided,
        time: Provided, // port TSF (0x111c), MEASURED 1.000 us/tick on this exact part
        profile: Provided,
    },
    Row {
        backend: "Mt7612uBackend",
        pids: crate::MT7612U_PIDS,
        campaign: false, // off the bus since the #110 wedge; needs a replug + the 0x09a8 poll fix
        frame_io: Provided,
        knobs: Provided,
        // Flipped 2026-08-27 with the register map corrected: `impl RadioTime for
        // Mt7612uBackend` now reports a real port TSF (MT_TSF_TIMER_DW0 0x111c, armed by
        // MT_BEACON_TIME_CFG 0x1114 bit 16), MEASURED at 1.000 us/tick on this dongle. The
        // old exclusion is quoted in that impl's doc comment along with why its evidence was
        // void. `can_common_view` is still false and correctly so — a PortTsf is not a
        // per-frame stamp — which the FaceTimeProfile derivation gets right on its own.
        time: Provided,
        profile: Provided,
    },
    Row {
        backend: "Rtl8821cuBackend",
        pids: crate::RTL8821CU_PIDS,
        campaign: false,
        frame_io: Provided,
        knobs: Excluded(
            "#79/#80 ruling (P3.11 confirms): bring-up is validated RX-side only; no control write \
             has been hardware-verified, and declaring knobs without a validated actuator is the \
             decided-but-unactuated defect this project keeps re-finding. Lifted by: a bench \
             session validating set_channel/set_rate writes on the part.",
        ),
        time: Excluded(
            "Same ruling as knobs: nothing true to say until the part is on a bench. \
             (The shared RX pump was also deliberately not ported — #80 — same reason.)",
        ),
        profile: Provided,
    },
    Row {
        backend: "SerialRadioBackend (RTL8720DN)",
        pids: &[], // serial bridge — no USB PID dispatch; opened by device path
        campaign: false,
        frame_io: Provided,
        knobs: Provided,
        // Provided, and it is the HOST clock — SHALLOW in the way this cell cannot show. ⚠ Corrected
        // 2026-08-31: this backend used to declare a `FreeRunRxStamp` (so `hw_rx_stamp = true`, a
        // 1 us floor, `LatchPoint::MacDone`) for a number that is `us_ticker_read()` called by
        // SOFTWARE at the top of the vendor blob's promiscuous RX callback — the MAC's own RXTSFL is
        // in the RX descriptor and the blob drops it first. It now reports the honest host stamp,
        // like the 7E-A5 fleet's software-counter nodes. A `CLOCK_REF_XTAL`-shaped answer from this
        // part would be true and would still not earn common view: it fails the LATCH half.
        time: Provided,
        profile: Provided,
    },
    Row {
        backend: "Ath9kHtcBackend (AR9271)",
        pids: &[0x9271], // AR9271_IDS is (vid,pid) pairs; 0x9271 under Atheros VID
        campaign: false,
        frame_io: Provided, // M3: stamped RX up + TX inject (build_tx_frame_bytes); RX proven on
        // silicon, TX framing bench-uncertain (node/rate handling) — flagged at the impl.
        // STALE-EXCLUSION FIX: this cell read `Excluded("no &self RadioKnobs yet")` long after
        // `impl RadioKnobs for Ath9kHtcBackend` landed with seven actuated methods (live channel +
        // HT20/40 retune, EDCCA force-rx-clear, the SDR-measured TX-gain ladder in both index and
        // dBm, AR_RCCNT occupancy, ScheduledAt discipline, the Tier-0 name filter). The gate below
        // could not catch it: an Excluded cell whose impl quietly appears adds to neither the
        // Provided count nor the witness count, so the two stayed equal. See the module header.
        knobs: Provided,
        time: Provided, // M2: per-frame rs_tstamp is a FreeRunRxStamp common-view clock.
        profile: Provided, // wifi_monitor_2ghz_1ss (1x1 2.4 GHz 11n, ch 1..13).
    },
];

/// Render the table (the visible artifact #79 asked for; printed by the coverage test).
pub fn render() -> String {
    let mut out = String::from(
        "backend                                  campaign  FrameIo  Knobs  Time  Profile\n",
    );
    for r in COVERAGE {
        let cell = |s: &Seam| match s {
            Provided => "yes",
            Excluded(_) => "EXCL",
        };
        out.push_str(&format!(
            "{:<41}{:<10}{:<9}{:<7}{:<6}{}\n",
            r.backend,
            if r.campaign { "YES" } else { "-" },
            cell(&r.frame_io),
            cell(&r.knobs),
            cell(&r.time),
            cell(&r.profile),
        ));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{RadioKnobs, RadioProfile, RadioTime};
    use ndn_frame_io::FrameIo;

    // Compile-time verification of every `Provided` cell: these functions only compile if the
    // impl exists, so the table cannot overclaim. One line per claimed cell, grouped per backend —
    // adding a `Provided` to the table without adding its line here fails the count check below.
    fn is_frame_io<T: FrameIo>() {}
    fn is_knobs<T: RadioKnobs>() {}
    fn is_time<T: RadioTime>() {}
    fn is_profile<T: RadioProfile>() {}

    fn compile_time_provided_cells() -> usize {
        let mut n = 0;
        macro_rules! claim {
            ($f:ident::<$t:ty>) => {{
                let _ = $f::<$t>;
                n += 1;
            }};
        }
        claim!(is_frame_io::<crate::LibUsbRtl88xxBackend>);
        claim!(is_knobs::<crate::LibUsbRtl88xxBackend>);
        claim!(is_time::<crate::LibUsbRtl88xxBackend>);
        claim!(is_profile::<crate::LibUsbRtl88xxBackend>);
        claim!(is_frame_io::<crate::Rtl8812auBackend>);
        claim!(is_knobs::<crate::Rtl8812auBackend>);
        claim!(is_time::<crate::Rtl8812auBackend>);
        claim!(is_profile::<crate::Rtl8812auBackend>);
        // Feature-gated backends: witnesses ride the gate; without the feature the cells are
        // counted as vacuously witnessed (the table describes the FULL build — campaign tooling
        // builds with these features on).
        // Three ROWS, one TYPE: the Waveshare SX1262, the nRF54L15+LR2021 bridge and the Heltec
        // SX1276 are all driven by `LoraSerialBackend`, so the same four witnesses are claimed once
        // per row. (The witness gate proves the impl exists; which of the three nodes is on the far
        // end is a runtime `NodeProfile` fact no type check can see.)
        #[cfg(feature = "lora")]
        for _node in 0..3 {
            claim!(is_frame_io::<crate::LoraSerialBackend>);
            claim!(is_knobs::<crate::LoraSerialBackend>);
            claim!(is_time::<crate::LoraSerialBackend>);
            claim!(is_profile::<crate::LoraSerialBackend>);
        }
        #[cfg(not(feature = "lora"))]
        {
            n += 12;
        }
        claim!(is_frame_io::<crate::Rtl8733buBackend>);
        claim!(is_knobs::<crate::Rtl8733buBackend>);
        claim!(is_time::<crate::Rtl8733buBackend>);
        claim!(is_profile::<crate::Rtl8733buBackend>);
        claim!(is_frame_io::<crate::Mt7921uBackend>);
        claim!(is_knobs::<crate::Mt7921uBackend>);
        claim!(is_time::<crate::Mt7921uBackend>);
        claim!(is_profile::<crate::Mt7921uBackend>);
        claim!(is_frame_io::<crate::Mt7610uBackend>);
        claim!(is_knobs::<crate::Mt7610uBackend>);
        claim!(is_time::<crate::Mt7610uBackend>);
        claim!(is_profile::<crate::Mt7610uBackend>);
        claim!(is_frame_io::<crate::Mt7612uBackend>);
        claim!(is_knobs::<crate::Mt7612uBackend>);
        claim!(is_time::<crate::Mt7612uBackend>);
        claim!(is_profile::<crate::Mt7612uBackend>);
        claim!(is_frame_io::<crate::Rtl8821cuBackend>);
        claim!(is_profile::<crate::Rtl8821cuBackend>);
        // AR9271 (ath9k_htc) — M3 FrameIo + M2 RX-stamp clock + capability profile + the full
        // knob surface (live retune, EDCCA, the measured dBm ladder, AR_RCCNT occupancy).
        claim!(is_frame_io::<crate::Ath9kHtcBackend>);
        claim!(is_knobs::<crate::Ath9kHtcBackend>);
        claim!(is_time::<crate::Ath9kHtcBackend>);
        claim!(is_profile::<crate::Ath9kHtcBackend>);
        #[cfg(feature = "serial-radio")]
        {
            claim!(is_frame_io::<crate::SerialRadioBackend>);
            claim!(is_knobs::<crate::SerialRadioBackend>);
            claim!(is_time::<crate::SerialRadioBackend>);
            claim!(is_profile::<crate::SerialRadioBackend>);
        }
        #[cfg(not(feature = "serial-radio"))]
        {
            n += 4;
        }
        n
    }

    /// ★ **The TX-intent gate, as a test** — see [`TX_INTENT`].
    ///
    /// Every transmitting backend must have ANSWERED whether it honours the basic-rate doctrine.
    /// A `Gap` is allowed and printed — a known unknown beats a silent one — but a blank is not.
    #[test]
    fn every_backend_states_whether_it_honours_tx_intent() {
        let mut gaps = vec![];
        for (name, t) in TX_INTENT {
            let reason = match t {
                TxIntentSupport::Honoured(r) | TxIntentSupport::NotApplicable(r) => r,
                TxIntentSupport::Gap(r) => {
                    gaps.push(*name);
                    r
                }
            };
            assert!(
                reason.len() >= 30,
                "{name}: TX-intent disposition needs a real written reason, got {reason:?}"
            );
        }
        for r in COVERAGE {
            if matches!(r.frame_io, Provided) {
                let short = r.backend.split_whitespace().next().unwrap_or(r.backend);
                assert!(
                    TX_INTENT.iter().any(|(n, _)| *n == short),
                    "{short} transmits but does not say whether it honours \
                     TxIntent::needs_basic_rate — a MostRobust frame going out at a throughput \
                     rate is the one-way link this doctrine exists to prevent"
                );
            }
        }
        if !gaps.is_empty() {
            println!("TX intent GAPS (radio could honour it, does not): {gaps:?}");
        }
    }

    /// ★ **The bring-up contention gate, as a test** — see [`CONTENTION`].
    ///
    /// Asserts only what a table can: that every backend has ANSWERED the question and that no
    /// answer is a blank. It cannot prove a radio does not leak — only hardware can, and the
    /// MT7610U measurement is what put this here. What it does prevent is the actual failure mode:
    /// a backend added, or a bring-up reordered, with nobody having asked.
    #[test]
    fn every_backend_states_how_its_contention_is_made_deterministic() {
        let mut unreviewed = vec![];
        for (name, c) in CONTENTION {
            let reason = match c {
                Contention::Pinned(r)
                | Contention::PowerCycled(r)
                | Contention::NotApplicable(r) => r,
                Contention::Unreviewed(r) => {
                    unreviewed.push(*name);
                    r
                }
            };
            assert!(
                reason.len() >= 30,
                "{name}: contention disposition needs a real written reason, got {reason:?}"
            );
        }
        // Every backend carrying a FrameIo seam must appear — that is what makes this a checklist
        // rather than a list of the ones somebody remembered.
        for r in COVERAGE {
            if matches!(r.frame_io, Provided) {
                let short = r.backend.split_whitespace().next().unwrap_or(r.backend);
                assert!(
                    CONTENTION.iter().any(|(n, _)| *n == short),
                    "{short} drives a radio but does not say how its contention is made \
                     deterministic — add a CONTENTION row (see the 2.5x run-order swing)"
                );
            }
        }
        if !unreviewed.is_empty() {
            println!("contention UNREVIEWED (known unknowns): {unreviewed:?}");
        }
    }

    /// ★ **The declaration-vs-actuator gate, as a test** — see [`PAYLOAD`].
    ///
    /// Two live bugs motivated it, both found by audit rather than by a test: the MT7921AU
    /// advertising 1500 B against an 11454 B guard, and the MT7612U advertising 1500 against 5650.
    /// Both are the same failure as the 8812au's `max_bw: 0` — a lever the hardware has and the
    /// declaration hides, so cognition never asks for it.
    #[test]
    fn declared_payload_never_exceeds_the_inject_guard() {
        for (name, declared, guard) in PAYLOAD {
            let d = declared().max_payload;
            assert!(
                d <= *guard,
                "{name} advertises max_payload {d} B but its inject guard refuses above {guard} B \
                 — it would promise a peer a frame size it then rejects"
            );
            assert_ne!(
                d, UNMEASURED_PRESET_PAYLOAD,
                "{name} has a MEASURED MPDU guard of {guard} B but still declares the preset's \
                 unmeasured {UNMEASURED_PRESET_PAYLOAD} B — the inherited-by-accident bug this \
                 gate exists to catch"
            );
            assert!(d > 0, "{name} declares a zero payload ceiling");
        }
    }

    /// The #79 gate, as a test: every row is full or excluded IN WRITING; every campaign radio is
    /// full; every `Provided` in the table has a compile-time witness above; no PID is claimed
    /// twice. Prints the table so the artifact is visible in test output.
    #[test]
    fn every_backend_shows_a_full_row_or_a_written_exclusion() {
        let mut provided_cells = 0;
        let mut pids_seen = std::collections::HashSet::new();
        for r in COVERAGE {
            for (seam, name) in [
                (&r.frame_io, "frame_io"),
                (&r.knobs, "knobs"),
                (&r.time, "time"),
                (&r.profile, "profile"),
            ] {
                match seam {
                    Provided => provided_cells += 1,
                    Excluded(reason) => {
                        assert!(
                            reason.len() > 40,
                            "{}: {name} excluded without a real written reason — that is the \
                             silent gap #79 exists to remove",
                            r.backend
                        );
                        assert!(
                            !r.campaign,
                            "{}: a CAMPAIGN radio may not carry exclusions ({name}) — full row \
                             or out of the campaign, no third option (P3 gate)",
                            r.backend
                        );
                    }
                }
            }
            for pid in r.pids {
                assert!(pids_seen.insert(*pid), "PID {pid:#06x} claimed by two rows");
            }
        }
        assert_eq!(
            provided_cells,
            compile_time_provided_cells(),
            "the table's Provided count differs from the compile-time witnesses — a cell was \
             claimed without its witness (or a witness added without its cell)"
        );
        println!("{}", render());
    }
}
