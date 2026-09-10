//! **§6.3 `tests/plan_shape.rs`, scoped to the three MediaTek parts and the RTL8821CU**
//! (contract §5-M5).
//!
//! Hardware-free. It reads each part's `Plan` the way a reviewer would and asserts the properties
//! the contract says a plan must have — the ones that are *not* already compile errors, plus a
//! belt over the ones that are (a `const` panic carries a literal message only, so the compile
//! error says *which rule* broke and not which rung).
//!
//! ⚠ Scope. §6.3's full sweep — *every* backend with a `FrameIo` impl declares a plan or a written
//! `bringup_coverage` exclusion — is M9. This file asserts what M5 actually landed.

use ndn_radio_drivers::{
    Mt7610uBackend, Mt7612uBackend, Mt7921uBackend, PLAN_8821CU_FW_STA, PLAN_8821CU_IBSS,
    PLAN_8821CU_MONITOR, PLAN_8821CU_NO_TXEN, PLAN_8821CU_STATION_REGS, PLAN_MT7610U, PLAN_MT7612U,
    PLAN_MT7921AU, Rtl8821cVariant, Rtl8821cuBackend,
};
use ndn_radio_hal::bringup::{BringUp, Plan, Role, Stage, StepClass};

// ── the ladders, transcribed independently of the plans ──────────────────────
//
// ★ Each list is written from the ORIGINAL `bring_up` (+ `setup_monitor_rx` + the tune) rather
// than read out of the plan, so a reordering shows up as a diff between two lists rather than as
// nothing at all. M5's whole claim is transcription.

const LADDER_MT7610U: &[&str] = &[
    "rx_drain",
    "firmware_ready",
    "init_usb_dma",
    "init_hardware",
    "usb_timing",
    "mac_address",
    "pin_contention",
    "rx_drain_stop",
    "monitor_rx",
    "tune_channel",
];

const LADDER_MT7612U: &[&str] = &[
    "rx_drain",
    "firmware_and_init",
    "pin_edca",
    "monitor_rx",
    "tune_channel",
];

const LADDER_MT7921AU: &[&str] = &[
    "firmware_ready",
    "rx_drain",
    "set_eeprom",
    "mac_init",
    "mac_enable",
    "factory_mac",
    "pin_edca",
    "rx_drain_stop",
    "tune_channel",
    "monitor_rx",
];

const LADDER_8821CU: &[&str] = &[
    "read_cut_version",
    "pre_system_cfg",
    "power_cycle_if_on",
    "card_enable",
    "read_chip_info",
    "download_firmware",
    "mac_init",
    "phy_set_param",
    "send_fw_info",
    "coex_grant_wl",
    "monitor_rx",
    "tune_channel",
    "bb_rx_path_enable",
    "iqk",
    "txen_golden_block",
    "hci_usb_cfg",
];

fn ids<B: 'static>(p: &'static Plan<B>) -> Vec<&'static str> {
    p.steps.iter().map(|s| s.id.as_str()).collect()
}

fn at<B: 'static>(p: &'static Plan<B>, id: &str) -> usize {
    ids(p)
        .iter()
        .position(|i| *i == id)
        .unwrap_or_else(|| panic!("{} has no rung `{id}`", p.id))
}

fn declares_follow<B: 'static>(p: &'static Plan<B>, later: &str, earlier: &str) -> bool {
    p.steps[at(p, later)]
        .must_follow
        .iter()
        .any(|i| i.as_str() == earlier)
        || p.steps[at(p, earlier)]
            .must_precede
            .iter()
            .any(|i| i.as_str() == later)
}

/// The rung bodies of one driver file — every `s_*` step function and nothing else. Used by the
/// source-level tests below, which must read CODE and not the prose that quotes the code it
/// rejects.
fn rung_section(src: &str) -> &str {
    src.split_once("// ── the rungs ─")
        .expect("the rung section")
        .1
        .split_once("// ── the rungs, as reviewable constants")
        .expect("the end of the rung section")
        .0
}

/// The same section with every `//` comment stripped.
///
/// ★ Necessary, not fussy: these tests reject a spelling, and the code that does the right thing
/// SAYS SO in a comment quoting the spelling it avoids (`restore_edca_defaults`, NOT
/// `set_contention`). A scanner that reads prose would fail the file for documenting the hazard,
/// which is precisely backwards. `tests/one_frame_builder.rs` learned the same lesson.
fn code_only(section: &str) -> String {
    section
        .lines()
        .map(|l| match l.find("//") {
            Some(i) => &l[..i],
            None => l,
        })
        .collect::<Vec<_>>()
        .join("\n")
}

// ── the plans hold together ──────────────────────────────────────────────────

/// No empty `why`, no blank degradation, no duplicate id, and every `must_follow`/`must_precede`
/// names a rung the plan contains **and** is satisfied by the sequence. A plan is checked before
/// the first register write; a malformed one must never reach the hardware.
#[test]
fn every_plan_holds_together() {
    macro_rules! check {
        ($p:expr) => {
            $p.check()
                .unwrap_or_else(|e| panic!("{} does not hold together: {e}", $p.id))
        };
    }
    check!(PLAN_MT7610U);
    check!(PLAN_MT7612U);
    check!(PLAN_MT7921AU);
    check!(PLAN_8821CU_MONITOR);
    check!(PLAN_8821CU_FW_STA);
    check!(PLAN_8821CU_NO_TXEN);
    check!(PLAN_8821CU_STATION_REGS);
    check!(PLAN_8821CU_IBSS);
}

/// ★ **Each plan IS its ladder** — same rungs, same order, nothing added, nothing dropped.
#[test]
fn every_plan_is_the_transcribed_ladder() {
    for (id, got, want) in [
        (PLAN_MT7610U.id, ids(&PLAN_MT7610U), LADDER_MT7610U),
        (PLAN_MT7612U.id, ids(&PLAN_MT7612U), LADDER_MT7612U),
        (PLAN_MT7921AU.id, ids(&PLAN_MT7921AU), LADDER_MT7921AU),
        (
            PLAN_8821CU_MONITOR.id,
            ids(&PLAN_8821CU_MONITOR),
            LADDER_8821CU,
        ),
    ] {
        assert_eq!(
            got, want,
            "{id} has drifted from the ladder `bring_up` ran. Every rung was transcribed VERBATIM, \
             IN ORDER; a plan that 'improves' a ladder is an unmeasured change to a radio nobody \
             at the keyboard can test."
        );
    }
}

/// **Every `why` is non-empty** — `coverage::Seam::Excluded`'s rule applied to rungs, to the §1.5
/// asserts, and to the written exclusions.
#[test]
fn every_why_and_every_exclusion_is_written() {
    macro_rules! written {
        ($p:expr) => {
            for s in $p.steps {
                assert!(
                    !s.why.trim().is_empty(),
                    "{}::{} has no stated reason. The measurement or vendor reference that puts a \
                     rung in a ladder is the only thing that stops the next person deleting it — \
                     or, worse, copying it into a private bring-up.",
                    $p.id,
                    s.id
                );
            }
            for (stage, reason) in $p.excluded {
                assert!(
                    !reason.trim().is_empty(),
                    "{} excludes {stage:?} with no ruling. An empty exclusion is an absence \
                     pretending to be a decision.",
                    $p.id
                );
            }
        };
    }
    written!(PLAN_MT7610U);
    written!(PLAN_MT7612U);
    written!(PLAN_MT7921AU);
    written!(PLAN_8821CU_MONITOR);
    written!(PLAN_8821CU_FW_STA);
    written!(PLAN_8821CU_NO_TXEN);
    written!(PLAN_8821CU_STATION_REGS);
    written!(PLAN_8821CU_IBSS);

    for a in <Mt7610uBackend as BringUp>::asserts() {
        assert!(!a.why.trim().is_empty(), "mt76x0 assert `{}`", a.id);
    }
    for a in <Mt7612uBackend as BringUp>::asserts() {
        assert!(!a.why.trim().is_empty(), "mt7612 assert `{}`", a.id);
    }
}

// ── §5-M5's headline: the three calls are one plan ───────────────────────────

/// ★★ **`MT_MAC_SYS_CTRL = ENABLE_TX | ENABLE_RX` can no longer be missing.**
///
/// That is §5-M5's stated reason for collapsing `bring_up` + `setup_monitor_rx` + the tune into one
/// plan. The property it buys is structural: every mt76 plan contains the monitor rung and the
/// tune rung, so no caller can run two of the three and get a radio that answers every register
/// read while receiving nothing.
#[test]
fn every_mt76_plan_contains_monitor_and_tune() {
    for (id, got) in [
        (PLAN_MT7610U.id, ids(&PLAN_MT7610U)),
        (PLAN_MT7612U.id, ids(&PLAN_MT7612U)),
        (PLAN_MT7921AU.id, ids(&PLAN_MT7921AU)),
    ] {
        for want in ["monitor_rx", "tune_channel"] {
            assert!(
                got.contains(&want),
                "{id} has no `{want}` rung. `bring_up` + `setup_monitor_rx` + the tune are ONE \
                 plan (§5-M5) precisely so that MT_MAC_SYS_CTRL cannot be missing because a caller \
                 forgot a line."
            );
        }
    }
}

/// The two parts that can read `MT_MAC_SYS_CTRL` back assert it. (The MT7921AU cannot — its MAC
/// enable is firmware state behind `MCU_CE_CMD`, and `ASSERTS_MT7921AU` says so in writing.)
#[test]
fn the_mac_enable_gate_is_read_back_where_it_is_readable() {
    for (part, asserts) in [
        ("mt76x0", <Mt7610uBackend as BringUp>::asserts().len()),
        ("mt7612", <Mt7612uBackend as BringUp>::asserts().len()),
    ] {
        assert!(
            asserts > 0,
            "{part} declares no asserts, but MT_MAC_SYS_CTRL is a host-writable register on this \
             part and §1.5 says read back every gate you write"
        );
    }
    assert!(
        <Mt7610uBackend as BringUp>::asserts()
            .iter()
            .any(|a| a.id.as_str() == "mac_tx_rx_enabled"),
        "the MT7610U must assert MT_MAC_SYS_CTRL"
    );
    assert!(
        <Mt7612uBackend as BringUp>::asserts()
            .iter()
            .any(|a| a.id.as_str() == "mac_tx_rx_enabled"),
        "the MT7612U must assert MT_MAC_SYS_CTRL"
    );
    // Empty, and the reason is a doc comment on the constant rather than a blank.
    assert!(
        <Mt7921uBackend as BringUp>::asserts().is_empty(),
        "if the MT7921AU has grown an assert, its ASSERTS_MT7921AU doc comment — which explains at \
         length why there is none — is now wrong"
    );
}

// ── the measured orderings, as constraints rather than prose ─────────────────

/// ★★ **MT7612U: the tune must come AFTER the monitor rung.**
///
/// MEASURED 2026-08-27: the captured channel op-stream contains the KERNEL's own
/// `MT_RX_FILTR_CFG` write, so replaying it overwrites whatever monitor state was installed —
/// `0x00001093` (drop CRC_ERR|PHY_ERR|VER_ERR|DUP|RTS), 9975 PHY CRC errors, essentially nothing
/// reaching the host. `replay_chanset` re-asserts the filter at its tail, which is why the tune
/// goes second rather than the monitor rung being moved after it.
#[test]
fn mt7612_tunes_after_monitor_and_declares_it() {
    assert!(at(&PLAN_MT7612U, "monitor_rx") < at(&PLAN_MT7612U, "tune_channel"));
    assert!(
        declares_follow(&PLAN_MT7612U, "tune_channel", "monitor_rx"),
        "PLAN_MT7612U satisfies monitor-before-tune by position but does not DECLARE it — an \
         undeclared ordering is prose again, and reordering the rungs would still build"
    );
}

/// ★★ **MT7921AU: the tune must come BEFORE the monitor rung.** The opposite of its sibling, and
/// the reason it must be declared rather than remembered.
///
/// `setup_monitor_rx` REFUSES while the channel is still 0 — "the sniffer carries its own copy of
/// the channel and has nothing to be told" — so `open_named_radio`'s arm, which had these two
/// backwards, returned an error for EVERY caller of this part until 2026-09-01.
#[test]
fn mt7921_tunes_before_monitor_and_declares_it() {
    assert!(at(&PLAN_MT7921AU, "tune_channel") < at(&PLAN_MT7921AU, "monitor_rx"));
    assert!(
        declares_follow(&PLAN_MT7921AU, "monitor_rx", "tune_channel"),
        "PLAN_MT7921AU satisfies tune-before-monitor by position but does not DECLARE it. This is \
         the ordering that made open_named_radio return an error for every MT7921AU caller."
    );
}

/// ★ **The bring-up drain stops before RX is enabled**, on both parts that bracket one.
///
/// Its span is exactly the rungs it covered before the plan existed. Past `monitor_rx` the chip
/// delivers real frames, and a drain thread reading the data pipe would compete with the RX pump
/// for them — which is what making it a `StepOutcome::Guard` would have done, for the life of the
/// process.
#[test]
fn the_bringup_drain_is_bracketed_and_stops_before_rx() {
    for (id, p) in [("mt76x0", &PLAN_MT7610U)] {
        let (start, stop) = (at(p, "rx_drain"), at(p, "rx_drain_stop"));
        assert!(start < stop, "{id}: the drain must start before it stops");
        assert!(
            stop < at(p, "monitor_rx"),
            "{id}: the drain must stop BEFORE monitor RX is enabled"
        );
        assert!(declares_follow(p, "rx_drain_stop", "rx_drain"));
    }
    let p = &PLAN_MT7921AU;
    let (start, stop) = (at(p, "rx_drain"), at(p, "rx_drain_stop"));
    assert!(start < stop);
    assert!(stop < at(p, "monitor_rx"));
    assert!(
        stop < at(p, "tune_channel"),
        "mt7921: the drain must also stop before the tune — if the MCU latched its responses onto \
         the data pipe, a drain running across a channel switch eats the switch's response for a \
         command the firmware actually executed"
    );
}

/// ★ **RTL8821CU: `bb_rx_path_enable` follows the tune, and `hci_usb_cfg` is last.**
///
/// `RX_PSEL_RST` (0x0808 bit 28|29) is pulsed clear during `phy_set_param` and the receiver only
/// runs with bit 29 SET — without it the FA and CRC counters read 0 and no frame reaches USB.
/// `hci_usb_cfg` is last per `rtw_hci_start`'s ordering so the BB table load and the channel set
/// cannot clobber REG_RXDMA_MODE.
#[test]
fn rtl8821c_rx_path_and_hci_ordering_is_declared() {
    for p in [
        &PLAN_8821CU_MONITOR,
        &PLAN_8821CU_FW_STA,
        &PLAN_8821CU_NO_TXEN,
        &PLAN_8821CU_STATION_REGS,
        &PLAN_8821CU_IBSS,
    ] {
        assert!(at(p, "tune_channel") < at(p, "bb_rx_path_enable"));
        assert!(declares_follow(p, "bb_rx_path_enable", "tune_channel"));
        assert_eq!(
            at(p, "hci_usb_cfg"),
            p.steps.len() - 1,
            "{}: hci_usb_cfg must be the LAST rung (rtw_hci_start ordering) so the BB table load \
             and the channel set cannot clobber REG_RXDMA_MODE",
            p.id
        );
    }
}

// ── the four TX-radiate hypotheses ───────────────────────────────────────────

/// ★★ **The four never-scored theories are four named variants, each differing from the canonical
/// plan by exactly one rung** — and each with its own `PlanId`, so its digest cannot collide with
/// a production one.
#[test]
fn the_four_tx_radiate_theories_are_four_named_variants() {
    let canon = ids(&PLAN_8821CU_MONITOR);

    for (v, added) in [
        (Rtl8821cVariant::FwStaEmulate, "fw_sta_emulate"),
        (Rtl8821cVariant::StationRegs, "station_identity_regs"),
        (Rtl8821cVariant::Ibss, "ibss_opmode"),
    ] {
        let got = ids(v.plan());
        let extra: Vec<&&str> = got.iter().filter(|i| !canon.contains(i)).collect();
        assert_eq!(
            extra,
            vec![&added],
            "{v:?} must differ from the canonical plan by exactly one ADDED rung"
        );
        let without: Vec<&&str> = got.iter().filter(|i| canon.contains(i)).collect();
        let canon_refs: Vec<&&str> = canon.iter().collect();
        assert_eq!(
            without, canon_refs,
            "{v:?} must not reorder or drop any canonical rung — the variant is the hypothesis, \
             not a second ladder"
        );
    }

    // The one subtractive variant.
    let no_txen = ids(Rtl8821cVariant::NoTxen.plan());
    assert!(
        !no_txen.contains(&"txen_golden_block"),
        "PLAN_8821CU_NO_TXEN must NOT contain the golden finalize block — it is the A/B for the \
         one hypothesis that ships ENABLED"
    );
    let dropped: Vec<&&str> = canon.iter().filter(|i| !no_txen.contains(i)).collect();
    assert_eq!(dropped, vec![&"txen_golden_block"]);
}

/// Five variants, five distinct plan names — so five distinct `plan_digest`s. A number taken under
/// an untested hypothesis can never be silently compared with a production number.
#[test]
fn every_variant_has_its_own_plan_identity() {
    let names: Vec<&str> = [
        Rtl8821cVariant::Canonical,
        Rtl8821cVariant::FwStaEmulate,
        Rtl8821cVariant::NoTxen,
        Rtl8821cVariant::StationRegs,
        Rtl8821cVariant::Ibss,
    ]
    .iter()
    .map(|v| v.plan().id.name)
    .collect();
    let mut sorted = names.clone();
    sorted.sort_unstable();
    sorted.dedup();
    assert_eq!(
        sorted.len(),
        names.len(),
        "two 8821CU variants share a PlanId name ({names:?}), so their reports would carry the \
         same plan identity and their digests could collide. The whole point of a variant is that \
         a hypothesis run labels itself."
    );
}

/// ★ **Each variant rung says, in its own `why`, that it is an untested hypothesis awaiting a
/// bench session against a witness.**
///
/// §5-M5 says these are to be "scored once against a witness in one bench session, then promoted
/// or deleted". None has been. This test is what stops the wording quietly softening into a claim
/// before that session happens.
#[test]
fn every_tx_radiate_variant_rung_says_it_is_untested() {
    for (v, rung) in [
        (Rtl8821cVariant::FwStaEmulate, "fw_sta_emulate"),
        (Rtl8821cVariant::NoTxen, "txen_golden_block"),
        (Rtl8821cVariant::StationRegs, "station_identity_regs"),
        (Rtl8821cVariant::Ibss, "ibss_opmode"),
    ] {
        // `txen_golden_block` lives in the CANONICAL plan (NoTxen is its removal), so look there.
        let p: &'static Plan<Rtl8821cuBackend> = if v == Rtl8821cVariant::NoTxen {
            &PLAN_8821CU_MONITOR
        } else {
            v.plan()
        };
        let why = p.steps[at(p, rung)].why;
        assert!(
            why.contains("UNTESTED HYPOTHESIS"),
            "`{rung}` does not say it is an UNTESTED HYPOTHESIS. It is one: this part has never \
             been observed to radiate and none of the four theories has been scored against a \
             witness. Do not soften this wording without the bench session."
        );
        assert!(
            why.contains("witness"),
            "`{rung}` does not name the witness that would settle it. Question (B) cannot be \
             answered from this host — only a peer can."
        );
    }
}

/// The canonical plan is what `BringUp::plan` hands out, and the variants are reachable only
/// through `Rtl8821cVariant`. A variant is a hypothesis about the same role, not a role.
#[test]
fn the_trait_hands_out_the_canonical_plan() {
    let p = <Rtl8821cuBackend as BringUp>::plan(Role::TransmitAndReceive)
        .expect("the 8821CU does TransmitAndReceive");
    assert_eq!(p.id.name, "monitor");
    assert!(<Rtl8821cuBackend as BringUp>::plan(Role::ReceiveOnly).is_none());
    assert!(<Rtl8821cuBackend as BringUp>::plan(Role::TransmitOnly).is_none());
    assert_eq!(Rtl8821cVariant::default(), Rtl8821cVariant::Canonical);
}

// ── LAW 1 / LAW 5 / §4, source-level ─────────────────────────────────────────

/// **LAW 1 — nothing inside a bring-up may read the process environment.**
///
/// Checked over the rung bodies of all four driver files. The knobs that used to live inside these
/// ladders (`NDN_RADIO_FORCE_FW`, and the 8821CU's four TX-radiate flags) are read once at the
/// wrapper boundary and arrive as an argument.
///
/// ⚠ Scope, stated rather than implied: this reads the `s_*` rung bodies, not everything they
/// transitively call. Deep transport helpers still read debug-print knobs (`NDN_RADIO_EP_DEBUG`,
/// `NDN_RADIO_MCU_DEBUG`, `NDN_RADIO_MCU_RESP_MS`), and `mt7612::cold_replay` reads the first of
/// them for its own progress output. Those change no register and decide no sequence; the ones
/// that decided a sequence are the ones that moved.
#[test]
fn no_rung_reads_the_environment() {
    for (label, path) in [
        ("mt76x0", "src/mt76x0/mod.rs"),
        ("mt7612", "src/mt7612/mod.rs"),
        ("mt7921", "src/mt7921/mod.rs"),
        ("rtl8821c", "src/rtl8821c/mod.rs"),
    ] {
        let src = std::fs::read_to_string(path).expect(path);
        let rungs = code_only(rung_section(&src));
        for pat in ["std::env::var", "env::var_os", "NDN_RADIO_FORCE_FW"] {
            assert!(
                !rungs.contains(pat),
                "{label}: a rung body contains `{pat}`. LAW 1 — configuration a caller cannot see \
                 is `load_tx_power_info` one level up: two runs that look identical in their own \
                 output. Read it at the wrapper boundary and pass it in."
            );
        }
    }
}

/// **LAW 5**: a rung that polls a hardware completion bit is always `Required`.
#[test]
fn every_completion_poll_is_required() {
    // mt76x0: `firmware_ready` polls XTAL_RDY|PLL_LD and MT_MCU_COM_REG0; `init_hardware` polls
    // WPDMA idle, MAC idle, TX/RX idle and BBP ready.
    for id in ["firmware_ready", "init_hardware"] {
        assert!(matches!(
            PLAN_MT7610U.steps[at(&PLAN_MT7610U, id)].class,
            StepClass::Required
        ));
    }
    // mt7612: `firmware_and_init` polls the FCE completion and the COM_REG0 ready signature.
    assert!(matches!(
        PLAN_MT7612U.steps[at(&PLAN_MT7612U, "firmware_and_init")].class,
        StepClass::Required
    ));
    // mt7921: `firmware_ready` polls FW_N9_RDY after FW_START_REQ.
    assert!(matches!(
        PLAN_MT7921AU.steps[at(&PLAN_MT7921AU, "firmware_ready")].class,
        StepClass::Required
    ));
    // 8821cu: `card_enable` polls each pwrseq entry; `download_firmware` polls the DDMA completion.
    for id in ["card_enable", "download_firmware"] {
        assert!(matches!(
            PLAN_8821CU_MONITOR.steps[at(&PLAN_8821CU_MONITOR, id)].class,
            StepClass::Required
        ));
    }
}

/// §4 — every one of these four parts refuses question (A), and every refusal carries the
/// measurement rather than a shrug. "`Unprovable`, reason quoted" is what §4's table asks for.
#[test]
fn every_transmit_refusal_is_quoted_not_silent() {
    for (part, instruments, reason) in [
        (
            "mt76x0",
            <Mt7610uBackend as BringUp>::tx_instruments().len(),
            <Mt7610uBackend as BringUp>::tx_unprovable_reason(),
        ),
        (
            "mt7612",
            <Mt7612uBackend as BringUp>::tx_instruments().len(),
            <Mt7612uBackend as BringUp>::tx_unprovable_reason(),
        ),
        (
            "mt7921",
            <Mt7921uBackend as BringUp>::tx_instruments().len(),
            <Mt7921uBackend as BringUp>::tx_unprovable_reason(),
        ),
        (
            "rtl8821c",
            <Rtl8821cuBackend as BringUp>::tx_instruments().len(),
            <Rtl8821cuBackend as BringUp>::tx_unprovable_reason(),
        ),
    ] {
        assert_eq!(
            instruments, 0,
            "{part} now declares a TxInstrument but no part in M5 has a probe — the runner would \
             report `names an instrument but implements no probe_tx`, which loses the measured \
             reason. Wire `BringUp::probe_tx` in the same change."
        );
        let reason = reason.unwrap_or_else(|| {
            panic!(
                "{part} declares no instrument and no reason. §4: a refusal is a first-class \
                 success value WITH THE REASON — silence is the defect."
            )
        });
        assert!(
            reason.len() > 80,
            "{part}'s transmit refusal is a shrug, not a measurement: {reason}"
        );
    }
}

/// The four parts name the roles they do and the ones they do not — a named refusal, never a
/// silent downgrade.
#[test]
fn every_part_names_the_roles_it_does_and_the_ones_it_does_not() {
    macro_rules! one_role {
        ($t:ty, $label:literal) => {{
            assert!(
                <$t as BringUp>::plan(Role::TransmitAndReceive).is_some(),
                "{} does not declare its only role",
                $label
            );
            for r in [Role::ReceiveOnly, Role::TransmitOnly] {
                assert!(
                    <$t as BringUp>::plan(r).is_none(),
                    "{} declares {r:?}. One plan is all these parts have ever run: the monitor \
                     rung writes ENABLE_TX and ENABLE_RX in a single register write, so an \
                     RX-only variant would be a ladder this silicon has never seen.",
                    $label
                );
            }
        }};
    }
    one_role!(Mt7610uBackend, "mt76x0");
    one_role!(Mt7612uBackend, "mt7612");
    one_role!(Mt7921uBackend, "mt7921");
    one_role!(Rtl8821cuBackend, "rtl8821c");
}

/// Every plan writes down the stages it deliberately does nothing in — including the ones a
/// reviewer would expect to find and will not (`Calibrate` on connac2, `MacInit` on a replayed
/// part, `Power` on the two with no actuator at all).
#[test]
fn the_notable_absences_are_excluded_in_writing() {
    fn excluded<B: 'static>(p: &'static Plan<B>, s: Stage) -> bool {
        p.excluded.iter().any(|(st, _)| *st == s)
    }
    assert!(excluded(&PLAN_MT7610U, Stage::Power));
    assert!(excluded(&PLAN_MT7610U, Stage::Calibrate));
    assert!(excluded(&PLAN_MT7612U, Stage::MacInit));
    assert!(excluded(&PLAN_MT7612U, Stage::Power));
    assert!(excluded(&PLAN_MT7921AU, Stage::Calibrate));
    assert!(excluded(&PLAN_MT7921AU, Stage::Power));
    assert!(excluded(&PLAN_8821CU_MONITOR, Stage::Power));
    assert!(excluded(&PLAN_8821CU_MONITOR, Stage::Posture));
}

/// ☠ **The MT7612U hazards, as source-level guards.**
///
/// Both of this part's contention actuators are replug-hazardous, and replaying registers to
/// "recover" a quiet MCU has wedged it three times. Neither is a thing a test can prove absent
/// from the whole tree — but both have a specific spelling in this file, and this catches the
/// specific spelling.
#[test]
fn the_mt7612_hazards_are_not_reintroduced() {
    let src = std::fs::read_to_string("src/mt7612/mod.rs").expect("mt7612");
    let rungs = code_only(rung_section(&src));
    assert!(
        !rungs.contains("set_contention"),
        "a MT7612U rung calls `set_contention`. ☠ On the mt76x2 a legal-but-lower cw_min exponent \
         of 2 — the value MEASURED good on every other part in this fleet — killed this radio \
         twice, past the reach of our restore, our cold bring-up and the kernel's own probe. Only \
         `restore_edca_defaults`, which writes the boot window and cannot go below it, is safe here."
    );
    assert!(
        rungs.contains("restore_edca_defaults"),
        "the MT7612U `pin_edca` rung must use `restore_edca_defaults` (boot window, cannot go \
         below it) and nothing else"
    );

    // ★ M8 moved the factory arms out of `open_named_radio` (deleted) into `open_radio`'s
    // per-part functions. The guard follows the code: it now reads `fn open_mt7612u`.
    let fac = std::fs::read_to_string("src/open_radio.rs").expect("open_radio");
    let arm = code_only(
        fac.split_once("fn open_mt7612u(")
            .expect("the MT7612U factory arm")
            .1
            .split_once("\n}\n")
            .expect("the end of the arm")
            .0,
    );
    for banned in ["set_contention", "NDN_POSTURE", "set_channel"] {
        assert!(
            !arm.contains(banned),
            "`open_radio`'s MT7612U arm (`fn open_mt7612u`) contains `{banned}`. ☠ It must not pin \
             EDCA (both actuators are replug-hazardous on this part) and must not re-tune (that \
             replays the whole captured channel op-stream to change nothing). The width override \
             is refused for this part by `KnobSet::NoWidth`, not by an `if` here."
        );
    }
}

/// ☠ **The warm/cold decision uses a round trip on both mt76x02 parts.**
///
/// `MT_MCU_COM_REG0` is a MAILBOX, and deciding from it was MEASURED wrong in BOTH directions on
/// the MT7612U — each mistake costing a physical replug. The MT7610U still read the latch until
/// M5; this is the guard on that fix.
///
/// ⚠ The MT7921AU is deliberately absent from this test: `MT_CONN_ON_MISC`'s `FW_N9_RDY` is
/// latched *hardware* state that nothing but a reset or a power cycle clears, not a mailbox — the
/// argument is written out on `PLAN_MT7921AU`, together with the fact that a connac2 round trip is
/// an OPEN ITEM and not a settled "unnecessary".
#[test]
fn mt76x02_warm_cold_uses_a_round_trip_not_a_latch() {
    for (label, path, rung) in [
        ("mt76x0", "src/mt76x0/mod.rs", "fn s_firmware_ready"),
        ("mt7612", "src/mt7612/mod.rs", "fn s_firmware_and_init"),
    ] {
        let src = std::fs::read_to_string(path).expect(path);
        let body = src
            .split_once(rung)
            .unwrap_or_else(|| panic!("{label}: no `{rung}`"))
            .1
            .split_once("\n}\n")
            .expect("the end of the rung")
            .0;
        assert!(
            body.contains("mcu_responsive()"),
            "{label}: the warm/cold rung does not take a round trip. ☠ Inferring liveness from a \
             status latch was wrong in BOTH directions on the MT7612U and cost a physical replug \
             each time."
        );
    }
    // And the MT7610U's latch reader is now explicitly not the decision.
    let src = std::fs::read_to_string("src/mt76x0/mod.rs").expect("mt76x0");
    assert!(
        src.contains("This is no longer the warm/cold decision"),
        "mt76x0::firmware_running has lost the note saying it decides nothing. It is a MAILBOX \
         read; if it becomes the decision again the part inherits the MT7612U's replug bill."
    );
}
