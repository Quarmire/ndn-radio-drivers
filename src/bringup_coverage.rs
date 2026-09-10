//! **The bring-up coverage table** (contract §6.3) — one cell per part × [`Role`], `Provided` or
//! `Excluded` **in writing**. There is no third state, and in particular there is no silent
//! `None`.
//!
//! The defect this closes is [`coverage`](crate::coverage)'s defect one layer down. That table
//! made a missing *trait impl* a visible row; this one makes a missing *sequence* a visible row.
//! Before it, a part that could not do a role said so by returning `None` from
//! [`BringUp::plan`] — sometimes with a paragraph of reasoning beside the match arm
//! (`LibUsbRtl88xxBackend`, `Rtl8812auBackend`, the mt76 family all wrote one), and sometimes with
//! nothing at all: `Nrc7292BringUp` and `Mm6108BringUp` spell their whole answer as
//! `(role == Role::TransmitAndReceive).then_some(&PLAN_NRC7292)`, which refuses two roles without
//! naming either. A caller that asked for [`Role::ReceiveOnly`] on a HaLow radio got
//! [`PlanError::NoPlan`] and no reason, and nothing in the tree said whether that was a decision
//! or an omission.
//!
//! ## What each cell means
//!
//! * **`Provided`** — `<Part as BringUp>::plan(role)` returns `Some`. Verified, not claimed: the
//!   test below *calls* it. A `Provided` cell with no plan behind it fails.
//! * **`Excluded(reason)`** — `plan(role)` returns `None` **on purpose**, and the reason says what
//!   that costs the caller and what would lift it. An empty or perfunctory reason fails.
//!
//! ## ⚠ The one-directional caveat, inherited from `coverage.rs` — and where it moved
//!
//! [`coverage`]'s header warns that its gate *"prevents over-claiming … but cannot catch
//! under-claiming: an `Excluded` cell whose impl quietly lands later adds to neither count, so the
//! stale exclusion survives"*. That is not hypothetical there — the AR9271 `knobs` cell sat
//! `Excluded("no &self RadioKnobs yet")` for the whole life of a seven-method `impl RadioKnobs`.
//!
//! Here **that half is closed, and a different half is not**, and using this table means knowing
//! which is which:
//!
//! * ✅ Over-claiming is caught: `Provided` + `plan(role) == None` fails.
//! * ✅ **Stale exclusion is caught too** — unlike `coverage.rs`. A `Plan` is a value, and
//!   `plan(role)` is a hardware-free function this test can *call*, so `Excluded` + `plan(role) ==
//!   Some` fails. `coverage.rs` cannot do this because Rust gives it no way to ask "does this impl
//!   exist?" and get `false`.
//! * ✅ A part absent from the table entirely is caught, by the source scan
//!   `every_bringup_impl_has_a_coverage_row` in `tests/plan_shape.rs` — the mechanism `coverage.rs`
//!   also lacks.
//! * ☠ **What is NOT caught, and is the caveat in its irreducible form: an exclusion whose PROSE
//!   has gone stale while its `None` stands.** `LibUsbRtl88xxBackend`'s TX-only cell says the
//!   variant "has never been run on this silicon". The day somebody runs it on the bench and does
//!   not come back here, this table states a measured-sounding fact that is false, every gate
//!   stays green, and the next reader believes it. Prose is reviewed, never derived. When you
//!   measure one of these, come here and rewrite the cell; nothing else will.
//!
//! ## ⚠ `Provided` carries no notion of QUALITY, exactly as in `coverage.rs`
//!
//! `Rtl8821cuBackend`'s `TransmitAndReceive` cell is `Provided` and its transmit path has **never
//! been seen at a witness** (440 + 439 injected frames, zero received — see
//! [`coverage::TX_INTENT`]). A cell says a named sequence exists for that role. Whether it
//! radiates is [`BringUpReport::tx`](ndn_radio_hal::BringUpReport) and, in the end, §7: a witness
//! receiver.

use ndn_radio_hal::bringup::Role;

/// One part × role cell.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PlanCoverage {
    /// A named [`Plan`](ndn_radio_hal::Plan) exists for this role.
    Provided,
    /// This part deliberately does not do this role. The reason must say what the caller loses and
    /// what would lift it; a blank fails the test below.
    Excluded(&'static str),
}

use PlanCoverage::{Excluded, Provided};

/// One row of the table: a part, a role, and the ruling.
pub struct Cell {
    /// The type that implements [`BringUp`](ndn_radio_hal::bringup::BringUp), spelled exactly as
    /// `impl BringUp for …` spells it. The source scan in `tests/plan_shape.rs` matches on this
    /// string, so a rename that does not reach here fails there.
    pub part: &'static str,
    pub role: Role,
    pub status: PlanCoverage,
    /// The plan's static name, for `Provided` cells — so the table points at the sequence rather
    /// than merely asserting one exists. `""` for `Excluded`.
    pub plan: &'static str,
}

/// A part whose `BringUp` impl is behind a cargo feature. Its cells are in the table (the table
/// describes the FULL build), and its runtime witness rides the feature — same arrangement as
/// `coverage.rs`'s `#[cfg(feature = "lora")]` witness block, and named here so an *unwitnessed*
/// row cannot be a third, silent state.
pub const FEATURE_GATED_PARTS: &[(&str, &str)] = &[
    ("LoraSerialBackend", "lora"),
    ("SerialRadioBackend", "serial-radio"),
];

/// ★ The table. One cell per part × role; 12 parts × 3 roles.
pub const BRINGUP_COVERAGE: &[Cell] = &[
    // ── AR9271 (ath9k_htc) — the only part with TWO role plans ────────────────────────────────
    Cell {
        part: "Ath9kHtcBackend",
        role: Role::TransmitAndReceive,
        status: Provided,
        plan: "PLAN_AR9271_MONITOR",
    },
    Cell {
        part: "Ath9kHtcBackend",
        role: Role::ReceiveOnly,
        status: Provided,
        plan: "PLAN_AR9271_RX",
    },
    Cell {
        part: "Ath9kHtcBackend",
        role: Role::TransmitOnly,
        status: Excluded(
            "`wmi_start` bundles START_RECV with the TX prerequisites (IC_UPDATE, the vif + node \
             create) in one WMI exchange, so a transmit-only ladder means splitting a firmware \
             command nobody here has split. Lifted by a bench session that shows the vif/node \
             create working without START_RECV; the caller loses nothing today by asking for \
             TransmitAndReceive and not draining the RX pump.",
        ),
        plan: "",
    },
    // ── RTL8733BU — the other part with two role plans, and the reference BringUp impl ────────
    Cell {
        part: "Rtl8733buBackend",
        role: Role::ReceiveOnly,
        status: Provided,
        plan: "PLAN_8733B_MONITOR",
    },
    Cell {
        part: "Rtl8733buBackend",
        role: Role::TransmitAndReceive,
        status: Provided,
        plan: "PLAN_8733B_TX",
    },
    Cell {
        part: "Rtl8733buBackend",
        role: Role::TransmitOnly,
        status: Excluded(
            "`set_monitor` (RCR + the three filter maps) sits in the MIDDLE of the sequence the TX \
             path is built on, and removing it has never been measured on this silicon. Lifted by \
             a bench run showing TX intact with the RCR left at reset. A caller wanting a pure TX \
             blast asks for TransmitAndReceive and sets PumpPolicy::None, which costs it nothing.",
        ),
        plan: "",
    },
    // ── RTL8822E "a81a" ───────────────────────────────────────────────────────────────────────
    Cell {
        part: "LibUsbRtl88xxBackend",
        role: Role::TransmitAndReceive,
        status: Provided,
        plan: "PLAN_A81A",
    },
    Cell {
        part: "LibUsbRtl88xxBackend",
        role: Role::ReceiveOnly,
        status: Excluded(
            "ONE ladder is all this part has ever run: all five deleted openers reached the same \
             sequence. An RX-only variant means deleting `bb_tx_datapath_init` and the cal chain, \
             which has never been run on this silicon — inventing it here is exactly the \
             unmeasured ladder the contract exists to remove. ⚠ PROSE, not a measurement: if \
             somebody runs it, rewrite this cell.",
        ),
        plan: "",
    },
    Cell {
        part: "LibUsbRtl88xxBackend",
        role: Role::TransmitOnly,
        status: Excluded(
            "`monitor_cfg` (the promiscuous RCR) sits in the middle of the sequence the TX path is \
             built on; a TX-only variant means deleting it, and that has never been run on this \
             silicon. Same ruling and same ⚠ as the ReceiveOnly cell above.",
        ),
        plan: "",
    },
    // ── RTL8812AU — the part the whole contract was written for ───────────────────────────────
    Cell {
        part: "Rtl8812auBackend",
        role: Role::TransmitAndReceive,
        status: Provided,
        plan: "PLAN_8812AU_MONITOR",
    },
    Cell {
        part: "Rtl8812auBackend",
        role: Role::ReceiveOnly,
        status: Excluded(
            "One ladder is all this part has ever run: `bring_up_monitor` was the only bring-up in \
             the tree, and the SIXTEEN hand-rolled example ladders were copies of it or deviations \
             from it, never alternatives. An RX-only variant means deleting the cal chain and the \
             power rung — i.e. re-creating, as a role, precisely the uncalibrated regime that made \
             two transmitters in different power REGIMES indistinguishable. Excluded on those grounds, not \
             merely for want of a measurement.",
        ),
        plan: "",
    },
    Cell {
        part: "Rtl8812auBackend",
        role: Role::TransmitOnly,
        status: Excluded(
            "A TX-only variant means lifting the monitor RCR out of the middle of \
             `mac_init_queues`, which has never been run on this silicon. A caller that only \
             transmits asks for TransmitAndReceive and does not start a pump; that is what \
             `tx_flood_8812au` does.",
        ),
        plan: "",
    },
    // ── RTL8821CU — Provided, and SHALLOW; see the module header ──────────────────────────────
    Cell {
        part: "Rtl8821cuBackend",
        role: Role::TransmitAndReceive,
        status: Provided,
        // ⚠ The four `Rtl8821cVariant` plans (FW_STA / NO_TXEN / STATION_REGS / IBSS) are NOT
        // roles — they are hypotheses about this same role, selected through `bring_up_planned`,
        // each reporting its own PlanId and therefore its own digest. `BringUp::plan` returns the
        // canonical one. `tests/plan_shape.rs` shape-checks all five.
        plan: "PLAN_8821CU_MONITOR",
    },
    Cell {
        part: "Rtl8821cuBackend",
        role: Role::ReceiveOnly,
        status: Excluded(
            "This is an INCOMPLETE port whose transmit path is unproven (440 + 439 injected \
             frames, zero at a witness on the same host). Inventing an RX-only variant of a \
             sequence that has never been shown to work end to end would be two unmeasured \
             ladders instead of one. Lifted by the same bench session that lifts its knobs/time \
             exclusions in `coverage::COVERAGE`.",
        ),
        plan: "",
    },
    Cell {
        part: "Rtl8821cuBackend",
        role: Role::TransmitOnly,
        status: Excluded(
            "Same ruling as ReceiveOnly: the one ladder this part has is not yet known to work end \
             to end, so a second one would be a fork of an unvalidated sequence.",
        ),
        plan: "",
    },
    // ── The mt76 / connac2 family ─────────────────────────────────────────────────────────────
    Cell {
        part: "Mt7610uBackend",
        role: Role::TransmitAndReceive,
        status: Provided,
        plan: "PLAN_MT7610U",
    },
    Cell {
        part: "Mt7610uBackend",
        role: Role::ReceiveOnly,
        status: Excluded(
            "`monitor_rx` writes ENABLE_TX and ENABLE_RX in the SAME MAC_SYS_CTRL write, so an \
             RX-only variant means inventing a MAC_SYS_CTRL value this silicon has never been \
             brought up with. Lifted only by measuring that value, and ☠ this part is one of the \
             two where a speculative register write costs a replug.",
        ),
        plan: "",
    },
    Cell {
        part: "Mt7610uBackend",
        role: Role::TransmitOnly,
        status: Excluded(
            "The same single MAC_SYS_CTRL write, from the other side: there is no TX-without-RX \
             value in evidence. Same ☠ replug cost for guessing one.",
        ),
        plan: "",
    },
    Cell {
        part: "Mt7612uBackend",
        role: Role::TransmitAndReceive,
        status: Provided,
        plan: "PLAN_MT7612U",
    },
    Cell {
        part: "Mt7612uBackend",
        role: Role::ReceiveOnly,
        status: Excluded(
            "`monitor_rx` writes ENABLE_TX and ENABLE_RX in one register write, AND the cold path \
             is a captured stream that cannot be partially replayed. ☠☠ This is the part where \
             BOTH contention actuators are hazardous and where a replayed register left the FCE \
             mid-transaction and cost a physical replug — a speculative role here is the most \
             expensive guess in the crate.",
        ),
        plan: "",
    },
    Cell {
        part: "Mt7612uBackend",
        role: Role::TransmitOnly,
        status: Excluded(
            "Same single ENABLE_TX|ENABLE_RX write and same un-splittable cold replay, from the \
             other side. Same ☠☠ replug cost.",
        ),
        plan: "",
    },
    Cell {
        part: "Mt7921uBackend",
        role: Role::TransmitAndReceive,
        status: Provided,
        plan: "PLAN_MT7921AU",
    },
    Cell {
        part: "Mt7921uBackend",
        role: Role::ReceiveOnly,
        status: Excluded(
            "`mac_enable` plus the sniffer commands are what make this part transmit or receive at \
             all, and they are one firmware-mediated sequence. A one-way variant would be an \
             unmeasured ladder against an MCU that must be talked to by round trip, never by \
             status latch.",
        ),
        plan: "",
    },
    Cell {
        part: "Mt7921uBackend",
        role: Role::TransmitOnly,
        status: Excluded(
            "Same firmware-mediated `mac_enable` + sniffer sequence, from the other side; the same \
             round-trip rule applies to any attempt to split it.",
        ),
        plan: "",
    },
    // ── The serial bridges ────────────────────────────────────────────────────────────────────
    Cell {
        part: "SerialRadioBackend",
        role: Role::TransmitAndReceive,
        status: Provided,
        plan: "PLAN_SERIAL_BRIDGE",
    },
    Cell {
        part: "SerialRadioBackend",
        role: Role::ReceiveOnly,
        status: Excluded(
            "★ The honest kind of exclusion: the 7E-A5 firmware brings both directions up together \
             and exposes NO opcode to bring up one without the other. A one-way role here would be \
             a claim about a radio this crate does not control — not an unmeasured ladder but an \
             unimplementable one. Lifted by a firmware change, not a driver change.",
        ),
        plan: "",
    },
    Cell {
        part: "SerialRadioBackend",
        role: Role::TransmitOnly,
        status: Excluded(
            "Same: no 7E-A5 opcode brings up one direction alone. Lifted by a firmware change, not \
             a driver change.",
        ),
        plan: "",
    },
    Cell {
        part: "LoraSerialBackend",
        role: Role::TransmitAndReceive,
        status: Provided,
        plan: "PLAN_LORA_NODE",
    },
    Cell {
        part: "LoraSerialBackend",
        role: Role::ReceiveOnly,
        status: Excluded(
            "LoRa is half-duplex on one carrier and the firmware returns to RX after every \
             transmit, so 'receive-only' is not a state the part can be left in — it is what the \
             part does between transmits. Claiming the role would be a claim about a radio this \
             crate does not control.",
        ),
        plan: "",
    },
    Cell {
        part: "LoraSerialBackend",
        role: Role::TransmitOnly,
        status: Excluded(
            "Same half-duplex single carrier: the firmware drops back to RX after every frame, so \
             a transmit-only role cannot be held. Lifted by a firmware mode this protocol version \
             does not have.",
        ),
        plan: "",
    },
    // ── HaLow — the two parts whose refusals were written NOWHERE before this table ───────────
    Cell {
        part: "Nrc7292BringUp",
        role: Role::TransmitAndReceive,
        status: Provided,
        plan: "PLAN_NRC7292",
    },
    Cell {
        part: "Nrc7292BringUp",
        role: Role::ReceiveOnly,
        status: Excluded(
            "★ This part brings NOTHING up: every rung is `OutOfBand`, established by `modprobe` / \
             `iw` / `hostapd_s1g` in a shell before this process starts. A role is a statement \
             about a sequence this plan runs, and there is no sequence here to vary — the monitor \
             vif either exists or it does not. Splitting the validation by role would say we \
             configured a direction we did not configure. Lifted only if the driver stops being \
             out of band (an nl80211 path).",
        ),
        plan: "",
    },
    Cell {
        part: "Nrc7292BringUp",
        role: Role::TransmitOnly,
        status: Excluded(
            "Same: nothing here is brought up in-process, so there is no direction to bring up on \
             its own. And ⚠ the out-of-tree injection patch this part needs is `NOT validated` — \
             500 sends, chip TX-OK +0 — so a role NAMED transmit-only would be the most misleading \
             cell in the table.",
        ),
        plan: "",
    },
    Cell {
        part: "Mm6108BringUp",
        role: Role::TransmitAndReceive,
        status: Provided,
        plan: "PLAN_MM6108",
    },
    Cell {
        part: "Mm6108BringUp",
        role: Role::ReceiveOnly,
        status: Excluded(
            "★ Every rung is `OutOfBand`, as on the NRC7292 — and here a one-way role would be \
             actively wrong: on the MM6108 it is the TX MONITOR vif that turns RECEIVE on, which \
             is why `PLAN_MM6108` asserts BOTH interfaces' `/sys/class/net/<if>/type`. A cell \
             claiming receive-only would name the exact rule whose violation gives zero frames and \
             no error.",
        ),
        plan: "",
    },
    Cell {
        part: "Mm6108BringUp",
        role: Role::TransmitOnly,
        status: Excluded(
            "Same out-of-band ruling, plus: the unpatched driver HARD-LOCKS the board on \
             injection, so nothing in this crate may present a transmit-only HaLow role as a \
             configuration it established.",
        ),
        plan: "",
    },
];

/// Render the table (printed by the test, so the artifact is visible in test output).
pub fn render() -> String {
    let mut out = String::from(
        "part                    role                  status  plan\n\
         ----------------------- --------------------- ------- ---------------------------\n",
    );
    for c in BRINGUP_COVERAGE {
        out.push_str(&format!(
            "{:<24}{:<22}{:<8}{}\n",
            c.part,
            format!("{:?}", c.role),
            match c.status {
                Provided => "yes",
                Excluded(_) => "EXCL",
            },
            c.plan,
        ));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use ndn_radio_hal::bringup::BringUp;

    /// Every cell the ACTIVE feature set can evaluate: `(part, role, plan(role).is_some())`.
    ///
    /// ★ This is the half `coverage.rs` cannot have. A `Plan` is a value and `plan(role)` is a
    /// pure function, so the witness is a *call*, and it answers in both directions — which is
    /// why the stale-exclusion hole is closed here and open there.
    fn witnesses() -> Vec<(&'static str, Role, bool)> {
        let mut w = Vec::new();
        macro_rules! witness {
            ($name:literal, $t:ty) => {
                for role in [
                    Role::ReceiveOnly,
                    Role::TransmitOnly,
                    Role::TransmitAndReceive,
                ] {
                    w.push(($name, role, <$t as BringUp>::plan(role).is_some()));
                }
            };
        }
        witness!("Ath9kHtcBackend", crate::Ath9kHtcBackend);
        witness!("Rtl8733buBackend", crate::Rtl8733buBackend);
        witness!("LibUsbRtl88xxBackend", crate::LibUsbRtl88xxBackend);
        witness!("Rtl8812auBackend", crate::Rtl8812auBackend);
        witness!("Rtl8821cuBackend", crate::Rtl8821cuBackend);
        witness!("Mt7610uBackend", crate::Mt7610uBackend);
        witness!("Mt7612uBackend", crate::Mt7612uBackend);
        witness!("Mt7921uBackend", crate::Mt7921uBackend);
        witness!("Nrc7292BringUp", crate::halow::Nrc7292BringUp);
        witness!("Mm6108BringUp", crate::halow::Mm6108BringUp);
        #[cfg(feature = "serial-radio")]
        witness!("SerialRadioBackend", crate::SerialRadioBackend);
        #[cfg(feature = "lora")]
        witness!("LoraSerialBackend", crate::LoraSerialBackend);
        w
    }

    /// ★ **The gate.** Every cell is `Provided` or excluded IN WRITING; every cell agrees with
    /// what `BringUp::plan` actually returns, in BOTH directions; every `Provided` names its plan;
    /// no part × role appears twice.
    #[test]
    fn every_part_and_role_is_provided_or_excluded_in_writing() {
        let w = witnesses();
        let mut seen = std::collections::HashSet::new();
        let mut unwitnessed: Vec<&str> = Vec::new();

        for c in BRINGUP_COVERAGE {
            assert!(
                seen.insert((c.part, format!("{:?}", c.role))),
                "{} / {:?} appears twice in BRINGUP_COVERAGE — two rulings for one cell",
                c.part,
                c.role
            );
            match c.status {
                Provided => assert!(
                    !c.plan.is_empty(),
                    "{} / {:?} is Provided but names no plan — the table must point at the \
                     sequence, not merely assert one exists",
                    c.part,
                    c.role
                ),
                Excluded(reason) => {
                    assert!(
                        reason.trim().len() > 80,
                        "{} / {:?} is excluded without a real written reason ({} chars). A bare \
                         `None` from `BringUp::plan` is what this table exists to replace: the \
                         caller gets `PlanError::NoPlan` and no way to tell a decision from an \
                         omission",
                        c.part,
                        c.role,
                        reason.trim().len()
                    );
                    assert!(
                        c.plan.is_empty(),
                        "{} / {:?} is Excluded but names a plan",
                        c.part,
                        c.role
                    );
                }
            }

            match w.iter().find(|(p, r, _)| *p == c.part && *r == c.role) {
                Some((_, _, has_plan)) => match c.status {
                    Provided => assert!(
                        *has_plan,
                        "{} / {:?} is `Provided` and `BringUp::plan` returns None — the table \
                         claims a sequence the code does not have",
                        c.part, c.role
                    ),
                    Excluded(_) => assert!(
                        !*has_plan,
                        "{} / {:?} is `Excluded` and `BringUp::plan` returns Some — a STALE \
                         EXCLUSION. Somebody added the ladder and left the ruling saying it had \
                         never been run, which is the failure mode that let the AR9271 `knobs` \
                         cell read `Excluded(\"no &self RadioKnobs yet\")` through the whole life \
                         of a seven-method impl. Rewrite the cell",
                        c.part, c.role
                    ),
                },
                None => unwitnessed.push(c.part),
            }
        }

        // An unwitnessed row is allowed ONLY for a part whose impl is behind a cargo feature that
        // is currently off — and only for a part NAMED as such. Otherwise it is a third, silent
        // state, which is the thing this table refuses to have.
        unwitnessed.sort_unstable();
        unwitnessed.dedup();
        for part in &unwitnessed {
            let (_, feat) = FEATURE_GATED_PARTS
                .iter()
                .find(|(p, _)| p == part)
                .unwrap_or_else(|| {
                    panic!(
                        "{part} has cells but no runtime witness, and is not in \
                         FEATURE_GATED_PARTS — an unwitnessed row is a cell nothing checks"
                    )
                });
            println!("(feature `{feat}` off: {part} cells declared, not witnessed)");
        }
        println!("{}", render());
    }

    /// Every feature-gated part named above really is gated — i.e. with the feature ON it IS
    /// witnessed. Without this, a stale `FEATURE_GATED_PARTS` entry would grant a permanent
    /// exemption to a part that is no longer gated at all.
    #[test]
    fn the_feature_gated_list_has_no_stale_entries() {
        let w = witnesses();
        for (part, feat) in FEATURE_GATED_PARTS {
            assert!(
                BRINGUP_COVERAGE.iter().any(|c| c.part == *part),
                "{part} is listed as feature-gated but has no cells in BRINGUP_COVERAGE"
            );
            let witnessed = w.iter().any(|(p, _, _)| p == part);
            let on = match *feat {
                "lora" => cfg!(feature = "lora"),
                "serial-radio" => cfg!(feature = "serial-radio"),
                other => panic!("FEATURE_GATED_PARTS names unknown feature `{other}`"),
            };
            assert_eq!(
                witnessed, on,
                "{part} is declared gated on `{feat}` (currently {on}) but its witness is \
                 {witnessed} — the gate and the list disagree, so the exemption is not the one \
                 that was reviewed"
            );
        }
    }
}
