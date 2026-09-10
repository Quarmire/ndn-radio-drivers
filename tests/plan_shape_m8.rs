//! **M8 — the consumers, and the doors that no longer exist.** Hardware-free.
//!
//! What this can check: that there is exactly ONE factory, that the six deleted openers are gone
//! from the whole workspace, that `BringUpRequest::from_env` is the only place a bring-up's
//! configuration is read from the environment, that every ladder rung is behind `rung!`, and that
//! the five refuted `usb_probe` flags are deleted with their refutations written into the plan
//! steps they were testing.
//!
//! What it cannot check, and this is the important half: **that any of it still radiates.** M8
//! moved every consumer onto one sequence; whether that sequence is the right one is an on-air
//! question, and the acceptance for the part where it matters most is the "M7 acceptance" section
//! of `docs/bringup-contract.md`.

use std::path::{Path, PathBuf};

fn ws() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("workspace root")
        .to_path_buf()
}

fn rust_sources(dir: &Path, out: &mut Vec<PathBuf>) {
    let Ok(rd) = std::fs::read_dir(dir) else {
        return;
    };
    for e in rd.flatten() {
        let p = e.path();
        if p.is_dir() {
            if p.file_name().is_some_and(|n| n == "target" || n == ".git") {
                continue;
            }
            rust_sources(&p, out);
        } else if p.extension().is_some_and(|x| x == "rs") {
            out.push(p);
        }
    }
}

fn workspace_sources() -> Vec<PathBuf> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let w = ws();
    let mut files = Vec::new();
    for d in [
        root.join("src"),
        root.join("examples"),
        root.join("tests"),
        root.join("crates"),
        w.join("ndn-radio"),
        w.join("ndn-ext"),
        w.join("ndn-fwd"),
    ] {
        rust_sources(&d, &mut files);
    }
    assert!(
        files.len() > 200,
        "only {} sources found — the guard would be vacuous",
        files.len()
    );
    files
}

/// Strip `//` comments so a paragraph *about* a deleted function is not read as a call to one.
fn code_only(src: &str) -> String {
    src.lines()
        .map(|l| match l.find("//") {
            Some(i) => &l[..i],
            None => l,
        })
        .collect::<Vec<_>>()
        .join("\n")
}

/// ★ **The six deleted openers stay deleted.**
///
/// Each of these was a way to get a radio that either discarded the [`BringUpReport`], claimed the
/// first device on the bus, or spelled its own ladder. `open_radio(pid, &sel, &req)` is the one
/// door. Re-adding any of them re-opens the divergence M8 closed, so it fails here rather than in
/// six months' worth of on-air numbers that cannot be compared.
#[test]
fn the_deleted_openers_stay_deleted() {
    const GONE: &[(&str, &str)] = &[
        (
            "open_named_radio(",
            "the PID-dispatching factory. Replaced by `open_radio(pid, &sel, &req)`, which takes a \
             `BringUpRequest` instead of a bare channel and returns `BringUpFailure` (carrying the \
             PARTIAL report) instead of `FaceError`.",
        ),
        (
            "open_ath9k(",
            "the AR9271 arm. It is now an arm of `open_radio`, and its `NDN_ATH9K_*` knobs are \
             `BringUpRequest`/`PartOpts` fields read in ONE place.",
        ),
        (
            ".bring_up_monitor(",
            "the RTL8812AU / RTL8733BU monitor wrappers. The role is now named at the call site: \
             `bring_up_planned(ch, Role::…, …)`.",
        ),
        (
            ".bring_up_tx(",
            "the 8733b one-shot TX wrapper. `Role::TransmitAndReceive` IS `PLAN_8733B_TX`, and the \
             plan's `Guards` (the `PowerTracker`) come back to the caller instead of being leaked \
             inside a wrapper that had nowhere to put them.",
        ),
        (
            ".bring_up_tx_tracked(",
            "same, plus the tracker. `bring_up_planned(...)?.1` is the guard set.",
        ),
        (
            ".bring_up_tx_until(",
            "the retry-until-a-witness-agrees wrapper. It is `ProofRequirement::WitnessOrFail` + a \
             `WitnessOracle` on the request — §4, where a witness belongs.",
        ),
        (
            "open_monitor(",
            "`LibUsbRtl88xxBackend::open_monitor` / `Rtl8821cuBackend::open_monitor`: claim the \
             FIRST Realtek on the bus and discard the report. §5-M4 says deleted, not deprecated — \
             a bench script must now name a device, which is the point.",
        ),
        (
            "open_monitor_pid(",
            "same, with a PID. Use `open_pid` + `bring_up_planned`, or `open_radio`.",
        ),
        (
            "open_monitor_pid_select(",
            "same, with a selector. Use `open_pid_select` + `bring_up_planned`, or `open_radio`.",
        ),
    ];
    let files = workspace_sources();
    let mut offenders: Vec<String> = Vec::new();
    for f in &files {
        // This test names them all on purpose, and so does the migration note in `open_radio.rs`.
        let name = f.file_name().and_then(|n| n.to_str()).unwrap_or("");
        if name == "plan_shape_m8.rs" {
            continue;
        }
        let Ok(src) = std::fs::read_to_string(f) else {
            continue;
        };
        let code = code_only(&src);
        for (pat, why) in GONE {
            if code.contains(pat) {
                offenders.push(format!("  {} calls `{pat}` — {why}", f.display()));
            }
        }
    }
    assert!(
        offenders.is_empty(),
        "a deleted bring-up door is back:\n{}",
        offenders.join("\n")
    );
}

/// ★ **LAW 1 — one reader.** No bring-up configuration is read from the environment anywhere but
/// `BringUpRequest::from_env`.
///
/// The list is the knobs the six deleted openers used to read, each from a different place. A knob
/// read where the caller cannot see it is `load_tx_power_info` one level up: configuration that
/// silently decides what a later call means.
#[test]
fn only_from_env_reads_the_bring_up_knobs() {
    // ⚠ Two exceptions, both written down elsewhere and both narrower than they look:
    //   * `start_rx_dma` reads three `NDN_RX*` aggregation knobs one call deep — pinned by
    //     `plan_shape_8812au::the_start_rx_dma_env_exception_does_not_grow`, which lets that set
    //     shrink and never grow;
    //   * `Rtl8821cVariant::from_env` and the two `*_env_deviation()` helpers read their own
    //     part-specific knobs. They are CALLED BY `from_env` and are its per-part halves; a bench
    //     instrument that drives `bring_up_planned` directly calls them for the same reason.
    const KNOBS: &[&str] = &[
        "NDN_RADIO_BW",
        "NDN_TX_PWR",
        "NDN_NO_PUMP",
        "NDN_ASYNC_PUMP",
        "NDN_RX_PUMP_DEPTH",
        "NDN_TX_PUMP",
        "NDN_CCA_OFF",
        "NDN_ATH9K_FW",
        "NDN_ATH9K_HIGHPWR",
        "NDN_ATH9K_NORMPWR",
        "NDN_ATH9K_HT40",
        "NDN_ATH9K_SETBOARD",
        "NDN_ATH9K_NO_CAL",
        "NDN_ATH9K_PUMP",
        "NDN_ATH9K_RX_ONLY",
        "NDN_8733B_RX_ONLY",
        "NDN_RADIO_FORCE_FW",
        "NDN_RADIO_TX_2T",
        "NDN_RADIO_RX_ONLY",
        "NDN_BRINGUP_DEVIATE",
    ];
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("src");
    let mut files = Vec::new();
    rust_sources(&root, &mut files);
    let mut offenders: Vec<String> = Vec::new();
    for f in &files {
        // `open_radio.rs` is `BringUpRequest::from_env` and its per-knob helpers — the one place.
        if f.file_name().and_then(|n| n.to_str()) == Some("open_radio.rs") {
            continue;
        }
        let Ok(src) = std::fs::read_to_string(f) else {
            continue;
        };
        let code = code_only(&src);
        for k in KNOBS {
            // Look for an actual READ, not a mention: `env::var("K")` / `env::var_os("K")`. The
            // knob names appear all over the rungs' `why` strings on purpose, and a `why` that
            // names the knob it replaced is the documentation this contract asks for.
            for form in [format!("var(\"{k}\")"), format!("var_os(\"{k}\")")] {
                if code.contains(&form) {
                    offenders.push(format!("  {} reads {k}", f.display()));
                }
            }
        }
    }
    assert!(
        offenders.is_empty(),
        "a bring-up knob is read outside `BringUpRequest::from_env`:\n{}\n\n\
         LAW 1: every `NDN_*` a bring-up depends on becomes a request field or a `Deviation`, read \
         once at the boundary where the caller can see it. Configuration a caller cannot see is \
         the `load_tx_power_info` defect one level up.",
        offenders.join("\n")
    );
}

/// ★ **§5-M8 — every ladder rung is `pub(crate)` unless `feature = "bench"`.**
///
/// Checked at the source, because the compiler only proves it for the configuration it was invoked
/// with: a `cargo check --all-features` build has `bench` on and would accept a bare `pub fn
/// power_on`.
#[test]
fn every_rung_is_declared_through_the_rung_macro() {
    const RUNGS: &[&str] = &[
        "power_on",
        "download_firmware",
        "mac_config",
        "mac_enable_dma",
        "mac_init_queues",
        "init_llt",
        "bb_config",
        "rf_config",
        "iq_calibrate",
        "lc_calibrate",
        "start_rx_dma",
        "init_trx",
        "enable_tx_path",
        "tssi_setup",
        "phy_init",
        "hw_reset",
        "wmi_start",
        "setup_monitor_rx",
        "mac_init",
    ];
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("src");
    let mut files = Vec::new();
    rust_sources(&root, &mut files);
    let mut offenders: Vec<String> = Vec::new();
    let mut found = 0usize;
    for f in &files {
        let Ok(src) = std::fs::read_to_string(f) else {
            continue;
        };
        for r in RUNGS {
            if src.contains(&format!("pub fn {r}(")) {
                offenders.push(format!("  {}: `pub fn {r}`", f.display()));
            }
            if src.contains(&format!("fn {r}(")) && src.contains("rung! {") {
                found += 1;
            }
        }
    }
    assert!(
        found >= 30,
        "only {found} rung declarations found inside `rung! {{ … }}` — did the macro get removed, \
         or the backends renamed?"
    );
    assert!(
        offenders.is_empty(),
        "these ladder rungs are unconditionally `pub`:\n{}\n\n\
         §5-M8: a rung is `pub` only under `feature = \"bench\"`. Wrap it in `rung! {{ … }}` (see \
         `lib.rs`) so a production consumer cannot compose its own bring-up out of them — which is \
         how the fleet ended up with sixteen RTL8812AU ladders and a ~20 dB power regime nobody \
         could see.",
        offenders.join("\n")
    );
}

/// ★ **The five refuted `usb_probe` flags are gone, and their refutations are IN the plan.**
///
/// §5-M8: *"a flag encoding a refuted hypothesis → deleted, with the refutation written into the
/// step it was testing."* Deleting the flag alone would just let the next person re-derive it.
#[test]
fn the_refuted_usb_probe_flags_are_deleted_and_their_refutations_recorded() {
    let files = workspace_sources();
    for flag in [
        "--txfix",
        "--bbfix",
        "--replayh2c",
        "--replayinit",
        "--useinit",
        "--forcebb",
        "--forcemac",
    ] {
        for f in &files {
            let name = f.file_name().and_then(|n| n.to_str()).unwrap_or("");
            // This test and the two replacement instruments name them on purpose, to say they are
            // gone and why.
            if matches!(name, "plan_shape_m8.rs" | "bringup_probe.rs" | "regs.rs") {
                continue;
            }
            let Ok(src) = std::fs::read_to_string(f) else {
                continue;
            };
            assert!(
                !src.contains(&format!("\"{flag}\"")),
                "{} still implements the refuted flag {flag}. Its hypothesis was tested against a \
                 witness and failed; the refutation lives in the `why` of the plan step it was \
                 probing, in `src/libusb_rtl88xx.rs`.",
                f.display()
            );
        }
    }
    let a81a = std::fs::read_to_string(
        Path::new(env!("CARGO_MANIFEST_DIR")).join("src/libusb_rtl88xx.rs"),
    )
    .expect("read the a81a driver");
    for (flag, step) in [
        ("--replayh2c", "send_general_info"),
        ("--replayinit", "phy_init"),
        ("--useinit", "phy_init"),
        ("--forcebb", "phy_init"),
        ("--txfix", "mac_init"),
        ("--forcemac", "mac_init"),
        ("--bbfix", "bb_tx_datapath_init"),
    ] {
        assert!(
            a81a.contains(flag),
            "the refutation of {flag} is not written into the plan. It belongs in the `why` of \
             `{step}` — the rung it was testing — so the next person reads the answer instead of \
             re-running the experiment."
        );
    }
    assert!(
        a81a.matches("☠ REFUTED").count() >= 4,
        "the refuted-hypothesis notes have gone missing from PLAN_A81A's rungs"
    );
}

/// `usb_probe.rs` is split, not ported. Both halves exist and neither is a bring-up.
#[test]
fn usb_probe_is_split_into_a_plan_runner_and_a_register_surface() {
    let ex = ws().join("ndn-radio/crates/faces/ndn-phy-wifi/examples");
    // This asserts about the ndn-radio SIBLING repo; when it is not checked out
    // (e.g. this crate's own CI, which only stages ndn-rs + ndn-ext), there is
    // nothing to check — a missing reader is not a violation.
    if !ex.exists() {
        return;
    }
    assert!(
        !ex.join("usb_probe.rs").exists(),
        "usb_probe.rs is back. It was 1084 lines and ~27 flags under the doc comment \"List USB \
         devices\" — 27 deviations wearing a trench coat, and already a hand-written plan \
         interpreter. §5-M8 splits it into `bringup_probe.rs` (the plan, with --skip / \
         --stop-after / --poke) and `regs.rs` (the register surface)."
    );
    for half in ["bringup_probe.rs", "regs.rs"] {
        assert!(
            ex.join(half).exists(),
            "the `{half}` half of the usb_probe split is missing"
        );
    }
    // `regs.rs` needs a live radio for half its arms — it must get one from the PLAN.
    let regs = std::fs::read_to_string(ex.join("regs.rs")).expect("read regs.rs");
    assert!(
        regs.contains("bring_up_planned("),
        "`regs.rs` must bring the radio up through the part's plan. A register read taken after a \
         DIFFERENT sequence is a reading of a different radio — which is exactly how sixteen \
         private ladders came to be compared with each other."
    );
}

/// The one factory exists, takes a request, and returns the failure that carries the partial report.
#[test]
fn open_radio_is_the_one_door() {
    let src =
        std::fs::read_to_string(Path::new(env!("CARGO_MANIFEST_DIR")).join("src/open_radio.rs"))
            .expect("read open_radio.rs");
    assert!(
        src.contains("pub fn open_radio(\n    pid: u16,\n    sel: &DeviceSelect,\n    req: &BringUpRequest,\n) -> Result<OpenRadio, BringUpFailure> {"),
        "`open_radio`'s signature has changed. §1.7: `open_radio(pid, &sel, &req) -> \
         Result<OpenRadio, BringUpFailure>`. The error type is not incidental — it is what lets a \
         failed open say HOW FAR IT GOT."
    );
    // Every arm must go through the part's plan, never through a hand-composed sequence.
    let arms = [
        "open_ar9271",
        "open_rtl8733b",
        "open_mt7610u",
        "open_mt7921au",
        "open_mt7612u",
        "open_rtl8821cu",
        "open_rtl8822e",
        "open_rtl8812au",
    ];
    for a in arms {
        assert!(
            src.contains(&format!("fn {a}(")),
            "`open_radio` lost its `{a}` arm — an undispatchable PID must be a NAMED error, never \
             a silent fall-through onto the first dongle on the bus."
        );
    }
    assert_eq!(
        src.matches("bring_up_planned(").count(),
        arms.len(),
        "every arm of `open_radio` runs its part's plan through `bring_up_planned`, exactly once. \
         A count that has changed means an arm composes something else."
    );
}

/// §5-M8's own warning, kept where a reader will find it: the three production paths that
/// bypassed `open_named_radio` gain four behaviours they silently did without.
#[test]
fn the_behaviour_change_is_written_down() {
    let src =
        std::fs::read_to_string(Path::new(env!("CARGO_MANIFEST_DIR")).join("src/open_radio.rs"))
            .expect("read open_radio.rs");
    for phrase in [
        "Behaviour changes M8 makes, on purpose",
        "NDN_CCA_OFF",
        "PowerTracker",
        "ndn-fwd::radio_face",
    ] {
        assert!(
            src.contains(phrase),
            "`open_radio`'s doc comment no longer names `{phrase}`. §5-M8 asks for these to be \
             said out loud: they are changes a DEPLOYED node will see, and the right direction is \
             not the same as no change."
        );
    }
}
