//! **§6.1 of the bring-up contract — the structural test, and the one that matters.**
//!
//! No **function body** outside `ndn-radio-drivers/src/` may compose its own bring-up ladder out
//! of driver primitives. Three or more is a ladder; two is a probe.
//!
//! ## What M9 changed, and why the file-level version was not enough
//!
//! The M8 draft scanned whole FILES and exempted whole FILES. Both halves were wrong in the same
//! direction — they moved the unit of review away from the thing being reviewed:
//!
//! * **Over-sensitive.** A bench file with `fn arm_a` calling `power_on`, `fn arm_b` calling
//!   `mac_config` and `fn main` calling `iq_calibrate` was reported as a ladder, though no
//!   function composed one. The only cure available was a file exemption, i.e. a hole.
//! * **Under-sensitive, and this is the one that would have cost something.** A file exemption is
//!   *permanent and total*: `rx8733b_bisect.rs` is exempt because its bisect arms must interleave
//!   a capture window with a cut — and that exemption also covered every OTHER function in the
//!   file, forever. A sixteenth private 8812au bring-up added to the bottom of an exempted bench
//!   file was invisible. That is precisely the growth pattern the fleet already lived through.
//!
//! So the offender is a `(file, fn)` pair, and so is the exemption: `const EXEMPT` is
//! `(file, fn, reason)`, per §6.1.
//!
//! ## ☠ The hole a naive per-function rule opens, and how it is closed
//!
//! §6.1 says "any function body … calls three or more". Read literally that is *weaker* than the
//! file-level draft in one direction: a ladder split across a `main` and a helper —
//! `main` downloads the firmware and calls `run(&mut dev)`, `run` does `hw_reset` and
//! `wmi_start` — has two bodies of one and two primitives, and passes. `ath9k_hw_reset.rs` is
//! shaped exactly like that today, and it would have gone from *exempted with a reason* to
//! *invisible*, which is a strictly worse outcome than the over-sensitivity being fixed.
//!
//! So a function's hits are its own **plus** those of every function it calls that is defined in
//! the same file, transitively. Only the outermost offender is reported (a helper called by an
//! offending caller is not listed twice), so the `(file, fn)` a reviewer is asked about is the one
//! that composes the ladder rather than the one that holds a fragment of it. Two self-tests below
//! pin both halves of the scanner, because a guard whose scanner silently stops finding things is
//! indistinguishable from a clean tree.
//!
//! ## Why a source-level scan at all, when M8 also made the rungs `pub(crate)`
//!
//! §6.1 answers this itself: *"after M8 the sub-steps are `pub(crate)`, so this test becomes a
//! belt over a compiler brace — and it stays, because it also catches a step re-exported under
//! `bench` being used outside a bench example."* The compiler stops a **production** crate. This
//! stops a **bench example** quietly growing a sixteenth private 8812au bring-up, which is what
//! actually happened: every one of those files was an example, and every one of them had `bench`.
//!
//! ## What it is guarding against, concretely
//!
//! On 2026-09-03 the shipped node and sixteen bench examples held indistinguishable handles to
//! transmitters **~18-33 dB apart**. The difference was not in any of the rungs. It was that the
//! examples' ladders never called `load_tx_power_info`, so their `set_tx_power` fell through to
//! the raw chip TXAGC axis — and nothing in the code, the logs, or the capability declaration
//! could tell the two regimes apart. Every number those files produced was taken on a different
//! radio than the one that ships.
//!
//! ⚠ The failure message below is §6.1's, **verbatim**, including its "~20 dB" — the contract's
//! own rounding of what M1 later pinned down as 18-33 dB. It is quoted rather than improved so
//! that the guard and the document it implements cannot drift apart; the measured range is in the
//! paragraph above and in `docs/bringup-root-cause-2026-09-03.md`.

use std::path::{Path, PathBuf};

/// The primitives a ladder is made of (contract §6.1's list, verbatim).
const LADDER: &[&str] = &[
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

/// **The exemptions ARE the review**, and each names ONE function, not a file. Each entry must
/// carry a reason a reader can check; an unexplained entry is the thing this file exists to
/// prevent.
const EXEMPT: &[(&str, &str, &str)] = &[
    (
        "ath9k_hw_reset.rs",
        "main",
        "This instrument IS `hw_reset`: it runs the rung in isolation and repeatedly, to answer \
         whether a reset on a live chip recovers it. A plan runs a rung once, in sequence, which \
         is the opposite of the question. ⚠ Listed against `main` and not `run` because the \
         ladder is SPLIT across the two — main downloads the firmware, run does hw_reset and \
         wmi_start — which is the shape the call expansion above exists to see.",
    ),
];

// ─────────────────────────────────────────────────────────────────────────────
// Source scanning — the lexer and the function splitter live in `tests/common/mod.rs`, shared
// with `plan_shape.rs`. They are pinned by the two `the_scanner_*` tests at the bottom of this
// file: a scanner that silently stops finding things reports a clean tree in a voice
// indistinguishable from a genuinely clean one.
// ─────────────────────────────────────────────────────────────────────────────
mod common;
use common::{blank_literals, calls, functions, rust_sources};

/// The ladder primitives a body calls DIRECTLY.
fn own_hits(body: &str) -> Vec<&'static str> {
    LADDER
        .iter()
        .copied()
        .filter(|p| body.contains(&format!(".{p}(")))
        .collect()
}

/// ★ A function's hits **plus every same-file function it reaches**, transitively.
///
/// This is what stops a ladder hiding behind a helper — see the module header. `seen` breaks
/// recursion (a mutually recursive pair is a fixed point, not a hang).
fn effective_hits<'a>(
    fns: &'a [(String, usize, String)],
    idx: usize,
    seen: &mut Vec<usize>,
) -> Vec<&'static str> {
    if seen.contains(&idx) {
        return Vec::new();
    }
    seen.push(idx);
    let mut hits = own_hits(&fns[idx].2);
    for (j, (name, _, _)) in fns.iter().enumerate() {
        if j != idx && calls(&fns[idx].2, name) {
            for h in effective_hits(fns, j, seen) {
                if !hits.contains(&h) {
                    hits.push(h);
                }
            }
        }
    }
    hits.sort_unstable_by_key(|h| LADDER.iter().position(|l| l == h).unwrap_or(usize::MAX));
    hits
}

/// Every offending `(index, hits)` in one file, with helpers suppressed: a function reached from
/// another offender in the same file is a FRAGMENT of that ladder, not a second one.
fn offenders_in(fns: &[(String, usize, String)]) -> Vec<(usize, Vec<&'static str>)> {
    let all: Vec<(usize, Vec<&'static str>)> = (0..fns.len())
        .map(|i| (i, effective_hits(fns, i, &mut Vec::new())))
        .filter(|(_, h)| h.len() >= 3)
        .collect();
    all.iter()
        .filter(|(i, _)| {
            !all.iter()
                .any(|(j, _)| j != i && calls(&fns[*j].2, &fns[*i].0))
        })
        .cloned()
        .collect()
}

fn scanned_files() -> Vec<PathBuf> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let ws = root.parent().expect("workspace root");
    let mut files = Vec::new();
    for d in [
        root.join("examples"),
        root.join("tests"),
        ws.join("ndn-radio"),
        ws.join("ndn-ext"),
        ws.join("ndn-fwd"),
    ] {
        rust_sources(&d, &mut files);
    }
    files.sort();
    files
}

/// ★ **The gate.** Contract §6.1.
#[test]
fn no_consumer_hand_rolls_a_ladder() {
    let files = scanned_files();
    assert!(
        files.len() > 100,
        "only {} sources found — the guard would be vacuous. Did the workspace layout move?",
        files.len()
    );

    let mut offenders: Vec<String> = Vec::new();
    let mut fns_scanned = 0usize;
    for f in &files {
        let name = f.file_name().and_then(|n| n.to_str()).unwrap_or("");
        let Ok(src) = std::fs::read_to_string(f) else {
            continue;
        };
        let blanked = blank_literals(&src);
        let fns = functions(&blanked);
        fns_scanned += fns.len();
        for (i, hits) in offenders_in(&fns) {
            let (func, line, _) = &fns[i];
            if EXEMPT.iter().any(|(fi, fu, _)| *fi == name && fu == func) {
                continue;
            }
            let quoted: Vec<String> = hits.iter().map(|h| format!("`{h}`")).collect();
            offenders.push(format!(
                "  {}:{line}  `{}::{func}` composes its own bring-up ladder from {} driver \
                 primitives ({}).",
                f.display(),
                name,
                hits.len(),
                quoted.join(", "),
            ));
        }
    }
    assert!(
        fns_scanned > 500,
        "only {fns_scanned} function bodies parsed across {} files — the scanner is broken and \
         the guard would be vacuous",
        files.len()
    );

    assert!(
        offenders.is_empty(),
        "\n{}\n\n\
         A hand-rolled ladder is how the fleet ended up with 16 different 8812au bring-ups whose \
         only difference was a ~20 dB power regime nobody could see. Call \
         `open_radio(pid, &sel, &req)`; to depart from the canonical plan use \
         `BringUpRequest::deviation` (which records the departure in the report and the digest); \
         to add a rung, edit the part's `PLAN_*` in `ndn-radio-drivers/src/` where the `why` is \
         required.\n\n\
         (`deviation` is the request FIELD; `BringUpRequest::with_deviation(d)` is the builder \
         that sets it. If a function genuinely must compose a ladder, add a `(file, fn, reason)` \
         entry to `EXEMPT` in this test — the list is the review, and it now exempts ONE function \
         rather than a whole file.)",
        offenders.join("\n")
    );
}

/// Every exemption names a `(file, fn)` pair that exists **and is still an offender**.
///
/// ⚠ Two stale shapes, and the second is the one a file-level list could not have:
/// * the file is gone — an entry guarding nothing;
/// * the file and function are there and the function **no longer composes a ladder** (somebody
///   migrated it to `open_radio` and left the exemption). That entry now silently covers whatever
///   that function grows into next, which is a hole with a reason attached to it.
#[test]
fn the_exemption_list_has_no_stale_entries() {
    let files = scanned_files();
    for (file, func, reason) in EXEMPT {
        assert!(
            reason.trim().len() > 60,
            "exemption {file}::{func} has no real reason; the list IS the review"
        );
        // ⚠ An exemption keys on the BASENAME, so a basename shared by two files would exempt a
        // function in both. Refuse rather than silently widen.
        let matching: Vec<&PathBuf> = files
            .iter()
            .filter(|f| f.file_name().and_then(|n| n.to_str()) == Some(*file))
            .collect();
        assert!(
            matching.len() <= 1,
            "exemption {file}::{func} matches {} files ({matching:?}) — a basename shared across \
             the workspace exempts that function in ALL of them, which is a wider hole than the \
             one that was reviewed",
            matching.len()
        );
        let Some(path) = matching.first().copied() else {
            panic!(
                "exemption {file}::{func} names a file that no longer exists — delete the entry \
                 rather than leaving a hole nobody is watching"
            );
        };
        let src = std::fs::read_to_string(path).expect("read exempted source");
        let blanked = blank_literals(&src);
        let fns = functions(&blanked);
        let idx = fns
            .iter()
            .position(|(n, _, _)| n == func)
            .unwrap_or_else(|| {
                panic!(
                    "exemption {file}::{func} names a function that no longer exists in {} — \
                     delete the entry rather than leaving a hole nobody is watching",
                    path.display()
                )
            });
        let hits = effective_hits(&fns, idx, &mut Vec::new()).len();
        assert!(
            hits >= 3,
            "exemption {file}::{func} is STALE: that function now calls {hits} ladder \
             primitives, so it is no longer an offender. Delete the entry — as written it is a \
             standing licence for whatever that function grows into next, with a reason attached \
             that no longer describes it"
        );
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Testing the test
// ─────────────────────────────────────────────────────────────────────────────
//
// ☠ Two sibling guards in this tree were WRONG ON THEIR FIRST DRAFT, and both were wrong the same
// way — they passed on the defective code:
//
//   * `one_frame_builder` twice: "calls `build_dot11` somewhere" was satisfied by the `other =>`
//     arm of the very `match` whose `RawNdn` arm was hand-rolled;
//   * `worst_receiver_rate` once: its pattern matched a `None => frame::build(..)` arm that every
//     backend has, so the rule was vacuous.
//
// A scanner is exactly where that happens again: it degrades to silence rather than to noise. If
// `blank_literals` mis-lexes one construct and swallows a file, or `calls` stops matching, this
// file reports a clean tree in a voice indistinguishable from a genuinely clean one. So the
// mechanism is pinned against synthetic sources whose right answer is known.

/// The scanner sees a ladder **split across a helper** — the evasion a literal reading of §6.1
/// permits, and the shape `ath9k_hw_reset.rs` already has.
#[test]
fn the_scanner_sees_a_ladder_split_across_a_helper() {
    let src = r#"
fn main() {
    let mut dev = Dev::open().unwrap();
    dev.power_on().unwrap();
    run(&mut dev);
}
fn run(dev: &mut Dev) {
    dev.mac_config().unwrap();
    dev.bb_config().unwrap();
}
fn unrelated(dev: &mut Dev) { dev.rf_config().unwrap(); }
"#;
    let fns = functions(&blank_literals(src));
    assert_eq!(fns.len(), 3, "parsed {fns:?}");

    // Per-function-body-alone, nobody offends: 1, 2 and 1 primitives.
    for (name, _, body) in &fns {
        assert!(
            own_hits(body).len() < 3,
            "{name} offends on its own, so this fixture does not test the split at all"
        );
    }

    let found = offenders_in(&fns);
    assert_eq!(
        found.len(),
        1,
        "expected exactly the outermost offender, got {:?}",
        found
            .iter()
            .map(|(i, h)| (&fns[*i].0, h))
            .collect::<Vec<_>>()
    );
    assert_eq!(
        fns[found[0].0].0, "main",
        "the OUTERMOST function is the one a reviewer is asked about; reporting the helper too would need two exemptions for one ladder"
    );
    assert_eq!(found[0].1, vec!["power_on", "mac_config", "bb_config"]);
}

/// The scanner does **not** see a ladder in prose or in a string. This is what lets the plan-shape
/// tests, this file's own `LADDER`, and every `Step::why` quote the rung names — and it is also
/// what keeps `body_after`'s brace counting from walking into a `"{"`.
///
/// ☠ **The first draft of this test passed with the lexer deliberately broken**, and the reason is
/// worth keeping: its comments sat at file scope, OUTSIDE any function body, so not blanking them
/// changed nothing a body-level scan looks at. Every construct below is therefore INSIDE the body,
/// and each of the five classes carries **exactly three** distinct primitives — one short of
/// nothing and one at the offence threshold — so that mis-lexing any single class on its own
/// trips the assertion.
#[test]
fn the_scanner_ignores_ladders_in_comments_strings_and_lifetimes() {
    let src = r###"
/// Doc at file scope: .power_on(), .mac_config(), .bb_config() — as PROSE.
fn talks_about_a_ladder<'a>(x: &'a str) -> &'a str {
    // line comment INSIDE the body: .power_on() .mac_config() .bb_config()
    /* block comment INSIDE the body: .rf_config() .init_trx() .tssi_setup() */
    const NAMES: &[&str] = &[".hw_reset(", ".wmi_start(", ".mac_init("];
    println!("{} .iq_calibrate() .lc_calibrate() .start_rx_dma( {{", NAMES.len());
    let raw = r##".enable_tx_path(" .setup_monitor_rx( .mac_enable_dma( .init_llt("##;
    let _ = raw;
    let brace = '{';
    let _ = brace;
    x
}
fn afterwards(d: &mut Dev) { d.phy_init().unwrap(); }
"###;
    let fns = functions(&blank_literals(src));
    let names: Vec<&str> = fns.iter().map(|(n, _, _)| n.as_str()).collect();
    assert_eq!(
        names,
        vec!["talks_about_a_ladder", "afterwards"],
        "the unbalanced `{{` in the format string, the `'a` lifetimes and the `'{{'` char literal \
         must not derail the brace counter — if `afterwards` is missing, the first body swallowed \
         it and every later function in a real file would be credited to the wrong one"
    );
    // Each class alone would reach the threshold, so this one assertion fails for whichever
    // class stops being blanked.
    assert!(
        own_hits(&fns[0].2).is_empty(),
        "prose, comments or string literals inside the body were read as CALLS: {:?} — the \
         classes are line comment / block comment / string array / format string / raw string, \
         three primitives each",
        own_hits(&fns[0].2)
    );
    assert!(offenders_in(&fns).is_empty());
    assert_eq!(
        own_hits(&fns[1].2),
        vec!["phy_init"],
        "…and the real call after all that prose is still seen — without this the test would \
         also pass if `own_hits` simply stopped working"
    );
}
