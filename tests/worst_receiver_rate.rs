//! **A stored rate may never silence a `MostRobust` frame.**
//!
//! ☠ This guard exists because the rule was taught to ten backends and **one was missed**, and the
//! miss was found from a deployment, not from a test:
//!
//! `AfPacketBackend::inject` built its radiotap header at `cur_mcs` whenever a rate had been set —
//! and `ndn-fwd` attaches a rate actuator to *every* bearer, af-packet included, so `cur_mcs` is
//! always `Some` on a real node. From that moment every frame went at the cognition MCS, including
//! the cooperative reports and control traffic that exist precisely so the **worst** receiver in
//! range can decode them (doctrine §5; 802.11 sends beacons and probes at a basic rate for the same
//! reason). The observable symptom was a one-way link: the only traffic that survived was the 1 Hz
//! `/localhop/radio/report/<node>` reports coming from the *other* leg, whose radio did honour the
//! intent.
//!
//! The reason it was missed is structural and will recur: `ndn-frame-io` is a different crate from
//! the one the coverage ratchet walks. So the rule is enforced here, mechanically, across every
//! `inject` in the workspace.
//!
//! **The rule.** If an `inject` body reads a *stored* rate (`cur_mcs`, `cur_rate`, `build_at`), it
//! must also consult the frame's *intent* (`needs_basic_rate`, `desc_rate_for`, `for_intent`, or
//! `frame::build(`, which resolves the intent itself). A backend that has no rate concept at all
//! trips nothing. Exemptions are listed below with a reason, so the list is itself the review.

use std::path::{Path, PathBuf};

fn rust_sources(dir: &Path, out: &mut Vec<PathBuf>) {
    let Ok(rd) = std::fs::read_dir(dir) else {
        return;
    };
    for e in rd.flatten() {
        let p = e.path();
        if p.is_dir() {
            if p.file_name().is_some_and(|n| n == "target") {
                continue;
            }
            rust_sources(&p, out);
        } else if p.extension().is_some_and(|x| x == "rs") {
            out.push(p);
        }
    }
}

/// The body of the function whose signature starts at `at`, by brace balance. Deliberately not a
/// parser: a false alarm here costs one human reading one function.
fn body_after(src: &str, at: usize) -> &str {
    let Some(open) = src[at..].find('{') else {
        return "";
    };
    let start = at + open;
    let (mut depth, mut end) = (0usize, start);
    for (i, c) in src[start..].char_indices() {
        match c {
            '{' => depth += 1,
            '}' => {
                depth -= 1;
                if depth == 0 {
                    end = start + i + 1;
                    break;
                }
            }
            _ => {}
        }
    }
    &src[start..end]
}

/// `(file suffix, why it may read a stored rate without consulting the intent)`.
const EXEMPT: &[(&str, &str)] = &[(
    "crates/ndn-radio-hal/src/lib.rs",
    "trait definitions and doc examples, not a backend transmit path",
)];

/// Reading one of these means the body is choosing a rate from BEARER STATE.
const STORED_RATE: &[&str] = &["cur_mcs", "cur_rate", "build_at(", "inject_at("];
/// Any one of these means the body makes an actual DECISION from the frame's intent.
///
/// ⚠ **`frame::build(` is deliberately NOT in this list, and the first draft of this test had it
/// there and PASSED ON THE REAL DEFECT.** Every backend has a `None => frame::build(..)` arm for the
/// no-stored-rate case, so matching on it made the rule vacuous: the `Some(mcs)` arm could ignore
/// the intent entirely and the body still "mentioned" the intent-resolving call. The rule has to
/// find the *decision*, not a sibling match arm. (`one_frame_builder` was wrong twice in this same
/// way — a guard that has never been seen to fail is not a guard.)
const HONOURS_INTENT: &[&str] = &["needs_basic_rate", "desc_rate_for", "for_intent"];

#[test]
fn a_stored_rate_never_overrides_a_most_robust_frame() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let mut files = Vec::new();
    rust_sources(&root.join("src"), &mut files);
    rust_sources(&root.join("crates"), &mut files);

    let mut offenders = Vec::new();
    for f in &files {
        let rel = f
            .strip_prefix(root)
            .unwrap_or(f)
            .to_string_lossy()
            .replace('\\', "/");
        if EXEMPT.iter().any(|(suffix, _)| rel.ends_with(suffix)) {
            continue;
        }
        let Ok(src) = std::fs::read_to_string(f) else {
            continue;
        };
        let mut from = 0usize;
        while let Some(hit) = src[from..].find("async fn inject(") {
            let at = from + hit;
            let body = body_after(&src, at);
            from = at + body.len().max(1);
            if body.is_empty() {
                continue;
            }
            let reads_state = STORED_RATE.iter().any(|k| body.contains(k));
            let asks_intent = HONOURS_INTENT.iter().any(|k| body.contains(k));
            if reads_state && !asks_intent {
                offenders.push(rel.clone());
            }
        }
    }

    assert!(
        offenders.is_empty(),
        "these `inject` implementations pick a rate from BEARER STATE without ever consulting the \
         frame's intent: {offenders:?}\n\n\
         A `MostRobust` frame — cooperative reports, discovery, control — must reach the WORST \
         receiver in range, so it may not ride the cognition MCS. This exact defect shipped in \
         `AfPacketBackend::inject` and produced a one-way link on a real deployment: a rate \
         actuator is attached to every bearer, so the stored rate was always set, and every robust \
         frame was quietly demoted to a rate the far node could not decode.\n\n\
         Fix: gate on `frame.tx.needs_basic_rate()` and fall back to `frame::build(..)` (which \
         resolves the intent), as `rtl8812au::desc_rate_for` does. If this backend genuinely has no \
         rate concept, add it to EXEMPT with the reason."
    );
}
