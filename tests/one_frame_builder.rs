//! **One 802.11 builder, mechanically enforced.**
//!
//! ☠ The 190-bit filter change found the SAME defect in two places at once, and both were silent:
//!
//! * `RadioMediumFace::with_wide_bloom` — one definition, **zero call sites**, so the fleet shipped
//!   the narrow filter while the wide one sat in the tree looking implemented;
//! * `libusb_rtl88xx::build_80211` and `rtl8821c::build_80211` — each hand-rolled its own
//!   **3-address** header for `RawNdn` and delegated to `frame::build_dot11` only for other formats.
//!   Neither read `frame.extra` or `frame.htc`. So the a81a/8822E — the fleet's primary NDR radio,
//!   the one carrying live Interest/Data through `ForwarderEngine` — physically could not emit the
//!   filter, and would have failed **silently and correctly**: base region intact, zero false
//!   negatives, zero benefit. Worse than the first case, because it fires *after* someone upstream
//!   has done the right thing.
//!
//! A unit test cannot catch this: these are private methods on backend structs that need a real USB
//! device to construct. So the guard is mechanical and source-level, which is also what makes it
//! cheap enough to keep. Every `build_80211` (and any sibling that builds an 802.11 header) must
//! route through `frame::build_dot11`, and nothing outside `ndn-frame-io` may emit an 802.11 Frame
//! Control field of its own.
//!
//! This is not elegant. It is the kind of guard that would have saved two silent regressions, and it
//! is the reason the HAL seam is named `extra` rather than `addr4`: a backend cannot map the extra
//! region to `addr4 ‖ QoS Control` by accident, and now it cannot bypass the mapping either.

use std::path::{Path, PathBuf};

fn rust_sources(dir: &Path, out: &mut Vec<PathBuf>) {
    let Ok(rd) = std::fs::read_dir(dir) else {
        return;
    };
    for e in rd.flatten() {
        let p = e.path();
        if p.is_dir() {
            rust_sources(&p, out);
        } else if p.extension().is_some_and(|x| x == "rs") {
            out.push(p);
        }
    }
}

/// The body of `fn name(...)` starting at `at`, by brace balance. Good enough for well-formed Rust
/// and deliberately not a parser — a false alarm here is a human reading one function.
fn body_after(src: &str, at: usize) -> &str {
    let Some(open) = src[at..].find('{') else {
        return "";
    };
    let start = at + open;
    let mut depth = 0usize;
    for (i, c) in src[start..].char_indices() {
        match c {
            '{' => depth += 1,
            '}' => {
                depth -= 1;
                if depth == 0 {
                    return &src[start..start + i + 1];
                }
            }
            _ => {}
        }
    }
    &src[start..]
}

/// Every backend's 802.11 builder delegates to the one in `ndn-frame-io`.
#[test]
fn every_backend_builds_its_802_11_frame_through_the_one_builder() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("src");
    let mut files = Vec::new();
    rust_sources(&root, &mut files);
    assert!(
        !files.is_empty(),
        "no driver sources found — the guard would be vacuous"
    );

    let mut offenders = Vec::new();
    let mut checked = 0usize;
    for f in &files {
        let src = std::fs::read_to_string(f).expect("read source");
        let mut from = 0usize;
        while let Some(rel) = src[from..].find("fn build_80211") {
            let at = from + rel;
            from = at + 1;
            checked += 1;
            let body = body_after(&src, at);
            // ⚠ "calls build_dot11 somewhere" is NOT enough, and getting that wrong the first time
            // is instructive: BOTH regressions called it — in the `other =>` arm of a `match
            // self.format`, for every format EXCEPT the `RawNdn` one that carries the filter. The
            // test that only looked for the call passed on the defective code.
            //
            // The tell of a hand-rolled builder is that it reads the frame's own fields to lay out
            // a header. A delegating one never does: it hands the whole `InjectFrame` over.
            let hand_rolled: Vec<&str> =
                ["&frame.dst", "&frame.src", "&frame.addr3", "&frame.extra"]
                    .into_iter()
                    .filter(|pat| body.contains(pat))
                    .collect();
            if !body.contains("build_dot11") || !hand_rolled.is_empty() {
                offenders.push(format!("{}::build_80211 {hand_rolled:?}", f.display()));
            }
        }
    }
    assert!(
        checked >= 2,
        "expected at least the two backends that regressed, found {checked}"
    );
    assert!(
        offenders.is_empty(),
        "these builders lay out an 802.11 header from the frame's own fields instead of handing \
         the whole `InjectFrame` to `frame::build_dot11`, so they cannot carry the filter's pushed \
         header and will drop it SILENTLY (base region intact, zero false negatives, zero \
         benefit): {offenders:?}"
    );
}

/// Nothing outside `ndn-frame-io` **emits** its own 802.11 Frame Control field for a data frame.
///
/// Both regressions began the same way: a literal `extend_from_slice(&[0x08, 0x00 …])` — FC Data,
/// subtype 0 — in a driver, followed by three hand-written addresses. `frame.rs` is the one place
/// allowed to do that; management/action frames built for a specific firmware quirk (the 8821CU
/// probe-request PA path, the ESP-NOW vendor action frame) are named exemptions, listed here so the
/// list itself is the review.
///
/// ⚠ It matches on the **emission shape**, not on the byte pair: a test that *asserts*
/// `&mpdu[0..2] == &[0x08, 0x00]` is checking the shared builder's output and is exactly the kind of
/// assertion this guard wants to see more of. The first draft of this test flagged one of those on
/// `ath9k_htc.rs`, which is how the distinction got written down.
#[test]
fn no_driver_emits_its_own_data_frame_control_field() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("src");
    let mut files = Vec::new();
    rust_sources(&root, &mut files);

    // Named exemptions: functions that build a MANAGEMENT frame on purpose, not a data frame.
    const EXEMPT_FNS: &[&str] = &["build_probe_req"];

    let mut offenders = Vec::new();
    for f in &files {
        let src = std::fs::read_to_string(f).expect("read source");
        let exempt: Vec<&str> = EXEMPT_FNS
            .iter()
            .filter_map(|n| src.find(&format!("fn {n}")).map(|at| body_after(&src, at)))
            .collect();
        for (ln, line) in src.lines().enumerate() {
            let code = line.split("//").next().unwrap_or("");
            let emits = code.contains("extend_from_slice(&[0x08, 0x00")
                || code.contains("extend_from_slice(&[0x88, 0x8");
            if emits && !exempt.iter().any(|b| b.contains(line)) {
                offenders.push(format!("{}:{}", f.display(), ln + 1));
            }
        }
    }
    assert!(
        offenders.is_empty(),
        "a driver is emitting its own 802.11 data Frame Control — build the frame with \
         `frame::build_dot11` instead, or the filter's addr4/QoS/HTC will be dropped silently: \
         {offenders:?}"
    );
}
