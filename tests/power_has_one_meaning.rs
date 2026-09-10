//! **One power call must mean one physical power.** (`docs/bringup-contract.md` §6.2.)
//!
//! ☠ On 2026-09-03 `Rtl8812auBackend::set_tx_power(idx)` meant **two different physical powers**,
//! decided by whether `load_tx_power_info()` had run three calls earlier:
//!
//! * calibration loaded → `index_base(path, rate, ch) + (idx − 63)` ≈ **27** on the ch6 adapter;
//! * not loaded → the function **fell through** to `set_tx_power_raw(idx)` → a flat **63**.
//!
//! Same call, same argument, same `Ok(())`, **~18–33 dB apart**. MEASURED at an AR9271 witness: raw
//! 63 → 2301 frames at −85.6 dBm, raw 55 → **0**. `nav_probe` never loaded calibration and ran hot;
//! `bring_up_monitor` did and ran at the fused base; the node binary went through the calibrated
//! path. So every on-air characterisation done with the bench examples — range, delivery ratio,
//! throughput, rate adaptation, contention — was measured on a transmitter up to ~20 dB hotter than
//! the node that ships, and nothing in the code, the logs or the capability declaration said so.
//!
//! A unit test cannot catch this class: these are inherent methods on backend structs that need a
//! real USB device to construct. So the guard is mechanical and source-level, in the same shape as
//! `tests/one_frame_builder.rs` — recursive `rust_sources`, brace-balanced `body_after`, a
//! `const EXEMPT` list of `(file, fn, reason)` so the exemption list is itself the review, and a
//! failure message that says what will go wrong on air.
//!
//! Three assertions, exactly as the contract specifies them:
//!
//! 1. **No fallthrough** — no `fn set_tx_power*` body may reach a sibling power writer on a
//!    non-error path.
//! 2. **No `std::env` inside a power writer** — a knob whose meaning depends on the environment is
//!    hidden state by another name.
//! 3. **`PowerReference::FusedBase` is constructed only in a driver** — only the part that read the
//!    fuse may claim one.
//!
//! ---
//!
//! ☠☠ **This file's first draft passed on the defect it exists to catch, which is the third time
//! that has happened in this tree** (`one_frame_builder.rs` was wrong twice, both times the same
//! way). MEASURED against two re-introductions of the real bug, both of which compile:
//!
//! * replacing the no-calibration `Err` with `self.write_txagc_flat(idx)` — **4 of 5 assertions
//!   passed**; only the 8812au-specific regression test caught it, and only as a side effect of
//!   `NO_CALIBRATION_MSG` disappearing from the body;
//! * adding an escape hatch *above* the refusal (`if info.is_none() && idx > 40 { raw }`), leaving
//!   the `Err` in place — **all 5 assertions passed**, on a driver where a calibrated
//!   `PowerRequest::Index` again silently lands on the raw axis and returns `Ok`.
//!
//! The hole was that assertion 1 matched a **name list** (`set_tx_power_raw`, `set_tx_power_idx`)
//! that the fix itself had made obsolete: the 8812au's raw writer was renamed to
//! `write_txagc_flat`, so the guard was watching a door that no longer existed. It now matches on
//! **reachability**: a raw writer may be called only from inside a `PowerRequest::Raw` arm, which is
//! the only place a caller has supplied an `RfAuthority`. That is a property of where the call is,
//! not of what it is called, and it survives the next rename.

use std::path::{Path, PathBuf};

// ── the exemption list IS the review ────────────────────────────────────────

/// `(file suffix, fn name — or `"*"` for the whole file, reason)`.
///
/// Every entry is a claim that this code cannot commit the defect, and the reason is the argument.
/// Adding a row is the review; a row without a measured or structural reason is a hole.
const EXEMPT: &[(&str, &str, &str)] = &[
    (
        "src/mt76x0/phy.rs",
        "set_tx_power",
        "SAME AXIS, unit conversion only: the free `set_tx_power` is a dBm→half-dB wrapper over \
         `set_tx_power_half_dbm`, and that is the ONE MT_TX_ALC_CFG_0 writer on this part. There \
         is no second regime to fall through to — mt76x0 resolves every request against the same \
         EEPROM per-rate calibration (`mod.rs::set_tx_power` records `DriverReference`, not \
         `FusedBase`, precisely because it has never separated the two).",
    ),
    (
        "crates/ndn-radio-hal/",
        "*",
        "the type's home: `PowerReference` is DEFINED here and `bringup.rs`'s own unit tests \
         construct every variant, including the two 2026-09-03 runs the report-diff test replays. \
         A definition site is not a claim about an adapter.",
    ),
    (
        "tests/power_has_one_meaning.rs",
        "*",
        "this guard names every forbidden pattern in its own failure text, and would otherwise \
         report itself.",
    ),
];

fn exempt(file: &Path, func: &str) -> Option<&'static str> {
    let p = file.to_string_lossy().replace('\\', "/");
    EXEMPT
        .iter()
        .find_map(|(f, n, why)| (p.contains(f) && (*n == "*" || func.contains(n))).then_some(*why))
}

// ── source walking, exactly as `one_frame_builder.rs` does it ───────────────

fn rust_sources(dir: &Path, out: &mut Vec<PathBuf>) {
    let Ok(rd) = std::fs::read_dir(dir) else {
        return;
    };
    for e in rd.flatten() {
        let p = e.path();
        let name = p.file_name().map(|s| s.to_string_lossy().into_owned());
        if name.as_deref() == Some("target") || name.as_deref() == Some(".git") {
            continue;
        }
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

/// Strip `//` line comments and `///` doc comments, replacing them with spaces so that **byte
/// offsets are preserved** — assertion 1 works in offsets, and a body that *documents* the old
/// defect (`rtl8812au.rs` documents it at length, on purpose) must not be reported as committing it.
///
/// Block comments are rare in this tree and are not stripped; if one ever hides a real call the
/// cost is a human reading one function, the same cost as a false alarm.
fn blank_line_comments(body: &str) -> String {
    let mut out = String::with_capacity(body.len());
    for line in body.split_inclusive('\n') {
        match line.find("//") {
            Some(i) => {
                out.push_str(&line[..i]);
                // Preserve length AND the trailing newline, so offsets and line numbers hold.
                for c in line[i..].chars() {
                    out.push(if c == '\n' { '\n' } else { ' ' });
                }
            }
            None => out.push_str(line),
        }
    }
    out
}

fn manifest() -> &'static Path {
    Path::new(env!("CARGO_MANIFEST_DIR"))
}

fn driver_src() -> PathBuf {
    manifest().join("src")
}

/// Everything downstream of the drivers that could hold a power writer or forge a `FusedBase`.
/// Sibling repos are scanned when present so a developer with a partial checkout still gets the
/// driver-local half of the guard rather than a spurious pass.
fn consumer_roots() -> Vec<PathBuf> {
    let mut roots: Vec<PathBuf> = vec![
        manifest().join("examples"),
        manifest().join("tests"),
        manifest().join("crates"),
        manifest().join("../ndn-radio"),
        manifest().join("../ndn-ext"),
        manifest().join("../ndn-fwd"),
        manifest().join("../zz-bypass-audit"),
    ];
    roots.retain(|r| r.is_dir());
    roots
}

/// Every `fn set_tx_power…` in the drivers **and** in every consumer, as
/// `(file, signature-line, comment-blanked body)`.
///
/// ⚠ Deliberately wider than `src/`: `RadioKnobs::set_tx_power` has 13 impls and three of them live
/// outside this crate (`ndn-phy-wifi`'s `SpyKnobs`, `zz-bypass-audit`'s `KnobLog`, the HAL default).
/// A face is exactly where a re-introduced environment read would go next.
fn power_writers() -> Vec<(PathBuf, String, String)> {
    let mut files = Vec::new();
    rust_sources(&driver_src(), &mut files);
    let driver_files = files.len();
    assert!(
        driver_files > 0,
        "no driver sources found — the guard would be vacuous"
    );
    for r in consumer_roots() {
        rust_sources(&r, &mut files);
    }

    let mut out = Vec::new();
    for f in &files {
        let Ok(src) = std::fs::read_to_string(f) else {
            continue;
        };
        let mut from = 0usize;
        while let Some(rel) = src[from..].find("fn set_tx_power") {
            let at = from + rel;
            from = at + 1;
            let sig = src[at..]
                .lines()
                .next()
                .unwrap_or_default()
                .trim()
                .to_string();
            out.push((f.clone(), sig, blank_line_comments(body_after(&src, at))));
        }
    }
    assert!(
        out.len() >= 20,
        "expected the fleet's power writers to be found (13 `RadioKnobs` impls plus the inherent \
         and PHY-level writers); got {} — the extractor is broken and this guard is vacuous",
        out.len()
    );
    out
}

// ── assertion 1 ─────────────────────────────────────────────────────────────

/// Writers that put a number on an axis **other than** the one the request resolved to, or that
/// bypass calibration entirely. Calling one of these is not wrong; calling one from anywhere except
/// a `PowerRequest::Raw` arm is.
///
/// * `set_tx_power_raw` — the 8812au's, **deleted** 2026-09-03. The name must never come back.
/// * `write_txagc_flat` — what that writer is called now. ★ The rename is why a name-list guard was
///   not enough: see this file's header.
/// * `set_tx_power_idx`, `set_txagc_table` — the 8733b's two TXAGC writers, both MEASURED INERT on
///   that part (efuse `0xc8[7:4] = 4`, `power_track_type = 4`); its real knob is TSSI DE. A
///   `set_tx_power` reaching either would be writing a register that does nothing and reporting it.
/// * `set_tx_power_pct` — `serial_radio`'s percentage axis.
/// * `set_tx_power_half_dbm` — mt76x0's `MT_TX_ALC_CFG_0` ceiling, in half-dB.
const RAW_WRITERS: &[&str] = &[
    "set_tx_power_raw(",
    "write_txagc_flat(",
    "set_tx_power_idx(",
    "set_txagc_table(",
    "set_tx_power_pct(",
    "set_tx_power_half_dbm(",
];

/// Byte spans of every `PowerRequest::Raw` match arm inside `body` — the only place a raw writer may
/// be reached, because it is the only place the caller supplied an [`RfAuthority`].
///
/// Block arms (`Raw { idx, authority } => { … }`) are taken by brace balance from the `=>`; a
/// one-line expression arm (`Raw { idx, .. } => self.write_txagc_flat(*idx)?,`) is taken to the end
/// of its line. Anything else authorises nothing, which fails closed.
fn raw_arm_spans(body: &str) -> Vec<(usize, usize)> {
    let mut spans = Vec::new();
    let mut from = 0usize;
    while let Some(rel) = body[from..].find("PowerRequest::Raw") {
        let at = from + rel;
        from = at + 1;
        let Some(arrow_rel) = body[at..].find("=>") else {
            continue;
        };
        let after = at + arrow_rel + 2;
        let rest = &body[after..];
        let lead = rest.len() - rest.trim_start().len();
        if rest.trim_start().starts_with('{') {
            let arm = body_after(body, after);
            let start = after + lead;
            spans.push((start, start + arm.len()));
        } else {
            let end = rest.find('\n').map(|i| after + i).unwrap_or(body.len());
            spans.push((after, end));
        }
    }
    spans
}

/// ★★ **Assertion 1 — no power knob may fall through to another regime** (LAW 2).
///
/// This is the 2026-09-03 defect stated as a source property. It is checked by **reachability, not
/// by name**: a raw writer may appear only inside a `PowerRequest::Raw` arm.
#[test]
fn no_power_knob_falls_through_to_another_regime() {
    let mut offenders = Vec::new();
    let mut exempted = Vec::new();
    let mut authorised = 0usize;

    for (file, sig, body) in power_writers() {
        let spans = raw_arm_spans(&body);
        for bad in RAW_WRITERS {
            let mut from = 0usize;
            while let Some(rel) = body[from..].find(bad) {
                let at = from + rel;
                from = at + 1;
                // A definition is not a call: `pub fn write_txagc_flat(` inside another body cannot
                // happen in this tree, but `fn <name>(` immediately before the match is cheap to skip.
                if body[..at].trim_end().ends_with("fn") {
                    continue;
                }
                if spans.iter().any(|&(s, e)| at >= s && at < e) {
                    authorised += 1;
                    continue;
                }
                match exempt(&file, &sig) {
                    Some(why) => exempted.push(format!("{}  ::  {sig}  [{why}]", file.display())),
                    None => offenders.push(format!(
                        "{}\n      {sig}\n      reaches `{}` OUTSIDE any `PowerRequest::Raw` arm",
                        file.display(),
                        bad.trim_end_matches('(')
                    )),
                }
            }
        }
    }

    // ⚠ Non-vacuity: the 8812au's `Raw` arm legitimately calls `write_txagc_flat`. If that stops
    // being found, the span finder has broken and every offender is being silently authorised.
    assert!(
        authorised >= 1,
        "no raw-writer call was found inside a `PowerRequest::Raw` arm. `rtl8812au::set_tx_power` \
         has one (`write_txagc_flat`), so either the driver changed shape or `raw_arm_spans` is \
         broken — and a broken span finder AUTHORISES EVERYTHING, which is how this guard passes \
         on the defect it exists to catch."
    );
    let _ = exempted;

    assert!(
        offenders.is_empty(),
        "a TX-power knob reaches a DIFFERENT power regime on a path the caller did not authorise:\n\
         \n    {}\n\n\
         That is the 2026-09-03 defect verbatim. `rtl8812au::set_tx_power` ended with \
         `self.set_tx_power_raw(idx)` whenever `load_tx_power_info()` had not run, so the same \
         call, the same argument and the same `Ok(())` meant EITHER the fused regulatory base \
         (~TXAGC 27 on the ch6 adapter) OR flat chip maximum (63) — ~18-33 dB apart, decided by a \
         step three calls earlier that the caller could not see.\n\n\
         WHAT GOES WRONG ON AIR: the bench examples and the shipped node stop being the same \
         transmitter. MEASURED at an AR9271 witness — raw 63 = 2301 frames at -85.6 dBm, raw 55 = \
         ZERO frames — so a range, delivery-ratio, rate-adaptation or contention number taken with \
         one and applied to the other is wrong by up to ~20 dB, and NOTHING in the code, the logs \
         or the capability declaration says so. It is also a regulatory exposure: above the fused \
         base this can exceed licensed EIRP with no operator having asked for it.\n\n\
         THE FIX IS NOT A RENAME. Reaching the raw axis is legal only from inside a \
         `PowerRequest::Raw` arm, because that is the only place the caller has supplied an \
         `RfAuthority` (which no library code can mint). With no calibration resolved, return \
         `Err(power_unsupported(NO_CALIBRATION_MSG))` — which names `PowerRequest::Raw` and \
         `NDN_RF_UNRESTRICTED` — and let the operator ask for the other regime out loud.",
        offenders.join("\n\n    ")
    );
}

/// ★ **Assertion 1b, the specific regression.** Non-vacuous where assertion 1 is only a ratchet:
/// the 8812au's calibrated arm must *refuse by name*, and the refusal must name the alternative.
#[test]
fn the_8812au_refuses_the_calibrated_scale_without_calibration() {
    let src = std::fs::read_to_string(driver_src().join("rtl8812au.rs")).expect("read rtl8812au");
    let at = src
        .find("pub fn set_tx_power(&self, req: PowerRequest)")
        .expect(
            "rtl8812au::set_tx_power has changed shape — it must take a PowerRequest and return an \
             AppliedPower (bring-up contract §1.3)",
        );
    let body = blank_line_comments(body_after(&src, at));
    assert!(
        body.contains("NO_CALIBRATION_MSG"),
        "rtl8812au::set_tx_power no longer returns the shared no-calibration refusal. With no \
         EFUSE calibration resolved, a Ceiling/Index request MUST be an `Err` that names \
         `PowerRequest::Raw` — never a silent write on the raw axis. On air that is the ~18-33 dB \
         step between the fused base and chip maximum, taken without anyone asking."
    );

    // ★ The `let Some(info) = … else { … }` binding must REFUSE. Checking only that
    // `NO_CALIBRATION_MSG` appears somewhere in the body is what let the second falsification
    // through: an escape hatch added ABOVE an intact refusal passed every assertion in the file.
    let miss = body
        .find("let Some(info)")
        .expect("the calibration binding has changed shape — re-read this guard against it");
    let tail = &body[miss..];
    let arm_end = tail.find("};").map(|i| i + 2).unwrap_or(tail.len());
    assert!(
        tail[..arm_end].contains("return Err("),
        "the no-calibration branch of rtl8812au::set_tx_power does not return an `Err`. It is the \
         single branch whose fallthrough caused the 2026-09-03 defect; it must refuse, not resolve \
         the request some other way."
    );

    // The message must name the alternative, or an operator who hits it learns nothing.
    let msg = std::fs::read_to_string(manifest().join("crates/ndn-radio-hal/src/bringup.rs"))
        .expect("read bringup.rs");
    let at = msg
        .find("pub const NO_CALIBRATION_MSG")
        .expect("NO_CALIBRATION_MSG must exist — it is the fix's own error text");
    let text = &msg[at..(at + 700).min(msg.len())];
    assert!(
        text.contains("PowerRequest::Raw") && text.contains("NDN_RF_UNRESTRICTED"),
        "NO_CALIBRATION_MSG must name BOTH the alternative request (`PowerRequest::Raw`) and how \
         to authorise it (`NDN_RF_UNRESTRICTED`). A refusal that does not say what to do instead \
         is how the raw path gets re-added by hand — which is the defect, arriving by a different \
         door."
    );
}

// ── assertion 2 ─────────────────────────────────────────────────────────────

/// ★ **Assertion 2 — a knob may not branch on the environment** (LAW 3).
///
/// `NDN_AU_TXAGC12` was read from inside `rtl8812au::set_tx_power` and silently turned 10 register
/// writes into 24. MEASURED against a witness receiver: 5 groups = 2704 frames in 11 s (246 f/s),
/// 12 groups = 7 frames in 11 s (1 f/s). A 250× difference on air, invisible in the offered rate
/// (711 vs 2805 f/s reads as a modest difference), decided by a variable nobody at the call site
/// could see.
///
/// ☠ A second instance was found by this test, not by reading: `rtl8821c/phy.rs::set_tx_power` read
/// `NDN_RADIO_TXPWR` from inside the writer — the same shape, a different part. It is now a caller
/// argument.
#[test]
fn no_power_writer_reads_the_environment() {
    const ENV_READS: &[&str] = &["env::var", "std::env", "env!(", "option_env!("];
    let mut offenders = Vec::new();
    for (file, sig, body) in power_writers() {
        for pat in ENV_READS {
            if body.contains(pat) && exempt(&file, &sig).is_none() {
                offenders.push(format!(
                    "{}\n      {sig}\n      reads the environment via `{pat}`",
                    file.display()
                ));
                break;
            }
        }
    }
    assert!(
        offenders.is_empty(),
        "a TX-power writer reads the process environment:\n\n    {}\n\n\
         A knob whose meaning depends on the environment is hidden state by another name — the same \
         disease as `load_tx_power_info`, one level down, and it defeats every other guard here \
         because the source looks single-valued.\n\n\
         WHAT GOES WRONG ON AIR: `NDN_AU_TXAGC12` did exactly this. Read from inside \
         `set_tx_power`, it turned 10 register writes into 24, and MEASURED against a witness that \
         is 246 f/s versus 1 f/s — writing the seven HT2SS/VHT groups STOPS THIS RADIO \
         TRANSMITTING. The offered rate cannot see it (711 vs 2805 f/s), so a run under the wrong \
         variable looks like a link problem, a range problem or a contention problem, and two \
         rounds of careful source-reading got the cause wrong in both directions.\n\n\
         Put it on the request — `PowerRequest`'s `RateGroupPolicy`, or a `BringUpRequest` field — \
         so it appears in the `AppliedPower` the caller gets back and in the bring-up report, where \
         a reader can see WHICH POLICY RAN. A report that hides that the 8812au writes 5 of 12 \
         Jaguar1 groups overclaims.",
        offenders.join("\n\n    ")
    );
}

// ── assertion 3 ─────────────────────────────────────────────────────────────

/// ★ **Assertion 3 — only the part that read the fuse may claim a fused base.**
///
/// `PowerReference::FusedBase` is a claim about *this adapter's* EFUSE. A face or an example
/// constructing one would be asserting a regulatory reference it has no way to know.
#[test]
fn fused_base_is_constructed_only_in_a_driver() {
    let mut files = Vec::new();
    for r in consumer_roots() {
        rust_sources(&r, &mut files);
    }
    assert!(
        !files.is_empty(),
        "no consumer sources found — the guard would be vacuous"
    );

    let mut offenders = Vec::new();
    for f in &files {
        if exempt(f, "*").is_some() {
            continue;
        }
        let Ok(src) = std::fs::read_to_string(f) else {
            continue;
        };
        for (n, line) in src.lines().enumerate() {
            let code = match line.find("//") {
                Some(i) => &line[..i],
                None => line,
            };
            if code.contains("FusedBase {") || code.contains("FusedBase{") {
                offenders.push(format!("{}:{}\n      {}", f.display(), n + 1, code.trim()));
            }
        }
    }
    assert!(
        offenders.is_empty(),
        "`PowerReference::FusedBase` is constructed outside a driver:\n\n    {}\n\n\
         `FusedBase {{ base_index, channel }}` is a claim that THIS adapter's EFUSE was read and \
         that the number written is referenced to its own per-channel regulatory point. Only the \
         code that did the fuse read can make it.\n\n\
         WHAT GOES WRONG ON AIR: nothing — and that is what makes it worse than the ambiguity this \
         type exists to remove. A fabricated `FusedBase` propagates into every `BringUpReport`, \
         every `AppliedPower` and every report `diff` downstream, where it LOOKS AUTHORITATIVE: it \
         is the field that distinguishes ~TXAGC 27 from a flat 63, so the one signal that would \
         reveal a repeat of 2026-09-03 starts agreeing with itself across two different \
         transmitters. Use `PowerReference::DriverReference` (a source string and an honest \
         `slope_db_per_idx: None`), or `ChipRaw` — the mt76x0 and 8733b drivers do exactly this, \
         because neither has separated the adapter's fuse from the vendor's reference.",
        offenders.join("\n\n    ")
    );
}

// ── §6.4's half that belongs with this file ─────────────────────────────────

/// ★ **§6.4: `src/` may not discard an `AppliedPower`.**
///
/// The applied value is the only thing that distinguishes the fused regulatory base from raw chip
/// maximum on this part. Examples may discard it; library code may not.
#[test]
fn library_code_does_not_discard_the_applied_power() {
    // ⚠ **Every `src/` in the tree, not just the drivers'.** The first draft scanned
    // `ndn-radio-drivers/src` alone, which is the one place the defect was already fixed. The
    // discarded knob results that MATTER live downstream — `ndn-phy-wifi/src/factory.rs` and
    // `ndn-fwd`'s `radio_face.rs` each dropped one, and `radio_face.rs`'s was the production
    // forwarder's only TX-power call. A library that is not this crate is still library code.
    let mut files = Vec::new();
    rust_sources(&driver_src(), &mut files);
    for r in consumer_roots() {
        let mut consumer = Vec::new();
        rust_sources(&r, &mut consumer);
        // `examples/` and `tests/` may discard it (§6.4 says so). A `src/` directory may not.
        consumer.retain(|f| {
            let p = f.to_string_lossy().replace('\\', "/");
            p.contains("/src/") && !p.contains("/examples/") && !p.contains("/tests/")
        });
        files.append(&mut consumer);
    }
    let mut offenders = Vec::new();
    for f in &files {
        if exempt(f, "*").is_some() {
            continue;
        }
        let Ok(src) = std::fs::read_to_string(f) else {
            continue;
        };
        for (n, line) in src.lines().enumerate() {
            let code = match line.find("//") {
                Some(i) => &line[..i],
                None => line,
            };
            let t = code.trim();
            // `set_tx_power_dbm` too: it returns the dBm the radio REPORTED APPLYING, which on
            // every part with a real absolute axis (Morse, NRC, LoRa, the C5) differs from the
            // request — the IDF quantises 21 dBm to 20. Dropping it is the same loss of the only
            // value that says what the radio did.
            if (t.starts_with("let _ =") || t.starts_with("let _:"))
                && (t.contains("set_tx_power") || t.contains("set_tx_power_dbm"))
            {
                offenders.push(format!("{}:{}\n      {t}", f.display(), n + 1));
            }
        }
    }
    assert!(
        offenders.is_empty(),
        "library code discards the applied TX power:\n\n    {}\n\n\
         `set_tx_power` returns an `AppliedPower` carrying the resolved `PowerReference`, the \
         registers written and whether the request was clamped. On the RTL8812AU that value is the \
         ONLY thing that distinguishes the fused regulatory base (~27) from raw chip maximum (63) \
         — ~18-33 dB, MEASURED at a witness as 2301 frames versus zero.\n\n\
         WHAT GOES WRONG ON AIR: a discarded refusal is a radio that is not transmitting at the \
         power the caller believes, with an `Ok` path through the code and nothing in the log. \
         Fold it into the bring-up report (`report.with_power(applied)`), or log the refusal; do \
         not drop it.",
        offenders.join("\n\n    ")
    );
}

// ── assertion 1c: the hole assertion 1 could not see ────────────────────────

/// The 8812au's calibration-bypassing writer, under both the name it had and the name it has.
/// Unlike [`RAW_WRITERS`], reaching one of these is a **~18-33 dB** step off the regulatory scale
/// on a part MEASURED to radiate — so the rule is checked over EVERY function in the tree, not just
/// over `fn set_tx_power*`.
const CALIBRATION_BYPASS_WRITERS: &[&str] = &["set_tx_power_raw(", "write_txagc_flat("];

/// ★★ **Assertion 1c — the raw axis is reachable only from a `PowerRequest::Raw` arm, ANYWHERE.**
///
/// ☠ MEASURED 2026-09-03, against this very file: with `write_txagc_flat` `pub`, a five-line
/// example in a *face* crate
///
/// ```ignore
/// let b = Arc::new(Rtl8812auBackend::open()?);
/// b.bring_up_monitor(6)?;
/// b.write_txagc_flat(63)?;          // flat chip maximum, no authority, no report, no warn
/// ```
///
/// compiled and **all five assertions in this file passed**. Assertion 1 only ever inspects bodies
/// of functions called `set_tx_power…`, so a caller that skips the knob entirely is invisible to
/// it — and the `RfAuthority` guarantee, which is supposed to be type-system enforcement, was
/// bypassable by one method call from any crate in the workspace.
///
/// Two things close it, and the compiler is the stronger of the two: `write_txagc_flat` is now
/// `pub(crate)`, so the example above no longer builds. This assertion is the belt — it also covers
/// a *driver-internal* caller (`bring_up_monitor` reaching for the flat write), which visibility
/// cannot stop.
#[test]
fn the_raw_axis_is_reachable_only_from_a_raw_arm() {
    let mut files = Vec::new();
    rust_sources(&driver_src(), &mut files);
    for r in consumer_roots() {
        rust_sources(&r, &mut files);
    }
    assert!(
        files.len() > 50,
        "the source walk found only {} files — this guard would be near-vacuous",
        files.len()
    );

    let mut offenders = Vec::new();
    let mut authorised = 0usize;
    let mut definitions = 0usize;
    for f in &files {
        if exempt(f, "*").is_some() {
            continue;
        }
        let Ok(raw) = std::fs::read_to_string(f) else {
            continue;
        };
        let src = blank_line_comments(&raw);
        // Raw arms are found over the whole file: the property is "this call sits inside a
        // `PowerRequest::Raw` arm", and which function that arm belongs to does not change it.
        let spans = raw_arm_spans(&src);
        for bad in CALIBRATION_BYPASS_WRITERS {
            let mut from = 0usize;
            while let Some(rel) = src[from..].find(bad) {
                let at = from + rel;
                from = at + 1;
                if src[..at].trim_end().ends_with("fn") {
                    definitions += 1;
                    continue;
                }
                if spans.iter().any(|&(s, e)| at >= s && at < e) {
                    authorised += 1;
                    continue;
                }
                let line = src[..at].matches('\n').count() + 1;
                offenders.push(format!(
                    "{}:{line}\n      calls `{}` outside any `PowerRequest::Raw` arm",
                    f.display(),
                    bad.trim_end_matches('(')
                ));
            }
        }
    }

    // ⚠ Non-vacuity, both halves: the writer must still be DEFINED once, and its one legitimate
    // call (the 8812au `Raw` arm) must still be found. If either count falls to zero the scan has
    // broken, and a broken scan reports no offenders — which is exactly how this guard would pass
    // on the defect it exists to catch.
    assert!(
        definitions >= 1 && authorised >= 1,
        "found {definitions} definition(s) and {authorised} authorised call(s) of the 8812au raw \
         writer; expected at least one of each (`write_txagc_flat`, called from \
         `set_tx_power`'s `PowerRequest::Raw` arm). The scanner is broken."
    );

    assert!(
        offenders.is_empty(),
        "the RAW chip TX-power axis is reached without an `RfAuthority`:\n\n    {}\n\n\
         `write_txagc_flat` writes a flat TXAGC index to ten registers with the EFUSE regulatory \
         calibration bypassed. MEASURED at an AR9271 witness: flat 63 = 2301 frames at -85.6 dBm, \
         and the calibrated scale on the same adapter lands at ~27 — ~18-33 dB apart.\n\n\
         WHAT GOES WRONG ON AIR: the caller transmits off the regulatory scale, possibly above \
         licensed EIRP, with no operator having asked and nothing in the report saying so — and \
         any number taken through that path cannot be compared with a number from the shipped \
         node. Route it through `PowerRequest::raw_from_env(idx)`, which needs \
         `NDN_RF_UNRESTRICTED=\"<operator>:<reason>\"` and prints the reason verbatim for the life \
         of the run.",
        offenders.join("\n\n    ")
    );
}
