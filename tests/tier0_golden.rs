//! Cross-check the **LR2021 firmware** Tier-0 copy against the shared golden vectors (F2 / P0.2).
//!
//! Three implementations of this filter exist — ndn-ext `tier0.rs` (which generates the vectors),
//! this firmware copy, and the ath9k-htc C copy (checked by `tools/ndr_vectors_test.c`). They agree
//! today by having been edited in sync, which is not a guarantee, and a divergence surfaces on air
//! as a **silent false negative**: two nodes in one group stop matching, with nothing logged.
//!
//! **Why the source is `include!`d rather than imported.** The firmware crate pins
//! `thumbv8m.main-none-eabihf` in its `.cargo/config.toml`, so `cargo test` there builds for the
//! device and cannot run a host harness. `tier0.rs` is pure integer code with no `use`, no
//! `crate::` references and no defmt, so mounting the real source with `#[path]` tests **the bytes
//! that ship on the device** without restructuring the firmware build or maintaining a second copy
//! — a copy being the exact problem this file exists to prevent.
//!
//! This copy has `insert_name`, so unlike the C receive-only copy it regenerates each row's wire
//! bytes from the name. That is the strongest form of the check: hash, position mapping, prefix
//! enumeration, depth cap and bit layout all have to agree, not just the match logic.

#[allow(dead_code, clippy::all)]
#[path = "../firmware/lr2021-nrf54l15-rs/src/tier0.rs"]
mod fw_tier0;

use fw_tier0::{FILL_CAP, K, M_BITS, MAX_DEPTH, PrefixFilter};

struct Row {
    label: String,
    key: [u8; 16],
    name: String,
    wire: [u8; 16],
    popcount: u32,
}

fn vectors() -> (Vec<Row>, (u32, u32, usize, u32)) {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/golden/tier0/vectors.txt");
    let text = std::fs::read_to_string(path).expect("golden vectors present");
    let mut rows = Vec::new();
    let mut params = None;
    for line in text.lines() {
        if let Some(rest) = line.strip_prefix("params ") {
            let get = |k: &str| -> u64 {
                rest.split_whitespace()
                    .find_map(|f| f.strip_prefix(k)?.parse::<u64>().ok())
                    .unwrap_or_else(|| panic!("params field {k} missing"))
            };
            params = Some((
                get("k=") as u32,
                get("m=") as u32,
                get("max_depth=") as usize,
                get("fill_cap=") as u32,
            ));
            continue;
        }
        let Some(rest) = line.strip_prefix("row ") else { continue };
        let f: Vec<&str> = rest.split_whitespace().collect();
        assert_eq!(f.len(), 5, "row shape: label key name wire popcount");
        let mut key = [0u8; 16];
        key.copy_from_slice(f[1].as_bytes());
        let mut wire = [0u8; 16];
        for (i, b) in wire.iter_mut().enumerate() {
            *b = u8::from_str_radix(&f[3][2 * i..2 * i + 2], 16).expect("hex");
        }
        rows.push(Row {
            label: f[0].into(),
            key,
            name: f[2].into(),
            wire,
            popcount: f[4].parse().expect("popcount"),
        });
    }
    (rows, params.expect("params header present"))
}

/// **The parameter header is itself a vector.** If the file and this implementation disagree about
/// k, M, the depth cap or the fill cap, the two are not speaking the same protocol — and every
/// downstream byte comparison would be explaining a symptom rather than the cause.
#[test]
fn firmware_params_match_the_vectors() {
    let (_, (k, m, depth, cap)) = vectors();
    assert_eq!(k, K, "k");
    assert_eq!(m, M_BITS, "M");
    assert_eq!(depth, MAX_DEPTH, "MAX_DEPTH");
    assert_eq!(cap, FILL_CAP, "FILL_CAP");
}

/// Every row regenerated from its name must reproduce the recorded wire bytes exactly.
#[test]
fn firmware_regenerates_every_vector_row() {
    let (rows, _) = vectors();
    assert!(rows.len() >= 4, "all rows read");
    for r in &rows {
        let mut f = PrefixFilter::default();
        f.insert_name(&r.key, r.name.as_bytes());
        assert_eq!(
            f.to_wire(),
            r.wire,
            "row '{}' ({}) diverged: firmware produced {:02x?}, vectors say {:02x?}. This is a \
             Tier0Params change or an implementation drift — on air it would be a SILENT false \
             negative between a Wi-Fi node and an LR2021 node in the same group.",
            r.label,
            r.name,
            f.to_wire(),
            r.wire
        );
        assert_eq!(f.popcount(), r.popcount, "row '{}' popcount", r.label);
    }
}

/// The keying and fill-cap properties the rows exist to pin, asserted against this copy directly.
#[test]
fn firmware_honours_keying_and_the_fill_cap() {
    let (rows, _) = vectors();
    let find = |l: &str| rows.iter().find(|r| r.label == l).expect("row present");

    // A different group key over the same name must not match (doctrine §8: the filter is keyed).
    // Both sides are rebuilt from name+key rather than lifted from the recorded bytes, so this
    // asserts the hash and position mapping directly instead of a wire round-trip.
    let wrong = find("wrongkey");
    let right = find("depth2");
    assert_eq!(wrong.name, right.name, "fixture: the two rows share a name");
    let mask = PrefixFilter::mask_for(&right.key, b"/ndn");

    let mut right_f = PrefixFilter::default();
    right_f.insert_name(&right.key, right.name.as_bytes());
    assert!(right_f.may_match(&mask), "the right key matches its own prefix mask");

    let mut wrong_f = PrefixFilter::default();
    wrong_f.insert_name(&wrong.key, wrong.name.as_bytes());
    assert!(!wrong_f.may_match(&mask), "the same name under another key must not match");

    // F1: an over-full filter is inert here too. A copy that skips the cap is a hole even if every
    // other byte agrees, which is why the cap is a vector parameter and not an implementation choice.
    let all_ones = PrefixFilter([0xff; 16]);
    assert!(all_ones.popcount() > FILL_CAP);
    assert!(!all_ones.may_match(&mask), "the amplified universal wake is dead in the firmware copy");
}

/// **The ath9k C copy, compiled and run from here** — so one `cargo test` is the gate for all three
/// implementations rather than two of them plus a command somebody has to remember.
///
/// The C copy is the one furthest from anyone's daily build: it is compiled by a CMake fragment
/// dropped into the vendor SDK, so nothing in this workspace would otherwise touch it, and drift
/// there is the most likely and the least visible. `ndr_tier0.h` already carried an `NDR_HOST_TEST`
/// branch for exactly this purpose; this is what finally calls it.
///
/// **Missing compiler is a failure, not a skip.** A skipped conformance test restores precisely the
/// state this file exists to remove — an implementation nobody checked — while reporting green.
#[test]
fn ath9k_c_copy_matches_the_vectors() {
    use std::process::Command;

    let root = concat!(env!("CARGO_MANIFEST_DIR"), "/firmware/ath9k-htc-ndr");
    let bin = std::env::temp_dir().join("ndr_vectors_test_from_cargo");

    let build = Command::new("cc")
        .args(["-DNDR_HOST_TEST", "-Isrc", "-O1", "-o"])
        .arg(&bin)
        .args(["tools/ndr_vectors_test.c", "src/ndr_tier0.c"])
        .current_dir(root)
        .output()
        .expect(
            "a C compiler is required: the ath9k Tier-0 copy is a shipping implementation of the \
             wire format, and leaving it unchecked is the exact condition these vectors exist to \
             remove",
        );
    assert!(
        build.status.success(),
        "ath9k C copy failed to compile:\n{}",
        String::from_utf8_lossy(&build.stderr)
    );

    let run = Command::new(&bin)
        .arg(concat!(env!("CARGO_MANIFEST_DIR"), "/golden/tier0/vectors.txt"))
        .output()
        .expect("run the C vector check");
    let out = String::from_utf8_lossy(&run.stdout);
    assert!(
        run.status.success(),
        "ath9k C copy disagrees with the golden vectors:\n{out}{}",
        String::from_utf8_lossy(&run.stderr)
    );
    assert!(out.contains("0 failure"), "unexpected C output: {out}");
}
