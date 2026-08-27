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

use fw_tier0::{FILL_CAP, K, M_BITS, MAX_DEPTH, PrefixFilter, wide_fields};

/// One `wide` row of the golden file: (label, key, name, id, flags, expected addr1..addr4 ‖ htc).
struct WideRow {
    label: String,
    key: [u8; 16],
    name: String,
    id: u8,
    flags: u8,
    fp: u32,
    addr1: [u8; 6],
    addr2: [u8; 6],
    addr3: [u8; 6],
    addr4: [u8; 6],
    htc: [u8; 4],
}

fn hex6(s: &str) -> [u8; 6] {
    let mut a = [0u8; 6];
    for (i, b) in a.iter_mut().enumerate() {
        *b = u8::from_str_radix(&s[2 * i..2 * i + 2], 16).expect("hex6");
    }
    a
}

fn wide_vectors() -> Vec<WideRow> {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/golden/tier0/vectors.txt");
    let text = std::fs::read_to_string(path).expect("golden vectors present");
    let mut rows = Vec::new();
    for line in text.lines() {
        let Some(rest) = line.strip_prefix("wide ") else {
            continue;
        };
        // wide <label> <key> <name> <id-hex> <flags-hex> <fp-hex> <addr1><addr2><addr3><addr4> <htc>
        let f: Vec<&str> = rest.split_whitespace().collect();
        assert_eq!(f.len(), 8, "wide row shape");
        let mut key = [0u8; 16];
        key.copy_from_slice(f[1].as_bytes());
        let bytes = f[6]; // 24 bytes = addr1..addr4 concatenated
        rows.push(WideRow {
            label: f[0].into(),
            key,
            name: f[2].into(),
            id: u8::from_str_radix(f[3], 16).expect("id"),
            flags: u8::from_str_radix(f[4], 16).expect("flags"),
            fp: u32::from_str_radix(f[5], 16).expect("fp"),
            addr1: hex6(&bytes[0..12]),
            addr2: hex6(&bytes[12..24]),
            addr3: hex6(&bytes[24..36]),
            addr4: hex6(&bytes[36..48]),
            htc: {
                let mut h = [0u8; 4];
                for (i, b) in h.iter_mut().enumerate() {
                    *b = u8::from_str_radix(&f[7][2 * i..2 * i + 2], 16).expect("htc");
                }
                h
            },
        });
    }
    rows
}

/// **The firmware wide-profile port reproduces every `wide` golden row** — base Blur in addr1..addr3,
/// the additive extra Blur in addr4, and the 24-bit fingerprint + marker in HT Control. A divergence
/// here is a silent false negative between a Wi-Fi wide sender and an LR2021 wide receiver.
#[test]
fn firmware_regenerates_every_wide_row() {
    let rows = wide_vectors();
    assert!(!rows.is_empty(), "at least one wide row present");
    for r in &rows {
        let f = wide_fields(&r.key, r.name.as_bytes(), r.id, r.flags);
        let got_fp = f.htc[0] as u32 | (f.htc[1] as u32) << 8 | (f.htc[2] as u32) << 16;
        assert_eq!(got_fp, r.fp, "wide row '{}' fingerprint", r.label);
        assert_eq!(f.addr1, r.addr1, "wide row '{}' addr1", r.label);
        assert_eq!(f.addr2, r.addr2, "wide row '{}' addr2", r.label);
        assert_eq!(f.addr3, r.addr3, "wide row '{}' addr3", r.label);
        assert_eq!(f.addr4, r.addr4, "wide row '{}' addr4 (extra Blur)", r.label);
        assert_eq!(f.htc, r.htc, "wide row '{}' HT Control", r.label);
    }
}

/// Print the firmware's wide fields for the canonical inputs — the source of truth pinned into
/// `vectors.txt`. Run with `--ignored --nocapture` to (re)generate the `wide` row.
#[test]
#[ignore]
fn emit_wide_golden_row() {
    let key: [u8; 16] = *b"ndr/tier0-vec-01";
    let f = wide_fields(&key, b"/ndn/test/v1", 0x37, 0x00);
    let fp = f.htc[0] as u32 | (f.htc[1] as u32) << 8 | (f.htc[2] as u32) << 16;
    let cat = |a: &[u8]| a.iter().map(|b| format!("{b:02x}")).collect::<String>();
    println!(
        "wide widev1 ndr/tier0-vec-01 /ndn/test/v1 37 00 {:06x} {}{}{}{} {}",
        fp,
        cat(&f.addr1),
        cat(&f.addr2),
        cat(&f.addr3),
        cat(&f.addr4),
        cat(&f.htc)
    );
}

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
        let Some(rest) = line.strip_prefix("row ") else {
            continue;
        };
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
    assert!(
        right_f.may_match(&mask),
        "the right key matches its own prefix mask"
    );

    let mut wrong_f = PrefixFilter::default();
    wrong_f.insert_name(&wrong.key, wrong.name.as_bytes());
    assert!(
        !wrong_f.may_match(&mask),
        "the same name under another key must not match"
    );

    // F1: an over-full filter is inert here too. A copy that skips the cap is a hole even if every
    // other byte agrees, which is why the cap is a vector parameter and not an implementation choice.
    let all_ones = PrefixFilter([0xff; 16]);
    assert!(all_ones.popcount() > FILL_CAP);
    assert!(
        !all_ones.may_match(&mask),
        "the amplified universal wake is dead in the firmware copy"
    );
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
        .arg(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/golden/tier0/vectors.txt"
        ))
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
