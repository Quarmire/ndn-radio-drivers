//! Shared source-scanning primitives for the §6 guards (`no_hand_rolled_ladder.rs`,
//! `plan_shape.rs`).
//!
//! ⚠ **Not a parser.** It is a lexer good enough that a paragraph *about* a ladder is not read as
//! one, and good enough that brace counting does not walk into a `"{"`. A false alarm here costs a
//! human one function read; a false SILENCE costs a guard. Both callers therefore pin these
//! functions against synthetic sources whose right answer is known — see
//! `no_hand_rolled_ladder.rs`'s two `the_scanner_*` tests. ☠ The first draft of one of those
//! passed with the lexer deliberately broken, because its fixture put the comments outside a
//! function body.
#![allow(dead_code)]

use std::path::{Path, PathBuf};

pub fn rust_sources(dir: &Path, out: &mut Vec<PathBuf>) {
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

/// **Blank every comment and every string/char literal, preserving byte offsets.**
///
/// ⚠ This is the part the file-level draft did not have, and it is load-bearing twice over:
///
/// 1. a doc paragraph or a `why:` string that *names* a rung must not read as a call to one —
///    otherwise the guard flags the plan-shape tests, whose whole job is to quote rung names;
/// 2. brace counting must not walk into a `"{"`. `body_after` counts braces, and a single
///    unbalanced brace inside a string literal silently swallows the rest of the file into one
///    "function body", which would report every later function's calls against the first one.
///
/// Offsets are preserved (each blanked byte becomes a space, newlines kept) so line numbers and
/// `fn` positions still refer to the real file.
pub fn blank_literals(src: &str) -> String {
    let b = src.as_bytes();
    let mut out = vec![b' '; b.len()];
    let mut i = 0usize;
    let keep = |out: &mut Vec<u8>, i: usize, b: &[u8]| out[i] = b[i];
    while i < b.len() {
        match b[i] {
            b'\n' => {
                out[i] = b'\n';
                i += 1;
            }
            b'/' if i + 1 < b.len() && b[i + 1] == b'/' => {
                while i < b.len() && b[i] != b'\n' {
                    i += 1;
                }
            }
            b'/' if i + 1 < b.len() && b[i + 1] == b'*' => {
                let mut depth = 1usize;
                i += 2;
                while i < b.len() && depth > 0 {
                    if b[i] == b'\n' {
                        out[i] = b'\n';
                    }
                    if i + 1 < b.len() && b[i] == b'/' && b[i + 1] == b'*' {
                        depth += 1;
                        i += 2;
                    } else if i + 1 < b.len() && b[i] == b'*' && b[i + 1] == b'/' {
                        depth -= 1;
                        i += 2;
                    } else {
                        i += 1;
                    }
                }
            }
            // Raw string: r"..." / r#"..."# / br##"..."##
            b'r' | b'b'
                if {
                    let mut j = i;
                    if b[j] == b'b' {
                        j += 1;
                    }
                    j < b.len() && b[j] == b'r' && {
                        let mut k = j + 1;
                        while k < b.len() && b[k] == b'#' {
                            k += 1;
                        }
                        k < b.len() && b[k] == b'"'
                    }
                } =>
            {
                let mut j = i;
                if b[j] == b'b' {
                    j += 1;
                }
                j += 1; // the `r`
                let hashes = {
                    let mut k = j;
                    while k < b.len() && b[k] == b'#' {
                        k += 1;
                    }
                    k - j
                };
                i = j + hashes + 1; // past the opening quote
                let close: Vec<u8> = std::iter::once(b'"')
                    .chain(std::iter::repeat_n(b'#', hashes))
                    .collect();
                while i < b.len() {
                    if b[i] == b'\n' {
                        out[i] = b'\n';
                    }
                    if b[i..].starts_with(&close) {
                        i += close.len();
                        break;
                    }
                    i += 1;
                }
            }
            b'"' => {
                i += 1;
                while i < b.len() {
                    if b[i] == b'\n' {
                        out[i] = b'\n';
                    }
                    if b[i] == b'\\' {
                        i += 2;
                        continue;
                    }
                    if b[i] == b'"' {
                        i += 1;
                        break;
                    }
                    i += 1;
                }
            }
            // Char literal ONLY where it is unambiguously one: `'x'` or `'\x'`. Anything else
            // starting with a tick is a LIFETIME (`'static`, `'a`), which must not swallow to the
            // next tick.
            b'\'' => {
                let is_char = (i + 2 < b.len() && b[i + 1] != b'\\' && b[i + 2] == b'\'')
                    || (i + 3 < b.len() && b[i + 1] == b'\\' && b[i + 3] == b'\'');
                if is_char {
                    i += if b[i + 1] == b'\\' { 4 } else { 3 };
                } else {
                    keep(&mut out, i, b);
                    i += 1;
                }
            }
            _ => {
                keep(&mut out, i, b);
                i += 1;
            }
        }
    }
    String::from_utf8(out).expect("blanking preserves UTF-8 boundaries of retained bytes")
}

/// The body of the item starting at `at`, by brace balance, over ALREADY-BLANKED source.
pub fn body_after(src: &str, at: usize) -> (usize, &str) {
    let Some(open) = src[at..].find('{') else {
        return (at, "");
    };
    let start = at + open;
    let mut depth = 0usize;
    for (i, c) in src[start..].char_indices() {
        match c {
            '{' => depth += 1,
            '}' => {
                depth -= 1;
                if depth == 0 {
                    return (start, &src[start..start + i + 1]);
                }
            }
            _ => {}
        }
    }
    (start, &src[start..])
}

/// Every `fn NAME` in the (blanked) source, with its body and 1-indexed line.
pub fn functions(blanked: &str) -> Vec<(String, usize, String)> {
    let mut out = Vec::new();
    let bytes = blanked.as_bytes();
    let mut from = 0usize;
    while let Some(rel) = blanked[from..].find("fn ") {
        let at = from + rel;
        from = at + 3;
        // `fn` must be a token, not the tail of `defn`/`asfn`.
        if at > 0 && (bytes[at - 1].is_ascii_alphanumeric() || bytes[at - 1] == b'_') {
            continue;
        }
        let rest = &blanked[at + 3..];
        let name: String = rest
            .chars()
            .take_while(|c| c.is_alphanumeric() || *c == '_')
            .collect();
        if name.is_empty() {
            continue;
        }
        let (_, body) = body_after(blanked, at);
        if body.is_empty() {
            continue; // a trait method signature, `fn(..)` pointer type, etc.
        }
        let line = blanked[..at].lines().count();
        out.push((name, line, body.to_string()));
    }
    out
}

/// Does `body` CALL `name` — `name(` not preceded by an identifier char or a `.`? (A `.name(`
/// is a method on some other type; a bare `name(` in the same file is this file's own function.)
pub fn calls(body: &str, name: &str) -> bool {
    let pat = format!("{name}(");
    let b = body.as_bytes();
    let mut from = 0usize;
    while let Some(rel) = body[from..].find(&pat) {
        let at = from + rel;
        from = at + 1;
        let prev = if at == 0 { b' ' } else { b[at - 1] };
        if !(prev.is_ascii_alphanumeric() || prev == b'_' || prev == b'.') {
            return true;
        }
    }
    false
}
