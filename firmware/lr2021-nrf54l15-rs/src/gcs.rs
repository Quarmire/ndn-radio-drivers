//! **GCS-in-frame** — the Golomb-Coded Set prefix-set filter for bit-starved, sequential-decode
//! bearers (LoRa / FLRC), where the filter rides the frame **body** rather than address fields.
//!
//! **Vendored verbatim from `ndn-radio-cognition::gcs`** (the shared host home), the same way
//! [`crate::tier0`] vendors the address Blur — the FLRC PHY has no address fields to hide a filter in,
//! so on this bearer the prefix-set filter is a GCS in the body (the cascade's `FLAG_BODY_PREFIX`
//! tier; `wire-format-spec.md` §2a). Byte-identical to the host codec: same keyed-SipHash keyspace
//! (#44, via [`crate::tier0::siphash24`]), same prefix-walk / clamp / depth cap, same
//! **zero-false-negative** guarantee. Keep this file in lockstep with the host `gcs.rs`, or the
//! shadow RX's `false_neg` counter is the only thing that will tell you they drifted.
//!
//! Wire: `[n: u8][Rice-coded sorted gaps, MSB-first]`. `n` fixes the hash range `U = n·2^P`; the Rice
//! parameter `P` sets `ε ≈ 2^-P`.

use crate::tier0::siphash24;

/// Rice/Golomb parameter (`M = 2^P`, `ε ≈ 2^-P`). Shared wire parameter — every implementation must
/// agree, like the Blur's `k`/`m`.
pub const GCS_P: u32 = 8;
/// Deepest prefix inserted — identical to the Blur's cap so the two structures cover the same set.
pub const MAX_DEPTH: usize = 8;
/// The self-signaling TLV type for the body-prefix filter on a bearer with no address fields.
/// `[TYPE][len:u8][gcs bytes]`. Distinct from an LP packet's first byte (`0x64`) and Interest/Data
/// (`0x05`/`0x06`) so its presence is unambiguous. The on-wire realization of `FLAG_BODY_PREFIX`.
pub const BODY_PREFIX_TLV: u8 = 0xF5;

const MAX_PREFIXES: usize = MAX_DEPTH + 1;
/// Body-field capacity for one name's GCS.
pub const GCS_MAX_BYTES: usize = 24;

/// Iterate the prefixes of a `/`-separated name, root first, capped at [`MAX_DEPTH`]. Byte-identical
/// to `tier0::for_each_prefix`.
fn for_each_prefix<F: FnMut(&[u8])>(name: &[u8], mut f: F) {
    f(b"/");
    let mut depth = 0;
    for (i, &b) in name.iter().enumerate() {
        if i > 0 && b == b'/' {
            depth += 1;
            if depth >= MAX_DEPTH {
                return;
            }
            f(&name[..i]);
        }
    }
    if !name.is_empty() && depth < MAX_DEPTH {
        f(name);
    }
}

/// Truncate a registered prefix to the deepest form a sender would have inserted — the clamp that
/// keeps the depth cap from producing false negatives. Byte-identical to `tier0::clamp_prefix`.
fn clamp_prefix(prefix: &[u8]) -> usize {
    let mut comps = 0;
    for i in 1..prefix.len() {
        if prefix[i] == b'/' {
            comps += 1;
            if comps >= MAX_DEPTH - 1 {
                return i;
            }
        }
    }
    prefix.len()
}

fn name_hash(key: &[u8; 16], name: &[u8]) -> u64 {
    siphash24(key, name)
}

/// A prefix-set as a Golomb-Coded Set. Fixed-buffer + bit length ⇒ no heap, no_std.
#[derive(Clone, Copy)]
pub struct GcsFilter {
    n: u8,
    bit_len: u16,
    bytes: [u8; GCS_MAX_BYTES],
}

struct BitW<'a> {
    buf: &'a mut [u8],
    pos: usize,
}
impl BitW<'_> {
    fn bit(&mut self, b: u32) {
        if b & 1 != 0 && self.pos / 8 < self.buf.len() {
            self.buf[self.pos / 8] |= 1 << (7 - self.pos % 8);
        }
        self.pos += 1;
    }
    fn bits(&mut self, v: u64, n: u32) {
        for i in (0..n).rev() {
            self.bit(((v >> i) & 1) as u32);
        }
    }
    fn rice(&mut self, gap: u64, p: u32) {
        for _ in 0..(gap >> p) {
            self.bit(1);
        }
        self.bit(0);
        self.bits(gap & ((1 << p) - 1), p);
    }
}

struct BitR<'a> {
    buf: &'a [u8],
    pos: usize,
    len: usize,
}
impl BitR<'_> {
    fn bit(&mut self) -> Option<u32> {
        if self.pos >= self.len {
            return None;
        }
        let b = (self.buf[self.pos / 8] >> (7 - self.pos % 8)) & 1;
        self.pos += 1;
        Some(b as u32)
    }
    fn bits(&mut self, n: u32) -> Option<u64> {
        let mut v = 0u64;
        for _ in 0..n {
            v = (v << 1) | self.bit()? as u64;
        }
        Some(v)
    }
    fn rice(&mut self, p: u32) -> Option<u64> {
        let mut q = 0u64;
        while self.bit()? == 1 {
            q += 1;
        }
        Some((q << p) | self.bits(p)?)
    }
}

impl GcsFilter {
    /// Encode every prefix of `name` (root-first, capped at [`MAX_DEPTH`]) as a GCS. `name` is the
    /// `/`-joined normalized form (the caller normalizes, exactly as the Blur path does).
    pub fn from_name(key: &[u8; 16], name: &[u8]) -> Self {
        let mut vals = [0u64; MAX_PREFIXES];
        let mut n = 0usize;
        for_each_prefix(name, |pfx| {
            if n < MAX_PREFIXES {
                vals[n] = name_hash(key, pfx);
                n += 1;
            }
        });
        let m = (n as u64) << GCS_P;
        let vs = &mut vals[..n];
        for v in vs.iter_mut() {
            *v = if m > 0 { *v % m } else { 0 };
        }
        vs.sort_unstable();

        let mut bytes = [0u8; GCS_MAX_BYTES];
        let mut w = BitW { buf: &mut bytes, pos: 0 };
        let mut prev = 0u64;
        let mut first = true;
        for &v in vs.iter() {
            if !first && v == prev {
                continue;
            }
            w.rice(v - prev, GCS_P);
            prev = v;
            first = false;
        }
        Self { n: n as u8, bit_len: w.pos as u16, bytes }
    }

    /// Could a name carrying this filter be under `prefix`? `false` is **exact** (zero false negatives).
    pub fn may_match(&self, key: &[u8; 16], prefix: &[u8]) -> bool {
        let prefix = &prefix[..clamp_prefix(prefix)];
        let m = (self.n as u64) << GCS_P;
        if m == 0 {
            return false;
        }
        let qv = name_hash(key, prefix) % m;
        let mut r = BitR { buf: &self.bytes, pos: 0, len: self.bit_len as usize };
        let mut acc = 0u64;
        while let Some(gap) = r.rice(GCS_P) {
            acc += gap;
            if acc == qv {
                return true;
            }
            if acc > qv {
                return false;
            }
        }
        false
    }

    /// Serialize to `out` (`[n][gap bytes]`); returns the byte count.
    pub fn to_wire(&self, out: &mut [u8]) -> usize {
        let nbytes = (self.bit_len as usize).div_ceil(8);
        out[0] = self.n;
        out[1..1 + nbytes].copy_from_slice(&self.bytes[..nbytes]);
        1 + nbytes
    }

    /// The wire size (`[n] + gap bytes`) this filter occupies.
    pub fn wire_len(&self) -> usize {
        1 + (self.bit_len as usize).div_ceil(8)
    }

    /// Reconstruct from on-wire bytes. Trailing pad bits decode as zero-gaps and cannot create a match.
    pub fn from_wire(wire: &[u8]) -> Self {
        let n = wire.first().copied().unwrap_or(0);
        let body = wire.get(1..).unwrap_or(&[]);
        let nb = body.len().min(GCS_MAX_BYTES);
        let mut bytes = [0u8; GCS_MAX_BYTES];
        bytes[..nb].copy_from_slice(&body[..nb]);
        Self { n, bit_len: (nb * 8) as u16, bytes }
    }
}
