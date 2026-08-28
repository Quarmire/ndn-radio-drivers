//! **Tier-0: the in-frame prefix-set Bloom filter** (#91) — zero-parse name matching.
//!
//! The design lives in `ndn-face-monitor-wifi/docs/named-filter-mac-redesign.md` §3. In short: an
//! in-frame *hash* of the name cannot express prefix matching, and a prefix is the normal FIB entry
//! in NDN. A hash destroys hierarchy; NDN names **are** hierarchy. The fix is to carry the name's
//! **prefix set** rather than the name:
//!
//! ```text
//!   sender:    /A/b/c → { /, /A, /A/b, /A/b/c } → K bits set per prefix in an M-bit filter
//!   receiver:  for each registered prefix P (mask precomputed once):
//!                  (frame & mask[P]) == mask[P]  ⇒ maybe under P → accept, parse
//!              else                              ⇒ DEFINITELY not under P → drop, never parse
//! ```
//!
//! The negative answer is **exact**: if the name really were under `P`, the sender would have set
//! precisely those bits. False positives cost a parse; false negatives cannot occur. That asymmetry
//! is what makes an aggressive MAC-layer filter safe to be wrong.
//!
//! ## Sizing, and why the source paper's k does not transfer
//!
//! NDN-NIC uses k=2, optimal for *its* regime (~10⁵ keys in 65536 bits — a *low* bits-per-key
//! loading, where few hashes are optimal). Ours is the opposite loading: **n ≈ one name's prefix
//! chain (4–8) in [`M_BITS`] = 126 bits**, ~16–31 bits-per-key. There the textbook optimum
//! `(M/n)·ln2` predicts ~11 (deepest names), but that formula assumes a query's k positions are
//! independent; in only 126 bits they are not (double-hashed positions collide), so the marginal FP
//! gain saturates far below the prediction. Measured optimum is [`K`] = 4 — see the table below.
//!
//! ## Why 126 bits
//!
//! The filter rides in the frame's address octets: `addr1 ‖ addr2 ‖ addr3[0..4]` = 128 bits, of which
//! the **I/G and U/L bits of the first octet must keep their locally-administered/group meaning**, or
//! we begin emitting frames that look like real devices' unicast traffic — a doctrine violation, and
//! one that would make our traffic indistinguishable from an ordinary station's to anything listening.
//! 128 − 2 = 126 usable. The last two octets of addr3 carry the 8-bit ephemeral id and the flags byte
//! (the 128:8 partition).
//!
//! ## Hashing
//!
//! One keyed 64-bit name hash per prefix, expanded to [`K`] bit positions by double hashing
//! (Kirsch–Mitzenmacher): `h_i = (h1 + i·h2) mod M`. That keeps the project to **one name-hash
//! keyspace** shared by the filter, the FIB, and the data plane — which is open task #44 — instead
//! of introducing a second, incompatible hash family. Keyed so a private group's filter is
//! unlinkable by an observer who does not hold the key.

/// Usable filter bits. 128 (addr1 ‖ addr2 ‖ addr3[0..4]) minus the two reserved bits of octet 0.
pub const M_BITS: u32 = 126;

/// **Hashes per prefix — k = 4.**
///
/// Chosen on the largest-sample measurement available, not on a claimed optimum. The history matters
/// because this constant has now been wrong in both directions:
///
/// 1. Originally k=4, justified by an on-device sweep that "disproved" the closed-form prediction of
///    ~6. That sweep's false-positive queries came from `make_name(d, 0x10000 + t)`, and the helper
///    formats the salt as four hex digits — the 0x10000 truncated away, so the "disjoint" queries
///    shared leading components with the registered name and genuine ancestors were counted as false
///    positives.
/// 2. With the generator fixed, the same device sweep inverted and showed k=6 far ahead. That was
///    12 names and 54 events — one harness, small sample, exactly the weakness that produced (1).
/// 3. An independent host replication at **200 names / 400 000 trials** (`ksweep_host_replication`),
///    with +/-1σ error bars, disagrees with the device beyond both their error bars:
///
/// | k | bits set | FP @ depth 8, host 200n/400k (±1σ) | FP @ depth 8, device 12n/20k |
/// |---|---|---|---|
/// | 3 | 25/126 | 1.025% ± 0.016 | 0.67% |
/// | **4** | **33/126** | **0.559% ± 0.012** | **0.45%** |
/// | 5 | 40/126 | 0.855% ± 0.015 | 0.28% |
/// | 6 | 47/126 | 0.881% ± 0.015 | 0.22% |
/// | 8 | 60/126 | 0.907% ± 0.015 | 0.30% |
///
/// **The lesson survived the repack.** The 200-name/400k host sweep says k=4 wins; the 12-name/20k
/// on-device sweep (`m7_filter_test`, re-run at m=126 on the XIAO 2026-08-17) says FP keeps falling
/// through k=6 — the two harnesses STILL disagree on the k≥5 ordering, exactly as at m=94, because the
/// *name distribution* dominates and 12 names is too small a sample. k=4 agrees between them (host
/// 0.56% ≈ device 0.45%); k≥5 is where the small harness wanders. So k=4 is anchored on the
/// large-sample host measurement, not the device. **Zero false negatives on every device line, at
/// every depth and every k** — the safety invariant confirmed on real silicon. Golden vectors
/// cross-verify the host/device/ath9k code is byte-identical, so this is a sampling difference, not a
/// code difference.
///
/// So k=4 on the tiebreakers too: fewest bits set (33/126, most saturation headroom), fewest hashes
/// per frame, and it is what the on-air shadow-mode result was measured at — #106 at m=94 gave 87.1%
/// reject / 0.46% FP, and the m=126 re-run (2026-08-17) gave 87.4% reject / ~0.12% FP over 5000 frames,
/// zero false negatives, the wider filter cutting on-air FP ~4×.
///
/// **Do not "improve" this from a single sweep.** That is what went wrong twice.
pub const K: u32 = 4;

/// Deepest prefix inserted. Beyond this the filter saturates and degrades *for every user of the
/// frame*, so the tail is bounded here and deeper matching is left to the software tier.
pub const MAX_DEPTH: usize = 8;

/// **Admission fill cap** — the maximum number of set bits a *received* filter may carry and still
/// be tested against any local mask.
///
/// Without it, [`PrefixFilter::may_match`] is a pure AND: a frame with all 126 bits set matches every
/// registered mask at every node, for free, computed once. That is a one-frame universal wake — and
/// once the scheduler keys on this field it becomes worse than a wake, because the same frame
/// matches every slot owner's mask: every slot reads busy, presence is forged for every owner
/// including departed ones, and claims are suppressed network-wide for a full presence window per
/// frame.
///
/// **Sizing.** A legitimate filter at the depth cap sets ~33 bits (measured, `MAX_DEPTH` = 8,
/// `K` = 4 — see the depth/popcount table in the tests). 64 leaves headroom for future class tokens
/// while bounding a just-under-cap adversary to roughly `(64/126)^4` ≈ 6.7% per targeted prefix rather
/// than 100%.
///
/// **Scope, honestly.** This removes the *amplified* attack — one frame forging presence for every
/// group at once. It does not stop an adversary forging presence for a single group it knows the
/// name of; that is inherent to unauthenticated MAC-level evidence and is not a property any
/// arrangement of these 126 bits can provide.
///
/// Coupled to `MAX_DEPTH`, `K` and any future class tokens, so it is a **shared wire parameter**:
/// every implementation must use the same value or they disagree about which frames are admissible.
pub const FILL_CAP: u32 = 64;

/// The two bits of octet 0 that must not be used by the filter (I/G and U/L).
const RESERVED_MASK0: u8 = 0b0000_0011;

/// A 128-bit in-frame filter: 126 usable bits plus the two reserved address bits.
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct PrefixFilter(pub [u8; 16]);

impl Default for PrefixFilter {
    fn default() -> Self {
        Self::new()
    }
}

/// **SipHash-2-4** — vendored verbatim from `ndn-rs/crates/core/ndn-frame-io/src/frame.rs`.
///
/// Copied rather than depended on because this module must stay `no_std` and dependency-free so it
/// compiles for the FLPR RISC-V coprocessor unchanged. It is pure integer code, so the copy is
/// exact; [`tests::siphash24_reference_vector`] pins it to the published Aumasson & Bernstein vector
/// so the two copies cannot drift silently.
pub fn siphash24(key: &[u8; 16], data: &[u8]) -> u64 {
    let k0 = u64::from_le_bytes(match key[0..8].try_into() {
        Ok(v) => v,
        Err(_) => [0; 8],
    });
    let k1 = u64::from_le_bytes(match key[8..16].try_into() {
        Ok(v) => v,
        Err(_) => [0; 8],
    });
    let mut v0 = 0x736f_6d65_7073_6575 ^ k0;
    let mut v1 = 0x646f_7261_6e64_6f6d ^ k1;
    let mut v2 = 0x6c79_6765_6e65_7261 ^ k0;
    let mut v3 = 0x7465_6462_7974_6573 ^ k1;
    macro_rules! round {
        () => {{
            v0 = v0.wrapping_add(v1);
            v1 = v1.rotate_left(13);
            v1 ^= v0;
            v0 = v0.rotate_left(32);
            v2 = v2.wrapping_add(v3);
            v3 = v3.rotate_left(16);
            v3 ^= v2;
            v0 = v0.wrapping_add(v3);
            v3 = v3.rotate_left(21);
            v3 ^= v0;
            v2 = v2.wrapping_add(v1);
            v1 = v1.rotate_left(17);
            v1 ^= v2;
            v2 = v2.rotate_left(32);
        }};
    }
    let mut chunks = data.chunks_exact(8);
    for c in &mut chunks {
        let m = u64::from_le_bytes(match c.try_into() {
            Ok(v) => v,
            Err(_) => [0; 8],
        });
        v3 ^= m;
        round!();
        round!();
        v0 ^= m;
    }
    let mut last = (data.len() as u64 & 0xff) << 56;
    for (i, &b) in chunks.remainder().iter().enumerate() {
        last |= (b as u64) << (8 * i);
    }
    v3 ^= last;
    round!();
    round!();
    v0 ^= last;
    v2 ^= 0xff;
    round!();
    round!();
    round!();
    round!();
    v0 ^ v1 ^ v2 ^ v3
}

/// **The one agreed name-hash for Tier-0: SipHash-2-4 under the full 16-byte group key.**
///
/// This was keyed FNV-1a-64. FNV is not a PRF and XOR-ing the key into its init state is invertible
/// from observed output, so an outsider could recover a private group's key and then compute — or
/// deliberately collide with — its pre-parse filter. That is exactly the guarantee the addressing
/// doctrine (§8) assigns to the group key.
///
/// **Both copies of this module must agree or they cannot share a group**, so this changed in
/// lockstep with `ndn-ext/crates/faces/ndn-face-monitor-wifi/src/tier0.rs`. A filter built under one
/// hash will not match masks built under the other — there is no partial interop, no graceful
/// degradation, and no error message: names simply stop matching.
pub fn name_hash(key: &[u8; 16], name: &[u8]) -> u64 {
    siphash24(key, name)
}

/// Domain separator for the second hash, so `h1` and `h2` are independent PRF evaluations under
/// different keys rather than two halves of one output.
const KEY2_DOMAIN: [u8; 16] = *b"ndn/tier0-h2\0\0\0\0";

/// The [`K`] bit positions one prefix occupies.
///
/// `h1` and `h2` are two **independent** keyed hashes, not the two halves of one. Splitting a single
/// FNV-1a output was the first implementation and it measured 1.3–3.4× worse than the independent-
/// hash model at depths 4–8 (and far worse in relative terms at low occupancy) — FNV's high bits are
/// its weak half, so using them as the double-hashing stride correlates the K positions. A second
/// pass over a short prefix costs a few cycles and buys back the model.
pub fn positions(key: &[u8; 16], prefix: &[u8]) -> [u8; K as usize] {
    let mut key2 = *key;
    let mut i = 0;
    while i < 16 {
        key2[i] ^= KEY2_DOMAIN[i];
        i += 1;
    }
    let h1 = name_hash(key, prefix) as u32;
    // `| 1` keeps the stride odd, so the K positions cannot collapse onto one bit.
    let h2 = (name_hash(&key2, prefix) as u32) | 1;
    let mut out = [0u8; K as usize];
    for (i, o) in out.iter_mut().enumerate() {
        *o = (h1.wrapping_add((i as u32).wrapping_mul(h2)) % M_BITS) as u8;
    }
    out
}

/// Iterate the prefixes of a `/`-separated name, root first, capped at [`MAX_DEPTH`].
///
/// `/A/b/c` yields `/`, `/A`, `/A/b`, `/A/b/c`.
pub fn for_each_prefix<F: FnMut(&[u8])>(name: &[u8], mut f: F) {
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

/// Truncate a *registered* prefix to the deepest form a sender would actually have inserted.
///
/// ★ Load-bearing. Without it the depth cap produces **true false negatives** — the one failure the
/// design forbids.
///
/// [`for_each_prefix`] stops at the cap, so a sender transmitting `/a/b/c/d/e/f/g/h/i` inserts at
/// deepest `/a/b/c/d/e/f/g` — **seven** components, not eight. A receiver registered on
/// `/a/b/c/d/e/f/g/h` would otherwise build a mask over bits the sender never set and drop a frame
/// that genuinely is under its prefix.
///
/// "Zero false negatives at every depth" holds only for registrations within the cap; the on-device
/// measurement could not see this, because it only queried prefixes that had been inserted, which
/// makes the property tautological. Clamping restores it: a too-deep registration degrades to its
/// 7-component ancestor, costing extra false positives and no false negatives, and Tier 1/2 does
/// the exact match — which only works if the frame survives Tier 0 to reach it.
///
/// Found by cross-checking the C port for the AR9271 firmware against this implementation.
pub fn clamp_prefix(prefix: &[u8]) -> usize {
    let mut comps = 0;
    for i in 1..prefix.len() {
        if prefix[i] == b'/' {
            comps += 1;
            // `comps` components precede this slash; the cap admits MAX_DEPTH - 1 of them.
            if comps >= MAX_DEPTH - 1 {
                return i;
            }
        }
    }
    prefix.len()
}

impl PrefixFilter {
    /// An empty filter (all usable bits clear).
    pub const fn new() -> Self {
        Self([0; 16])
    }

    /// Set one bit, skipping the two reserved positions by construction.
    fn set_bit(&mut self, pos: u8) {
        // Bit p of the usable space maps to physical bit p+2, so 0 and 1 of octet 0 stay free.
        let p = pos as usize + 2;
        self.0[p / 8] |= 1 << (p % 8);
    }

    fn get_bit(&self, pos: u8) -> bool {
        let p = pos as usize + 2;
        self.0[p / 8] & (1 << (p % 8)) != 0
    }

    /// Insert every prefix of `name`.
    pub fn insert_name(&mut self, key: &[u8; 16], name: &[u8]) {
        let mut tmp = *self;
        for_each_prefix(name, |pfx| {
            for &p in positions(key, pfx).iter() {
                tmp.set_bit(p);
            }
        });
        *self = tmp;
    }

    /// The mask a receiver precomputes once per registered prefix.
    ///
    /// The prefix is clamped by [`clamp_prefix`] first — without that, a registration deeper than
    /// the cap produces a **true false negative**.
    pub fn mask_for(key: &[u8; 16], prefix: &[u8]) -> Self {
        let prefix = &prefix[..clamp_prefix(prefix)];
        let mut m = Self::new();
        for &p in positions(key, prefix).iter() {
            m.set_bit(p);
        }
        m
    }

    /// Could this frame's name be under the prefix `mask` was built from?
    ///
    /// `false` is **exact** — the name is definitely not under it. `true` means *probably*, and the
    /// software tier decides.
    pub fn may_match(&self, mask: &Self) -> bool {
        // Fill cap first — see FILL_CAP. Must match the host copy exactly or the two disagree about
        // which frames are admissible, which is a silent interop split.
        if self.popcount() > FILL_CAP {
            return false;
        }
        for i in 0..16 {
            let want = mask.0[i] & !if i == 0 { RESERVED_MASK0 } else { 0 };
            if self.0[i] & want != want {
                return false;
            }
        }
        true
    }

    /// Count of usable bits set — the saturation the false-positive rate follows.
    pub fn popcount(&self) -> u32 {
        let mut n = 0;
        for p in 0..M_BITS as u8 {
            if self.get_bit(p) {
                n += 1;
            }
        }
        n
    }

    /// The 16 wire bytes (addr1 ‖ addr2 ‖ addr3[0..4]), reserved bits forced to locally-administered group.
    ///
    /// Applied at the boundary rather than trusted from the caller: a filter whose bit pattern
    /// happens to clear these would put a globally-unique unicast address on the air.
    pub fn to_wire(&self) -> [u8; 16] {
        let mut w = self.0;
        w[0] = (w[0] & !RESERVED_MASK0) | 0b0000_0011; // I/G = group, U/L = local
        w
    }
}

// ── WIDE PROFILE (#39) — layered Blur + exact-match fingerprint on the pushed 802.11 header ──────────
//
// Mirrors `ndn-radio/crates/faces/ndn-radio/src/mac/tier0.rs`. A wide sender emits a 4-address
// QoS-Data+HTC frame: the base 126-bit Blur stays byte-identical in addr1‖addr2‖addr3[0:4] (a base
// receiver reads it unchanged), an additive 48-bit second projection rides addr4, and the 24-bit name
// fingerprint rides HT Control. All three implementations (this firmware, the ath9k-htc C port, the
// host) MUST reproduce the wide golden vector byte-for-byte, or a wide sender and receiver disagree.

/// Exact-match fingerprint width (bits), carried in HT Control. A *separate* field — never carved
/// from the Blur, so the Blur never shrinks.
pub const FP_BITS: u32 = 24;

/// Extra Blur bytes on the Wi-Fi wide profile — `addr4` only (48 bits).
pub const WIFI_WIDE_EXTRA_BYTES: usize = 6;

/// Profile marker written to `HT Control[3]`.
pub const WIDE_PROFILE_MARKER: u8 = 0x01;

/// Domain separator for the extra projection — a second, independent keyed projection so the extra
/// region is not correlated with the base.
const EXTRA_DOMAIN: [u8; 16] = *b"ndn/tier0-xtra!\0";

/// [`positions`] parameterized by the Blur width `m_blur` — one algorithm for every profile (126 on
/// the base frame, 48 for the extra region). `positions` is the `M_BITS` case, bit-identical.
pub fn positions_m(key: &[u8; 16], prefix: &[u8], m_blur: u32) -> [u16; K as usize] {
    let mut key2 = *key;
    let mut i = 0;
    while i < 16 {
        key2[i] ^= KEY2_DOMAIN[i];
        i += 1;
    }
    let h1 = name_hash(key, prefix) as u32;
    let h2 = (name_hash(&key2, prefix) as u32) | 1;
    let mut out = [0u16; K as usize];
    for (i, o) in out.iter_mut().enumerate() {
        *o = (h1.wrapping_add((i as u32).wrapping_mul(h2)) % m_blur) as u16;
    }
    out
}

/// The `FP_BITS`-wide exact-match fingerprint of a full name — the low bits of the keyed name hash.
pub fn name_fingerprint(key: &[u8; 16], name: &[u8]) -> u32 {
    (name_hash(key, name) as u32) & ((1u32 << FP_BITS) - 1)
}

/// A wide-profile frame's pushed-header fields, exactly as they land on the wire.
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct WideFields {
    pub addr1: [u8; 6],
    pub addr2: [u8; 6],
    pub addr3: [u8; 6], // base[12:16] ‖ id ‖ flags
    pub addr4: [u8; 6], // extra Blur (48 bits)
    pub htc: [u8; 4],   // fingerprint (24 bits, LE) ‖ profile marker
}

/// Build the wide-profile header fields for one name — the sender path. The base region is exactly
/// [`PrefixFilter::insert_name`]; the extra region is an independent projection under the extra key.
pub fn wide_fields(key: &[u8; 16], name: &[u8], id: u8, flags: u8) -> WideFields {
    // Base region — identical to a base-only frame.
    let mut base = PrefixFilter::new();
    base.insert_name(key, name);
    let bw = base.to_wire();

    // Extra region — second projection into WIFI_WIDE_EXTRA_BYTES*8 bits, no reserved bits.
    let mut xkey = *key;
    let mut i = 0;
    while i < 16 {
        xkey[i] ^= EXTRA_DOMAIN[i];
        i += 1;
    }
    let m_extra = (WIFI_WIDE_EXTRA_BYTES * 8) as u32;
    let mut extra = [0u8; WIFI_WIDE_EXTRA_BYTES];
    for_each_prefix(name, |pfx| {
        for &p in positions_m(&xkey, pfx, m_extra).iter() {
            extra[p as usize / 8] |= 1 << (p % 8);
        }
    });

    let fp = name_fingerprint(key, name);
    let mut f = WideFields {
        addr1: [0; 6],
        addr2: [0; 6],
        addr3: [0; 6],
        addr4: [0; 6],
        htc: [0; 4],
    };
    f.addr1.copy_from_slice(&bw[0..6]);
    f.addr2.copy_from_slice(&bw[6..12]);
    f.addr3[0..4].copy_from_slice(&bw[12..16]);
    f.addr3[4] = id;
    f.addr3[5] = flags;
    f.addr4.copy_from_slice(&extra);
    f.htc[0] = fp as u8;
    f.htc[1] = (fp >> 8) as u8;
    f.htc[2] = (fp >> 16) as u8;
    f.htc[3] = WIDE_PROFILE_MARKER;
    f
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `/Users/…/ndn-radio-drivers/golden/tier0/vectors.txt`, transcribed.
    ///
    /// That file has claimed **"Reproduced by the firmware (Rust)"** since the wide profile landed,
    /// and until now this crate had no test at all — the claim rested on a hand-run comparison. It is
    /// exactly the claim that must not rest on one: a filter built under a different hash, k, m or
    /// clamp does not degrade gracefully or report an error. Names simply stop matching, on air,
    /// between two nodes that each believe they are correct.
    ///
    /// Regenerate the file with
    /// `NDN_TIER0_REGEN=1 cargo test -p ndn-face-monitor-wifi tier0_golden`, then update here.
    const KEY1: [u8; 16] = *b"ndr/tier0-vec-01";
    const KEY2: [u8; 16] = *b"ndr/tier0-vec-02";

    fn unhex(s: &str) -> Vec<u8> {
        (0..s.len() / 2).map(|i| u8::from_str_radix(&s[2 * i..2 * i + 2], 16).unwrap()).collect()
    }

    /// `params version=2 k=4 m=126 max_depth=8 fill_cap=64 hash=siphash24 key_len=16
    /// reserved_mask0=0x03` — the header line of the golden file is itself part of the contract.
    /// A row can only be reproduced by an implementation that agrees on all of it.
    #[test]
    fn golden_params_line() {
        assert_eq!(K, 4);
        assert_eq!(M_BITS, 126);
        assert_eq!(MAX_DEPTH, 8);
        assert_eq!(FILL_CAP, 64);
        assert_eq!(RESERVED_MASK0, 0x03);
        // wide-params fp_bits=24 extra_bytes=6 marker=0x01
        assert_eq!(FP_BITS, 24);
        assert_eq!(WIFI_WIDE_EXTRA_BYTES, 6);
        assert_eq!(WIDE_PROFILE_MARKER, 0x01);
    }

    /// The four base rows: `row <label> <key> <name> <16 wire bytes> <popcount>`.
    #[test]
    fn golden_base_rows() {
        for (label, key, name, wire, popcount) in [
            ("depth2", &KEY1, "/ndn/alarm", "83080804010000820040040090000010", 12),
            ("depth8", &KEY1, "/c0/c1/c2/c3/c4/c5/c6/c7", "332819522211a01084010e60c4006218", 33),
            // Past the depth cap: the filter stops inserting, so it is a *different* (sparser)
            // filter than the depth-8 row — 30 bits, not 33. That divergence is the cap working.
            (
                "depth12-over-cap",
                &KEY1,
                "/c0/c1/c2/c3/c4/c5/c6/c7/c8/c9/c10/c11",
                "332019102211a01084010e60c4006218",
                30,
            ),
            // Same name, different group key: an observer without the key cannot recompute the
            // filter, which is the unlinkability property the addressing doctrine assigns to it.
            ("wrongkey", &KEY2, "/ndn/alarm", "83000100010082100804084080100000", 12),
        ] {
            let mut f = PrefixFilter::new();
            f.insert_name(key, name.as_bytes());
            assert_eq!(f.to_wire().to_vec(), unhex(wire), "golden row {label}: wire bytes");
            assert_eq!(f.popcount(), popcount, "golden row {label}: popcount");
        }
    }

    /// The wide row (#39): the base Blur must be **byte-identical** to a base-only frame, so a base
    /// receiver reads a wide sender's frame unchanged. addr4 carries the additive 48-bit second
    /// projection; HT Control carries the 24-bit fingerprint little-endian, then the marker.
    #[test]
    fn golden_wide_row() {
        let name = b"/ndn/test/v1";
        let w = wide_fields(&KEY1, name, 0x37, 0x00);
        let mut all = Vec::new();
        all.extend_from_slice(&w.addr1);
        all.extend_from_slice(&w.addr2);
        all.extend_from_slice(&w.addr3);
        all.extend_from_slice(&w.addr4);
        assert_eq!(all, unhex("87000800c10308820040040080000011370041a14230d880"));
        assert_eq!(w.htc.to_vec(), unhex("1486e901"));
        assert_eq!(name_fingerprint(&KEY1, name), 0x00e9_8614);

        // The base half of a wide frame IS a base frame.
        let mut base = PrefixFilter::new();
        base.insert_name(&KEY1, name);
        let bw = base.to_wire();
        assert_eq!(&all[0..12], &bw[0..12]);
        assert_eq!(&all[12..16], &bw[12..16]);
        // …and the last two octets of addr3 are the 8-bit id and the flags byte (the 128:8 split).
        assert_eq!(all[16], 0x37);
        assert_eq!(all[17], 0x00);
    }

    /// Aumasson & Bernstein's published SipHash-2-4 vectors, key `00 01 … 0f`, input `00 01 … n-1`.
    ///
    /// Pinned because [`siphash24`] is **vendored** from `ndn-frame-io` rather than depended on, so
    /// that this module compiles for the FLPR RISC-V coprocessor with no dependencies. A vendored
    /// copy is only safe if something stops it drifting.
    #[test]
    fn siphash24_reference_vector() {
        let key: [u8; 16] = core::array::from_fn(|i| i as u8);
        let inp = |n: usize| -> Vec<u8> { (0..n).map(|i| i as u8).collect() };
        assert_eq!(siphash24(&key, &inp(0)), 0x726f_db47_dd0e_0e31);
        assert_eq!(siphash24(&key, &inp(1)), 0x74f8_39c5_93dc_67fd);
        assert_eq!(siphash24(&key, &inp(8)), 0x93f5_f579_9a93_2462);
        assert_eq!(siphash24(&key, &inp(15)), 0xa129_ca61_49be_45e5);
    }

    /// **The safety invariant**: a receiver registered on a genuine ancestor of the transmitted name
    /// must never drop the frame. False positives cost a parse; a false negative is a lost packet
    /// with no error anywhere, and no amount of software-tier work can recover it.
    #[test]
    fn zero_false_negatives_at_every_depth() {
        let key = KEY1;
        for depth in 1..=12usize {
            let mut name = String::new();
            for c in 0..depth {
                name.push_str(&format!("/c{c}"));
            }
            let mut f = PrefixFilter::new();
            f.insert_name(&key, name.as_bytes());

            // Every registrable prefix of the name, including ones deeper than the cap — those are
            // exactly the case `clamp_prefix` exists for.
            for cut in 0..=depth {
                let mut pfx = String::new();
                for c in 0..cut {
                    pfx.push_str(&format!("/c{c}"));
                }
                if pfx.is_empty() {
                    pfx.push('/');
                }
                let mask = PrefixFilter::mask_for(&key, pfx.as_bytes());
                assert!(
                    f.may_match(&mask),
                    "FALSE NEGATIVE: name {name} vs registered prefix {pfx}"
                );
            }
        }
    }

    /// ★ The specific false negative `clamp_prefix` was added for, isolated so a "simplification"
    /// that removes the clamp fails here instead of on air. Found by cross-checking the C port for
    /// the AR9271 firmware against this implementation.
    #[test]
    fn clamp_prefix_saves_an_over_deep_registration() {
        let key = KEY1;
        let name = b"/a/b/c/d/e/f/g/h/i";
        let mut f = PrefixFilter::new();
        f.insert_name(&key, name);

        // The sender's deepest inserted prefix is SEVEN components (`for_each_prefix` stops at the
        // cap), so a registration on eight must degrade to that ancestor rather than build a mask
        // over bits nobody set.
        assert_eq!(clamp_prefix(b"/a/b/c/d/e/f/g/h"), b"/a/b/c/d/e/f/g".len());
        assert!(f.may_match(&PrefixFilter::mask_for(&key, b"/a/b/c/d/e/f/g/h")));
        assert!(f.may_match(&PrefixFilter::mask_for(&key, b"/a/b/c/d/e/f/g")));
        // A prefix inside the cap that the name is genuinely NOT under stays droppable.
        assert!(!f.may_match(&PrefixFilter::mask_for(&key, b"/z/y/x")));
    }

    /// The depth → popcount curve the `FILL_CAP` sizing is argued from: a legitimate filter at the
    /// depth cap sets ~33 of 126 bits, so 64 leaves headroom while still bounding an adversary.
    /// Measured here rather than asserted in prose.
    #[test]
    fn depth_popcount_table() {
        let key = KEY1;
        let mut deepest = 0;
        for depth in 1..=MAX_DEPTH {
            let mut name = String::new();
            for c in 0..depth {
                name.push_str(&format!("/c{c}"));
            }
            let mut f = PrefixFilter::new();
            f.insert_name(&key, name.as_bytes());
            let pc = f.popcount();
            // K bits per prefix, depth+1 prefixes, minus collisions — so never above the product.
            assert!(pc <= K * (depth as u32 + 1), "depth {depth}: popcount {pc} exceeds K*(d+1)");
            assert!(pc > 0);
            deepest = pc;
        }
        // At the cap this is 33/126 — comfortably under FILL_CAP, which is the whole sizing claim.
        assert_eq!(deepest, 33);
        assert!(deepest < FILL_CAP);
    }

    /// The admission cap: a saturated filter matches every mask at every node for free, which is a
    /// one-frame universal wake and — once the scheduler keys on this field — a network-wide claim
    /// suppression. It must be rejected before the AND is even considered.
    #[test]
    fn over_filled_filters_are_inadmissible() {
        let all_ones = PrefixFilter([0xFF; 16]);
        assert_eq!(all_ones.popcount(), M_BITS);
        assert!(all_ones.popcount() > FILL_CAP);
        assert!(!all_ones.may_match(&PrefixFilter::mask_for(&KEY1, b"/ndn/alarm")));
        // …while a legitimate filter at the depth cap still passes.
        let mut f = PrefixFilter::new();
        f.insert_name(&KEY1, b"/c0/c1/c2/c3/c4/c5/c6/c7");
        assert!(f.popcount() <= FILL_CAP);
        assert!(f.may_match(&PrefixFilter::mask_for(&KEY1, b"/c0/c1")));
    }

    /// The two reserved bits of octet 0 must always leave the wire as locally-administered group,
    /// whatever the filter's own bit pattern is — otherwise we put a globally-unique unicast address
    /// on the air, which is a doctrine violation and makes our traffic look like a real station's.
    #[test]
    fn reserved_bits_are_forced_on_the_wire() {
        for name in ["/ndn/alarm", "/a", "/c0/c1/c2/c3/c4/c5/c6/c7"] {
            let mut f = PrefixFilter::new();
            f.insert_name(&KEY1, name.as_bytes());
            assert_eq!(f.to_wire()[0] & RESERVED_MASK0, RESERVED_MASK0, "{name}");
        }
    }
}
