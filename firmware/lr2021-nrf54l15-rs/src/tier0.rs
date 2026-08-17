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
/// | k | bits set | FP @ depth 8, host (±1σ) |
/// |---|---|---|
/// | 3 | 25/126 | 1.025% ± 0.016 |
/// | **4** | **33/126** | **0.559% ± 0.012** |
/// | 5 | 40/126 | 0.855% ± 0.015 |
/// | 6 | 47/126 | 0.881% ± 0.015 |
/// | 8 | 60/126 | 0.907% ± 0.015 |
///
/// **At m=126 the optimum is unambiguous: k=4 wins by more than the error bars.** (Re-measured after
/// the 94→126 repack; the earlier m=94 table had two independent harnesses disagreeing on the k=4..8
/// ordering — the *name distribution* dominated, and widening the filter separated the candidates.)
/// k=4 minimizes FP at 0.56%, ~half of k=3 and clearly under k=5/6/8, with **zero false negatives** at
/// every k. This module runs byte-identical code to the host (golden-vector cross-verified), so the
/// host sweep IS the firmware's; confirm on-device via `m7_filter_test`.
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
    let k0 = u64::from_le_bytes(match key[0..8].try_into() { Ok(v) => v, Err(_) => [0; 8] });
    let k1 = u64::from_le_bytes(match key[8..16].try_into() { Ok(v) => v, Err(_) => [0; 8] });
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
        let m = u64::from_le_bytes(match c.try_into() { Ok(v) => v, Err(_) => [0; 8] });
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
