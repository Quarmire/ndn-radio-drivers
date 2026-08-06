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
//! NDN-NIC uses k=2, which is optimal for *its* regime — ~10⁵ keys in 65536 bits. Ours is the
//! opposite: **n ≈ name depth (4–8) in [`M_BITS`] = 94 bits**, where optimal k is far higher.
//! Copying k=2 would cost roughly two orders of magnitude of false-positive rate. See [`K`].
//!
//! ## Why 94 and not 96
//!
//! The filter rides in the two 48-bit address fields of the frame. The **I/G and U/L bits of the
//! first octet must keep their locally-administered/group meaning**, or we begin emitting frames
//! that look like real devices' unicast traffic — a doctrine violation, and one that would make our
//! traffic indistinguishable from an ordinary station's to anything listening.
//!
//! ## Hashing
//!
//! One keyed 64-bit name hash per prefix, expanded to [`K`] bit positions by double hashing
//! (Kirsch–Mitzenmacher): `h_i = (h1 + i·h2) mod M`. That keeps the project to **one name-hash
//! keyspace** shared by the filter, the FIB, and the data plane — which is open task #44 — instead
//! of introducing a second, incompatible hash family. Keyed so a private group's filter is
//! unlinkable by an observer who does not hold the key.

/// Usable filter bits. 96 (two address fields) minus the two reserved bits of octet 0.
pub const M_BITS: u32 = 94;

/// Bit positions set per inserted prefix — **4, measured, not 6 or 7 as the formula predicts.**
///
/// The textbook optimum `(M/n)·ln2` gives ~7 here, and #91's design chose 6 on that basis. Measured
/// at the depth cap on hardware (`m7_filter_test`, 20 000 trials per point):
///
/// | k | bits set | FP at depth 8 |
/// |---|---|---|
/// | 3 | 25/94 | 1.99% |
/// | **4** | **29/94** | **0.94%** ← measured optimum |
/// | 5 | 35/94 | 0.98% |
/// | 6 | 42/94 | 1.09% |
/// | 8 | 53/94 | 1.50% |
///
/// The formula is wrong here for a specific reason: it assumes a query's k positions are
/// independent. With only 94 bits they are not — k=6 positions collide *with each other* roughly
/// 15% of the time, and a query whose 6 positions collapse to 3 distinct bits has the
/// false-positive rate of k=3, not k=6. That effect is invisible to the formula and grows with k,
/// which is why the true optimum sits below the predicted one. Small-m Bloom filters are their own
/// regime; do not size this one from the asymptotic formula.
///
/// Lower k is also cheaper: 4 hash positions per prefix instead of 6.
pub const K: u32 = 4;

/// Deepest prefix inserted. Beyond this the filter saturates and degrades *for every user of the
/// frame*, so the tail is bounded here and deeper matching is left to the software tier.
pub const MAX_DEPTH: usize = 8;

/// The two bits of octet 0 that must not be used by the filter (I/G and U/L).
const RESERVED_MASK0: u8 = 0b0000_0011;

/// A 96-bit in-frame filter: 94 usable bits plus the two reserved address bits.
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct PrefixFilter(pub [u8; 12]);

impl Default for PrefixFilter {
    fn default() -> Self {
        Self::new()
    }
}

/// FNV-1a 64, keyed — the same name-hash family the on-device data plane already uses, so the
/// filter shares one keyspace with the FIB and dedup rather than adding a second (#44).
pub fn name_hash(key: u64, name: &[u8]) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325 ^ key;
    for &b in name {
        h ^= b as u64;
        h = h.wrapping_mul(0x0000_0100_0000_01b3);
    }
    h
}

/// Second-hash key derivation constant (golden ratio), so `h2` comes from an **independent** FNV
/// pass rather than the high half of `h1`.
const KEY2_MIX: u64 = 0x9E37_79B9_7F4A_7C15;

/// The [`K`] bit positions one prefix occupies.
///
/// `h1` and `h2` are two **independent** keyed hashes, not the two halves of one. Splitting a single
/// FNV-1a output was the first implementation and it measured 1.3–3.4× worse than the independent-
/// hash model at depths 4–8 (and far worse in relative terms at low occupancy) — FNV's high bits are
/// its weak half, so using them as the double-hashing stride correlates the K positions. A second
/// pass over a short prefix costs a few cycles and buys back the model.
pub fn positions(key: u64, prefix: &[u8]) -> [u8; K as usize] {
    let h1 = name_hash(key, prefix) as u32;
    // `| 1` keeps the stride odd, so the K positions cannot collapse onto one bit.
    let h2 = (name_hash(key ^ KEY2_MIX, prefix) as u32) | 1;
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
        Self([0; 12])
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
    pub fn insert_name(&mut self, key: u64, name: &[u8]) {
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
    pub fn mask_for(key: u64, prefix: &[u8]) -> Self {
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
        for i in 0..12 {
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

    /// The 12 wire bytes, with the reserved bits forced to locally-administered group.
    ///
    /// Applied at the boundary rather than trusted from the caller: a filter whose bit pattern
    /// happens to clear these would put a globally-unique unicast address on the air.
    pub fn to_wire(&self) -> [u8; 12] {
        let mut w = self.0;
        w[0] = (w[0] & !RESERVED_MASK0) | 0b0000_0011; // I/G = group, U/L = local
        w
    }
}
