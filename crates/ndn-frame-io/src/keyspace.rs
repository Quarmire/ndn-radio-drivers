//! **The one name keyspace** (#44) — `prefix_hash`, the unkeyed digest a name's *schedule* is
//! derived from, and the `/`-string splitting every caller must agree on to compute it.
//!
//! # Why this lives here and not where it was written
//!
//! This function was `ndn_radio::mac::prefix_hash` (re-exported as `ndn_radio_cognition::prefix_hash`),
//! and it is the hash the named airtime lease already uses everywhere: `SlotSchedule::owner_slot`,
//! `HopSchedule`, the demand tracker, the sense bus, `NameContext`, the consistency digest, and the
//! two on-air slot examples (`slot_ab_onair.rs`, `c5_slot.rs`) all key on it.
//!
//! The bearer that has to *actuate* the lease is a driver — and the repo graph is a deliberate DAG,
//! `ndn-rs <- ndn-radio-drivers <- ndn-ext`, so a driver cannot depend on the control plane. The only
//! two ways to give the driver the same slot map were to copy the function (a second implementation,
//! free to drift, with every test still passing on both sides of the drift) or to move the one
//! implementation *below* both. This crate is below both, and it is already where the peer primitive
//! lives: [`siphash24`](crate::siphash24) sits here for exactly this reason. So the function moved
//! down and `ndn_radio::mac::prefix_hash` is now a re-export of this — one implementation, two paths.
//!
//! ⚠ It is pinned by golden vectors ([`tests`]) taken from the pre-move implementation by running it,
//! not by reading it. A hash whose value changes silently re-slots every name in the fleet while
//! every unit test still passes, so the move had to be checked against output, not against source.
//!
//! # Why this one is unkeyed while the name *filter* is keyed
//!
//! Both hash a name; they answer different questions and have different adversaries.
//!
//! * The in-frame prefix-set filter ([`siphash24`] under a [`GroupKey`](crate::GroupKey)) is a
//!   **pre-parse admission** test an outsider would like to forge or collide with. It must be a PRF
//!   under a secret, or a private group's receive filter can be flooded by anyone who watched a few
//!   frames.
//! * The slot map is a **public rendezvous**: every node holding a name must compute the same slot,
//!   and there is nothing to hide — knowing the name already tells you the slot under any keying that
//!   the holders themselves share. Keying it would buy no secrecy and would cost the property the
//!   schedule exists for, which is that a stranger holding only the name computes the identical grid.
//!
//! Keeping them distinct is deliberate: they are one *keyspace* (#44) in the sense that everything
//! keyed on a name's identity keys on `prefix_hash`, not in the sense that one hash serves both jobs.

/// The canonical prefix-hash — **FNV-1a over the name's components, with a `0x2f` separator after
/// each** — the opaque key that ties the slot/hop schedules, demand, the sense bus, `NameContext`
/// and the consistency digest to one another.
///
/// The separator is load-bearing: without it `["ab","c"]` and `["a","bc"]` hash identically, and two
/// unrelated names would silently share a slot.
///
/// Component values are taken verbatim, so a caller starting from a wire `Name` TLV should render it
/// with `ndn_radio::mac::name::ndn_name_to_slash` and split with [`slash_components`] — the two paths
/// then agree byte for byte.
pub fn prefix_hash(components: &[&[u8]]) -> u64 {
    let mut h: u64 = 0xcbf29ce484222325;
    for c in components {
        for &b in *c {
            h ^= b as u64;
            h = h.wrapping_mul(0x100000001b3);
        }
        // component separator so ["ab","c"] ≠ ["a","bc"]
        h ^= 0x2f;
        h = h.wrapping_mul(0x100000001b3);
    }
    h
}

/// Split a `/`-joined name (`/ndn/lease/a`, the form
/// `ndn_radio::mac::name::ndn_name_to_slash` produces) into the components [`prefix_hash`] eats.
///
/// Empty segments are dropped, so a leading `/`, a trailing `/` and a doubled `//` all normalize to
/// the same component list — and the root name `/` yields the empty list, whose hash is the bare FNV
/// offset basis. That is a real, addressable name, not an error case.
pub fn slash_components(name: &[u8]) -> Vec<&[u8]> {
    name.split(|&b| b == b'/')
        .filter(|c| !c.is_empty())
        .collect()
}

/// [`prefix_hash`] of a `/`-joined name — the one-call form for a caller holding a string.
pub fn prefix_hash_slash(name: &[u8]) -> u64 {
    prefix_hash(&slash_components(name))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// ★ **Golden vectors, produced by RUNNING the pre-move implementation**
    /// (`ndn_radio_cognition::prefix_hash`, 2026-09-01) rather than by reading it.
    ///
    /// This is the test the move exists to make possible and the one that makes it safe. Every other
    /// property here (determinism, separation, spread) is satisfied just as well by a *different*
    /// hash — so they would all pass while every node in the fleet re-slotted itself. Only a value
    /// taken from the other implementation catches that.
    const GOLDEN: &[(&str, u64)] = &[
        ("/ndn/lease/a", 0xe6d632ff406ee907),
        ("/ndn/lease/b", 0xe6d2b2ff406bd9b0),
        ("/ndn/lease/node-a", 0x3e2c92bde12da526),
        ("/ndn/lease/node-b", 0x3e22b2bde1258701),
        ("/", 0xcbf29ce484222325),
        ("/ndn", 0xefafe0baa69c5dd4),
        ("/ndn/test/halow", 0xcc606517bbef2b3d),
        ("/ndn/mds/o5p-1/telemetry", 0x83ca8f6acce04704),
    ];

    #[test]
    fn matches_the_control_planes_prefix_hash_bit_for_bit() {
        for (name, want) in GOLDEN {
            assert_eq!(
                prefix_hash_slash(name.as_bytes()),
                *want,
                "{name} re-slotted: this hash no longer agrees with ndn_radio::mac::prefix_hash, so \
                 a driver and the control plane would compute different slots for the same name"
            );
        }
    }

    /// The golden vector for `/` is the bare FNV offset basis, which is what an EMPTY component list
    /// hashes to — so the root name is genuinely `prefix_hash(&[])` and not a special case anyone
    /// has to remember.
    #[test]
    fn the_root_name_is_the_empty_component_list() {
        assert_eq!(prefix_hash_slash(b"/"), prefix_hash(&[]));
        assert_eq!(slash_components(b"/").len(), 0);
    }

    #[test]
    fn leading_trailing_and_doubled_separators_normalize_together() {
        let h = prefix_hash_slash(b"/ndn/lease/a");
        for spelling in [&b"ndn/lease/a"[..], b"/ndn/lease/a/", b"//ndn//lease//a"] {
            assert_eq!(
                prefix_hash_slash(spelling),
                h,
                "{spelling:?} must normalize"
            );
        }
    }

    /// The separator's whole job. Without it these two collide, and two unrelated names share a slot
    /// for reasons no operator could ever diagnose from the outside.
    #[test]
    fn the_component_separator_keeps_regroupings_apart() {
        assert_ne!(
            prefix_hash(&[b"ab", b"c"]),
            prefix_hash(&[b"a", b"bc"]),
            "component regrouping must not collide"
        );
    }

    /// Spread, on the population a small superframe actually sees. Not a claim about the hash's
    /// cryptographic quality — just that `% slots` does not pile every name onto one slot, which is
    /// the failure that would make the schedule useless while looking like it worked.
    #[test]
    fn names_spread_over_a_small_slot_count() {
        const SLOTS: usize = 8;
        let mut counts = [0usize; SLOTS];
        for i in 0..2000u32 {
            let n = format!("/ndn/node{i}/data");
            counts[(prefix_hash_slash(n.as_bytes()) % SLOTS as u64) as usize] += 1;
        }
        let (lo, hi) = (*counts.iter().min().unwrap(), *counts.iter().max().unwrap());
        let mean = 2000 / SLOTS;
        assert!(
            lo > mean / 2 && hi < mean * 2,
            "slot occupancy {counts:?} is not close to uniform (mean {mean})"
        );
    }
}
