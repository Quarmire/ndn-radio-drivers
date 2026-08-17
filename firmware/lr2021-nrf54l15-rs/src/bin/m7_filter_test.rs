//! **M7a — measure the Tier-0 filter's false-positive rate on device.**
//!
//! #91's design predicts FP from `(1 − e^(−kn/M))^k`. That formula assumes independent, uniform hash
//! positions; whether it holds for *our* hash on *our* name shapes is the open validation item in
//! that design, and it is answered by counting, not by trusting the algebra.
//!
//! Method: for each depth, build a filter from one name, then test it against many prefixes that are
//! **known not to be ancestors of it**. Every `may_match` is by definition a false positive — there
//! is no ambiguity to adjudicate. Also asserts the property the whole design rests on: every
//! *genuine* ancestor must match, always. A single miss there would be a false negative and would
//! invalidate the filter outright, so it is checked on every iteration rather than sampled.

#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use panic_probe as _;

use lr2021_nrf54l15_rs::tier0::{self, PrefixFilter};

/// Runtime-k variants of the filter, so K can be *measured* rather than assumed.
///
/// The textbook optimum `(M/n)·ln2` says ~11 for our sizes, but that formula assumes the k positions
/// of a query are independent — and with only 126 bits they are not: k positions collide with each
/// other, and a query whose positions collapse to fewer distinct bits has the false
/// positive rate of k=3, not k=6. That effect is invisible to the formula and grows with k, so the
/// optimum has to be found by counting.
mod vk {
    use lr2021_nrf54l15_rs::tier0::{name_hash, M_BITS};

    /// Domain separator for the second hash — must match `tier0::KEY2_DOMAIN`, or this sweep would
    /// be measuring a different filter from the one that ships.
    const KEY2_DOMAIN: [u8; 16] = *b"ndn/tier0-h2\0\0\0\0";

    fn key2(key: &[u8; 16]) -> [u8; 16] {
        let mut k = *key;
        let mut i = 0;
        while i < 16 {
            k[i] ^= KEY2_DOMAIN[i];
            i += 1;
        }
        k
    }

    pub fn set_bits(bits: &mut [u8; 16], key: &[u8; 16], prefix: &[u8], k: u32) {
        let h1 = name_hash(key, prefix) as u32;
        let h2 = (name_hash(&key2(key), prefix) as u32) | 1;
        for i in 0..k {
            let pos = (h1.wrapping_add(i.wrapping_mul(h2)) % M_BITS) as usize + 2;
            bits[pos / 8] |= 1 << (pos % 8);
        }
    }

    pub fn contains(bits: &[u8; 16], key: &[u8; 16], prefix: &[u8], k: u32) -> bool {
        let h1 = name_hash(key, prefix) as u32;
        let h2 = (name_hash(&key2(key), prefix) as u32) | 1;
        for i in 0..k {
            let pos = (h1.wrapping_add(i.wrapping_mul(h2)) % M_BITS) as usize + 2;
            if bits[pos / 8] & (1 << (pos % 8)) == 0 {
                return false;
            }
        }
        true
    }

    pub fn popcount(bits: &[u8; 16]) -> u32 {
        let mut n = 0;
        for p in 0..M_BITS as usize {
            let q = p + 2;
            if bits[q / 8] & (1 << (q % 8)) != 0 {
                n += 1;
            }
        }
        n
    }
}

/// Group key. Any fixed value; the filter is keyed so a private group is unlinkable.
const KEY: [u8; 16] = *b"ndn/tier0-group!";

/// Non-matching prefixes tested per depth.
const TRIALS: u32 = 20_000;

/// Write `/p0/p1/.../p{depth-1}` with a varying leaf, returning the used length.
/// A name from a namespace that shares **no prefix at any depth** with [`make_name`]'s.
///
/// The first component is `/f...` where [`make_name`]'s is always `/0000`, so divergence happens at
/// component 0 and every deeper prefix inherits it. This is the property the false-positive
/// measurement depends on: a query that is a genuine ancestor is a true match, not a false positive,
/// and counting it as one silently flatters — or here, wrecks — the number.
fn make_disjoint_name(buf: &mut [u8], depth: usize, salt: u32) -> usize {
    let mut n = 0;
    for c in 0..depth {
        buf[n] = b'/';
        n += 1;
        // 0xf000 | .. keeps the first component (and every component) out of make_name's 0x0000..
        // index range, so the two namespaces cannot meet.
        let v = 0xf000u32 | ((if c + 1 == depth { salt } else { c as u32 }) & 0x0fff);
        for shift in [12, 8, 4, 0] {
            let d = ((v >> shift) & 0xf) as u8;
            buf[n] = if d < 10 { b'0' + d } else { b'a' + d - 10 };
            n += 1;
        }
    }
    n
}

fn make_name(buf: &mut [u8], depth: usize, salt: u32) -> usize {
    let mut n = 0;
    for c in 0..depth {
        buf[n] = b'/';
        n += 1;
        let v = if c + 1 == depth { salt } else { c as u32 };
        // Fixed 4 hex digits keeps component length constant so depth is the only variable.
        for shift in [12, 8, 4, 0] {
            let d = ((v >> shift) & 0xf) as u8;
            buf[n] = if d < 10 { b'0' + d } else { b'a' + d - 10 };
            n += 1;
        }
    }
    n
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let _p = embassy_nrf::init(Default::default());
    Timer::after(Duration::from_millis(200)).await;

    defmt::info!(
        "m7_filter_test: M={=u32} bits, K={=u32}, depth cap {}",
        tier0::M_BITS,
        tier0::K,
        tier0::MAX_DEPTH
    );

    // **Pin the vendored SipHash to the published vector before measuring anything with it.**
    //
    // This module's `siphash24` is a hand-copy of the one in `ndn-rs/.../ndn-frame-io/src/frame.rs`
    // (copied because this crate must stay no_std and dependency-free for the FLPR RISC-V core).
    // Two copies of a hash is exactly the kind of thing that drifts silently: a filter built by the
    // Wi-Fi face would simply stop matching masks built here, with no error — names would just quit
    // matching. Both copies assert the same Aumasson & Bernstein vector, so a divergence fails loudly
    // on whichever side broke.
    {
        let key: [u8; 16] = [
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e,
            0x0f,
        ];
        let data: [u8; 15] = [
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e,
        ];
        let got = tier0::siphash24(&key, &data);
        if got == 0xa129_ca61_49be_45e5 {
            defmt::info!("m7: siphash24 reference vector OK — this copy matches ndn-frame-io");
        } else {
            defmt::error!(
                "m7: *** siphash24 REFERENCE VECTOR FAILED: {=u64:#018x} (want 0xa129ca6149be45e5) — \
                 this copy has drifted from ndn-frame-io; Tier-0 filters will not interoperate ***",
                got
            );
        }
    }

    let mut nb = [0u8; 128];
    let mut ob = [0u8; 128];

    for depth in [2usize, 4, 6, 8] {
        let n = make_name(&mut nb, depth, 0xABCD);
        let name = &nb[..n];

        let mut f = PrefixFilter::new();
        f.insert_name(&KEY, name);
        let bits = f.popcount();

        // No false negatives, ever: every genuine ancestor must match.
        let mut fn_count = 0u32;
        tier0::for_each_prefix(name, |pfx| {
            if !f.may_match(&PrefixFilter::mask_for(&KEY, pfx)) {
                fn_count += 1;
            }
        });

        // False positives: names from a namespace disjoint AT EVERY DEPTH, so none is an ancestor.
        //
        // This used to pass `0x10000 + t` as the salt to `make_name`, intending "a different
        // namespace". `make_name` formats the salt as **4 hex digits**, so the 0x10000 was silently
        // truncated away and the query became `make_name(d, t)` — whose leading components are the
        // same `/0000/0001/...` index sequence as the registered name. The queries were therefore
        // siblings of genuine ancestors rather than disjoint names, which inflated the count with
        // real matches. It showed as 13.4% "FP" at depth 8 while the k-sweep on the same device, at
        // the same depth and k, measured 1.06% — two numbers from one run that could not both be
        // right. `make_disjoint_name` differs in the FIRST component, so no prefix of a query can be
        // a prefix of the registered name at any depth.
        let mut fp = 0u32;
        for t in 0..TRIALS {
            let m = make_disjoint_name(&mut ob, 1 + (t as usize % depth.max(1)), t);
            if f.may_match(&PrefixFilter::mask_for(&KEY, &ob[..m])) {
                fp += 1;
            }
        }

        // ppm avoids floats on a target with no formatter for them.
        let ppm = ((fp as u64) * 1_000_000) / TRIALS as u64;
        defmt::info!(
            "  depth {=usize}: bits_set {=u32}/{=u32} | FALSE POSITIVES {=u32}/{=u32} = {=u64} ppm | false negatives {=u32}",
            depth,
            bits,
            tier0::M_BITS,
            fp,
            TRIALS,
            ppm,
            fn_count
        );
    }

    // ── k sweep at the depth cap, where the filter is most loaded and k matters most ───────────
    defmt::info!("m7_filter_test: k sweep at depth {} (the cap — worst case)", tier0::MAX_DEPTH);
    // **Averaged over R registered names, not one.** The sweep that originally chose k = 4 used a
    // single name and a false-positive generator whose "disjoint" namespace was not disjoint (the
    // salt truncated — see `make_disjoint_name`). Fixing the generator inverted the ordering, so the
    // k decision is being re-measured properly: one draw is what produced the wrong answer the first
    // time, and re-running one draw with a better generator would only replace it with a luckier one.
    const R: u32 = 12;
    for k in [3u32, 4, 5, 6, 8] {
        let (mut fp_tot, mut fneg_tot, mut bits_tot, mut trials_tot) = (0u32, 0u32, 0u32, 0u32);
        for r in 0..R {
            let n = make_name(&mut nb, tier0::MAX_DEPTH, 0x1000 + r * 0x111);
            let name = &nb[..n];
            let mut bits = [0u8; 16];
            tier0::for_each_prefix(name, |pfx| vk::set_bits(&mut bits, &KEY, pfx, k));
            bits_tot += vk::popcount(&bits);

            tier0::for_each_prefix(name, |pfx| {
                if !vk::contains(&bits, &KEY, pfx, k) {
                    fneg_tot += 1;
                }
            });

            let per = TRIALS / R;
            for t in 0..per {
                let m = make_disjoint_name(&mut ob, 1 + (t as usize % tier0::MAX_DEPTH), r * TRIALS + t);
                if vk::contains(&bits, &KEY, &ob[..m], k) {
                    fp_tot += 1;
                }
            }
            trials_tot += per;
        }
        let ppm = ((fp_tot as u64) * 1_000_000) / trials_tot as u64;
        defmt::info!(
            "  k={=u32}: bits_set(avg) {=u32}/126 | FP {=u32}/{=u32} = {=u64} ppm | false negatives {=u32}",
            k,
            bits_tot / R,
            fp_tot,
            trials_tot,
            ppm,
            fneg_tot
        );
    }

    defmt::info!("m7_filter_test: done (false negatives MUST be 0 on every line)");
    loop {
        Timer::after(Duration::from_secs(5)).await;
    }
}
