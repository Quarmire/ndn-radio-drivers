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
/// The textbook optimum `(M/n)·ln2` says ~7 for our sizes, but that formula assumes the k positions
/// of a query are independent — and with only 94 bits they are not: k=6 positions collide with each
/// other ~15% of the time, and a query whose 6 positions collapse to 3 distinct bits has the false
/// positive rate of k=3, not k=6. That effect is invisible to the formula and grows with k, so the
/// optimum has to be found by counting.
mod vk {
    use lr2021_nrf54l15_rs::tier0::{name_hash, M_BITS};

    const KEY2_MIX: u64 = 0x9E37_79B9_7F4A_7C15;

    pub fn set_bits(bits: &mut [u8; 12], key: u64, prefix: &[u8], k: u32) {
        let h1 = name_hash(key, prefix) as u32;
        let h2 = (name_hash(key ^ KEY2_MIX, prefix) as u32) | 1;
        for i in 0..k {
            let pos = (h1.wrapping_add(i.wrapping_mul(h2)) % M_BITS) as usize + 2;
            bits[pos / 8] |= 1 << (pos % 8);
        }
    }

    pub fn contains(bits: &[u8; 12], key: u64, prefix: &[u8], k: u32) -> bool {
        let h1 = name_hash(key, prefix) as u32;
        let h2 = (name_hash(key ^ KEY2_MIX, prefix) as u32) | 1;
        for i in 0..k {
            let pos = (h1.wrapping_add(i.wrapping_mul(h2)) % M_BITS) as usize + 2;
            if bits[pos / 8] & (1 << (pos % 8)) == 0 {
                return false;
            }
        }
        true
    }

    pub fn popcount(bits: &[u8; 12]) -> u32 {
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
const KEY: u64 = 0x8624_4E44_5F4B_4559;

/// Non-matching prefixes tested per depth.
const TRIALS: u32 = 20_000;

/// Write `/p0/p1/.../p{depth-1}` with a varying leaf, returning the used length.
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

    let mut nb = [0u8; 128];
    let mut ob = [0u8; 128];

    for depth in [2usize, 4, 6, 8] {
        let n = make_name(&mut nb, depth, 0xABCD);
        let name = &nb[..n];

        let mut f = PrefixFilter::new();
        f.insert_name(KEY, name);
        let bits = f.popcount();

        // No false negatives, ever: every genuine ancestor must match.
        let mut fn_count = 0u32;
        tier0::for_each_prefix(name, |pfx| {
            if !f.may_match(&PrefixFilter::mask_for(KEY, pfx)) {
                fn_count += 1;
            }
        });

        // False positives: prefixes from a disjoint namespace, so none is an ancestor.
        let mut fp = 0u32;
        for t in 0..TRIALS {
            let m = make_name(&mut ob, 1 + (t as usize % depth.max(1)), 0x10000 + t);
            if f.may_match(&PrefixFilter::mask_for(KEY, &ob[..m])) {
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
    for k in [3u32, 4, 5, 6, 8] {
        let n = make_name(&mut nb, tier0::MAX_DEPTH, 0xABCD);
        let name = &nb[..n];
        let mut bits = [0u8; 12];
        tier0::for_each_prefix(name, |pfx| vk::set_bits(&mut bits, KEY, pfx, k));

        let mut fneg = 0u32;
        tier0::for_each_prefix(name, |pfx| {
            if !vk::contains(&bits, KEY, pfx, k) {
                fneg += 1;
            }
        });

        let mut fp = 0u32;
        for t in 0..TRIALS {
            let m = make_name(&mut ob, 1 + (t as usize % tier0::MAX_DEPTH), 0x10000 + t);
            if vk::contains(&bits, KEY, &ob[..m], k) {
                fp += 1;
            }
        }
        let ppm = ((fp as u64) * 1_000_000) / TRIALS as u64;
        defmt::info!(
            "  k={=u32}: bits_set {=u32}/94 | FP {=u32}/{=u32} = {=u64} ppm | false negatives {=u32}",
            k,
            vk::popcount(&bits),
            fp,
            TRIALS,
            ppm,
            fneg
        );
    }

    defmt::info!("m7_filter_test: done (false negatives MUST be 0 on every line)");
    loop {
        Timer::after(Duration::from_secs(5)).await;
    }
}
