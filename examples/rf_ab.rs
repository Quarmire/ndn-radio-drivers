//! `rf_ab` — a repeated-measures A/B(/C) runner for **any** radio in this fleet.
//!
//! ## Why this exists
//!
//! On 2026-08-28 three conclusions were drawn from single-run comparisons on one RTL8812AU and all
//! three were retracted within the hour. The numbers behind them (MEASURED, same binary, same
//! command, six back-to-back runs on ch36) were `199, 201, 112, 62, 41, 134` f/s: mean 124.8,
//! sd 67.1, **CV 53.8 %**. For a single unpaired A-vs-B run the sd of the log-ratio is
//! `0.538·√2 = 0.761`, so **under a true null a one-run-vs-one-run comparison shows a ≥10 %
//! "effect" with probability 0.90, ≥20 % with 0.81, and ≥2× with 0.36.** Three retractions in an
//! hour is the empirical confirmation, not a coincidence.
//!
//! This harness is built so that specific failure cannot repeat. Six properties do the work:
//!
//! 1. **Paired and interleaved.** The unit is a *round*: one rep of every arm, adjacent in time, in
//!    a randomised (seeded, printed) order. Drift shared by both halves of a round cancels in the
//!    ratio. MEASURED lag-1 autocorrelation of the six runs was +0.44 and Spearman(rate, index)
//!    −0.60 — the noise has slow structure, so blocked designs alias drift onto the arm label.
//! 2. **Every arm re-asserts its FULL configuration on every rep.** Not "when it differs from the
//!    last arm". An earlier A/B in this campaign was invalid precisely because its control arm
//!    skipped the write and silently inherited the previous arm's state. This harness goes further
//!    and *refuses to start* if an arm omits a knob that any other arm sets: an unmentioned knob is
//!    an inherited knob.
//! 3. **Read-backs prove which state the radio reached** — the four EDCA words, REG_SLOT, TXPAUSE,
//!    the CCA nibble, EDCCA honor, IGI/occupancy, TXAGC — compared *programmatically* against what
//!    was asked for. A mismatch invalidates the rep; it does not print a warning a human is
//!    supposed to catch at 3 a.m.
//! 4. **A robust statistic with an INCONCLUSIVE verdict.** Hodges-Lehmann pseudomedian of the
//!    paired differences, bootstrap-percentile CI, exact sign test + Wilcoxon. Three outcomes:
//!    `DIFFERENT`, `INDISTINGUISHABLE`, `INCONCLUSIVE`. The third is the one this tree has never
//!    printed and the one that stops the retractions. A point estimate from an INCONCLUSIVE run may
//!    not be quoted anywhere.
//! 5. **Raw per-rep numbers are printed**, so a reader sees the distribution rather than a mean.
//! 6. **A drift guard**: the first arm is repeated last, and that comparison must contain 0.
//!
//! ## The estimator, and why not f/s
//!
//! The metric is the **median per-call period in µs**, never `n/secs`. `inject` on the USB backends
//! is one synchronous `write_bulk` with a 100 ms timeout (`TX_TIMEOUT`, src/rtl8812au.rs:4023,
//! :6639), so a stalled call costs 100 ms while a healthy one costs ~2 ms. Over that bimodal
//! mixture `1/mean` is a lever on the *outlier rate* while the mode is stable — which is the
//! mathematical reason a small censoring change reads as a 5× throughput change. p50 does not move
//! when 5 % of calls stall; it moves when the medium access changes, which is what a contention
//! knob does. Outcomes are counted separately (`ok / timeout / other`) and a rep whose arms differ
//! in censoring by >2 points is reported as a mechanism finding, not a knob result.
//!
//! ⚠ `FrameIo::inject` currently discards the transferred byte count
//! (`write_bulk(...).map(|_| ())`, src/rtl8812au.rs:6639), so a **short write is indistinguishable
//! from a full one here**. The a81a path already asserts `n == buf.len()`
//! (src/libusb_rtl88xx.rs:4748-4751). Until the 8812au does the same, this harness reports `ok`,
//! never "delivered". Likewise `read_tx_counters` is unimplemented for the 8812au (HAL default
//! `Ok(None)`), so with no truth counter every rate printed here is labelled **offered**.
//!
//! ## Usage
//!
//! ```text
//! # 8812AU, ch36, contention posture, with a contemporaneous sham arm (base vs base):
//! NDN_AB_ARMS='shared:contention=shared|owned:contention=owned|sham:contention=shared' \
//! NDN_AB_ROUNDS=25 NDN_AB_REP_MS=1500 NDN_AB_LEN=200 NDN_AB_PHY_MBPS=6 \
//!   cargo run --release --example rf_ab -- 36
//!
//! # any other radio: pick the PID, use the same arm grammar (knobs are HAL-generic)
//! NDN_AB_PID=a81a NDN_AB_ARMS='lo:txpower=20|hi:txpower=63' cargo run --release --example rf_ab -- 36
//!
//! # harness floor, NO HARDWARE: the identical loop against the loopback bus
//! NDN_AB_LOOPBACK=1 NDN_AB_ARMS='a:|b:' cargo run --release --example rf_ab
//! ```
//!
//! Arm grammar: `name:key=val,key=val | name:key=val,...`. Keys (all applied through the HAL, so
//! they work on every backend that implements them; a knob a radio does not have returns
//! `Unsupported` and **invalidates the rep** rather than silently doing nothing):
//! `contention=shared|owned|yielding`, `txpower=<idx>`, `txdbm=<dbm>`, `edcca_ignore=0|1`,
//! `channel=<n>`, `rxgain=auto|boosted`, `mcs=<idx>[+vht][+sgi][+stbc][+ldpc]`, `sf=<7..12>`,
//! `cr=<1..4>`, `bwkhz=<125|250|500>`.
//!
//! Env knobs: `NDN_AB_PID` (hex, default 8812), `NDN_AB_ROUNDS` (20), `NDN_AB_REP_MS` (1500),
//! `NDN_AB_SETTLE_MS` (200), `NDN_AB_LEN` (200), `NDN_AB_PHY_MBPS` (6.0), `NDN_AB_THETA_PCT` (10),
//! `NDN_AB_SEED`, `NDN_AB_TAIL` (3), `NDN_AB_MIN_SAMPLES` (30), `NDN_AB_PUMP` (0 = no RX pump),
//! `NDN_AB_RAW80211` (build the 802.11 header here instead of in the driver),
//! `NDN_AB_STRICT` (1 = a register read-back mismatch invalidates the rep), `NDN_AB_FORCE`
//! (run even when the gates say the comparison cannot resolve its own effect), `NDN_AB_LOOPBACK`.
//!
//! Device selection reuses the driver's own `NDN_USB_ADDR` / `NDN_USB_INDEX` (`DeviceSelect::from_env`).

use std::collections::BTreeMap;
use std::sync::Arc;
use std::time::{Duration, Instant};

use bytes::Bytes;
use ndn_radio_drivers::{
    BROADCAST, DEFAULT_SRC, DeviceSelect, FrameFormat, FrameIo, InjectFrame, LibUsbRtl88xxBackend,
    McsDescriptor, RTL8812AU_PIDS, Reliability, Rtl8812auBackend, TxIntent, frame as dot11,
    realtek_contention,
};
use ndn_radio_hal::bringup::{PowerRequest, ProofRequirement, Role};
use ndn_radio_hal::{
    Bandwidth, ContentionApplied, ContentionPosture, FaceError, RadioKnobs, RxGain,
};

// ════════════════════════════════════════════════════════════════════════════════════════════
// RNG — seeded, printed, replayable. splitmix64; no external crate.
// ════════════════════════════════════════════════════════════════════════════════════════════

struct Rng(u64);

impl Rng {
    fn new(seed: u64) -> Self {
        Rng(seed)
    }
    fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
    fn below(&mut self, n: usize) -> usize {
        if n == 0 {
            0
        } else {
            (self.next_u64() % n as u64) as usize
        }
    }
    fn shuffle<T>(&mut self, v: &mut [T]) {
        for i in (1..v.len()).rev() {
            let j = self.below(i + 1);
            v.swap(i, j);
        }
    }
}

// ════════════════════════════════════════════════════════════════════════════════════════════
// Statistics — distribution-free throughout. The data is bimodal (a ~2 ms mode plus a 100 ms
// timeout mode) and bounded above by physics, so nothing here assumes normality or symmetry of
// the marginals; the paired tests assume only symmetry of the DIFFERENCES, and the sign test not
// even that.
// ════════════════════════════════════════════════════════════════════════════════════════════

fn sorted(xs: &[f64]) -> Vec<f64> {
    let mut v = xs.to_vec();
    v.sort_by(|a, b| a.partial_cmp(b).unwrap());
    v
}

/// Quantile of an already-sorted slice (nearest-rank, no interpolation — honest for small n).
fn quant(s: &[f64], q: f64) -> f64 {
    if s.is_empty() {
        return f64::NAN;
    }
    let i = ((q * s.len() as f64).ceil() as usize).clamp(1, s.len()) - 1;
    s[i]
}

fn median(xs: &[f64]) -> f64 {
    let s = sorted(xs);
    if s.is_empty() {
        return f64::NAN;
    }
    if s.len() % 2 == 1 {
        s[s.len() / 2]
    } else {
        0.5 * (s[s.len() / 2 - 1] + s[s.len() / 2])
    }
}

fn mean(xs: &[f64]) -> f64 {
    if xs.is_empty() {
        return f64::NAN;
    }
    xs.iter().sum::<f64>() / xs.len() as f64
}

fn sd(xs: &[f64]) -> f64 {
    if xs.len() < 2 {
        return f64::NAN;
    }
    let m = mean(xs);
    (xs.iter().map(|x| (x - m) * (x - m)).sum::<f64>() / (xs.len() - 1) as f64).sqrt()
}

/// Hodges-Lehmann pseudomedian: median of all pairwise Walsh averages `(d_i + d_j)/2`, i ≤ j.
/// Breakdown 0.29 and matched to the Wilcoxon signed-rank test, so the point estimate and the
/// p-value are talking about the same quantity.
fn hodges_lehmann(d: &[f64]) -> f64 {
    if d.is_empty() {
        return f64::NAN;
    }
    let mut w = Vec::with_capacity(d.len() * (d.len() + 1) / 2);
    for i in 0..d.len() {
        for j in i..d.len() {
            w.push(0.5 * (d[i] + d[j]));
        }
    }
    median(&w)
}

/// Bootstrap-percentile CI **of the pairs** (resample rounds, not calls — the round is the
/// experimental unit). Falls back to the median statistic above 120 pairs where HL's O(n²) inner
/// loop stops being free; the label says which was used.
fn bootstrap_ci(d: &[f64], rng: &mut Rng, iters: usize) -> (f64, f64, &'static str) {
    if d.len() < 3 {
        return (f64::NAN, f64::NAN, "n<3");
    }
    let use_hl = d.len() <= 120;
    let mut stats = Vec::with_capacity(iters);
    let mut buf = vec![0.0; d.len()];
    for _ in 0..iters {
        for b in buf.iter_mut() {
            *b = d[rng.below(d.len())];
        }
        stats.push(if use_hl {
            hodges_lehmann(&buf)
        } else {
            median(&buf)
        });
    }
    let s = sorted(&stats);
    (
        quant(&s, 0.025),
        quant(&s, 0.975),
        if use_hl { "HL" } else { "median" },
    )
}

fn ln_binom(n: usize, k: usize) -> f64 {
    let mut acc = 0.0;
    for i in 0..k {
        acc += ((n - i) as f64).ln() - ((i + 1) as f64).ln();
    }
    acc
}

/// Exact two-sided sign test — the assumption-free backstop. Zeros are dropped (the standard
/// treatment); `(pos, neg, p)`.
fn sign_test(d: &[f64]) -> (usize, usize, f64) {
    let pos = d.iter().filter(|x| **x > 0.0).count();
    let neg = d.iter().filter(|x| **x < 0.0).count();
    let n = pos + neg;
    if n == 0 {
        return (0, 0, 1.0);
    }
    let k = pos.max(neg);
    let ln_half_n = (n as f64) * 0.5f64.ln();
    let mut tail = 0.0;
    for i in k..=n {
        tail += (ln_binom(n, i) + ln_half_n).exp();
    }
    (pos, neg, (2.0 * tail).min(1.0))
}

/// Wilcoxon signed-rank, normal approximation with tie correction. Approximate below n≈10 — the
/// sign test above is exact and is the tie-breaker when the two disagree.
fn wilcoxon_p(d: &[f64]) -> f64 {
    let nz: Vec<f64> = d.iter().copied().filter(|x| *x != 0.0).collect();
    let n = nz.len();
    if n < 2 {
        return f64::NAN;
    }
    let mut idx: Vec<usize> = (0..n).collect();
    idx.sort_by(|a, b| nz[*a].abs().partial_cmp(&nz[*b].abs()).unwrap());
    let mut ranks = vec![0.0f64; n];
    let mut tie_sum = 0.0f64;
    let mut i = 0;
    while i < n {
        let mut j = i;
        while j + 1 < n && (nz[idx[j + 1]].abs() - nz[idx[i]].abs()).abs() < f64::EPSILON {
            j += 1;
        }
        let avg = ((i + 1 + j + 1) as f64) / 2.0;
        for k in i..=j {
            ranks[idx[k]] = avg;
        }
        let t = (j - i + 1) as f64;
        tie_sum += t * t * t - t;
        i = j + 1;
    }
    let w_pos: f64 = (0..n).filter(|k| nz[*k] > 0.0).map(|k| ranks[k]).sum();
    let nf = n as f64;
    let mu = nf * (nf + 1.0) / 4.0;
    let var = nf * (nf + 1.0) * (2.0 * nf + 1.0) / 24.0 - tie_sum / 48.0;
    if var <= 0.0 {
        return f64::NAN;
    }
    let z = (w_pos - mu) / var.sqrt();
    2.0 * (1.0 - normal_cdf(z.abs()))
}

/// Φ(x) via erf; good to ~1e-7, which is far more than a p-value needs.
fn normal_cdf(x: f64) -> f64 {
    0.5 * (1.0 + erf(x / std::f64::consts::SQRT_2))
}

fn erf(x: f64) -> f64 {
    // Abramowitz & Stegun 7.1.26.
    let s = if x < 0.0 { -1.0 } else { 1.0 };
    let x = x.abs();
    let t = 1.0 / (1.0 + 0.3275911 * x);
    let y = 1.0
        - (((((1.061405429 * t - 1.453152027) * t) + 1.421413741) * t - 0.284496736) * t
            + 0.254829592)
            * t
            * (-x * x).exp();
    s * y
}

fn rank(xs: &[f64]) -> Vec<f64> {
    let n = xs.len();
    let mut idx: Vec<usize> = (0..n).collect();
    idx.sort_by(|a, b| xs[*a].partial_cmp(&xs[*b]).unwrap());
    let mut r = vec![0.0; n];
    let mut i = 0;
    while i < n {
        let mut j = i;
        while j + 1 < n && (xs[idx[j + 1]] - xs[idx[i]]).abs() < f64::EPSILON {
            j += 1;
        }
        let avg = ((i + 1 + j + 1) as f64) / 2.0;
        for k in i..=j {
            r[idx[k]] = avg;
        }
        i = j + 1;
    }
    r
}

/// Spearman ρ plus an approximate two-sided p (t-approximation). Used as the within-arm trend
/// test: the six MEASURED runs score ρ = −0.60, and a run scoring like that is VOID.
fn spearman(xs: &[f64], ys: &[f64]) -> (f64, f64) {
    let n = xs.len();
    if n < 3 {
        return (f64::NAN, f64::NAN);
    }
    let (rx, ry) = (rank(xs), rank(ys));
    let (mx, my) = (mean(&rx), mean(&ry));
    let mut num = 0.0;
    let (mut dx, mut dy) = (0.0, 0.0);
    for i in 0..n {
        num += (rx[i] - mx) * (ry[i] - my);
        dx += (rx[i] - mx).powi(2);
        dy += (ry[i] - my).powi(2);
    }
    if dx <= 0.0 || dy <= 0.0 {
        return (0.0, 1.0);
    }
    let rho = num / (dx * dy).sqrt();
    let t = rho * (((n - 2) as f64) / (1.0 - rho * rho).max(1e-12)).sqrt();
    (rho, 2.0 * (1.0 - normal_cdf(t.abs())))
}

/// Bootstrap CI for the difference of medians of two INDEPENDENT groups (the drift guard: the
/// first arm's opening reps vs the same arm's closing reps are not paired).
fn boot_ci_diff_median(a: &[f64], b: &[f64], rng: &mut Rng, iters: usize) -> (f64, f64) {
    if a.len() < 2 || b.len() < 2 {
        return (f64::NAN, f64::NAN);
    }
    let mut stats = Vec::with_capacity(iters);
    let mut ba = vec![0.0; a.len()];
    let mut bb = vec![0.0; b.len()];
    for _ in 0..iters {
        for x in ba.iter_mut() {
            *x = a[rng.below(a.len())];
        }
        for x in bb.iter_mut() {
            *x = b[rng.below(b.len())];
        }
        stats.push(median(&bb) - median(&ba));
    }
    let s = sorted(&stats);
    (quant(&s, 0.025), quant(&s, 0.975))
}

// ════════════════════════════════════════════════════════════════════════════════════════════
// Physics — printed BEFORE the first frame, so a run that beats the medium is caught immediately
// rather than becoming a paragraph in a commit message.
// ════════════════════════════════════════════════════════════════════════════════════════════

/// 802.11a/g/n OFDM PPDU airtime, µs: 20 µs preamble+SIGNAL, then
/// `ceil((16 service + 8·(L+4 FCS) + 6 tail) / N_dbps) · 4 µs`, with `N_dbps = mbps · 4`.
/// This is standard PHY arithmetic, not a chip register — it is the only way to know whether a
/// measured rate is physically possible.
fn ofdm_airtime_us(payload_bytes: usize, mbps: f64) -> f64 {
    let n_dbps = mbps * 4.0;
    let bits = 16.0 + 8.0 * ((payload_bytes + 4) as f64) + 6.0;
    20.0 + (bits / n_dbps).ceil() * 4.0
}

// ════════════════════════════════════════════════════════════════════════════════════════════
// Arms
// ════════════════════════════════════════════════════════════════════════════════════════════

#[derive(Clone, Debug)]
struct Arm {
    name: String,
    spec: BTreeMap<String, String>,
    /// Canonical spec string — two arms with the same one are a SHAM pair (A vs A) and their
    /// comparison is the contemporaneous noise floor.
    key: String,
}

fn parse_arms(s: &str) -> Result<Vec<Arm>, String> {
    let mut arms = Vec::new();
    for chunk in s.split('|').map(str::trim).filter(|c| !c.is_empty()) {
        let (name, rest) = chunk.split_once(':').unwrap_or((chunk, ""));
        let mut spec = BTreeMap::new();
        for kv in rest.split(',').map(str::trim).filter(|c| !c.is_empty()) {
            let (k, v) = kv
                .split_once('=')
                .ok_or_else(|| format!("arm '{name}': setting '{kv}' is not key=value"))?;
            spec.insert(k.trim().to_ascii_lowercase(), v.trim().to_ascii_lowercase());
        }
        let key = spec
            .iter()
            .map(|(k, v)| format!("{k}={v}"))
            .collect::<Vec<_>>()
            .join(",");
        arms.push(Arm {
            name: name.trim().to_string(),
            spec,
            key,
        });
    }
    if arms.len() < 2 {
        return Err("need at least two arms (add a sham arm identical to the baseline)".into());
    }
    // ★ The rule that closes the inheritance hole: every arm must name every knob any arm uses.
    // A knob an arm does not mention is a knob it inherits from whichever arm ran before it, and
    // that is exactly what invalidated an earlier A/B in this campaign.
    let mut union: Vec<String> = Vec::new();
    for a in &arms {
        for k in a.spec.keys() {
            if !union.contains(k) {
                union.push(k.clone());
            }
        }
    }
    for a in &arms {
        for k in &union {
            if !a.spec.contains_key(k) {
                return Err(format!(
                    "arm '{}' does not set '{k}', which another arm does. An unmentioned knob is \
                     an INHERITED knob: state it explicitly (every arm asserts its full \
                     configuration on every rep).",
                    a.name
                ));
            }
        }
    }
    Ok(arms)
}

fn parse_mcs(v: &str) -> Result<McsDescriptor, String> {
    let mut parts = v.split('+');
    let idx: u8 = parts
        .next()
        .unwrap_or("")
        .parse()
        .map_err(|_| format!("mcs='{v}': index is not a number"))?;
    let mut m = McsDescriptor {
        index: idx,
        short_gi: false,
        vht: false,
        nss: 1,
        stbc: false,
        ldpc: false,
        he: false,
        dcm: false,
        er_su: false,
    };
    for f in parts {
        match f {
            "vht" => m.vht = true,
            "sgi" => m.short_gi = true,
            "stbc" => m.stbc = true,
            "ldpc" => m.ldpc = true,
            "he" => m.he = true,
            other => return Err(format!("mcs='{v}': unknown flag '{other}'")),
        }
    }
    Ok(m)
}

// ════════════════════════════════════════════════════════════════════════════════════════════
// Device — one concrete backend where deep register read-back exists, the generic opener
// otherwise. Every radio path yields the same two handles: an `Arc<dyn FrameIo>` to inject with
// and an optional `&dyn RadioKnobs` to actuate with.
// ════════════════════════════════════════════════════════════════════════════════════════════

enum Dev {
    /// RTL8812AU (`0bda:8812` …) — full register read-back.
    Au(Arc<Rtl8812auBackend>),
    /// RTL8822E / a81a (`0bda:a81a`, `0xa811`, `0x8814`) — EDCA + occupancy read-back.
    Xx(Arc<LibUsbRtl88xxBackend>),
    /// Anything `open_named_radio` dispatches (8733bu, mt7610/7612/7921, ath9k, …).
    Generic {
        io: Arc<dyn FrameIo>,
        knobs: Option<Arc<dyn RadioKnobs>>,
    },
    /// The harness's own floor: the hardware-free loopback bus (`LoopbackMonitorBus`).
    Loopback {
        io: Arc<dyn FrameIo>,
        _bus: Arc<ndn_frame_io::LoopbackMonitorBus>,
    },
}

impl Dev {
    fn io(&self) -> Arc<dyn FrameIo> {
        match self {
            Dev::Au(d) => d.clone(),
            Dev::Xx(d) => d.clone(),
            Dev::Generic { io, .. } => io.clone(),
            Dev::Loopback { io, .. } => io.clone(),
        }
    }

    fn knobs(&self) -> Option<&dyn RadioKnobs> {
        match self {
            Dev::Au(d) => Some(&**d),
            Dev::Xx(d) => Some(&**d),
            Dev::Generic { knobs, .. } => knobs.as_deref(),
            Dev::Loopback { .. } => None,
        }
    }

    fn label(&self) -> String {
        match self {
            Dev::Au(d) => format!("RTL8812AU pid {:#06x} ep {:?}", d.pid(), d.endpoints()),
            Dev::Xx(_) => "RTL8822E/88xx (a81a family)".into(),
            Dev::Generic { .. } => "generic (open_named_radio)".into(),
            Dev::Loopback { .. } => "LOOPBACK — harness floor, no radio".into(),
        }
    }

    /// One state vector. Every field is reachable through an accessor that already exists; nothing
    /// here is a guessed register. The **B-vector**: snapshot it after bring-up, compare it at
    /// every arm entry and exit, and a run that drifts is a skipped run rather than an invisible
    /// confounder.
    fn readback(&self) -> Vec<(&'static str, String)> {
        let mut v = Vec::new();
        match self {
            Dev::Au(d) => {
                // Four AC words, not just BE: the frames may ride QSLT_MGNT (the HIGH queue),
                // whose parameters are the VO word 0x0500 — reading only 0x0508 proves nothing
                // about the queue in use. (realtek_contention::EDCA_REGS = 0x500/0x504/0x508/0x50c)
                for (n, r) in ["edca_vo", "edca_vi", "edca_be", "edca_bk"]
                    .iter()
                    .zip(realtek_contention::EDCA_REGS)
                {
                    v.push((*n, fmt_res32(d.read32(r))));
                }
                // REG_SLOT 0x051b: NOT in the vendored MAC init table — pure inherited state, and
                // set_contention derives AIFS from it (realtek_contention.rs:184-191).
                v.push((
                    "slot_us(0x51b)",
                    fmt_res8(d.read8(realtek_contention::REG_SLOT)),
                ));
                // TXPAUSE 0x0522: 0x00 healthy / 0x3f aborted-IQK residue / 0xff aborted-LCK.
                // Written by lc_calibrate (rtl8812au.rs:5039/5062) and iqk_configure_mac (:5085),
                // and absent from the MAC init table, so it survives a re-open.
                v.push(("txpause(0x522)", fmt_res8(d.read8(0x0522))));
                // TX_PTCL_CTRL 0x0520 bit15 = ignore-EDCCA (rtl8812au.rs:331, :6189-6207).
                v.push(("tx_ptcl(0x520)", fmt_res32(d.read32(0x0520))));
                // REG_CR 0x0100 — MACTXEN (1<<6) | MACRXEN (1<<7) (rtl8812au.rs:377-378, :5679).
                v.push((
                    "cr(0x100)",
                    match d.read_cr() {
                        Ok(cr) => format!(
                            "{cr:#06x} tx_en={} rx_en={}",
                            cr & 0x40 != 0,
                            cr & 0x80 != 0
                        ),
                        Err(e) => format!("ERR {e}"),
                    },
                ));
                // REG_RXPKT_NUM 0x0284 bit18 = RW_RELEASE_EN (rtl8812au.rs:6318): clear = RX DMA
                // running. A pumped run and an unpumped run are different experiments.
                v.push(("rxpkt(0x284)", fmt_res32(d.read32(0x0284))));
                // BB 0x838[3:0]: 0x4 = OFDM packet CCA ARMED (the PHY_REG default), 0xc = CCA off
                // (what set_cca_ignore writes, and what an ABORTED IQK leaves behind —
                // iqk_configure_mac at rtl8812au.rs:5088 writes it before the restore block).
                // MEASURED consequence of that nibble on this part: ~600 frames in 20 s deferred
                // vs ~14000 not deferring — a >20x swing, larger than the 5x under investigation.
                v.push((
                    "bb_cca(0x838)",
                    match d.bb_read(0x838) {
                        Ok(w) => format!("{w:#010x} nibble={:#x}", w & 0xf),
                        Err(e) => format!("ERR {e}"),
                    },
                ));
                v.push(("bb_0x808", fmt_res32(d.bb_read(0x808))));
                // TXAGC — four distinguishable signatures: 0x12121212 (set_tx_power never landed),
                // 0x3f3f3f3f (uncalibrated fallback), ~0x2d (empty/garbled EFUSE), else real fuse.
                for a in [0xc20u16, 0xc24, 0xc28, 0xc2c, 0xc30] {
                    v.push((
                        Box::leak(format!("txagc({a:#06x})").into_boxed_str()),
                        fmt_res32(d.bb_read(a)),
                    ));
                }
                // IQ corrections actually applied: 0x200/0x000 = did not converge, identity used.
                for a in [0xcccu16, 0xcd4, 0xecc, 0xed4, 0xc10, 0xe10] {
                    v.push((
                        Box::leak(format!("iqk({a:#06x})").into_boxed_str()),
                        fmt_res32(d.bb_read(a)),
                    ));
                }
                v.push((
                    "edcca",
                    match d.edcca_state() {
                        Ok((l2h, h2l, honored)) => format!("l2h={l2h} h2l={h2l} honored={honored}"),
                        Err(e) => format!("ERR {e}"),
                    },
                ));
                v.push((
                    "phy_sense",
                    match d.read_phy_sense() {
                        Ok(s) => format!(
                            "igi_a={} igi_b={} rx_activity={}",
                            s.igi_a, s.igi_b, s.rx_activity
                        ),
                        Err(e) => format!("ERR {e}"),
                    },
                ));
            }
            Dev::Xx(d) => {
                for (n, r) in ["edca_vo", "edca_vi", "edca_be", "edca_bk"]
                    .iter()
                    .zip(realtek_contention::EDCA_REGS)
                {
                    v.push((*n, fmt_res32(d.read32(r))));
                }
                v.push((
                    "slot_us(0x51b)",
                    fmt_res8(d.read8(realtek_contention::REG_SLOT)),
                ));
                // 0x520 bit15 / 0x524 bit11 — the pair set_edcca_ignore drives on this backend
                // (libusb_rtl88xx.rs, set_edcca_ignore).
                v.push(("tx_ptcl(0x520)", fmt_res32(d.read32(0x0520))));
                v.push(("rd_ctrl(0x524)", fmt_res32(d.read32(0x0524))));
                v.push((
                    "rx_activity(0x664)",
                    match d.read_channel_activity() {
                        Ok(a) => a.to_string(),
                        Err(e) => format!("ERR {e}"),
                    },
                ));
            }
            _ => {}
        }
        // Portable half — works on every backend that implements the HAL knob.
        if let Some(k) = self.knobs() {
            v.push((
                "hal_activity",
                match k.read_channel_activity() {
                    Ok(Some(a)) => a.to_string(),
                    Ok(None) => "none".into(),
                    Err(e) => format!("ERR {e}"),
                },
            ));
            v.push((
                "hal_tx_counters",
                match k.read_tx_counters() {
                    Ok(Some((en, on))) => format!("tx_en={en} tx_on={on}"),
                    Ok(None) => "UNIMPLEMENTED (rate is OFFERED, never delivered)".into(),
                    Err(e) => format!("ERR {e}"),
                },
            ));
            v.push((
                "hal_ofdm_counters",
                match k.read_ofdm_counters() {
                    Ok(Some((ok, err))) => format!("ok={ok} err={err}"),
                    Ok(None) => "none".into(),
                    Err(e) => format!("ERR {e}"),
                },
            ));
        }
        v
    }

    /// The HAL occupancy counter, for the per-rep covariate (`None` when the radio has none).
    fn activity(&self) -> Option<u16> {
        self.knobs()
            .and_then(|k| k.read_channel_activity().ok().flatten())
    }

    /// `(tx_en, tx_on)` when the radio has a truth counter. `None` on the 8812au today — which is
    /// exactly why nothing in this harness is allowed to say "delivered".
    fn tx_counters(&self) -> Option<(u16, u16)> {
        self.knobs()
            .and_then(|k| k.read_tx_counters().ok().flatten())
    }
}

fn fmt_res32(r: Result<u32, FaceError>) -> String {
    match r {
        Ok(v) => format!("{v:#010x}"),
        Err(e) => format!("ERR {e}"),
    }
}

fn fmt_res8(r: Result<u8, FaceError>) -> String {
    match r {
        Ok(v) => format!("{v:#04x} ({v})"),
        Err(e) => format!("ERR {e}"),
    }
}

// ════════════════════════════════════════════════════════════════════════════════════════════
// Applying an arm — the whole configuration, every rep, checked
// ════════════════════════════════════════════════════════════════════════════════════════════

struct Applied {
    /// What each knob reported back, for printing.
    notes: Vec<String>,
    /// The contention the radio says it applied, when the arm sets that knob.
    contention: Option<ContentionApplied>,
    /// Non-empty ⇒ the rep is invalid.
    failures: Vec<String>,
}

fn apply_arm(dev: &Dev, io: &Arc<dyn FrameIo>, arm: &Arm) -> Applied {
    let mut out = Applied {
        notes: Vec::new(),
        contention: None,
        failures: Vec::new(),
    };
    let knobs = dev.knobs();
    for (k, v) in &arm.spec {
        let knobs = match knobs {
            Some(k) => k,
            None => {
                out.failures
                    .push(format!("{k}={v}: radio exposes no RadioKnobs"));
                continue;
            }
        };
        let res: Result<String, String> = match k.as_str() {
            "contention" => {
                let p = match v.as_str() {
                    "owned" => ContentionPosture::Owned,
                    "shared" => ContentionPosture::Shared,
                    "yielding" => ContentionPosture::Yielding,
                    _ => {
                        out.failures.push(format!("contention='{v}' unknown"));
                        continue;
                    }
                };
                match knobs.set_contention(p) {
                    Ok(a) => {
                        out.contention = Some(a);
                        Ok(format!(
                            "contention={v} => cw {}..{} aifs {} slot {} us, backoff {} us, \
                             medium access {} us",
                            a.cw_min,
                            a.cw_max,
                            a.aifs,
                            a.slot_us,
                            a.avg_backoff_us,
                            a.medium_access_us()
                        ))
                    }
                    Err(e) => Err(format!("set_contention({v}): {e}")),
                }
            }
            "txpower" => match v.parse::<u32>() {
                Ok(idx) => knobs
                    .set_tx_power(ndn_radio_hal::PowerRequest::index(idx.min(255) as u8))
                    // ★ Report what was APPLIED, not what was asked for: the whole point of the
                    // return type. A clamp or a different power reference shows up here.
                    .map(|a| format!("txpower={idx} -> {}", a.render()))
                    .map_err(|e| format!("set_tx_power({idx}): {e}")),
                Err(_) => Err(format!("txpower='{v}' is not a number")),
            },
            "txdbm" => match v.parse::<i8>() {
                Ok(dbm) => knobs
                    .set_tx_power_dbm(dbm)
                    .map(|got| format!("txdbm={dbm} => applied {got} dBm"))
                    .map_err(|e| format!("set_tx_power_dbm({dbm}): {e}")),
                Err(_) => Err(format!("txdbm='{v}' is not a number")),
            },
            "edcca_ignore" => {
                let on = v == "1" || v == "true" || v == "on";
                knobs
                    .set_edcca_ignore(on)
                    .map(|_| format!("edcca_ignore={on}"))
                    .map_err(|e| format!("set_edcca_ignore({on}): {e}"))
            }
            "channel" => match v.parse::<u8>() {
                Ok(ch) => knobs
                    .set_channel(ch, Bandwidth::Bw20)
                    .map(|_| format!("channel={ch}/20MHz"))
                    .map_err(|e| format!("set_channel({ch}): {e}")),
                Err(_) => Err(format!("channel='{v}' is not a number")),
            },
            "rxgain" => {
                let g = match v.as_str() {
                    "auto" => RxGain::Auto,
                    "boosted" => RxGain::Boosted,
                    _ => {
                        out.failures.push(format!("rxgain='{v}' unknown"));
                        continue;
                    }
                };
                knobs
                    .set_rx_gain(g)
                    .map(|_| format!("rxgain={v}"))
                    .map_err(|e| format!("set_rx_gain({v}): {e}"))
            }
            "mcs" => match parse_mcs(v) {
                Ok(m) => io
                    .set_rate(m)
                    .map(|_| format!("mcs={v} => {m:?}"))
                    .map_err(|e| format!("set_rate({v}): {e}")),
                Err(e) => Err(e),
            },
            "sf" => match v.parse::<u8>() {
                Ok(sf) => knobs
                    .set_spreading_factor(sf)
                    .map(|_| format!("sf={sf}"))
                    .map_err(|e| format!("set_spreading_factor({sf}): {e}")),
                Err(_) => Err(format!("sf='{v}' is not a number")),
            },
            "cr" => match v.parse::<u8>() {
                Ok(cr) => knobs
                    .set_coding_rate(cr)
                    .map(|_| format!("cr={cr}"))
                    .map_err(|e| format!("set_coding_rate({cr}): {e}")),
                Err(_) => Err(format!("cr='{v}' is not a number")),
            },
            "bwkhz" => match v.parse::<u32>() {
                Ok(b) => knobs
                    .set_bandwidth_khz(b)
                    .map(|_| format!("bwkhz={b}"))
                    .map_err(|e| format!("set_bandwidth_khz({b}): {e}")),
                Err(_) => Err(format!("bwkhz='{v}' is not a number")),
            },
            other => Err(format!("unknown knob '{other}'")),
        };
        match res {
            Ok(n) => out.notes.push(n),
            Err(e) => out.failures.push(e),
        }
    }
    out
}

/// Verify the contention posture actually landed in the registers, on the Realtek parts where the
/// encoding is known (`realtek_contention::encode_ac`). All four ACs, plus the slot the MAC is
/// really counting in. Returns the list of mismatches — empty is the only acceptable result.
fn verify_contention(dev: &Dev, arm: &Arm) -> Vec<String> {
    let posture = match arm.spec.get("contention").map(String::as_str) {
        Some("owned") => ContentionPosture::Owned,
        Some("shared") => ContentionPosture::Shared,
        Some("yielding") => ContentionPosture::Yielding,
        _ => return Vec::new(),
    };
    let rd32: Box<dyn Fn(u16) -> Result<u32, FaceError> + '_>;
    let rd8: Box<dyn Fn(u16) -> Result<u8, FaceError> + '_>;
    match dev {
        Dev::Au(d) => {
            rd32 = Box::new(move |a| d.read32(a));
            rd8 = Box::new(move |a| d.read8(a));
        }
        Dev::Xx(d) => {
            rd32 = Box::new(move |a| d.read32(a));
            rd8 = Box::new(move |a| d.read8(a));
        }
        // Not a Realtek EDCA block: the HAL's returned `ContentionApplied` is the only read-back
        // available, and the caller already checks it for stability across reps.
        _ => return Vec::new(),
    }
    let (cw_min, cw_max, aifsn) = realtek_contention::rtl_posture(posture);
    let cw_min = cw_min.max(realtek_contention::MIN_CW_EXPONENT);
    let cw_max = cw_max.max(cw_min);
    let slot = rd8(realtek_contention::REG_SLOT).unwrap_or(9);
    let slot = if (9..=20).contains(&slot) { slot } else { 9 };
    // ★ **Only BE is verified against the formula, and that is a CONTRACT point, not a loosening.**
    //
    // `ContentionApplied` is one tuple, but the MAC has four access categories which a real radio
    // does NOT boot with the same values in — MEASURED on this part: VO 0x002fa226, VI 0x005ea328,
    // BE 0x005ea42b, BK 0x0000a44f, each with its own AIFS and TXOP. So the returned tuple can only
    // honestly describe ONE queue, and the queue that carries our data frames is **BE**.
    //
    // `ContentionPosture::Shared` restores every AC to the value it booted with, deliberately: a
    // uniform rewrite would permanently destroy the vendor's per-AC tuning after the first `Owned`.
    // Those boot values are per-part facts that no formula predicts, so checking VO/VI/BK against
    // `encode_ac` fails on a CORRECT restore. That mismatch is what this harness reported on its
    // first real run, and the bug it exposed was in the contract's wording, not in the driver.
    //
    // The other three ACs are still read and reported — a change in them between reps is real
    // information — they simply do not invalidate a rep on their own.
    let mut bad = Vec::new();
    for (name, reg) in ["VO", "VI", "BE", "BK"]
        .iter()
        .zip(realtek_contention::EDCA_REGS)
    {
        match rd32(reg) {
            Ok(got) => {
                if *name != "BE" {
                    continue; // read for the record; see above
                }
                let want = realtek_contention::encode_ac(cw_min, cw_max, aifsn, slot, got >> 16);
                if got != want {
                    bad.push(format!(
                        "{name} {reg:#06x}: got {got:#010x}, expected {want:#010x} for {posture:?} \
                         (cw {cw_min}..{cw_max} aifsn {aifsn} slot {slot})"
                    ));
                }
            }
            Err(e) => bad.push(format!("{name} {reg:#06x}: read failed: {e}")),
        }
    }
    bad
}

// ════════════════════════════════════════════════════════════════════════════════════════════
// One rep
// ════════════════════════════════════════════════════════════════════════════════════════════

struct Rep {
    round: usize,
    arm: usize,
    ok: u64,
    timeout: u64,
    other_err: u64,
    p10: f64,
    p50: f64,
    p90: f64,
    p99: f64,
    max: f64,
    /// Per-second buckets of successful calls — within-run decay and between-run state look
    /// identical in one aggregate and have opposite causes.
    buckets: Vec<u64>,
    activity_delta: Option<u16>,
    txc_delta: Option<(u16, u16)>,
    valid: bool,
    why_invalid: Vec<String>,
}

impl Rep {
    fn n(&self) -> u64 {
        self.ok + self.timeout + self.other_err
    }
    fn censor_pct(&self) -> f64 {
        if self.n() == 0 {
            0.0
        } else {
            100.0 * (self.timeout + self.other_err) as f64 / self.n() as f64
        }
    }
    /// OFFERED calls per second (never "throughput", never "delivered" — see the module header).
    fn offered_fps(&self, secs: f64) -> f64 {
        self.ok as f64 / secs
    }
}

fn is_timeout(e: &FaceError) -> bool {
    let s = e.to_string().to_ascii_lowercase();
    s.contains("timed out") || s.contains("timeout")
}

#[allow(clippy::too_many_arguments)]
async fn run_rep(
    dev: &Dev,
    io: &Arc<dyn FrameIo>,
    arm: &Arm,
    arm_idx: usize,
    round: usize,
    frame: &InjectFrame,
    settle: Duration,
    measure: Duration,
    min_samples: u64,
    strict: bool,
    contention_seen: &mut Vec<Option<ContentionApplied>>,
) -> (
    Rep,
    Applied,
    Vec<(&'static str, String)>,
    Vec<(&'static str, String)>,
) {
    let mut why = Vec::new();

    // 1. Re-assert the FULL configuration. Every rep, every knob, no "only if it changed".
    let applied = apply_arm(dev, io, arm);
    why.extend(applied.failures.iter().cloned());

    // 2. Prove it landed.
    let bad = verify_contention(dev, arm);
    if !bad.is_empty() {
        if strict {
            why.extend(bad.iter().map(|b| format!("READBACK {b}")));
        } else {
            for b in &bad {
                println!("      ! readback (non-strict) {b}");
            }
        }
    }
    if let Some(a) = applied.contention {
        match contention_seen[arm_idx] {
            None => contention_seen[arm_idx] = Some(a),
            Some(first) if first != a => why.push(format!(
                "contention DRIFT within arm '{}': first {first:?}, now {a:?}",
                arm.name
            )),
            _ => {}
        }
    }

    let entry = dev.readback();
    let act0 = dev.activity();
    let txc0 = dev.tx_counters();

    // 3. Settle — the register write, the MAC reprogram and the TX-FIFO drain must all land
    //    OUTSIDE the measured window, identically in both arms (which is why a sham arm still
    //    performs the full write).
    let stop = Instant::now() + settle;
    while Instant::now() < stop {
        let _ = io.inject(frame.clone()).await;
    }

    // 4. Measure. Time every call; count outcomes separately. A failed call is NOT a frame.
    let mut periods: Vec<f64> = Vec::with_capacity(4096);
    let (mut ok, mut timeout, mut other_err) = (0u64, 0u64, 0u64);
    let secs = measure.as_secs_f64();
    let n_buckets = secs.ceil() as usize;
    let mut buckets = vec![0u64; n_buckets.max(1)];
    let t_start = Instant::now();
    let deadline = t_start + measure;
    loop {
        let a = Instant::now();
        if a >= deadline {
            break;
        }
        let r = io.inject(frame.clone()).await;
        let b = Instant::now();
        let us = (b - a).as_secs_f64() * 1e6;
        match r {
            Ok(()) => {
                ok += 1;
                periods.push(us);
                let bi = ((b - t_start).as_secs_f64().floor() as usize).min(buckets.len() - 1);
                buckets[bi] += 1;
            }
            Err(e) if is_timeout(&e) => timeout += 1,
            Err(_) => other_err += 1,
        }
    }

    let act1 = dev.activity();
    let txc1 = dev.tx_counters();
    let exit = dev.readback();

    let s = sorted(&periods);
    let rep = Rep {
        round,
        arm: arm_idx,
        ok,
        timeout,
        other_err,
        p10: quant(&s, 0.10),
        p50: quant(&s, 0.50),
        p90: quant(&s, 0.90),
        p99: quant(&s, 0.99),
        max: s.last().copied().unwrap_or(f64::NAN),
        buckets,
        activity_delta: match (act0, act1) {
            (Some(a), Some(b)) => Some(b.wrapping_sub(a)),
            _ => None,
        },
        txc_delta: match (txc0, txc1) {
            (Some((a0, b0)), Some((a1, b1))) => Some((a1.wrapping_sub(a0), b1.wrapping_sub(b0))),
            _ => None,
        },
        valid: true,
        why_invalid: Vec::new(),
    };
    let mut rep = rep;
    if ok < min_samples {
        why.push(format!(
            "thin: {ok} successful calls < NDN_AB_MIN_SAMPLES {min_samples}"
        ));
    }
    // A p50 of zero means the clock could not resolve the call — the harness, not the radio, is
    // what is being measured. Never divide by it.
    if !(rep.p50 > 0.0) {
        why.push("p50 <= 0: the per-call period is below the clock's resolution".into());
    }
    // An arm-entry-to-arm-exit difference that is not the knob is a confound, not noise.
    for ((n0, v0), (_, v1)) in entry.iter().zip(exit.iter()) {
        let watched = matches!(
            *n0,
            "txpause(0x522)" | "bb_cca(0x838)" | "cr(0x100)" | "edcca" | "slot_us(0x51b)"
        );
        if watched && v0 != v1 {
            why.push(format!("STATE MOVED during rep: {n0}: {v0} -> {v1}"));
        }
    }
    rep.valid = why.is_empty();
    rep.why_invalid = why;
    (rep, applied, entry, exit)
}

// ════════════════════════════════════════════════════════════════════════════════════════════
// main
// ════════════════════════════════════════════════════════════════════════════════════════════

fn env_u64(k: &str, d: u64) -> u64 {
    std::env::var(k)
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(d)
}
fn env_f64(k: &str, d: f64) -> f64 {
    std::env::var(k)
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(d)
}
fn env_flag(k: &str, d: bool) -> bool {
    match std::env::var(k) {
        Ok(v) => !(v == "0" || v.eq_ignore_ascii_case("false") || v.is_empty()),
        Err(_) => d,
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let channel: u8 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(36);
    let pid = u16::from_str_radix(
        std::env::var("NDN_AB_PID")
            .unwrap_or_else(|_| "8812".into())
            .trim_start_matches("0x"),
        16,
    )
    .map_err(|_| "NDN_AB_PID must be hex, e.g. 8812 or a81a")?;
    let rounds = env_u64("NDN_AB_ROUNDS", 20) as usize;
    let rep_ms = env_u64("NDN_AB_REP_MS", 1500);
    let settle_ms = env_u64("NDN_AB_SETTLE_MS", 200);
    let len = env_u64("NDN_AB_LEN", 200) as usize;
    let mbps = env_f64("NDN_AB_PHY_MBPS", 6.0);
    let theta = env_f64("NDN_AB_THETA_PCT", 10.0) / 100.0;
    let tail = env_u64("NDN_AB_TAIL", 3) as usize;
    let min_samples = env_u64("NDN_AB_MIN_SAMPLES", 30);
    let pump_depth = env_u64("NDN_AB_PUMP", 0) as usize;
    let strict = env_flag("NDN_AB_STRICT", true);
    let force = env_flag("NDN_AB_FORCE", false);
    let loopback = env_flag("NDN_AB_LOOPBACK", false);
    let raw80211 = env_flag("NDN_AB_RAW80211", false);
    let seed = std::env::var("NDN_AB_SEED")
        .ok()
        .and_then(|s| u64::from_str_radix(s.trim_start_matches("0x"), 16).ok())
        .unwrap_or_else(|| {
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map(|d| d.as_nanos() as u64)
                .unwrap_or(0x5f3a_91c2)
        });
    let arms = parse_arms(&std::env::var("NDN_AB_ARMS").unwrap_or_else(|_| {
        "shared:contention=shared|owned:contention=owned|sham:contention=shared".into()
    }))?;
    let mut rng = Rng::new(seed);

    let measure = Duration::from_millis(rep_ms);
    let settle = Duration::from_millis(settle_ms);
    let secs = measure.as_secs_f64();

    // ── Header: everything needed to reproduce or reject this run ───────────────────────────
    println!("rf_ab — repeated-measures A/B for named radios");
    println!(
        "  seed {seed:#018x} | ch{channel} | pid {pid:#06x} | {} arms | {rounds} rounds | \
         {rep_ms} ms/rep (+{settle_ms} ms settle discarded) | payload {len} B",
        arms.len()
    );
    for a in &arms {
        println!(
            "  arm '{}': {}",
            a.name,
            if a.key.is_empty() {
                "(no knobs)"
            } else {
                &a.key
            }
        );
    }
    for (i, a) in arms.iter().enumerate() {
        for b in arms.iter().skip(i + 1) {
            if a.key == b.key {
                println!(
                    "  ★ SHAM PAIR '{}' vs '{}' — identical configuration; their spread IS the \
                     contemporaneous noise floor.",
                    a.name, b.name
                );
            }
        }
    }
    println!(
        "  env: NDN_USB_ADDR={:?} NDN_USB_INDEX={:?} NDN_RADIO_TX_RATE={:?} NDN_RADIO_NO_RESET={:?} \
         NDN_NAVUSEHDR={:?} NDN_RX_AGG_OFF={:?} NDN_TX_PWR={:?}",
        std::env::var("NDN_USB_ADDR").ok(),
        std::env::var("NDN_USB_INDEX").ok(),
        std::env::var("NDN_RADIO_TX_RATE").ok(),
        std::env::var("NDN_RADIO_NO_RESET").ok(),
        std::env::var("NDN_NAVUSEHDR").ok(),
        std::env::var("NDN_RX_AGG_OFF").ok(),
        std::env::var("NDN_TX_PWR").ok(),
    );
    if let Ok(la) = std::fs::read_to_string("/proc/loadavg") {
        println!("  loadavg: {}", la.trim());
    }

    // ── The physics, before the first frame ─────────────────────────────────────────────────
    let air_us = ofdm_airtime_us(len, mbps);
    println!(
        "\n  PHY: {len} B (+4 FCS) at {mbps} Mbit/s OFDM = {air_us:.0} us of airtime. \
         (This model is OFDM only — on a LoRa/S1G bearer treat the ceiling gate as advisory.)"
    );

    let rt = tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()?;

    rt.block_on(async move {
        // ── Open ────────────────────────────────────────────────────────────────────────────
        let fmt = if raw80211 {
            FrameFormat::Raw80211
        } else {
            FrameFormat::RawNdn { ethertype: 0x8624 }
        };
        let dev: Dev = if loopback {
            let bus = Arc::new(ndn_frame_io::LoopbackMonitorBus::new());
            // A second endpoint keeps the broadcast channel non-degenerate.
            let _peer = bus.endpoint(2, -50);
            let ep = Arc::new(bus.endpoint(1, -50));
            Dev::Loopback { io: ep, _bus: bus }
        } else if RTL8812AU_PIDS.contains(&pid) {
            let sel = DeviceSelect::from_env();
            let d = Arc::new(Rtl8812auBackend::open_select(&sel)?.with_format(fmt));
            d.bring_up_planned(
                channel,
                Role::TransmitAndReceive,
                PowerRequest::ceiling(),
                None,
                ProofRequirement::BestAvailable,
            )?;
            if pump_depth > 0 {
                std::mem::forget(d.spawn_rx_pump(pump_depth));
            }
            Dev::Au(d)
        } else if matches!(pid, 0xa81a | 0xa811 | 0x8814) {
            let sel = DeviceSelect::from_env();
            let d = {
                // M8: `open_monitor*` is deleted. Claim, then run the ONE plan (`PLAN_A81A`)
                // with the role named at the call site — and keep the report instead of
                // discarding it, which is the thing those openers got wrong.
                let d = std::sync::Arc::new(LibUsbRtl88xxBackend::open_pid_select(pid, &sel)?);
                d.bring_up_planned(
                    channel,
                    ndn_radio_hal::bringup::Role::TransmitAndReceive,
                    ndn_radio_drivers::a81a_env_deviation(),
                    ndn_radio_hal::bringup::ProofRequirement::BestAvailable,
                )?;
                d
            };
            if pump_depth > 0 {
                std::mem::forget(d.spawn_rx_pump(pump_depth));
            }
            Dev::Xx(d)
        } else {
            // Everything else: the standardized opener. NB it starts its own RX pump unless
            // NDN_NO_PUMP=1 — a pumped run and an unpumped run are different experiments, so say
            // which one this is in the write-up.
            let o = ndn_radio_drivers::open_radio(
                pid,
                &ndn_radio_drivers::DeviceSelect::from_env(),
                &ndn_radio_drivers::BringUpRequest::from_env(channel),
            )?;
            Dev::Generic {
                io: o.io.clone(),
                knobs: o.knobs.clone(),
            }
        };
        let io = dev.io();
        println!("\n  device: {}", dev.label());
        if dev.knobs().is_none() && arms.iter().any(|a| !a.spec.is_empty()) {
            return Err::<(), Box<dyn std::error::Error>>(
                "this radio exposes no RadioKnobs, so no arm configuration can be asserted — \
                 refusing to produce a number"
                    .into(),
            );
        }

        // ── Post-bring-up state vector (the reference B-vector) ─────────────────────────────
        println!("  post-bring-up read-back:");
        let boot = dev.readback();
        for (k, v) in &boot {
            println!("    {k:<20} {v}");
        }

        // ── The frame, and proof of what it is ──────────────────────────────────────────────
        let payload: Vec<u8> = if raw80211 {
            // Raw80211 injects the payload VERBATIM as the 802.11 frame, so the harness must build
            // a real header: a 1400-byte block of filler is a MANAGEMENT frame with a UNICAST
            // addr1 (frame.rs:353-356 + the driver's queue pick from fc0), which rides a different
            // queue and waits out an ACK timeout on every frame.
            let mut p = Vec::with_capacity(24 + len);
            p.extend_from_slice(&[0x08, 0x00]); // FC: type=Data, subtype=Data
            p.extend_from_slice(&[0x00, 0x00]); // Duration
            p.extend_from_slice(&BROADCAST); // addr1 — MUST be group-addressed
            p.extend_from_slice(&DEFAULT_SRC); // addr2
            p.extend_from_slice(&BROADCAST); // addr3
            p.extend_from_slice(&[0x00, 0x00]); // SeqCtrl
            p.extend(std::iter::repeat(0x42).take(len));
            p
        } else {
            vec![0x42u8; len]
        };
        let frame = InjectFrame {
            payload: Bytes::from(payload),
            tx: TxIntent::broadcast(if arms.iter().any(|a| a.spec.contains_key("mcs")) {
                Reliability::Throughput
            } else {
                Reliability::MostRobust
            }),
            dst: BROADCAST,
            src: DEFAULT_SRC,
            addr3: None,
            extra: None,
            htc: None,
        };
        match dot11::build_dot11(fmt, &frame) {
            Ok(bytes) => {
                let head: Vec<String> = bytes.iter().take(32).map(|b| format!("{b:02x}")).collect();
                println!("  on-air frame ({} B): {}", bytes.len(), head.join(" "));
                if bytes.len() > 10 && bytes[4] & 1 != 1 {
                    return Err(
                        "FRAME IDENTITY GATE: addr1 is UNICAST — the MAC will wait out an ACK \
                         timeout on every frame and the DCF window can latch at CWmax. Fix the \
                         frame before measuring."
                            .into(),
                    );
                }
            }
            Err(e) => println!("  ! could not build the on-air frame for inspection: {e}"),
        }

        // ── Schedule: rounds of interleaved, randomised arms, plus the tail repeat ──────────
        let mut order: Vec<Vec<usize>> = Vec::with_capacity(rounds);
        for _ in 0..rounds {
            let mut o: Vec<usize> = (0..arms.len()).collect();
            rng.shuffle(&mut o);
            order.push(o);
        }
        println!(
            "\n  schedule: {rounds} rounds x {} arms, order randomised per round (seed above); \
             then {tail} tail reps of '{}' for the drift guard.\n",
            arms.len(),
            arms[0].name
        );

        let mut reps: Vec<Rep> = Vec::new();
        let mut contention_seen: Vec<Option<ContentionApplied>> = vec![None; arms.len()];
        let mut printed_apply = vec![false; arms.len()];

        println!(
            "  {:>5} {:>9}  {:>7} {:>6} {:>6}  {:>7} {:>7} {:>7} {:>7} {:>8}  {:>7} {:>6}  {}",
            "round",
            "arm",
            "ok",
            "t/out",
            "err",
            "p10 us",
            "p50 us",
            "p90 us",
            "p99 us",
            "max us",
            "off f/s",
            "act",
            "buckets"
        );

        for (r, ord) in order.iter().enumerate() {
            for &ai in ord {
                let (rep, applied, _entry, _exit) = run_rep(
                    &dev,
                    &io,
                    &arms[ai],
                    ai,
                    r,
                    &frame,
                    settle,
                    measure,
                    min_samples,
                    strict,
                    &mut contention_seen,
                )
                .await;
                if !printed_apply[ai] {
                    for n in &applied.notes {
                        println!("  [{}] applied: {n}", arms[ai].name);
                    }
                    printed_apply[ai] = true;
                }
                print_rep(&rep, &arms[ai].name, secs);
                reps.push(rep);
            }
        }
        // ★ Drift guard: the FIRST arm, repeated LAST.
        for t in 0..tail {
            let (rep, _a, _e, _x) = run_rep(
                &dev,
                &io,
                &arms[0],
                0,
                rounds + t,
                &frame,
                settle,
                measure,
                min_samples,
                strict,
                &mut contention_seen,
            )
            .await;
            print_rep(&rep, &format!("{}*tail", arms[0].name), secs);
            reps.push(rep);
        }

        // Leave the radio in the baseline arm rather than in whatever ran last.
        let _ = apply_arm(&dev, &io, &arms[0]);

        analyse(
            &dev, &arms, &reps, secs, air_us, theta, tail, force, &mut rng, &boot,
        );
        Ok::<(), Box<dyn std::error::Error>>(())
    })?;
    Ok(())
}

fn print_rep(rep: &Rep, arm: &str, secs: f64) {
    let b: Vec<String> = rep.buckets.iter().map(|x| x.to_string()).collect();
    println!(
        "  {:>5} {:>9}  {:>7} {:>6} {:>6}  {:>7.1} {:>7.1} {:>7.1} {:>7.1} {:>8.1}  {:>7.0} {:>6}  {}{}",
        rep.round,
        arm,
        rep.ok,
        rep.timeout,
        rep.other_err,
        rep.p10,
        rep.p50,
        rep.p90,
        rep.p99,
        rep.max,
        rep.offered_fps(secs),
        rep.activity_delta
            .map(|a| a.to_string())
            .unwrap_or_else(|| "-".into()),
        b.join(" "),
        if rep.valid {
            String::new()
        } else {
            format!("  <-- DROPPED: {}", rep.why_invalid.join("; "))
        }
    );
}

// ════════════════════════════════════════════════════════════════════════════════════════════
// Analysis
// ════════════════════════════════════════════════════════════════════════════════════════════

#[allow(clippy::too_many_arguments)]
fn analyse(
    dev: &Dev,
    arms: &[Arm],
    reps: &[Rep],
    secs: f64,
    air_us: f64,
    theta: f64,
    tail: usize,
    force: bool,
    rng: &mut Rng,
    boot: &[(&'static str, String)],
) {
    let valid: Vec<&Rep> = reps.iter().filter(|r| r.valid).collect();
    let dropped = reps.len() - valid.len();
    println!("\n──────── per-arm distribution (median per-call period, us; raw per-rep) ────────");
    for (i, a) in arms.iter().enumerate() {
        let p50s: Vec<f64> = valid.iter().filter(|r| r.arm == i).map(|r| r.p50).collect();
        let fps: Vec<f64> = valid
            .iter()
            .filter(|r| r.arm == i)
            .map(|r| r.offered_fps(secs))
            .collect();
        let cens: Vec<f64> = valid
            .iter()
            .filter(|r| r.arm == i)
            .map(|r| r.censor_pct())
            .collect();
        let idx: Vec<f64> = (0..p50s.len()).map(|k| k as f64).collect();
        let (rho, prho) = spearman(&idx, &p50s);
        println!(
            "  {:<10} n={:<3} p50 median {:.1} us (min {:.1}, max {:.1})  offered {:.0} f/s  \
             censor {:.2}%  trend rho={rho:+.2} (p={prho:.2})",
            a.name,
            p50s.len(),
            median(&p50s),
            p50s.iter().cloned().fold(f64::INFINITY, f64::min),
            p50s.iter().cloned().fold(f64::NEG_INFINITY, f64::max),
            median(&fps),
            median(&cens),
        );
        let raw: Vec<String> = p50s.iter().map(|x| format!("{x:.1}")).collect();
        println!("             per-rep p50: {}", raw.join(" "));
        // The truth counter, where the radio has one: tx_en = MAC->baseband transmit requests,
        // tx_on = baseband->RF keys. Compare against `ok`: a gap is frames the host offered and
        // the MAC never asked for. Absent on the 8812au, which is why nothing here says delivered.
        let txd: Vec<String> = valid
            .iter()
            .filter(|r| r.arm == i)
            .filter_map(|r| r.txc_delta.map(|(en, on)| format!("{en}/{on}")))
            .collect();
        if !txd.is_empty() {
            println!(
                "             per-rep tx_en/tx_on (DELIVERED, from read_tx_counters): {}",
                txd.join(" ")
            );
        }
        if rho.abs() > 0.5 && prho < 0.05 {
            println!(
                "             ★ VOID-GRADE TREND: this arm walked monotonically during the run. \
                 The six MEASURED runs that started this investigation score rho=-0.60."
            );
        }
    }
    if dropped > 0 {
        println!(
            "\n  {dropped} rep(s) DROPPED (see the DROPPED annotations above). Dropped reps \
                  are excluded, never averaged in."
        );
    }

    // ── Gate G1: the airtime ceiling ────────────────────────────────────────────────────────
    // Medium access for the ceiling: the baseline arm's own DCF budget when it sets a posture
    // (SIFS + AIFSN*slot + E[backoff], slot 9), else the stock `Shared` budget as the reference.
    let ma_us = match contention_of(arms, 0) {
        Some((cw_min, aifsn)) => {
            16.0 + (aifsn as f64) * 9.0 + ((1u32 << cw_min) - 1) as f64 * 9.0 / 2.0
        }
        None => 110.5,
    };
    let ceiling = 1e6 / (air_us + ma_us);
    let best = valid
        .iter()
        .map(|r| r.offered_fps(secs))
        .fold(0.0f64, f64::max);
    println!(
        "\n  GATE G1 ceiling: 1e6/({air_us:.0} us air + {ma_us:.0} us medium access) = {ceiling:.0} \
         f/s. Fastest arm offered {best:.0} f/s."
    );
    if best > ceiling {
        println!(
            "  ★ G1 FAILED: the offered rate EXCEEDS what the medium can carry, so those calls are \
             not frames on air (a full bulk-OUT queue absorbs writes; a halted endpoint returns \
             instantly). Every number below is a host-side call rate. Do not report it as \
             throughput."
        );
    }
    if dev.tx_counters().is_none() {
        println!(
            "  NOTE: this radio implements no read_tx_counters(), so nothing here distinguishes \
             'the MAC transmitted' from 'the host handed USB a buffer'. All rates are OFFERED."
        );
    }

    // ── Post-run vs post-bring-up: did anything autonomous move? ────────────────────────────
    let after = dev.readback();
    let mut moved = Vec::new();
    for ((k, v0), (_, v1)) in boot.iter().zip(after.iter()) {
        if v0 != v1 && !k.starts_with("phy_sense") && !k.contains("activity") {
            moved.push(format!("{k}: {v0} -> {v1}"));
        }
    }
    if moved.is_empty() {
        println!("  post-run state vector matches post-bring-up (nothing autonomous moved).");
    } else {
        println!("  ★ STATE MOVED between bring-up and end of run:");
        for m in &moved {
            println!("      {m}");
        }
    }

    // ── Drift guard: first arm, repeated last ───────────────────────────────────────────────
    let a0_all: Vec<f64> = valid.iter().filter(|r| r.arm == 0).map(|r| r.p50).collect();
    if a0_all.len() >= 2 * tail.max(2) {
        let k = tail.max(3).min(a0_all.len() / 2);
        let head = &a0_all[..k];
        let tailv = &a0_all[a0_all.len() - k..];
        let (lo, hi) = boot_ci_diff_median(head, tailv, rng, 10_000);
        let shift = median(tailv) - median(head);
        let void = lo > 0.0 || hi < 0.0;
        println!(
            "\n  DRIFT GUARD (arm '{}' first {k} reps vs last {k}): {:+.1} us  95% CI [{lo:+.1}, \
             {hi:+.1}] us  => {}",
            arms[0].name,
            shift,
            if void {
                "★ VOID — the instrument changed underneath the comparison; no arm difference \
                 from this run is interpretable"
            } else {
                "OK (the CI contains 0)"
            }
        );
        if void && !force {
            println!(
                "  Nothing below may be quoted. Re-run after finding what drifted (start with the \
                 per-rep p50 series and the state-moved list above)."
            );
        }
    }

    // ── Pairwise, paired by round, against arm 0 ────────────────────────────────────────────
    for i in 1..arms.len() {
        let sham = arms[i].key == arms[0].key;
        println!(
            "\n──────── {} vs {} ({}) ────────",
            arms[i].name,
            arms[0].name,
            if sham {
                "SHAM — expected null"
            } else {
                "TREATMENT"
            }
        );
        // Pair strictly within a round; a round missing either arm is dropped whole.
        let mut d_us = Vec::new();
        let mut r_ln = Vec::new();
        let mut rounds_used = 0usize;
        let max_round = reps.iter().map(|r| r.round).max().unwrap_or(0);
        for rd in 0..=max_round {
            let a = valid.iter().find(|r| r.round == rd && r.arm == 0);
            let b = valid.iter().find(|r| r.round == rd && r.arm == i);
            if let (Some(a), Some(b)) = (a, b) {
                d_us.push(b.p50 - a.p50);
                r_ln.push((a.p50 / b.p50).ln());
                rounds_used += 1;
                // Censoring balance: arms that censor differently were not measuring the same thing.
                if (a.censor_pct() - b.censor_pct()).abs() > 2.0 {
                    println!(
                        "  ! round {rd}: censoring differs by {:.1} points ({:.1}% vs {:.1}%) — \
                         VOID as a knob result, report it as a mechanism finding.",
                        (a.censor_pct() - b.censor_pct()).abs(),
                        a.censor_pct(),
                        b.censor_pct()
                    );
                }
            }
        }
        if rounds_used < 3 {
            println!("  only {rounds_used} usable pairs — nothing can be concluded.");
            continue;
        }
        let hl_us = hodges_lehmann(&d_us);
        let (lo_us, hi_us, stat) = bootstrap_ci(&d_us, rng, 10_000);
        let hl_r = hodges_lehmann(&r_ln);
        let (lo_r, hi_r, _) = bootstrap_ci(&r_ln, rng, 10_000);
        let (pos, neg, p_sign) = sign_test(&d_us);
        let p_wil = wilcoxon_p(&d_us);
        let pct = |x: f64| (x.exp() - 1.0) * 100.0;

        println!("  n = {rounds_used} pairs (paired within round; bootstrap statistic = {stat})");
        println!("  paired differences (us, arm - baseline, negative = arm is FASTER):");
        let raw: Vec<String> = d_us.iter().map(|x| format!("{x:+.1}")).collect();
        println!("    {}", raw.join(" "));
        println!("  Hodges-Lehmann {hl_us:+.1} us   95% CI [{lo_us:+.1}, {hi_us:+.1}] us");
        println!(
            "  as a rate       {:+.2}%   95% CI [{:+.2}%, {:+.2}%]   (positive = arm is faster)",
            pct(hl_r),
            pct(lo_r),
            pct(hi_r)
        );
        println!(
            "  sign test {pos}+/{neg}- p={p_sign:.3} (exact) | Wilcoxon p={p_wil:.3} (normal approx){}",
            if (p_sign < 0.05) != (p_wil < 0.05) {
                "  ← THE TESTS DISAGREE; the sign test is authoritative"
            } else {
                ""
            }
        );

        // Sigma, MDE, and the sample size this radio actually needs tonight.
        let sigma = sd(&r_ln);
        let n = rounds_used as f64;
        let mde = ((2.802 * sigma / n.sqrt()).exp() - 1.0) * 100.0;
        let need = (2.802 * sigma / (1.0 + theta).ln()).powi(2).ceil();
        println!(
            "  sigma_d (paired log-ratio) = {sigma:.3}  =>  this run resolves +-{mde:.1}%; \
             resolving +-{:.0}% needs {need:.0} pairs",
            theta * 100.0
        );

        // The available effect, when the arms differ only in contention: a knob that cannot move
        // the period by more than X% cannot legitimately measure as more than X%.
        if let (Some(c0), Some(c1)) = (contention_of(arms, 0), contention_of(arms, i)) {
            let ma0 = 16.0 + (c0.1 as f64) * 9.0 + ((1u32 << c0.0) - 1) as f64 * 9.0 / 2.0;
            let ma1 = 16.0 + (c1.1 as f64) * 9.0 + ((1u32 << c1.0) - 1) as f64 * 9.0 / 2.0;
            let avail = (ma0 - ma1).abs() / (air_us + ma0) * 100.0;
            println!(
                "  MAX AVAILABLE EFFECT (DCF budget at slot 9): |{ma0:.0} - {ma1:.0}| us of a \
                 {:.0} us period = {avail:.2}%",
                air_us + ma0
            );
            if mde > avail && !force {
                println!(
                    "  ★ THIS COMPARISON CANNOT RESOLVE ITS OWN EFFECT: MDE {mde:.1}% > available \
                     {avail:.2}%. Shorten the frame or raise the rate until medium access is a \
                     large fraction of the period, or test the ABSOLUTE microsecond prediction \
                     ({:+.0} us) instead of a percentage.",
                    ma1 - ma0
                );
            }
            if hl_us.abs() > (ma0 - ma1).abs() * 3.0 {
                println!(
                    "  ★ The measured shift is >3x what the mechanism can produce. An effect much \
                     LARGER than the budget is as much a red flag as one much smaller."
                );
            }
        }

        // ── The verdict ─────────────────────────────────────────────────────────────────────
        let (lo_pct, hi_pct) = (pct(lo_r), pct(hi_r));
        let t = theta * 100.0;
        let excludes_zero = lo_pct > 0.0 || hi_pct < 0.0;
        let inside_theta = lo_pct > -t && hi_pct < t;
        if excludes_zero && pct(hl_r).abs() >= t {
            println!(
                "  VERDICT: DIFFERENT. 95% CI [{lo_pct:+.1}%, {hi_pct:+.1}%] excludes 0 and |HL| \
                 = {:.1}% >= theta {t:.0}%.{}",
                pct(hl_r).abs(),
                if sham {
                    "  ★★ THIS IS A SHAM PAIR. A significant difference between two identical \
                     configurations means the PIPELINE is broken, not the knob. Stop and fix it; \
                     every other comparison in this run is void."
                } else {
                    ""
                }
            );
        } else if inside_theta {
            println!(
                "  VERDICT: INDISTINGUISHABLE at theta={t:.0}%. The 95% CI [{lo_pct:+.1}%, \
                 {hi_pct:+.1}%] lies wholly inside +-{t:.0}%. This run resolved effects down to \
                 +-{mde:.1}%; no effect of that size exists here."
            );
        } else {
            println!(
                "  VERDICT: INCONCLUSIVE. The 95% CI [{lo_pct:+.1}%, {hi_pct:+.1}%] both contains 0 \
                 and extends past +-{t:.0}%: this run neither found nor excluded a {t:.0}% effect; \
                 it resolved only +-{mde:.1}%. DO NOT QUOTE THE POINT ESTIMATE ({:+.1}%) — it is \
                 not a measurement. To resolve {t:.0}% at the observed sigma_d = {sigma:.3}: \
                 {need:.0} pairs.",
                pct(hl_r)
            );
        }
        if sham && !excludes_zero {
            println!(
                "  (sham noise floor: sd of the paired log-ratio {sigma:.3}. A treatment effect \
                 must exceed 1.96*{sigma:.3}/sqrt(n) = {:.1}% to be believable at this n.)",
                ((1.96 * sigma / n.sqrt()).exp() - 1.0) * 100.0
            );
        }
    }
    println!(
        "\n  Every difference above is printed with its CI on the same line. A number without an \
         interval is not permitted to leave this harness."
    );
}

/// `(cw_min exponent, aifsn)` for an arm's contention posture, from the driver's own posture map.
fn contention_of(arms: &[Arm], i: usize) -> Option<(u8, u8)> {
    let p = match arms[i].spec.get("contention").map(String::as_str)? {
        "owned" => ContentionPosture::Owned,
        "shared" => ContentionPosture::Shared,
        "yielding" => ContentionPosture::Yielding,
        _ => return None,
    };
    let (cw_min, _cw_max, aifsn) = realtek_contention::rtl_posture(p);
    Some((cw_min.max(realtek_contention::MIN_CW_EXPONENT), aifsn))
}
