#!/usr/bin/env python3
"""**On-air acceptance for the named airtime lease** — folds the RECEIVER's chip stamps and
scores C1/C2/C3/P1/N1/N2 from `examples/halow_lease.rs rx --csv`.

WHY A SEPARATE TOOL. The sender can only ever report where it *aimed*. The claim under test is
about the air, so the arrival instant has to come from an instrument the sender does not touch:
the receiving chip's own radiotap TSFT (latch `MacDone`, ~1 us), not either host's
CLOCK_REALTIME (the two NTP-sync to different pool servers and are MEASURED ~6.9 ms apart —
7x the guard).

  ☠ THE CIRCULARITY THIS TOOL IS BUILT TO AVOID. The obvious move is to regress the chip stamp
  on the sender's INTENDED target times and fold on the residual. That fit's intercept absorbs
  the schedule, so the run then "confirms" the phase it was handed and every arm scores
  perfectly. The map used for every phase number below is therefore fitted on
  `tsft` vs the RECEIVER's OWN wall clock — no schedule input at all — and moved onto the
  sender's clock by an offset measured over the WIRED LAN (`tools/clock_offset.py`),
  independently of the radio run. The schedule-referenced fit is still computed, but only to
  report path latency and to cross-check the rate; it never sets a phase.

  The two decisive numbers are offset-free by construction: R (a modulus of the phase
  distribution) and the A->B phase SHIFT (a difference, so any constant cancels). Neither can
  be manufactured by a mis-measured clock offset.

USAGE
  lease_accept.py --csv rx.csv [--csv more.csv ...] --rx-minus-tx-us N
                  [--fold-mult F] [--shift A_RUN:B_RUN ...] [--expect-slots K]

  --rx-minus-tx-us   clock(receiver) - clock(sender), microseconds, from tools/clock_offset.py.
                     Affects ONLY the absolute in-window column; never R, never a shift.
  --fold-mult F      fold at F x the superframe instead of 1.0. F=1.37 is the N2 control:
                     concentration that survives an incommensurate fold is an artefact of the
                     instrument, not of the lease.
  --shift A:B        report the circular-mean phase shift from group A to group B and the
                     slot-width prediction it has to match (C2).

Groups are (run id, name, arm) — one sender's one run of one name, so two runs can never be
folded together.
"""
import argparse
import csv
import math
import sys
from collections import OrderedDict

M64 = (1 << 64) - 1


def prefix_hash_slash(name: str) -> int:
    """A THIRD independent implementation of the canonical `prefix_hash` (FNV-1a over
    components, 0x2f separator). The Rust sender carries its hash in the frame and the Rust
    receiver recomputes it; this recomputes it again in another language. If all three agree,
    the keyspace is not an artefact of one compiler's arithmetic."""
    h = 0xCBF29CE484222325
    for comp in [c for c in name.split("/") if c]:
        for b in comp.encode():
            h ^= b
            h = (h * 0x100000001B3) & M64
        h ^= 0x2F
        h = (h * 0x100000001B3) & M64
    return h


def lsq(xs, ys):
    """Ordinary least squares y = a*x + b; returns (a, b, residuals)."""
    n = len(xs)
    mx = sum(xs) / n
    my = sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    a = sxy / sxx if sxx else float("nan")
    b = my - a * mx
    return a, b, [y - (a * x + b) for x, y in zip(xs, ys)]


def qstats(v):
    if not v:
        return "n/a"
    s = sorted(v)
    q = lambda p: s[min(len(s) - 1, int(p * len(s)))]
    mean = sum(s) / len(s)
    sd = math.sqrt(sum((x - mean) ** 2 for x in s) / len(s)) if len(s) > 1 else 0.0
    return (f"n={len(s)} p50={q(.5):.0f} p90={q(.9):.0f} p99={q(.99):.0f} "
            f"min={s[0]:.0f} max={s[-1]:.0f} sd={sd:.0f}")


def p99(v):
    s = sorted(v)
    return s[min(len(s) - 1, int(0.99 * len(s)))] if s else float("nan")


def circular(phases, period):
    """Rayleigh R, circular mean phase (in the same units as `period`), circular sd, and the
    standard error of the mean phase.

    R is the resultant length of the unit vectors: 1.0 = every frame at one phase, 0.0 = phase
    uniform on the circle. It is the concentration statistic C1 and N2 are stated in, and it is
    invariant to any constant added to every phase — which is exactly why a mis-measured clock
    offset cannot inflate it."""
    n = len(phases)
    c = sum(math.cos(2 * math.pi * p / period) for p in phases) / n
    s = sum(math.sin(2 * math.pi * p / period) for p in phases) / n
    R = math.hypot(c, s)
    mean = (math.atan2(s, c) / (2 * math.pi)) * period % period
    # circular sd in radians -> the same units as period
    csd = math.sqrt(-2 * math.log(R)) if 0 < R < 1 else 0.0
    csd_u = csd / (2 * math.pi) * period
    return R, mean, csd_u, csd_u / math.sqrt(n)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", action="append", required=True)
    ap.add_argument("--rx-minus-tx-us", type=float, default=0.0)
    ap.add_argument("--fold-mult", type=float, default=1.0)
    ap.add_argument("--shift", action="append", default=[])
    ap.add_argument("--latency-us", type=float, default=None,
                    help="override the measured constant path latency removed for the "
                         "transmitter-confinement column")
    args = ap.parse_args()

    rows = []
    for path in args.csv:
        with open(path) as f:
            for r in csv.DictReader(f):
                if not r.get("tsft"):
                    continue
                for k in ("run", "seq", "hash", "slot", "slots", "slot_us", "guard_us",
                          "target_us", "presend_us", "rx_wall_us", "tsft"):
                    r[k] = int(r[k])
                r["src_csv"] = path
                rows.append(r)
    if not rows:
        print("no stamped frames in any csv — nothing to score.")
        print("  If a transmitter WAS running this is a failure of the RX chip stamp;")
        print("  if none was, this is N1 (the negative control) and the correct result.")
        return 0
    rows.sort(key=lambda r: r["tsft"])

    # ── the map: chip stamp -> the SENDER's wall clock, with no schedule input ───────────────
    a_h, b_h, res_h = lsq([float(r["tsft"]) for r in rows], [float(r["rx_wall_us"]) for r in rows])
    third = len(res_h) // 3
    drift = (sum(res_h[-third:]) / max(third, 1)) - (sum(res_h[:third]) / max(third, 1))

    print("=" * 78)
    print("RX CHIP CLOCK  (radiotap TSFT on the receiving MM6108, latch MacDone)")
    print("=" * 78)
    print(f"  stamped frames    : {len(rows)}   span {(rows[-1]['tsft']-rows[0]['tsft'])/1e6:.1f} s")
    print(f"  host us per tick  : {a_h:.9f}   ({(a_h-1)*1e6:+.3f} ppm vs the RECEIVER's CLOCK_REALTIME)")
    print(f"  fit residual      : {qstats([abs(x) for x in res_h])}  us")
    print(f"  first/last third  : {drift:+.0f} us of residual drift (NTP slew during the capture)")
    print(f"  rx - tx clock     : {args.rx_minus_tx_us:+.0f} us  (WIRED measurement, independent of this run)")
    print("    ^ this fit uses only the chip stamp and the receiver's own wall clock. The")
    print("      sender's schedule is not an input to it, so it cannot encode the answer.")

    # tsft -> sender's wall clock
    def tx_wall(r):
        return a_h * r["tsft"] + b_h - args.rx_minus_tx_us

    # schedule-referenced fit — for path latency and a rate cross-check ONLY
    lease_rows = [r for r in rows if r["arm"] == "L"]
    if lease_rows:
        a_s, b_s, res_s = lsq([float(r["target_us"]) for r in lease_rows],
                              [float(r["tsft"]) for r in lease_rows])
        print(f"  cross-check rate  : {1/a_s:.9f} host us per tick from the SCHEDULE-referenced fit")
        print(f"                      ({((1/a_s)-a_h)*1e6:+.3f} ppm from the schedule-free one)")
    lat = [tx_wall(r) - r["target_us"] for r in lease_rows]
    latency = args.latency_us if args.latency_us is not None else (
        sorted(lat)[len(lat) // 2] if lat else 0.0)
    floor_lat = sorted(lat)[max(0, len(lat) // 100)] if lat else 0.0
    if lat:
        print(f"  path latency      : {qstats(lat)}  us   (sender's INTENDED instant -> chip stamp:")
        print("                      driver + SPI + MAC + airtime + the RX latch. Its MEDIAN is a")
        print("                      constant, not a defect; its SPREAD is what the guard must cover.)")

    # ── per group ───────────────────────────────────────────────────────────────────────────
    groups = OrderedDict()
    for r in rows:
        groups.setdefault((r["run"], r["name"], r["arm"]), []).append(r)

    summary = {}
    for (run, name, arm), rs in groups.items():
        g = rs[0]
        slots, slot_us, guard_us = g["slots"], g["slot_us"], g["guard_us"]
        cycle = slots * slot_us
        fold = cycle * args.fold_mult
        my_hash = prefix_hash_slash(name)
        own = my_hash % slots
        agree = (my_hash == g["hash"]) and (own == g["slot"])

        ph = [tx_wall(r) % fold for r in rs]
        R, mean, csd, sem = circular(ph, fold)
        # absolute occupancy, on the honest map
        hist = [0] * slots
        inwin = 0
        for p in (x % cycle for x in ph):
            s = int(p // slot_us)
            hist[s] += 1
            if s == own and (p - s * slot_us) < (slot_us - guard_us):
                inwin += 1
        # Transmitter confinement: the same test with the CONSTANT part of the path removed.
        #
        # The constant is estimated as the FASTEST path (1st percentile of target->stamp), not the
        # median. A frame cannot be transmitted before its target — the sender sleeps to it — so the
        # true fixed delay is the floor of the distribution, and everything above it is queueing and
        # jitter that the guard genuinely has to cover. Subtracting the MEDIAN instead would put half
        # the frames "before" their own target and drop the score to ~50% for a perfectly confined
        # sender: the wrong polarity, from the wrong statistic.
        hist_c = [0] * slots
        inwin_c = 0
        for p in ((tx_wall(r) - floor_lat) % cycle for r in rs):
            s = int(p // slot_us)
            hist_c[s] += 1
            if s == own and (p - s * slot_us) < (slot_us - guard_us):
                inwin_c += 1
        seqs = [r["seq"] for r in rs]
        span = max(seqs) - min(seqs) + 1
        rssi = [int(r["rssi"]) for r in rs if r["rssi"]]
        place = [tx_wall(r) - floor_lat - r["target_us"] for r in rs]

        summary[run] = dict(name=name, arm=arm, own=own, mean=mean, R=R, sem=sem,
                            slot_us=slot_us, slots=slots, cycle=cycle, n=len(rs),
                            p99=p99([abs(x) for x in place]))

        print()
        print("-" * 78)
        print(f"run 0x{run:04x}  {name}  [{ 'LEASE' if arm=='L' else 'FREE (control)' }]")
        print("-" * 78)
        print(f"  frames heard      : {len(rs)} of {span} offered  ({100*len(rs)/span:.1f}% delivered)")
        print(f"  hash (3rd impl)   : 0x{my_hash:016x}  {'IDENTICAL' if agree else '*** DIFFERENT ***'}"
              f"  -> slot {own} of {slots}")
        print(f"  grid              : {slots} x {slot_us} us, {guard_us} us guard, cycle {cycle} us")
        if args.fold_mult != 1.0:
            print(f"  ** folded at      : {fold:.0f} us = {args.fold_mult}x the cycle  (N2 control)")
        print(f"  R (concentration) : {R:.4f}   circular mean phase {mean:.0f} us  (+/- {sem:.0f} sem)")
        print(f"  slot histogram    : " + " ".join(f"[{c}]" if i == own else str(c)
                                                   for i, c in enumerate(hist)))
        print(f"  in-window ABS     : {inwin}/{len(rs)} = {100*inwin/len(rs):.2f}%"
              f"   (arrival, on the wired-measured clock offset)")
        print(f"  in-window CONFINED: {inwin_c}/{len(rs)} = {100*inwin_c/len(rs):.2f}%"
              f"   (fastest path {floor_lat:.0f} us removed: tests SPREAD)")
        print(f"  placement error   : {qstats([abs(x) for x in place])}  us")
        if rssi:
            print(f"  rssi dbm          : {qstats(rssi)}")

    # ── C2 ──────────────────────────────────────────────────────────────────────────────────
    for spec in args.shift:
        ra, rb = (int(x, 0) for x in spec.split(":"))
        A, B = summary[ra], summary[rb]
        d_slots = B["own"] - A["own"]
        pred = d_slots * A["slot_us"]
        got = B["mean"] - A["mean"]
        tol = max(A["p99"], B["p99"])
        err = got - pred
        print()
        print("=" * 78)
        print(f"C2  TWO-POINT PHASE CALIBRATION   {A['name']} (slot {A['own']})"
              f"  ->  {B['name']} (slot {B['own']})")
        print("=" * 78)
        print(f"  predicted shift   : {pred:+.0f} us  = {d_slots:+d} slot(s) x {A['slot_us']} us")
        print(f"  MEASURED shift    : {got:+.0f} us   (circular means {A['mean']:.0f} -> {B['mean']:.0f})")
        print(f"  error             : {err:+.0f} us   tolerance +/-{tol:.0f} us (the larger p99 placement error)")
        print(f"  sem of the shift  : +/-{math.hypot(A['sem'], B['sem']):.0f} us")
        print(f"  VERDICT           : {'PASS' if abs(err) <= tol else '*** FAIL ***'}"
              "  — a signed, quantitative prediction: the phase moved where the NAME said it")
        print("                      would. A merely periodic transmitter cannot do this.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
