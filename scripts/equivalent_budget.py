#!/usr/bin/env python3
"""
Equivalent-computational-budget analysis across the full trial set
(PROJECT_PLAN.md, Phase 2C -- extends the Phase 0 single-trial analysis in
analyze_equivalent_budget.cpp to every adaptive trial).

For each adaptive trial: take its mean N (whole-trial and in-load-window),
interpolate the expected tracking quality and per-call compute cost from the
Phase 0 Pareto table (test_n_quality_sweep, 10 seeds/N, standalone sim), and
compare the predicted quality gap vs N=400 against the table's own noise
floor. Then compare the *prediction* (gap smaller than noise floor) against
the *observed* flight results (adaptive vs fixed-400 steady-state RMS).

Usage:
    python3 equivalent_budget.py results/            # 20Hz primary set
    python3 equivalent_budget.py results_40hz/       # 40Hz secondary set
"""
import argparse
import csv
import glob
import math
import os
import sys

# Phase 0 Pareto table (test_n_quality_sweep, 10 seeds per N, standalone
# kinematic sim; RMS in that sim's units, cost = mean ms per MPPI call).
# Must match src/analyze_equivalent_budget.cpp.
PARETO = [
    # N,   mean_rms, std_rms, cost_ms
    (20,  0.9115, 0.0405, 1.1387),
    (50,  0.8719, 0.0280, 2.7977),
    (100, 0.8393, 0.0491, 5.5407),
    (150, 0.8187, 0.0215, 8.4159),
    (200, 0.7906, 0.0195, 11.6946),
    (250, 0.8075, 0.0295, 14.4366),
    (300, 0.7980, 0.0291, 17.2648),
    (350, 0.7872, 0.0269, 19.6126),
    (400, 0.7893, 0.0301, 22.0119),
]


def interp(n, col):
    """Linear interpolation of a Pareto column (1=rms, 2=std, 3=cost)."""
    if n <= PARETO[0][0]:
        return PARETO[0][col]
    if n >= PARETO[-1][0]:
        return PARETO[-1][col]
    for lo, hi in zip(PARETO, PARETO[1:]):
        if lo[0] <= n <= hi[0]:
            t = (n - lo[0]) / (hi[0] - lo[0])
            return lo[col] + t * (hi[col] - lo[col])
    return PARETO[-1][col]


def tracking_rows(path):
    with open(path) as f:
        return [r for r in csv.DictReader(f) if r["phase"] == "TRACKING"]


def mean(v):
    return sum(v) / len(v) if v else float("nan")


def trial_mean_n(rows, window=None):
    t0 = float(rows[0]["epoch_sec"])
    ns = [int(r["N"]) for r in rows
          if window is None
          or window[0] <= float(r["epoch_sec"]) - t0 < window[1]]
    return mean(ns)


def condition_trials(base, cond):
    return sorted(glob.glob(os.path.join(base, cond, "trial_*.csv")))


def ss_rms(rows, thresh=0.5, consec=10):
    errs = [float(r["pos_error_m"]) for r in rows]
    run = 0
    for i, e in enumerate(errs):
        run = run + 1 if e < thresh else 0
        if run == consec:
            tail = errs[i - consec + 1:]
            return math.sqrt(sum(x * x for x in tail) / len(tail))
    return float("nan")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("results_dir")
    parser.add_argument("--load-window", nargs=2, type=float,
                        default=[20.0, 50.0], metavar=("T0", "T1"))
    args = parser.parse_args()

    adaptive = condition_trials(args.results_dir, "adaptive")
    fixed = condition_trials(args.results_dir, "fixed400")
    if not adaptive:
        sys.exit(f"No adaptive trials under {args.results_dir}")

    rms400, std400, cost400 = (interp(400, 1), interp(400, 2), interp(400, 3))

    print(f"Pareto reference at N=400: RMS {rms400:.4f} +/- {std400:.4f}, "
          f"cost {cost400:.2f} ms/call (standalone-sim units)\n")
    print(f"{'trial':<14}{'meanN':>7}{'N(load)':>9}{'estRMS':>8}"
          f"{'gap':>8}{'gap/std':>9}{'cost%':>7}")

    gaps, savings, mean_ns = [], [], []
    for path in adaptive:
        rows = tracking_rows(path)
        n_all = trial_mean_n(rows)
        n_load = trial_mean_n(rows, window=tuple(args.load_window))
        est = interp(n_all, 1)
        gap = est - rms400
        noise = max(interp(n_all, 2), std400)
        saving = 100.0 * (1.0 - interp(n_all, 3) / cost400)
        gaps.append(gap)
        savings.append(saving)
        mean_ns.append(n_all)
        print(f"{os.path.basename(path):<14}{n_all:>7.1f}{n_load:>9.1f}"
              f"{est:>8.4f}{gap:>+8.4f}{gap/noise:>9.2f}{saving:>7.1f}")

    print(f"\nAcross {len(adaptive)} adaptive trials:")
    print(f"  mean N: {mean(mean_ns):.1f}")
    print(f"  predicted quality gap vs N=400: {mean(gaps):+.4f} "
          f"(table noise floor ~{std400:.4f}) -> "
          f"{'WITHIN' if abs(mean(gaps)) < std400 else 'EXCEEDS'} noise floor")
    print(f"  predicted per-call compute saving vs N=400: "
          f"{mean(savings):.1f}%")

    if fixed:
        a_rms = [ss_rms(tracking_rows(p)) for p in adaptive]
        f_rms = [ss_rms(tracking_rows(p)) for p in fixed]
        a_rms = [x for x in a_rms if not math.isnan(x)]
        f_rms = [x for x in f_rms if not math.isnan(x)]
        print(f"\nPrediction vs observation (flight, steady-state RMS):")
        print(f"  predicted: gap within the Pareto table's noise floor")
        print(f"  observed:  adaptive {mean(a_rms):.3f} m vs fixed-400 "
              f"{mean(f_rms):.3f} m across {len(a_rms)}/{len(f_rms)} trials")
        print(f"  (statistical test of this comparison: see "
              f"analyze_trials.py output)")


if __name__ == "__main__":
    main()
