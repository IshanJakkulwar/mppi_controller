#!/usr/bin/env python3
"""
Phase 2 trial-set analysis (see PROJECT_PLAN.md, Phase 2C).

Takes any number of flight CSVs and/or directories (searched recursively),
groups trials by the CSV `mode` column (ADAPTIVE / CONST330 / FIXED400 --
legacy "FIXED" logs are treated as FIXED400), and reports:

Per trial:
  - whole-trajectory RMS error and steady-state-only RMS error
  - steady-state onset (algorithmic: first run of 10 consecutive TRACKING
    cycles with pos_error < 0.5 m -- no hardcoded time cutoff)
  - deadline misses (count + rate), mean N, mean call time, mean latency
  - scheduler chatter: mean |dN| and variance of N

Aggregate, per condition:
  - mean +/- std of every per-trial metric
  - Welch's t-test and Mann-Whitney U: ADAPTIVE vs FIXED400 and ADAPTIVE vs
    CONST330, on deadline misses and steady-state RMS
  - scheduler-validation metrics for ADAPTIVE trials (spike response latency,
    recovery time) -- heuristic event extraction, see extract_spike_events()

Figures (written to <output-dir>):
  - overlay_timeseries.png : N / call time / error, all trials by condition
  - dn_distribution.png    : dN histogram per condition (chatter check)
  - boxplots.png           : misses + steady-state RMS per condition

Usage:
    python3 analyze_trials.py results/
    python3 analyze_trials.py results/adaptive results/fixed400 --output-dir results/analysis
"""
import argparse
import csv
import math
import os
import sys
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

try:
    from scipy import stats as scipy_stats
except ImportError:
    scipy_stats = None

# Algorithmic steady-state definition (PROJECT_PLAN.md, Transient vs.
# Steady-State Reporting): steady state begins at the first control cycle of
# the first run of SS_CONSECUTIVE consecutive TRACKING cycles with
# pos_error < SS_THRESHOLD_M.
SS_THRESHOLD_M = 0.5
SS_CONSECUTIVE = 10

DEADLINE_MS = 50.0

CONDITION_ORDER = ["ADAPTIVE", "CONST200", "CONST330", "FIXED400"]
CONDITION_COLORS = {"ADAPTIVE": "tab:blue", "CONST200": "tab:red",
                    "CONST330": "tab:orange", "FIXED400": "tab:green"}


def collect_csv_files(paths):
    files = []
    for p in paths:
        if os.path.isdir(p):
            for root, _, names in os.walk(p):
                files += [os.path.join(root, n) for n in sorted(names)
                          if n.endswith(".csv")]
        elif p.endswith(".csv"):
            files.append(p)
    return files


def load_tracking_rows(path):
    with open(path) as f:
        rows = [r for r in csv.DictReader(f) if r["phase"] == "TRACKING"]
    return rows


def condition_of(rows):
    mode = rows[0]["mode"]
    return "FIXED400" if mode == "FIXED" else mode  # legacy label


def steady_state_start(errors):
    """Index where steady state begins, or None if never reached."""
    run = 0
    for i, e in enumerate(errors):
        run = run + 1 if e < SS_THRESHOLD_M else 0
        if run == SS_CONSECUTIVE:
            return i - SS_CONSECUTIVE + 1
    return None


def rms(values):
    return math.sqrt(sum(v * v for v in values) / len(values)) if values else float("nan")


def mean(values):
    return sum(values) / len(values) if values else float("nan")


def variance(values):
    if len(values) < 2:
        return 0.0
    m = mean(values)
    return sum((v - m) ** 2 for v in values) / (len(values) - 1)


def rolling_median(values, i, window):
    lo = max(0, i - window)
    chunk = sorted(values[lo:i])
    return chunk[len(chunk) // 2] if chunk else float("nan")


def extract_spike_events(n_values, call_ms):
    """Heuristic scheduler-validation event extraction (ADAPTIVE trials).

    A spike onset is a cycle where time-per-sample rises >50% above the
    rolling median of the previous 20 cycles. For each episode we report:
      response_cycles: cycles from onset until N drops to <=90% of its
                       onset value (the scheduler reacting)
      recovery_cycles: cycles from the end of the spike (time-per-sample
                       back within 20% of baseline for 5 consecutive cycles)
                       until N recovers to >=95% of its pre-spike value
    Heuristic, intended for the controlled load window -- inspect the
    overlay plots alongside these numbers.
    """
    tps = [c / n if n > 0 else 0.0 for n, c in zip(n_values, call_ms)]
    events = []
    i, count = 1, len(tps)
    while i < count:
        base = rolling_median(tps, i, 20)
        if not (base > 0) or tps[i] <= 1.5 * base:
            i += 1
            continue
        onset, n_pre = i, n_values[i - 1]
        response = next(
            (j - onset for j in range(onset, min(onset + 20, count))
             if n_values[j] <= 0.9 * n_values[onset]), None)

        # find spike end: tps back near baseline for 5 consecutive cycles
        end, run = None, 0
        for j in range(onset + 1, count):
            run = run + 1 if tps[j] < 1.2 * base else 0
            if run == 5:
                end = j - 4
                break
        recovery = None
        if end is not None:
            recovery = next(
                (j - end for j in range(end, min(end + 100, count))
                 if n_values[j] >= 0.95 * n_pre), None)
        events.append({"onset": onset, "response_cycles": response,
                       "recovery_cycles": recovery})
        i = (end if end is not None else onset + 1) + 1
    return events


def analyze_trial(path):
    rows = load_tracking_rows(path)
    if not rows:
        print(f"WARNING: no TRACKING rows in {path}, skipping", file=sys.stderr)
        return None

    t0 = float(rows[0]["epoch_sec"])
    t = [float(r["epoch_sec"]) - t0 for r in rows]
    errors = [float(r["pos_error_m"]) for r in rows]
    n_values = [int(r["N"]) for r in rows]
    call_ms = [float(r["mppi_call_ms"]) for r in rows]
    latency = [float(r.get("state_to_command_latency_ms",
                           r.get("mavros_roundtrip_ms", 0))) for r in rows]
    misses = sum(int(r["deadline_miss"]) for r in rows)

    ss_idx = steady_state_start(errors)
    dn = [n_values[i] - n_values[i - 1] for i in range(1, len(n_values))]

    return {
        "path": path,
        "condition": condition_of(rows),
        "t": t, "errors": errors, "n_values": n_values, "call_ms": call_ms,
        "samples": len(rows),
        "duration_s": t[-1],
        "rms_whole": rms(errors),
        "ss_start_idx": ss_idx,
        "ss_start_t": t[ss_idx] if ss_idx is not None else None,
        "rms_ss": rms(errors[ss_idx:]) if ss_idx is not None else float("nan"),
        "misses": misses,
        "miss_rate": misses / len(rows),
        "mean_n": mean(n_values),
        "mean_call_ms": mean(call_ms),
        "mean_latency_ms": mean(latency),
        "mean_abs_dn": mean([abs(d) for d in dn]),
        "var_n": variance(n_values),
    }


def agg(trials, key):
    vals = [tr[key] for tr in trials if not math.isnan(tr[key])]
    if not vals:
        return "n/a"
    if len(vals) == 1:
        return f"{vals[0]:.3f}"
    return f"{mean(vals):.3f} +/- {math.sqrt(variance(vals)):.3f}"


def print_per_trial(trials):
    print("\n== Per-trial metrics ==")
    for tr in trials:
        ss = (f"ss@{tr['ss_start_t']:.1f}s rms_ss={tr['rms_ss']:.3f}m"
              if tr["ss_start_idx"] is not None else "steady state NOT reached")
        print(f"  [{tr['condition']:>8}] {os.path.basename(tr['path'])}: "
              f"{tr['samples']} cycles/{tr['duration_s']:.0f}s, "
              f"rms={tr['rms_whole']:.3f}m, {ss}, "
              f"misses={tr['misses']} ({100 * tr['miss_rate']:.1f}%), "
              f"meanN={tr['mean_n']:.1f}, mean|dN|={tr['mean_abs_dn']:.2f}, "
              f"varN={tr['var_n']:.1f}")


def print_aggregates(by_condition):
    print("\n== Aggregate (mean +/- std across trials) ==")
    metrics = [("trials", None), ("misses", "deadline misses"),
               ("miss_rate", "miss rate"), ("rms_whole", "RMS whole (m)"),
               ("rms_ss", "RMS steady-state (m)"), ("mean_n", "mean N"),
               ("mean_call_ms", "mean call (ms)"),
               ("mean_latency_ms", "mean latency (ms)"),
               ("mean_abs_dn", "mean |dN|"), ("var_n", "var N")]
    for cond in CONDITION_ORDER:
        if cond not in by_condition:
            continue
        trials = by_condition[cond]
        print(f"\n  {cond} (n={len(trials)}):")
        for key, label in metrics[1:]:
            print(f"    {label:>22}: {agg(trials, key)}")


def _pairwise(a, b):
    """(welch_stat, welch_p, mwu_stat, mwu_p) for two samples."""
    welch = scipy_stats.ttest_ind(a, b, equal_var=False)
    mwu = scipy_stats.mannwhitneyu(a, b, alternative="two-sided")
    return welch.statistic, welch.pvalue, mwu.statistic, mwu.pvalue


def cliffs_delta(a, b):
    """Cliff's delta effect size: P(a>b) - P(a<b), in [-1, 1]."""
    gt = sum(1 for x in a for y in b if x > y)
    lt = sum(1 for x in a for y in b if x < y)
    return (gt - lt) / (len(a) * len(b))


def _loo_range(a, b):
    """Leave-one-out sensitivity: (welch_p_min, welch_p_max, mwu_p_min,
    mwu_p_max) over all datasets with one trial removed (from either
    group). Shows whether a conclusion hinges on a single trial."""
    wps, mps = [], []
    for i in range(len(a)):
        sub = a[:i] + a[i + 1:]
        if len(sub) >= 2:
            _, wp, _, mp = _pairwise(sub, b)
            wps.append(wp); mps.append(mp)
    for j in range(len(b)):
        sub = b[:j] + b[j + 1:]
        if len(sub) >= 2:
            _, wp, _, mp = _pairwise(a, sub)
            wps.append(wp); mps.append(mp)
    return min(wps), max(wps), min(mps), max(mps)


def holm(pvals):
    """Holm-Bonferroni adjusted p-values (same order as input)."""
    m = len(pvals)
    order = sorted(range(m), key=lambda i: pvals[i])
    adj = [0.0] * m
    running = 0.0
    for rank, idx in enumerate(order):
        running = max(running, (m - rank) * pvals[idx])
        adj[idx] = min(1.0, running)
    return adj


def print_stats_tests(by_condition):
    print("\n== Statistical tests (ADAPTIVE vs baselines) ==")
    if scipy_stats is None:
        print("  scipy not available -- install python3-scipy")
        return
    if "ADAPTIVE" not in by_condition:
        print("  no ADAPTIVE trials found")
        return
    adaptive = by_condition["ADAPTIVE"]
    rows = []
    for baseline in ["CONST200", "CONST330", "FIXED400"]:
        if baseline not in by_condition:
            continue
        base = by_condition[baseline]
        for key, label in [("misses", "deadline misses"),
                           ("rms_ss", "steady-state RMS")]:
            a = [tr[key] for tr in adaptive if not math.isnan(tr[key])]
            b = [tr[key] for tr in base if not math.isnan(tr[key])]
            if len(a) < 2 or len(b) < 2:
                print(f"  ADAPTIVE vs {baseline} on {label}: "
                      "need >=2 trials per condition")
                continue
            rows.append((baseline, label, a, b) + _pairwise(a, b))

    # Holm correction across the full family of reported comparisons,
    # per test type.
    welch_adj = holm([r[5] for r in rows])
    mwu_adj = holm([r[7] for r in rows])

    for r, wadj, madj in zip(rows, welch_adj, mwu_adj):
        baseline, label, a, b, wstat, wp, mstat, mp = r
        print(f"  ADAPTIVE vs {baseline} on {label}:")
        print(f"    Welch t={wstat:.3f} p={wp:.4f} (Holm-adj {wadj:.4f}) | "
              f"Mann-Whitney U={mstat:.1f} p={mp:.4f} (Holm-adj {madj:.4f}) | "
              f"Cliff's delta={cliffs_delta(a, b):+.2f}")
        wlo, whi, mlo, mhi = _loo_range(a, b)
        print(f"    leave-one-out sensitivity: Welch p in "
              f"[{wlo:.4f}, {whi:.4f}] | MWU p in [{mlo:.4f}, {mhi:.4f}]")


def print_scheduler_validation(by_condition):
    print("\n== Scheduler validation (ADAPTIVE trials, heuristic events) ==")
    if "ADAPTIVE" not in by_condition:
        print("  no ADAPTIVE trials found")
        return
    responses, recoveries = [], []
    for tr in by_condition["ADAPTIVE"]:
        events = extract_spike_events(tr["n_values"], tr["call_ms"])
        for ev in events:
            if ev["response_cycles"] is not None:
                responses.append(ev["response_cycles"])
            if ev["recovery_cycles"] is not None:
                recoveries.append(ev["recovery_cycles"])
        print(f"  {os.path.basename(tr['path'])}: {len(events)} spike "
              f"episode(s) detected")
    if responses:
        print(f"  Response latency: mean {mean(responses):.1f} cycles "
              f"(n={len(responses)}, expected 1-2)")
    if recoveries:
        print(f"  Recovery time:    mean {mean(recoveries):.1f} cycles "
              f"(n={len(recoveries)})")
    if not responses and not recoveries:
        print("  No spike episodes with measurable response found -- check "
              "overlay plots / load window manually.")


def plot_overlay(by_condition, out_path, deadline_ms=DEADLINE_MS):
    fig, axes = plt.subplots(3, 1, figsize=(10, 11), sharex=True)
    for cond in CONDITION_ORDER:
        for k, tr in enumerate(by_condition.get(cond, [])):
            label = cond if k == 0 else None
            color = CONDITION_COLORS.get(cond)
            axes[0].plot(tr["t"], tr["n_values"], color=color, alpha=0.5,
                         linewidth=0.9, label=label)
            axes[1].plot(tr["t"], tr["call_ms"], color=color, alpha=0.5,
                         linewidth=0.9, label=label)
            axes[2].plot(tr["t"], tr["errors"], color=color, alpha=0.5,
                         linewidth=0.9, label=label)
    axes[0].set_ylabel("N")
    axes[0].set_title("Sample count (all trials, by condition)")
    axes[1].axhline(deadline_ms, color="red", linestyle="--", alpha=0.5)
    axes[1].set_ylabel("MPPI call time (ms)")
    axes[1].set_title(f"Computation time (red dashed = {deadline_ms:.0f}ms deadline)")
    axes[2].axhline(SS_THRESHOLD_M, color="gray", linestyle=":", alpha=0.5)
    axes[2].set_ylabel("Position error (m)")
    axes[2].set_xlabel("Time since tracking start (s)")
    axes[2].set_title("Tracking error (gray dotted = steady-state threshold)")
    for ax in axes:
        ax.grid(True, alpha=0.3)
        ax.legend()
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def plot_dn_distribution(by_condition, out_path):
    conds = [c for c in CONDITION_ORDER if c in by_condition]
    fig, axes = plt.subplots(1, len(conds), figsize=(5 * len(conds), 4),
                             squeeze=False)
    for ax, cond in zip(axes[0], conds):
        dn = []
        for tr in by_condition[cond]:
            n = tr["n_values"]
            dn += [n[i] - n[i - 1] for i in range(1, len(n))]
        ax.hist(dn, bins=41, color=CONDITION_COLORS.get(cond), alpha=0.8)
        ax.set_title(f"dN distribution -- {cond}")
        ax.set_xlabel("dN = N_t - N_(t-1)")
        ax.set_ylabel("count")
        ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def plot_boxplots(by_condition, out_path):
    conds = [c for c in CONDITION_ORDER if c in by_condition]
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))
    for ax, key, title in [(axes[0], "misses", "Deadline misses per trial"),
                           (axes[1], "rms_ss", "Steady-state RMS error (m)")]:
        data = [[tr[key] for tr in by_condition[c]
                 if not math.isnan(tr[key])] for c in conds]
        ax.boxplot(data, labels=conds)
        ax.set_title(title)
        ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("paths", nargs="+",
                        help="CSV files and/or directories of trials")
    parser.add_argument("--output-dir", default=None,
                        help="figure output dir (default: <first path>/analysis)")
    parser.add_argument("--deadline-ms", type=float, default=DEADLINE_MS,
                        help="deadline line drawn on the call-time plot "
                             "(50 for the 20Hz condition, 25 for 40Hz)")
    args = parser.parse_args()

    files = collect_csv_files(args.paths)
    if not files:
        sys.exit("No CSV files found.")

    trials = [tr for tr in (analyze_trial(f) for f in files) if tr]
    by_condition = defaultdict(list)
    for tr in trials:
        by_condition[tr["condition"]].append(tr)

    print(f"Loaded {len(trials)} trial(s): " + ", ".join(
        f"{c}={len(by_condition[c])}" for c in CONDITION_ORDER
        if c in by_condition))

    print_per_trial(trials)
    print_aggregates(by_condition)
    print_stats_tests(by_condition)
    print_scheduler_validation(by_condition)

    out_dir = args.output_dir or os.path.join(
        args.paths[0] if os.path.isdir(args.paths[0]) else ".", "analysis")
    os.makedirs(out_dir, exist_ok=True)
    plot_overlay(by_condition, os.path.join(out_dir, "overlay_timeseries.png"),
                 deadline_ms=args.deadline_ms)
    plot_dn_distribution(by_condition, os.path.join(out_dir, "dn_distribution.png"))
    plot_boxplots(by_condition, os.path.join(out_dir, "boxplots.png"))
    print(f"\nFigures written to {out_dir}/")


if __name__ == "__main__":
    main()
