#!/usr/bin/env python3
"""
Generate the paper's figures (PDF, IEEE single-column sizing) from the
committed trial data. Outputs to paper/figures/.

    python3 scripts/paper_figures.py

Figures:
  fig_pareto.pdf       quality-compute trade-off (Phase 0 sweep) with the
                       adaptive operating points from both regimes
  fig_overlay_20hz.pdf N / call time / error, all trials, 20Hz
  fig_overlay_40hz.pdf same, 40Hz
  fig_dn_hist.pdf      dN distribution, adaptive, both regimes
  fig_fault.pdf        fault-injection episode (fallback validation)
"""
import csv
import glob
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams.update({
    "font.size": 8, "axes.titlesize": 8, "axes.labelsize": 8,
    "legend.fontsize": 7, "xtick.labelsize": 7, "ytick.labelsize": 7,
    "figure.dpi": 150, "savefig.bbox": "tight",
    "lines.linewidth": 0.9,
})

COL_W = 3.45  # IEEE column width, inches

CONDS = ["ADAPTIVE", "CONST200", "CONST330", "FIXED400"]
COLORS = {"ADAPTIVE": "tab:blue", "CONST200": "tab:red",
          "CONST330": "tab:orange", "FIXED400": "tab:green"}
LABELS = {"ADAPTIVE": "Adaptive", "CONST200": "Const-200",
          "CONST330": "Const-330", "FIXED400": "Fixed-400"}

PARETO = [  # N, mean_rms, std_rms, cost_ms (Phase 0 sweep, 10 seeds/N)
    (20, 0.9115, 0.0405, 1.1387), (50, 0.8719, 0.0280, 2.7977),
    (100, 0.8393, 0.0491, 5.5407), (150, 0.8187, 0.0215, 8.4159),
    (200, 0.7906, 0.0195, 11.6946), (250, 0.8075, 0.0295, 14.4366),
    (300, 0.7980, 0.0291, 17.2648), (350, 0.7872, 0.0269, 19.6126),
    (400, 0.7893, 0.0301, 22.0119),
]

OUT = os.path.join(os.path.dirname(__file__), "..", "paper", "figures")


def tracking(path):
    with open(path) as f:
        return [r for r in csv.DictReader(f) if r["phase"] == "TRACKING"]


def by_condition(results_dir):
    out = {}
    for f in sorted(glob.glob(os.path.join(results_dir, "*", "trial_*.csv"))):
        tr = tracking(f)
        if not tr:
            continue
        mode = tr[0]["mode"]
        mode = "FIXED400" if mode == "FIXED" else mode
        out.setdefault(mode, []).append(tr)
    return out


def fig_pareto():
    fig, ax = plt.subplots(figsize=(COL_W, 2.2))
    ns = [p[0] for p in PARETO]
    ax.errorbar(ns, [p[1] for p in PARETO], yerr=[p[2] for p in PARETO],
                marker="o", markersize=3, capsize=2, color="tab:blue",
                label="RMS error (10 seeds)")
    ax.set_xlabel("Sample count $N$")
    ax.set_ylabel("RMS tracking error", color="tab:blue")
    ax.tick_params(axis="y", labelcolor="tab:blue")
    ax2 = ax.twinx()
    ax2.plot(ns, [p[3] for p in PARETO], marker="s", markersize=3,
             color="tab:red", linestyle="--", label="Compute cost")
    ax2.set_ylabel("Cost per call [ms]", color="tab:red")
    ax2.tick_params(axis="y", labelcolor="tab:red")
    # adaptive operating points (mean N per regime)
    for n_op, lab in [(365, "20 Hz"), (220, "40 Hz")]:
        ax.axvline(n_op, color="gray", linestyle=":", linewidth=0.8)
        ax.annotate(f"adaptive\n({lab})", xy=(n_op, 0.905),
                    ha="center", fontsize=6, color="gray")
    ax.grid(True, alpha=0.3)
    fig.savefig(os.path.join(OUT, "fig_pareto.pdf"))
    plt.close(fig)


def fig_overlay(results_dir, deadline_ms, fname):
    data = by_condition(results_dir)
    fig, axes = plt.subplots(3, 1, figsize=(COL_W, 4.6), sharex=True)
    for cond in CONDS:
        for k, tr in enumerate(data.get(cond, [])):
            t0 = float(tr[0]["epoch_sec"])
            t = [float(r["epoch_sec"]) - t0 for r in tr]
            lab = LABELS[cond] if k == 0 else None
            c = COLORS[cond]
            axes[0].plot(t, [int(r["N"]) for r in tr], color=c, alpha=0.35,
                         linewidth=0.6, label=lab)
            axes[1].plot(t, [float(r["mppi_call_ms"]) for r in tr], color=c,
                         alpha=0.35, linewidth=0.6, label=lab)
            axes[2].plot(t, [float(r["pos_error_m"]) for r in tr], color=c,
                         alpha=0.35, linewidth=0.6, label=lab)
    axes[0].set_ylabel("$N$")
    axes[1].axhline(deadline_ms, color="red", linestyle="--", linewidth=0.8)
    axes[1].annotate(f"{deadline_ms:.0f} ms deadline",
                     xy=(0.99, deadline_ms), xycoords=("axes fraction", "data"),
                     ha="right", va="bottom", fontsize=6, color="red")
    axes[1].set_ylabel("Call time [ms]")
    axes[2].set_ylabel("Pos. error [m]")
    axes[2].set_xlabel("Time since tracking start [s]")
    axes[2].set_ylim(0, 3)
    for ax in axes:
        ax.grid(True, alpha=0.3)
    leg = axes[0].legend(loc="lower right", ncol=2, framealpha=0.9)
    for lh in leg.legendHandles:
        lh.set_alpha(1.0)
        lh.set_linewidth(1.5)
    fig.savefig(os.path.join(OUT, fname))
    plt.close(fig)


def fig_dn_hist():
    fig, axes = plt.subplots(1, 2, figsize=(COL_W, 1.7), sharey=True)
    for ax, (rd, lab) in zip(
            axes, [("results", "20 Hz"), ("results_40hz", "40 Hz")]):
        dn = []
        for tr in by_condition(rd).get("ADAPTIVE", []):
            n = [int(r["N"]) for r in tr]
            dn += [n[i] - n[i - 1] for i in range(1, len(n))]
        ax.hist(dn, bins=61, range=(-150, 150), color="tab:blue", alpha=0.85)
        ax.set_yscale("log")
        ax.set_title(f"{lab} (adaptive)")
        ax.set_xlabel(r"$\Delta N_t$")
        ax.grid(True, alpha=0.3)
    axes[0].set_ylabel("Cycles (log)")
    fig.savefig(os.path.join(OUT, "fig_dn_hist.pdf"))
    plt.close(fig)


def fig_fault():
    f = sorted(glob.glob("results_fault_injection/adaptive/trial_*.csv"))[0]
    tr = tracking(f)
    t0 = float(tr[0]["epoch_sec"])
    rows = [r for r in tr if 57 <= float(r["epoch_sec"]) - t0 <= 65]
    t = [float(r["epoch_sec"]) - t0 for r in rows]
    fig, axes = plt.subplots(2, 1, figsize=(COL_W, 2.8), sharex=True)
    axes[0].plot(t, [float(r["mppi_call_ms"]) for r in rows],
                 color="tab:blue", marker=".", markersize=2)
    axes[0].axhline(50, color="red", linestyle="--", linewidth=0.8)
    axes[0].set_ylabel("Call time [ms]")
    axes[0].annotate("injected stall", xy=(60.6, 78), fontsize=6,
                     ha="left", va="top")
    axes[1].plot(t, [int(r["N"]) for r in rows], color="tab:blue",
                 marker=".", markersize=2)
    fb = [(float(r["epoch_sec"]) - t0, int(r["N"])) for r in rows
          if r.get("fallback_active") == "1"]
    if fb:
        axes[1].plot([x for x, _ in fb], [y for _, y in fb], "r*",
                     markersize=8, label="fallback signaled")
        axes[1].legend(loc="lower right")
    axes[1].set_ylabel("$N$")
    axes[1].set_xlabel("Time since tracking start [s]")
    for ax in axes:
        ax.grid(True, alpha=0.3)
    fig.savefig(os.path.join(OUT, "fig_fault.pdf"))
    plt.close(fig)


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    fig_pareto()
    fig_overlay("results", 50, "fig_overlay_20hz.pdf")
    fig_overlay("results_40hz", 25, "fig_overlay_40hz.pdf")
    fig_dn_hist()
    fig_fault()
    print("Figures written to", os.path.abspath(OUT))
