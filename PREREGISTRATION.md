# Pre-registration record

This project fixed its experimental design decisions in version control *before* the
corresponding data were collected and analysed. This document lists each pre-registered
decision, the commit that fixed it, and the commit in which the resulting data first
appeared, so that the ordering can be independently verified.

Every claim below is checkable from this repository's history. To inspect the exact text
of a decision as it stood when it was fixed:

```
git show <commit>:PROJECT_PLAN.md
```

To confirm the ordering of any pair:

```
git show -s --format='%ci %s' <pre-registration commit> <data commit>
```

Commit timestamps establish when a decision was recorded. Because trials write their own
wall-clock timestamps into the first column of every CSV, the actual time of each flight
can also be recovered independently of the commit history, and is reported below where
relevant.

---

## 1. Equivalent-budget baseline fixed at N = 330

**Decision.** The constant-sample-count control condition was set to N = 330, chosen as
the adaptive scheduler's measured mean sample count (329.77) in a preliminary
stress-test experiment. Fixing the value in advance prevents the baseline from being
tuned to the results it is compared against.

| | Commit | Timestamp |
|---|---|---|
| Decision fixed | `70110de`, reaffirmed in `ba1406c` | 2026-07-16 12:55 BST |
| First N = 330 data | `f50d5a3` | 2026-07-17 17:29 BST |

**Status:** decision precedes data by approximately one day. The value was not revised
after any Phase 2 result.

---

## 2. Stress condition calibrated, then locked

**Decision.** The compute-contention profile (controller and load generator pinned to
two cores; six busy-wait threads; 30 s window beginning 20 s after tracking starts) was
calibrated on single fixed-budget trials and then locked before any comparison trials
were run. Calibration selected for a condition in which the conventional fixed baseline
breaches its deadline while flight remains valid; it could not, and did not, tune the
adaptive-versus-constant comparison that was subsequently reported as null.

| | Commit / event | Timestamp |
|---|---|---|
| Condition locked | `93988f3` | 2026-07-17 21:15:59 BST |
| Earliest Phase 2 flight | first CSV row in `results/` | 2026-07-17 21:19:52 BST |

**Status:** all reported Phase 2 flights postdate the lock.

---

## 3. Constant-200 arm

**Decision.** A second constant baseline near the Pareto elbow (N = 200) was added after
external review, with interpretation rules recorded in advance: a Constant-200 failure in
either operating regime would support the transfer claim, whereas Constant-200 succeeding
in both would narrow the paper's claims to the tuning-free framing alone. Both outcomes
were committed to in advance; the former occurred.

| | Commit / event | Timestamp |
|---|---|---|
| Interpretation rules fixed | `6271fe7` | 2026-07-18 22:28:32 BST |
| Earliest 20 Hz flight | first CSV row, `results/const200/` | 2026-07-18 22:24:50 BST |
| Earliest 40 Hz flight | first CSV row, `results_40hz/const200/` | 2026-07-18 22:39:32 BST |
| Data committed | `8b166de` | 2026-07-18 22:59:34 BST |

**Status — partial, disclosed.** The 40 Hz arm is clean: all flights postdate the
commit. For the 20 Hz arm, three of eight trials (01–03) began between three minutes
before and two seconds before the interpretation rules were committed, because the batch
was launched while the rules were being written. No data from either arm were inspected
or analysed before the commit, so the substantive protection — fixing the interpretation
before seeing results — held. The timestamp evidence for those three trials nevertheless
does not establish it independently, and is reported here rather than omitted.

---

## 4. Quality-floor scheduler (ADAPTIVE-QF)

**Decision.** The Pareto-informed quality floor was implemented as a pre-registered
refinement, with per-regime predictions recorded before any trial: that the floor would
never activate at 20 Hz (the deadline law's allocation remaining far above it), that it
would bind at 40 Hz under load, and that the true conflict case would be unreachable
through natural contention and demonstrable only by fault injection. The floor value
itself, N = 150, was derived from the offline sweep as the smallest sample count whose
predicted quality lies within one noise-floor standard deviation of the maximum, not
chosen to fit a result.

| | Commit / event | Timestamp |
|---|---|---|
| Predictions fixed | `ae45409` | 2026-07-18 23:47:33 BST |
| Earliest 20 Hz flight | first CSV row, `results/adaptiveqf/` | 2026-07-18 23:50:49 BST |
| Earliest 40 Hz flight | first CSV row, `results_40hz/adaptiveqf/` | 2026-07-19 00:05:35 BST |
| Data and outcome committed | `0401105` | 2026-07-19 15:13:32 BST |

**Status:** clean. All flights in both regimes postdate the commit fixing the
predictions. Both regime-level predictions were confirmed.

---

## Scope and limitations of this record

This is a self-administered pre-registration in version control, not registration with a
third-party registry. It establishes that a decision existed in an immutable, timestamped
form before the corresponding data, which is weaker than an external registry — a
repository owner can rewrite history, and the record's value rests on the commits not
having been rewritten in ways that alter the recorded decisions or their timing.

This history was rewritten once, after all data collection and analysis were complete,
to remove personal working documents that were never part of the scientific record. That
rewrite changed commit identifiers; it preserved every author and committer timestamp,
the full text of the recorded decisions, and the ordering between each decision and its
data. The identifiers cited above are those of the current history. A copy of the
pre-rewrite history is retained separately.

The pre-registrations cover experimental design decisions: baseline values, the stress
condition, added arms, and their interpretation rules. They do not cover the hypotheses
themselves, which were stated at the outset of the project, nor the analysis code, which
was written before the corresponding data in each case but is not separately timestamped
here.
