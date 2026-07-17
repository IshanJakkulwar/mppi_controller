# Project Plan — Compute-Aware Anytime MPPI for UAV Control
**Target: ICRA/ECC submission. Timeline: ~7 weeks remaining from this point.**

---

## Research Questions

- **RQ1:** Can runtime sample-count adaptation reduce control-deadline misses under variable
  compute availability, relative to a fixed-sample baseline?
- **RQ2:** Does adaptation preserve tracking accuracy relative to a fixed-sample baseline?
- **RQ3:** Does adaptation reduce average computational cost relative to a fixed-sample baseline?
- **RQ4:** Is the specific adaptation policy responsible for any observed benefit, or would a
  constant reduced sample count (matched to the adaptive average) achieve the same result?
  *(This is the question a constant-N control condition — see § Phase 2 — is designed to answer.)*

## Hypotheses

*All hypotheses compare controllers. Verification of the scheduler's own implementation
behaviour is not a hypothesis — see § Scheduler Validation below.*

- **H1:** Adaptive MPPI produces fewer deadline misses than fixed-N=400 MPPI under sustained
  CPU contention.
- **H2:** Adaptive MPPI's tracking error (RMS, steady-state) is not significantly worse than
  fixed-N=400's.
- **H3:** Adaptive MPPI's average sample count is measurably lower than 400 under contention.
- **H4 (addresses RQ4):** Adaptive MPPI outperforms (fewer deadline misses and/or better
  tracking) a *constant*-N controller fixed at **N=330** (see § Constant-N baseline for why
  this value is predetermined), under the same variable-load conditions. This isolates
  "adaptation helps" from "a lower fixed budget would have been enough all along."

## Scheduler Validation (engineering metrics, not hypotheses)

*(Restructured per review: "scheduler responds within 1-2 cycles" verifies implementation
behaviour, so it is reported as engineering validation, not as a scientific hypothesis.)*
Reported from adaptive-condition trials, around the controlled load window:

- **Response latency:** cycles between a measured call-time spike and the resulting drop in N
  (expected: 1-2 cycles, matching the smoothing window of 3).
- **Recovery time:** cycles from load easing until N returns near its pre-load level.
- **Maximum overshoot/undershoot:** how far N transiently over-corrects during transitions.
- **Stability (chatter):** ΔN_t = N_t − N_{t−1}; report **mean |ΔN|** and **variance of N**,
  plus a ΔN distribution figure. A scheduler oscillating e.g. 400→150→400→150 technically
  meets deadlines but is unstable — the ΔN distribution answers "does the scheduler chatter?"
  directly.

## Success Criteria

- Deadline misses: adaptive < fixed, under matched load conditions.
- Tracking error: adaptive not significantly worse than fixed (statistical test, not eyeballing).
- Average N: adaptive measurably below n_max under contention.
- Adaptive outperforms constant-N=330 (H4) on at least one of deadline misses or tracking error.
- Control loop frequency (20Hz nominal) maintained without sustained degradation in all
  conditions tested.
- Scheduler validation metrics within expected ranges (response ≤ ~2 cycles, no sustained
  chatter in the ΔN distribution).

---

## Phase 0 — Sharpen the Contribution — ✅ CORE DONE

### Completed
- [x] `test_n_quality_sweep.cpp`: 10-seed sweep, N ∈ {20..400}. Elbow identified ~N=150-200;
  gains beyond that are within the measurement's own noise floor.
- [x] `analyze_equivalent_budget.cpp`: cross-referenced Pareto table against real stress-test
  log (31 real samples, exact mean N=329.77). Estimated quality gap vs fixed N=400 was smaller
  than the table's own noise floor.
- [x] Literature check completed; gap identified between MPPI-for-UAV work (fixed N) and
  control-scheduling co-design (adapts online, but for deterministic control). `references.bib`
  started, several entries flagged for author-list verification before final citation.

### Remaining Phase 0 work
- [ ] Write the formal contribution statement bridging the two literatures (draft exists in
  PROJECT_OVERVIEW.md § 8, needs finalizing after Phase 2 data is in).
- [ ] **Important correction (per review):** the scheduler's *current implementation* is
  `N = budget / measured_time_per_sample` — a direct heuristic. The "smallest N satisfying
  deadline AND quality threshold" framing is a **planned refinement, not yet implemented.**
  Documentation must describe what the code currently does, with the Pareto-informed version
  explicitly marked as future work, until the refinement is actually coded. Do not describe the
  refined version as already implemented anywhere in paper drafts.
- [ ] Decide whether to actually implement the Pareto-informed refinement before Phase 2, or
  defer it and run Phase 2 against the current heuristic, adding the refinement as a later
  ablation arm. **Recommendation: defer** — Phase 2 needs to start soon; the current heuristic
  is a legitimate, describable baseline in its own right, and the refinement can be Phase 2's
  final ablation condition rather than a blocking prerequisite.
- [ ] Verify author lists for flagged references.bib entries.

**Estimated remaining time: 1 day** (reduced — deferring the scheduler refactor).

---

## Phase 1 — Infrastructure — ✅ DONE

### Completed
- [x] CSV logging in `offboard_node.cpp`.
- [x] Communication-latency timer added, **renamed from `mavros_roundtrip_ms` to
  `state_to_command_latency_ms`** for accuracy (it measures pose-received → command-published
  wall time, which is a latency measurement, not strictly a MAVROS-internal round-trip).
- [x] `plot_flight_log.py`: 4-panel plot + summary stats, multi-file overlay support.
- [x] **Tracking-phase gating bug fixed and verified.** Root cause: tracking phase was
  originally gated only on the takeoff ramp's fixed timer, not actual PX4 arm+OFFBOARD state.
  Fix: gate on `current_state_.armed && current_state_.mode == "OFFBOARD"`. Confirmed via new
  flight log — node now logs "Vehicle is armed and in OFFBOARD. Beginning MPPI tracking." at
  the correct moment, and the resulting tracking-error plot no longer shows the artificial flat
  plateau seen in the pre-fix run.
- [x] Automated run script — *(confirm status; if not yet built, this remains the top blocker
  for Phase 2 — see Immediate Next Actions)*.

**Status: essentially complete.** Remaining item is confirming the automated run script exists
before Phase 2 trials begin at scale.

---

## Phase 2 — SITL Statistical Validation — STAGED (per review feedback)

**Restructured into three explicit stages, rather than "run 10-20x" as a single block — this
catches logging/script problems early rather than after 18 wasted trials.**

### Phase 2A — Pilot (small, fast, catches problems)
Run order per review: **Adaptive → Constant-330 → Fixed-400.** (With N=330 predetermined —
see § Constant-N baseline — the order no longer matters scientifically; kept as workflow
hygiene.)
- [ ] Run 3x adaptive, 3x constant-N=330, 3x fixed (N=400), same 32-thread/30s load profile
  as the confirmed single-trial result, load window at a fixed offset after tracking begins.
- [ ] Manually inspect all 9 resulting CSVs and plots. Confirm: correct phase gating, no
  crashes, no missing rows, no clock/timestamp anomalies, sensible N/latency/error ranges.
- [ ] Sanity-check that adaptive's average N in the pilot lands reasonably near 330 — if it
  is wildly different (e.g. <250 or >390), document why before Phase 2B (the constant stays
  330 regardless; it is predetermined and does not get re-tuned to Phase 2 results).
- [ ] Fix anything broken here before proceeding — this is the checkpoint the review
  specifically recommended.

### Phase 2B — Full trial set
- [ ] Run remaining trials to reach 10-20x per condition (adaptive, constant N=330, fixed
  N=400) — three conditions total, not two.
- [ ] Same load profile throughout for the primary comparison; see Phase 2B-extended below for
  the second stress condition.
- [ ] **Phase 2B-extended:** repeat the three-condition comparison under a second stress
  condition (ROS2 latency injection or reduced loop rate) — at reduced trial count (5-10x) if
  time is tight, full count if time allows.

### Phase 2C — Statistics & reporting
- [ ] Compute mean/stddev across trials for: deadline misses, RMS tracking error (both
  whole-trajectory and **steady-state-only**, see § Transient/Steady-State Reporting below),
  mean N, mean MPPI call time, mean state-to-command latency.
- [ ] **Scheduler stability metrics (per review):** ΔN_t = N_t − N_{t−1}; report mean |ΔN|
  and variance of N per adaptive trial, plus a ΔN distribution figure. Answers "does the
  scheduler chatter?" — see § Scheduler Validation.
- [ ] **Scheduler validation metrics** (response latency, recovery time, overshoot) extracted
  from the controlled load window of adaptive trials — see § Scheduler Validation.
- [ ] Run an appropriate statistical test (e.g. Welch's t-test or Mann-Whitney U, given likely
  non-normal small-sample distributions) comparing adaptive vs fixed and adaptive vs
  constant-N, for both deadline-miss count and RMS error.
- [ ] Report the **equivalent-computational-budget comparison** (per-trial average N vs
  Pareto-table-interpolated quality) across the full trial set, not just the single Phase 0
  instance.
- [ ] Regenerate all plots as aggregate/overlay figures using the full trial set.

**Estimated time: 7–12 days total across 2A/2B/2C.** Budget slack here — this remains the
highest schedule-risk phase.

### Constant-N baseline (addresses RQ4 / H4) — value PREDETERMINED
In addition to Fixed (N=400) and Adaptive, a third condition: **constant N = 330**.

**The value is fixed a priori, before any Phase 2 trial runs** (per review): 330 ≈ the
adaptive scheduler's measured average N (329.77, computed from 31 real log samples) during the
Phase 0 preliminary stress-test experiment. Deriving the constant from Phase 2's own adaptive
runs would make the baseline depend on the result it is being compared against, inviting the
criticism that the baseline was tuned after seeing results. It is locked at 330 now and does
not change regardless of what average N Phase 2's adaptive runs produce.

This condition directly tests whether the *scheduler itself* is doing useful work, or whether
a fixed lower budget would have sufficed all along. If adaptive still shows fewer deadline
misses and/or better tracking than constant-330 under the same variable load, that is strong,
specific evidence for the adaptation mechanism itself — not just for "use less compute."

### Transient vs. Steady-State Reporting — algorithmic definition
The most recent confirmed flight (see PROJECT_OVERVIEW.md § 6) showed whole-trajectory RMS of
1.03m, inflated by the initial takeoff-to-first-target transient; steady-state error after the
transient is visibly under 0.15m. **Report both numbers going forward** — the transient is a
real, explainable phenomenon (physical distance between arming location and first active
waypoint) and splitting it out prevents a reviewer mistakenly reading 1.03m as steady-state
tracking quality.

**Steady-state is defined algorithmically, not by a hardcoded time cutoff** (per review — "the
first 15 seconds" would be criticised as arbitrary):

> Steady state begins at the first control cycle of the first run of **10 consecutive
> TRACKING-phase cycles** (0.5s at 20Hz) in which position error **e(t) < 0.5m**.

Applied uniformly to every trial in every condition by `scripts/analyze_trials.py`.
- [x] Analysis script implementing this definition (`scripts/analyze_trials.py`).

---

## Phase 3 — Literature & Framing — PARTIALLY DONE

- [x] Initial lit search, gap identified.
- [ ] Full Related Work section.
- [ ] **Language correction (per review):** avoid absolute novelty claims like "no identified
  work exists." Use "to the best of our knowledge" framing throughout — standard, defensible,
  and does not overclaim.
- [ ] Finalize framing once Phase 2 data is in.

**Estimated time: 2–3 days, spread across Phase 0/2 timeline.**

---

## Phase 4 — HITL via UCL — NOT STARTED
*(Unchanged from prior plan — see PROJECT_OVERVIEW.md for architecture/context. Summary:)*
- [ ] Confirm rig specs (real Pixhawk/Jetson vs simulated physics; MAVROS vs native DDS).
- [ ] Port `offboard_node`, retune scheduler bounds for real hardware compute profile.
- [ ] Re-run core comparison (adaptive / fixed / constant-N) on HITL — confirmatory scope if
  time is short, full statistical rigor if time allows.

**Estimated time: 10–16 days.** Highest schedule risk. Fallback options (SITL-only submission
vs thin HITL confirmatory results) remain live — decide once Phase 2 is complete.

---

## Phase 5 — Writing — NOT STARTED (Abstract/Intro can start early)

- [ ] Method section.
- [ ] Related Work.
- [ ] **Threats to Validity subsection (new, per review — reviewers specifically value this):**
  - *Internal validity:* stochastic MPPI sampling introduces run-to-run variance; Gazebo physics
    engine variability; scheduler timing jitter from OS/ROS2 scheduling noise on the test
    machine.
  - *External validity:* evaluated on a single quadrotor trajectory-tracking task and vehicle
    model; single-scenario Pareto sweep (straight-line tracking) may not generalize to all
    trajectory shapes; development/SITL machine's CPU differs substantially from an embedded
    companion computer (Jetson Nano/Orin) — HITL results (Phase 4) address this if completed.
  - *Construct validity:* the synthetic CPU load generator (busy-wait threads) approximates
    compute contention but may not perfectly replicate real contention sources (e.g. a
    concurrent perception pipeline's actual CPU usage pattern, cache effects, memory bandwidth
    contention).
- [ ] Experimental Design (finalized from actual Phase 2/4 runs).
- [ ] Results — **report hedged, not absolute, language for anything based on limited trials**
  (per review: "In an initial stress-test experiment, X..." not "X proves Y" until Phase 2C's
  statistics back it up).
- [ ] Discussion/Limitations.
- [ ] Abstract/Intro — draft early, finalize last.

**Estimated time: 7–10 days.**

---

## Phase 6 — Polish — NOT STARTED
- [ ] Advisor/mentor review pass.
- [ ] Figure formatting, page limit compliance.
- [ ] Proofreading, LaTeX template conformance, submission logistics.

**Estimated time: 4–6 days.**

---

## Overall Timeline Reality Check

Sum of realistic-case estimates: **~5–8 weeks** against **~7 weeks remaining**. Fits only in the
optimistic-to-middle case. Phase 2 (now with a third condition and staged rollout) and Phase 4
remain the biggest schedule risks. Keep Phase 4 fallback options genuinely live.

## Immediate Next Actions (in order)
1. ~~Automated run script~~ — **DONE**: `scripts/run_trial.py` (single/batch trials, fixed
   load-window offset, auto CSV collection into `results/<condition>/`).
2. ~~Finalize constant-N value~~ — **DONE, predetermined**: N=330 (locked a priori from the
   Phase 0 preliminary average of 329.77 — see § Constant-N baseline). `offboard_node` now
   accepts a `fixed_n` parameter.
3. ~~Steady-state-RMS analysis~~ — **DONE**: `scripts/analyze_trials.py` (algorithmic
   steady-state definition, per-trial + aggregate stats, ΔN chatter metrics, statistical
   tests, aggregate figures).
4. Run Phase 2A (3x each: adaptive → constant-330 → fixed-400) — inspect everything before
   scaling up.
5. Proceed to Phase 2B once 2A is clean.
