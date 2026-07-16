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

- **H1:** Adaptive MPPI produces fewer deadline misses than fixed-N=400 MPPI under sustained
  CPU contention.
- **H2:** Adaptive MPPI's tracking error (RMS, steady-state) is not significantly worse than
  fixed-N=400's.
- **H3:** Adaptive MPPI's average sample count is measurably lower than 400 under contention.
- **H4:** The scheduler's sample count responds proportionally and promptly to changes in
  measured per-cycle compute cost (visible as N dropping within 1-2 cycles of a call-time spike
  and recovering within a few cycles of load easing).
- **H5 (new, addresses RQ4):** Adaptive MPPI outperforms (fewer deadline misses and/or better
  tracking) a *constant*-N controller fixed at adaptive's own observed average N, under the
  same variable-load conditions. This isolates "adaptation helps" from "a lower fixed budget
  would have been enough all along."

## Success Criteria

- Deadline misses: adaptive < fixed, under matched load conditions.
- Tracking error: adaptive not significantly worse than fixed (statistical test, not eyeballing).
- Average N: adaptive measurably below n_max under contention.
- Adaptive outperforms constant-N-at-adaptive-average (H5) on at least one of deadline misses
  or tracking error.
- Control loop frequency (20Hz nominal) maintained without sustained degradation in all
  conditions tested.

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
- [ ] Run 3x adaptive, 3x fixed (N=400), same 32-thread/30s load profile as the confirmed
  single-trial result.
- [ ] **Also run 3x constant-N-at-adaptive-average** (H5 / RQ4 condition — see below).
- [ ] Manually inspect all 9 resulting CSVs and plots. Confirm: correct phase gating, no
  crashes, no missing rows, no clock/timestamp anomalies, sensible N/latency/error ranges.
- [ ] Fix anything broken here before proceeding — this is the checkpoint the review
  specifically recommended.

### Phase 2B — Full trial set
- [ ] Run remaining trials to reach 10-20x per condition (adaptive, fixed N=400, constant-N-
  at-adaptive-average) — three conditions total, not two.
- [ ] Same load profile throughout for the primary comparison; see Phase 2B-extended below for
  the second stress condition.
- [ ] **Phase 2B-extended:** repeat the three-condition comparison under a second stress
  condition (ROS2 latency injection or reduced loop rate) — at reduced trial count (5-10x) if
  time is tight, full count if time allows.

### Phase 2C — Statistics & reporting
- [ ] Compute mean/stddev across trials for: deadline misses, RMS tracking error (both
  whole-trajectory and **steady-state-only**, see § Transient/Steady-State Reporting below),
  mean N, mean MPPI call time, mean state-to-command latency.
- [ ] Run an appropriate statistical test (e.g. Welch's t-test or Mann-Whitney U, given likely
  non-normal small-sample distributions) comparing adaptive vs fixed and adaptive vs
  constant-N, for both deadline-miss count and RMS error.
- [ ] Report the **equivalent-computational-budget comparison** (per-trial average N vs
  Pareto-table-interpolated quality) across the full trial set, not just the single Phase 0
  instance.
- [ ] Regenerate all plots as aggregate/overlay figures using the full trial set.

**Estimated time: 7–12 days total across 2A/2B/2C.** Budget slack here — this remains the
highest schedule-risk phase.

### New addition: Constant-N baseline (addresses RQ4 / H5)
Per review: in addition to Fixed (N=400) and Adaptive, add a third condition — **Fixed at a
constant N equal to adaptive's own observed average** (currently ≈330-367 depending on run;
finalize the exact constant once Phase 2A's adaptive pilot runs give a stable average). This
directly tests whether the *scheduler itself* is doing useful work, or whether a fixed lower
budget would have suffficed all along. If adaptive still shows fewer deadline misses and/or
better tracking than this constant-N condition under the same variable load, that is strong,
specific evidence for the adaptation mechanism itself — not just for "use less compute."

### Transient vs. Steady-State Reporting (new, from latest flight analysis)
The most recent confirmed flight (see PROJECT_OVERVIEW.md § 6) showed whole-trajectory RMS of
1.03m, but steady-state (t > 15s, after the initial ~3.4m takeoff-to-first-target transient
resolves) RMS is visibly under 0.15m from the plot. **Report both numbers going forward**, not
just whole-trajectory RMS — the transient is a real, explainable phenomenon (physical distance
between arming location and first active waypoint) and splitting it out prevents a reviewer
mistakenly reading 1.03m as steady-state tracking quality.
- [ ] Write a small script/analysis step (extending `plot_flight_log.py` or a new script) that
  computes steady-state-only RMS by excluding an initial transient window (e.g. first 15s, or
  more robustly, everything before tracking error first drops below some threshold like 0.5m
  and stays there).

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
1. Confirm the automated run script exists and works (top blocker for Phase 2 at scale).
2. Finalize the constant-N value to test (based on adaptive's average across a few pilot runs).
3. Run Phase 2A (3x each of adaptive / fixed / constant-N) — inspect everything before scaling up.
4. Write the steady-state-RMS filtering analysis (small, high-value, addresses a real reviewer
   question about the 1.03m whole-trajectory RMS number).
5. Proceed to Phase 2B once 2A is clean.
