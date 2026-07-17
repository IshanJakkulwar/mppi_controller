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

### Phase 2A — Pilot (small, fast, catches problems) — ✅ RUN 2026-07-17, PROBLEMS FOUND
Run order per review: **Adaptive → Constant-330 → Fixed-400.** (With N=330 predetermined —
see § Constant-N baseline — the order no longer matters scientifically; kept as workflow
hygiene.)
- [x] Ran 3x adaptive, 3x constant-N=330, 3x fixed (N=400), 32-thread/30s load at t+20.
- [x] Inspected all 9 CSVs. **Compute-side results clean and strong** (this is the checkpoint
  working as intended):
  - Deadline misses: adaptive ~14 avg vs const330 ~333 vs fixed400 ~302 (Welch p<0.001 both).
  - Loop rate: adaptive holds 20.0 Hz; const330/fixed400 degrade to 17-18 Hz under load.
  - Scheduler mechanism textbook: N drops within ~1 cycle of load onset, recovers within ~1
    cycle of load end. tag: `phase2a-pilot`.
- [x] **Problems found — tracking-quality comparison NOT usable from this pilot:**
  1. **Between-trial reset failed:** 8 of 9 trials started at ~5m altitude (residual hover
     from the previous trial), not a ground takeoff — the old AUTO.LAND+disarm-wait did not
     actually land the vehicle.
  2. **Inconsistent trajectory coverage:** trials flew 1/4 to 4/4 waypoints; within-condition
     RMS ranged 0.43-2.20 m — noise, not signal.
  3. **Intermittent stall:** some trials the drone wedged ~0.5m from a target and never
     recovered, persisting even after the load ended (e.g. adaptive_03 stuck for 100s despite
     20Hz/0 misses). A persistent-after-load stall implies the sim/estimator got wedged.
- [x] **Fix decided & implemented (before Phase 2B):**
  - **Reset strategy = full PX4 SITL restart per trial** (`run_trial.py --restart-sim`, now
    the default). Chosen because the stall persisted after load ended → the sim gets into a
    bad state that an in-sim reset would inherit; only a clean process restart guarantees an
    identical, uncontaminated initial condition and independent trials. Strongest answer to a
    reviewer questioning trial independence. `--no-restart` kept as a fallback for quick
    single checks against a hand-launched sim.
  - **Validity guard + auto-retry:** each trial's CSV is checked for circuit completion +
    steady-state; a stalled trial is flagged invalid and re-run (up to `--max-retries`), so
    intermittent stalls never silently pollute the dataset. Verified against the pilot CSVs —
    it flags exactly the 5 confounded trials and passes the 4 clean ones.
  - **Open construct-validity item:** 32 busy-wait threads may starve Gazebo/PX4 itself (not
    just the MPPI compute), possibly causing the stalls and partly confounding "fixed does
    worse." Revisit load level / core-pinning if restart+guard doesn't yield clean tracking.
  - **Note:** the waypoint circuit completes in ~10-15s then hovers at (0,0); with load at
    t+20 the contention lands during hover-hold, not active trajectory-following. Valid test
    of steady-state hold under load; drop `--load-offset` to ~3-5s if active-tracking stress
    is wanted instead.
- [x] **Shakedown of `--restart-sim` — PASSED (2026-07-17, run end-to-end on the dev
  machine).** One full adaptive trial: fresh PX4+Gazebo boot (headless), MAVROS heartbeat
  gate, ground takeoff, full 4/4 waypoint circuit at 20.0 Hz, load window on schedule at
  t+20-50 (mean N 288 in-window vs 392 after), error converged to 0.06 m, validity guard
  passed, clean teardown with zero leftover processes. Three lifecycle bugs found and fixed
  during the shakedown: (1) no pre-flight check — a hand-launched or orphaned sim caused a
  cryptic "PX4 exited during startup" (instance conflict); now detected and cleaned loudly
  with a 5s Ctrl-C grace window; (2) PX4 stdin must be a held-open pipe — /dev/null stdin
  makes pxh busy-spin on EOF (produced a 512MB log in under a minute); (3) the MAVROS
  connection probe via `ros2 topic echo` was unreliable (CLI discovery-cache miss while
  MAVROS was in fact connected) — replaced with the authoritative "Got HEARTBEAT" marker in
  the captured MAVROS log. Ctrl-C/exit now always runs sim cleanup.
- [x] **Fresh 3x3 pilot via `--restart-sim` — RUN 2026-07-17, structurally clean, but load
  found toothless in a clean environment.** All 9 trials valid: ground takeoffs, full 4/4
  circuits, 20.0 Hz throughout, tracking converged (<0.12m end error), 0-2 deadline misses
  per trial in EVERY condition. The 32-thread load raised fixed-400 call time only from
  ~28ms to ~37ms — under the 50ms deadline, so H1 had nothing to compare. Conclusion: the
  old pilot's ~300 misses/trial were an artifact of the degraded long-running sim
  environment, not the load itself. Archived as `results_pilot_unpinned32thr/` — retained
  as the no-deadline-pressure control observation (adaptive: ~25% less compute at equal
  tracking, regulated at exactly 30.0ms = its 0.6x50ms design target; answers RQ3 even
  without misses).

### Stress-condition calibration — DONE, condition PREDETERMINED (2026-07-17)
Root cause of the toothless load: the Linux scheduler (CFS/EEVDF) shields a periodic
control thread from fair-share busy-wait contention whenever core headroom exists — thread
count alone cannot induce deadline pressure on a 16-hw-thread machine. Calibration ladder
(single fixed-N=400 trials, fresh sim each; affinity propagation through `ros2 run`
verified):

| Load config                  | call time in window | in-window misses | flight       |
|------------------------------|---------------------|------------------|--------------|
| 32 threads, unpinned         | ~37ms               | 0%               | valid        |
| 64 threads, unpinned         | ~37ms               | 0%               | valid        |
| 4 cores pinned, 16 threads   | ~36ms               | 0%               | valid        |
| 1 core pinned, 4 threads     | —                   | —                | DESTABILIZED |
| 2 cores pinned, 4 threads    | 43ms mean / 53 peak | 3%               | valid        |
| **2 cores pinned, 6 threads**| 41ms mean / 52 peak | 4%               | valid        |

**Locked stress condition for Phase 2 (now `run_trial.py`'s defaults): controller node +
load generator pinned to 2 cores (`taskset -c 0-1`, emulating an embedded-class compute
envelope, cf. Jetson-class companion computers), 6 busy-wait threads, 30s window at t+20s.**
PX4/Gazebo/MAVROS stay unpinned (conceptually a separate machine). Head-to-head validation
under this exact condition: **fixed-400 → 22 in-window misses (4%), call 40.5ms mean/52.9
peak; adaptive → 0 misses, call regulated at 30.0ms, N 399→301 (min 270), recovery after
window; equal tracking quality; both flights valid.** This also upgrades construct
validity: the pinned envelope directly models the target deployment class instead of
relying on machine-wide oversubscription.
- [x] **Fresh 3x3 pilot under the calibrated condition — ✅ PASSED (2026-07-17).** All 9
  trials valid (ground takeoffs, 4/4 circuits, 20.0 Hz, steady-state reached). Per-condition
  (mean ± std across 3 trials): adaptive **0 misses**, in-window call locked at 30.0ms,
  in-window N ≈ 300-302, ss-RMS 0.108±0.006; const330 0.67±1.15 misses, 32.8ms, ss-RMS
  0.115±0.010; fixed400 **20.0±6.2 misses**, 40.6ms, ss-RMS 0.105±0.008. Even at n=3,
  adaptive-vs-fixed on misses reaches Welch p=0.031; tracking quality statistically
  indistinguishable across conditions (H2 as desired). No scheduler chatter (mean |ΔN| ≈ 2).
  **Honest H4 note:** at this contention level the matched constant budget (330) also
  rarely misses — adaptive's advantage over const330 here is quality headroom (mean N 366 vs
  330: full 400 samples whenever load is absent, at equal deadline safety), not miss count.
  H4's miss-count comparison likely needs the 2B-extended second (heavier) condition, where
  a constant tuned to one contention level becomes insufficient — that is the actual
  argument for adaptation. → **GO for Phase 2B.**

### Phase 2B — Full trial set — ✅ PRIMARY DONE (2026-07-17)
- [x] 13 trials per condition (adaptive / constant N=330 / fixed N=400), all valid, under
  the locked stress condition. **H1 confirmed** (adaptive 0.15±0.38 vs fixed400 18.0±4.1
  misses, Welch & Mann-Whitney p<0.0001). **H2 confirmed** (ss-RMS indistinguishable,
  adaptive 0.131 vs fixed400 0.155, p=0.35). **H3 confirmed** (mean N 365 vs 400; in-window
  ~300 regulated at 30.0ms). Scheduler validation: response 1.4 cycles, recovery 1.6, mean
  |ΔN| ~2 (no chatter).
- [x] **H4 vs const330: statistical tie at 20Hz (0.15 vs 0.23 misses, RMS equal) — accepted
  and reframed, not hidden.** Under the tested condition a well-chosen constant matches
  adaptation. Reframed contribution (per external review — lead with robustness, not
  "discovery"): **adaptive provides robust performance without offline tuning.** The
  constant baseline requires an offline engineer to run a Pareto sweep and stress
  calibration, then hard-code the result for one operating regime; adaptive needs none of
  that and transfers when conditions change. Supporting points: (1) decisive win over the
  conventional fixed-N=400 deployment; (2) parity with the carefully-tuned offline
  baseline exactly where that baseline is expected to perform well; (3) the RMS tie
  *confirms* Phase 0's equivalent-budget interpolation prediction; (4) transfer across
  timing regimes without retuning — see 2B-extended. **Wording caution for the paper:
  never phrase the 40Hz result as "const330 is bad/fails" — the claim is "const330 is
  tuned for one operating regime and does not transfer to a tighter timing regime without
  retuning, whereas adaptive transfers automatically."**
- [x] **Load-saturation finding (report in paper, likely Limitations/Discussion):** at 20Hz
  on this platform, no fair-share load level can push const330 over the deadline — CPU
  contention saturates at ~1.4x per-sample inflation (6/10/mem-thrash threads all
  equivalent; next step, 1-core pin, destabilizes flight). The OS scheduler (CFS/EEVDF)
  structurally shields periodic control tasks from fair-share contention. Combined with the
  Pareto curve's flatness beyond N≈200, **no 20Hz experiment on this platform can separate
  adaptive from const330 in either direction** — motivating the deadline-scarcity condition
  below rather than further load escalation.

### Phase 2B-extended — Tight-deadline condition (pre-registered option) — CALIBRATED, GO
The plan pre-specified "ROS2 latency injection **or reduced loop rate**" as the second
stress condition before any Phase 2 data existed; reduced loop rate is the one executed.
**Framing discipline (per external review): this is a separate, clearly-labeled secondary
experiment about deadline scarcity — same trajectory, same controller code, same load
profile, same everything except the control period (40Hz / 25ms deadline vs 20Hz / 50ms).
Designed as a fair test, reported whatever the outcome. Paper narrative: "the experimental
plan included a second operating regime (reduced control period) to evaluate whether the
conclusions generalize across timing constraints" — NOT "since H4 tied, we ran 40Hz."**
Justification wording (kept deliberately modest — do NOT claim "40Hz on this i7 is
equivalent to 20Hz on a Jetson"; that would require benchmarking the Jetson): *"a reduced
control period represents a tighter real-time constraint; such constraints are expected on
lower-performance embedded processors, motivating evaluation under this additional
operating regime."*
- [x] Node gains a `loop_rate_hz` parameter (deadline = loop period; scheduler budget =
  0.6 x period, unchanged law). `run_trial.py --loop-hz`. Load generator gains a `mem`
  mode (64MB/thread cache-thrash; kept for completeness — saturates like spin at 20Hz).
- [x] **Calibration head-to-head @40Hz, 2-core/6-thread load:** const330 idle-OK (19.3ms,
  3 misses/801) but **100% in-window miss rate** (28.9ms vs 25ms deadline, 1035/1035),
  effective rate degrades to 38.2Hz; adaptive re-regulates to 15.0ms (=0.6x25ms) with
  N 252 idle → 143 in-window, **0 in-window misses**, full 40.0Hz. Both flights valid,
  tracking equal. Adaptive discovered a different budget for a different deadline with
  zero retuning — the cross-regime claim now has data at both regimes.
- [x] **8 trials per condition @40Hz — ✅ DONE (2026-07-18), decisive.** All 24 valid.
  Adaptive 0.75±0.71 misses/trial (call regulated at 14.98ms = 0.6×25ms, mean N 220.5);
  const330 1048.9±5.2 misses (30.5% of all cycles); fixed400 876.5±7.6 (26.9%). Adaptive
  vs both on misses: Welch p<0.0001, MWU p=0.0008. Response latency 1.0 cycle. **Honest
  reporting notes:** const330's tracking stays good (ss-RMS 0.095m, best of three) — the
  cost of misses is the broken real-time contract + degraded effective rate (38.2Hz), not
  SITL tracking accuracy; say exactly that. Adaptive-vs-fixed400 ss-RMS tests disagree
  (Welch p=0.90, MWU p=0.01; n=8 rank artifact) — report both, claim no consistent
  difference. Data: `results_40hz/`, commit 41eb7a2.

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

**N=330 remains locked after the stress-condition recalibration** (§ Stress-condition
calibration). Under the calibrated condition adaptive's in-window average was ~301 and its
idle level ~400, so 330 still sits squarely in the equivalent-budget band; re-deriving it
from new adaptive runs would reintroduce exactly the post-hoc-tuning criticism the lock
exists to prevent. Report adaptive's actual per-trial average alongside, for transparency.

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
