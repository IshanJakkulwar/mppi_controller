# Project Plan — Compute-Aware Anytime MPPI for UAV Control
**Target venue — DECISION RECORD (2026-07-19): ECC (deadline late October 2026, user to
verify exact date) over ACC's joint L-CSS option (deadline 6 September 2026).** Reasoning:
L-CSS is a theory-letters journal — the worst-fit reviewer pool for a paper whose stated
contribution is application+rigor with explicitly no mechanism novelty; Sept 6 leaves zero
slack for the Jetson HITL upgrade (the main strong-accept lever) and kills the CEM
checkpoint; the venues are mutually exclusive and L-CSS decisions land after ECC's
deadline, so ACC forfeits ECC 2027 entirely. If a journal-indexed publication is needed,
the planned path is ECC now + an extended journal version (HITL + CEM results) to RA-L —
fit-appropriate for experimental robotics — rather than a high-mismatch L-CSS bet.
(ICRA excluded by user's schedule clashes.)

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
  shields periodic control tasks from fair-share contention in our measurements. Combined with the
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
- [x] **Paper wording for the 40Hz results (locked, per external review):**
  - **Opening sentence of the 40Hz discussion:** *"The constant budget tuned at 20 Hz does
    not transfer to the tighter timing regime — it misses ~30% of all control cycles —
    while the adaptive scheduler re-regulates to the new budget automatically."*
  - **Never write "zero misses."** Write *"essentially zero deadline misses (0.75 ± 0.71
    per trial)"* or *"fewer than one missed deadline per trial on average."* The single
    calibration trial's literal 0/1200 may be cited only as that single trial.
  - **Never write "structurally cannot" (or similar theoretical-impossibility language)
    about the constant baselines.** The demonstrated claim is experimental: *"the constant
    tuned for one operating regime did not transfer to the tighter timing regime without
    retuning."*
  - **The generalization claim, stated precisely:** *"a single fixed budget works when
    tuned for one operating regime but does not generalize across operating regimes
    without retuning"* — not "N=330 is wrong" and not "adaptive beats every baseline."
  - **The three-sentence story arc for the Discussion:** 20 Hz: adaptive ≈ tuned constant
    > untuned constant. 40 Hz: adaptive adapts automatically; the previously tuned
    constant no longer satisfies the timing requirements. Value of adaptation: robustness
    across operating regimes without offline retuning — not outperforming the best
    hand-tuned constant within every fixed regime.

### Post-review addition — Constant-200 arm (pre-registered 2026-07-18, BEFORE data)
Per aggregated external review (M2, "advisor" recommendation): one additional constant
baseline at **N=200** (near the Pareto elbow), 8 trials per regime, run under the
identical locked conditions. Question: *could a much smaller fixed budget have solved
this?* **Interpretation rules locked before any trial runs:**
- If const200 misses deadlines in either regime → strengthens the transfer claim (even
  elbow-level constants embed a regime assumption).
- If const200 survives both regimes with tracking within noise → the empirical advantage
  claims narrow fully to the tuning-free framing: choosing N=200 a priori requires
  exactly the offline Pareto+stress characterization that adaptation replaces; adaptive
  additionally recovers idle-time quality headroom (within Pareto noise) and retains
  graceful degradation beyond tested conditions. **This outcome is published as-is, not
  hidden** — it converges with the M1 reframe (application/methodology paper).
- Either way, "adaptive ≈ const330" gains the missing context: with two constants
  bracketing the adaptive range, in-regime parity is either shared by all reasonable
  constants (supporting the flat-Pareto explanation) or specific to the tuned value.

**OUTCOME (2026-07-19, 8 trials/regime, all valid):** const200 @20Hz: 0.25±0.46 misses,
tracking within noise → in-regime parity is shared by all reasonable constants, as the
flat-Pareto + shielding explanation predicts. const200 @40Hz: **122.3±6.9 misses/trial
(3.5%)** vs adaptive 0.75±0.71 (Welch p<1e-3, MWU Holm-adj 0.0049, LOO-robust) — even the
elbow-level constant breaks at the tighter deadline; failure scales with budget size
(200→3.5%, 330→30.5%, 400→26.9%). First pre-registered branch fired: strengthens the
transfer claim. Integrated into paper Tables I/II and Results text.

### Phase 2C — Statistics & reporting — ✅ COMPLETE (2026-07-18)
- [x] Mean/stddev across trials for all metrics — `scripts/analyze_trials.py`, run on both
  `results/` (20Hz) and `results_40hz/` (25ms); outputs in each set's `analysis/` dir.
- [x] Scheduler stability metrics (mean |ΔN| ~2 at 20Hz / ~2.6 at 40Hz, ΔN distribution
  figure) — no chatter in either regime.
- [x] Scheduler validation metrics: response latency 1.4 cycles (20Hz) / 1.0 (40Hz),
  recovery 1.6 / 4.7 cycles.
- [x] Welch + Mann-Whitney tests, adaptive vs each baseline, misses + ss-RMS, both regimes
  (see Phase 2B / 2B-extended entries for the numbers).
- [x] **Equivalent-budget comparison across the full trial set** —
  `scripts/equivalent_budget.py`, using the Phase 0 Pareto table:
  - 20Hz (13 trials): mean N 365.1 → predicted quality gap **−0.0015** (5% of the table's
    0.0301 noise floor); predicted per-call compute saving 7.6% whole-trial (~22%
    in-window). Observed flight ss-RMS: adaptive 0.131 vs fixed-400 0.155 (n.s.) —
    prediction confirmed.
  - 40Hz (8 trials): mean N 220.5 (in-window ~144) → predicted gap **+0.0082**, still only
    0.27x the noise floor; predicted saving 41.8% whole-trial (~63% in-window). Observed:
    0.136 vs 0.130 (no consistent difference) — prediction confirmed.
  - **Paper point: the offline Pareto characterization correctly predicted flight-quality
    outcomes in both regimes — adaptation operates on the flat region of the
    quality-compute curve, which is precisely why halving the budget cost nothing
    measurable.**
- [x] Aggregate/overlay figures regenerated for both trial sets.

**Actual time: 2 days against the 7–12 budgeted** (automation + staged pilots paid off).

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

## Novelty upgrade — Pareto-informed quality-floor scheduler (ADAPTIVEQF ablation)
**Decided 2026-07-19 after external "improve the novelty?" review. Full context recorded
here deliberately (chat context may not survive).**

### Why this, and why now
External review assessed three novelty-upgrade options: (1) quality-floor-aware scheduler,
(2) second sampler class (CEM) to make the actuator claim class-level, (3) closed-loop
stability theorem. Verdicts: **(1) EXECUTE NOW** — it is the pre-registered Phase 0
refinement ("Pareto-informed refinement… deferred to a later Phase 2 ablation condition",
written before ANY Phase 2 data), so it is not post-hoc novelty-chasing; it is additive
(new ablation arm only — baselines and all existing adaptive data stay untouched and
valid); and it is cheap (~1 day code, ~16 unattended trials), NOT the 1–2 weeks the
external review estimated, because it did not know the infrastructure. It converts paper
contribution 2 from "we characterized a curve that explains our results" to "we fed the
curve back into the controller."
**(2) CEM/second-sampler: CHECKPOINT ITEM — highlighted for later.** The stronger novelty
upgrade in principle (single-instance → class-level actuator claim: "sampling budgets of
stochastic optimizers are a feedback-scheduling actuator class"), but new-controller-from-
scratch risk. Rule: attempt ONLY if QF ablation + HITL both land with ≥3 weeks of runway
remaining; otherwise it is the named resubmission upgrade. Do not start it in parallel.
**(3) Theorem: REJECTED** for this submission — a rushed/hand-wavy proof damages a paper
built on rigor. Only viable with a control-theory co-author driving it.

### The law (implemented in AnytimeScheduler, `n_quality_floor` config; node param
`quality_floor_n`; runner condition `adaptiveqf`; CSV mode label ADAPTIVEQF)
Lexicographic priority: (1) deadline hard, (2) quality floor soft, (3) β-margin best
effort. Concretely: base allocation N = clamp(⌊βT/τ̂⌋, N_min, N_max); if N < N_floor AND
N_floor·τ̂ ≤ T (floor affordable within the FULL period, sacrificing margin not deadline)
→ allocate N_floor; else the deadline law wins (floor yields) and the consecutive-miss
fallback detector is the escape hatch. **N_floor = 150, DERIVED not chosen**: the smallest
sweep N whose predicted quality is within one noise-floor σ of N_max (0.8187 ≤ 0.7893 +
0.0301 = 0.8194; N=100 fails at 0.8393).

### Pre-registered predictions (written BEFORE any ADAPTIVEQF trial ran — same discipline
as the const-200 arm; publish whichever outcome occurs)
- **20 Hz**: floor never binds (deadline law's minimum in-window allocation ≈300 ≫ 150).
  ADAPTIVEQF statistically identical to ADAPTIVE on all metrics. Report straight.
- **40 Hz under load**: floor BINDS — the deadline law sits at N≈143, just below 150; QF
  holds 150; in-window call ≈15.7 ms (above the 15 ms β-budget, still ≪ 25 ms deadline);
  0-ish misses; quality within noise. This is the "quality floor active, deadline still
  satisfied" region — the margin is the resource being spent.
- **True conflict** (deadline wants < floor AND floor breaks the deadline, i.e.
  τ̂ > T/150 ≈ 0.167 ms/sample at 40 Hz): unreachable via natural contention on this
  platform (saturation ~1.4×), reachable via fault injection — under an injected stall the
  affordability test fails, the floor yields, and the detector fires: the complete
  lexicographic story demonstrated end-to-end. Optional 1-2 fault-injection ADAPTIVEQF
  trials to demonstrate (recommended).
- If any prediction is wrong, that is a finding — report it, per house rules.

### Execution
- [x] Law implemented + built; ADAPTIVEQF label smoke-tested at 40 Hz.
- [ ] 1 live smoke trial @40 Hz confirming the floor binds in-window (N pinned at 150).
- [ ] Batches (user, unattended): `run_trial.py --condition adaptiveqf --trials 8` (20 Hz,
  into results/) and `--trials 8 --loop-hz 40 --results-dir results_40hz`; optional
  `--trials 2 --inject-stall 60,60,3 --results-dir results_fault_injection` for the
  conflict demo. Then `analyze_trials.py` on both dirs (ADAPTIVEQF auto-included in
  stats/plots as a baseline vs ADAPTIVE).
- [ ] Paper integration: reshape contribution 2; scheduler section gets the lexicographic
  law; results gain an ablation subsection; conclusion's future-work loses the "quality
  floor" line (now done) and keeps CEM/class-level claim + theorem + HITL as the roadmap.
- Sequencing per external review (agreed): HITL (Jetson+6C) runs in parallel weeks 2-4,
  not sequentially.

## Phase 3 — Literature & Framing — ✅ DRAFT COMPLETE (2026-07-18)

- [x] Initial lit search, gap identified.
- [x] **Full Related Work section drafted** — `paper/related_work.md`. Three threads:
  (A) MPPI on aerial/embedded platforms (Williams ICRA'16 / T-RO'18 foundations; Minařík
  et al. IROS'24 first onboard-GPU MPPI flight; Enrico/Mancini/Capello Applied Sciences'25
  — NMPC-vs-MPPI on Jetson Orin Nano with ROS2/PX4, fixed config derived offline);
  (B) resource-aware control (anytime control: Fontanelli/Greco/Bicchi HSCC'08,
  Quevedo/Gupta TAC'13, Pant et al. TCST'21 anytime-estimation co-design; event/self-
  triggered MPC: Heemels CDC'12, Gommans/Heemels SCL'15; weakly-hard: Bernat TC'01,
  Maggio ECRTS'20 stability under consecutive misses);
  (C) adaptive-importance-sampling MPPI (Asmar et al. ICRA'23) — adapts distribution,
  not count, not compute-driven.
- [x] **References verified against primary sources** — `paper/references.bib` created
  (was listed as "started" but never committed; built from scratch). Notable: "Enrico,
  Mancini & Capello" confirmed as real surnames (Enrico is the first author's surname).
  Pant et al. TCST'21 author list verified from the paper PDF itself. Remaining
  TODO-pages/TODO-verify flags recorded inside the .bib (Minařík + Asmar page numbers,
  Maggio co-author spellings, Enrico first names).
- [x] **Language correction applied throughout:** "to the best of our knowledge" framing,
  no absolute novelty claims.
- [x] **Framing finalized against Phase 2 data:** positioning paragraph ties the threads
  to the measured results (Pareto-predicted quality across regimes; deadline adherence as
  the weakly-hard-motivated metric where adaptation shows value; safety fallback motivated
  by consecutive-miss models).

**Actual time: <1 day.**

---

## Phase 4 — HITL via UCL — PLANNED IN FULL, NOT STARTED
**Complete execution plan: `HITL_PLAN.md`** (written 2026-07-18, before starting) — board
triage for the reportedly-dud Pixhawk 6C, wiring (3-wire FTDI on TELEM2 + USB HIL link),
firmware/params, Gazebo Classic HITL (new Gazebo/Harmonic does NOT support HITL — key
constraint), jMAVSim smoke-test path, MAVROS serial vs UDP trade-off, `--hitl` runner
changes, confirmatory experiment schedule (2x pilot → 5x/condition @20Hz, optional 3x
@40Hz), Pixhawk 2.4.8/FMUv2-vs-v3 fallback analysis, pre-answered gotchas.
**Estimated time: 3.5–4.5 days** (down from 10–16 via automation reuse). Confirmatory
scope; SITL-only submission fallback stays live by design. Blocked on: UCL access + 6C
triage outcome.

---

## Phase 5 — Writing — ✅ FULL FIRST DRAFT (2026-07-18): `paper/main.tex`

Complete IEEEtran draft with real Phase-2 numbers throughout: Abstract (hedged, SITL scope
explicit, robustness-without-retuning lead), Introduction (fixed-N-exists paragraph, 4
contribution bullets incl. the honest negative result), Related Work (from
related_work.md, now 9 MPPI citations after landscape additions), Method (MPPI
formulation, Pareto characterization, scheduler ratio law Eq. 4 + safety fallback,
implementation), Experimental Setup (conditions with predetermination rationale, calibrated
stress condition + saturation ladder, pre-registered regimes, protocol/metrics/tests),
Results (Tables I/II with locked wording: "essentially zero (0.75±0.71)", 40Hz opening
sentence verbatim, Welch+MWU disagreement reported), Discussion (three-sentence arc,
why-tuned-constant-is-hard-to-beat with both axes, cost-of-a-miss honesty, composability),
Threats to Validity, Conclusion. TODOs marked inline: author block, 3 figures (Pareto,
20Hz overlay, 40Hz overlay, ΔN distribution), bib page numbers.

Remaining for submission:
- [x] Figures — DONE (2026-07-19): `scripts/paper_figures.py` → `paper/figures/` (Pareto,
  20Hz overlay, 40Hz overlay, ΔN distribution, fault-injection episode), wired into
  main.tex, visually inspected.
- [ ] Author block / acknowledgments (user-only: affiliation, email, co-authors).
- [x] Bib flags — DONE except one: all pages/authors verified against primary sources
  (Minařík 13144-13151, Asmar 3182-3188, log-MPPI 10240-10247, Shield-MPPI 8(11):7106-7113,
  Maggio confirmed from proceedings PDF, Tsallis authors from RSS site). Remaining:
  williams2018robust (Tube-MPPI RSS'18) co-author tail — verify on Overleaf day.
- [ ] ECC page-limit pass + LaTeX compile (no TeX + no sudo on dev machine — first compile
  on Overleaf; static validation passed: envs/braces balanced, all refs/cites resolve,
  all figures present, only remaining \todo is the author block).
- [ ] Advisor read-through (Phase 6).

### Original Phase 5 checklist (superseded by the draft above)

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
