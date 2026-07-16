# Project Plan — Compute-Aware Anytime MPPI for UAV Control
**Target: ICRA submission. Timeline: ~7 weeks remaining from this point.**

---

## Phase 0 — Sharpen the Contribution — ✅ CORE DONE

**Goal:** move from "heuristic that seemed to work" to a defensible, principled contribution.

### Completed
- [x] `test_n_quality_sweep.cpp`: multi-seed (10 seeds/N) sweep of N ∈ {20,50,100,150,200,250,300,350,400}
  against a fixed straight-line tracking scenario (start (0,0,5), target (2,2,5)).
  - **Result:** quality (MeanRMS) improves from 0.9115m (N=20) to 0.7893m (N=400) — a 13.4%
    improvement for a 20x increase in N and ~19x increase in cost.
  - **Elbow identified around N≈150–200.** Beyond N≈200, RMS changes are within the
    measurement's own stddev (noise floor), i.e. statistically indistinguishable.
  - Cost (`AvgCallTime_ms`) scales linearly with N (~0.055ms/sample).
- [x] `analyze_equivalent_budget.cpp`: cross-referenced real stress-test log against the Pareto table.
  - Adaptive scheduler averaged N=329.77 during a 30s CPU-load window (computed exactly from
    31 real log samples, not estimated).
  - Estimated RMS at that N (interpolated): 0.7916m, vs Fixed N=400's 0.7893m — a difference
    of 0.0023m, an order of magnitude smaller than the table's own noise floor (~0.02–0.03m).
  - **Headline finding:** adaptive achieved statistically indistinguishable quality using ~17.6%
    less average compute, with 0 deadline misses vs fixed's 5, during the same load window.
    This directly refutes "adaptive just used more compute."
- [x] Literature check completed (see PROJECT_OVERVIEW.md § Related Work / references.bib).
  Confirmed genuine gap: no existing work combines (a) MPPI specifically, (b) runtime/online
  adaptation of sample count, (c) driven by measured compute availability, (d) with an explicit
  deadline/safety mechanism, (e) validated on real or SITL UAV flight.
- [x] `references.bib` created with confirmed + flagged-for-verification entries.

### Remaining Phase 0 work
- [ ] **Write the formal contribution statement** (1 paragraph) explicitly bridging the two
  literatures identified: MPPI-for-UAV work (treats N as fixed/offline-tuned — cite Enrico et al.
  2025) and control-scheduling co-design (adapts online, but for deterministic/linear control,
  not stochastic sampling-based control — cite weakly-hard real-time survey, self-triggered MPC).
- [ ] **Reformulate the scheduler's justification** using the Pareto data: restate the policy as
  "select the smallest N that (a) fits within the deadline given current measured per-sample
  cost, and (b) does not fall below the empirically-identified quality elbow (~N=150–200)."
  This replaces "N = budget / time_per_sample" as a bare heuristic with a stated two-part
  criterion grounded in data.
- [ ] Reconsider scheduler config bounds in light of the elbow: current `n_min=20` may be too
  low (real quality loss below ~100–150); current `n_max=400` may be higher than useful
  (negligible gain above ~200–250). Decide whether to retune before Phase 2 reruns.
- [ ] Verify author lists for the 5 "NEEDS VERIFICATION" entries in references.bib via Google
  Scholar/arXiv before they're used in the actual paper.

**Estimated remaining time: 1–2 days.**

---

## Phase 1 — Infrastructure — ✅ MOSTLY DONE

### Completed
- [x] CSV logging added to `offboard_node.cpp` (epoch_sec, mode, phase, N, mppi_call_ms,
  mavros_roundtrip_ms, position, target, pos_error_m, deadline_miss). Writes to
  `/tmp/mppi_flight_log_<timestamp>.csv`.
- [x] MAVROS round-trip timer added (`last_pose_received_time_` → `publishVelocity`), isolating
  communication overhead from MPPI-only compute time.
  - **First real measurement:** Mean MPPI call time 29.60ms vs Mean MAVROS round-trip 45.31ms
    — roughly 15ms of non-MPPI overhead per cycle. This is the data point needed for the
    Discussion/Limitations paragraph addressing the MAVROS-vs-native-DDS critique.
  - **Known caveat to state honestly in the paper:** round-trip is measured as "time since last
    pose arrival to next publish," which partially conflates real communication latency with
    the natural asynchronous gap between MAVROS pose delivery and the 20Hz timer's own cadence.
    It's an upper bound on overhead, not a purified isolated measurement.
- [x] `plot_flight_log.py`: reads CSV(s), produces 4-panel plot (N over time, MPPI call time
  with 50ms deadline line, MAVROS round-trip, tracking error), prints summary stats
  (mean N, mean call time, mean round-trip, mean/RMS error, deadline miss count/rate).
  Supports multi-file overlay for adaptive-vs-fixed comparison.
- [x] **Bug found and fixed:** tracking-phase logging/waypoint advancement was gated only on the
  3-second takeoff ramp timer, not on actual PX4 arm+OFFBOARD state. This caused ~9-10 seconds
  of "TRACKING" phase data to be logged while the vehicle wasn't yet actually being commanded
  effectively (visible as a flat ~4.5m error plateau in the first real flight plot). **User has
  fixed this** — tracking phase (and CSV logging of it) now gates on `current_state_.armed &&
  current_state_.mode == "OFFBOARD"`, not just the ramp timer.

### Remaining Phase 1 work
- [ ] **Re-verify the fix**: rerun a flight, confirm the tracking-error plot no longer shows a
  flat plateau before dropping — error should start decreasing shortly after arm+OFFBOARD
  actually engage, not ~9s later.
- [ ] **Build the automated run script** (`run_trial.sh` or Python equivalent): launches
  Gazebo+PX4+MAVROS+offboard_node (with configurable `use_scheduler` param)
  +cpu_load_generator in the correct sequence, waits for completion, saves CSV with a
  descriptive filename (e.g. `trial_adaptive_load32_run03.csv`), and exits cleanly. This is
  the single biggest unlock for Phase 2 — without it, 10-20 repeated manual trials per
  condition is unrealistic given the timeline.
- [ ] Add basic crash/hang detection to the run script (timeout + process check) since SITL
  stacks are known to occasionally hang or crash silently during long automated sessions.

**Estimated remaining time: 2–3 days** (automated run script is the main remaining effort).

---

## Phase 2 — SITL Statistical Validation — NOT STARTED

**Goal:** convert the single confirmed stress-test result into statistically defensible evidence.

- [ ] Run **adaptive vs fixed, 10–20x each**, same load profile (32-thread, 30s), using the
  automated run script from Phase 1. Save all CSVs with clear naming.
- [ ] Compute **mean/stddev across trials** for: deadline miss count, RMS tracking error, mean N
  (adaptive only), mean MPPI call time.
- [ ] **Report performance at equivalent average computational budget where possible** (per
  reviewer-critique addition) — for each adaptive trial, compute its actual average N, then
  compare its tracking error against fixed-baseline trials, and/or against the interpolated
  Pareto-table quality at that N, exactly as done once already in Phase 0 but now across many
  trials with statistics rather than a single run.
- [ ] **Run the ablation**: implement at least one alternative/naive adaptation rule (e.g. a
  fixed linear ramp of N based on elapsed time, or a simple threshold on/off rule with no
  proportional response) and run the same trial count against it. This directly answers "would
  any adaptation have worked, or does the specific policy matter?"
- [ ] **Add a second stress condition** beyond CPU load — either ROS2 topic-delivery latency
  injection or an artificially reduced control loop rate — and repeat the adaptive-vs-fixed
  comparison under that condition too.
- [ ] Sanity-check reproducibility: confirm the core result (adaptive holds deadline misses low,
  fixed's misses climb under load) is consistent across the trial set, not a lucky single run.
- [ ] Regenerate all plots from Phase 1's script using the full trial set (aggregate/overlay
  plots, not just single-run plots).

**Estimated time: 7–12 days.** This is the highest-risk phase for schedule slip — automated
repeated Gazebo/PX4/MAVROS runs commonly surface flaky crashes, hangs, or nondeterministic SITL
behavior not seen in single manual runs. Budget slack here specifically.

---

## Phase 3 — Literature & Framing — PARTIALLY DONE (runs parallel to Phase 0/2)

- [x] Initial lit search completed, gap identified, `references.bib` started.
- [ ] Full Related Work section written (not just the bib file) — organize into: (a) MPPI/UAV
  work treating N as fixed, (b) control-scheduling co-design for deterministic control, (c)
  adaptive-importance-sampling MPPI variants (different axis of adaptation, worth
  distinguishing explicitly).
- [ ] Finalize paper title/framing based on what Phase 2's data actually shows (don't lock this
  in until real statistical results exist).

**Estimated time: 2–3 days dedicated, spread across Phase 0/2 timeline.**

---

## Phase 4 — HITL via UCL — NOT STARTED

- [ ] Confirm exactly what UCL's rig provides: real Pixhawk + real companion computer (Jetson
  Nano/Orin) vs simulated vehicle physics; confirm whether it uses MAVROS/MAVLink or native
  PX4 uXRCE-DDS (relevant to the MAVROS-overhead discussion — if it's native DDS, that
  partially validates results on a cleaner communication path "for free").
- [ ] Port `offboard_node` to the HITL setup. Expect topic names / QoS profiles to need
  adjustment depending on rig configuration.
- [ ] **Retune scheduler bounds** (`n_min`, `n_max`, `deadline_margin`) for the HITL platform's
  actual compute budget — a Jetson Nano/Orin will have a very different baseline call-time
  profile than the development laptop; do NOT assume the SITL-tuned values transfer directly.
  Consider re-running a small version of the Phase 0 Pareto sweep on the actual HITL compute
  hardware if time allows, since T(N) will differ substantially from laptop measurements.
- [ ] Re-run the adaptive-vs-fixed comparison on HITL. **Decision point:** if time is short,
  treat this as confirmatory (fewer trials, e.g. 3-5x) rather than requiring full Phase 2-level
  statistical rigor — the SITL results remain the primary evidence base either way.

**Estimated time: 10–16 days.** Highest schedule risk in the entire plan — access logistics,
unfamiliar hardware, and first-contact hardware issues routinely eat time even when nothing
goes conceptually wrong.

**Fallback plan (decide once Phase 2 is complete and remaining time is known):**
1. Full scope but HITL results thin/exploratory rather than fully statistically validated, or
2. Drop HITL from this submission entirely, submit SITL-only, treat HITL as a follow-up/journal
   extension (a normal, accepted pattern: SITL-validated conference paper → hardware-validated
   journal extension).

---

## Phase 5 — Writing — NOT STARTED (Abstract/Intro can start early, see below)

- [ ] Method section (largely reusable from existing architecture description in
  PROJECT_OVERVIEW.md + the sharpened Phase 0 formulation).
- [ ] Related Work (from Phase 3).
- [ ] Experimental Design (finalized once Phase 2/4 actually run, not written speculatively).
- [ ] Results (figures generated from Phase 2 + Phase 4 CSV data via `plot_flight_log.py` or
  extensions of it).
- [ ] Discussion/Limitations (MAVROS overhead measurement, SITL-vs-HITL scope, single-scenario
  Pareto sweep caveat, interpolation-based equivalent-budget analysis caveat — all already
  identified honestly above).
- [ ] Abstract/Intro — **draft early** (see note below), finalize last once actual results are
  final.

**Estimated time: 7–10 days** for a full first draft, assuming Abstract/Intro drafted early.

---

## Phase 6 — Polish — NOT STARTED

- [ ] Advisor/mentor review pass.
- [ ] Figure formatting to ICRA spec, page limit compliance.
- [ ] Proofreading, LaTeX template conformance, submission logistics (author info, supplementary
  material if applicable).

**Estimated time: 4–6 days.**

---

## Overall Timeline Reality Check

Sum of realistic-case phase estimates: **~5.5–8.5 weeks** against **~7 weeks remaining**. This
plan fits only in the optimistic-to-middle case and has little slack if Phase 2 or Phase 4 runs
long — which are exactly the phases most prone to real-world delay (flaky repeated simulation
runs, hardware access/logistics). Keep both Phase 4 fallback options genuinely live; make the
call concretely once Phase 2 is done and actual remaining runway is known, not before.

## Immediate Next Actions (in order)
1. Re-verify the tracking-phase gating fix with a fresh flight + plot.
2. Build the automated run script (unlocks all of Phase 2).
3. Write the Phase 0 contribution statement + reformulated scheduler justification.
4. Begin Phase 2 statistical trials.
5. Start drafting Abstract/Intro in parallel (see PROJECT_OVERVIEW.md as the source document).
