# Compute-Aware Anytime MPPI for UAV Control — Full Project Overview

*Self-contained: a person or AI with zero prior context should be able to read this and fully
understand the goal, system, code, experiments, and results to date.*

---

## 1. Abstract (working draft — will be finalized only after Phase 2 statistical validation)

Real-time control of unmanned aerial vehicles (UAVs) using sampling-based optimal control
methods such as Model Predictive Path Integral (MPPI) control requires selecting a number of
trajectory samples per control cycle. Existing deployments treat this sample count as a fixed,
offline-tuned design parameter. This assumption breaks down under real-world conditions where
available compute varies at runtime. We present an Anytime MPPI framework that treats sample
count as a runtime-adaptive control variable. In an initial stress-test experiment under
sustained synthetic CPU contention, our adaptive scheduler eliminated control-deadline misses
while using approximately 18% less average compute than a fixed-budget baseline, with an
estimated tracking-quality difference smaller than the measurement noise floor. [PLANNED:
statistically validated results across repeated trials, and a constant-reduced-budget control
condition, to confirm whether these effects hold and are attributable to the adaptation
mechanism specifically, not merely to using less compute on average.] [PLANNED: hardware-in-
-the-loop validation via UCL resources.]

---

## 2. Introduction — Motivation and Problem Statement

*(Unchanged from prior draft — see § 2 of the original for full text. Core problem: MPPI sample
count is conventionally a fixed, offline-tuned design choice; real-world compute availability
fluctuates at runtime; this project treats sample count as a runtime-adaptive control variable
instead.)*

---

## 3. Related Work (summary — see references.bib for full entries)

Two bodies of literature are relevant:

**(A) MPPI-for-UAV work.** Enrico, Mancini & Capello (2025, *Applied Sciences*) compare NMPC and
GPU-parallelized MPPI on embedded UAV hardware (Jetson Orin Nano, ROS2, PX4 — closely matching
this project's stack), finding MPPI sample count is constrained by real-time requirements, and
deriving an optimal *fixed* configuration via offline analysis. Similarly, MPPI-based agile drone
racing work notes reduced sample counts (due to onboard compute limits) measurably degrade
tracking, using a fixed sample count chosen to fit hardware constraints. Neither adapts sample
count online.

**(B) Control-scheduling co-design.** Weakly-hard real-time models permit a bounded number of
missed control deadlines without compromising stability; resource-aware/self-triggered MPC
adaptively determines control timing online — but for deterministic/linear control, not
stochastic sampling-based methods.

**To the best of our knowledge**, no existing work combines runtime sample-count adaptation for
MPPI specifically, driven by measured compute availability, with an explicit deadline/safety
mechanism, validated on UAV flight. *(Corrected per review: avoid absolute "no work exists"
claims — always hedge with "to the best of our knowledge.")*

A third, orthogonal thread — adaptive-importance-sampling MPPI — adapts the sampling
*distribution*, not *count*, and is not driven by compute availability.

---

## 4. System Architecture

```
Gazebo (Harmonic) ──▶ PX4 SITL ──▶ MAVROS ──▶ offboard_node ──▶ DroneState
                                                    │
                                                    ▼
                                              Controller (PID / MPC / MPPI)
                                                    │
                                                    ▼
                                        Publish MAVROS velocity setpoint
                                                    │
                                                    ▼
                                          PX4 ──▶ Gazebo (closed loop)
```

Core principle: `offboard_node.cpp` never performs control mathematics — only subscribes,
publishes, and calls controller classes. Stack: Ubuntu 22.04, ROS2 Humble, Gazebo Harmonic,
PX4 SITL, MAVROS (MAVLink — not native uXRCE-DDS; see Limitations), C++17, colcon/CMake, Eigen3.

Development history (Stages 1-6): waypoint publisher → PID → kinematic model → gradient-descent
MPC → baseline MPPI → `AnytimeScheduler`. All stages standalone-validated before integration;
each stage flown in Gazebo before proceeding to the next.

---

## 5. Code Walkthrough

*(Unchanged from prior version for most files — see original § 5 for `drone_state.hpp`,
`math_utils.hpp`, `drone_model.hpp/.cpp`, `pid_controller.hpp/.cpp`, `mpc_controller.hpp/.cpp`,
and the `MppiController` sampling/weighting/averaging mechanics, all of which remain accurate.)*

### `include/mppi_controller/controllers/anytime_scheduler.hpp` / `.cpp`
**Current implementation (as actually coded — important to describe accurately, not
aspirationally):**
```
recommended_N = clamp( (target_loop_time * deadline_margin) / avg_time_per_sample,
                        n_min, n_max )
```
`avg_time_per_sample` is smoothed over a window of the 3 most recent calls.
`consecutive_deadline_misses_ >= 2` triggers `inSafetyFallback()`.

**This is a direct heuristic, not yet the Pareto-informed policy described in earlier drafts of
this document.** *(Correction per review — a prior version of this document described the
scheduler as "select the smallest N satisfying deadline AND an empirical quality threshold,"
which is a **planned refinement**, not the current implementation. That refinement is deferred
to a later Phase 2 ablation condition — see PROJECT_PLAN.md.)*

Current config: `n_min=20, n_max=400, target_loop_time=0.05s, deadline_margin=0.6`.

### `src/offboard_node.cpp`
ROS2 node; 20Hz timer; `use_scheduler` parameter toggles adaptive vs fixed at runtime.

**CSV logging column note:** the communication-timing column was renamed from
`mavros_roundtrip_ms` to **`state_to_command_latency_ms`** for accuracy — it measures wall time
from pose reception to command publication, which is an end-to-end latency measurement, not
strictly an internal MAVROS round-trip.

**Tracking-phase gating — bug fixed and verified.** Root cause: tracking phase was originally
gated only on the takeoff ramp's fixed 3-second timer, not actual PX4 arm+OFFBOARD state,
causing several seconds of "tracking" data to log before the vehicle was actually being
effectively commanded. **Fixed**: now gated on `current_state_.armed && current_state_.mode ==
"OFFBOARD"`, with an explicit log line ("Vehicle is armed and in OFFBOARD. Beginning MPPI
tracking.") confirming the transition. Verified via a new flight log showing tracking-error
convergence beginning promptly after this transition, without the earlier flat-plateau artifact.

---

## 6. Key Results So Far

*(Language hedged per review — these are observations from initial/limited trials, not yet
statistically validated claims. Phase 2 will determine whether they hold across repeated runs.)*

### 6.1 Baseline MPPI tuning (Gazebo, no injected load)
Systematic single-variable tuning (noise_std, lambda, control_weight) resolved most hover
jitter/overshoot observed with default parameters. Minor residual jitter remains and is
expected — MPPI's continuous re-sampling means it does not converge to exactly zero error even
in a noiseless standalone simulation (confirmed in the Stage 5 standalone test).

### 6.2 Initial CPU-load stress-test observation (single trial, timestamp-verified)
32-thread synthetic CPU load, confirmed overlapping a live Gazebo flight via matching wall-clock
timestamps. In this trial: adaptive showed 0 deadline misses during the 30s load window (N
dropping from ~400 idle to a sustained ~310-337 band, recovering after load ended); fixed
(N=400 always) accumulated 5 deadline misses in the same window, with call time spiking to
58.99ms against the 50ms deadline. **This is one trial; Phase 2 will determine whether this
pattern is consistent across repeated runs.**

### 6.3 N-vs-Quality Pareto sweep (10 seeds/N, standalone)
RMS tracking error improved only 13.4% from N=20 to N=400, while compute cost increased ~19x.
Diminishing returns clearly visible beyond N≈150-200 (differences within the sweep's own
measurement noise floor beyond that point).

### 6.4 Equivalent-computational-budget analysis (single trial)
Using the Pareto table to interpolate expected quality at adaptive's measured average N (329.77,
computed exactly from 31 real log samples during the confirmed stress test): the estimated
quality gap versus fixed N=400 was smaller than the Pareto table's own noise floor, while
adaptive used ~18% less average compute and had zero deadline misses versus fixed's five, in
this trial. *(Per review: this observation should be stated once, in Results, with appropriate
hedging — not repeated as a standing headline claim throughout this document.)*

### 6.5 Latest confirmed flight — post gating-fix (most recent, most reliable data to date)
```
Mean N:                       366.5
Mean MPPI call time:          29.19 ms
Mean state-to-command latency: 43.15 ms
Mean position error:          0.5045 m
RMS position error (whole trajectory): 1.0267 m
Deadline misses:               17 (1.6%)
```

**Transient vs. steady-state (important framing, from visual inspection of the tracking-error
plot):** the whole-trajectory RMS of 1.03m is substantially inflated by an initial ~3.4m
transient error during the first ~12-15 seconds — the physical distance between the vehicle's
arming location and the first active MPPI target. After this transient resolves, steady-state
tracking error is visibly under 0.15m. **Steady state is defined algorithmically** (per review
— no hardcoded time cutoff): it begins at the first run of 10 consecutive TRACKING cycles with
position error < 0.5m. `scripts/analyze_trials.py` implements this and computes both
whole-trajectory and steady-state-only RMS for every trial.

**Scheduler responsiveness (qualitative observation — a Scheduler Validation metric, not a
hypothesis; see PROJECT_PLAN.md § Scheduler Validation):** around t≈30s in this log,
MPPI call time spikes toward the 50ms deadline; the scheduler visibly drops N from ~400 toward
~170-280 within 1-2 cycles, call time recovers to ~20-25ms, and N climbs back toward 400 shortly
after. This is a clear qualitative example of the intended adaptive mechanism, though it occurs
here from natural system noise/contention rather than the controlled `cpu_load_generator` —
worth reproducing under controlled load in Phase 2 for a cleaner, attributable example.

### 6.6 State-to-command latency vs MPPI-only compute time
Mean MPPI call time 29.19ms vs mean state-to-command latency 43.15ms — approximately 14ms of
non-algorithmic overhead per cycle (peaks observed up to ~100ms). This directly supports the
Discussion/Limitations point regarding MAVROS/MAVLink communication overhead being distinct
from, and smaller than, but non-negligible relative to, the algorithm's own compute time.
**Known caveat:** this measurement partially conflates real communication latency with the
natural asynchronous gap between MAVROS pose delivery and the 20Hz timer's own cadence — an
upper bound on overhead, not a fully isolated measurement.

---

## 7. Known Limitations (to be stated explicitly in the paper)

1. **MAVROS/MAVLink, not native PX4 uXRCE-DDS.** Addressed via direct measurement of MPPI
   compute time in isolation (§ 6.6), explicit state-to-command latency measurement (§ 6.6), and
   an honest limitations discussion — not a stack migration, given project timeline.
2. **Single-scenario Pareto sweep** (straight-line tracking only) — elbow generalization to
   other trajectory shapes not yet confirmed.
3. **Equivalent-budget analysis (§ 6.4) uses interpolation**, not yet direct measurement of real
   tracking error during an actual stress-test flight with matched conditions.
4. **Kinematic (single-integrator) dynamics model** — no thrust/attitude dynamics.
5. **Gradient-descent MPC, not QP-solver-based** — only relevant if Linear MPC appears as a
   reported comparison baseline (currently it does not).
6. **Scheduler is currently a direct heuristic**, not yet the Pareto-informed refinement
   described as a future direction (§ 5, `anytime_scheduler.hpp` note) — describe accurately as
   implemented, mark refinement as planned/future work.
7. **n=1 statistical basis** for most results to date — Phase 2 (staged repeated trials,
   including a constant-N control condition) is required before these become statistically
   defensible claims. **Do not present § 6 observations as validated findings in any external
   document until Phase 2C statistics exist.**
8. **SITL only, to date.** HITL validation via UCL planned but not started.
9. **No constant-N control condition yet** — current comparisons only span "fully adaptive" vs
   "always N=400," which cannot by itself distinguish "adaptation helps" from "any lower budget
   would have sufficed." Addressed by Phase 2's added constant-N condition, **predetermined at
   N=330** (locked a priori from Phase 0's preliminary adaptive average of 329.77, so the
   baseline does not depend on the Phase 2 results it is compared against — see
   PROJECT_PLAN.md § Constant-N baseline).

### Threats to Validity (new section, per review)
- **Internal validity:** stochastic MPPI sampling introduces inherent run-to-run variance;
  Gazebo physics engine has its own variability; scheduler timing is subject to OS/ROS2
  scheduling jitter on the test machine, independent of the algorithm itself.
- **External validity:** evaluated only on a single quadrotor model and a single
  trajectory-tracking task shape (4-waypoint square); the Pareto sweep uses one scenario; the
  development/test machine's CPU characteristics differ substantially from an embedded
  companion computer (Jetson Nano/Orin) — a gap Phase 4 (HITL) is intended to close.
- **Construct validity:** the synthetic CPU load generator (busy-wait threads) approximates
  compute contention but may not perfectly replicate real contention sources such as a
  concurrent perception pipeline's actual CPU usage pattern, cache pressure, or memory
  bandwidth contention.

---

## 8. Contribution Statement (working draft)

*(Sharpened per review — "we bridge this gap" replaced with an explicit problem formulation.)*

*"While prior work has established that MPPI's sample count is a critical parameter constrained
by available compute, existing UAV deployments treat this as a fixed, offline-tuned design
choice. Separately, real-time control-scheduling co-design has developed principled frameworks
for adapting controller behavior under variable compute, but these target deterministic or
linear control formulations, not sampling-based stochastic optimal control. **We formulate
runtime MPPI sample-count selection as an online computational resource allocation problem,
using measured per-cycle execution time to adapt computational effort while maintaining
closed-loop control deadlines**, with an explicit safety fallback mechanism. [Once the
Pareto-informed refinement is implemented: ...further informed by an empirically characterized
quality-compute trade-off.]"*

---

## 9. Reproducibility Notes

- Repository: `mppi_controller` ROS2 package (private GitHub, tagged commits at major
  milestones).
- Build: `colcon build --packages-select mppi_controller`.
- Fly: `ros2 run mppi_controller offboard_node` (adaptive, default);
  `--ros-args -p use_scheduler:=false` (fixed N=400);
  `--ros-args -p use_scheduler:=false -p fixed_n:=330` (predetermined constant-N control
  condition). The CSV `mode` column records ADAPTIVE / FIXED400 / CONST330 accordingly.
- Pareto sweep: `ros2 run mppi_controller test_n_quality_sweep` (standalone, no ROS2/Gazebo).
- Stress test: run `offboard_node`, then in a separate terminal once flight is stable,
  `ros2 run mppi_controller cpu_load_generator <threads> <duration_sec>`.
- **Automated Phase 2 trials:** `python3 scripts/run_trial.py --condition
  {adaptive|const330|fixed400} --trials <k>` — fires the 32-thread/30s load at a fixed offset
  after tracking begins, collects CSVs into `results/<condition>/`, validates each trial
  (circuit completion + steady-state) and auto-retries stalls. **Default `--restart-sim`
  brings up and tears down a fresh PX4/Gazebo/MAVROS stack per trial** (independent, clean
  initial condition — the Phase 2A fix; requires `--px4-dir`, default `~/PX4-Autopilot`).
  `--no-restart` assumes a hand-launched sim (quick checks only; state can leak).
- **Trial-set analysis:** `python3 scripts/analyze_trials.py results/` — per-trial and
  aggregate stats (whole + steady-state RMS via the algorithmic definition, deadline misses,
  ΔN chatter metrics), Welch/Mann-Whitney tests, scheduler-validation metrics, and aggregate
  figures.
- Logs: `/tmp/mppi_flight_log_*.csv`. Single-run plot: `python3 scripts/plot_flight_log.py
  <csv files...>`.
- CSV columns (current): `epoch_sec, mode, phase, N, mppi_call_ms,
  state_to_command_latency_ms, pos_x, pos_y, pos_z, target_x, target_y, target_z, pos_error_m,
  deadline_miss`.
