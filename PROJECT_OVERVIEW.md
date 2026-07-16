# Compute-Aware Anytime MPPI for UAV Control — Full Project Overview

*This document is written to be self-contained: a person or AI with zero prior context on this
project should be able to read this and fully understand the goal, the system, the code, the
experiments run so far, the results, and what remains to be done.*

---

## 1. Abstract (working draft)

Real-time control of unmanned aerial vehicles (UAVs) using sampling-based optimal control
methods such as Model Predictive Path Integral (MPPI) control requires selecting a number of
trajectory samples per control cycle. Existing deployments treat this sample count as a fixed,
offline-tuned design parameter, chosen once to fit a target platform's worst-case compute
budget. This assumption breaks down under real-world conditions where available compute varies
at runtime — due to concurrent processes, OS scheduling, or shared hardware contention — leading
either to wasted compute (when the fixed budget is conservative) or missed control deadlines
(when it is not). We present an Anytime MPPI framework that treats sample count as a
runtime-adaptive control variable, informed by an empirically characterized trade-off between
sample count, tracking quality, and computation time. Our scheduler selects, at each control
cycle, the smallest sample count that both fits within a measured real-time deadline and remains
above an empirically identified quality threshold, rather than applying an unprincipled
proportional heuristic. We validate the approach in a ROS2/PX4/Gazebo software-in-the-loop UAV
simulation under injected CPU contention, and [PLANNED: on hardware-in-the-loop hardware via UCL
resources]. Under sustained synthetic compute load, our adaptive scheduler maintained zero
control-deadline misses while using approximately 18% less average compute than a fixed-budget
baseline, with no statistically significant difference in tracking accuracy.

*(This abstract will be finalized only after Phase 2/4 results are complete — the numbers above
reflect the single confirmed stress-test run to date, not yet a statistically validated result
across multiple trials.)*

---

## 2. Introduction — Motivation and Problem Statement

Autonomous UAV control increasingly relies on computationally intensive trajectory optimization.
MPPI is a popular choice: it handles nonlinear dynamics and non-convex costs by sampling many
candidate control sequences, evaluating each against a cost function, and combining them into a
weighted-average control action. Its quality scales with the number of samples drawn per control
cycle — more samples generally means a better-informed control decision, at proportionally
higher compute cost.

In practice, MPPI is deployed on embedded compute (e.g. Jetson Nano/Orin) with a fixed control
loop frequency (commonly 20-50Hz for UAVs). The number of samples is chosen once, offline, to fit
the worst-case available compute time per cycle. This is the dominant approach in current
literature and practice (see Related Work). The problem: **actual available compute at runtime
is not constant.** Concurrent ROS2 nodes, perception pipelines, OS scheduling noise, and shared
hardware contention all cause the real per-cycle compute budget to fluctuate. A fixed sample
count is therefore either wastefully conservative (most of the time) or a latent risk of missing
the real-time control deadline (whenever contention spikes) — with no mechanism to detect or
respond to either condition.

**This project's core idea:** treat the MPPI sample count as a first-class, runtime-adaptive
control variable — informed by (a) a live estimate of current per-sample compute cost, and (b) an
empirically characterized understanding of how sample count trades off against tracking quality
— rather than a static offline choice.

---

## 3. Related Work (summary — see references.bib for full entries)

Two bodies of literature are relevant, and the gap this project addresses sits between them:

**(A) MPPI-for-UAV work.** Recent work (Enrico, Mancini & Capello, 2025, *Applied Sciences*)
directly compares NMPC and GPU-parallelized MPPI on embedded UAV hardware (Jetson Orin Nano,
ROS2, PX4 — nearly identical stack to this project), explicitly finding that MPPI sample count is
constrained by the need to meet real-time control frequency, and derives an optimal *fixed*
configuration (800-1250 samples) via offline systematic parameter analysis. Similarly, work on
MPPI for agile drone racing notes that reduced sample counts (necessitated by onboard compute
limits) measurably degrade tracking performance, and again uses a fixed sample count (M=2048)
chosen to fit hardware constraints. **Neither adapts sample count online.**

**(B) Control-scheduling co-design.** A separate, mature literature addresses controller behavior
under variable compute/scheduling conditions — "weakly-hard real-time" models that permit a
bounded number of missed control deadlines without compromising closed-loop stability, and
resource-aware/self-triggered MPC approaches that adaptively determine control timing online.
This work is principled and directly relevant to *how* to reason about deadline-bounded
adaptation — but targets deterministic/linear control formulations, not stochastic
sampling-based methods like MPPI.

**The gap:** no identified work combines runtime sample-count adaptation, driven by measured
compute availability, with an explicit deadline/safety mechanism, for MPPI specifically,
validated on UAV flight (SITL or hardware). This project sits at the intersection of (A) and (B).

A third, orthogonal thread — adaptive-importance-sampling MPPI (e.g. Asmar et al.) — adapts the
*sampling distribution* (proposal mean/covariance) rather than sample *count*, and is not
driven by compute availability; worth citing as a related-but-different axis of adaptation.

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

**Core design principle (unchanged since project start):** the ROS2 node
(`offboard_node.cpp`) never performs control mathematics itself. It only subscribes to state,
publishes setpoints, and calls controller classes. All optimization logic lives in
`include/mppi_controller/controllers/`.

**Stack:** Ubuntu 22.04, ROS2 Humble, Gazebo Harmonic, PX4 SITL, MAVROS (MAVLink-based — not
native PX4 uXRCE-DDS; see Limitations), C++17, colcon/CMake, Eigen3 for linear algebra.

**Development history (Stages 1-6, all completed and validated):**
1. Waypoint publisher (raw position setpoints) — flew a 4-waypoint square in Gazebo.
2. PID controller (velocity commands) + smooth takeoff ramp — flew cleanly.
3. `DroneModel`: simple kinematic (single-integrator) prediction model, standalone-verified.
4. `LinearMpcController`: gradient-descent-based MPC (deliberate choice over a QP solver —
   see Limitations), standalone-verified (exact convergence to target).
5. `MppiController`: baseline MPPI, standalone-verified, then flown in Gazebo. Required real
   tuning (see § Tuning History) to reduce jitter/overshoot from default parameters.
6. `AnytimeScheduler`: compute-aware sample-count adaptation, standalone-verified, then flown
   in Gazebo, then validated under genuine injected CPU contention (see § Key Results).

---

## 5. Code Walkthrough (file by file)

### `include/mppi_controller/models/drone_state.hpp`
Plain struct: `position`, `velocity`, `orientation` (quaternion), `angular_velocity`,
`timestamp`. No behavior — a stable data-passing interface used by every controller. Note:
`velocity`/`angular_velocity` are currently never populated from real sensor data (only
position/orientation come from `/mavros/local_position/pose`); this is a known gap, not a bug.

### `include/mppi_controller/utils/math_utils.hpp`
- `clamp<T>(value, min, max)`: generic scalar clamp, templated so it works for any numeric type.
- `clampVector(v, max_norm)`: caps a vector's magnitude while preserving direction (used for
  velocity/control saturation — critical that it doesn't naively clamp each axis independently,
  which would distort direction).
- `GaussianSampler`: wraps `std::mt19937` (Mersenne Twister PRNG) + `std::normal_distribution`.
  `sample(std_dev)` draws one N(0,1) value per axis and scales each by the configured
  `std_dev` — this is the noise-injection mechanism MPPI uses to generate randomized candidate
  trajectories every control cycle.

### `include/mppi_controller/models/drone_model.hpp` / `.cpp`
`DroneModel::predict(state, control, dt)`: single-integrator kinematic model —
`position += velocity * dt`, yaw integrated from `yaw_rate`, velocity assumed to track the
commanded value instantly (no thrust/attitude dynamics modeled yet — a known, documented
simplification). Pure function (no internal state) — required so it can be called repeatedly
inside MPPI/MPC rollouts without side effects.

### `include/mppi_controller/controllers/pid_controller.hpp` / `.cpp`
Standard PID producing velocity commands from position error. `ki` is deliberately zero (never
tuned up — P and D alone were sufficient for the PID-stage validation). `reset()` clears
integral/derivative history to prevent windup carrying across waypoint transitions.

### `include/mppi_controller/controllers/mpc_controller.hpp` / `.cpp`
`LinearMpcController`: solves a horizon-based tracking problem via **projected gradient
descent with warm-starting**, not a QP solver (see Limitations for justification). Standalone
Stage 4 test confirmed exact convergence. **Not currently used in the live flight node** — it
was a validation stepping-stone toward MPPI, not part of the adaptive-vs-fixed comparison.

### `include/mppi_controller/controllers/mppi_controller.hpp` / `.cpp`
The core controller. `computeVelocityCommandWithN(state, target, N)`:
1. **Sample:** generate N randomly perturbed control sequences by adding Gaussian noise
   (`GaussianSampler`) to the current nominal sequence, clamped to `max_speed`.
2. **Evaluate:** roll each sequence forward through `DroneModel::predict`, accumulate a cost
   = quadratic tracking error (`state_weight`) + quadratic control effort (`control_weight`) at
   each step, plus a terminal cost (`terminal_weight`) on the final predicted position.
3. **Weight & average:** compute softmax weights `exp(-(cost - min_cost)/lambda)`, normalize,
   and take the weighted average of all N sampled sequences as the new nominal sequence.
4. Return the first control of the new nominal sequence (receding horizon); shift the buffer
   for warm-starting next call.

`computeVelocityCommand()` (fixed N from config) vs `computeVelocityCommandWithN()` (N
overridden per-call) — the latter is what `AnytimeScheduler` drives.

**Tuned parameters (current, validated by real flight):**
```
noise_std      = (0.5, 0.5, 0.5)   [default was 1.0 — reduced hover jitter]
lambda         = 3.0               [default was 1.0 — reduced trembling-at-rest]
control_weight = (0.3, 0.3, 0.3)   [default was 0.1 — reduced overshoot/correction]
state_weight    = (10.0, 10.0, 10.0)
terminal_weight = (20.0, 20.0, 20.0)
horizon = 20, dt = 0.1, max_speed = 2.0
```

### `include/mppi_controller/controllers/anytime_scheduler.hpp` / `.cpp`
`AnytimeScheduler::recommendNextSampleCount(last_call_duration, last_call_N)`:
- Maintains a smoothed history (deque, window=3) of *time-per-sample* (duration/N) from
  recent calls.
- Computes `recommended_N = (target_loop_time * deadline_margin) / avg_time_per_sample`,
  clamped to `[n_min, n_max]`.
- Tracks `consecutive_deadline_misses_`; `inSafetyFallback()` returns true after 2 consecutive
  misses (threshold of 2, not 1, to avoid flapping on a single noisy measurement).

**Current config:** `n_min=20, n_max=400, target_loop_time=0.05s (20Hz), deadline_margin=0.6`
(tightened from an initial 0.8 during testing — see § Key Results for why).

**Known limitation (Phase 0 in progress):** the core formula is presently a bare heuristic
(division), not yet reformulated against the Pareto-curve elbow finding (~N=150-200) — this
reformulation is the main remaining Phase 0 task.

### `src/offboard_node.cpp`
The ROS2 node. Subscribes to `/mavros/state`, `/mavros/local_position/pose`; publishes to
`/mavros/setpoint_velocity/cmd_vel`; uses service clients for arm/OFFBOARD-mode requests
(auto-triggered after ~1s of streamed setpoints). Runs a 20Hz (`50ms`) wall timer. Contains a
3-second proportional takeoff ramp (climbs straight up before handing control to MPPI/scheduler)
separate from the main tracking controller. A `use_scheduler` ROS2 parameter toggles between
adaptive (`AnytimeScheduler`-driven N) and fixed (`N=400` always) modes at runtime, without
recompilation — used for the adaptive-vs-fixed comparison experiments.

**CSV logging (Phase 1):** every control cycle writes a row to `/tmp/mppi_flight_log_*.csv`
(timestamp, mode, phase, N, MPPI call time, MAVROS round-trip time, position, target, tracking
error, deadline-miss flag). A separate timer isolates MAVROS communication round-trip
(pose-received → command-published) from MPPI's own compute time, to support the
middleware-overhead discussion (see Limitations).

**Bug fixed (recent):** tracking-phase logic was originally gated only on the takeoff ramp's
fixed 3-second timer, not on actual PX4 arm+OFFBOARD engagement — causing several seconds of
"tracking" data to be logged while the vehicle wasn't yet being effectively commanded. Now
gated on `current_state_.armed && current_state_.mode == "OFFBOARD"`.

### Standalone test executables
`test_drone_model.cpp`, `test_mpc_controller.cpp`, `test_mppi_controller.cpp`,
`test_anytime_mppi.cpp` — each validates one controller in isolation (no ROS2/Gazebo), using a
simulated closed loop against `DroneModel` directly. `test_n_quality_sweep.cpp` — the Phase 0
Pareto-curve generator (see § Key Results). `cpu_load_generator.cpp` — standalone tool that
spins N busy-wait threads for a configurable duration, used to inject genuine CPU contention
during live flight tests (separate process/terminal from `offboard_node`).
`analyze_equivalent_budget.cpp` — offline tool that interpolates the Pareto table against real
measured average-N from a stress test, for the equivalent-computational-budget comparison.

---

## 6. Key Results So Far

### 6.1 Baseline MPPI tuning (Gazebo, no injected load)
Default parameters produced visible jitter, hover trembling, and overshoot/correction
oscillation. Systematic single-variable tuning (see § Code Walkthrough for final values)
resolved hover trembling almost entirely and substantially reduced overshoot, leaving only
minor residual jitter — expected and inherent to MPPI's continuous re-sampling (confirmed by
Stage 5's noiseless standalone test, which also never converges to exactly zero error).

### 6.2 First confirmed CPU-load stress test (single trial, timestamp-verified)
32-thread synthetic CPU load injected via `cpu_load_generator`, overlapping a live Gazebo
flight, confirmed via matching wall-clock timestamps (not just epoch inference):
- **Adaptive:** 0 deadline misses during the 30s load window; N dropped from ~400 (idle) to a
  sustained ~310-337 band during load, recovering to ~400 immediately after load ended.
- **Fixed (N=400 always):** 5 deadline misses accumulated during/after the same load window;
  call time spiked as high as 58.99ms (vs the 50ms deadline) with no adaptive response.

### 6.3 N-vs-Quality Pareto sweep (10 seeds/N, standalone)
See Phase 0 in PROJECT_PLAN.md for the full table. Headline: RMS tracking error improves only
13.4% from N=20 to N=400, while compute cost increases ~19x — clear diminishing returns beyond
N≈150-200.

### 6.4 Equivalent-computational-budget analysis
Using the Pareto table to interpolate expected quality at adaptive's *actual measured* average N
(329.77, computed exactly from 31 real log samples during the confirmed stress test): estimated
RMS 0.7916m vs fixed's 0.7893m — a difference of 0.0023m, far smaller than the table's own
noise floor (~0.02-0.03m stddev). **Adaptive matched fixed's quality using ~18% less average
compute, while also avoiding all 5 of fixed's deadline misses.** This is the strongest single
result obtained to date and directly answers "adaptive just used more compute."

### 6.5 MAVROS round-trip vs MPPI-only timing (first real measurement, Phase 1)
Mean MPPI call time 29.60ms vs Mean MAVROS round-trip 45.31ms — approximately 15ms of
non-algorithmic overhead per cycle in the current live-flight measurement. Caveat: round-trip
measurement partially conflates real communication latency with natural pose-arrival/timer-tick
asynchrony (documented limitation, not yet fully isolated).

---

## 7. Known Limitations (to be stated explicitly in the paper)

1. **MAVROS/MAVLink, not native PX4 uXRCE-DDS.** Since the paper's central claim involves
   latency/timing, this is a legitimate methodological point a reviewer may raise. Addressed by
   (a) measuring MPPI compute time in isolation from communication overhead (already done), (b)
   measuring MAVROS round-trip time explicitly (done, § 6.5), and (c) an honest limitations
   paragraph rather than a stack migration (not planned given timeline).
2. **Single-scenario Pareto sweep.** The N-vs-quality characterization uses one straight-line
   tracking scenario; the elbow location (~N=150-200) has not yet been confirmed to generalize
   to other trajectory shapes (e.g. turns, multi-waypoint sequences).
3. **Equivalent-budget analysis uses interpolation, not direct measurement**, from the actual
   stress-test flight's real tracking error. To be strengthened once Phase 2 CSV logging
   captures real tracking error directly during stress-test flights.
3. **Kinematic (single-integrator) dynamics model** — no thrust/attitude dynamics, velocity
   assumed instantaneous. Standard simplification for this class of controller but worth stating.
4. **Gradient-descent MPC, not QP-solver-based.** Only relevant if Linear MPC appears as a
   comparison baseline in final results (currently it does not — MPC was a validation
   stepping-stone, not part of the adaptive-vs-fixed experimental comparison).
5. **n=1 statistical basis** for the core stress-test result as of this writing — Phase 2
   (10-20x repeated trials) is required before this becomes a statistically defensible claim.
6. **SITL only, to date.** HITL validation via UCL resources is planned (Phase 4) but not yet
   started; the final paper's hardware-validation scope depends on how Phase 2/4 timeline plays
   out (see PROJECT_PLAN.md fallback options).

---

## 8. Contribution Statement (working draft, to be finalized in Phase 0 close-out)

*"While prior work has established that MPPI's sample count is a critical parameter constrained
by available compute (Enrico et al. 2025; [drone racing MPPI citation]), existing UAV
deployments treat this as a fixed, offline-tuned design choice. Separately, real-time
control-scheduling co-design has developed principled frameworks for adapting controller
behavior under variable compute (weakly-hard real-time models; self-triggered/resource-aware
MPC), but these target deterministic or linear control formulations, not sampling-based
stochastic optimal control. We bridge this gap by treating MPPI's sample count as a
runtime-adaptive control variable, informed by an empirically characterized quality-compute
trade-off, with an explicit deadline-satisfaction objective and safety fallback mechanism."*

---

## 9. Reproducibility Notes

- Repository: `mppi_controller` ROS2 package (private GitHub repo, tagged commits at each major
  milestone — e.g. `baseline-mppi-tuned`, `headline-result-confirmed`, `phase0-pareto-analysis`).
- To reproduce the flown baseline: build with `colcon build --packages-select mppi_controller`,
  run `ros2 run mppi_controller offboard_node` (adaptive, default) or with
  `--ros-args -p use_scheduler:=false` (fixed baseline), against a running PX4 SITL + Gazebo
  Harmonic + MAVROS stack.
- To reproduce the Pareto sweep: `ros2 run mppi_controller test_n_quality_sweep` (no ROS2/Gazebo
  required — pure standalone C++ executable).
- To reproduce the stress test: run `offboard_node` and, in a separate terminal,
  `ros2 run mppi_controller cpu_load_generator <threads> <duration_sec>` once the flight has
  stabilized.
- Flight logs: CSV files at `/tmp/mppi_flight_log_*.csv`; plot via
  `python3 scripts/plot_flight_log.py <csv files...>`.
