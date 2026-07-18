# Related Work (draft — Phase 3)

*Markdown staging draft for the ECC paper's Related Work section. Citation keys refer to
`references.bib`. Hedging discipline: "to the best of our knowledge" framing throughout; no
absolute novelty claims (PROJECT_PLAN Phase 3 language correction).*

---

Our work sits at the intersection of three research threads: sampling-based model
predictive control on aerial platforms, computational-resource-aware control, and adaptive
sampling within MPPI itself.

## A. MPPI on aerial platforms and embedded hardware

Model Predictive Path Integral control samples a large number of candidate control
trajectories per cycle and combines them via an information-theoretic weighting
[williams2016aggressive, williams2018information]. The number of samples $N$ is a central
parameter: it determines both the quality of the optimized control and, linearly, the
per-cycle computational cost.

A substantial body of work has since extended MPPI along several axes: robustness to
disturbance via ancillary tracking controllers and augmented-state formulations
[williams2018robust, gandhi2021robust], improved sampling distributions
[mohamed2022logmppi], generalized information-theoretic objectives [wang2021tsallis], and
safety guarantees through control barrier functions [yin2023shield]. Existing MPPI
research has thus largely focused on improving the sampling distribution, robustness,
constraint handling, or computational acceleration — the *size* of the sampling budget
itself, and its relationship to fluctuating runtime compute availability, has received
comparatively little attention.

Recent work has brought MPPI onto UAVs with onboard, embedded compute. Minařík et al.
[minarik2024mppi] demonstrate, to the best of our knowledge, the first onboard MPPI-based
control in real UAV flight, running the sampling-based optimization on an onboard GPU and
noting that real-time feasibility constrains the usable sample count. Enrico, Mancini and
Capello [enrico2025comparison] compare NMPC against GPU-parallelized MPPI on a Jetson Orin
Nano under a ROS 2 / PX4 stack closely matching ours, and select an MPPI configuration
(horizon 40, 800–1250 samples) explicitly to satisfy the 50 Hz deadline imposed by the
flight controller — a *fixed* configuration derived through *offline* analysis.

Across representative embedded MPPI implementations, the sampling budget is selected
offline based on expected computational resources and then held fixed during deployment.
This design choice is not accidental: a fixed budget simplifies timing analysis,
implementation, and parameter tuning, making it the de facto standard in existing MPPI
deployments. It nevertheless embeds the assumption that the compute available to the
controller is constant at runtime — an assumption that fails when the controller shares a
companion computer with perception, mapping, logging, or other variable workloads.

## B. Computational-resource-aware control

A separate literature explicitly treats computation as a limited resource for control.

**Anytime control** designs controllers whose output quality improves monotonically with
allotted computation, so that early termination still yields a usable command. Fontanelli,
Greco and Bicchi [fontanelli2008anytime] switch among control laws of increasing quality
under processor preemption; Quevedo and Gupta [quevedo2013sequence] compute control
*sequences* under stochastic processor availability so that stale entries cover cycles
where computation is cut short. Closest in spirit to our setting, Pant et al.
[pant2021anytime] co-design an anytime perception-based *estimator* with a robust
controller, using an offline-characterized delay/error trade-off curve as a runtime
contract. These frameworks established the principle we build on — trading computation
against control quality at runtime — but target deterministic or linear control
formulations (or the estimation side of the loop), not the internal sampling budget of a
stochastic sampling-based optimizer.

**Feedback scheduling.** The real-time systems community has long applied feedback
control to computation itself: measuring execution behavior and adjusting task parameters
to hold a utilization target. Feedback control real-time scheduling [lu2002feedback]
regulates CPU utilization via measured execution times; the elastic task model
[buttazzo1998elastic] adapts task rates within declared ranges under overload; and
feedback–feedforward scheduling [cervin2002feedback] applies these ideas to control tasks
specifically, adjusting sampling periods from measured load. **Our budget law is an
instance of this family** — a measured-cost, utilization-targeting allocator with
smoothing and clamping — and we claim no novelty for the mechanism itself. What is new,
to the best of our knowledge, is the *actuator*: rather than adjusting task periods or
admission, we adjust the internal sampling budget of a stochastic sampling-based
optimizer, exploiting a property those classical works could not assume — MPPI's
per-cycle computation is fungible in fine increments with a *characterizable, gracefully
degrading* relationship between computation and control quality (the Pareto curve of
Sec. A), rather than the step change of dropping or delaying a whole task.

**Event- and self-triggered MPC** adapts *when* control is computed rather than *how much*
computation each solve receives [heemels2012introduction]. Resource-aware self-triggered
MPC [gommans2015resource] chooses inter-sample times online to save computation and
communication, again for deterministic linear/nonlinear formulations with a solver whose
per-solve cost is fixed.

**Weakly-hard real-time systems** formalize the tolerance of control loops to a bounded
pattern of deadline misses [bernat2001weakly], and recent work characterizes closed-loop
stability under consecutive-miss constraints [maggio2020control]. This thread treats the
miss pattern as given and analyzes its consequences; our scheduler instead acts *upstream*,
adapting per-cycle computation to avoid misses in the first place, with the weakly-hard
perspective informing our safety-fallback design (consecutive-miss triggering).

## C. Adaptive sampling within MPPI

A third, orthogonal thread adapts MPPI's sampling *distribution*. Asmar et al.
[asmar2023mpopis] generalize MPPI to admit adaptive importance sampling, iteratively
matching the proposal's moments to improve sample efficiency at fixed budget. Such methods
adapt *where* to sample, driven by optimization performance; they do not adapt *how many*
samples to draw, and the adaptation is not driven by measured compute availability.

## Positioning

We are not aware of prior work that applies feedback-scheduling principles to the
sampling budget of a stochastic sampling-based optimal controller, with an explicit
deadline-satisfaction objective and safety fallback, validated in closed-loop UAV
experiments. We
formulate runtime sample-count selection as an online computational resource allocation
problem: the scheduler measures per-sample execution time, sets the next cycle's budget to
fit within a fixed fraction of the control period, and falls back safely under consecutive
deadline misses. Our experimental findings connect the threads above: the offline
quality–compute characterization (in the spirit of [pant2021anytime]'s trade-off curves)
predicts flight-quality outcomes across operating regimes, because adaptation operates on
the flat region of MPPI's quality–compute curve — the regime where samples are cheap to
give up — while deadline adherence, the weakly-hard-motivated metric, is where adaptation
shows its value.

---

*TODO before submission: pull final page numbers for [minarik2024mppi], [asmar2023mpopis];
confirm co-author spellings for [maggio2020control] and full first names for
[enrico2025comparison] against the proceedings PDFs (flagged in references.bib).*
