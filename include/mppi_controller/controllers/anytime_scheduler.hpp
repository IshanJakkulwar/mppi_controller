#ifndef MPPI_CONTROLLER__CONTROLLERS__ANYTIME_SCHEDULER_HPP_
#define MPPI_CONTROLLER__CONTROLLERS__ANYTIME_SCHEDULER_HPP_

#include <deque>

namespace mppi_controller
{

struct SchedulerConfig
{
  int n_min = 20;                  // minimum samples, safety floor
  int n_max = 400;                 // maximum samples, quality ceiling
  int n_default = 200;             // starting point before any measurements exist
  double target_loop_time = 0.05;  // desired control period, seconds (20 Hz)
  double deadline_margin = 0.8;    // fraction of target_loop_time we budget for MPPI itself
  int smoothing_window = 3;        // number of past timings to average over

  // Pareto-informed quality floor (0 = disabled -> pure deadline law).
  // When enabled, the scheduler will not allocate below this N as long as
  // the floor is affordable within the FULL control period (sacrificing
  // the deadline margin, not the deadline). Lexicographic priority:
  //   1. deadline (hard): if n_quality_floor * tau_hat > target_loop_time,
  //      the floor yields and the pure deadline law applies (the
  //      consecutive-miss fallback detector is the escape hatch);
  //   2. quality floor (soft): otherwise never drop below it;
  //   3. margin/quality (best effort): the beta*T budget law above it.
  // The value is derived offline from the Pareto sweep: the smallest N
  // whose predicted quality is within one noise-floor sigma of N_max
  // (N=150 for this platform's sweep).
  int n_quality_floor = 0;
};

// Observes how long each MPPI call actually took and proposes a new
// sample count N for the *next* call, aiming to keep MPPI's compute time
// under (target_loop_time * deadline_margin). This is the g(tau) function
// from the project spec: tau is estimated as time-per-sample from recent
// history, N = g(tau) = budget / time_per_sample.
class AnytimeScheduler
{
public:
  explicit AnytimeScheduler(const SchedulerConfig & config);

  // Call after every MPPI rollout with how long it took (seconds) and
  // how many samples it used. Returns the recommended N for the next call.
  int recommendNextSampleCount(double last_call_duration_sec, int last_call_n);

  // True if the two most recent calls both blew the deadline badly enough
  // that the caller should consider falling back to a cheap controller
  // instead of MPPI entirely this cycle.
  bool inSafetyFallback() const;

  void reset();

private:
  SchedulerConfig config_;
  std::deque<double> time_per_sample_history_;  // seconds/sample, most recent last
  int consecutive_deadline_misses_ = 0;
};

}  // namespace mppi_controller

#endif  // MPPI_CONTROLLER__CONTROLLERS__ANYTIME_SCHEDULER_HPP_