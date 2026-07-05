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