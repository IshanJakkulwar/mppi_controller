#include "mppi_controller/controllers/anytime_scheduler.hpp"
#include "mppi_controller/utils/math_utils.hpp"
#include <numeric>

namespace mppi_controller
{

AnytimeScheduler::AnytimeScheduler(const SchedulerConfig & config)
: config_(config)
{
}

int AnytimeScheduler::recommendNextSampleCount(
  double last_call_duration_sec, int last_call_n)
{
  // Guard against nonsense input (e.g. first call, or n=0).
  if (last_call_n <= 0 || last_call_duration_sec <= 0.0) {
    return config_.n_default;
  }

  double time_per_sample = last_call_duration_sec / last_call_n;
  time_per_sample_history_.push_back(time_per_sample);
  if (static_cast<int>(time_per_sample_history_.size()) > config_.smoothing_window) {
    time_per_sample_history_.pop_front();
  }

  double avg_time_per_sample =
    std::accumulate(time_per_sample_history_.begin(),
                     time_per_sample_history_.end(), 0.0) /
    time_per_sample_history_.size();

  // Track deadline misses for the safety layer, independent of the
  // smoothed estimate above -- this reacts to the raw last call, not
  // the average, since a fallback decision should be about "right now."
  double budget = config_.target_loop_time * config_.deadline_margin;
  if (last_call_duration_sec > config_.target_loop_time) {
    consecutive_deadline_misses_++;
  } else {
    consecutive_deadline_misses_ = 0;
  }

  if (avg_time_per_sample <= 1e-9) {
    return config_.n_default;
  }

  int recommended_n = static_cast<int>(budget / avg_time_per_sample);
  recommended_n = math_utils::clamp(recommended_n, config_.n_min, config_.n_max);

  return recommended_n;
}

bool AnytimeScheduler::inSafetyFallback() const
{
  // Two consecutive real deadline blowouts -> recommend fallback.
  // (Threshold of 2, not 1, so a single hiccup doesn't trigger it --
  // avoids flapping between MPPI and fallback on noisy timing.)
  return consecutive_deadline_misses_ >= 2;
}

void AnytimeScheduler::reset()
{
  time_per_sample_history_.clear();
  consecutive_deadline_misses_ = 0;
}

}  // namespace mppi_controller