#ifndef MPPI_CONTROLLER__CONTROLLERS__MPPI_CONTROLLER_HPP_
#define MPPI_CONTROLLER__CONTROLLERS__MPPI_CONTROLLER_HPP_

#include <vector>
#include <Eigen/Dense>
#include "mppi_controller/models/drone_state.hpp"
#include "mppi_controller/models/drone_model.hpp"
#include "mppi_controller/utils/math_utils.hpp"

namespace mppi_controller
{

struct MppiConfig
{
  int horizon = 20;                                 // steps per rollout
  int num_samples = 200;                             // default N, used unless overridden per-call
  double dt = 0.1;                                   // seconds per step
  Eigen::Vector3d state_weight{10.0, 10.0, 10.0};     // running cost on position error
  Eigen::Vector3d control_weight{0.1, 0.1, 0.1};      // running cost on control effort
  Eigen::Vector3d terminal_weight{20.0, 20.0, 20.0};  // extra cost on final position error
  Eigen::Vector3d noise_std{1.0, 1.0, 1.0};           // per-axis sampling std dev, m/s
  double max_speed = 2.0;                             // m/s box constraint
  double lambda = 1.0;                                // temperature: lower = greedier weighting
};

// Baseline Model Predictive Path Integral controller. Sample count N can
// either come from config_.num_samples (fixed, via computeVelocityCommand)
// or be overridden per-call (via computeVelocityCommandWithN) -- the
// latter is what AnytimeScheduler drives for the compute-aware variant.
class MppiController
{
public:
  explicit MppiController(const MppiConfig & config, unsigned int seed = 42);

  // Fixed-N entry point: always uses config_.num_samples.
  // This is "plain baseline MPPI" -- what you'd fly to get the
  // fixed-budget comparison point in the experimental design.
  Eigen::Vector3d computeVelocityCommand(
    const DroneState & current,
    const Eigen::Vector3d & target_position);

  // Compute-aware entry point: num_samples overrides config_.num_samples
  // for this call only. AnytimeScheduler decides what to pass in here
  // based on measured timing from the previous call.
  Eigen::Vector3d computeVelocityCommandWithN(
    const DroneState & current,
    const Eigen::Vector3d & target_position,
    int num_samples);

  void reset();

private:
  double rolloutCost(
    const DroneState & current,
    const std::vector<Eigen::Vector3d> & controls,
    const Eigen::Vector3d & target) const;

  MppiConfig config_;
  DroneModel model_;
  math_utils::GaussianSampler sampler_;
  std::vector<Eigen::Vector3d> nominal_sequence_;  // warm-start, size = horizon
};

}  // namespace mppi_controller

#endif  // MPPI_CONTROLLER__CONTROLLERS__MPPI_CONTROLLER_HPP_