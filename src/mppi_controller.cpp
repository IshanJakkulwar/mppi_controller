#include "mppi_controller/controllers/mppi_controller.hpp"
#include <algorithm>
#include <cmath>

namespace mppi_controller
{

MppiController::MppiController(const MppiConfig & config, unsigned int seed)
: config_(config), sampler_(seed)
{
  nominal_sequence_.assign(config_.horizon, Eigen::Vector3d::Zero());
}

double MppiController::rolloutCost(
  const DroneState & current,
  const std::vector<Eigen::Vector3d> & controls,
  const Eigen::Vector3d & target) const
{
  DroneState state = current;
  double cost = 0.0;

  for (size_t k = 0; k < controls.size(); ++k) {
    ControlInput control;
    control.linear_velocity = controls[k];
    state = model_.predict(state, control, config_.dt);

    Eigen::Vector3d pos_error = state.position - target;
    cost += pos_error.cwiseProduct(config_.state_weight).dot(pos_error);
    cost += controls[k].cwiseProduct(config_.control_weight).dot(controls[k]);
  }

  Eigen::Vector3d terminal_error = state.position - target;
  cost += terminal_error.cwiseProduct(config_.terminal_weight).dot(terminal_error);

  return cost;
}

Eigen::Vector3d MppiController::computeVelocityCommand(
  const DroneState & current,
  const Eigen::Vector3d & target)
{
  // Plain baseline: always use the configured default N.
  return computeVelocityCommandWithN(current, target, config_.num_samples);
}

Eigen::Vector3d MppiController::computeVelocityCommandWithN(
  const DroneState & current,
  const Eigen::Vector3d & target,
  int num_samples)
{
  const int N = num_samples;
  const int H = config_.horizon;

  std::vector<std::vector<Eigen::Vector3d>> sampled_controls(N);
  std::vector<double> costs(N);

  // --- Sample and evaluate ---
  for (int i = 0; i < N; ++i) {
    sampled_controls[i].resize(H);
    for (int k = 0; k < H; ++k) {
      Eigen::Vector3d noise = sampler_.sample(config_.noise_std);
      Eigen::Vector3d perturbed = nominal_sequence_[k] + noise;
      sampled_controls[i][k] = math_utils::clampVector(perturbed, config_.max_speed);
    }
    costs[i] = rolloutCost(current, sampled_controls[i], target);
  }

  // --- Weights: softmax(-cost / lambda), stabilized by subtracting min ---
  double min_cost = *std::min_element(costs.begin(), costs.end());

  std::vector<double> weights(N);
  double weight_sum = 0.0;
  for (int i = 0; i < N; ++i) {
    weights[i] = std::exp(-(costs[i] - min_cost) / config_.lambda);
    weight_sum += weights[i];
  }
  for (int i = 0; i < N; ++i) {
    weights[i] /= weight_sum;
  }

  // --- Weighted average of sampled sequences becomes new nominal ---
  std::vector<Eigen::Vector3d> new_nominal(H, Eigen::Vector3d::Zero());
  for (int i = 0; i < N; ++i) {
    for (int k = 0; k < H; ++k) {
      new_nominal[k] += weights[i] * sampled_controls[i][k];
    }
  }

  nominal_sequence_ = new_nominal;
  Eigen::Vector3d first_control = nominal_sequence_.front();

  // --- Shift warm-start buffer for next call (receding horizon) ---
  for (int k = 0; k + 1 < H; ++k) {
    nominal_sequence_[k] = nominal_sequence_[k + 1];
  }
  if (H > 1) {
    nominal_sequence_.back() = nominal_sequence_[H - 2];
  }

  return first_control;
}

void MppiController::reset()
{
  nominal_sequence_.assign(config_.horizon, Eigen::Vector3d::Zero());
}

}  // namespace mppi_controller