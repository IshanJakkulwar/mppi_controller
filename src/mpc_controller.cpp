#include "mppi_controller/controllers/mpc_controller.hpp"
#include "mppi_controller/utils/math_utils.hpp"

namespace mppi_controller
{

LinearMpcController::LinearMpcController(const MpcConfig & config)
: config_(config)
{
  control_sequence_.assign(config_.horizon, Eigen::Vector3d::Zero());
}

std::vector<DroneState> LinearMpcController::rollout(
  const DroneState & current,
  const std::vector<Eigen::Vector3d> & controls) const
{
  std::vector<DroneState> states;
  states.reserve(controls.size());
  DroneState state = current;
  for (const auto & u : controls) {
    ControlInput control;
    control.linear_velocity = u;  // yaw_rate left at 0, MPC only tracks position
    state = model_.predict(state, control, config_.dt);
    states.push_back(state);
  }
  return states;
}

Eigen::Vector3d LinearMpcController::computeVelocityCommand(
  const DroneState & current,
  const Eigen::Vector3d & target)
{
  // Warm-start from last call's solution rather than zero every time --
  // makes convergence much faster in a receding-horizon loop.
  std::vector<Eigen::Vector3d> controls = control_sequence_;

  for (int iter = 0; iter < config_.max_iterations; ++iter) {
    std::vector<DroneState> states = rollout(current, controls);

    // weighted_errors[k] = 2 * Q * (p_k - target), one per horizon step
    std::vector<Eigen::Vector3d> weighted_errors(states.size());
    for (size_t k = 0; k < states.size(); ++k) {
      Eigen::Vector3d err = states[k].position - target;
      weighted_errors[k] = 2.0 * err.cwiseProduct(config_.state_weight);
    }

    // Analytic gradient: since p_k = p0 + dt * sum_{i=0}^{k} u_i,
    // dCost/du_j = dt * sum_{k=j}^{N-1} weighted_errors[k] + 2*R*u_j
    // Accumulate the sum backward through the horizon (standard adjoint
    // trick for a chain of linear steps -- avoids an O(N^2) loop).
    std::vector<Eigen::Vector3d> grad(controls.size());
    Eigen::Vector3d running_sum = Eigen::Vector3d::Zero();
    for (int j = static_cast<int>(controls.size()) - 1; j >= 0; --j) {
      running_sum += weighted_errors[j];
      grad[j] = config_.dt * running_sum +
        2.0 * controls[j].cwiseProduct(config_.control_weight);
    }

    // Gradient step + projection onto the velocity box constraint.
    for (size_t j = 0; j < controls.size(); ++j) {
      controls[j] -= config_.learning_rate * grad[j];
      controls[j] = math_utils::clampVector(controls[j], config_.max_speed);
    }
  }

  control_sequence_ = controls;
  Eigen::Vector3d first_control = control_sequence_.front();

  // Shift the warm-start buffer left by one for next call (receding
  // horizon: the plan for t+1..t+N becomes the starting guess for
  // t+1..t+N+1, with the new last step repeating the previous last step).
  for (size_t i = 0; i + 1 < control_sequence_.size(); ++i) {
    control_sequence_[i] = control_sequence_[i + 1];
  }
  if (control_sequence_.size() > 1) {
    control_sequence_.back() = control_sequence_[control_sequence_.size() - 2];
  }

  return first_control;
}

void LinearMpcController::reset()
{
  control_sequence_.assign(config_.horizon, Eigen::Vector3d::Zero());
}

}  // namespace mppi_controller