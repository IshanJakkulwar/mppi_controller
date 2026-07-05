#ifndef MPPI_CONTROLLER__CONTROLLERS__MPC_CONTROLLER_HPP_
#define MPPI_CONTROLLER__CONTROLLERS__MPC_CONTROLLER_HPP_

#include <vector>
#include <Eigen/Dense>
#include "mppi_controller/models/drone_state.hpp"
#include "mppi_controller/models/drone_model.hpp"

namespace mppi_controller
{

struct MpcConfig
{
  int horizon = 10;                                   // number of steps N
  double dt = 0.1;                                     // seconds per step
  Eigen::Vector3d state_weight{10.0, 10.0, 10.0};       // Q: tracking penalty
  Eigen::Vector3d control_weight{0.1, 0.1, 0.1};        // R: effort penalty
  double max_speed = 2.0;                               // m/s, per-axis-magnitude box
  int max_iterations = 50;                              // gradient descent steps per call
  double learning_rate = 0.05;
};

// Linear MPC over a single-integrator kinematic model. Solves
//   min sum_k (p_k - target)^T Q (p_k - target) + sum_k u_k^T R u_k
//   s.t. |u_k| <= max_speed
// via projected gradient descent (dynamics are linear in u, cost is convex,
// so this converges reliably without needing a QP solver dependency yet).
class LinearMpcController
{
public:
  explicit LinearMpcController(const MpcConfig & config);

  // Solves the horizon problem for the current state/target and returns
  // only the first control action (standard receding-horizon behavior).
  Eigen::Vector3d computeVelocityCommand(
    const DroneState & current,
    const Eigen::Vector3d & target_position);

  void reset();

private:
  std::vector<DroneState> rollout(
    const DroneState & current,
    const std::vector<Eigen::Vector3d> & controls) const;

  MpcConfig config_;
  DroneModel model_;
  std::vector<Eigen::Vector3d> control_sequence_;  // warm-start buffer, size = horizon
};

}  // namespace mppi_controller

#endif  // MPPI_CONTROLLER__CONTROLLERS__MPC_CONTROLLER_HPP_