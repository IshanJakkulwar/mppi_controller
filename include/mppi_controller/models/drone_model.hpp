#ifndef MPPI_CONTROLLER__MODELS__DRONE_MODEL_HPP_
#define MPPI_CONTROLLER__MODELS__DRONE_MODEL_HPP_

#include <Eigen/Dense>
#include "mppi_controller/models/drone_state.hpp"

namespace mppi_controller
{

// Control input for the simple kinematic model: commanded body-independent
// (world-frame) linear velocity plus a yaw rate. This intentionally assumes
// the vehicle achieves commanded velocity instantly -- no thrust/attitude
// dynamics yet. That upgrade comes later ("higher fidelity dynamics").
struct ControlInput
{
  Eigen::Vector3d linear_velocity{0.0, 0.0, 0.0};  // m/s, world frame
  double yaw_rate{0.0};                            // rad/s
};

class DroneModel
{
public:
  DroneModel() = default;

  // Predicts the next state given the current state, a control input,
  // and a timestep. Pure function: no internal state, safe to call
  // repeatedly (e.g. rolled out many steps for MPC/MPPI prediction).
  DroneState predict(
    const DroneState & current,
    const ControlInput & control,
    double dt) const;

private:
  // Extracts yaw (rotation about world z) from a quaternion.
  static double yawFromQuaternion(const Eigen::Quaterniond & q);

  // Builds a quaternion representing a pure yaw rotation.
  static Eigen::Quaterniond quaternionFromYaw(double yaw);
};

}  // namespace mppi_controller

#endif  // MPPI_CONTROLLER__MODELS__DRONE_MODEL_HPP_