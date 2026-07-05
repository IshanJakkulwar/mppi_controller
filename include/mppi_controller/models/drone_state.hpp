#ifndef MPPI_CONTROLLER__MODELS__DRONE_STATE_HPP_
#define MPPI_CONTROLLER__MODELS__DRONE_STATE_HPP_

#include <Eigen/Dense>

namespace mppi_controller
{

struct DroneState
{
  Eigen::Vector3d position{0.0, 0.0, 0.0};
  Eigen::Vector3d velocity{0.0, 0.0, 0.0};
  Eigen::Quaterniond orientation{1.0, 0.0, 0.0, 0.0};  // w, x, y, z
  Eigen::Vector3d angular_velocity{0.0, 0.0, 0.0};
  double timestamp{0.0};  // seconds
};

}  // namespace mppi_controller

#endif  // MPPI_CONTROLLER__MODELS__DRONE_STATE_HPP_