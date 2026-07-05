#ifndef MPPI_CONTROLLER__CONTROLLERS__PID_CONTROLLER_HPP_
#define MPPI_CONTROLLER__CONTROLLERS__PID_CONTROLLER_HPP_

#include <Eigen/Dense>
#include "mppi_controller/models/drone_state.hpp"

namespace mppi_controller
{

class PidController
{
public:
  PidController(
    const Eigen::Vector3d & kp,
    const Eigen::Vector3d & ki,
    const Eigen::Vector3d & kd,
    double max_speed);

  // Produces a velocity command (m/s, world frame) to drive
  // current toward target_position. dt in seconds.
  Eigen::Vector3d computeVelocityCommand(
    const DroneState & current,
    const Eigen::Vector3d & target_position,
    double dt);

  void reset();

private:
  Eigen::Vector3d kp_;
  Eigen::Vector3d ki_;
  Eigen::Vector3d kd_;
  double max_speed_;

  Eigen::Vector3d integral_error_{0.0, 0.0, 0.0};
  Eigen::Vector3d previous_error_{0.0, 0.0, 0.0};
  bool has_previous_error_{false};
};

}  // namespace mppi_controller

#endif  // MPPI_CONTROLLER__CONTROLLERS__PID_CONTROLLER_HPP_