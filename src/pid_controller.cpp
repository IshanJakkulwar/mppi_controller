#include "mppi_controller/controllers/pid_controller.hpp"
#include "mppi_controller/utils/math_utils.hpp"

namespace mppi_controller
{

PidController::PidController(
  const Eigen::Vector3d & kp,
  const Eigen::Vector3d & ki,
  const Eigen::Vector3d & kd,
  double max_speed)
: kp_(kp), ki_(ki), kd_(kd), max_speed_(max_speed)
{
}

Eigen::Vector3d PidController::computeVelocityCommand(
  const DroneState & current,
  const Eigen::Vector3d & target_position,
  double dt)
{
  Eigen::Vector3d error = target_position - current.position;

  integral_error_ += error * dt;

  Eigen::Vector3d derivative = Eigen::Vector3d::Zero();
  if (has_previous_error_ && dt > 1e-6) {
    derivative = (error - previous_error_) / dt;
  }
  previous_error_ = error;
  has_previous_error_ = true;

  Eigen::Vector3d command =
    kp_.cwiseProduct(error) +
    ki_.cwiseProduct(integral_error_) +
    kd_.cwiseProduct(derivative);

  return math_utils::clampVector(command, max_speed_);
}

void PidController::reset()
{
  integral_error_.setZero();
  previous_error_.setZero();
  has_previous_error_ = false;
}

}  // namespace mppi_controller