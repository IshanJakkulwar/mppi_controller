#include "mppi_controller/models/drone_model.hpp"
#include <cmath>

namespace mppi_controller
{

double DroneModel::yawFromQuaternion(const Eigen::Quaterniond & q)
{
  // Standard yaw extraction from quaternion (ZYX convention),
  // ignoring roll/pitch since the kinematic model doesn't need them yet.
  double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
  double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  return std::atan2(siny_cosp, cosy_cosp);
}

Eigen::Quaterniond DroneModel::quaternionFromYaw(double yaw)
{
  return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
}

DroneState DroneModel::predict(
  const DroneState & current,
  const ControlInput & control,
  double dt) const
{
  DroneState next = current;

  // Position: simple Euler integration of commanded velocity.
  next.position = current.position + control.linear_velocity * dt;

  // Velocity: kinematic model assumes velocity tracks command instantly.
  next.velocity = control.linear_velocity;

  // Orientation: integrate yaw only (roll/pitch untouched for now).
  double current_yaw = yawFromQuaternion(current.orientation);
  double next_yaw = current_yaw + control.yaw_rate * dt;
  next.orientation = quaternionFromYaw(next_yaw);

  // Angular velocity: only yaw rate is modeled currently.
  next.angular_velocity = Eigen::Vector3d(0.0, 0.0, control.yaw_rate);

  next.timestamp = current.timestamp + dt;

  return next;
}

}  // namespace mppi_controller