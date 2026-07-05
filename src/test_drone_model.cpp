#include <iostream>
#include "mppi_controller/models/drone_model.hpp"

using namespace mppi_controller;

int main()
{
  DroneModel model;

  DroneState state;
  state.position = Eigen::Vector3d(0.0, 0.0, 5.0);
  state.orientation = Eigen::Quaterniond::Identity();

  ControlInput control;
  control.linear_velocity = Eigen::Vector3d(1.0, 0.0, 0.0);  // 1 m/s in x
  control.yaw_rate = 0.0;

  double dt = 0.1;

  std::cout << "Simulating 10 steps at 1 m/s in x, dt=0.1s\n";
  for (int i = 0; i < 10; ++i) {
    state = model.predict(state, control, dt);
    std::cout << "step " << i
              << " pos=(" << state.position.x() << ", "
              << state.position.y() << ", "
              << state.position.z() << ")\n";
  }

  std::cout << "\nExpected final x approx 1.0 (1 m/s * 1.0s total)\n";

  // Now test yaw integration.
  DroneState yaw_state;
  ControlInput yaw_control;
  yaw_control.yaw_rate = 0.5;  // rad/s
  std::cout << "\nSimulating 10 steps of yaw_rate=0.5 rad/s, dt=0.1s\n";
  for (int i = 0; i < 10; ++i) {
    yaw_state = model.predict(yaw_state, yaw_control, dt);
  }
  double final_yaw = std::atan2(
    2.0 * (yaw_state.orientation.w() * yaw_state.orientation.z()),
    1.0 - 2.0 * yaw_state.orientation.z() * yaw_state.orientation.z());
  std::cout << "Final yaw approx " << final_yaw
            << " rad, expected approx 0.5 rad\n";

  return 0;
}