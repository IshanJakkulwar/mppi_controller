#include <iostream>
#include <chrono>
#include "mppi_controller/controllers/mppi_controller.hpp"

using namespace mppi_controller;

int main()
{
  MppiConfig config;
  config.horizon = 20;
  config.num_samples = 200;
  config.dt = 0.1;
  config.state_weight = Eigen::Vector3d(10.0, 10.0, 10.0);
  config.control_weight = Eigen::Vector3d(0.1, 0.1, 0.1);
  config.terminal_weight = Eigen::Vector3d(20.0, 20.0, 20.0);
  config.noise_std = Eigen::Vector3d(1.0, 1.0, 1.0);
  config.max_speed = 2.0;
  config.lambda = 1.0;

  MppiController mppi(config);
  DroneModel true_model;

  DroneState state;
  state.position = Eigen::Vector3d(0.0, 0.0, 5.0);
  Eigen::Vector3d target(2.0, 2.0, 5.0);

  std::cout << "Closed-loop MPPI simulation, target=(2, 2, 5), N="
            << config.num_samples << "\n";

  auto t_start = std::chrono::steady_clock::now();

  for (int step = 0; step < 60; ++step) {
    Eigen::Vector3d cmd = mppi.computeVelocityCommand(state, target);

    ControlInput control;
    control.linear_velocity = cmd;
    state = true_model.predict(state, control, config.dt);

    if (step % 5 == 0) {
      double dist = (state.position - target).norm();
      std::cout << "t=" << step * config.dt
                << "s pos=(" << state.position.x() << ", "
                << state.position.y() << ", " << state.position.z()
                << ") dist_to_target=" << dist << "\n";
    }
  }

  auto t_end = std::chrono::steady_clock::now();
  double elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

  double final_dist = (state.position - target).norm();
  std::cout << "\nFinal distance to target: " << final_dist << "\n";
  std::cout << "Total compute time for 60 control calls: " << elapsed_ms
            << " ms (" << elapsed_ms / 60.0 << " ms/call avg)\n";

  return 0;
}