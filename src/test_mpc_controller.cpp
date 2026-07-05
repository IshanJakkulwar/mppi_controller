#include <iostream>
#include "mppi_controller/controllers/mpc_controller.hpp"

using namespace mppi_controller;

int main()
{
  MpcConfig config;
  config.horizon = 10;
  config.dt = 0.1;
  config.state_weight = Eigen::Vector3d(10.0, 10.0, 10.0);
  config.control_weight = Eigen::Vector3d(0.1, 0.1, 0.1);
  config.max_speed = 2.0;
  config.max_iterations = 50;
  config.learning_rate = 0.05;

  LinearMpcController mpc(config);
  DroneModel true_model;  // simulates the "real" drone

  DroneState state;
  state.position = Eigen::Vector3d(0.0, 0.0, 5.0);
  Eigen::Vector3d target(2.0, 2.0, 5.0);

  std::cout << "Closed-loop MPC simulation, target=(2, 2, 5)\n";
  for (int step = 0; step < 60; ++step) {
    Eigen::Vector3d cmd = mpc.computeVelocityCommand(state, target);

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

  double final_dist = (state.position - target).norm();
  std::cout << "\nFinal distance to target: " << final_dist
            << " (expect this to approach 0)\n";

  return 0;
}