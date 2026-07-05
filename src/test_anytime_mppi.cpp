#include <iostream>
#include <chrono>
#include "mppi_controller/controllers/mppi_controller.hpp"
#include "mppi_controller/controllers/anytime_scheduler.hpp"

using namespace mppi_controller;

int main()
{
  MppiConfig mppi_config;
  mppi_config.horizon = 20;
  mppi_config.dt = 0.1;
  mppi_config.state_weight = Eigen::Vector3d(10.0, 10.0, 10.0);
  mppi_config.control_weight = Eigen::Vector3d(0.1, 0.1, 0.1);
  mppi_config.terminal_weight = Eigen::Vector3d(20.0, 20.0, 20.0);
  mppi_config.noise_std = Eigen::Vector3d(1.0, 1.0, 1.0);
  mppi_config.max_speed = 2.0;
  mppi_config.lambda = 1.0;

  SchedulerConfig sched_config;
  sched_config.n_min = 20;
  sched_config.n_max = 400;
  sched_config.n_default = 200;
  sched_config.target_loop_time = 0.02;  // 50 Hz target
  sched_config.deadline_margin = 0.8;

  MppiController mppi(mppi_config);
  AnytimeScheduler scheduler(sched_config);
  DroneModel true_model;

  DroneState state;
  state.position = Eigen::Vector3d(0.0, 0.0, 5.0);
  Eigen::Vector3d target(2.0, 2.0, 5.0);

  int next_n = sched_config.n_default;

  std::cout << "Compute-aware MPPI simulation, target=(2, 2, 5)\n";
  std::cout << "target_loop_time=" << sched_config.target_loop_time
            << "s, deadline_margin=" << sched_config.deadline_margin << "\n\n";

  for (int step = 0; step < 60; ++step) {
    auto t0 = std::chrono::steady_clock::now();
    Eigen::Vector3d cmd = mppi.computeVelocityCommandWithN(state, target, next_n);
    auto t1 = std::chrono::steady_clock::now();
    double duration_sec = std::chrono::duration<double>(t1 - t0).count();

    int used_n = next_n;
    next_n = scheduler.recommendNextSampleCount(duration_sec, used_n);

    ControlInput control;
    control.linear_velocity = cmd;
    state = true_model.predict(state, control, mppi_config.dt);

    if (step % 5 == 0) {
      double dist = (state.position - target).norm();
      std::cout << "t=" << step * mppi_config.dt
                << "s N=" << used_n
                << " call_time=" << duration_sec * 1000.0 << "ms"
                << " dist_to_target=" << dist
                << " next_N=" << next_n << "\n";
    }
  }

  std::cout << "\nFinal N converged to approximately: " << next_n << "\n";
  std::cout << "(N should stabilize near a value consistent with "
            << sched_config.target_loop_time * sched_config.deadline_margin
            << "s budget)\n";

  return 0;
}