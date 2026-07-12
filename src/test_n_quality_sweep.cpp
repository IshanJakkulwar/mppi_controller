#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <chrono>
#include <numeric>
#include "mppi_controller/controllers/mppi_controller.hpp"

using namespace mppi_controller;

struct SweepResult
{
  int n;
  double mean_rms_error;
  double stddev_rms_error;
  double mean_final_error;
  double stddev_final_error;
  double avg_call_time_ms;
};

struct SingleRunResult
{
  double rms_error;
  double final_error;
  double call_time_ms;
};

SingleRunResult runOnce(int n, unsigned int seed, const MppiConfig & base_config)
{
  MppiController mppi(base_config, seed);
  DroneModel true_model;

  DroneState state;
  state.position = Eigen::Vector3d(0.0, 0.0, 5.0);
  Eigen::Vector3d target(2.0, 2.0, 5.0);

  const int num_steps = 80;
  std::vector<double> errors;
  double total_call_time_sec = 0.0;

  for (int step = 0; step < num_steps; ++step) {
    auto t0 = std::chrono::steady_clock::now();
    Eigen::Vector3d cmd = mppi.computeVelocityCommandWithN(state, target, n);
    auto t1 = std::chrono::steady_clock::now();
    total_call_time_sec += std::chrono::duration<double>(t1 - t0).count();

    ControlInput control;
    control.linear_velocity = cmd;
    state = true_model.predict(state, control, base_config.dt);

    errors.push_back((state.position - target).norm());
  }

  double sum_sq = 0.0;
  for (double e : errors) {
    sum_sq += e * e;
  }
  double rms_error = std::sqrt(sum_sq / errors.size());
  double final_error = errors.back();
  double avg_call_time_ms = (total_call_time_sec / num_steps) * 1000.0;

  return {rms_error, final_error, avg_call_time_ms};
}

SweepResult runSweepForN(int n, const MppiConfig & base_config, int num_seeds)
{
  std::vector<double> rms_errors;
  std::vector<double> final_errors;
  double total_call_time_ms = 0.0;

  for (int seed = 0; seed < num_seeds; ++seed) {
    // Different seed per trial, but same set of seeds reused across every
    // N value (0..num_seeds-1) so comparisons across N aren't confounded
    // by different random draws.
    SingleRunResult r = runOnce(n, static_cast<unsigned int>(seed), base_config);
    rms_errors.push_back(r.rms_error);
    final_errors.push_back(r.final_error);
    total_call_time_ms += r.call_time_ms;
  }

  auto mean = [](const std::vector<double> & v) {
    return std::accumulate(v.begin(), v.end(), 0.0) / v.size();
  };
  auto stddev = [&](const std::vector<double> & v, double m) {
    double sum_sq_diff = 0.0;
    for (double x : v) {
      sum_sq_diff += (x - m) * (x - m);
    }
    return std::sqrt(sum_sq_diff / v.size());
  };

  double mean_rms = mean(rms_errors);
  double mean_final = mean(final_errors);

  return {
    n,
    mean_rms,
    stddev(rms_errors, mean_rms),
    mean_final,
    stddev(final_errors, mean_final),
    total_call_time_ms / num_seeds
  };
}

int main()
{
  MppiConfig base_config;
  base_config.horizon = 20;
  base_config.dt = 0.1;
  base_config.state_weight = Eigen::Vector3d(10.0, 10.0, 10.0);
  base_config.control_weight = Eigen::Vector3d(0.3, 0.3, 0.3);
  base_config.terminal_weight = Eigen::Vector3d(20.0, 20.0, 20.0);
  base_config.noise_std = Eigen::Vector3d(0.5, 0.5, 0.5);
  base_config.max_speed = 2.0;
  base_config.lambda = 3.0;

  std::vector<int> n_values = {20, 50, 100, 150, 200, 250, 300, 350, 400};
  const int num_seeds = 10;

  std::cout << "N-vs-Quality Pareto sweep (" << num_seeds
            << " seeds per N, target=(2,2,5), 80 steps)\n";
  std::cout << std::left
            << std::setw(6) << "N"
            << std::setw(18) << "MeanRMS(m)"
            << std::setw(14) << "StdRMS"
            << std::setw(18) << "MeanFinal(m)"
            << std::setw(14) << "StdFinal"
            << std::setw(16) << "AvgTime(ms)" << "\n";
  std::cout << std::string(86, '-') << "\n";

  std::vector<SweepResult> results;
  for (int n : n_values) {
    SweepResult r = runSweepForN(n, base_config, num_seeds);
    results.push_back(r);
    std::cout << std::left << std::fixed << std::setprecision(4)
              << std::setw(6) << r.n
              << std::setw(18) << r.mean_rms_error
              << std::setw(14) << r.stddev_rms_error
              << std::setw(18) << r.mean_final_error
              << std::setw(14) << r.stddev_final_error
              << std::setw(16) << r.avg_call_time_ms << "\n";
  }

  std::cout << "\nCSV output (paste into a spreadsheet/plot):\n";
  std::cout << "N,MeanRMS,StdRMS,MeanFinal,StdFinal,AvgCallTime_ms\n";
  for (const auto & r : results) {
    std::cout << r.n << "," << r.mean_rms_error << "," << r.stddev_rms_error
               << "," << r.mean_final_error << "," << r.stddev_final_error
               << "," << r.avg_call_time_ms << "\n";
  }

  return 0;
}