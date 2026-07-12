#include <iostream>
#include <iomanip>
#include <vector>
#include <algorithm>

// Standalone analysis tool -- not part of the flight pipeline.
// Takes the Pareto table produced by test_n_quality_sweep and interpolates
// MeanRMS at an arbitrary N, so you can ask: "what quality would we expect
// at the average N the adaptive scheduler actually used during a real
// stress test?" This directly supports the equivalent-computational-budget
// comparison flagged as necessary to preempt the "adaptive just used more
// compute" objection.
//
// Usage: edit the PARETO_TABLE below with your actual sweep output,
// then edit the SCENARIOS list with real average-N values pulled from
// your stress-test logs.

struct ParetoPoint
{
  int n;
  double mean_rms;
  double std_rms;
  double avg_time_ms;
};

// Paste your actual test_n_quality_sweep CSV output here.
const std::vector<ParetoPoint> PARETO_TABLE = {
  {20,  0.9115, 0.0405, 1.1387},
  {50,  0.8719, 0.0280, 2.7977},
  {100, 0.8393, 0.0491, 5.5407},
  {150, 0.8187, 0.0215, 8.4159},
  {200, 0.7906, 0.0195, 11.6946},
  {250, 0.8075, 0.0295, 14.4366},
  {300, 0.7980, 0.0291, 17.2648},
  {350, 0.7872, 0.0269, 19.6126},
  {400, 0.7893, 0.0301, 22.0119},
};

// Linear interpolation between the two nearest table entries.
double interpolateRms(double target_n)
{
  if (target_n <= PARETO_TABLE.front().n) {
    return PARETO_TABLE.front().mean_rms;
  }
  if (target_n >= PARETO_TABLE.back().n) {
    return PARETO_TABLE.back().mean_rms;
  }

  for (size_t i = 0; i + 1 < PARETO_TABLE.size(); ++i) {
    const auto & lo = PARETO_TABLE[i];
    const auto & hi = PARETO_TABLE[i + 1];
    if (target_n >= lo.n && target_n <= hi.n) {
      double t = (target_n - lo.n) / static_cast<double>(hi.n - lo.n);
      return lo.mean_rms + t * (hi.mean_rms - lo.mean_rms);
    }
  }
  return PARETO_TABLE.back().mean_rms;  // unreachable given the checks above
}

struct Scenario
{
  std::string label;
  double avg_n_during_load;
  int deadline_misses;
};

double computeMean(const std::vector<int> & values)
{
  int sum = 0;
  for (int v : values) {
    sum += v;
  }
  return static_cast<double>(sum) / values.size();
}

int main()
{
  // N values read directly off the adaptive run's log during the
  // confirmed 30-second load window (epoch ~1783681937 to ~1783681967).
  std::vector<int> adaptive_n_values_during_load = {
    400, 329, 317, 325, 318, 325, 333, 325, 325, 322,
    328, 325, 332, 320, 332, 315, 328, 336, 336, 336,
    321, 310, 326, 330, 333, 337, 329, 333, 337, 337, 323
  };

  double adaptive_mean_n = computeMean(adaptive_n_values_during_load);

  std::vector<Scenario> scenarios = {
    {"Adaptive (during load window)", adaptive_mean_n, 0},
    {"Fixed (N=400 always)",          400.0, 5},
  };

  std::cout << "Equivalent-Computational-Budget Analysis\n";
  std::cout << "Interpolated from N-vs-Quality Pareto sweep (10 seeds/N)\n\n";
  std::cout << std::left
            << std::setw(34) << "Scenario"
            << std::setw(12) << "Avg N"
            << std::setw(18) << "Est. MeanRMS(m)"
            << std::setw(16) << "Deadline Misses" << "\n";
  std::cout << std::string(80, '-') << "\n";

  for (const auto & s : scenarios) {
    double est_rms = interpolateRms(s.avg_n_during_load);
    std::cout << std::left << std::fixed << std::setprecision(4)
              << std::setw(34) << s.label
              << std::setw(12) << s.avg_n_during_load
              << std::setw(18) << est_rms
              << std::setw(16) << s.deadline_misses << "\n";
  }

  std::cout << "\nInterpretation:\n";
  std::cout << "If adaptive's estimated RMS is close to or better than fixed's,\n";
  std::cout << "despite using less average compute AND fewer deadline misses,\n";
  std::cout << "this directly counters the 'adaptive just used more compute' objection.\n";

  return 0;
}