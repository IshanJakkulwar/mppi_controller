#include <iostream>
#include <thread>
#include <vector>
#include <atomic>
#include <chrono>
#include <cstdlib>

// Standalone CPU load generator. Run this in a separate terminal while
// offboard_node is flying, to simulate real compute contention (e.g. a
// perception pipeline or other process competing for CPU) and observe
// whether AnytimeScheduler correctly throttles N down in response.
//
// Usage:
//   ros2 run mppi_controller cpu_load_generator <num_threads> <duration_sec>
//
// Example: spin up 3 busy threads for 20 seconds
//   ros2 run mppi_controller cpu_load_generator 3 20

void burnCpu(std::atomic<bool> & stop_flag)
{
  // Tight busy-loop with no sleeps -- pins one CPU core at ~100% until
  // stop_flag is set. Doing real (wasted) floating point work rather than
  // an empty loop discourages the compiler from optimizing this away.
  volatile double x = 1.0;
  while (!stop_flag.load(std::memory_order_relaxed)) {
    for (int i = 0; i < 100000; ++i) {
      x = x * 1.0000001 + 0.0000001;
    }
  }
}

int main(int argc, char ** argv)
{
  int num_threads = 2;
  double duration_sec = 15.0;

  if (argc >= 2) {
    num_threads = std::atoi(argv[1]);
  }
  if (argc >= 3) {
    duration_sec = std::atof(argv[2]);
  }

  if (num_threads < 1) {
    std::cerr << "num_threads must be >= 1\n";
    return 1;
  }

  std::cout << "Starting CPU load: " << num_threads
            << " busy threads for " << duration_sec << " seconds\n";
  std::cout << "(run this alongside offboard_node to test AnytimeScheduler under load)\n";

  std::atomic<bool> stop_flag{false};
  std::vector<std::thread> threads;
  for (int i = 0; i < num_threads; ++i) {
    threads.emplace_back(burnCpu, std::ref(stop_flag));
  }

  auto start = std::chrono::steady_clock::now();
  while (std::chrono::duration<double>(
           std::chrono::steady_clock::now() - start).count() < duration_sec)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  stop_flag.store(true);
  for (auto & t : threads) {
    t.join();
  }

  std::cout << "CPU load generator finished.\n";
  return 0;
}