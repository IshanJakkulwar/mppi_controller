#include <iostream>
#include <thread>
#include <vector>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <string>

// Standalone CPU load generator. Run this in a separate terminal while
// offboard_node is flying, to simulate real compute contention (e.g. a
// perception pipeline or other process competing for CPU) and observe
// whether AnytimeScheduler correctly throttles N down in response.
//
// Usage:
//   ros2 run mppi_controller cpu_load_generator <num_threads> <duration_sec> [mode]
//
// mode:
//   spin (default) -- tight FP busy-loop; contends for CPU time only. The
//                     OS scheduler (CFS/EEVDF) largely shields a periodic
//                     control thread from this kind of fair-share load.
//   mem            -- each thread repeatedly sweeps a 64MB buffer,
//                     thrashing the shared L3 cache and memory bandwidth.
//                     Models a perception-like memory-intensive pipeline;
//                     this slows co-located compute *within* its scheduled
//                     time, which no scheduler policy can shield against.
//
// Example: 6 memory-thrashing threads for 30 seconds
//   ros2 run mppi_controller cpu_load_generator 6 30 mem

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

void burnMemory(std::atomic<bool> & stop_flag)
{
  // Stream through a buffer much larger than the shared L3 cache (24MB on
  // the dev machine's i7-13620H), reading and writing every cache line so
  // the hardware cannot serve it from cache. Evicts co-located processes'
  // working sets and saturates DRAM bandwidth -- the contention mechanism
  // of a real image-processing pipeline, unlike a pure ALU spin.
  constexpr size_t kBufBytes = 64ull * 1024 * 1024;
  constexpr size_t kStride = 64;  // one cache line
  std::vector<unsigned char> buf(kBufBytes, 1);
  volatile unsigned long sink = 0;
  while (!stop_flag.load(std::memory_order_relaxed)) {
    for (size_t i = 0; i < kBufBytes; i += kStride) {
      sink += buf[i];
      buf[i] = static_cast<unsigned char>(sink);
    }
  }
}

int main(int argc, char ** argv)
{
  int num_threads = 2;
  double duration_sec = 15.0;
  std::string mode = "spin";

  if (argc >= 2) {
    num_threads = std::atoi(argv[1]);
  }
  if (argc >= 3) {
    duration_sec = std::atof(argv[2]);
  }
  if (argc >= 4) {
    mode = argv[3];
  }

  if (num_threads < 1) {
    std::cerr << "num_threads must be >= 1\n";
    return 1;
  }
  if (mode != "spin" && mode != "mem") {
    std::cerr << "mode must be 'spin' or 'mem'\n";
    return 1;
  }

  std::time_t start_time = std::time(nullptr);
  std::cout << "[" << std::ctime(&start_time);  // ctime() includes a trailing newline
  std::cout << "Starting CPU load: " << num_threads << " " << mode
            << " threads for " << duration_sec << " seconds\n";
  std::cout << "(run this alongside offboard_node to test AnytimeScheduler under load)\n";

  std::atomic<bool> stop_flag{false};
  std::vector<std::thread> threads;
  for (int i = 0; i < num_threads; ++i) {
    threads.emplace_back(mode == "mem" ? burnMemory : burnCpu,
                         std::ref(stop_flag));
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

  std::time_t end_time = std::time(nullptr);
  std::cout << "CPU load generator finished at " << std::ctime(&end_time);
  return 0;
}