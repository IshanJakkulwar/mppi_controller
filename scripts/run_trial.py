#!/usr/bin/env python3
"""
Automated Phase 2 trial runner.

Runs offboard_node under one of the three experimental conditions, fires the
CPU load generator at a fixed offset after MPPI tracking begins (so the load
window lands identically in every trial), collects the flight CSV into
results/<condition>/trial_<k>.csv, and lands/disarms the vehicle between
trials.

Assumes PX4 SITL + Gazebo + MAVROS are already running, and that this shell
has the workspace sourced (ros2 on PATH, mppi_controller built).

Usage:
    python3 run_trial.py --condition adaptive --trials 3
    python3 run_trial.py --condition const330 --trials 3
    python3 run_trial.py --condition fixed400 --trials 3

Conditions (see PROJECT_PLAN.md, Phase 2):
    adaptive  -- use_scheduler:=true  (AnytimeScheduler)
    const330  -- use_scheduler:=false fixed_n:=330  (predetermined equivalent-
                 budget control condition, locked a priori from Phase 0)
    fixed400  -- use_scheduler:=false fixed_n:=400  (conventional baseline)
"""
import argparse
import os
import re
import shutil
import signal
import subprocess
import sys
import threading
import time

CONDITIONS = {
    "adaptive": ["-p", "use_scheduler:=true"],
    "const330": ["-p", "use_scheduler:=false", "-p", "fixed_n:=330"],
    "fixed400": ["-p", "use_scheduler:=false", "-p", "fixed_n:=400"],
}

LOG_PATH_RE = re.compile(r"Logging to: (\S+\.csv)")
TRACKING_STARTED_MARKER = "Beginning MPPI tracking"


class NodeMonitor:
    """Reads offboard_node output on a background thread and exposes the
    CSV log path and the tracking-start event."""

    def __init__(self, process):
        self.process = process
        self.csv_path = None
        self.tracking_started = threading.Event()
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    def _reader(self):
        for line in self.process.stdout:
            line = line.rstrip()
            print(f"    [node] {line}")
            m = LOG_PATH_RE.search(line)
            if m:
                self.csv_path = m.group(1)
            if TRACKING_STARTED_MARKER in line:
                self.tracking_started.set()


def ros_env():
    env = dict(os.environ)
    # Force unbuffered rclcpp console output so we see log lines immediately
    # even though stdout is a pipe, not a terminal.
    env["RCUTILS_LOGGING_BUFFERED_STREAM"] = "0"
    env["RCUTILS_LOGGING_USE_STDOUT"] = "1"
    return env


def wait_for_disarm(timeout_sec):
    """Poll /mavros/state until armed: false (vehicle landed and disarmed)."""
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        try:
            out = subprocess.run(
                ["ros2", "topic", "echo", "--once", "/mavros/state"],
                capture_output=True, text=True, timeout=10, env=ros_env())
            if "armed: false" in out.stdout:
                return True
        except subprocess.TimeoutExpired:
            pass
        time.sleep(2)
    return False


def land_and_disarm(args):
    print("  Commanding AUTO.LAND...")
    subprocess.run(
        ["ros2", "service", "call", "/mavros/set_mode",
         "mavros_msgs/srv/SetMode", "{custom_mode: AUTO.LAND}"],
        capture_output=True, text=True, timeout=15, env=ros_env())
    if wait_for_disarm(args.land_timeout):
        print("  Vehicle disarmed.")
    else:
        print("  WARNING: vehicle did not disarm within timeout -- "
              "check the sim before the next trial.", file=sys.stderr)
    time.sleep(args.settle_time)


def next_trial_path(results_dir, condition):
    cond_dir = os.path.join(results_dir, condition)
    os.makedirs(cond_dir, exist_ok=True)
    k = 1
    while os.path.exists(os.path.join(cond_dir, f"trial_{k:02d}.csv")):
        k += 1
    return os.path.join(cond_dir, f"trial_{k:02d}.csv")


def run_one_trial(args):
    node_cmd = [
        "ros2", "run", "mppi_controller", "offboard_node",
        "--ros-args"] + CONDITIONS[args.condition]

    print(f"  Launching: {' '.join(node_cmd)}")
    node = subprocess.Popen(
        node_cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, bufsize=1, env=ros_env())
    monitor = NodeMonitor(node)

    try:
        if not monitor.tracking_started.wait(timeout=args.arm_timeout):
            raise RuntimeError(
                f"Node did not reach TRACKING within {args.arm_timeout}s "
                "(is PX4/MAVROS up?)")
        t_tracking_start = time.time()
        print(f"  Tracking started. Load window: "
              f"t+{args.load_offset}s for {args.load_duration}s "
              f"({args.load_threads} threads).")

        time.sleep(args.load_offset)
        print("  Starting CPU load generator...")
        load = subprocess.Popen(
            ["ros2", "run", "mppi_controller", "cpu_load_generator",
             str(args.load_threads), str(args.load_duration)],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            env=ros_env())

        remaining = args.tracking_duration - (time.time() - t_tracking_start)
        if remaining > 0:
            time.sleep(remaining)

        load.wait(timeout=args.load_duration + 30)
    finally:
        print("  Stopping offboard_node...")
        node.send_signal(signal.SIGINT)
        try:
            node.wait(timeout=15)
        except subprocess.TimeoutExpired:
            node.kill()
            node.wait()

    if not monitor.csv_path or not os.path.exists(monitor.csv_path):
        raise RuntimeError("Could not locate the flight CSV for this trial")

    dest = next_trial_path(args.results_dir, args.condition)
    shutil.copy2(monitor.csv_path, dest)
    print(f"  Saved: {dest}")
    return dest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--condition", required=True, choices=CONDITIONS)
    parser.add_argument("--trials", type=int, default=1)
    parser.add_argument("--results-dir", default="results")
    parser.add_argument("--load-threads", type=int, default=32)
    parser.add_argument("--load-duration", type=int, default=30,
                        help="load window length, seconds")
    parser.add_argument("--load-offset", type=int, default=20,
                        help="seconds after tracking starts to begin the load")
    parser.add_argument("--tracking-duration", type=int, default=90,
                        help="total tracking time per trial, seconds")
    parser.add_argument("--arm-timeout", type=int, default=120)
    parser.add_argument("--land-timeout", type=int, default=90)
    parser.add_argument("--settle-time", type=int, default=10,
                        help="pause after disarm before the next trial")
    args = parser.parse_args()

    if shutil.which("ros2") is None:
        sys.exit("ros2 not on PATH -- source the workspace first.")
    if args.load_offset + args.load_duration > args.tracking_duration:
        sys.exit("Load window extends past tracking duration -- adjust "
                 "--load-offset/--load-duration/--tracking-duration.")

    saved = []
    for i in range(args.trials):
        print(f"\n=== {args.condition} trial {i + 1}/{args.trials} ===")
        saved.append(run_one_trial(args))
        land_and_disarm(args)

    print(f"\nDone. {len(saved)} trial(s) saved:")
    for path in saved:
        print(f"  {path}")


if __name__ == "__main__":
    main()
