#!/usr/bin/env python3
"""
Automated Phase 2 trial runner.

Runs offboard_node under one of the three experimental conditions, fires the
CPU load generator at a fixed offset after MPPI tracking begins (so the load
window lands identically in every trial), collects the flight CSV into
results/<condition>/trial_<k>.csv, validates the trial, and resets the
initial condition between trials.

RESET STRATEGY (see PROJECT_PLAN.md, Phase 2 -- decided after the Phase 2A
pilot found the vehicle was not resetting between trials and would
occasionally get wedged in a bad state that persisted even after load ended):

  --restart-sim  (RECOMMENDED, default): each trial gets a fresh PX4 SITL +
      Gazebo + MAVROS stack, torn down completely afterwards. Guarantees an
      identical, uncontaminated initial condition and independent trials.
      Requires this script to own the sim lifecycle -- do NOT also launch
      PX4/MAVROS by hand in this mode.

  --no-restart: assumes PX4 SITL + Gazebo + MAVROS are already running (you
      launched them by hand). Faster per trial, but state can leak between
      trials -- only use for quick single-trial checks, not the real batch.

VALIDITY GUARD: after each trial the CSV is checked for (a) completion of the
waypoint circuit (drone returns to the final (0,0) target within tolerance)
and (b) reaching steady state (error < 0.5m for 10 consecutive cycles). A
trial that fails either is flagged invalid and re-run, up to --max-retries,
so intermittent stalls never silently pollute the dataset.

Usage (restart-sim, the real batch):
    python3 run_trial.py --condition adaptive --trials 3
    python3 run_trial.py --condition const330 --trials 3
    python3 run_trial.py --condition fixed400 --trials 3

Usage (quick check against an already-running sim):
    python3 run_trial.py --condition adaptive --trials 1 --no-restart

Conditions (see PROJECT_PLAN.md, Phase 2):
    adaptive  -- use_scheduler:=true  (AnytimeScheduler)
    const330  -- use_scheduler:=false fixed_n:=330  (predetermined equivalent-
                 budget control condition, locked a priori from Phase 0)
    fixed400  -- use_scheduler:=false fixed_n:=400  (conventional baseline)
"""
import argparse
import csv
import math
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

# Validity thresholds -- mirror analyze_trials.py's steady-state definition.
SS_THRESHOLD_M = 0.5
SS_CONSECUTIVE = 10
FINAL_TARGET_TOL_M = 0.4  # drone must get this close to the final (0,0) target


def ros_env():
    env = dict(os.environ)
    env["RCUTILS_LOGGING_BUFFERED_STREAM"] = "0"
    env["RCUTILS_LOGGING_USE_STDOUT"] = "1"
    return env


# --------------------------------------------------------------------------
# Process helpers
# --------------------------------------------------------------------------
class LineMonitor:
    """Reads a process's merged stdout on a background thread, echoes it with
    a prefix, and exposes matched markers."""

    def __init__(self, process, prefix):
        self.process = process
        self.prefix = prefix
        self.csv_path = None
        self.tracking_started = threading.Event()
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    def _reader(self):
        for line in self.process.stdout:
            line = line.rstrip()
            print(f"    [{self.prefix}] {line}")
            m = LOG_PATH_RE.search(line)
            if m:
                self.csv_path = m.group(1)
            if TRACKING_STARTED_MARKER in line:
                self.tracking_started.set()


def popen_group(cmd, prefix, cwd=None, extra_env=None):
    """Launch cmd in its own process group (so we can kill the whole tree),
    with merged stdout/stderr piped for monitoring."""
    env = ros_env()
    if extra_env:
        env.update(extra_env)
    proc = subprocess.Popen(
        cmd, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, bufsize=1, env=env, start_new_session=True)
    return proc


def kill_group(proc, timeout=10):
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=timeout)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            proc.wait(timeout=timeout)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            pass


def hard_cleanup_sim():
    """Safety net: kill any lingering PX4/Gazebo/MAVROS processes by name.
    Runs on teardown so a fresh trial never inherits a stale sim."""
    for name in ["px4", "gz sim", "gzserver", "ruby", "mavros_node",
                 "px4_sitl"]:
        subprocess.run(["pkill", "-9", "-f", name],
                       capture_output=True, text=True)
    time.sleep(3)


# --------------------------------------------------------------------------
# Sim lifecycle (restart-sim mode)
# --------------------------------------------------------------------------
def mavros_connected(timeout=1.5):
    """True if /mavros/state reports connected: true."""
    try:
        out = subprocess.run(
            ["ros2", "topic", "echo", "--once", "/mavros/state"],
            capture_output=True, text=True, timeout=timeout, env=ros_env())
        return "connected: true" in out.stdout.lower()
    except subprocess.TimeoutExpired:
        return False


def bring_up_sim(args):
    """Launch PX4 SITL (+Gazebo) and MAVROS; wait for FCU connection.
    Returns (px4_proc, mavros_proc)."""
    print("  Bringing up PX4 SITL + Gazebo (HEADLESS)...")
    px4 = popen_group(
        ["make", "px4_sitl", args.px4_target],
        prefix="px4", cwd=args.px4_dir,
        extra_env={"HEADLESS": "1"} if args.headless else None)

    # Give PX4/Gazebo time to spin up before MAVROS tries to connect.
    time.sleep(args.px4_warmup)

    print("  Launching MAVROS...")
    mavros = popen_group(
        ["ros2", "launch", "mavros", "px4.launch",
         f"fcu_url:={args.fcu_url}"],
        prefix="mavros")

    print("  Waiting for MAVROS FCU connection...")
    deadline = time.time() + args.connect_timeout
    while time.time() < deadline:
        if mavros_connected():
            print("  MAVROS connected to PX4.")
            return px4, mavros
        if px4.poll() is not None:
            raise RuntimeError("PX4 exited during startup")
        time.sleep(3)
    raise RuntimeError(
        f"MAVROS did not connect within {args.connect_timeout}s")


def tear_down_sim(px4, mavros):
    print("  Tearing down sim stack...")
    kill_group(mavros)
    kill_group(px4)
    hard_cleanup_sim()


# --------------------------------------------------------------------------
# Trial validity
# --------------------------------------------------------------------------
def validate_csv(path):
    """Return (ok, reason). Checks the drone completed the circuit and
    reached steady state -- catches the intermittent stall seen in the
    Phase 2A pilot."""
    with open(path) as f:
        tr = [r for r in csv.DictReader(f) if r["phase"] == "TRACKING"]
    if len(tr) < 100:
        return False, f"only {len(tr)} tracking cycles"

    errors = [float(r["pos_error_m"]) for r in tr]

    # steady state reached?
    run = 0
    reached_ss = False
    for e in errors:
        run = run + 1 if e < SS_THRESHOLD_M else 0
        if run >= SS_CONSECUTIVE:
            reached_ss = True
            break
    if not reached_ss:
        return False, "steady state never reached (likely stall)"

    # circuit completed: final target is (0,0) and drone got close to it?
    last = tr[-1]
    if abs(float(last["target_x"])) > 1e-6 or abs(float(last["target_y"])) > 1e-6:
        return False, "did not advance to final (0,0) waypoint"
    if float(last["pos_error_m"]) > FINAL_TARGET_TOL_M:
        return False, (f"ended {float(last['pos_error_m']):.2f}m from final "
                       "target (did not settle)")
    return True, "ok"


# --------------------------------------------------------------------------
# One trial (node + load + collect), sim assumed up
# --------------------------------------------------------------------------
def run_node_trial(args, dest_path):
    node_cmd = (["ros2", "run", "mppi_controller", "offboard_node",
                 "--ros-args"] + CONDITIONS[args.condition])
    print(f"  Launching node: {' '.join(node_cmd)}")
    node = popen_group(node_cmd, prefix="node")
    monitor = LineMonitor(node, "node")

    try:
        if not monitor.tracking_started.wait(timeout=args.arm_timeout):
            raise RuntimeError(
                f"Node did not reach TRACKING within {args.arm_timeout}s")
        t_start = time.time()
        print(f"  Tracking started. Load window: t+{args.load_offset}s for "
              f"{args.load_duration}s ({args.load_threads} threads).")

        time.sleep(args.load_offset)
        print("  Starting CPU load generator...")
        load = popen_group(
            ["ros2", "run", "mppi_controller", "cpu_load_generator",
             str(args.load_threads), str(args.load_duration)], prefix="load")

        remaining = args.tracking_duration - (time.time() - t_start)
        if remaining > 0:
            time.sleep(remaining)
        try:
            load.wait(timeout=args.load_duration + 30)
        except subprocess.TimeoutExpired:
            kill_group(load)
    finally:
        print("  Stopping node...")
        kill_group(node)

    if not monitor.csv_path or not os.path.exists(monitor.csv_path):
        raise RuntimeError("Could not locate the flight CSV for this trial")
    shutil.copy2(monitor.csv_path, dest_path)
    return dest_path


def next_trial_path(results_dir, condition):
    cond_dir = os.path.join(results_dir, condition)
    os.makedirs(cond_dir, exist_ok=True)
    k = 1
    while os.path.exists(os.path.join(cond_dir, f"trial_{k:02d}.csv")):
        k += 1
    return os.path.join(cond_dir, f"trial_{k:02d}.csv")


def run_one_trial_with_retries(args):
    """Runs a valid trial, restarting the sim (if in restart-sim mode) and
    retrying on validation failure. Returns the saved path or None."""
    dest = next_trial_path(args.results_dir, args.condition)
    for attempt in range(1, args.max_retries + 2):
        px4 = mavros = None
        try:
            if args.restart_sim:
                px4, mavros = bring_up_sim(args)
            run_node_trial(args, dest)
        except Exception as exc:  # noqa: BLE001 -- report and retry
            print(f"  Trial attempt {attempt} errored: {exc}", file=sys.stderr)
            if args.restart_sim:
                tear_down_sim(px4, mavros)
            if attempt <= args.max_retries:
                print("  Retrying...")
                continue
            return None

        # Reached only on a successful run (no exception).
        ok, reason = validate_csv(dest)
        if args.restart_sim:
            tear_down_sim(px4, mavros)
        else:
            reset_in_sim(args)

        if ok:
            print(f"  VALID trial saved: {dest}")
            return dest
        print(f"  INVALID trial ({reason}) -- discarding "
              f"{os.path.basename(dest)}", file=sys.stderr)
        os.remove(dest)
        if attempt > args.max_retries:
            print("  Out of retries.", file=sys.stderr)
            return None
        print(f"  Retry {attempt}/{args.max_retries}...")
    return None


def reset_in_sim(args):
    """No-restart mode only: command AUTO.LAND and wait for the drone to
    actually descend (poll altitude), not just for a possibly-stale disarm
    flag. Best-effort -- restart-sim is the reliable path."""
    print("  (no-restart) Commanding AUTO.LAND and waiting for descent...")
    subprocess.run(
        ["ros2", "service", "call", "/mavros/set_mode",
         "mavros_msgs/srv/SetMode", "{custom_mode: AUTO.LAND}"],
        capture_output=True, text=True, timeout=15, env=ros_env())
    deadline = time.time() + args.land_timeout
    while time.time() < deadline:
        try:
            out = subprocess.run(
                ["ros2", "topic", "echo", "--once",
                 "/mavros/local_position/pose"],
                capture_output=True, text=True, timeout=8, env=ros_env())
            m = re.search(r"z:\s*(-?\d+\.?\d*)", out.stdout)
            if m and float(m.group(1)) < 0.3:
                print("  Drone landed.")
                time.sleep(args.settle_time)
                return
        except subprocess.TimeoutExpired:
            pass
        time.sleep(2)
    print("  WARNING: drone did not confirm landing -- state may leak into "
          "the next trial. Prefer --restart-sim.", file=sys.stderr)
    time.sleep(args.settle_time)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--condition", required=True, choices=CONDITIONS)
    parser.add_argument("--trials", type=int, default=1)
    parser.add_argument("--results-dir", default="results")

    # reset strategy
    parser.add_argument("--restart-sim", dest="restart_sim",
                        action="store_true", default=True,
                        help="fresh PX4/Gazebo/MAVROS per trial (default)")
    parser.add_argument("--no-restart", dest="restart_sim",
                        action="store_false",
                        help="assume the sim is already running")

    # load window
    parser.add_argument("--load-threads", type=int, default=32)
    parser.add_argument("--load-duration", type=int, default=30)
    parser.add_argument("--load-offset", type=int, default=20,
                        help="seconds after tracking start to begin the load")
    parser.add_argument("--tracking-duration", type=int, default=90,
                        help="total tracking time per trial, seconds")

    # validity / retries
    parser.add_argument("--max-retries", type=int, default=2,
                        help="re-runs allowed per trial on validation failure")

    # sim lifecycle (restart-sim mode)
    parser.add_argument("--px4-dir",
                        default=os.path.expanduser("~/PX4-Autopilot"))
    parser.add_argument("--px4-target", default="gz_x500")
    parser.add_argument("--fcu-url",
                        default="udp://:14540@127.0.0.1:14557")
    parser.add_argument("--headless", action="store_true", default=True)
    parser.add_argument("--no-headless", dest="headless",
                        action="store_false")
    parser.add_argument("--px4-warmup", type=int, default=25,
                        help="seconds to let PX4/Gazebo boot before MAVROS")
    parser.add_argument("--connect-timeout", type=int, default=120)

    # timeouts
    parser.add_argument("--arm-timeout", type=int, default=120)
    parser.add_argument("--land-timeout", type=int, default=90)
    parser.add_argument("--settle-time", type=int, default=10)
    args = parser.parse_args()

    if shutil.which("ros2") is None:
        sys.exit("ros2 not on PATH -- source the workspace first.")
    if args.load_offset + args.load_duration > args.tracking_duration:
        sys.exit("Load window extends past tracking duration -- adjust "
                 "--load-offset/--load-duration/--tracking-duration.")
    if args.restart_sim and not os.path.isdir(args.px4_dir):
        sys.exit(f"--px4-dir {args.px4_dir} not found. Pass --px4-dir or "
                 "use --no-restart with a hand-launched sim.")

    mode = "restart-sim" if args.restart_sim else "no-restart"
    print(f"Reset strategy: {mode}. Condition: {args.condition}. "
          f"Trials: {args.trials}.")

    saved = []
    for i in range(args.trials):
        print(f"\n=== {args.condition} trial {i + 1}/{args.trials} ===")
        path = run_one_trial_with_retries(args)
        if path:
            saved.append(path)
        else:
            print(f"  Trial {i + 1} FAILED after retries -- see log above.",
                  file=sys.stderr)

    print(f"\nDone. {len(saved)}/{args.trials} valid trial(s) saved:")
    for path in saved:
        print(f"  {path}")
    if len(saved) < args.trials:
        sys.exit(1)


if __name__ == "__main__":
    main()
