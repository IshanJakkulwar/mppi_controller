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
    "adaptiveqf": ["-p", "use_scheduler:=true", "-p", "quality_floor_n:=150"],
    "const200": ["-p", "use_scheduler:=false", "-p", "fixed_n:=200"],
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


def popen_group(cmd, prefix, cwd=None, extra_env=None, stdout_file=None):
    """Launch cmd in its own process group (so we can kill the whole tree).

    stdout_file=None: merged stdout/stderr piped for line monitoring (use
    with LineMonitor, which drains the pipe -- an undrained pipe blocks the
    child after ~64KB).
    stdout_file=<path>: merged output appended to that file instead (for
    chatty long-lived processes like PX4/Gazebo/MAVROS).

    stdin is always a held-open pipe: PX4's pxh shell busy-spins on stdin
    EOF (verified: /dev/null stdin produced a ~512MB prompt-loop log in
    under a minute), so never give these processes /dev/null stdin.
    """
    env = ros_env()
    if extra_env:
        env.update(extra_env)
    if stdout_file is not None:
        out = open(stdout_file, "ab", buffering=0)
        proc = subprocess.Popen(
            cmd, cwd=cwd, stdin=subprocess.PIPE, stdout=out,
            stderr=subprocess.STDOUT, env=env, start_new_session=True)
        out.close()  # child holds its own fd
    else:
        proc = subprocess.Popen(
            cmd, cwd=cwd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True, bufsize=1, env=env,
            start_new_session=True)
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


# Patterns that identify sim-stack processes. Specific on purpose: a bare
# "px4" or "ruby" would kill unrelated processes (this very script's path
# contains no such string, but a user's editor or shell might).
SIM_PROC_PATTERNS = ["bin/px4", "gz sim", "gzserver", "px4_sitl",
                     "mavros_node", "make px4_sitl"]


def find_sim_procs():
    """Return [(pid, cmdline)] of live sim-stack processes, excluding
    ourselves and shell wrappers that merely mention the pattern."""
    out = subprocess.run(["ps", "-eo", "pid,args"], capture_output=True,
                         text=True).stdout
    procs = []
    for line in out.splitlines()[1:]:
        line = line.strip()
        pid_str, _, cmdline = line.partition(" ")
        if not any(p in cmdline for p in SIM_PROC_PATTERNS):
            continue
        if "bash -c" in cmdline or "run_trial.py" in cmdline \
                or "ps -eo" in cmdline:
            continue
        procs.append((int(pid_str), cmdline.strip()))
    return procs


def hard_cleanup_sim():
    """Safety net: kill any lingering sim-stack processes. Runs on teardown
    so a fresh trial never inherits a stale sim."""
    for pid, _ in find_sim_procs():
        try:
            os.kill(pid, signal.SIGKILL)
        except (ProcessLookupError, PermissionError):
            pass
    time.sleep(3)


def preflight_check():
    """restart-sim owns the sim lifecycle. If a sim stack is already running
    (hand-launched, or orphaned by a previous crash/Ctrl-C), a fresh PX4
    will hit an instance conflict and exit cryptically -- so kill it here,
    loudly, before the first trial."""
    procs = find_sim_procs()
    if not procs:
        return
    print("  WARNING: found an already-running sim stack. restart-sim mode "
          "owns the sim lifecycle, so these will be terminated now:",
          file=sys.stderr)
    for pid, cmdline in procs:
        print(f"    pid {pid}: {cmdline[:90]}", file=sys.stderr)
    print("  (If you meant to use your own hand-launched sim, Ctrl-C now "
          "and re-run with --no-restart.)", file=sys.stderr)
    time.sleep(5)  # grace window to Ctrl-C before we pull the trigger
    hard_cleanup_sim()


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


PX4_READY_MARKER = b"Startup script returned successfully"
MAVROS_CONNECTED_MARKER = b"Got HEARTBEAT"


# --------------------------------------------------------------------------
# HITL mode (see HITL_PLAN.md). UNTESTED UNTIL HARDWARE IS AVAILABLE --
# the first run MUST be supervised. Lifecycle per trial: launch Gazebo
# Classic's HITL world (which owns the FC's USB/serial HIL link) + MAVROS on
# the TELEM2 FTDI serial; run the trial; tear both down and reboot the FC
# so every trial starts from a fresh autopilot state. PX4 SITL is never
# launched in this mode -- the real flight controller replaces it.
# --------------------------------------------------------------------------
def hitl_bring_up(args, log_dir):
    """Launch Gazebo Classic HITL world + MAVROS (serial). Returns
    (gazebo_proc, mavros_proc)."""
    for dev, what in [(args.hitl_sim_device, "FC USB (HIL link)"),
                      (args.mavros_serial.split(":")[0], "MAVROS FTDI")]:
        if not os.path.exists(dev):
            raise RuntimeError(f"{what} device {dev} not present -- "
                               "check cables/HITL_PLAN.md wiring")
    stamp = time.strftime("%Y%m%d_%H%M%S")
    gz_log = os.path.join(log_dir, f"{stamp}_gazebo.log")
    mavros_log = os.path.join(log_dir, f"{stamp}_mavros.log")

    print(f"  [HITL] Launching Gazebo Classic HITL world; log: {gz_log}")
    setup = os.path.join(args.px4_dir,
                         "Tools/simulation/gazebo-classic/setup_gazebo.bash")
    build = os.path.join(args.px4_dir, "build/px4_sitl_default")
    gazebo = popen_group(
        ["bash", "-c",
         f"source {setup} {args.px4_dir} {build} && "
         f"exec gazebo --verbose {args.hitl_world}"],
        prefix="gazebo", stdout_file=gz_log)
    time.sleep(args.hitl_gazebo_warmup)
    if gazebo.poll() is not None:
        raise RuntimeError("Gazebo Classic exited during startup. Log "
                           "tail:\n" + tail_bytes(gz_log))

    print(f"  [HITL] Launching MAVROS on {args.mavros_serial}; "
          f"log: {mavros_log}")
    mavros = popen_group(
        ["ros2", "launch", "mavros", "px4.launch",
         f"fcu_url:=serial://{args.mavros_serial}"],
        prefix="mavros", stdout_file=mavros_log)

    deadline = time.time() + args.connect_timeout
    while time.time() < deadline:
        try:
            with open(mavros_log, "rb") as f:
                if MAVROS_CONNECTED_MARKER in f.read():
                    print("  [HITL] MAVROS connected to the FC.")
                    time.sleep(3)
                    return gazebo, mavros
        except OSError:
            pass
        if gazebo.poll() is not None:
            raise RuntimeError("Gazebo died while waiting for MAVROS. Log "
                               "tail:\n" + tail_bytes(gz_log))
        if mavros.poll() is not None:
            raise RuntimeError("MAVROS exited. Log tail:\n"
                               + tail_bytes(mavros_log))
        time.sleep(2)
    raise RuntimeError(
        f"MAVROS heartbeat not seen within {args.connect_timeout}s. "
        "Log tail:\n" + tail_bytes(mavros_log))


def hitl_tear_down(gazebo, mavros, args):
    """Reboot the FC (fresh autopilot state per trial), then stop MAVROS
    and Gazebo."""
    print("  [HITL] Rebooting flight controller...")
    subprocess.run(
        ["ros2", "service", "call", "/mavros/cmd/command",
         "mavros_msgs/srv/CommandLong", "{command: 246, param1: 1}"],
        capture_output=True, text=True, timeout=15, env=ros_env())
    time.sleep(3)
    kill_group(mavros)
    kill_group(gazebo)
    # FC re-enumeration after reboot takes ~10-15s; wait so the next
    # trial's device check doesn't race it.
    time.sleep(args.hitl_reboot_wait)


def tail_bytes(path, n=2000):
    try:
        with open(path, "rb") as f:
            f.seek(0, os.SEEK_END)
            size = f.tell()
            f.seek(max(0, size - n))
            return f.read().decode(errors="replace")
    except OSError:
        return "(no log)"


def bring_up_sim(args, log_dir):
    """Launch PX4 SITL (+Gazebo) and MAVROS; wait for PX4's startup marker,
    then for the MAVROS FCU connection. Returns (px4_proc, mavros_proc).
    Sim output goes to log files in log_dir, not pipes."""
    stamp = time.strftime("%Y%m%d_%H%M%S")
    px4_log = os.path.join(log_dir, f"{stamp}_px4.log")
    mavros_log = os.path.join(log_dir, f"{stamp}_mavros.log")

    print(f"  Bringing up PX4 SITL + Gazebo "
          f"({'headless' if args.headless else 'with GUI'}); "
          f"log: {px4_log}")
    px4 = popen_group(
        ["make", "px4_sitl", args.px4_target],
        prefix="px4", cwd=args.px4_dir, stdout_file=px4_log,
        extra_env={"HEADLESS": "1"} if args.headless else None)

    # Wait for PX4's own readiness marker rather than a blind sleep.
    deadline = time.time() + args.px4_boot_timeout
    ready = False
    while time.time() < deadline:
        if px4.poll() is not None:
            raise RuntimeError(
                "PX4 exited during startup. Last log lines:\n"
                + tail_bytes(px4_log))
        try:
            with open(px4_log, "rb") as f:
                if PX4_READY_MARKER in f.read():
                    ready = True
                    break
        except OSError:
            pass
        time.sleep(2)
    if not ready:
        raise RuntimeError(
            f"PX4 not ready within {args.px4_boot_timeout}s. Last log "
            "lines:\n" + tail_bytes(px4_log))
    print("  PX4 startup script completed.")

    print(f"  Launching MAVROS; log: {mavros_log}")
    mavros = popen_group(
        ["ros2", "launch", "mavros", "px4.launch",
         f"fcu_url:={args.fcu_url}"],
        prefix="mavros", stdout_file=mavros_log)

    # Gate on MAVROS's own heartbeat log line rather than a `ros2 topic
    # echo` probe: the CLI depends on the ros2 daemon's discovery cache and
    # was observed reporting "topic not published yet" while MAVROS was in
    # fact connected -- the log marker is authoritative.
    print("  Waiting for MAVROS FCU connection (heartbeat in log)...")
    deadline = time.time() + args.connect_timeout
    while time.time() < deadline:
        try:
            with open(mavros_log, "rb") as f:
                if MAVROS_CONNECTED_MARKER in f.read():
                    print("  MAVROS connected to PX4.")
                    # brief settle so subscribers/publishers finish wiring
                    time.sleep(3)
                    return px4, mavros
        except OSError:
            pass
        if px4.poll() is not None:
            raise RuntimeError("PX4 died while waiting for MAVROS. Last "
                               "log lines:\n" + tail_bytes(px4_log))
        if mavros.poll() is not None:
            raise RuntimeError("MAVROS exited during startup. Last log "
                               "lines:\n" + tail_bytes(mavros_log))
        time.sleep(2)
    raise RuntimeError(
        f"MAVROS did not connect within {args.connect_timeout}s. MAVROS "
        "log tail:\n" + tail_bytes(mavros_log))


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
    # Optional CPU pinning: restrict the controller node AND the load
    # generator to the same core set (taskset), emulating an embedded
    # companion computer's compute envelope (e.g. a 4-6 core Jetson). On a
    # 16-hw-thread dev machine, unpinned busy-wait load never breaches the
    # deadline -- CFS's sleeper fairness protects the periodic MPPI thread
    # regardless of thread count (verified: 32 and 64 threads both left
    # fixed-N=400 at ~37ms, zero in-window misses). PX4/Gazebo/MAVROS stay
    # unpinned (conceptually a separate machine).
    pin = ["taskset", "-c", args.cpuset] if args.cpuset else []

    node_cmd = (pin + ["ros2", "run", "mppi_controller", "offboard_node",
                       "--ros-args"] + CONDITIONS[args.condition]
                + ["-p", f"loop_rate_hz:={args.loop_hz}"])
    if args.inject_stall:
        t_sec, ms, cycles = args.inject_stall.split(",")
        node_cmd += ["-p", f"inject_stall_t_sec:={float(t_sec)}",
                     "-p", f"inject_stall_ms:={int(ms)}",
                     "-p", f"inject_stall_cycles:={int(cycles)}"]
    print(f"  Launching node: {' '.join(node_cmd)}")
    node = popen_group(node_cmd, prefix="node")
    monitor = LineMonitor(node, "node")

    try:
        if not monitor.tracking_started.wait(timeout=args.arm_timeout):
            raise RuntimeError(
                f"Node did not reach TRACKING within {args.arm_timeout}s")
        t_start = time.time()
        print(f"  Tracking started. Load window: t+{args.load_offset}s for "
              f"{args.load_duration}s ({args.load_threads} threads"
              + (f", cpuset {args.cpuset}" if args.cpuset else "") + ").")

        time.sleep(args.load_offset)
        print("  Starting CPU load generator...")
        load = popen_group(
            pin + ["ros2", "run", "mppi_controller", "cpu_load_generator",
                   str(args.load_threads), str(args.load_duration),
                   args.load_mode],
            prefix="load", stdout_file=os.devnull)

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
            if args.hitl:
                px4, mavros = hitl_bring_up(args, args.sim_log_dir)
            elif args.restart_sim:
                px4, mavros = bring_up_sim(args, args.sim_log_dir)
            run_node_trial(args, dest)
        except Exception as exc:  # noqa: BLE001 -- report and retry
            print(f"  Trial attempt {attempt} errored: {exc}", file=sys.stderr)
            if args.hitl:
                hitl_tear_down(px4, mavros, args)
            elif args.restart_sim:
                tear_down_sim(px4, mavros)
            if attempt <= args.max_retries:
                print("  Retrying...")
                continue
            return None

        # Reached only on a successful run (no exception).
        ok, reason = validate_csv(dest)
        if args.hitl:
            hitl_tear_down(px4, mavros, args)
        elif args.restart_sim:
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

    # HITL mode (HITL_PLAN.md; UNTESTED until hardware -- supervise the
    # first run). Real flight controller replaces PX4 SITL; Gazebo Classic
    # provides physics over the FC's USB serial; MAVROS talks to TELEM2.
    parser.add_argument("--hitl", action="store_true", default=False)
    parser.add_argument("--hitl-world",
                        default=os.path.expanduser(
                            "~/PX4-Autopilot/Tools/simulation/gazebo-classic/"
                            "sitl_gazebo-classic/worlds/hitl_iris.world"))
    parser.add_argument("--hitl-sim-device", default="/dev/ttyACM0",
                        help="FC USB device carrying the HIL sensor stream")
    parser.add_argument("--mavros-serial", default="/dev/ttyUSB0:921600",
                        help="MAVROS FTDI serial device:baud (TELEM2)")
    parser.add_argument("--hitl-gazebo-warmup", type=int, default=15)
    parser.add_argument("--hitl-reboot-wait", type=int, default=20,
                        help="seconds to wait for FC re-enumeration after "
                             "the between-trial reboot")

    # load window -- defaults are the CALIBRATED stress condition (see
    # PROJECT_PLAN.md "Stress-condition calibration"): controller + load
    # pinned to 2 cores (embedded-class envelope), 6 busy threads. On this
    # 16-hw-thread machine, unpinned load of any size never breaches the
    # deadline (CFS/EEVDF shields the periodic MPPI thread), and 1 core
    # destabilizes flight entirely; 2 cores / 6 threads yields fixed-N=400
    # call times of ~40ms mean / ~53ms peak with a ~4% in-window miss rate
    # while flight remains valid.
    parser.add_argument("--cpuset", default="0-1",
                        help="pin node+load to these cores (emulates "
                             "embedded compute envelope); '' disables")
    parser.add_argument("--load-threads", type=int, default=6)
    parser.add_argument("--load-mode", choices=["spin", "mem"],
                        default="spin",
                        help="spin = ALU busy-wait (primary condition); "
                             "mem = 64MB cache/bandwidth thrash per thread "
                             "(perception-like, 2B-extended condition)")
    parser.add_argument("--inject-stall", default=None, metavar="T,MS,CYC",
                        help="fault injection for safety-fallback "
                             "validation, e.g. '60,60,3' = 60ms synthetic "
                             "stall for 3 cycles starting t+60s")
    parser.add_argument("--loop-hz", type=float, default=20.0,
                        help="control loop rate / deadline (20 = primary "
                             "50ms condition; 40 = 2B-extended 25ms "
                             "tight-deadline condition)")
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
    parser.add_argument("--px4-boot-timeout", type=int, default=90,
                        help="max seconds to wait for PX4's startup marker")
    parser.add_argument("--connect-timeout", type=int, default=120)
    parser.add_argument("--sim-log-dir", default="results/sim_logs",
                        help="where PX4/MAVROS boot logs are written")

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

    mode = ("HITL" if args.hitl
            else "restart-sim" if args.restart_sim else "no-restart")
    print(f"Reset strategy: {mode}. Condition: {args.condition}. "
          f"Trials: {args.trials}.")

    if args.hitl:
        print("  *** HITL MODE IS UNTESTED UNTIL HARDWARE IS AVAILABLE. ***\n"
              "  *** SUPERVISE THE FIRST RUN (see HITL_PLAN.md). ***",
              file=sys.stderr)
        os.makedirs(args.sim_log_dir, exist_ok=True)
    elif args.restart_sim:
        os.makedirs(args.sim_log_dir, exist_ok=True)
        preflight_check()

    saved = []
    try:
        for i in range(args.trials):
            print(f"\n=== {args.condition} trial {i + 1}/{args.trials} ===")
            path = run_one_trial_with_retries(args)
            if path:
                saved.append(path)
            else:
                print(f"  Trial {i + 1} FAILED after retries -- see log "
                      "above.", file=sys.stderr)
    except KeyboardInterrupt:
        print("\nInterrupted -- cleaning up sim processes before exit...",
              file=sys.stderr)
    finally:
        if args.restart_sim and not args.hitl:
            # Never leave orphaned PX4/Gazebo/MAVROS behind (a previous
            # version did, and the stale instance broke the next run).
            hard_cleanup_sim()

    print(f"\nDone. {len(saved)}/{args.trials} valid trial(s) saved:")
    for path in saved:
        print(f"  {path}")
    if len(saved) < args.trials:
        sys.exit(1)


if __name__ == "__main__":
    main()
