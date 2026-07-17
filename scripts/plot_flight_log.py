#!/usr/bin/env python3
"""
Plot flight logs produced by offboard_node's CSV logging.

Usage:
    python3 plot_flight_log.py flight_log.csv
    python3 plot_flight_log.py adaptive_run.csv fixed_run.csv   # overlay comparison

Produces:
    - N over time
    - MPPI call time over time (with 50ms deadline line)
    - MAVROS round-trip time over time
    - Position tracking error over time
    - Summary stats printed to console
"""
import sys
import csv
import matplotlib.pyplot as plt


def load_csv(path):
    rows = []
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    return rows


def to_float_list(rows, key):
    out = []

    for r in rows:

        if key in r:
            out.append(float(r[key]))

        elif key == "state_to_command_latency_ms" and \
             "mavros_roundtrip_ms" in r:
            out.append(float(r["mavros_roundtrip_ms"]))

        else:
            raise KeyError(key)

    return out


def to_int_list(rows, key):
    return [int(r[key]) for r in rows]


def relative_time(rows):
    t0 = float(rows[0]["epoch_sec"])
    return [float(r["epoch_sec"]) - t0 for r in rows]


def tracking_only(rows):
    return [r for r in rows if r["phase"] == "TRACKING"]


def print_summary(label, rows):
    tracking = tracking_only(rows)
    if not tracking:
        print(f"{label}: no TRACKING rows found")
        return

    call_times = to_float_list(tracking, "mppi_call_ms")
    latency = to_float_list(
        tracking,
        "state_to_command_latency_ms")
    errors = to_float_list(tracking, "pos_error_m")
    n_values = to_int_list(tracking, "N")
    misses = sum(int(r["deadline_miss"]) for r in tracking)

    print(f"\n=== {label} ===")
    print(f"  Tracking-phase samples: {len(tracking)}")
    print(f"  Mean N: {sum(n_values)/len(n_values):.1f}")
    print(f"  Mean MPPI call time: {sum(call_times)/len(call_times):.2f} ms")
    print(
        f"  Mean state-to-command latency: "
        f"{sum(latency)/len(latency):.2f} ms")
    print(f"  Mean position error: {sum(errors)/len(errors):.4f} m")
    print(f"  RMS position error: {(sum(e**2 for e in errors)/len(errors))**0.5:.4f} m")
    print(f"  Deadline misses: {misses} ({100*misses/len(tracking):.1f}%)")


def plot_comparison(files):
    labels = [f.split("/")[-1] for f in files]
    all_rows = [load_csv(f) for f in files]

    for label, rows in zip(labels, all_rows):
        print_summary(label, rows)

    fig, axes = plt.subplots(4, 1, figsize=(10, 14), sharex=False)

    for rows, label in zip(all_rows, labels):
        tracking = tracking_only(rows)
        if not tracking:
            continue
        t = relative_time(tracking)

        axes[0].plot(t, to_int_list(tracking, "N"), label=label, alpha=0.8)
        axes[1].plot(t, to_float_list(tracking, "mppi_call_ms"), label=label, alpha=0.8)
        axes[2].plot(t, to_float_list(tracking, "state_to_command_latency_ms"), label=label, alpha=0.8)
        axes[3].plot(t, to_float_list(tracking, "pos_error_m"), label=label, alpha=0.8)

    axes[0].set_ylabel("N (sample count)")
    axes[0].set_title("Sample Count Over Time")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].axhline(y=50, color='red', linestyle='--', alpha=0.5, label='50ms deadline')
    axes[1].set_ylabel("MPPI call time (ms)")
    axes[1].set_title("MPPI Computation Time")
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    axes[2].set_ylabel("State→command latency (ms)")
    axes[2].set_title("State-to-command latency")
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)

    axes[3].set_ylabel("Position error (m)")
    axes[3].set_xlabel("Time (s)")
    axes[3].set_title("Tracking Error Over Time")
    axes[3].legend()
    axes[3].grid(True, alpha=0.3)

    plt.tight_layout()
    output_path = "flight_comparison.png"
    plt.savefig(output_path, dpi=150)
    print(f"\nSaved plot to {output_path}")
    plt.show()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 plot_flight_log.py <csv_file> [<csv_file2> ...]")
        sys.exit(1)

    plot_comparison(sys.argv[1:])