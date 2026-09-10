#!/usr/bin/env python3
"""
Analyse one ZED X live run: trajectory plot, motion statistics, timing table.

Reads a run folder produced by run_zedx_live.sh:

    ~/ov_results_zedx/zedx_<TIMESTAMP>/
        vio_path_run_1.csv   trajectory
        combined.log         timing log (--timing runs only)

Usage:
    python3 scripts/analyze_zedx_run.py                    # newest run
    python3 scripts/analyze_zedx_run.py <run_dir>          # a specific run
    python3 scripts/analyze_zedx_run.py --no-show          # save without displaying

Writes trajectory.png and analysis.txt into the run folder (or --out).
"""

import argparse
import glob
import os
import sys

import numpy as np
import pandas as pd

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)

DEFAULT_RESULTS_DIR = os.path.expanduser("~/ov_results_zedx")


def latest_run(results_dir):
    runs = [d for d in glob.glob(os.path.join(results_dir, "zedx_*")) if os.path.isdir(d)]
    if not runs:
        sys.exit(f"No zedx_* run folders in {results_dir}\n"
                 f"Record one with: ./scripts/run_zedx_live.sh")
    return max(runs, key=os.path.getmtime)


def load_path(run_dir, prefix):
    hits = sorted(glob.glob(os.path.join(run_dir, f"{prefix}_run_*.csv")))
    for f in hits:
        df = pd.read_csv(f)
        if not df.empty:
            return df, f
    return None, (hits[0] if hits else None)


def describe(df):
    """Motion statistics from a trajectory dataframe."""
    t = df["timestamp"].to_numpy()
    xyz = df[["x", "y", "z"]].to_numpy()

    steps = np.linalg.norm(np.diff(xyz, axis=0), axis=1)
    dt = np.diff(t)
    moving = dt > 0                      # duplicate stamps would divide by zero
    speed = np.zeros_like(steps)
    speed[moving] = steps[moving] / dt[moving]

    duration = float(t[-1] - t[0]) if len(t) > 1 else 0.0
    drift = float(np.linalg.norm(xyz[-1] - xyz[0]))

    return {
        "poses": len(df),
        "duration_s": duration,
        "rate_hz": (len(df) - 1) / duration if duration > 0 else float("nan"),
        "path_length_m": float(steps.sum()),
        "path_length_xy_m": float(np.linalg.norm(np.diff(xyz[:, :2], axis=0), axis=1).sum()),
        "straight_line_m": drift,
        "max_from_start_m": float(np.linalg.norm(xyz - xyz[0], axis=1).max()),
        "mean_speed_ms": float(speed.mean()) if len(speed) else float("nan"),
        "max_speed_ms": float(speed.max()) if len(speed) else float("nan"),
        "z_range_m": float(xyz[:, 2].max() - xyz[:, 2].min()),
        "z_final_m": float(xyz[-1, 2] - xyz[0, 2]),
        "_t": t, "_xyz": xyz, "_speed": speed,
    }


def format_stats(s, gt_stats=None):
    L = [
        f"  poses                 {s['poses']}",
        f"  duration              {s['duration_s']:.1f} s",
        f"  pose rate             {s['rate_hz']:.1f} Hz",
        f"  path length (3D)      {s['path_length_m']:.2f} m",
        f"  path length (XY)      {s['path_length_xy_m']:.2f} m",
        f"  start → end           {s['straight_line_m']:.2f} m",
        f"  max dist from start   {s['max_from_start_m']:.2f} m",
        f"  mean speed            {s['mean_speed_ms']:.2f} m/s",
        f"  max speed             {s['max_speed_ms']:.2f} m/s",
        f"  vertical range        {s['z_range_m']:.2f} m",
        f"  net vertical change   {s['z_final_m']:+.2f} m",
    ]
    # A run that returns to its start makes start→end a usable drift estimate.
    if s["max_from_start_m"] > 1e-6:
        closure = 100.0 * s["straight_line_m"] / s["max_from_start_m"]
        if closure < 25:
            L.append(f"  NOTE: ends within {closure:.0f}% of its farthest point — "
                     f"loop-like, so start→end ≈ {s['straight_line_m']:.2f} m of drift")
    if gt_stats:
        L.append(f"  GT path length        {gt_stats['path_length_m']:.2f} m")
    return "\n".join(L)


def timing_section(run_dir):
    log = os.path.join(run_dir, "combined.log")
    if not os.path.isfile(log):
        return None, "  no combined.log in this run (record one with --timing)"
    try:
        from parse_timing import parse_log, print_table
    except ImportError as e:
        return None, f"  could not import parse_timing.py ({e})"
    data = parse_log(log)
    if not any(data.values()):
        return None, f"  {os.path.basename(log)} holds no [TIMING]/[TIME] lines"
    return data, None


def make_plots(s, timing, out_png, title):
    import matplotlib
    if not os.environ.get("DISPLAY"):
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = s["_t"] - s["_t"][0]
    xyz = s["_xyz"]

    fig, ax = plt.subplots(2, 2, figsize=(13, 9))
    fig.suptitle(title, fontsize=13)

    a = ax[0][0]
    a.plot(xyz[:, 0], xyz[:, 1], lw=1.4, color="#2b6cb0")
    a.scatter(*xyz[0, :2], c="#2f855a", s=70, zorder=5, label="start")
    a.scatter(*xyz[-1, :2], c="#c53030", s=70, marker="X", zorder=5, label="end")
    a.set_xlabel("x [m]"); a.set_ylabel("y [m]"); a.set_title("Trajectory (top-down)")
    a.axis("equal"); a.grid(alpha=.3); a.legend()

    a = ax[0][1]
    a.plot(t, xyz[:, 2], lw=1.2, color="#805ad5")
    a.set_xlabel("time [s]"); a.set_ylabel("z [m]"); a.set_title("Height vs time")
    a.grid(alpha=.3)

    a = ax[1][0]
    a.plot(t[1:], s["_speed"], lw=1.0, color="#dd6b20")
    a.set_xlabel("time [s]"); a.set_ylabel("speed [m/s]"); a.set_title("Speed vs time")
    a.grid(alpha=.3)

    a = ax[1][1]
    if timing:
        # Only components this pipeline actually produced; ZED X runs have no HSE.
        measured = {k: v for k, v in timing.items() if v}
        names = list(measured.keys())
        means = [float(np.mean(measured[n])) for n in names]
        a.barh(names, means, color="#3182ce")
        a.axvline(50, color="#c53030", ls="--", lw=1, label="50 ms budget")
        a.set_xlabel("mean latency [ms]"); a.set_title("Timing"); a.legend()
        a.grid(alpha=.3, axis="x")
    else:
        for i, lbl in enumerate("xyz"):
            a.plot(t, xyz[:, i], lw=1.0, label=lbl)
        a.set_xlabel("time [s]"); a.set_ylabel("position [m]")
        a.set_title("Position vs time (no timing log)")
        a.grid(alpha=.3); a.legend()

    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(out_png, dpi=140)
    return plt


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run_dir", nargs="?", help="Run folder (default: newest under --results-dir)")
    ap.add_argument("--results-dir", default=DEFAULT_RESULTS_DIR,
                    help=f"Where run folders live (default: {DEFAULT_RESULTS_DIR})")
    ap.add_argument("--out", help="Where to write trajectory.png / analysis.txt (default: the run folder)")
    ap.add_argument("--no-show", action="store_true", help="Save without opening a window")
    args = ap.parse_args()

    run_dir = args.run_dir or latest_run(args.results_dir)
    if not os.path.isdir(run_dir):
        sys.exit(f"Not a directory: {run_dir}")
    out_dir = args.out or run_dir
    os.makedirs(out_dir, exist_ok=True)

    vio, vio_file = load_path(run_dir, "vio_path")
    if vio is None:
        sys.exit(f"No non-empty vio_path_run_*.csv in {run_dir}"
                 + ("" if vio_file is None else f"\n({vio_file} exists but is empty — "
                                                "the run recorded no poses)"))
    gt, _ = load_path(run_dir, "gt_path")

    stats = describe(vio)
    gt_stats = describe(gt) if gt is not None else None
    timing, timing_note = timing_section(run_dir)

    name = os.path.basename(os.path.normpath(run_dir))
    header = f"ZED X run: {name}"
    body = [header, "=" * len(header), "",
            f"folder     {run_dir}",
            f"trajectory {os.path.basename(vio_file)}",
            "", "Motion", "------", format_stats(stats, gt_stats), "", "Timing", "------"]

    # Build the figure before printing the table, so the plot is on disk even if
    # anything downstream misbehaves.
    png = os.path.join(out_dir, "trajectory.png")
    plt = make_plots(stats, timing, png, f"{name}  —  {stats['path_length_m']:.1f} m "
                                         f"over {stats['duration_s']:.0f} s")

    print("\n".join(body))
    if timing:
        from parse_timing import print_table
        print_table(timing, f"ZED X live — {name}")
        n_measured = sum(1 for v in timing.values() if v)
        body.append(f"  {n_measured} components measured; see the table above "
                    f"or re-run: python3 scripts/parse_timing.py {run_dir}/combined.log")
    else:
        print(timing_note)
        body.append(timing_note)

    txt = os.path.join(out_dir, "analysis.txt")
    with open(txt, "w") as f:
        f.write("\n".join(body) + "\n")

    print(f"\n  plot     → {png}")
    print(f"  analysis → {txt}")

    if not args.no_show:
        if os.environ.get("DISPLAY"):
            plt.show()
        else:
            print("  (no DISPLAY — saved only; copy the png off the Jetson to view)")


if __name__ == "__main__":
    main()
