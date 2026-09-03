#!/usr/bin/env python3
"""
kaist_gt_to_csv.py
Convert a KAIST global_pose.csv (3x4 SE3 matrices in UTM) to a relative
trajectory CSV matching the path_recorder format: timestamp,x,y,z,qx,qy,qz,qw

The first pose within the requested time window is used as the origin.
Output timestamps are in seconds (float).

Usage:
    python3 kaist_gt_to_csv.py \
        --seq_dir /path/to/urban39-pankyo \
        --t_start_ns 1559195915612690236 \
        --t_end_ns   1559196515606383717 \
        --out_csv kaist_urban39_gt.csv
"""

import argparse
import numpy as np
import pandas as pd
from pathlib import Path


def rot_to_quat(R: np.ndarray):
    """3x3 rotation matrix → quaternion (x, y, z, w)."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        w = 0.25 * S
        x = (R[2, 1] - R[1, 2]) / S
        y = (R[0, 2] - R[2, 0]) / S
        z = (R[1, 0] - R[0, 1]) / S
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / S
        x = 0.25 * S
        y = (R[0, 1] + R[1, 0]) / S
        z = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / S
        x = (R[0, 1] + R[1, 0]) / S
        y = 0.25 * S
        z = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / S
        x = (R[0, 2] + R[2, 0]) / S
        y = (R[1, 2] + R[2, 1]) / S
        z = 0.25 * S
    return float(x), float(y), float(z), float(w)


def load_poses(pose_csv: Path, t_start_ns: int, t_end_ns: int):
    """
    Parse global_pose.csv rows within [t_start_ns, t_end_ns].
    Column layout: timestamp, r00,r01,r02,tx, r10,r11,r12,ty, r20,r21,r22,tz
    """
    poses = []
    with open(pose_csv) as f:
        for line in f:
            parts = line.strip().split(",")
            if len(parts) < 13:
                continue
            ts = int(parts[0])
            if ts < t_start_ns or ts > t_end_ns:
                continue
            R = np.array([
                [float(parts[1]),  float(parts[2]),  float(parts[3])],
                [float(parts[5]),  float(parts[6]),  float(parts[7])],
                [float(parts[9]),  float(parts[10]), float(parts[11])],
            ])
            t = np.array([float(parts[4]), float(parts[8]), float(parts[12])])
            poses.append((ts, R, t))
    return poses


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--seq_dir",    required=True,  help="Path to e.g. .../urban39-pankyo")
    ap.add_argument("--out_csv",    required=True,  help="Output CSV path")
    ap.add_argument("--t_start_ns", type=int, required=True, help="Bag start timestamp (ns)")
    ap.add_argument("--t_end_ns",   type=int, required=True, help="Bag end timestamp (ns)")
    args = ap.parse_args()

    seq_dir  = Path(args.seq_dir)
    seq_name = seq_dir.name  # e.g. "urban39-pankyo"
    pose_csv = seq_dir / f"{seq_name}_pose" / seq_name / "global_pose.csv"

    if not pose_csv.exists():
        raise FileNotFoundError(f"Pose file not found: {pose_csv}")

    print(f"[*] Loading poses from {pose_csv}")
    poses = load_poses(pose_csv, args.t_start_ns, args.t_end_ns)

    if not poses:
        raise RuntimeError(
            f"No poses found in [{args.t_start_ns}, {args.t_end_ns}] ns. "
            "Check that the sequence covers the bag time range."
        )

    print(f"[*] {len(poses)} poses in time window")

    ts0, R0, t0 = poses[0]
    R0_inv = R0.T  # rotation matrices are orthogonal: R^{-1} = R^T

    rows = []
    for ts, R, t in poses:
        R_rel = R0_inv @ R
        t_rel = R0_inv @ (t - t0)
        qx, qy, qz, qw = rot_to_quat(R_rel)
        rows.append({
            'timestamp': ts * 1e-9,
            'x': t_rel[0],
            'y': t_rel[1],
            'z': t_rel[2],
            'qx': qx,
            'qy': qy,
            'qz': qz,
            'qw': qw,
        })

    pd.DataFrame(rows).to_csv(args.out_csv, index=False)
    print(f"[OK] Wrote {len(rows)} poses → {args.out_csv}")
    total_dist = np.linalg.norm(
        np.array([[r['x'], r['y'], r['z']] for r in rows[-1:]], dtype=float) -
        np.array([[r['x'], r['y'], r['z']] for r in rows[:1]], dtype=float)
    )
    print(f"     Start→end displacement: {total_dist:.1f} m")


if __name__ == "__main__":
    main()
