#!/usr/bin/env python3
"""
convert_kaist_to_ros2bag.py
Convert a raw KAIST Complex Urban Dataset sequence (stereo PNGs + xsens_imu.csv)
into a native ROS2 (Humble) bag that OpenVINS can play directly.

Unlike the TUM-VI pipeline, this does NOT go through rosbags-convert (ROS1->ROS2),
so it avoids the version/metadata.yaml incompatibilities we hit there. It writes
the bag directly with rosbags.rosbag2.Writer (version=8), which Humble's
`ros2 bag play` accepts unpatched.

Images are undistorted + stereo-rectified on the fly using the dataset's own
left.yaml/right.yaml (K, D, R, P), so the OpenVINS config can use distortion=0
and the post-rectification P-matrix intrinsics directly.

Usage:
    python3 convert_kaist_to_ros2bag.py --seq_dir "/path/to/urban39-pankyo" \
        --out_bag "/path/to/urban39_ros2bag" [--duration_sec 30]

    # Full sequence (no cap):
    python3 convert_kaist_to_ros2bag.py --seq_dir "/path/to/urban39-pankyo" \
        --out_bag "/path/to/urban39_ros2bag"
"""

import argparse
import bisect
from pathlib import Path

import cv2
import numpy as np
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore
from rosbags.typesys.stores.ros2_humble import (
    builtin_interfaces__msg__Time as Time,
    geometry_msgs__msg__Quaternion as Quaternion,
    geometry_msgs__msg__Vector3 as Vector3,
    sensor_msgs__msg__Image as Image,
    sensor_msgs__msg__Imu as Imu,
    std_msgs__msg__Header as Header,
)

TYPESTORE = get_typestore(Stores.ROS2_HUMBLE)


def load_cam_yaml(path: Path):
    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    K = fs.getNode("camera_matrix").mat()
    D = fs.getNode("distortion_coefficients").mat()
    R = fs.getNode("rectification_matrix").mat()
    P = fs.getNode("projection_matrix").mat()
    w = int(fs.getNode("image_width").real())
    h = int(fs.getNode("image_height").real())
    fs.release()
    return K, D, R, P, (w, h)


def ts_to_time(ts_ns: int) -> Time:
    return Time(sec=ts_ns // 1_000_000_000, nanosec=ts_ns % 1_000_000_000)


def make_image_msg(img: np.ndarray, ts_ns: int, frame_id: str) -> Image:
    h, w = img.shape
    return Image(
        header=Header(stamp=ts_to_time(ts_ns), frame_id=frame_id),
        height=h,
        width=w,
        encoding="mono8",
        is_bigendian=0,
        step=w,
        data=np.ascontiguousarray(img).reshape(-1),
    )


def make_imu_msg(ts_ns: int, gyro: np.ndarray, accel: np.ndarray) -> Imu:
    return Imu(
        header=Header(stamp=ts_to_time(ts_ns), frame_id="imu0"),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        orientation_covariance=np.array([-1.0] + [0.0] * 8, dtype=np.float64),
        angular_velocity=Vector3(x=float(gyro[0]), y=float(gyro[1]), z=float(gyro[2])),
        angular_velocity_covariance=np.zeros(9, dtype=np.float64),
        linear_acceleration=Vector3(x=float(accel[0]), y=float(accel[1]), z=float(accel[2])),
        linear_acceleration_covariance=np.zeros(9, dtype=np.float64),
    )


def build_nearest_lookup(image_dir: Path):
    """timestamp (int) -> filename, sorted, for nearest-match lookup."""
    stamps = []
    for p in image_dir.glob("*.png"):
        stamps.append(int(p.stem))
    stamps.sort()
    return stamps


def find_nearest(stamps_sorted, target: int, tol_ns: int):
    idx = bisect.bisect_left(stamps_sorted, target)
    candidates = [c for c in (idx - 1, idx) if 0 <= c < len(stamps_sorted)]
    if not candidates:
        return None
    best = min(candidates, key=lambda c: abs(stamps_sorted[c] - target))
    if abs(stamps_sorted[best] - target) > tol_ns:
        return None
    return stamps_sorted[best]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--seq_dir", required=True, help="Path to e.g. .../urban39-pankyo")
    ap.add_argument("--out_bag", required=True, help="Output ROS2 bag directory (must not exist)")
    ap.add_argument("--start_sec", type=float, default=0.0,
                     help="Skip the first N seconds of the sequence before converting")
    ap.add_argument("--duration_sec", type=float, default=None,
                     help="Only convert N seconds starting at --start_sec (for quick debug bags)")
    ap.add_argument("--right_tol_ms", type=float, default=20.0,
                     help="Max time gap allowed when nearest-matching the right image to the left timestamp")
    args = ap.parse_args()

    seq_dir = Path(args.seq_dir)
    seq_name = seq_dir.name  # e.g. "urban39-pankyo"
    out_bag = Path(args.out_bag)

    calib_dir = seq_dir / f"{seq_name}_calibration" / seq_name / "calibration"
    left_yaml = calib_dir / "left.yaml"
    right_yaml = calib_dir / "right.yaml"
    if not left_yaml.exists() or not right_yaml.exists():
        raise FileNotFoundError(f"Calibration not found under {calib_dir}")

    print(f"[*] Loading calibration from {calib_dir}")
    Kl, Dl, Rl, Pl, size_l = load_cam_yaml(left_yaml)
    Kr, Dr, Rr, Pr, size_r = load_cam_yaml(right_yaml)

    print("[*] Computing rectification maps ...")
    map_l = cv2.initUndistortRectifyMap(Kl, Dl, Rl, Pl, size_l, cv2.CV_16SC2)
    map_r = cv2.initUndistortRectifyMap(Kr, Dr, Rr, Pr, size_r, cv2.CV_16SC2)

    stereo_stamp_csv = seq_dir / "sensor_data" / "stereo_stamp.csv"
    xsens_csv = seq_dir / "sensor_data" / "xsens_imu.csv"
    left_dir = seq_dir / "image" / "stereo_left"
    right_dir = seq_dir / "image" / "stereo_right"

    left_stamps = [int(x.strip()) for x in stereo_stamp_csv.read_text().splitlines() if x.strip()]
    left_stamps.sort()
    right_stamps_sorted = build_nearest_lookup(right_dir)
    right_tol_ns = int(args.right_tol_ms * 1e6)

    print(f"[*] {len(left_stamps)} stereo timestamps, {len(right_stamps_sorted)} right images on disk")

    # IMU rows
    imu_rows = []
    with open(xsens_csv) as f:
        for line in f:
            parts = line.strip().split(",")
            if len(parts) < 17:
                continue
            ts = int(parts[0])
            gyro = np.array(parts[8:11], dtype=np.float64)
            accel = np.array(parts[11:14], dtype=np.float64)
            imu_rows.append((ts, gyro, accel))
    imu_rows.sort(key=lambda r: r[0])
    print(f"[*] {len(imu_rows)} IMU samples")

    t_base = min(left_stamps[0], imu_rows[0][0])
    t0 = t_base + int(args.start_sec * 1e9)
    t_end = t0 + int(args.duration_sec * 1e9) if args.duration_sec else None
    if args.start_sec:
        print(f"[*] Skipping first {args.start_sec:.1f}s (t0={t0})")
    if t_end:
        print(f"[*] Capping output to {args.duration_sec:.1f}s after start (t_end={t_end})")

    if out_bag.exists():
        raise FileExistsError(f"{out_bag} already exists — remove it first or pick a new path")

    n_img, n_imu, n_skip_right = 0, 0, 0
    with Writer(out_bag, version=8) as writer:
        conn_left = writer.add_connection("/stereo/left/image_rect", Image.__msgtype__, typestore=TYPESTORE)
        conn_right = writer.add_connection("/stereo/right/image_rect", Image.__msgtype__, typestore=TYPESTORE)
        conn_imu = writer.add_connection("/imu0", Imu.__msgtype__, typestore=TYPESTORE)

        for ts in left_stamps:
            if ts < t0:
                continue
            if t_end and ts > t_end:
                break
            right_ts = find_nearest(right_stamps_sorted, ts, right_tol_ns)
            if right_ts is None:
                n_skip_right += 1
                continue

            img_l = cv2.imread(str(left_dir / f"{ts}.png"), cv2.IMREAD_GRAYSCALE)
            img_r = cv2.imread(str(right_dir / f"{right_ts}.png"), cv2.IMREAD_GRAYSCALE)
            if img_l is None or img_r is None:
                n_skip_right += 1
                continue

            rect_l = cv2.remap(img_l, *map_l, cv2.INTER_LINEAR)
            rect_r = cv2.remap(img_r, *map_r, cv2.INTER_LINEAR)

            msg_l = make_image_msg(rect_l, ts, "cam0")
            msg_r = make_image_msg(rect_r, ts, "cam1")
            writer.write(conn_left, ts, TYPESTORE.serialize_cdr(msg_l, Image.__msgtype__))
            writer.write(conn_right, ts, TYPESTORE.serialize_cdr(msg_r, Image.__msgtype__))
            n_img += 1

        for ts, gyro, accel in imu_rows:
            if ts < t0:
                continue
            if t_end and ts > t_end:
                break
            msg = make_imu_msg(ts, gyro, accel)
            writer.write(conn_imu, ts, TYPESTORE.serialize_cdr(msg, Imu.__msgtype__))
            n_imu += 1

    print(f"[OK] Wrote {out_bag}")
    print(f"     stereo pairs: {n_img}  (skipped, no right match: {n_skip_right})")
    print(f"     imu samples:  {n_imu}")


if __name__ == "__main__":
    main()
