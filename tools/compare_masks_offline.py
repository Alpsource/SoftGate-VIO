"""
Offline YOLO-vs-GT mask comparison for VIODE bags.

Reads a bag, extracts /cam0/image_raw and /cam0/segmentation,
runs YOLO segmentation and GT color-matching side by side,
then computes per-frame metrics and saves visualizations.

Usage:
    python3 compare_masks_offline.py [DATASET] [LEVEL] [--frames N] [--save-video]

    DATASET : parking_lot | city_day | city_night  (default: parking_lot)
    LEVEL   : none | low | mid | high              (default: high)
    --frames  : max frames to process              (default: 100)
    --save-video : also write compare.mp4

Example:
    python3 compare_masks_offline.py city_day high --frames 200
    python3 compare_masks_offline.py parking_lot high --frames 150 --save-video
"""

import argparse
import sys
import os
import numpy as np
import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pathlib import Path

# ── Paths ──────────────────────────────────────────────────────────────────
WORKSPACE     = os.environ.get("SOFTGATE_WORKSPACE", "/path/to/your/workspace")
VIODE_ROOT    = f"{WORKSPACE}/Downloads_Ext/VIODE_Dataset"
MODEL_PATH    = f"{WORKSPACE}/openvins_ws/yolo26s-seg.pt"
OUTPUT_DIR    = f"{WORKSPACE}/sim_results/mask_comparison"

# ── GT dynamic color palette (BGR, from semantic_masker.py) ───────────────
GT_DYNAMIC_COLORS = [
    (170, 237, 115),   # id=241  city only dynamic
    (239, 169, 255),   # id=242  city only dynamic
    (227, 202,  99),   # id=243  city only dynamic
    (160, 239, 236),   # id=244  city only dynamic
    (243, 234, 143),   # id=245  city only dynamic
    (155, 221, 166),   # id=246  all envs dynamic
    (245, 248, 154),   # id=247  all envs dynamic
    (188, 210, 253),   # id=248  all envs dynamic
    (251,  59, 226),   # id=249  all envs dynamic
    (207,  91, 108),   # id=250  all envs dynamic
    (231, 196, 243),   # id=251  all envs dynamic
]

# Static parked cars — explicitly NOT in GT dynamic list
GT_STATIC_COLORS = [
    (181, 231, 209),   # id=254  parking_lot + city_day static
    (232, 119, 114),   # id=255  all envs static
]

# YOLO classes to mask (COCO): person bicycle car motorcycle bus train truck
YOLO_CLASSES      = [0, 1, 2, 3, 5, 6, 7]
CONF_THRESHOLD    = 0.25
DILATION_KERNEL   = 13
MAX_MASK_FRACTION = 0.80

_CLAHE = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))


def apply_clahe(bgr: np.ndarray) -> np.ndarray:
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
    lab[:, :, 0] = _CLAHE.apply(lab[:, :, 0])
    return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)


def build_gt_mask(seg_bgr: np.ndarray, dilation: int, max_frac: float):
    """GT dynamic mask (same logic as semantic_masker.py)."""
    h, w = seg_bgr.shape[:2]
    mask = np.zeros((h, w), dtype=np.uint8)
    for color in GT_DYNAMIC_COLORS:
        lo = np.array(color, dtype=np.uint8)
        mask = cv2.bitwise_or(mask, cv2.inRange(seg_bgr, lo, lo))
    k = np.ones((dilation, dilation), np.uint8)
    mask = cv2.dilate(mask, k, iterations=1)
    if np.count_nonzero(mask) / mask.size > max_frac:
        mask = np.zeros((h, w), dtype=np.uint8)
    return mask


def build_gt_static_mask(seg_bgr: np.ndarray):
    """Static parked-car pixels — what GT deliberately leaves unmasked."""
    h, w = seg_bgr.shape[:2]
    mask = np.zeros((h, w), dtype=np.uint8)
    for color in GT_STATIC_COLORS:
        lo = np.array(color, dtype=np.uint8)
        mask = cv2.bitwise_or(mask, cv2.inRange(seg_bgr, lo, lo))
    return mask


def build_yolo_mask(model, bgr: np.ndarray, dilation: int, max_frac: float, clahe: bool = True):
    """YOLO segmentation VIO mask (same logic as yolo_masker_node.py)."""
    h, w = bgr.shape[:2]
    yolo_input = apply_clahe(bgr) if clahe else bgr
    results = model(yolo_input, classes=YOLO_CLASSES, conf=CONF_THRESHOLD, verbose=False)
    mask = np.zeros((h, w), dtype=np.uint8)
    if results and results[0].masks is not None and len(results[0].masks) > 0:
        seg_np = results[0].masks.data.cpu().numpy()
        for i in range(seg_np.shape[0]):
            m = cv2.resize(seg_np[i], (w, h), interpolation=cv2.INTER_LINEAR)
            mask[m > 0.5] = 255
        k = np.ones((dilation, dilation), np.uint8)
        mask = cv2.dilate(mask, k, iterations=1)
        if np.count_nonzero(mask) / mask.size > max_frac:
            mask = np.zeros((h, w), dtype=np.uint8)
    return mask


def iou(a: np.ndarray, b: np.ndarray) -> float:
    ab = (a > 0) & (b > 0)
    au = (a > 0) | (b > 0)
    return float(np.count_nonzero(ab)) / max(float(np.count_nonzero(au)), 1)


def false_positive_rate(yolo: np.ndarray, gt: np.ndarray, static: np.ndarray) -> float:
    """Fraction of YOLO-masked pixels that are static cars (GT leaves them alone)."""
    yolo_bin   = yolo > 0
    static_bin = static > 0
    yolo_masks_static = np.count_nonzero(yolo_bin & static_bin)
    yolo_total        = np.count_nonzero(yolo_bin)
    return float(yolo_masks_static) / max(yolo_total, 1)


def fn_metrics(gt: np.ndarray, yolo: np.ndarray, gray: np.ndarray, num_pts: int = 300):
    """
    False-negative zone: pixels GT masks but YOLO doesn't.
    These dynamic pixels leak into VIO as corrupt features.

    Returns:
        fn_cov       : fraction of image area in the FN zone
        feat_leak    : KLT features detected in the FN zone (proxy for corrupt VIO measurements)
        feat_total   : total KLT features in unmasked image (denominator for context)
    """
    fn_zone = ((gt > 0) & (yolo == 0)).astype(np.uint8) * 255   # missed by YOLO
    fn_cov  = np.count_nonzero(fn_zone) / fn_zone.size

    # Detect KLT features on the raw gray image (same as OpenVINS would)
    pts = cv2.goodFeaturesToTrack(gray, maxCorners=num_pts, qualityLevel=0.01, minDistance=10)
    feat_total = len(pts) if pts is not None else 0

    feat_leak = 0
    if pts is not None and np.count_nonzero(fn_zone) > 0:
        for p in pts.reshape(-1, 2):
            x, y = int(p[0]), int(p[1])
            if 0 <= y < fn_zone.shape[0] and 0 <= x < fn_zone.shape[1]:
                if fn_zone[y, x] > 0:
                    feat_leak += 1

    return fn_cov, feat_leak, feat_total


def mask_flicker(prev_mask: np.ndarray | None, cur_mask: np.ndarray) -> float:
    """Fraction of pixels that changed state frame-to-frame (mask instability)."""
    if prev_mask is None:
        return 0.0
    return float(np.count_nonzero((prev_mask > 0) != (cur_mask > 0))) / cur_mask.size


def make_panel(rgb: np.ndarray, gt_mask: np.ndarray, yolo_mask: np.ndarray,
               static_mask: np.ndarray, frame_id: int, metrics: dict) -> np.ndarray:
    """Build a side-by-side comparison panel (BGR)."""
    h, w = rgb.shape[:2]
    out_w = w * 3
    panel = np.zeros((h, out_w, 3), dtype=np.uint8)

    # Left: GT overlay (red = dynamic masked, blue = static, i.e., what GT preserves)
    left = rgb.copy()
    left[gt_mask > 0]     = (left[gt_mask > 0] * 0.4 + np.array([0, 0, 200]) * 0.6).clip(0, 255)
    left[static_mask > 0] = (left[static_mask > 0] * 0.5 + np.array([200, 0, 0]) * 0.5).clip(0, 255)

    # Center: YOLO overlay (green = yolo-only mask, i.e., over-masking static cars)
    ctr  = rgb.copy()
    both = (gt_mask > 0) & (yolo_mask > 0)
    yolo_only = (yolo_mask > 0) & (gt_mask == 0)
    ctr[both]      = (ctr[both]      * 0.4 + np.array([0, 200, 0])   * 0.6).clip(0, 255)  # agreed
    ctr[yolo_only] = (ctr[yolo_only] * 0.4 + np.array([0, 200, 255]) * 0.6).clip(0, 255)  # YOLO extra

    # Right: diff — yellow = GT-only, cyan = YOLO-only
    diff = rgb.copy()
    gt_only = (gt_mask > 0) & (yolo_mask == 0)
    diff[yolo_only] = (diff[yolo_only] * 0.4 + np.array([0, 200, 255]) * 0.6).clip(0, 255)
    diff[gt_only]   = (diff[gt_only]   * 0.4 + np.array([0, 255, 255]) * 0.6).clip(0, 255)

    panel[:, :w]       = left.astype(np.uint8)
    panel[:, w:2*w]    = ctr.astype(np.uint8)
    panel[:, 2*w:3*w]  = diff.astype(np.uint8)

    # Labels
    def label(img, text, pos=(10, 25)):
        cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2, cv2.LINE_AA)
        cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0),       1, cv2.LINE_AA)

    label(panel[:, :w],       f"GT  mask (red=dyn  blue=static) cov={metrics['gt_cov']:.0%}")
    label(panel[:, w:2*w],    f"YOLO mask (green=agree  cyan=yolo-extra) cov={metrics['yolo_cov']:.0%}")
    label(panel[:, 2*w:3*w],  f"Diff  IoU={metrics['iou']:.2f}  FP(static)={metrics['fp_static']:.0%}")
    label(panel[:, :w],       f"Frame {frame_id}", (10, 50))

    return panel


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('dataset', nargs='?', default='parking_lot')
    parser.add_argument('level',   nargs='?', default='high')
    parser.add_argument('--frames',     type=int, default=100)
    parser.add_argument('--skip',       type=int, default=200,
                        help='frames to skip at bag start (avoids empty scene at t=0)')
    parser.add_argument('--save-video', action='store_true')
    args = parser.parse_args()

    bag_path = f"{VIODE_ROOT}/{args.dataset}_{args.level}"
    out_dir  = Path(OUTPUT_DIR) / f"{args.dataset}_{args.level}"
    out_dir.mkdir(parents=True, exist_ok=True)

    if not os.path.isdir(bag_path):
        print(f"[ERROR] Bag not found: {bag_path}")
        sys.exit(1)

    # ── Load YOLO model ────────────────────────────────────────────────────
    try:
        from ultralytics import YOLO
    except ImportError:
        print("[ERROR] pip install ultralytics")
        sys.exit(1)

    print(f"Loading YOLO model: {MODEL_PATH}")
    model = YOLO(MODEL_PATH)
    model.to('cuda')
    print("Model loaded.")

    # ── Open bag ───────────────────────────────────────────────────────────
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import Image
        from cv_bridge import CvBridge
    except ImportError:
        print("[ERROR] rosbag2_py / rclpy not available. Source your ROS2 install.")
        sys.exit(1)

    bridge   = CvBridge()
    storage  = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    convert  = rosbag2_py.ConverterOptions('', '')
    reader   = rosbag2_py.SequentialReader()
    reader.open(storage, convert)

    topic_types = {m.name: m.type for m in reader.get_all_topics_and_types()}
    print(f"Topics: {list(topic_types.keys())}")

    WANT = {'/cam0/image_raw', '/cam0/segmentation'}
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(WANT)))

    # ── Collect paired frames ──────────────────────────────────────────────
    buf_raw = {}   # stamp_ns -> bgr
    buf_seg = {}   # stamp_ns -> bgr
    frames  = []   # list of (raw_bgr, seg_bgr)

    if args.skip > 0:
        print(f"Skipping first {args.skip} frames…")
        skipped = 0
        while reader.has_next() and skipped < args.skip:
            topic, _, _ = reader.read_next()
            if topic == '/cam0/image_raw':
                skipped += 1

    print(f"Reading bag (max {args.frames} frames)…")
    while reader.has_next() and len(frames) < args.frames * 4:
        topic, data, stamp = reader.read_next()
        msg = deserialize_message(data, Image)
        ns  = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec

        if topic == '/cam0/image_raw':
            try:    buf_raw[ns] = bridge.imgmsg_to_cv2(msg, 'bgr8')
            except: pass
        elif topic == '/cam0/segmentation':
            try:    buf_seg[ns] = bridge.imgmsg_to_cv2(msg, 'bgr8')
            except: pass

        # Pair on exact timestamp
        if ns in buf_raw and ns in buf_seg:
            frames.append((buf_raw.pop(ns), buf_seg.pop(ns)))
            if len(frames) >= args.frames:
                break

    print(f"Paired {len(frames)} frames.")
    if not frames:
        print("[ERROR] No paired frames found. Check topic names in bag.")
        sys.exit(1)

    # ── Warmup ────────────────────────────────────────────────────────────
    model(frames[0][0], classes=YOLO_CLASSES, conf=CONF_THRESHOLD, verbose=False)
    model(frames[0][0], classes=YOLO_CLASSES, conf=CONF_THRESHOLD, verbose=False)

    # ── Per-frame comparison ───────────────────────────────────────────────
    metrics_list = []
    panels       = []
    prev_yolo    = None
    SAVE_SAMPLE_EVERY = max(1, len(frames) // 10)

    for i, (raw_bgr, seg_bgr) in enumerate(frames):
        gt_mask     = build_gt_mask(seg_bgr, DILATION_KERNEL, MAX_MASK_FRACTION)
        yolo_mask   = build_yolo_mask(model, raw_bgr, DILATION_KERNEL, MAX_MASK_FRACTION)
        static_mask = build_gt_static_mask(seg_bgr)
        gray        = cv2.cvtColor(raw_bgr, cv2.COLOR_BGR2GRAY)

        fn_cov, feat_leak, feat_total = fn_metrics(gt_mask, yolo_mask, gray)
        flicker = mask_flicker(prev_yolo, yolo_mask)
        prev_yolo = yolo_mask.copy()

        m = {
            'iou':        iou(gt_mask, yolo_mask),
            'gt_cov':     np.count_nonzero(gt_mask)   / gt_mask.size,
            'yolo_cov':   np.count_nonzero(yolo_mask) / yolo_mask.size,
            'fp_static':  false_positive_rate(yolo_mask, gt_mask, static_mask),
            'fn_cov':     fn_cov,       # dynamic zone YOLO misses → leaks to VIO
            'feat_leak':  feat_leak,    # KLT features on moving cars that reach VIO
            'feat_total': feat_total,   # total KLT features (context)
            'flicker':    flicker,      # frame-to-frame mask instability
        }
        metrics_list.append(m)

        if i % SAVE_SAMPLE_EVERY == 0:
            rgb   = cv2.cvtColor(raw_bgr, cv2.COLOR_BGR2RGB)
            panel = make_panel(rgb, gt_mask, yolo_mask, static_mask, i, m)
            panels.append((i, panel))
            cv2.imwrite(str(out_dir / f"frame_{i:04d}.png"), panel)

        if (i + 1) % 20 == 0 or i == len(frames) - 1:
            print(f"  [{i+1:3d}/{len(frames)}]  "
                  f"IoU={m['iou']:.2f}  "
                  f"GT_cov={m['gt_cov']:.0%}  "
                  f"YOLO_cov={m['yolo_cov']:.0%}  "
                  f"FN_cov={m['fn_cov']:.1%}  "
                  f"leak={m['feat_leak']}/{m['feat_total']} feats  "
                  f"flicker={m['flicker']:.1%}")

    # ── Aggregate statistics ───────────────────────────────────────────────
    ious        = [m['iou']        for m in metrics_list]
    gt_covs     = [m['gt_cov']     for m in metrics_list]
    yolo_covs   = [m['yolo_cov']   for m in metrics_list]
    fp_statics  = [m['fp_static']  for m in metrics_list]
    fn_covs     = [m['fn_cov']     for m in metrics_list]
    feat_leaks  = [m['feat_leak']  for m in metrics_list]
    feat_totals = [m['feat_total'] for m in metrics_list]
    flickers    = [m['flicker']    for m in metrics_list]

    mean_leak_pct = (np.mean(feat_leaks) / max(np.mean(feat_totals), 1)) * 100

    print()
    print("=" * 65)
    print(f"  Scenario : {args.dataset}_{args.level}   ({len(frames)} frames)")
    print("=" * 65)
    print(f"  Mask IoU (GT vs YOLO)              : {np.mean(ious):.3f}  ± {np.std(ious):.3f}")
    print(f"  GT mask coverage                   : {np.mean(gt_covs):.1%}  ± {np.std(gt_covs):.1%}")
    print(f"  YOLO mask coverage                 : {np.mean(yolo_covs):.1%}  ± {np.std(yolo_covs):.1%}")
    print(f"  YOLO FP rate (static cars masked)  : {np.mean(fp_statics):.1%}  ± {np.std(fp_statics):.1%}")
    print()
    print(f"  ── VIO corruption indicators ──────────────────────────────")
    print(f"  FN coverage (dynamic zone YOLO misses)  : {np.mean(fn_covs):.1%}  ± {np.std(fn_covs):.1%}")
    print(f"  Feature leak into VIO (mean per frame)  : {np.mean(feat_leaks):.1f} / {np.mean(feat_totals):.0f} ({mean_leak_pct:.1f}%)")
    print(f"  Peak feature leak                       : {max(feat_leaks)} feats")
    print(f"  YOLO mask flicker (frame-to-frame Δ)    : {np.mean(flickers[1:]):.1%}  ± {np.std(flickers[1:]):.1%}")
    print()
    print(f"  Frames where YOLO > GT cov              : {sum(y>g for y,g in zip(yolo_covs, gt_covs))}/{len(frames)}")
    print(f"  Frames YOLO coverage > 80%              : {sum(y>0.80 for y in yolo_covs)} (guard triggered)")
    print("=" * 65)
    print()
    if np.mean(feat_leaks) > 10:
        print("  [VERDICT] Feature leak is HIGH — YOLO masking is likely causing VIO corruption.")
    elif np.mean(feat_leaks) > 3:
        print("  [VERDICT] Feature leak is MODERATE — YOLO masking may be contributing to VIO drift.")
    else:
        print("  [VERDICT] Feature leak is LOW — VIO instability is likely NOT caused by masking quality.")
    print(f"  Saved sample frames → {out_dir}/frame_NNNN.png")

    # ── Time-series plot ───────────────────────────────────────────────────
    fig, axes = plt.subplots(4, 1, figsize=(14, 10), sharex=True)
    xs = list(range(len(metrics_list)))

    axes[0].plot(xs, ious, label='IoU(GT,YOLO)', color='steelblue')
    axes[0].set_ylabel('Mask IoU'); axes[0].set_ylim(0, 1); axes[0].legend(); axes[0].grid(True)

    axes[1].plot(xs, gt_covs,   label='GT coverage',   color='green')
    axes[1].plot(xs, yolo_covs, label='YOLO coverage', color='orange')
    axes[1].plot(xs, fn_covs,   label='FN zone (YOLO misses = corrupt VIO zone)', color='red', alpha=0.7)
    axes[1].axhline(0.80, color='red', linestyle='--', alpha=0.3, label='80% guard')
    axes[1].set_ylabel('Coverage'); axes[1].set_ylim(0, 1); axes[1].legend(); axes[1].grid(True)

    axes[2].plot(xs, feat_leaks,  label='Features leaked to VIO (on missed dynamic zones)', color='crimson')
    axes[2].plot(xs, feat_totals, label='Total detectable KLT features', color='gray', alpha=0.5)
    axes[2].set_ylabel('Feature count'); axes[2].legend(); axes[2].grid(True)

    axes[3].plot(xs[1:], flickers[1:], label='YOLO mask flicker (frame-to-frame Δ)', color='purple', alpha=0.8)
    axes[3].set_ylabel('Flicker rate'); axes[3].set_ylim(0, None); axes[3].legend(); axes[3].grid(True)
    axes[3].set_xlabel('Frame')

    fig.suptitle(f'GT vs YOLO Mask Comparison — {args.dataset}_{args.level}')
    fig.tight_layout()
    plot_path = out_dir / 'comparison_metrics.png'
    fig.savefig(str(plot_path), dpi=120)
    print(f"  Saved metrics plot  → {plot_path}")

    # ── Optional video ─────────────────────────────────────────────────────
    if args.save_video and frames:
        raw0, seg0 = frames[0]
        h, w = raw0.shape[:2]
        vid_path = str(out_dir / 'compare.mp4')
        writer = cv2.VideoWriter(vid_path, cv2.VideoWriter_fourcc(*'mp4v'), 10, (w * 3, h))
        for i, (raw_bgr, seg_bgr) in enumerate(frames):
            gt_m  = build_gt_mask(seg_bgr, DILATION_KERNEL, MAX_MASK_FRACTION)
            yo_m  = build_yolo_mask(model, raw_bgr, DILATION_KERNEL, MAX_MASK_FRACTION)
            st_m  = build_gt_static_mask(seg_bgr)
            panel = make_panel(cv2.cvtColor(raw_bgr, cv2.COLOR_BGR2RGB),
                               gt_m, yo_m, st_m, i, metrics_list[i])
            writer.write(panel)
        writer.release()
        print(f"  Saved video         → {vid_path}")


if __name__ == '__main__':
    main()
