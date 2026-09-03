#!/bin/bash
#
# run_zedx_live.sh — SoftGate-VIO live pipeline for ZED X stereo camera
#
# Usage:
#   ./scripts/run_zedx_live.sh [--rviz] [--timing] [--no-mask]
#
#   --rviz      Open RViz2 with the OpenVINS display config
#   --timing    Save verbosity:=ALL logs to timings/zedx_TIMESTAMP/; print table on exit
#   --no-mask   Run without YOLO masking (unmasked VIO baseline)
#
# Requirements:
#   1. ZED ROS2 wrapper running externally (or uncomment the launch block below)
#   2. Xsens driver running (or use ZED X internal IMU — see kalibr_imu_chain_zedx.yaml)
#   3. src/open_vins/config/zedx_config/ kalibr files filled with real calibration
#
# Hold Ctrl+C to stop all nodes cleanly.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"

# ── Source workspace ────────────────────────────────────────────────────────
source "$WS_DIR/install/setup.bash"

# ╔══════════════════════════════════════════════════════════════════════════╗
# ║                     USER CONFIG                                         ║
# ║  Edit these fields before the first run.                               ║
# ╚══════════════════════════════════════════════════════════════════════════╝

# Path to estimator config (fill kalibr files in the same directory first)
CONFIG_PATH="$WS_DIR/src/open_vins/config/zedx_config/estimator_config_zedx.yaml"

# YOLO segmentation model weights
MODEL_PATH="$WS_DIR/models/yolo26s-seg.pt"
# Lighter alternative for Jetson Orin NX / AGX (same COCO classes, faster):
# MODEL_PATH="$WS_DIR/models/yolo11n-seg.pt"

# ── ZED X camera topics ────────────────────────────────────────────────────
# Default: ZED ROS2 wrapper v4, namespace "zed", node_name "zed_node".
# If you changed camera_name in zed_camera.yaml, replace "zed_node" below.
# Use "image_rect_gray" (rectified) — no distortion needed in kalibr file.
ZED_LEFT_TOPIC="/zed/zed_node/left/image_rect_gray"
ZED_RIGHT_TOPIC="/zed/zed_node/right/image_rect_gray"

# ── IMU topic ──────────────────────────────────────────────────────────────
# Must match rostopic in kalibr_imu_chain_zedx.yaml.
#   Xsens MTi (bluespace_ai or xsens_mti_ros2_driver): /imu/data
#   ZED X internal IMU:                                 /zed/zed_node/imu/data
IMU_TOPIC="/imu/data"

# ── YOLO masker parameters ─────────────────────────────────────────────────
CONF_THRESHOLD=0.25       # detection confidence cutoff (0.25 = VIODE/KAIST default)
MAX_MASK_FRACTION=0.80    # if mask covers >80% of frame, publish empty mask (safety guard)
DILATION_KERNEL=13        # mask dilation in pixels (larger = more margin around detections)
USE_FLOW_CLASSIFIER=true  # ego-motion-compensated static/dynamic discrimination

# ── YOLO device ────────────────────────────────────────────────────────────
# "cuda" on desktop; "cuda:0" or "cuda" on Jetson; "cpu" as fallback
YOLO_DEVICE="${YOLO_DEVICE:-cuda}"

# ── Output directory for path_recorder CSV files ──────────────────────────
OUTPUT_DIR="${HOME}/ov_results_zedx"

# ╚══════════════════════════════════════════════════════════════════════════╝

# ── Parse flags ────────────────────────────────────────────────────────────
USE_RVIZ=false
USE_TIMING=false
USE_MASK=true

for arg in "$@"; do
    case "$arg" in
        --rviz)    USE_RVIZ=true ;;
        --timing)  USE_TIMING=true ;;
        --no-mask) USE_MASK=false ;;
        *)
            echo "Unknown argument: $arg"
            echo "Usage: $0 [--rviz] [--timing] [--no-mask]"
            exit 1
            ;;
    esac
done

VERBOSITY="INFO"
TIMING_DIR=""
TIMING_LOG=""
if $USE_TIMING; then
    TIMING_DIR="$WS_DIR/timings/zedx_$(date +%Y%m%d_%H%M%S)"
    mkdir -p "$TIMING_DIR"
    TIMING_LOG="$TIMING_DIR/combined.log"
    VERBOSITY="ALL"
    echo "[run_zedx_live] Timing logs → $TIMING_LOG"
fi

mkdir -p "$OUTPUT_DIR"
PIDS=()

cleanup() {
    echo ""
    echo "[run_zedx_live] Shutting down..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true

    if $USE_TIMING && [[ -f "$TIMING_LOG" ]]; then
        echo ""
        echo "[run_zedx_live] Parsing timing logs..."
        python3 "$SCRIPT_DIR/parse_timing.py" "$TIMING_LOG" --label "ZED X live" || \
            echo "[run_zedx_live] parse_timing.py failed — raw log: $TIMING_LOG"
    fi

    echo "[run_zedx_live] Done."
}
trap cleanup SIGINT SIGTERM EXIT

# ────────────────────────────────────────────────────────────────────────────
# (Optional) ZED ROS2 wrapper
# Uncomment to let this script manage the ZED wrapper itself.
# Make sure zed_ros2_wrapper is built and sourced before running.
# ────────────────────────────────────────────────────────────────────────────
# ros2 launch zed_wrapper zed_camera.launch.py \
#     camera_model:=zedx \
#     camera_name:=zed \
#     node_name:=zed_node &
# PIDS+=($!)
# echo "[run_zedx_live] Waiting for ZED camera to initialise..."
# sleep 6

# ────────────────────────────────────────────────────────────────────────────
# 1. YOLO masker
#    Subscribes to /cam0/image_raw and /cam1/image_raw — remapped to ZED topics.
#    Publishes /cam0/masked and /cam1/masked for OpenVINS.
# ────────────────────────────────────────────────────────────────────────────
if $USE_MASK; then
    echo "[run_zedx_live] Starting YOLO masker (device: $YOLO_DEVICE)..."
    _yolo_cmd=(ros2 run yolo_masker yolo_masker
        --ros-args
        -r /cam0/image_raw:="$ZED_LEFT_TOPIC"
        -r /cam1/image_raw:="$ZED_RIGHT_TOPIC"
        -p model_path:="$MODEL_PATH"
        -p confidence_threshold:="$CONF_THRESHOLD"
        -p max_mask_fraction:="$MAX_MASK_FRACTION"
        -p dilation_kernel:="$DILATION_KERNEL"
        -p use_flow_classifier:="$USE_FLOW_CLASSIFIER"
        -p device:="$YOLO_DEVICE"
        -p force_empty:=false)
    if $USE_TIMING; then
        "${_yolo_cmd[@]}" 2>&1 | tee -a "$TIMING_LOG" &
    else
        "${_yolo_cmd[@]}" &
    fi
    PIDS+=($!)

    echo "[run_zedx_live] Waiting for YOLO masker to load model..."
    timeout 90 bash -c \
        'until ros2 topic echo --once /yolo_masker/ready 2>/dev/null | grep -q "data: true"; do sleep 0.5; done' \
        || echo "[run_zedx_live] [WARN] /yolo_masker/ready not seen — proceeding anyway"
    sleep 1
else
    # No-mask baseline: start masker in force_empty mode (publishes all-zeros masks)
    echo "[run_zedx_live] Starting masker in force_empty mode (unmasked baseline)..."
    ros2 run yolo_masker yolo_masker \
        --ros-args \
        -r /cam0/image_raw:="$ZED_LEFT_TOPIC" \
        -r /cam1/image_raw:="$ZED_RIGHT_TOPIC" \
        -p model_path:="$MODEL_PATH" \
        -p force_empty:=true &
    PIDS+=($!)
    sleep 2
fi

# ────────────────────────────────────────────────────────────────────────────
# 2. OpenVINS MSCKF estimator
#    Topic subscriptions come from kalibr_imucam_chain_zedx.yaml (rostopic fields).
#    Camera topics must match $ZED_LEFT_TOPIC / $ZED_RIGHT_TOPIC set above.
#    IMU topic must match $IMU_TOPIC and kalibr_imu_chain_zedx.yaml:rostopic.
# ────────────────────────────────────────────────────────────────────────────
echo "[run_zedx_live] Starting OpenVINS..."
if $USE_TIMING; then
    # Tee stdout+stderr to the combined timing log so parse_timing.py can read it.
    # YOLO masker [TIMING] lines also land in the same file (see masker launch above).
    ros2 launch ov_msckf subscribe.launch.py \
        config_path:="$CONFIG_PATH" \
        rviz_enable:="$USE_RVIZ" \
        verbosity:=ALL 2>&1 | tee -a "$TIMING_LOG" &
else
    ros2 launch ov_msckf subscribe.launch.py \
        config_path:="$CONFIG_PATH" \
        rviz_enable:="$USE_RVIZ" \
        verbosity:=INFO &
fi
PIDS+=($!)

# ────────────────────────────────────────────────────────────────────────────
# 3. path_recorder (optional — comment out if you don't need trajectory CSV)
#    Saves vio_path_run_1.csv and gt_path_run_1.csv to $OUTPUT_DIR.
#    Note: gt_path_run_1.csv will be empty for live runs (no ground truth).
# ────────────────────────────────────────────────────────────────────────────
echo "[run_zedx_live] Starting path_recorder (output: $OUTPUT_DIR)..."
ros2 run ov_softgate path_recorder -- 1 \
    --ros-args \
    -p output_dir:="$OUTPUT_DIR" &
PIDS+=($!)

# ────────────────────────────────────────────────────────────────────────────

echo ""
echo "┌──────────────────────────────────────────────────────────┐"
echo "│  SoftGate-VIO running — press Ctrl+C to stop             │"
echo "│                                                           │"
printf  "│  Config:    %-46s│\n" "$(basename "$CONFIG_PATH")"
printf  "│  Masking:   %-46s│\n" "$(if $USE_MASK; then echo "YOLO ($MODEL_PATH | conf=$CONF_THRESHOLD)"; else echo "disabled (force_empty)"; fi)"
printf  "│  IMU:       %-46s│\n" "$IMU_TOPIC"
printf  "│  Camera L:  %-46s│\n" "$ZED_LEFT_TOPIC"
printf  "│  Output:    %-46s│\n" "$OUTPUT_DIR"
echo "└──────────────────────────────────────────────────────────┘"
echo ""

wait
