#!/bin/bash
# =============================================================================
# run_zedx_live.sh — Live pipeline for ZED X stereo camera
#
# Starts OpenVINS + YOLO masker + HSE. Runs until Ctrl+C, then shuts down
# all nodes cleanly. Assumes the ZED ROS2 wrapper is already running (or
# launches it once the TODO markers below are filled in).
#
# Usage:
#   ./scripts/run_zedx_live.sh [--rviz] [--timing]
#
#   --rviz    Open RViz with the default OpenVINS display config
#   --timing  Save per-node logs and print a timing table on exit
#
# Before first use — fill in every line marked TODO: ZEDX
# =============================================================================

# ── Flags ─────────────────────────────────────────────────────────────────────
ENABLE_RVIZ=false
ENABLE_TIMING=false
for arg in "$@"; do
    case $arg in
        --rviz)   ENABLE_RVIZ=true ;;
        --timing) ENABLE_TIMING=true ;;
    esac
done

# ── TODO: ZEDX — edit these before running ────────────────────────────────────
#
# Step 1: Install ZED ROS2 wrapper
#         https://github.com/stereolabs/zed-ros2-wrapper
#
# Step 2: Confirm the camera namespace ("zed" is the default for ZED X)
ZEDX_NS="zed"                                       # TODO: ZEDX — confirm namespace

# Step 3: Confirm topic names (these match zed-ros2-wrapper v4 defaults)
ZEDX_LEFT_TOPIC="/${ZEDX_NS}/zed_node/left/image_rect_color"   # TODO: ZEDX
ZEDX_RIGHT_TOPIC="/${ZEDX_NS}/zed_node/right/image_rect_color" # TODO: ZEDX
ZEDX_IMU_TOPIC="/${ZEDX_NS}/zed_node/imu/data"                 # TODO: ZEDX (OpenVINS reads this from config)

# Step 4: Create the estimator config and calib YAML for ZEDX
#         Place them in config/zedx_config/ inside the workspace
CONFIG_PATH="/media/neurolab/60a72ba2-3a9d-47d0-88e3-852b0f67f283/openvins_ws/src/open_vins/config/zedx_config/estimator_config_zedx.yaml"  # TODO: ZEDX
CALIB_FILE="/media/neurolab/60a72ba2-3a9d-47d0-88e3-852b0f67f283/openvins_ws/src/open_vins/config/zedx_config/kalibr_imucam_chain_zedx.yaml" # TODO: ZEDX

# Step 5: Tune YOLO confidence for your environment (0.25 indoor, 0.45 city)
YOLO_CONF=0.35                                      # TODO: ZEDX — tune for scene
# ─────────────────────────────────────────────────────────────────────────────

WORKSPACE="/media/neurolab/60a72ba2-3a9d-47d0-88e3-852b0f67f283"
OPENVINS_WS="${WORKSPACE}/openvins_ws"
MODEL_PATH="${OPENVINS_WS}/models/yolo26s-seg.pt"
RVIZ_CFG="${OPENVINS_WS}/install/ov_msckf/share/ov_msckf/launch/display_ros2.rviz"
PARSE_SCRIPT="${OPENVINS_WS}/scripts/parse_timing.py"

LOG_DIR="${OPENVINS_WS}/timings/zedx_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"

KILL_LIST=(
    "ov_msckf" "ov_softgate" "yolo_masker" "hybrid_speed_estimator"
    "zed_node" "zed_wrapper" "zed_camera" "rviz2"
)

cleanup_nodes() {
    echo ""
    echo "[CLEANUP] Stopping all nodes..."
    for t in "${KILL_LIST[@]}"; do pkill -f "$t" > /dev/null 2>&1 || true; done
    sleep 1
    for t in "${KILL_LIST[@]}"; do pkill -9 -f "$t" > /dev/null 2>&1 || true; done
    ros2 daemon stop  > /dev/null 2>&1 || true
    pkill -9 -f ros2  > /dev/null 2>&1 || true
    pkill -9 -f fastdds > /dev/null 2>&1 || true
    rm -rf ~/.ros/ros2daemon /dev/shm/fastrtps_* /dev/shm/rtps_* 2>/dev/null || true
    if $ENABLE_TIMING; then
        echo "[TIMING] Parsing logs..."
        cat "${LOG_DIR}/openvins.log" "${LOG_DIR}/yolo_masker.log" "${LOG_DIR}/hse.log" \
            > "${LOG_DIR}/combined.log" 2>/dev/null
        python3 "$PARSE_SCRIPT" "${LOG_DIR}/combined.log" --label "zedx-live" \
            | tee "${LOG_DIR}/timing_table.txt"
        echo ""
        echo "  Logs saved to: ${LOG_DIR}/"
    fi
    echo "[CLEANUP] Done."
}

source "${OPENVINS_WS}/install/setup.bash"
trap 'cleanup_nodes; exit 0' INT TERM

# ── Preflight ─────────────────────────────────────────────────────────────────
if [[ ! -f "$CONFIG_PATH" ]]; then
    echo "ERROR: config not found: $CONFIG_PATH"
    echo "       Create estimator_config_zedx.yaml (see TODO markers at top of script)."
    exit 1
fi

echo "========================================================"
echo "  ZEDX Live Pipeline"
echo "  Config : $(basename $CONFIG_PATH)"
echo "  YOLO   : conf=${YOLO_CONF}, device=${YOLO_DEVICE:-cuda}"
echo "  RViz   : $ENABLE_RVIZ"
echo "  Timing : $ENABLE_TIMING  (logs → $LOG_DIR)"
echo "========================================================"

for t in "${KILL_LIST[@]}"; do pkill -9 -f "$t" > /dev/null 2>&1 || true; done
sleep 2

# ── 1. ZED ROS2 Wrapper ───────────────────────────────────────────────────────
echo "[1/4] Checking for ZED camera topics..."
#
# TODO: ZEDX — uncomment and fix this once zed-ros2-wrapper is installed:
#
# ros2 launch zed_wrapper zedx.launch.py camera_model:=zedx \
#     > "${LOG_DIR}/zed_wrapper.log" 2>&1 &
# echo "      Waiting for ZED wrapper to initialize (~10s)..."
# sleep 10
#
# For now the script expects you to start the ZED wrapper yourself first:
echo "      Waiting for ${ZEDX_LEFT_TOPIC} (start ZED wrapper if not running)..."
timeout 30 bash -c \
    "until ros2 topic list 2>/dev/null | grep -q '${ZEDX_LEFT_TOPIC}'; do sleep 0.5; done" \
    || { echo ""; echo "  [ERROR] ZED camera topic not found after 30 s."; \
         echo "          Start 'ros2 launch zed_wrapper zedx.launch.py camera_model:=zedx' first."; \
         exit 1; }
echo "      ZED topics found."

# ── 2. OpenVINS ───────────────────────────────────────────────────────────────
echo "[2/4] Starting OpenVINS..."
if $ENABLE_TIMING; then
    OV_OUT="${LOG_DIR}/openvins.log"
else
    OV_OUT=/dev/null
fi
ros2 launch ov_msckf subscribe.launch.py \
    config_path:="$CONFIG_PATH" \
    use_sim_time:=false \
    > "$OV_OUT" 2>&1 &

timeout 15 bash -c \
    'until ros2 node list 2>/dev/null | grep -q "ov_msckf"; do sleep 0.3; done' || true
sleep 2

# ── 3. YOLO masker ────────────────────────────────────────────────────────────
echo "[3/4] Starting YOLO masker..."
# Topic remapping: the masker's internal subscriptions use /cam0 and /cam1.
# We remap them to the actual ZED topic names here so nothing else changes.
if $ENABLE_TIMING; then
    YOLO_OUT="${LOG_DIR}/yolo_masker.log"
else
    YOLO_OUT=/dev/null
fi
ros2 run yolo_masker yolo_masker \
    --ros-args \
    -p model_path:="${MODEL_PATH}" \
    -p device:="${YOLO_DEVICE:-cuda}" \
    -p confidence_threshold:="${YOLO_CONF}" \
    -p dilation_kernel:=5 \
    -p max_mask_fraction:=0.80 \
    -p use_flow_classifier:=true \
    -p flow_dynamic_threshold:=2.0 \
    -p flow_min_features:=5 \
    --remap /cam0/image_raw:="${ZEDX_LEFT_TOPIC}" \
    --remap /cam1/image_raw:="${ZEDX_RIGHT_TOPIC}" \
    > "$YOLO_OUT" 2>&1 &

echo "      Waiting for /cam0/masked (YOLO model loading ~5s)..."
timeout 60 bash -c \
    'until ros2 topic list 2>/dev/null | grep -q "/cam0/masked"; do sleep 0.5; done' \
    || echo "  [WARN] /cam0/masked not seen — continuing anyway"
sleep 2

# ── 4. Hybrid speed estimator ─────────────────────────────────────────────────
echo "[4/4] Starting hybrid_speed_estimator..."
if $ENABLE_TIMING; then
    HSE_OUT="${LOG_DIR}/hse.log"
else
    HSE_OUT=/dev/null
fi
ros2 run ov_softgate hybrid_speed_estimator 1 \
    --ros-args \
    -p mask_source:=yolo \
    -p min_disparity:=1.2 \
    -p min_features:=8 \
    -p orb_nfeatures:=100 \
    -p calib_file:="${CALIB_FILE}" \
    -p output_dir:="${LOG_DIR}" \
    > "$HSE_OUT" 2>&1 &
sleep 1

# ── Optional: RViz ────────────────────────────────────────────────────────────
if $ENABLE_RVIZ; then
    echo "[OPT] Starting RViz..."
    rviz2 -d "$RVIZ_CFG" --ros-args --log-level warn &
fi

echo ""
echo "========================================================"
echo "  All nodes running. Press Ctrl+C to stop cleanly."
if $ENABLE_TIMING; then
    echo "  Timing logs: ${LOG_DIR}/"
fi
echo "========================================================"
echo ""

wait
