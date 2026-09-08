#!/bin/bash
# =============================================================================
# run_timing_test.sh — Per-component latency measurement
#
# Runs one bag replay with all nodes, captures each node's stdout to a separate
# log file, then prints a combined timing table via parse_timing.py.
#
# Usage:
#   ./scripts/run_timing_test.sh [DATASET] [LEVEL] [MASK_MODE]
#
#   DATASET   : parking_lot | city_day | city_night  (default: parking_lot)
#   LEVEL     : none | low | mid | high              (default: high)
#   MASK_MODE : unmasked | yolo | yolo_imu           (default: yolo)
#
# Examples:
#   ./scripts/run_timing_test.sh
#   ./scripts/run_timing_test.sh city_day high yolo
#   ./scripts/run_timing_test.sh parking_lot high unmasked
# =============================================================================

DATASET="${1:-parking_lot}"
LEVEL="${2:-high}"
MASK_MODE="${3:-yolo}"

# Derive workspace from script location — works regardless of where the repo is cloned.
_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OPENVINS_WS="${OPENVINS_WS:-$(dirname "$_SCRIPT_DIR")}"
# VIODE dataset and results live outside the repo — override these two with env vars on each machine.
VIODE_DATASET="${VIODE_DATASET:-/media/neurolab/60a72ba2-3a9d-47d0-88e3-852b0f67f283/Downloads_Ext/VIODE_Dataset}"
RESULTS_BASE="${RESULTS_BASE:-/media/neurolab/60a72ba2-3a9d-47d0-88e3-852b0f67f283/sim_results}"
CONFIG_BASE="${OPENVINS_WS}/src/open_vins/config/viode_config"
MODEL_PATH="${OPENVINS_WS}/models/yolo26s-seg.pt"
OTP_SCRIPT="${VIODE_DATASET}/odom_to_path.py"
PARSE_SCRIPT="${OPENVINS_WS}/scripts/parse_timing.py"

LOG_DIR="/tmp/timing_logs_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"

# ── Config selection (same logic as run_phase1_evaluation.sh) ─────────────────
if [[ "$DATASET" == city_* ]]; then
    if [[ "$MASK_MODE" == "yolo_imu" || "$MASK_MODE" == "imu_only" ]]; then
        CONFIG_PATH="${CONFIG_BASE}/estimator_config_city_imu_residual.yaml"
    else
        CONFIG_PATH="${CONFIG_BASE}/estimator_config_city.yaml"
    fi
    CONF_THRESH=0.45
else
    if [[ "$MASK_MODE" == "yolo_imu" || "$MASK_MODE" == "imu_only" ]]; then
        CONFIG_PATH="${CONFIG_BASE}/estimator_config_imu_residual.yaml"
    else
        CONFIG_PATH="${CONFIG_BASE}/estimator_config.yaml"
    fi
    CONF_THRESH=0.25
fi

BAG_PATH="${VIODE_DATASET}/${DATASET}_${LEVEL}"
LABEL="${MASK_MODE} — ${DATASET}/${LEVEL}"

KILL_LIST=(
    "ov_msckf" "ov_softgate" "yolo_masker" "odom_to_path.py"
    "static_transform_publisher" "path_recorder" "hybrid_speed_estimator" "masker"
)

cleanup_nodes() {
    echo ""
    echo "[CLEANUP] Stopping nodes..."
    for t in "${KILL_LIST[@]}"; do pkill -f "$t" > /dev/null 2>&1 || true; done
    sleep 1
    for t in "${KILL_LIST[@]}"; do pkill -9 -f "$t" > /dev/null 2>&1 || true; done
    ros2 daemon stop  > /dev/null 2>&1 || true
    pkill -9 -f ros2  > /dev/null 2>&1 || true
    pkill -9 -f fastdds > /dev/null 2>&1 || true
    rm -rf ~/.ros/ros2daemon /dev/shm/fastrtps_* /dev/shm/rtps_* 2>/dev/null || true
    sleep 2
    ros2 daemon start > /dev/null 2>&1 || true
    timeout 15 bash -c 'until ros2 node list > /dev/null 2>&1; do sleep 0.5; done' || true
    echo "[CLEANUP] Done."
}

source "${OPENVINS_WS}/install/setup.bash"
trap cleanup_nodes INT TERM

# ── Preflight ─────────────────────────────────────────────────────────────────
if [[ ! -d "$BAG_PATH" ]]; then
    echo "ERROR: Bag not found: ${BAG_PATH}"
    exit 1
fi

echo "========================================================"
echo "  Timing test: ${MASK_MODE} — ${DATASET}/${LEVEL}"
echo "  Config : $(basename $CONFIG_PATH)"
echo "  YOLO conf: ${CONF_THRESH}"
echo "  Logs   : ${LOG_DIR}/"
echo "========================================================"

for t in "${KILL_LIST[@]}"; do pkill -9 -f "$t" > /dev/null 2>&1 || true; done
sleep 2

# ── 1. OpenVINS ───────────────────────────────────────────────────────────────
echo "[1/6] Starting OpenVINS..."
ros2 launch ov_msckf subscribe.launch.py \
    config_path:="$CONFIG_PATH" use_sim_time:=true verbosity:=ALL \
    > "${LOG_DIR}/openvins.log" 2>&1 &

timeout 15 bash -c \
    'until ros2 node list 2>/dev/null | grep -q "ov_msckf"; do sleep 0.3; done' || true
sleep 2

# ── 2. YOLO masker ────────────────────────────────────────────────────────────
echo "[2/6] Starting YOLO masker (mask_mode=${MASK_MODE})..."
if [[ "$MASK_MODE" == "yolo" || "$MASK_MODE" == "yolo_imu" ]]; then
    ros2 run yolo_masker yolo_masker --ros-args \
        -p model_path:="${MODEL_PATH}" \
        -p device:="${YOLO_DEVICE:-cuda}" \
        -p confidence_threshold:="${CONF_THRESH}" \
        -p dilation_kernel:=5 \
        -p max_mask_fraction:=0.80 \
        -p use_flow_classifier:=true \
        -p flow_dynamic_threshold:=2.0 \
        -p flow_min_features:=5 \
        -p use_clahe:=false \
        > "${LOG_DIR}/yolo_masker.log" 2>&1 &
else
    ros2 run yolo_masker yolo_masker --ros-args \
        -p model_path:="${MODEL_PATH}" \
        -p device:="${YOLO_DEVICE:-cuda}" \
        -p force_empty:=true \
        -p use_clahe:=false \
        > "${LOG_DIR}/yolo_masker.log" 2>&1 &
fi

echo "  Waiting for /cam0/masked..."
timeout 60 bash -c \
    'until ros2 topic list 2>/dev/null | grep -q "/cam0/masked"; do sleep 0.5; done' \
    || echo "  [WARN] /cam0/masked not seen — continuing"
sleep 5

# ── 3. GT path converter ──────────────────────────────────────────────────────
echo "[3/6] Starting GT path converter..."
python3 "$OTP_SCRIPT" \
    --ros-args -p target_frame:=gt_frame -p scale_factor:=1.0 \
    > "${LOG_DIR}/odom_to_path.log" 2>&1 &

# ── 4. Static TF publishers ───────────────────────────────────────────────────
echo "[4/6] Starting TF publishers..."
ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 --yaw -1.57 --pitch 0 --roll 3.14159 \
    --frame-id global --child-frame-id gt_frame \
    --ros-args -p use_sim_time:=true > /dev/null 2>&1 &

ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 --qx 0.5 --qy 0.5 --qz 0.5 --qw 0.5 \
    --frame-id imu --child-frame-id cam0 \
    --ros-args -p use_sim_time:=false > /dev/null 2>&1 &

ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0.05 --z 0 --qx 0.5 --qy 0.5 --qz 0.5 --qw 0.5 \
    --frame-id imu --child-frame-id cam1 \
    --ros-args -p use_sim_time:=false > /dev/null 2>&1 &

# ── 5. Hybrid speed estimator ─────────────────────────────────────────────────
echo "[5/6] Starting hybrid_speed_estimator..."
ros2 run ov_softgate hybrid_speed_estimator 1 \
    --ros-args \
    -p mask_source:=yolo \
    -p min_disparity:=1.2 \
    -p min_features:=8 \
    -p orb_nfeatures:=100 \
    -p calib_file:="${OPENVINS_WS}/src/open_vins/config/viode_config/kalibr_imucam_chain.yaml" \
    -p output_dir:="${RESULTS_BASE}" \
    > "${LOG_DIR}/hse.log" 2>&1 &
sleep 1

# ── 6. Play bag ───────────────────────────────────────────────────────────────
echo "[6/6] Playing bag: $(basename $BAG_PATH)"
echo "      (watching for [TIMING] output...)"
echo ""
ros2 bag play "$BAG_PATH" --clock --read-ahead-queue-size 10000
echo ""
echo "Bag finished."

sleep 2
cleanup_nodes

# ── Parse results ─────────────────────────────────────────────────────────────
COMBINED_LOG="${LOG_DIR}/combined.log"
cat "${LOG_DIR}/openvins.log" \
    "${LOG_DIR}/yolo_masker.log" \
    "${LOG_DIR}/hse.log" \
    > "$COMBINED_LOG" 2>/dev/null

echo ""
echo "========================================================"
echo "  Logs saved to: ${LOG_DIR}/"
echo "  Combined: ${COMBINED_LOG}"
echo "========================================================"
echo ""

python3 "$PARSE_SCRIPT" "$COMBINED_LOG" --label "$LABEL"

echo ""
echo "  To re-parse later:"
echo "  python3 scripts/parse_timing.py ${COMBINED_LOG} --label \"${LABEL}\""
echo ""
