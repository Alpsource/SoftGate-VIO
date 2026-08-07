#!/bin/bash
# =============================================================================
# debug_run.sh — Interactive one-shot run with RViz + rqt_image_view
#
# Usage:
#   ./debug_run.sh [DATASET] [LEVEL] [MASK_SOURCE] [IMU_RESIDUAL]
#
#   DATASET      : parking_lot | city_day | city_night  (default: parking_lot)
#   LEVEL        : none | low | mid | high              (default: high)
#   MASK_SOURCE  : none | gt | yolo                     (default: yolo)
#                  none  → all-zeros masks, pure OpenVINS
#                  gt    → ground-truth segmentation masks
#                  yolo  → YOLO neural-net masks
#   IMU_RESIDUAL : true | false                         (default: false)
#
# Examples:
#   ./debug_run.sh parking_lot high none false   # pure OpenVINS, no masking
#   ./debug_run.sh parking_lot high gt   true    # GT masking + IMU residual
#   ./debug_run.sh parking_lot high yolo false   # YOLO masking only
# =============================================================================

WORKSPACE="/media/neurolab/60a72ba2-3a9d-47d0-88e3-852b0f67f283"   # ← set this to your data root
OPENVINS_WS="${WORKSPACE}/openvins_ws"
VIODE_DATASET="${WORKSPACE}/Downloads_Ext/VIODE_Dataset"
OTP_SCRIPT="${VIODE_DATASET}/odom_to_path.py"
CONFIG_PATH="${OPENVINS_WS}/install/ov_msckf/share/ov_msckf/config/viode_config/estimator_config.yaml"
RVIZ_CONFIG="${OPENVINS_WS}/src/open_vins/ov_msckf/launch/display_ros2.rviz"
SETUP="${OPENVINS_WS}/install/setup.bash"

DATASET="${1:-parking_lot}"
LEVEL="${2:-high}"
MASK_SOURCE="${3:-yolo}"   # yolo | gt | none
IMU_RESIDUAL="${4:-false}" # true | false
YOLO_DEVICE="${YOLO_DEVICE:-cuda}"  # cuda or cpu; override: YOLO_DEVICE=cpu ./debug_run.sh
BAG_PATH="${VIODE_DATASET}/${DATASET}_${LEVEL}"

source "${SETUP}"

SOURCE_CMD="source ${SETUP}"

new_term() {
    local title="$1"; local cmd="$2"
    gnome-terminal --title="$title" -- bash -c "${SOURCE_CMD}; ${cmd}; echo '--- ENDED ---'; read" &
    sleep 0.3
}

echo "============================================================"
echo "  DEBUG RUN: ${DATASET}_${LEVEL}  masker=${MASK_SOURCE}  imu_residual=${IMU_RESIDUAL}"
echo "============================================================"

# ── Full cleanup (same as run_all_experiments_v2.sh) ───────────────────
echo "[CLEANUP] Stopping all ROS2 nodes..."
for t in run_subscribe_msckf yolo_masker odom_to_path.py path_recorder hybrid_speed_estimator masker rviz2 rqt_image_view static_transform_publisher; do
    pkill -9 -f "$t" > /dev/null 2>&1 || true
done
ros2 daemon stop    > /dev/null 2>&1 || true
pkill -9 -f ros2    > /dev/null 2>&1 || true
pkill -9 -f fastdds > /dev/null 2>&1 || true
rm -rf ~/.ros/ros2daemon   2>/dev/null || true
rm -rf /dev/shm/fastrtps_* 2>/dev/null || true
rm -rf /dev/shm/rtps_*     2>/dev/null || true
sleep 2
ros2 daemon start > /dev/null 2>&1 || true
echo "[CLEANUP] Done."

# ── 1. OpenVINS ────────────────────────────────────────────────────────
new_term "OpenVINS" "ros2 launch ov_msckf subscribe.launch.py \
    config_path:=${CONFIG_PATH} \
    use_sim_time:=true \
    use_imu_residual:=${IMU_RESIDUAL}"
echo "OpenVINS launching..."
sleep 3

# ── 2. Masker ──────────────────────────────────────────────────────────
if [[ "$MASK_SOURCE" == "yolo" ]]; then
    new_term "yolo_masker" \
        "ros2 run yolo_masker yolo_masker --ros-args -p model_path:=${OPENVINS_WS}/models/yolo26s-seg.pt -p device:=${YOLO_DEVICE}"
elif [[ "$MASK_SOURCE" == "none" ]]; then
    # All-zeros masks → pure OpenVINS, no dynamic object masking
    new_term "masker_empty" "ros2 run ov_softgate masker --ros-args -p force_empty:=true"
else
    new_term "semantic_masker" "ros2 run ov_softgate masker"
fi

# ── 3. Ground-truth path converter ─────────────────────────────────────
new_term "odom_to_path" "python3 ${OTP_SCRIPT} \
    --ros-args -p target_frame:=gt_frame -p scale_factor:=1.0"

# ── 4. Static TF publishers ─────────────────────────────────────────────
new_term "tf_publishers" "\
ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 \
    --yaw -1.57 --pitch 0 --roll 3.14159 \
    --frame-id global --child-frame-id gt_frame \
    --ros-args -p use_sim_time:=true & \
ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 \
    --qx 0.5 --qy 0.5 --qz 0.5 --qw 0.5 \
    --frame-id imu --child-frame-id cam0 & \
ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0.05 --z 0 \
    --qx 0.5 --qy 0.5 --qz 0.5 --qw 0.5 \
    --frame-id imu --child-frame-id cam1 & \
wait"

# ── 5. hybrid_speed_estimator ───────────────────────────────────────────
new_term "speed_estimator" "ros2 run ov_softgate hybrid_speed_estimator 1 \
    --ros-args -p mask_source:=${MASK_SOURCE}"

sleep 0.5

# ── 6. RViz ─────────────────────────────────────────────────────────────
new_term "RViz" "ros2 run rviz2 rviz2 -d ${RVIZ_CONFIG}"

# ── 7. rqt_image_view ───────────────────────────────────────────────────
new_term "rqt_image_view" "ros2 run rqt_image_view rqt_image_view"

echo ""
echo "Nodes up. In rqt_image_view select: /ov_msckf/trackhist"
echo ""
if [[ "$MASK_SOURCE" == "yolo" ]]; then
    echo "Waiting for yolo_masker (GPU warm-up ~20 s)..."
    timeout 60 bash -c \
        "${SOURCE_CMD}; until ros2 node list 2>/dev/null | grep -q 'yolo_masker'; do sleep 0.5; done" \
        && sleep 3 && echo "  -> yolo_masker ready." \
        || echo "  [WARN] yolo_masker timeout — check its terminal."
fi
echo ""
echo "Press Enter to start bag playback, or Ctrl+C to abort."
read -r

# ── 8. Bag playback ─────────────────────────────────────────────────────
echo "Playing: ${BAG_PATH}"
ros2 bag play "${BAG_PATH}" --clock
echo ""
echo "Bag finished."
