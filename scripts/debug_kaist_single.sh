#!/bin/bash
# Usage: ./scripts/debug_kaist_single.sh [urban38|urban39] [unmasked|imu_only|yolo|yolo_imu]
# Starts all nodes. Play the bag yourself:
#   cd "/media/neurolab/60a72ba2-3a9d-47d0-88e3-852b0f67f283/Downloads_Ext/KAIST Urban"
#   ros2 bag play urban39_10min --clock

SEQUENCE="${1:-urban39}"
CONDITION="${2:-unmasked}"

WORKSPACE="/media/neurolab/60a72ba2-3a9d-47d0-88e3-852b0f67f283"
OPENVINS_WS="${WORKSPACE}/openvins_ws"
MODEL_PATH="${OPENVINS_WS}/models/yolo26s-seg.pt"
CONFIG_BASE="${OPENVINS_WS}/install/ov_msckf/share/ov_msckf/config/kaist_urban"
YOLO_DEVICE="${YOLO_DEVICE:-cuda}"

if [[ "$CONDITION" == "yolo_imu" || "$CONDITION" == "imu_only" ]]; then
    CONFIG="${CONFIG_BASE}/estimator_config_imu_residual.yaml"
else
    CONFIG="${CONFIG_BASE}/estimator_config.yaml"
fi

cleanup() {
    echo "--- cleaning up ---"
    pkill -f "ov_msckf"       2>/dev/null || true
    pkill -f "yolo_masker"    2>/dev/null || true
    pkill -f "rqt_image_view" 2>/dev/null || true
    pkill -f "rviz2"          2>/dev/null || true
}
trap cleanup EXIT INT TERM

source "${OPENVINS_WS}/install/setup.bash"

echo "[1] killing stale nodes..."
cleanup
sleep 2

echo "[2] launching OpenVINS..."
ros2 launch ov_msckf subscribe.launch.py \
    config_path:="$CONFIG" \
    use_sim_time:=true \
    rviz_enable:=true &
sleep 8

echo "[3] launching masker..."
if [[ "$CONDITION" == "yolo" || "$CONDITION" == "yolo_imu" ]]; then
    ros2 run yolo_masker yolo_masker --ros-args \
        -r /cam0/image_raw:=/stereo/left/image_rect \
        -r /cam1/image_raw:=/stereo/right/image_rect \
        -p model_path:="${MODEL_PATH}" -p device:="${YOLO_DEVICE}" \
        -p confidence_threshold:=0.25 -p dilation_kernel:=5 \
        -p max_mask_fraction:=0.80 -p use_flow_classifier:=true \
        -p flow_dynamic_threshold:=5.0 > /dev/null 2>&1 &
else
    ros2 run yolo_masker yolo_masker --ros-args \
        -r /cam0/image_raw:=/stereo/left/image_rect \
        -r /cam1/image_raw:=/stereo/right/image_rect \
        -p model_path:="${MODEL_PATH}" -p device:="${YOLO_DEVICE}" \
        -p force_empty:=true > /dev/null 2>&1 &
fi
sleep 10

echo "[4] opening image viewers..."
ros2 run rqt_image_view rqt_image_view /stereo/left/image_rect > /dev/null 2>&1 &
sleep 1
ros2 run rqt_image_view rqt_image_view /cam0/masked > /dev/null 2>&1 &
sleep 2

echo ""
echo "=== Nodes ready. Play the bag yourself in another terminal: ==="
echo "  cd \"${WORKSPACE}/Downloads_Ext/KAIST Urban\""
echo "  ros2 bag play ${SEQUENCE}_10min --clock"
echo ""
echo "Press Ctrl+C to stop all nodes."
wait
