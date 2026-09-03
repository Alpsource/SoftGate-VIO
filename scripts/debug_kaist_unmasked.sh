#!/bin/bash
# Debug script: runs OpenVINS with the plain (unmasked) config with full visible output.
# Plays 90 seconds of urban38 so we can see whether it initializes or crashes.
#
# Usage: ./scripts/debug_kaist_unmasked.sh
# Run from workspace root. Ctrl+C to stop early.

set -euo pipefail

WORKSPACE="/media/neurolab/60a72ba2-3a9d-47d0-88e3-852b0f67f283"
OPENVINS_WS="${WORKSPACE}/openvins_ws"
KAIST_DIR="${WORKSPACE}/Downloads_Ext/KAIST Urban"
MODEL_PATH="${OPENVINS_WS}/models/yolo26s-seg.pt"
CONFIG="${OPENVINS_WS}/install/ov_msckf/share/ov_msckf/config/kaist_urban/estimator_config.yaml"
LOG="${OPENVINS_WS}/scripts/debug_kaist_unmasked.log"

set +u
source "${OPENVINS_WS}/install/setup.bash"
set -u

echo "=== KAIST unmasked debug run ==="
echo "Config: $CONFIG"
echo "use_imu_residual: $(grep 'use_imu_residual' "$CONFIG" | head -1)"
echo "Log: $LOG"
echo ""

# Kill any stale nodes
for t in ov_msckf yolo_masker; do
    pkill -9 -f "$t" 2>/dev/null || true
done
ros2 daemon stop 2>/dev/null || true
sleep 2
ros2 daemon start 2>/dev/null || true
sleep 3

# 1. OpenVINS — full stderr visible AND saved to log
echo "[1] Starting OpenVINS..."
ros2 launch ov_msckf subscribe.launch.py \
    config_path:="$CONFIG" \
    use_sim_time:=true 2>&1 | tee "$LOG" &
OV_PID=$!

# Wait for node
timeout 20 bash -c \
    'until ros2 node list 2>/dev/null | grep -q "ov_msckf"; do sleep 0.5; done' \
    || echo "[WARN] ov_msckf node not seen — may have crashed immediately"
sleep 2

# 2. Masker with force_empty
echo "[2] Starting yolo_masker (force_empty)..."
ros2 run yolo_masker yolo_masker \
    --ros-args \
    -r /cam0/image_raw:=/stereo/left/image_rect \
    -r /cam1/image_raw:=/stereo/right/image_rect \
    -p model_path:="${MODEL_PATH}" \
    -p force_empty:=true \
    > /dev/null 2>&1 &

echo "   Waiting for /cam0/masked..."
timeout 60 bash -c \
    'until ros2 topic list 2>/dev/null | grep -q "/cam0/masked"; do sleep 0.5; done' \
    || echo "[WARN] /cam0/masked not seen"
sleep 5

# 3. Play 90 seconds only
echo "[3] Playing urban38 (90 seconds)..."
ros2 bag play "${KAIST_DIR}/urban38_10min" \
    --clock \
    --read-ahead-queue-size 10000 \
    --duration 90 \
    2>/dev/null

echo ""
echo "=== Bag finished. Check output above and: $LOG ==="
echo "Key things to look for:"
echo "  - 'INITIALIZED' or 'initializing' → init succeeded or in progress"
echo "  - 'Propagator unable to propagate' → IMU gap / timestamp issue"
echo "  - 'waiting for enough clone states' → init still waiting"
echo "  - Segfault / terminate / exception → crash"
echo ""

# Show last 40 lines of log
echo "--- Last 40 lines of OpenVINS output ---"
tail -40 "$LOG" 2>/dev/null || echo "(log empty)"

# Cleanup
for t in ov_msckf yolo_masker; do
    pkill -f "$t" 2>/dev/null || true
done
