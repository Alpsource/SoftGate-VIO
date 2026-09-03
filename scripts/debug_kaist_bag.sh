#!/bin/bash
# =============================================================================
# debug_kaist_bag.sh
# Play a KAIST Complex Urban ROS2 bag through OpenVINS (config/kaist_urban) and
# show the live feature-tracking overlay (/ov_msckf/trackhist) so you can
# visually confirm tracking/initialization before committing to a full collection run.
#
# Usage:
#   ./debug_kaist_bag.sh                       # urban39, first 30s
#   ./debug_kaist_bag.sh urban38                # urban38, first 30s
#   ./debug_kaist_bag.sh urban39 90 30          # urban39, seconds 90-120 (skip first 90s)
#   ./debug_kaist_bag.sh /path/to/existing_bag_dir --reuse   # play an existing bag as-is
#
# OpenVINS console output is always saved to /tmp/ov_log_kaist_debug.txt and
# tailed at the end — rqt_image_view / ros2 bag piping can swallow stdout, so
# this is the reliable way to see init/divergence messages after the run.
# =============================================================================

set -euo pipefail

WORKSPACE="/media/neurolab/60a72ba2-3a9d-47d0-88e3-852b0f67f283"
OPENVINS_WS="${WORKSPACE}/openvins_ws"
KAIST_DIR="${WORKSPACE}/Downloads_Ext/KAIST Urban"
OV_LOG="/tmp/ov_log_kaist_debug.txt"

SEQ="${1:-urban39}"
START_SEC="${2:-0}"
DEBUG_SEC="${3:-30}"

CONFIG_PATH="${OPENVINS_WS}/install/ov_msckf/share/ov_msckf/config/kaist_urban/estimator_config.yaml"

if [[ ! -f "$CONFIG_PATH" ]]; then
    echo "[ERROR] kaist_urban config not installed at: $CONFIG_PATH"
    echo "        Run: colcon build --symlink-install --packages-select ov_msckf"
    exit 1
fi

if [[ "$SEQ" == /* ]]; then
    BAG_DIR="$SEQ"
else
    SEQ_DIR="${KAIST_DIR}/${SEQ}-pankyo"
    BAG_DIR="${KAIST_DIR}/${SEQ}_debug_s${START_SEC}_d${DEBUG_SEC}"
    if [[ ! -d "$BAG_DIR" ]]; then
        echo "══════════════════════════════════════════════════════════"
        echo "  Converting ${SEQ} [${START_SEC}s -> ${START_SEC}+${DEBUG_SEC}s] -> ROS2 bag"
        echo "══════════════════════════════════════════════════════════"
        if [[ ! -d "$SEQ_DIR" ]]; then
            echo "[ERROR] Raw sequence not found: $SEQ_DIR"; exit 1
        fi
        python3 "${OPENVINS_WS}/convert_kaist_to_ros2bag.py" \
            --seq_dir "$SEQ_DIR" \
            --out_bag "$BAG_DIR" \
            --start_sec "$START_SEC" \
            --duration_sec "$DEBUG_SEC"
    else
        echo "[*] Reusing existing debug bag: $BAG_DIR"
    fi
fi

cleanup() {
    echo; echo "[*] Cleaning up ..."
    pkill -9 -f "rqt_image_view" > /dev/null 2>&1 || true
    pkill -9 -f "subscribe.launch" > /dev/null 2>&1 || true
    pkill -9 -f "run_subscribe_msckf" > /dev/null 2>&1 || true
}
trap cleanup EXIT

echo "[*] Killing any leftover OpenVINS/rqt processes from earlier runs ..."
cleanup
sleep 1

set +u
source /opt/ros/humble/setup.bash
source "${OPENVINS_WS}/install/setup.bash"
set -u

ros2 daemon stop > /dev/null 2>&1 || true; sleep 1
ros2 daemon start > /dev/null 2>&1 || true; sleep 1

echo "══════════════════════════════════════════════════════════"
echo "  Launching OpenVINS (kaist_urban config)"
echo "══════════════════════════════════════════════════════════"
rm -f "$OV_LOG"
ros2 launch ov_msckf subscribe.launch.py \
    config_path:="$CONFIG_PATH" \
    use_sim_time:=true > "$OV_LOG" 2>&1 &
sleep 3

echo "[*] Opening feature-tracking viewer (/ov_msckf/trackhist) ..."
ros2 run rqt_image_view rqt_image_view /ov_msckf/trackhist > /dev/null 2>&1 &
sleep 1

echo ""
echo "  -> Watch the rqt_image_view window for tracked features (colored dots/lines)."
echo "  -> OpenVINS console output is being saved to: $OV_LOG"
echo ""
echo "  -> Playing bag: $BAG_DIR"
ros2 bag play "$BAG_DIR" --clock
echo "  -> Bag finished."

sleep 1
echo ""
echo "══════════════════════════════════════════════════════════"
echo "  OpenVINS log summary (last 40 lines of $OV_LOG)"
echo "══════════════════════════════════════════════════════════"
tail -40 "$OV_LOG" || true
echo ""
if grep -qi "successful initialization" "$OV_LOG" 2>/dev/null; then
    echo "[RESULT] OpenVINS INITIALIZED during this clip."
else
    echo "[RESULT] OpenVINS did NOT initialize during this clip."
fi
