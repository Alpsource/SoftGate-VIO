#!/bin/bash
# =============================================================================
# run_timing_all.sh — Full timing sweep across conditions and environments
#
# Runs each (mask_mode × dataset × level) combination once, saves logs and
# per-run timing tables to a timestamped timings/ folder, then prints a
# combined summary across all runs at the end.
#
# Usage:
#   ./scripts/run_timing_all.sh [LEVEL]
#
#   LEVEL : none | low | mid | high | all   (default: high)
#           "all" runs all 4 density levels (slow — 36 runs)
#
# Default (high only): 3 conditions × 3 environments = 9 runs ≈ 10 min
# =============================================================================

LEVEL_ARG="${1:-high}"

WORKSPACE="/media/neurolab/60a72ba2-3a9d-47d0-88e3-852b0f67f283"
OPENVINS_WS="${WORKSPACE}/openvins_ws"
VIODE_DATASET="${WORKSPACE}/Downloads_Ext/VIODE_Dataset"
RESULTS_BASE="${WORKSPACE}/sim_results"
CONFIG_BASE="${OPENVINS_WS}/install/ov_msckf/share/ov_msckf/config/viode_config"
MODEL_PATH="${OPENVINS_WS}/models/yolo26s-seg.pt"
OTP_SCRIPT="${VIODE_DATASET}/odom_to_path.py"
PARSE_SCRIPT="${OPENVINS_WS}/scripts/parse_timing.py"

TIMINGS_DIR="${OPENVINS_WS}/timings/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$TIMINGS_DIR"

MASK_MODES=("unmasked" "yolo" "yolo_imu")
DATASETS=("parking_lot" "city_day" "city_night")
if [[ "$LEVEL_ARG" == "all" ]]; then
    LEVELS=("none" "low" "mid" "high")
else
    LEVELS=("$LEVEL_ARG")
fi

KILL_LIST=(
    "ov_msckf" "ov_softgate" "yolo_masker" "odom_to_path.py"
    "static_transform_publisher" "path_recorder" "hybrid_speed_estimator" "masker"
)

cleanup_nodes() {
    for t in "${KILL_LIST[@]}"; do pkill -f  "$t" > /dev/null 2>&1 || true; done
    sleep 1
    for t in "${KILL_LIST[@]}"; do pkill -9 -f "$t" > /dev/null 2>&1 || true; done
    ros2 daemon stop  > /dev/null 2>&1 || true
    pkill -9 -f ros2   > /dev/null 2>&1 || true
    pkill -9 -f fastdds > /dev/null 2>&1 || true
    rm -rf ~/.ros/ros2daemon /dev/shm/fastrtps_* /dev/shm/rtps_* 2>/dev/null || true
    sleep 2
    ros2 daemon start > /dev/null 2>&1 || true
    timeout 15 bash -c 'until ros2 node list > /dev/null 2>&1; do sleep 0.5; done' || true
}

run_one() {
    local mask_mode="$1"
    local dataset="$2"
    local level="$3"
    local log_dir="${TIMINGS_DIR}/${mask_mode}-${dataset}-${level}"
    mkdir -p "$log_dir"

    # Config selection
    local config_path conf_thresh
    if [[ "$dataset" == city_* ]]; then
        conf_thresh=0.45
        if [[ "$mask_mode" == "yolo_imu" || "$mask_mode" == "imu_only" ]]; then
            config_path="${CONFIG_BASE}/estimator_config_city_imu_residual.yaml"
        else
            config_path="${CONFIG_BASE}/estimator_config_city.yaml"
        fi
    else
        conf_thresh=0.25
        if [[ "$mask_mode" == "yolo_imu" || "$mask_mode" == "imu_only" ]]; then
            config_path="${CONFIG_BASE}/estimator_config_imu_residual.yaml"
        else
            config_path="${CONFIG_BASE}/estimator_config.yaml"
        fi
    fi

    local bag_path="${VIODE_DATASET}/${dataset}_${level}"
    if [[ ! -d "$bag_path" ]]; then
        echo "  [SKIP] Bag not found: ${bag_path}"
        return
    fi

    echo ""
    echo "  ── ${mask_mode} / ${dataset} / ${level} ──────────────────────────────"

    for t in "${KILL_LIST[@]}"; do pkill -9 -f "$t" > /dev/null 2>&1 || true; done
    sleep 2

    # OpenVINS
    ros2 launch ov_msckf subscribe.launch.py \
        config_path:="$config_path" use_sim_time:=true \
        > "${log_dir}/openvins.log" 2>&1 &
    timeout 15 bash -c \
        'until ros2 node list 2>/dev/null | grep -q "ov_msckf"; do sleep 0.3; done' || true
    sleep 2

    # YOLO masker
    if [[ "$mask_mode" == "yolo" || "$mask_mode" == "yolo_imu" ]]; then
        ros2 run yolo_masker yolo_masker --ros-args \
            -p model_path:="${MODEL_PATH}" \
            -p device:="${YOLO_DEVICE:-cuda}" \
            -p confidence_threshold:="${conf_thresh}" \
            -p dilation_kernel:=5 \
            -p max_mask_fraction:=0.80 \
            -p use_flow_classifier:=true \
            -p flow_dynamic_threshold:=2.0 \
            -p flow_min_features:=5 \
            > "${log_dir}/yolo_masker.log" 2>&1 &
    else
        ros2 run yolo_masker yolo_masker --ros-args \
            -p model_path:="${MODEL_PATH}" \
            -p device:="${YOLO_DEVICE:-cuda}" \
            -p force_empty:=true \
            > "${log_dir}/yolo_masker.log" 2>&1 &
    fi
    timeout 60 bash -c \
        'until ros2 topic list 2>/dev/null | grep -q "/cam0/masked"; do sleep 0.5; done' \
        > /dev/null 2>&1 || true
    sleep 5

    # GT + TF
    python3 "$OTP_SCRIPT" \
        --ros-args -p target_frame:=gt_frame -p scale_factor:=1.0 > /dev/null 2>&1 &
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

    # HSE
    ros2 run ov_softgate hybrid_speed_estimator 1 \
        --ros-args \
        -p mask_source:=yolo \
        -p min_disparity:=1.2 \
        -p min_features:=8 \
        -p orb_nfeatures:=100 \
        -p calib_file:="${OPENVINS_WS}/src/open_vins/config/viode_config/kalibr_imucam_chain.yaml" \
        -p output_dir:="${RESULTS_BASE}" \
        > "${log_dir}/hse.log" 2>&1 &
    sleep 1

    # Play bag
    ros2 bag play "$bag_path" --clock --read-ahead-queue-size 10000 > /dev/null 2>&1
    sleep 2
    cleanup_nodes

    # Combine logs
    cat "${log_dir}/openvins.log" \
        "${log_dir}/yolo_masker.log" \
        "${log_dir}/hse.log" \
        > "${log_dir}/combined.log" 2>/dev/null

    # Per-run timing table saved to file
    python3 "$PARSE_SCRIPT" "${log_dir}/combined.log" \
        --label "${mask_mode} — ${dataset}/${level}" \
        > "${log_dir}/timing_table.txt" 2>&1

    echo "  -> Done. Table: ${log_dir}/timing_table.txt"
}

# ── Source ROS ────────────────────────────────────────────────────────────────
source "${OPENVINS_WS}/install/setup.bash"
trap cleanup_nodes INT TERM

total=$(( ${#MASK_MODES[@]} * ${#DATASETS[@]} * ${#LEVELS[@]} ))
echo "========================================================"
echo "  Timing sweep — ${total} runs"
echo "  Conditions : ${MASK_MODES[*]}"
echo "  Datasets   : ${DATASETS[*]}"
echo "  Levels     : ${LEVELS[*]}"
echo "  Output     : ${TIMINGS_DIR}/"
echo "========================================================"

run_num=0
for mask_mode in "${MASK_MODES[@]}"; do
    for dataset in "${DATASETS[@]}"; do
        for level in "${LEVELS[@]}"; do
            run_num=$((run_num + 1))
            echo ""
            echo "[${run_num}/${total}] ${mask_mode} / ${dataset} / ${level}"
            run_one "$mask_mode" "$dataset" "$level"
        done
    done
done

# ── Combined summary across all runs ─────────────────────────────────────────
echo ""
echo "========================================================"
echo "  ALL RUNS COMPLETE — Combined summary"
echo "  Results saved to: ${TIMINGS_DIR}/"
echo "========================================================"
echo ""

# Print each per-run table in order
for mask_mode in "${MASK_MODES[@]}"; do
    for dataset in "${DATASETS[@]}"; do
        for level in "${LEVELS[@]}"; do
            table="${TIMINGS_DIR}/${mask_mode}-${dataset}-${level}/timing_table.txt"
            [[ -f "$table" ]] && cat "$table"
        done
    done
done

# Save combined summary to file
SUMMARY="${TIMINGS_DIR}/summary_all.txt"
echo "Timing sweep — $(date)" > "$SUMMARY"
echo "Conditions: ${MASK_MODES[*]} | Levels: ${LEVELS[*]}" >> "$SUMMARY"
echo "" >> "$SUMMARY"
for mask_mode in "${MASK_MODES[@]}"; do
    for dataset in "${DATASETS[@]}"; do
        for level in "${LEVELS[@]}"; do
            table="${TIMINGS_DIR}/${mask_mode}-${dataset}-${level}/timing_table.txt"
            [[ -f "$table" ]] && cat "$table" >> "$SUMMARY"
        done
    done
done

echo ""
echo "  Full summary saved to: ${SUMMARY}"
echo "  To re-analyze one run:"
echo "  python3 scripts/parse_timing.py ${TIMINGS_DIR}/<condition>/combined.log"
