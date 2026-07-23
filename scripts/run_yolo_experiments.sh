#!/bin/bash
# =============================================================================
# run_yolo_experiments.sh
# VIODE-VIO YOLO Masker Experiment Runner
#
# Usage:
#   ./run_yolo_experiments.sh [RUNS_PER_SCENARIO]
#
#   RUNS_PER_SCENARIO : repetitions per scenario (default: 20)
#
# Runs masked (YOLO) and unmasked conditions for all 3 envs × 4 densities.
# Saves to ablation_batches/batch_N/ (auto-numbered after existing batches).
# Uses yolo_masker with ego-motion-compensated optical flow classifier.
#
# Unmasked condition still launches yolo_masker with force_empty:=true
# to keep identical 4-topic sync overhead in both conditions.
# =============================================================================

WORKSPACE="/path/to/your/workspace"   # ← set this to your data root
RESULTS_BASE="${WORKSPACE}/sim_results"
OPENVINS_WS="${WORKSPACE}/openvins_ws"
VIODE_DATASET="${WORKSPACE}/Downloads_Ext/VIODE_Dataset"
OTP_SCRIPT="${VIODE_DATASET}/odom_to_path.py"
CONFIG_PATH="${OPENVINS_WS}/install/ov_msckf/share/ov_msckf/config/viode_config/estimator_config.yaml"
DOV_SCRIPT="${RESULTS_BASE}/dov_postprocessor.py"
ANALYZE_SCRIPT="${RESULTS_BASE}/analyze_results_v5.py"
VENV_PYTHON="${RESULTS_BASE}/env/bin/python3"
BATCH_ROOT="${RESULTS_BASE}/ablation_batches"
MODEL_PATH="${OPENVINS_WS}/models/yolo26s-seg.pt"

RUNS_PER_SCENARIO="${1:-20}"

# ── YOLO parameters (tune here if needed) ────────────────────────────────────
DILATION_KERNEL=9
MAX_MASK_FRACTION=0.80
MIN_DISPARITY=1.2
MIN_FEATURES=8
ORB_NFEATURES=100
CONF_THRESHOLD=0.25
FLOW_DYNAMIC_THRESHOLD=2.0
FLOW_MIN_FEATURES=5
USE_FLOW_CLASSIFIER=true
# ─────────────────────────────────────────────────────────────────────────────

declare -A DENSITY_LEVELS
DENSITY_LEVELS["parking_lot"]="none low mid high"
DENSITY_LEVELS["city_day"]="none low mid high"
DENSITY_LEVELS["city_night"]="none low mid high"

DATASETS=("parking_lot" "city_day" "city_night")
MASK_MODES=("unmasked" "masked")

KILL_LIST=(
    "ov_msckf"
    "ov_softgate"
    "yolo_masker"
    "odom_to_path.py"
    "static_transform_publisher"
    "path_recorder"
    "hybrid_speed_estimator"
    "masker"
)

# ── Auto-number: find next available batch_N ──────────────────────────────────
BATCH_NUM=1
while [[ -d "${BATCH_ROOT}/batch_${BATCH_NUM}" ]]; do
    BATCH_NUM=$((BATCH_NUM + 1))
done
BATCH_OUT="${BATCH_ROOT}/batch_${BATCH_NUM}"
ANALYSIS_LOG="${BATCH_OUT}/analysis.log"
PARAMS_FILE="${BATCH_OUT}/params.txt"

BATCH_LABEL="yolo_flow${FLOW_DYNAMIC_THRESHOLD}px_conf${CONF_THRESHOLD}"

# ── CLEANUP ───────────────────────────────────────────────────────────────────
cleanup_nodes() {
    echo "  [CLEANUP] Stopping all nodes..."
    timeout 5 ros2 topic pub --once /recorder/save std_msgs/msg/Bool "{data: true}" \
        > /dev/null 2>&1 || true
    sleep 2
    for target in "${KILL_LIST[@]}"; do
        pkill -f "$target" > /dev/null 2>&1 || true
    done
    sleep 1
    for target in "${KILL_LIST[@]}"; do
        pkill -9 -f "$target" > /dev/null 2>&1 || true
    done
    ros2 daemon stop    > /dev/null 2>&1 || true
    pkill -9 -f ros2    > /dev/null 2>&1 || true
    pkill -9 -f fastdds > /dev/null 2>&1 || true
    rm -rf ~/.ros/ros2daemon   2>/dev/null || true
    rm -rf /dev/shm/fastrtps_* 2>/dev/null || true
    rm -rf /dev/shm/rtps_*     2>/dev/null || true
    sleep 2
    ros2 daemon start > /dev/null 2>&1 || true
    echo "  [CLEANUP] Done."
}

move_run_files() {
    local run_id="$1"
    local dest_folder="$2"
    mkdir -p "$dest_folder"
    for fname in "vio_path_run_${run_id}.csv" "gt_path_run_${run_id}.csv" "dynamic_objects_run_${run_id}.csv"; do
        local src="${RESULTS_BASE}/${fname}"
        if [[ -f "$src" ]]; then
            mv "$src" "${dest_folder}/${fname}"
            echo "    [MOVE] ${fname} -> $(basename "$dest_folder")/"
        else
            echo "    [WARN] Expected file not found: ${src}"
        fi
    done
}

# ── CORE: RUN ONE SCENARIO ────────────────────────────────────────────────────
run_scenario() {
    local mask_mode="$1"
    local dataset="$2"
    local level="$3"

    local scenario_name="${mask_mode}-${dataset}-${level}"
    local dest_folder="${RESULTS_BASE}/${scenario_name}"
    local bag_path="${VIODE_DATASET}/${dataset}_${level}"

    if [[ ! -d "$bag_path" ]]; then
        echo "  [SKIP] Bag not found: ${bag_path}"
        return 0
    fi

    echo ""
    echo "  ============================================================"
    echo "  SCENARIO : ${scenario_name}  (${RUNS_PER_SCENARIO} runs)"
    echo "  ============================================================"
    mkdir -p "$dest_folder"

    echo "  [PRE-FLIGHT] Clearing stale processes..."
    for target in "${KILL_LIST[@]}"; do
        pkill -9 -f "$target" > /dev/null 2>&1 || true
    done
    sleep 2

    for ((i=1; i<=RUNS_PER_SCENARIO; i++)); do
        echo ""
        echo "    ── Run ${i} / ${RUNS_PER_SCENARIO} ──────────────────"

        # ── 1. OpenVINS ─────────────────────────────────────────────────
        ros2 launch ov_msckf subscribe.launch.py \
            config_path:="$CONFIG_PATH" \
            use_sim_time:=true > /dev/null 2>&1 &
        timeout 15 bash -c \
            'until ros2 node list 2>/dev/null | grep -q "ov_msckf"; do sleep 0.3; done' || true
        sleep 2

        # ── 2. YOLO masker ──────────────────────────────────────────────
        if [[ "$mask_mode" == "masked" ]]; then
            ros2 run yolo_masker yolo_masker \
                --ros-args \
                -p model_path:="${MODEL_PATH}" \
                -p confidence_threshold:="${CONF_THRESHOLD}" \
                -p dilation_kernel:="${DILATION_KERNEL}" \
                -p max_mask_fraction:="${MAX_MASK_FRACTION}" \
                -p use_flow_classifier:="${USE_FLOW_CLASSIFIER}" \
                -p flow_dynamic_threshold:="${FLOW_DYNAMIC_THRESHOLD}" \
                -p flow_min_features:="${FLOW_MIN_FEATURES}" \
                > /dev/null 2>&1 &
        else
            ros2 run yolo_masker yolo_masker \
                --ros-args \
                -p model_path:="${MODEL_PATH}" \
                -p force_empty:=true \
                > /dev/null 2>&1 &
        fi
        # Wait for yolo_masker node to appear, then allow extra time for GPU warm-up.
        # (ros2 topic echo uses volatile durability and misses TRANSIENT_LOCAL messages,
        #  so we poll node list instead.)
        echo "      -> Waiting for yolo_masker to initialize..."
        timeout 15 bash -c \
            'until pgrep -f "yolo_masker" > /dev/null 2>&1; do sleep 0.3; done' \
            || echo "      [WARN] yolo_masker process did not appear — proceeding anyway"
        sleep 3   # GPU warm-up (model is already loaded by pgrep time, 3s is enough)

        # ── 3. GT path converter ─────────────────────────────────────────
        python3 "$OTP_SCRIPT" \
            --ros-args -p target_frame:=gt_frame -p scale_factor:=1.0 \
            > /dev/null 2>&1 &

        # ── 4. Static TF publishers ──────────────────────────────────────
        ros2 run tf2_ros static_transform_publisher \
            --x 0 --y 0 --z 0 \
            --yaw -1.57 --pitch 0 --roll 3.14159 \
            --frame-id global --child-frame-id gt_frame \
            --ros-args -p use_sim_time:=true > /dev/null 2>&1 &

        ros2 run tf2_ros static_transform_publisher \
            --x 0 --y 0 --z 0 \
            --qx 0.5 --qy 0.5 --qz 0.5 --qw 0.5 \
            --frame-id imu --child-frame-id cam0 \
            --ros-args -p use_sim_time:=false > /dev/null 2>&1 &

        ros2 run tf2_ros static_transform_publisher \
            --x 0 --y 0.05 --z 0 \
            --qx 0.5 --qy 0.5 --qz 0.5 --qw 0.5 \
            --frame-id imu --child-frame-id cam1 \
            --ros-args -p use_sim_time:=false > /dev/null 2>&1 &

        # ── 5. Data recorders ────────────────────────────────────────────
        ros2 run ov_softgate path_recorder -- "$i" > /dev/null 2>&1 &
        ros2 run ov_softgate hybrid_speed_estimator "$i" \
            --ros-args \
            -p mask_source:=yolo \
            -p min_disparity:="${MIN_DISPARITY}" \
            -p min_features:="${MIN_FEATURES}" \
            -p orb_nfeatures:="${ORB_NFEATURES}" \
            > /dev/null 2>&1 &
        sleep 0.5

        # ── 6. Play bag ──────────────────────────────────────────────────
        echo "      -> Playing: $(basename "$bag_path")"
        ros2 bag play "$bag_path" --clock
        echo "      -> Bag finished."

        # ── 7. Flush + kill ──────────────────────────────────────────────
        sleep 1
        cleanup_nodes
        sleep 1

        # ── 8. Move CSVs ─────────────────────────────────────────────────
        move_run_files "$i" "$dest_folder"
        sleep 3
        echo "    Run ${i} complete."
    done

    # ── DOV post-processor ────────────────────────────────────────────────
    echo ""
    echo "  -> DOV post-processor: ${scenario_name}..."
    if [[ -f "$DOV_SCRIPT" ]]; then
        (cd "$RESULTS_BASE" && "$VENV_PYTHON" "$DOV_SCRIPT" --folder "$scenario_name") \
            && echo "  -> DOV done." \
            || echo "  [WARN] DOV failed for ${scenario_name}"
    fi
    echo "  [DONE] ${scenario_name}"
}

# ── SOURCE ROS ────────────────────────────────────────────────────────────────
echo "Sourcing ROS 2 environment..."
source "${OPENVINS_WS}/install/setup.bash"

trap cleanup_nodes INT TERM
mkdir -p "$BATCH_OUT"

echo ""
echo "=========================================================="
echo "  VIODE-VIO YOLO Experiment Runner"
echo "  Batch         : batch_${BATCH_NUM} (${BATCH_LABEL})"
echo "  Runs/scenario : ${RUNS_PER_SCENARIO}"
echo "  Output        : ${BATCH_OUT}"
echo "=========================================================="
echo ""
echo "  YOLO params:"
echo "    model             : ${MODEL_PATH}"
echo "    confidence        : ${CONF_THRESHOLD}"
echo "    dilation_kernel   : ${DILATION_KERNEL}"
echo "    max_mask_fraction : ${MAX_MASK_FRACTION}"
echo "    flow_classifier   : ${USE_FLOW_CLASSIFIER}"
echo "    flow_threshold    : ${FLOW_DYNAMIC_THRESHOLD} px"
echo "    flow_min_features : ${FLOW_MIN_FEATURES}"
echo ""
echo "  Estimator params:"
echo "    min_disparity : ${MIN_DISPARITY}"
echo "    min_features  : ${MIN_FEATURES}"
echo "    orb_nfeatures : ${ORB_NFEATURES}"
echo "=========================================================="

# ── Run all scenarios ─────────────────────────────────────────────────────────
for mask_mode in "${MASK_MODES[@]}"; do
    for dataset in "${DATASETS[@]}"; do
        for level in ${DENSITY_LEVELS[$dataset]}; do
            run_scenario "$mask_mode" "$dataset" "$level"
        done
    done
done

# ── Analyze + archive ─────────────────────────────────────────────────────────
echo ""
echo "  [ANALYSIS] Running analyze_results_v5.py..."

{
    echo "========================================================================"
    echo "  BATCH ${BATCH_NUM} — ${BATCH_LABEL}"
    echo "  mask_source       = yolo"
    echo "  model_path        = ${MODEL_PATH}"
    echo "  confidence        = ${CONF_THRESHOLD}"
    echo "  dilation_kernel   = ${DILATION_KERNEL}"
    echo "  max_mask_fraction = ${MAX_MASK_FRACTION}"
    echo "  use_flow_clf      = ${USE_FLOW_CLASSIFIER}"
    echo "  flow_threshold    = ${FLOW_DYNAMIC_THRESHOLD}"
    echo "  flow_min_features = ${FLOW_MIN_FEATURES}"
    echo "  min_disparity     = ${MIN_DISPARITY}"
    echo "  min_features      = ${MIN_FEATURES}"
    echo "  orb_nfeatures     = ${ORB_NFEATURES}"
    echo "  runs_per_scenario = ${RUNS_PER_SCENARIO}"
    echo "========================================================================"
    echo ""
} > "$ANALYSIS_LOG"

for env in "parking_lot" "city_day" "city_night"; do
    echo "    -> Analyzing: ${env}"
    (cd "$RESULTS_BASE" && python3 "$ANALYZE_SCRIPT" --env "$env") \
        >> "$ANALYSIS_LOG" 2>&1 \
        || echo "    [WARN] analyze_results failed for ${env}"
done

# ── params.txt ────────────────────────────────────────────────────────────────
cat > "$PARAMS_FILE" << EOF
batch_number      = ${BATCH_NUM}
label             = ${BATCH_LABEL}
mask_source       = yolo
model_path        = ${MODEL_PATH}
confidence        = ${CONF_THRESHOLD}
dilation_kernel   = ${DILATION_KERNEL}
max_mask_fraction = ${MAX_MASK_FRACTION}
use_flow_clf      = ${USE_FLOW_CLASSIFIER}
flow_threshold    = ${FLOW_DYNAMIC_THRESHOLD}
flow_min_features = ${FLOW_MIN_FEATURES}
min_disparity     = ${MIN_DISPARITY}
min_features      = ${MIN_FEATURES}
orb_nfeatures     = ${ORB_NFEATURES}
runs_per_scenario = ${RUNS_PER_SCENARIO}
EOF

# ── Archive scenario folders ──────────────────────────────────────────────────
echo "  [ARCHIVE] Archiving to batch_${BATCH_NUM}/..."
ARCHIVED=0
for mask_mode in "${MASK_MODES[@]}"; do
    for dataset in "${DATASETS[@]}"; do
        for level in ${DENSITY_LEVELS[$dataset]}; do
            src="${RESULTS_BASE}/${mask_mode}-${dataset}-${level}"
            if [[ -d "$src" ]]; then
                cp -r "$src" "${BATCH_OUT}/${mask_mode}-${dataset}-${level}"
                ARCHIVED=$((ARCHIVED + 1))
            else
                echo "  [WARN] Missing: ${mask_mode}-${dataset}-${level}"
            fi
        done
    done
done
echo "  [ARCHIVE] ${ARCHIVED}/24 folders archived."

for env in "parking_lot" "city_day" "city_night"; do
    plot="${RESULTS_BASE}/Trajectory_Analysis_${env}.png"
    [[ -f "$plot" ]] && cp "$plot" "${BATCH_OUT}/"
done

# ── Clean up sim_results/ ─────────────────────────────────────────────────────
echo "  [CLEANUP] Clearing scenario folders from sim_results/..."
for mask_mode in "${MASK_MODES[@]}"; do
    for dataset in "${DATASETS[@]}"; do
        for level in ${DENSITY_LEVELS[$dataset]}; do
            rm -rf "${RESULTS_BASE}/${mask_mode}-${dataset}-${level}"
        done
    done
done

echo ""
echo "=========================================================="
echo "  YOLO batch complete."
echo "  Results : ${BATCH_OUT}"
echo "  Analysis: ${ANALYSIS_LOG}"
echo ""
echo "  Compare against GT baseline (ablation_batches_v1/batch_10/):"
echo "  python3 ${RESULTS_BASE}/analyze_ablation_batches.py"
echo "=========================================================="
