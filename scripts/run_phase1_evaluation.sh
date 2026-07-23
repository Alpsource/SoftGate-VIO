#!/bin/bash
# =============================================================================
# run_gate2_comparison.sh
# VIODE-VIO Phase 1 Gate 2 — Three-condition fair comparison
#
# Usage:
#   ./run_gate2_comparison.sh [RUNS_PER_SCENARIO]
#
# Conditions (all in the same batch, same stochastic draws):
#   unmasked  — force_empty VIO mask, use_imu_residual: false  → U-VIO baseline
#   yolo      — YOLO VIO mask,        use_imu_residual: false  → M-VIO-YOLO
#   yolo_imu  — YOLO VIO mask,        use_imu_residual: true   → M+IMU-VIO (Option C)
#
# Running all three conditions in the same script eliminates the cross-batch
# VIO stochasticity confound: identical bag replays guarantee fair comparison.
#
# Gate 2 criterion (per IMPLEMENTATION_PLAN.md §4.4):
#   M+IMU-VIO improves over M-VIO-YOLO by ≥5% ATE on ≥8/12 cells.
#
# Saves to ablation_batches/batch_N/ (auto-numbered).
# =============================================================================

WORKSPACE="/path/to/your/workspace"   # ← set this to your data root
RESULTS_BASE="${WORKSPACE}/sim_results"
OPENVINS_WS="${WORKSPACE}/openvins_ws"
VIODE_DATASET="${WORKSPACE}/Downloads_Ext/VIODE_Dataset"
OTP_SCRIPT="${VIODE_DATASET}/odom_to_path.py"
CONFIG_BASE="${OPENVINS_WS}/install/ov_msckf/share/ov_msckf/config/viode_config"
CONFIG_PLAIN="${CONFIG_BASE}/estimator_config.yaml"
CONFIG_IMU="${CONFIG_BASE}/estimator_config_imu_residual.yaml"
DOV_SCRIPT="${RESULTS_BASE}/dov_postprocessor.py"
ANALYZE_SCRIPT="${RESULTS_BASE}/analyze_results_v5.py"
VENV_PYTHON="${RESULTS_BASE}/env/bin/python3"
BATCH_ROOT="${RESULTS_BASE}/ablation_batches"
MODEL_PATH="${OPENVINS_WS}/models/yolo26s-seg.pt"

RUNS_PER_SCENARIO="${1:-10}"

# ── YOLO parameters (identical to run_yolo_experiments.sh) ───────────────────
DILATION_KERNEL=9
MAX_MASK_FRACTION=0.80
MIN_DISPARITY=1.2
MIN_FEATURES=8
ORB_NFEATURES=100
CONF_THRESHOLD=0.25
FLOW_DYNAMIC_THRESHOLD=2.0
FLOW_MIN_FEATURES=5
USE_FLOW_CLASSIFIER=true
# ── IMU residual parameters (Option C) ───────────────────────────────────────
IMU_RESIDUAL_ALPHA=3.0
IMU_RESIDUAL_SIGMA_PX=5.0
# ─────────────────────────────────────────────────────────────────────────────

declare -A DENSITY_LEVELS
DENSITY_LEVELS["parking_lot"]="none low mid high"
DENSITY_LEVELS["city_day"]="none low mid high"
DENSITY_LEVELS["city_night"]="none low mid high"

DATASETS=("parking_lot" "city_day" "city_night")

# Three conditions in one batch:
#   unmasked  → U-VIO  (plain config, no YOLO mask, no IMU residual)
#   yolo      → M-VIO-YOLO (plain config, YOLO mask active)
#   yolo_imu  → M+IMU-VIO  (IMU residual config, YOLO mask active)
MASK_MODES=("unmasked" "yolo" "yolo_imu")

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

# ── Auto-number ───────────────────────────────────────────────────────────────
BATCH_NUM=1
while [[ -d "${BATCH_ROOT}/batch_${BATCH_NUM}" ]]; do
    BATCH_NUM=$((BATCH_NUM + 1))
done
BATCH_OUT="${BATCH_ROOT}/batch_${BATCH_NUM}"
ANALYSIS_LOG="${BATCH_OUT}/analysis.log"
PARAMS_FILE="${BATCH_OUT}/params.txt"

BATCH_LABEL="gate2_optionC_alpha${IMU_RESIDUAL_ALPHA}_sigma${IMU_RESIDUAL_SIGMA_PX}"

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

        # ── Select config based on condition ────────────────────────────────
        local config_path
        if [[ "$mask_mode" == "yolo_imu" ]]; then
            config_path="$CONFIG_IMU"    # use_imu_residual: true
        else
            config_path="$CONFIG_PLAIN"  # use_imu_residual: false
        fi

        # ── 1. OpenVINS ─────────────────────────────────────────────────────
        ros2 launch ov_msckf subscribe.launch.py \
            config_path:="$config_path" \
            use_sim_time:=true > /dev/null 2>&1 &
        timeout 15 bash -c \
            'until ros2 node list 2>/dev/null | grep -q "ov_msckf"; do sleep 0.3; done' || true
        sleep 2

        # ── 2. YOLO masker ──────────────────────────────────────────────────
        if [[ "$mask_mode" == "yolo" || "$mask_mode" == "yolo_imu" ]]; then
            # Active YOLO masking
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
            # unmasked: force_empty keeps 4-topic sync but publishes zero mask
            ros2 run yolo_masker yolo_masker \
                --ros-args \
                -p model_path:="${MODEL_PATH}" \
                -p force_empty:=true \
                > /dev/null 2>&1 &
        fi
        echo "      -> Waiting for yolo_masker..."
        timeout 15 bash -c \
            'until pgrep -f "yolo_masker" > /dev/null 2>&1; do sleep 0.3; done' \
            || echo "      [WARN] yolo_masker did not appear — proceeding anyway"
        sleep 3

        # ── 3. GT path converter ─────────────────────────────────────────────
        python3 "$OTP_SCRIPT" \
            --ros-args -p target_frame:=gt_frame -p scale_factor:=1.0 \
            > /dev/null 2>&1 &

        # ── 4. Static TF publishers ──────────────────────────────────────────
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

        # ── 5. Data recorders ─────────────────────────────────────────────────
        ros2 run ov_softgate path_recorder -- "$i" > /dev/null 2>&1 &
        ros2 run ov_softgate hybrid_speed_estimator "$i" \
            --ros-args \
            -p mask_source:=yolo \
            -p min_disparity:="${MIN_DISPARITY}" \
            -p min_features:="${MIN_FEATURES}" \
            -p orb_nfeatures:="${ORB_NFEATURES}" \
            > /dev/null 2>&1 &
        sleep 0.5

        # ── 6. Play bag ──────────────────────────────────────────────────────
        echo "      -> Playing: $(basename "$bag_path")"
        ros2 bag play "$bag_path" --clock
        echo "      -> Bag finished."

        # ── 7. Flush + kill ──────────────────────────────────────────────────
        sleep 1
        cleanup_nodes
        sleep 1

        # ── 8. Move CSVs ──────────────────────────────────────────────────────
        move_run_files "$i" "$dest_folder"
        sleep 3
        echo "    Run ${i} complete."
    done

    # ── DOV post-processor ────────────────────────────────────────────────────
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
echo "  VIODE-VIO Phase 1 Gate 2 — Three-Condition Comparison"
echo "  Batch         : batch_${BATCH_NUM} (${BATCH_LABEL})"
echo "  Runs/scenario : ${RUNS_PER_SCENARIO}"
echo "  Output        : ${BATCH_OUT}"
echo "=========================================================="
echo ""
echo "  Conditions:"
echo "    unmasked  : force_empty + use_imu_residual=false  → U-VIO baseline"
echo "    yolo      : YOLO mask  + use_imu_residual=false   → M-VIO-YOLO"
echo "    yolo_imu  : YOLO mask  + use_imu_residual=true    → M+IMU-VIO (Option C)"
echo ""
echo "  IMU residual params (Option C - triangulated depth):"
echo "    alpha    : ${IMU_RESIDUAL_ALPHA}"
echo "    sigma_px : ${IMU_RESIDUAL_SIGMA_PX}  (dead-zone = 3 * up_msckf_sigma_px)"
echo "=========================================================="

# ── Run all scenarios ─────────────────────────────────────────────────────────
for mask_mode in "${MASK_MODES[@]}"; do
    for dataset in "${DATASETS[@]}"; do
        for level in ${DENSITY_LEVELS[$dataset]}; do
            run_scenario "$mask_mode" "$dataset" "$level"
        done
    done
done

# ── Analyze ───────────────────────────────────────────────────────────────────
echo ""
echo "  [ANALYSIS] Running analyze_results_v5.py..."

{
    echo "========================================================================"
    echo "  BATCH ${BATCH_NUM} — ${BATCH_LABEL}"
    echo "  Conditions: unmasked (U-VIO) | yolo (M-VIO-YOLO) | yolo_imu (M+IMU-VIO)"
    echo "  use_imu_residual (yolo_imu)  = true (Option C: triangulated depth)"
    echo "  imu_residual_alpha           = ${IMU_RESIDUAL_ALPHA}"
    echo "  imu_residual_sigma_px        = ${IMU_RESIDUAL_SIGMA_PX}"
    echo "  dead_zone                    = 3 * up_msckf_sigma_px = 4.5 px"
    echo "  yolo conf                    = ${CONF_THRESHOLD}"
    echo "  dilation_kernel              = ${DILATION_KERNEL}"
    echo "  runs_per_scenario            = ${RUNS_PER_SCENARIO}"
    echo "========================================================================"
    echo ""
} > "$ANALYSIS_LOG"

for env in "parking_lot" "city_day" "city_night"; do
    echo "    -> Analyzing: ${env}"
    (cd "$RESULTS_BASE" && python3 "$ANALYZE_SCRIPT" --env "$env") \
        >> "$ANALYSIS_LOG" 2>&1 \
        || echo "    [WARN] analyze_results failed for ${env}"
done

echo "    -> Gate 2 criterion analysis..."
GATE2_SCRIPT="${RESULTS_BASE}/analyze_gate2.py"
if [[ -f "$GATE2_SCRIPT" ]]; then
    python3 "$GATE2_SCRIPT" --batch "$BATCH_OUT" \
        | tee -a "$ANALYSIS_LOG"
else
    echo "    [WARN] analyze_gate2.py not found at ${GATE2_SCRIPT}"
fi

# ── params.txt ────────────────────────────────────────────────────────────────
cat > "$PARAMS_FILE" << EOF
batch_number          = ${BATCH_NUM}
label                 = ${BATCH_LABEL}
conditions            = unmasked | yolo | yolo_imu
use_imu_residual      = true (yolo_imu only)
imu_residual_method   = Option C (triangulated p_FinG, not stereo disparity)
imu_residual_alpha    = ${IMU_RESIDUAL_ALPHA}
imu_residual_sigma_px = ${IMU_RESIDUAL_SIGMA_PX}
dead_zone_px          = 3 * up_msckf_sigma_px (= 4.5 px for sigma_pix=1.5)
yolo_conf             = ${CONF_THRESHOLD}
dilation_kernel       = ${DILATION_KERNEL}
max_mask_fraction     = ${MAX_MASK_FRACTION}
use_flow_clf          = ${USE_FLOW_CLASSIFIER}
flow_threshold        = ${FLOW_DYNAMIC_THRESHOLD}
runs_per_scenario     = ${RUNS_PER_SCENARIO}
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
echo "  [ARCHIVE] ${ARCHIVED}/36 folders archived."

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
echo "  Gate 2 batch complete."
echo "  Results : ${BATCH_OUT}"
echo "  Analysis: ${ANALYSIS_LOG}"
echo ""
echo "  Gate 2 criterion:"
echo "    yolo_imu (M+IMU-VIO) improves over yolo (M-VIO-YOLO) by >=5% on >=8/12 cells"
echo "  All three conditions in same batch → no cross-batch stochasticity confound."
echo "=========================================================="
