#!/bin/bash
# =============================================================================
# run_all_experiments.sh
# VIODE-VIO Full Experiment Runner — OpenVINS + DOV Post-processor
#
# Usage:
#   ./run_all_experiments.sh [DATASET] [MODE] [RUNS] [MASK_SOURCE]
#
#   DATASET     : parking_lot | city_day | city_night | all
#   MODE        : masked | unmasked | both
#   RUNS        : repetitions per scenario (default: 10)
#   MASK_SOURCE : gt | yolo  (default: gt)
#                 gt   — VIODE ground-truth segmentation (semantic_masker)
#                 yolo — YOLOv11n real-time detection (yolo_masker)
#
# Examples:
#   ./run_all_experiments.sh parking_lot both 10
#   ./run_all_experiments.sh city_day masked 5
#   ./run_all_experiments.sh all both 10
#   ./run_all_experiments.sh all both 20 yolo    # Phase 0 Gate 1 runs
#
# Pipeline per run:
#   1. Launch OpenVINS (with or without semantic masker, per MODE)
#   2. Launch GT converter, TF publishers, path_recorder,
#      hybrid_speed_estimator
#   3. Play the ROS2 bag (blocks until finished)
#   4. Flush recorders, stop all nodes, move CSVs to scenario folder
#   After all runs in a scenario: run DOV post-processor
#
# Supported VIODE scenarios (none/low/mid/high for each environment):
#   parking_lot_*  |  city_day_*  |  city_night_*
#
# NOTE — Recompile between masked and unmasked:
#   When MODE=both, the script pauses between the two batches and
#   prompts you to toggle use_dynamic_mask in estimator_config.yaml
#   and rebuild OpenVINS before continuing.
# =============================================================================

# ==========================================
# ── EDIT THESE FOR YOUR MACHINE ───────────
# ==========================================

WORKSPACE="/path/to/your/workspace"   # ← set this to your data root
RESULTS_BASE="${WORKSPACE}/sim_results"
OPENVINS_WS="${WORKSPACE}/openvins_ws"

# Root directory containing all VIODE bag folders
VIODE_DATASET="${WORKSPACE}/Downloads_Ext/VIODE_Dataset"

# odom_to_path.py lives inside the VIODE_Dataset folder
OTP_SCRIPT="${VIODE_DATASET}/odom_to_path.py"

CONFIG_PATH="${OPENVINS_WS}/install/ov_msckf/share/ov_msckf/config/viode_config/estimator_config.yaml"

DOV_SCRIPT="${RESULTS_BASE}/dov_postprocessor.py"

# ==========================================
# ── ARGUMENT PARSING ──────────────────────
# ==========================================
DATASET_ARG="${1:-parking_lot}"
MODE_ARG="${2:-both}"
TOTAL_RUNS="${3:-10}"
MASK_SOURCE="${4:-gt}"   # 'gt' uses semantic_masker; 'yolo' uses yolo_masker

case "$MASK_SOURCE" in
    gt|yolo) ;;
    *)
        echo "[ERROR] Invalid MASK_SOURCE: '${MASK_SOURCE}'. Use: gt | yolo"
        exit 1
        ;;
esac

case "$MODE_ARG" in
    masked|unmasked|both) ;;
    *)
        echo "[ERROR] Invalid MODE: '${MODE_ARG}'. Use: masked | unmasked | both"
        exit 1
        ;;
esac

# Density levels available for each environment
declare -A DENSITY_LEVELS
DENSITY_LEVELS["parking_lot"]="none low mid high"
DENSITY_LEVELS["city_day"]="none low mid high"
DENSITY_LEVELS["city_night"]="none low mid high"

case "$DATASET_ARG" in
    parking_lot|city_day|city_night)
        DATASETS=("$DATASET_ARG")
        ;;
    all)
        DATASETS=("parking_lot" "city_day" "city_night")
        ;;
    *)
        echo "[ERROR] Invalid DATASET: '${DATASET_ARG}'."
        echo "        Use: parking_lot | city_day | city_night | all"
        exit 1
        ;;
esac

# Unmasked runs first so the recompile pause falls between the two batches
case "$MODE_ARG" in
    masked)   MASK_MODES=("masked")            ;;
    unmasked) MASK_MODES=("unmasked")          ;;
    both)     MASK_MODES=("unmasked" "masked") ;;
esac

# ==========================================
# ── NODE KILL LIST ────────────────────────
# ==========================================
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

# ==========================================
# ── HELPER: CLEANUP ───────────────────────
# ==========================================
cleanup_nodes() {
    echo "  [CLEANUP] Stopping all nodes..."

    # Signal path_recorder to flush before killing (5 s timeout — node may already be dead)
    timeout 5 ros2 topic pub --once /recorder/save std_msgs/msg/Bool "{data: true}" \
        > /dev/null 2>&1 || true
    sleep 2

    # Graceful SIGTERM pass
    for target in "${KILL_LIST[@]}"; do
        pkill -f "$target" > /dev/null 2>&1 || true
    done
    sleep 1

    # Force SIGKILL pass
    for target in "${KILL_LIST[@]}"; do
        pkill -9 -f "$target" > /dev/null 2>&1 || true
    done

    # Reset ROS2 DDS middleware — prevents stale discovery state between runs
    ros2 daemon stop    > /dev/null 2>&1 || true
    pkill -9 -f ros2    > /dev/null 2>&1 || true
    pkill -9 -f fastdds > /dev/null 2>&1 || true
    rm -rf ~/.ros/ros2daemon   2>/dev/null || true
    rm -rf /dev/shm/fastrtps_* 2>/dev/null || true
    rm -rf /dev/shm/rtps_*     2>/dev/null || true
    sleep 2
    ros2 daemon start > /dev/null 2>&1 || true
    timeout 15 bash -c 'until ros2 node list > /dev/null 2>&1; do sleep 0.5; done' || true
    echo "  [CLEANUP] Done."
}

# ==========================================
# ── HELPER: MOVE CSV FILES ────────────────
# ==========================================
# path_recorder          -> vio_path_run_N.csv, gt_path_run_N.csv
# hybrid_speed_estimator -> dynamic_objects_run_N.csv
move_run_files() {
    local run_id="$1"
    local dest_folder="$2"

    mkdir -p "$dest_folder"

    local patterns=(
        "vio_path_run_${run_id}.csv"
        "gt_path_run_${run_id}.csv"
        "dynamic_objects_run_${run_id}.csv"
    )

    for fname in "${patterns[@]}"; do
        local src="${RESULTS_BASE}/${fname}"
        if [[ -f "$src" ]]; then
            mv "$src" "${dest_folder}/${fname}"
            echo "    [MOVE] ${fname} -> $(basename "$dest_folder")/"
        else
            echo "    [WARN] Expected file not found: ${src}"
        fi
    done
}

# ==========================================
# ── CORE: RUN ONE FULL SCENARIO ───────────
# ==========================================
run_scenario() {
    local mask_mode="$1"   # "masked" or "unmasked"
    local dataset="$2"     # e.g. "parking_lot"
    local level="$3"       # e.g. "high"

    local scenario_name="${mask_mode}-${dataset}-${level}"
    local dest_folder="${RESULTS_BASE}/${scenario_name}"
    local bag_path="${VIODE_DATASET}/${dataset}_${level}"

    if [[ ! -d "$bag_path" ]]; then
        echo "  [SKIP] Bag directory not found: ${bag_path}"
        return 0
    fi

    echo ""
    echo "============================================================"
    echo "  SCENARIO : ${scenario_name}"
    echo "  Bag      : ${bag_path}"
    echo "  Output   : ${dest_folder}"
    echo "  Runs     : ${TOTAL_RUNS}  |  Mode: ${mask_mode}"
    echo "============================================================"

    mkdir -p "$dest_folder"

    # Clear any stale processes before starting
    echo "  [PRE-FLIGHT] Clearing stale processes..."
    for target in "${KILL_LIST[@]}"; do
        pkill -9 -f "$target" > /dev/null 2>&1 || true
    done
    sleep 2

    for ((i=1; i<=TOTAL_RUNS; i++)); do
        echo ""
        echo "  ── Run ${i} / ${TOTAL_RUNS} ──────────────────────────"

        # ── 1. OpenVINS ───────────────────────────────────────────────
        ros2 launch ov_msckf subscribe.launch.py \
            config_path:="$CONFIG_PATH" \
            use_sim_time:=true > /dev/null 2>&1 &
        sleep 1

        # ── 2. Masker (always running for fair 4-topic sync comparison) ──
        # force_empty:=true publishes all-zeros masks for unmasked experiments
        if [[ "$MASK_SOURCE" == "yolo" ]]; then
            if [[ "$mask_mode" == "masked" ]]; then
                ros2 run yolo_masker yolo_masker \
                    --ros-args -p model_path:="${OPENVINS_WS}/models/yolo11n-seg.pt" > /dev/null 2>&1 &
            else
                ros2 run yolo_masker yolo_masker \
                    --ros-args -p model_path:="${OPENVINS_WS}/models/yolo11n-seg.pt" -p force_empty:=true > /dev/null 2>&1 &
            fi
        else
            if [[ "$mask_mode" == "masked" ]]; then
                ros2 run ov_softgate masker > /dev/null 2>&1 &
            else
                ros2 run ov_softgate masker \
                    --ros-args -p force_empty:=true > /dev/null 2>&1 &
            fi
        fi
        # Wait for the masker to advertise /cam0/masked (YOLO load can take 5-10s).
        timeout 60 bash -c \
            'until ros2 topic list 2>/dev/null | grep -q "/cam0/masked"; do sleep 0.5; done' \
            || echo "      [WARN] /cam0/masked not found — proceeding anyway"
        sleep 5  # DDS warm-up: let masker→OpenVINS 4-topic sync establish

        # ── 3. Ground-truth path converter ───────────────────────────
        python3 "$OTP_SCRIPT" \
            --ros-args -p target_frame:=gt_frame -p scale_factor:=1.0 \
            > /dev/null 2>&1 &

        # ── 4. Static TF publishers ───────────────────────────────────
        # Global -> GT frame (VIODE GT coordinate alignment)
        ros2 run tf2_ros static_transform_publisher \
            --x 0 --y 0 --z 0 \
            --yaw -1.57 --pitch 0 --roll 3.14159 \
            --frame-id global --child-frame-id gt_frame \
            --ros-args -p use_sim_time:=true > /dev/null 2>&1 &

        # IMU -> cam0 (camera extrinsic: qx=0.5 qy=0.5 qz=0.5 qw=0.5)
        ros2 run tf2_ros static_transform_publisher \
            --x 0 --y 0 --z 0 \
            --qx 0.5 --qy 0.5 --qz 0.5 --qw 0.5 \
            --frame-id imu --child-frame-id cam0 \
            --ros-args -p use_sim_time:=false > /dev/null 2>&1 &

        # IMU -> cam1 (stereo baseline: 5 cm in Y)
        ros2 run tf2_ros static_transform_publisher \
            --x 0 --y 0.05 --z 0 \
            --qx 0.5 --qy 0.5 --qz 0.5 --qw 0.5 \
            --frame-id imu --child-frame-id cam1 \
            --ros-args -p use_sim_time:=false > /dev/null 2>&1 &

        # ── 5. Data recorders and object estimator ────────────────────
        ros2 run ov_softgate path_recorder -- "$i" \
            --ros-args -p output_dir:="${RESULTS_BASE}" > /dev/null 2>&1 &
        ros2 run ov_softgate hybrid_speed_estimator "$i" \
            --ros-args \
            -p mask_source:="$MASK_SOURCE" \
            -p calib_file:="${OPENVINS_WS}/src/open_vins/config/viode_config/kalibr_imucam_chain.yaml" \
            -p output_dir:="${RESULTS_BASE}" > /dev/null 2>&1 &
        timeout 15 bash -c \
            'until ros2 node list 2>/dev/null | grep -q "path_recorder"; do sleep 0.3; done' || true
        sleep 1

        # ── 6. Play bag (blocks until the bag finishes) ───────────────
        echo "    -> Playing bag: $(basename "$bag_path")"
        ros2 bag play "$bag_path" --clock --read-ahead-queue-size 10000
        echo "    -> Bag playback finished."

        # ── 7. Flush recorders, kill all nodes ────────────────────────
        echo "    -> Flushing recorders and stopping nodes..."
        sleep 1
        cleanup_nodes
        sleep 1

        # ── 8. Move output CSVs into the scenario folder ──────────────
        echo "    -> Moving output files..."
        move_run_files "$i" "$dest_folder"

        sleep 3
        echo "    Run ${i} complete."
    done

    # ── DOV post-processor on the completed scenario ──────────────────
    echo ""
    echo "  -> Running DOV post-processor for ${scenario_name}..."
    if [[ -f "$DOV_SCRIPT" ]]; then
        (cd "$RESULTS_BASE" && python3 "$DOV_SCRIPT" --folder "$scenario_name") \
            && echo "  -> DOV-corrected paths generated." \
            || echo "  [WARN] DOV post-processor failed for ${scenario_name}. Run manually."
    else
        echo "  [WARN] dov_postprocessor.py not found at: ${DOV_SCRIPT}"
        echo "         Run it manually after all experiments finish."
    fi

    echo "  [DONE] Scenario ${scenario_name} complete."
}

# ==========================================
# ── SOURCE ROS ENVIRONMENT ────────────────
# ==========================================
echo "Sourcing ROS 2 environment..."
if [[ -f "${OPENVINS_WS}/install/setup.bash" ]]; then
    source "${OPENVINS_WS}/install/setup.bash"
    echo "  -> ${OPENVINS_WS}/install/setup.bash"
else
    echo "[ERROR] setup.bash not found. Check OPENVINS_WS at the top of this script."
    exit 1
fi

# Trap Ctrl+C and SIGTERM — intentionally NOT EXIT
# (an EXIT trap fires even on successful completion)
trap cleanup_nodes INT TERM

# ==========================================
# ── MAIN LOOP ─────────────────────────────
# ==========================================
echo ""
echo "=========================================================="
echo "  VIODE-VIO Experiment Runner"
echo "  Dataset(s)    : ${DATASET_ARG}"
echo "  Mode          : ${MODE_ARG}"
echo "  Runs/scenario : ${TOTAL_RUNS}"
echo "  Mask source   : ${MASK_SOURCE}"
echo "  Results dir   : ${RESULTS_BASE}"
echo "=========================================================="

first_mode=true
for mask_mode in "${MASK_MODES[@]}"; do

    # No recompile needed — use_dynamic_mask: true always, masker uses force_empty for unmasked mode
    if [[ "$MODE_ARG" == "both" && "$first_mode" == "false" ]]; then
        echo ""
        echo "  [INFO] Starting masked runs (no recompile needed)."
    fi

    first_mode=false

    for dataset in "${DATASETS[@]}"; do
        levels="${DENSITY_LEVELS[$dataset]}"
        for level in $levels; do
            run_scenario "$mask_mode" "$dataset" "$level"
        done
    done
done

# ==========================================
# ── FINAL SUMMARY ─────────────────────────
# ==========================================
echo ""
echo "=========================================================="
echo "  All experiments complete."
echo ""
echo "  Scenario folders created:"
for mask_mode in "${MASK_MODES[@]}"; do
    for dataset in "${DATASETS[@]}"; do
        for level in ${DENSITY_LEVELS[$dataset]}; do
            echo "    ${RESULTS_BASE}/${mask_mode}-${dataset}-${level}/"
        done
    done
done
echo ""
echo "  Each folder contains:"
echo "    vio_path_run_N.csv        — raw OpenVINS trajectory"
echo "    gt_path_run_N.csv         — ground truth trajectory"
echo "    dynamic_objects_run_N.csv — detected object centroids"
echo "    dov_path_run_N.csv        — DOV-corrected trajectory"
echo ""
echo "  Next step: run analyze_results_v4.py to generate plots."
echo "=========================================================="