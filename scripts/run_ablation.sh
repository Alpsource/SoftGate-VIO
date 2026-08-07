#!/bin/bash
# =============================================================================
# run_all_experiments_v3.sh
# VIODE-VIO Ablation Study Runner — Masker + Estimator Parameter Sweep
#
# Usage:
#   ./run_all_experiments_v3.sh [RUNS_PER_SCENARIO]
#
#   RUNS_PER_SCENARIO : repetitions per scenario (default: 5)
#
# Example:
#   ./run_all_experiments_v3.sh 5
#
# For each batch defined in BATCH_DEFS:
#   1. Runs all 3 envs × 4 densities × masked + unmasked × RUNS experiments
#      with the batch's masker/estimator parameters injected via --ros-args
#   2. Runs DOV post-processor on every completed scenario folder
#   3. Runs analyze_results_v5.py for all 3 environments
#   4. Archives all 24 scenario folders + params.txt + analysis.log
#      to: sim_results/ablation_batches/batch_N/
#   5. Deletes the scenario folders from sim_results/ before next batch
#
# Tunable parameters injected per batch:
#   Masker          : dilation_kernel  max_mask_fraction
#   HybridEstimator : min_disparity    min_features    orb_nfeatures
# =============================================================================

# ==========================================
# ── PATHS ─────────────────────────────────
# ==========================================
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

RUNS_PER_SCENARIO="${1:-5}"

# ==========================================
# ── BATCH DEFINITIONS ─────────────────────
# ==========================================
# Format (pipe-separated):
#   "label | dilation_kernel | max_mask_fraction | min_disparity | min_features | orb_nfeatures"
#
# Defaults (batch_1 BASELINE):
#   dilation_kernel=13  max_mask_fraction=0.80
#   min_disparity=1.2   min_features=8   orb_nfeatures=200
#
# Single-parameter sweeps: one parameter changes, all others at default.

BATCH_DEFS=(
    # "BASELINE        | 13 | 0.80 | 1.2 |  8 | 200"   # batch_1
    # "dilation=7      |  7 | 0.80 | 1.2 |  8 | 200"   # batch_2
    # "dilation=21     | 21 | 0.80 | 1.2 |  8 | 200"   # batch_3
    # "mask_frac=0.30  | 13 | 0.30 | 1.2 |  8 | 200"   # batch_4
    # "mask_frac=0.50  | 13 | 0.50 | 1.2 |  8 | 200"   # batch_5
    # "min_disp=0.8    | 13 | 0.80 | 0.8 |  8 | 200"   # batch_6
    # "min_disp=2.0    | 13 | 0.80 | 2.0 |  8 | 200"   # batch_7
    # "min_feat=4      | 13 | 0.80 | 1.2 |  4 | 200"   # batch_8
    # "min_feat=12     | 13 | 0.80 | 1.2 | 12 | 200"   # batch_9
    # "orb_nfeat=100   | 13 | 0.80 | 1.2 |  8 | 100"   # batch_10
    # "orb_nfeat=400   | 13 | 0.80 | 1.2 |  8 | 400"   # batch_11
    "min8_orb100    | 13 | 0.80 | 1.2 | 8 | 100"  # combined candidate
)

# ==========================================
# ── ENVIRONMENT / DENSITY MATRIX ──────────
# ==========================================
declare -A DENSITY_LEVELS
DENSITY_LEVELS["parking_lot"]="none low mid high"
DENSITY_LEVELS["city_day"]="none low mid high"
DENSITY_LEVELS["city_night"]="none low mid high"

DATASETS=("parking_lot" "city_day" "city_night")
MASK_MODES=("unmasked" "masked")

# ==========================================
# ── NODE KILL LIST ────────────────────────
# ==========================================
KILL_LIST=(
    "ov_msckf"
    "ov_softgate"
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
    ros2 topic pub --once /recorder/save std_msgs/msg/Bool "{data: true}" \
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
    timeout 15 bash -c 'until ros2 node list > /dev/null 2>&1; do sleep 0.5; done' || true
    echo "  [CLEANUP] Done."
}

# ==========================================
# ── HELPER: MOVE CSV FILES ────────────────
# ==========================================
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
# ── CORE: RUN ONE SCENARIO ────────────────
# ==========================================
run_scenario() {
    local mask_mode="$1"
    local dataset="$2"
    local level="$3"
    local dilation="$4"
    local max_mask="$5"
    local min_disp="$6"
    local min_feat="$7"
    local orb_nfeat="$8"

    local scenario_name="${mask_mode}-${dataset}-${level}"
    local dest_folder="${RESULTS_BASE}/${scenario_name}"
    local bag_path="${VIODE_DATASET}/${dataset}_${level}"

    if [[ ! -d "$bag_path" ]]; then
        echo "  [SKIP] Bag directory not found: ${bag_path}"
        return 0
    fi

    echo ""
    echo "  ============================================================"
    echo "  SCENARIO : ${scenario_name}"
    echo "  Params   : dilation=${dilation} max_mask=${max_mask} min_disp=${min_disp} min_feat=${min_feat} orb=${orb_nfeat}"
    echo "  Runs     : ${RUNS_PER_SCENARIO}"
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

        # ── 1. OpenVINS ─────────────────────────────────────────────
        ros2 launch ov_msckf subscribe.launch.py \
            config_path:="$CONFIG_PATH" \
            use_sim_time:=true > /dev/null 2>&1 &
        # Wait until the node is actually registered, then add 2s settling buffer
        timeout 15 bash -c \
            'until ros2 node list 2>/dev/null | grep -q "ov_msckf"; do sleep 0.3; done' || true
        sleep 2

        # ── 2. Semantic masker (always running; force_empty for unmasked) ──
        if [[ "$mask_mode" == "masked" ]]; then
            ros2 run ov_softgate masker \
                --ros-args \
                -p dilation_kernel:="${dilation}" \
                -p max_mask_fraction:="${max_mask}" > /dev/null 2>&1 &
        else
            ros2 run ov_softgate masker \
                --ros-args \
                -p force_empty:=true \
                -p dilation_kernel:="${dilation}" \
                -p max_mask_fraction:="${max_mask}" > /dev/null 2>&1 &
        fi
        sleep 5  # DDS warm-up: let masker→OpenVINS 4-topic sync establish

        # ── 3. Ground-truth path converter ──────────────────────────
        python3 "$OTP_SCRIPT" \
            --ros-args -p target_frame:=gt_frame -p scale_factor:=1.0 \
            > /dev/null 2>&1 &

        # ── 4. Static TF publishers ──────────────────────────────────
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

        # ── 5. Path recorder + object estimator ─────────────────────
        ros2 run ov_softgate path_recorder -- "$i" \
            --ros-args -p output_dir:="${RESULTS_BASE}" > /dev/null 2>&1 &

        ros2 run ov_softgate hybrid_speed_estimator "$i" \
            --ros-args \
            -p min_disparity:="${min_disp}" \
            -p min_features:="${min_feat}" \
            -p orb_nfeatures:="${orb_nfeat}" \
            -p calib_file:="${OPENVINS_WS}/src/open_vins/config/viode_config/kalibr_imucam_chain.yaml" \
            -p output_dir:="${RESULTS_BASE}" > /dev/null 2>&1 &
        sleep 0.5

        # ── 6. Play bag ──────────────────────────────────────────────
        echo "      -> Playing: $(basename "$bag_path")"
        ros2 bag play "$bag_path" --clock --read-ahead-queue-size 10000
        echo "      -> Bag finished."

        # ── 7. Flush + kill nodes ────────────────────────────────────
        sleep 1
        cleanup_nodes
        sleep 1

        # ── 8. Move CSVs ─────────────────────────────────────────────
        move_run_files "$i" "$dest_folder"

        sleep 3
        echo "    Run ${i} complete."
    done

    # ── DOV post-processor for this scenario ────────────────────────
    echo ""
    echo "  -> DOV post-processor: ${scenario_name}..."
    if [[ -f "$DOV_SCRIPT" ]]; then
        (cd "$RESULTS_BASE" && "$VENV_PYTHON" "$DOV_SCRIPT" --folder "$scenario_name") \
            && echo "  -> DOV done." \
            || echo "  [WARN] DOV failed for ${scenario_name}"
    else
        echo "  [WARN] dov_postprocessor.py not found: ${DOV_SCRIPT}"
    fi

    echo "  [DONE] ${scenario_name}"
}

# ==========================================
# ── SOURCE ROS ENVIRONMENT ────────────────
# ==========================================
echo "Sourcing ROS 2 environment..."
if [[ -f "${OPENVINS_WS}/install/setup.bash" ]]; then
    source "${OPENVINS_WS}/install/setup.bash"
    echo "  -> ${OPENVINS_WS}/install/setup.bash"
else
    echo "[ERROR] setup.bash not found. Check OPENVINS_WS."
    exit 1
fi

trap cleanup_nodes INT TERM

mkdir -p "$BATCH_ROOT"

# ==========================================
# ── MAIN BATCH LOOP ───────────────────────
# ==========================================
TOTAL_BATCHES="${#BATCH_DEFS[@]}"

echo ""
echo "=========================================================="
echo "  VIODE-VIO Ablation Study — v3"
echo "  Total batches         : ${TOTAL_BATCHES}"
echo "  Runs per scenario     : ${RUNS_PER_SCENARIO}"
echo "  Archive root          : ${BATCH_ROOT}"
echo "=========================================================="

for batch_idx in "${!BATCH_DEFS[@]}"; do
    BATCH_NUM=$((batch_idx + 1))
    RAW="${BATCH_DEFS[$batch_idx]}"

    # Parse pipe-separated fields
    IFS='|' read -r B_LABEL B_DILATION B_MAX_MASK B_MIN_DISP B_MIN_FEAT B_ORB <<< "$RAW"
    B_LABEL="${B_LABEL// /}"      # trim spaces from label
    B_DILATION="${B_DILATION// /}"
    B_MAX_MASK="${B_MAX_MASK// /}"
    B_MIN_DISP="${B_MIN_DISP// /}"
    B_MIN_FEAT="${B_MIN_FEAT// /}"
    B_ORB="${B_ORB// /}"

    BATCH_OUT="${BATCH_ROOT}/batch_${BATCH_NUM}"
    ANALYSIS_LOG="${BATCH_OUT}/analysis.log"
    PARAMS_FILE="${BATCH_OUT}/params.txt"

    echo ""
    echo "######################################################################"
    echo "  BATCH ${BATCH_NUM} / ${TOTAL_BATCHES} : ${B_LABEL}"
    echo "  dilation_kernel=${B_DILATION}  max_mask_fraction=${B_MAX_MASK}"
    echo "  min_disparity=${B_MIN_DISP}    min_features=${B_MIN_FEAT}    orb_nfeatures=${B_ORB}"
    echo "  Output: ${BATCH_OUT}"
    echo "######################################################################"

    # ── Run all scenarios ────────────────────────────────────────────
    for mask_mode in "${MASK_MODES[@]}"; do
        for dataset in "${DATASETS[@]}"; do
            for level in ${DENSITY_LEVELS[$dataset]}; do
                run_scenario \
                    "$mask_mode" "$dataset" "$level" \
                    "$B_DILATION" "$B_MAX_MASK" "$B_MIN_DISP" "$B_MIN_FEAT" "$B_ORB"
            done
        done
    done

    # ── Analyze all environments ─────────────────────────────────────
    echo ""
    echo "  [ANALYSIS] Running analyze_results_v5.py for batch ${BATCH_NUM}..."
    mkdir -p "$BATCH_OUT"

    {
        echo "========================================================================"
        echo "  BATCH ${BATCH_NUM} — ${B_LABEL}"
        echo "  dilation_kernel   = ${B_DILATION}"
        echo "  max_mask_fraction = ${B_MAX_MASK}"
        echo "  min_disparity     = ${B_MIN_DISP}"
        echo "  min_features      = ${B_MIN_FEAT}"
        echo "  orb_nfeatures     = ${B_ORB}"
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

    echo "  [ANALYSIS] Done. Log: ${ANALYSIS_LOG}"

    # ── Write params.txt ─────────────────────────────────────────────
    cat > "$PARAMS_FILE" << EOF
batch_number      = ${BATCH_NUM}
label             = ${B_LABEL}
dilation_kernel   = ${B_DILATION}
max_mask_fraction = ${B_MAX_MASK}
min_disparity     = ${B_MIN_DISP}
min_features      = ${B_MIN_FEAT}
orb_nfeatures     = ${B_ORB}
runs_per_scenario = ${RUNS_PER_SCENARIO}
EOF

    # ── Archive all 24 scenario folders ─────────────────────────────
    echo "  [ARCHIVE] Copying scenario folders to batch_${BATCH_NUM}/..."
    ARCHIVED=0
    EXPECTED_FOLDERS=()
    for mask_mode in "${MASK_MODES[@]}"; do
        for dataset in "${DATASETS[@]}"; do
            for level in ${DENSITY_LEVELS[$dataset]}; do
                EXPECTED_FOLDERS+=("${mask_mode}-${dataset}-${level}")
            done
        done
    done

    for scenario_name in "${EXPECTED_FOLDERS[@]}"; do
        src="${RESULTS_BASE}/${scenario_name}"
        if [[ -d "$src" ]]; then
            cp -r "$src" "${BATCH_OUT}/${scenario_name}"
            ARCHIVED=$((ARCHIVED + 1))
        else
            echo "    [WARN] Scenario folder missing: ${scenario_name}"
        fi
    done
    echo "  [ARCHIVE] Archived ${ARCHIVED}/24 folders."

    # ── Copy trajectory analysis plots ──────────────────────────────
    for env in "parking_lot" "city_day" "city_night"; do
        plot="${RESULTS_BASE}/Trajectory_Analysis_${env}.png"
        if [[ -f "$plot" ]]; then
            cp "$plot" "${BATCH_OUT}/Trajectory_Analysis_${env}.png"
        fi
    done

    # ── Clean up scenario folders from sim_results/ ──────────────────
    echo "  [CLEANUP] Removing scenario folders from sim_results/..."
    for scenario_name in "${EXPECTED_FOLDERS[@]}"; do
        src="${RESULTS_BASE}/${scenario_name}"
        if [[ -d "$src" ]]; then
            rm -rf "$src"
        fi
    done
    echo "  [CLEANUP] sim_results/ cleared for next batch."

    echo ""
    echo "  [BATCH ${BATCH_NUM} COMPLETE] Results at: ${BATCH_OUT}"
    echo "  params.txt   : ${PARAMS_FILE}"
    echo "  analysis.log : ${ANALYSIS_LOG}"
done

# ==========================================
# ── FINAL SUMMARY ─────────────────────────
# ==========================================
echo ""
echo "=========================================================="
echo "  All ${TOTAL_BATCHES} batches complete."
echo ""
echo "  Results archived to:"
for batch_idx in "${!BATCH_DEFS[@]}"; do
    BATCH_NUM=$((batch_idx + 1))
    IFS='|' read -r B_LABEL _ <<< "${BATCH_DEFS[$batch_idx]}"
    B_LABEL="${B_LABEL// /}"
    echo "    batch_${BATCH_NUM}/  [${B_LABEL}]"
done
echo ""
echo "  Each batch_N/ contains:"
echo "    params.txt                   — parameter values for that batch"
echo "    analysis.log                 — analyze_results output for all 3 envs"
echo "    Trajectory_Analysis_*.png    — plots"
echo "    masked-*/unmasked-*/         — 24 scenario folders with all CSVs"
echo "=========================================================="
