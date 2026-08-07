#!/bin/bash
# =============================================================================
# run_nm_sweep.sh
# Stages 2.1–2.4 nm-parameter sweep
#
# Usage:
#   ./run_nm_sweep.sh [RUNS_PER_SCENARIO]
#
#   RUNS_PER_SCENARIO : repetitions per scenario (default: 5)
#
# Stages covered (activate by uncommenting entries in BATCH_DEFS):
#   2.1  imu_residual_max_tri_error sweep {2, 5, 10, 20} px
#   2.2  α × σ grid (15 combinations)
#   2.3  dead-zone multiplier sweep {0, 1, 2, 3, 4}
#   2.4  depth gate sweep {off, 10, 15, 25} m
#
# Each batch runs all 12 MASKED scenarios (3 envs × 4 densities).
# Unmasked is not re-run — unmasked ATE does not depend on nm params.
#
# Per batch, a temporary estimator config is generated inside the install
# config directory so relative calibration file paths resolve correctly.
# All other estimator params are kept at the VIODE defaults.
#
# Masker params are fixed at the Stage 2 baseline:
#   dilation_kernel=13  max_mask_fraction=0.80
#   min_disparity=1.2   min_features=8   orb_nfeatures=100
#
# Results archived to: sim_results/ablation_nm/batch_N/
#   params.txt      — all parameter values for the batch
#   analysis.log    — analyze_results_v5.py output for all 3 envs
#   masked-*/       — 12 scenario folders with all CSVs
# =============================================================================

# ==========================================
# ── PATHS ─────────────────────────────────
# ==========================================
WORKSPACE="/media/neurolab/60a72ba2-3a9d-47d0-88e3-852b0f67f283"   # ← set this to your data root
RESULTS_BASE="${WORKSPACE}/sim_results"
OPENVINS_WS="${WORKSPACE}/openvins_ws"
VIODE_DATASET="${WORKSPACE}/Downloads_Ext/VIODE_Dataset"
OTP_SCRIPT="${VIODE_DATASET}/odom_to_path.py"
# IMPORTANT: SWEEP_CONFIG must live in the same directory as the calibration
# YAMLs so that relative_config_imu / relative_config_imucam resolve correctly.
INSTALL_CFG_DIR="${OPENVINS_WS}/install/ov_msckf/share/ov_msckf/config/viode_config"
BASE_CONFIG="${INSTALL_CFG_DIR}/estimator_config.yaml"
SWEEP_CONFIG="${INSTALL_CFG_DIR}/estimator_config_sweep.yaml"
DOV_SCRIPT="${RESULTS_BASE}/dov_postprocessor.py"
ANALYZE_SCRIPT="${RESULTS_BASE}/analyze_results_v5.py"
VENV_PYTHON="${RESULTS_BASE}/env/bin/python3"
BATCH_ROOT="${RESULTS_BASE}/ablation_nm"

# Fixed masker / hybrid-estimator params (Stage 2 baseline)
DILATION=13
MAX_MASK=0.80
MIN_DISP=1.2
MIN_FEAT=8
ORB_NFEAT=100

RUNS_PER_SCENARIO="${1:-5}"

# ==========================================
# ── BATCH DEFINITIONS ─────────────────────
# ==========================================
# Format (pipe-separated, 6 fields):
#   "label | alpha | sigma_px | dead_zone | max_depth | max_tri_error"
#
#   max_depth:      imu_residual_max_depth in metres; 0.0 = disabled
#   max_tri_error:  imu_residual_max_tri_error in px RMS; 0.0 = disabled
#
# Run one stage at a time: uncomment that stage's block, comment the others.

BATCH_DEFS=(
    # ── Stage 2.1: triangulation quality gate sweep ───────────────────
    # Fixed: α=3 σ=5 dead_zone=3.0 max_depth=0.0 — only tri_error varies
    # "tri2   | 3 | 5 | 3.0 | 0.0 |  2.0"   # tight gate: ~2 px RMS
    # "tri5   | 3 | 5 | 3.0 | 0.0 |  5.0"   # moderate
    # "tri10  | 3 | 5 | 3.0 | 0.0 | 10.0"   # loose
    # "tri20  | 3 | 5 | 3.0 | 0.0 | 20.0"   # very loose (almost never fires)

    # ── Stage 2.2: α × σ grid ─────────────────────────────────────────
    # Fixed: dead_zone=3.0 max_depth=0.0 max_tri_error=0.0
    # "a1_s2   | 1 |  2 | 3.0 | 0.0 | 0.0"
    # "a1_s5   | 1 |  5 | 3.0 | 0.0 | 0.0"
    # "a1_s10  | 1 | 10 | 3.0 | 0.0 | 0.0"
    # "a2_s2   | 2 |  2 | 3.0 | 0.0 | 0.0"
    # "a2_s5   | 2 |  5 | 3.0 | 0.0 | 0.0"
    # "a2_s10  | 2 | 10 | 3.0 | 0.0 | 0.0"
    # "a3_s2   | 3 |  2 | 3.0 | 0.0 | 0.0"
    # "a3_s5   | 3 |  5 | 3.0 | 0.0 | 0.0"
    # "a3_s10  | 3 | 10 | 3.0 | 0.0 | 0.0"
    # "a4_s2   | 4 |  2 | 3.0 | 0.0 | 0.0"
    # "a4_s5   | 4 |  5 | 3.0 | 0.0 | 0.0"
    # "a4_s10  | 4 | 10 | 3.0 | 0.0 | 0.0"
    # "a5_s2   | 5 |  2 | 3.0 | 0.0 | 0.0"
    # "a5_s5   | 5 |  5 | 3.0 | 0.0 | 0.0"
    # "a5_s10  | 5 | 10 | 3.0 | 0.0 | 0.0"

    # ── Stage 2.3: dead-zone ablation ─────────────────────────────────
    # Fixed: α=3 σ=5 max_depth=0.0 max_tri_error=0.0 — τ×σ_px varies
    # "dz0  | 3 | 5 | 0.0 | 0.0 | 0.0"   # no dead zone  → τ = 0.0 px
    # "dz1  | 3 | 5 | 1.0 | 0.0 | 0.0"   #               → τ = 1.5 px
    # "dz2  | 3 | 5 | 2.0 | 0.0 | 0.0"   #               → τ = 3.0 px
    # "dz3  | 3 | 5 | 3.0 | 0.0 | 0.0"   # current default → τ = 4.5 px (in grid above)
    # "dz4  | 3 | 5 | 4.0 | 0.0 | 0.0"   #               → τ = 6.0 px

    # ── Stage 2.4: depth gate ablation ────────────────────────────────
    # Fixed: α=3 σ=5 dead_zone=3.0 max_tri_error=0.0 — max_depth varies
    "d_off | 3 | 5 | 3.0 |  0.0 | 0.0"   # no depth gate
    "d_10  | 3 | 5 | 3.0 | 10.0 | 0.0"   # gate at 10 m
    "d_15  | 3 | 5 | 3.0 | 15.0 | 0.0"   # gate at 15 m  ← repo config
    "d_25  | 3 | 5 | 3.0 | 25.0 | 0.0"   # gate at 25 m
)

# ==========================================
# ── ENVIRONMENT / DENSITY MATRIX ──────────
# ==========================================
declare -A DENSITY_LEVELS
DENSITY_LEVELS["parking_lot"]="none low mid high"
DENSITY_LEVELS["city_day"]="none low mid high"
DENSITY_LEVELS["city_night"]="none low mid high"

DATASETS=("parking_lot" "city_day" "city_night")

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
    # Wait until the daemon is accepting connections before returning.
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
        else
            echo "    [WARN] Expected file not found: ${src}"
        fi
    done
}

# ==========================================
# ── HELPER: GENERATE SWEEP CONFIG YAML ────
# ==========================================
# Writes a copy of the base config with nm params overridden.
# Always sets use_imu_residual: true — the sweep tests active nm.
make_sweep_config() {
    local alpha="$1"
    local sigma_px="$2"
    local dead_zone="$3"
    local max_depth="$4"
    local max_tri_error="$5"

    cp "$BASE_CONFIG" "$SWEEP_CONFIG"

    # Override all six nm parameters in-place.
    # SWEEP_CONFIG lives in the same directory as kalibr_imu_chain.yaml and
    # kalibr_imucam_chain.yaml so OpenVINS resolves relative paths correctly.
    sed -i "s/^use_imu_residual:.*$/use_imu_residual: true/"                                   "$SWEEP_CONFIG"
    sed -i "s/^imu_residual_alpha:.*$/imu_residual_alpha: ${alpha}/"                           "$SWEEP_CONFIG"
    sed -i "s/^imu_residual_sigma_px:.*$/imu_residual_sigma_px: ${sigma_px}/"                  "$SWEEP_CONFIG"
    sed -i "s/^imu_residual_dead_zone:.*$/imu_residual_dead_zone: ${dead_zone}/"               "$SWEEP_CONFIG"
    sed -i "s/^imu_residual_max_depth:.*$/imu_residual_max_depth: ${max_depth}/"               "$SWEEP_CONFIG"
    sed -i "s/^imu_residual_max_tri_error:.*$/imu_residual_max_tri_error: ${max_tri_error}/"   "$SWEEP_CONFIG"
}

# ==========================================
# ── CORE: RUN ONE SCENARIO ────────────────
# ==========================================
run_scenario() {
    local dataset="$1"
    local level="$2"

    local scenario_name="masked-${dataset}-${level}"
    local dest_folder="${RESULTS_BASE}/${scenario_name}"
    local bag_path="${VIODE_DATASET}/${dataset}_${level}"

    if [[ ! -d "$bag_path" ]]; then
        echo "  [SKIP] Bag directory not found: ${bag_path}"
        return 0
    fi

    echo ""
    echo "  ============================================================"
    echo "  SCENARIO : ${scenario_name}"
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

        # ── 1. OpenVINS (with sweep config) ─────────────────────────────
        OV_LOG="${BATCH_OUT}/${scenario_name}_run${i}.log"
        ros2 launch ov_msckf subscribe.launch.py \
            config_path:="$SWEEP_CONFIG" \
            > "$OV_LOG" 2>&1 &
        OV_PID=$!
        timeout 20 bash -c \
            'until ros2 node list 2>/dev/null | grep -q "ov_msckf"; do sleep 0.3; done' || true
        if ! ros2 node list 2>/dev/null | grep -q "ov_msckf"; then
            echo "    [ERROR] OpenVINS did not start in 20 s. Log: ${OV_LOG}"
            echo "    [ERROR] Skipping run ${i} — check ${OV_LOG} for the crash."
            kill "$OV_PID" 2>/dev/null || true
            cleanup_nodes
            continue
        fi
        sleep 2

        # ── 2. Semantic masker (masked mode) ────────────────────────────
        ros2 run ov_softgate masker \
            --ros-args \
            -p dilation_kernel:="${DILATION}" \
            -p max_mask_fraction:="${MAX_MASK}" > /dev/null 2>&1 &
        # Wait for the masker to advertise /cam0/masked before continuing.
        # OpenVINS (use_dynamic_mask=true) won't initialize until it sees that topic.
        timeout 20 bash -c \
            'until ros2 topic list 2>/dev/null | grep -q "/cam0/masked"; do sleep 0.3; done' || true
        sleep 5  # DDS warm-up: let masker→OpenVINS 4-topic sync establish

        # ── 3. Ground-truth path converter ──────────────────────────────
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

        # ── 5. Path recorder + object estimator ─────────────────────────
        ros2 run ov_softgate path_recorder -- "$i" \
            --ros-args -p output_dir:="${RESULTS_BASE}" > /dev/null 2>&1 &

        ros2 run ov_softgate hybrid_speed_estimator "$i" \
            --ros-args \
            -p min_disparity:="${MIN_DISP}" \
            -p min_features:="${MIN_FEAT}" \
            -p orb_nfeatures:="${ORB_NFEAT}" \
            -p calib_file:="${OPENVINS_WS}/src/open_vins/config/viode_config/kalibr_imucam_chain.yaml" \
            -p output_dir:="${RESULTS_BASE}" > /dev/null 2>&1 &

        # Wait for path_recorder to be registered before playing the bag.
        # Python nodes are slow to start on the first run of a session (bytecode
        # compilation + ROS discovery). Without this wait, run 1 misses messages.
        sleep 1

        # ── 6. Play bag ──────────────────────────────────────────────────
        echo "      -> Playing: $(basename "$bag_path")"
        ros2 bag play "$bag_path" --clock --read-ahead-queue-size 10000
        echo "      -> Bag finished."

        # ── 7. Flush + kill nodes ────────────────────────────────────────
        sleep 1
        cleanup_nodes
        sleep 1

        # ── 8. Move CSVs ─────────────────────────────────────────────────
        move_run_files "$i" "$dest_folder"

        sleep 3
        echo "    Run ${i} complete."
    done

    # ── DOV post-processor for this scenario ────────────────────────────
    echo ""
    echo "  -> DOV post-processor: ${scenario_name}..."
    if [[ -f "$DOV_SCRIPT" ]]; then
        (cd "$RESULTS_BASE" && "$VENV_PYTHON" "$DOV_SCRIPT" --folder "$scenario_name") \
            && echo "  -> DOV done." \
            || echo "  [WARN] DOV failed for ${scenario_name}"
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

if [[ ! -f "$BASE_CONFIG" ]]; then
    echo "[ERROR] Base config not found: ${BASE_CONFIG}"
    exit 1
fi

trap cleanup_nodes INT TERM

mkdir -p "$BATCH_ROOT"

# ==========================================
# ── MAIN BATCH LOOP ───────────────────────
# ==========================================
TOTAL_BATCHES="${#BATCH_DEFS[@]}"

# Continue from existing batches so successive stage runs don't overwrite each other.
EXISTING_BATCHES=$(ls -d "${BATCH_ROOT}/batch_"* 2>/dev/null | wc -l)
START_BATCH=$((EXISTING_BATCHES + 1))

echo ""
echo "=========================================================="
echo "  nm Parameter Sweep (Stages 2.1–2.4)"
echo "  Total batches         : ${TOTAL_BATCHES}"
echo "  Starting batch        : ${START_BATCH}"
echo "  Runs per scenario     : ${RUNS_PER_SCENARIO}"
echo "  Scenarios per batch   : 12 (masked only)"
echo "  Archive root          : ${BATCH_ROOT}"
echo "=========================================================="

for batch_idx in "${!BATCH_DEFS[@]}"; do
    BATCH_NUM=$((START_BATCH + batch_idx))
    RAW="${BATCH_DEFS[$batch_idx]}"

    IFS='|' read -r B_LABEL B_ALPHA B_SIGMA B_DEAD B_DEPTH B_TRI <<< "$RAW"
    B_LABEL="${B_LABEL// /}"
    B_ALPHA="${B_ALPHA// /}"
    B_SIGMA="${B_SIGMA// /}"
    B_DEAD="${B_DEAD// /}"
    B_DEPTH="${B_DEPTH// /}"; B_DEPTH="${B_DEPTH:-0.0}"
    B_TRI="${B_TRI// /}";     B_TRI="${B_TRI:-0.0}"

    BATCH_OUT="${BATCH_ROOT}/batch_${BATCH_NUM}"
    ANALYSIS_LOG="${BATCH_OUT}/analysis.log"
    PARAMS_FILE="${BATCH_OUT}/params.txt"
    mkdir -p "$BATCH_OUT"

    echo ""
    echo "######################################################################"
    echo "  BATCH ${BATCH_NUM} / ${TOTAL_BATCHES} : ${B_LABEL}"
    echo "  alpha=${B_ALPHA}  sigma_px=${B_SIGMA}  dead_zone=${B_DEAD}  max_depth=${B_DEPTH}  max_tri_error=${B_TRI}"
    echo "  Output: ${BATCH_OUT}"
    echo "######################################################################"

    # Generate sweep config for this batch
    make_sweep_config "$B_ALPHA" "$B_SIGMA" "$B_DEAD" "$B_DEPTH" "$B_TRI"
    echo "  [CONFIG] Sweep config written: ${SWEEP_CONFIG}"
    echo "    use_imu_residual=true  α=${B_ALPHA}  σ=${B_SIGMA}  dz=${B_DEAD}  depth=${B_DEPTH}  tri=${B_TRI}"

    # ── Run all 12 masked scenarios ──────────────────────────────────────
    for dataset in "${DATASETS[@]}"; do
        for level in ${DENSITY_LEVELS[$dataset]}; do
            run_scenario "$dataset" "$level"
        done
    done

    # ── Analyze all environments ─────────────────────────────────────────
    echo ""
    echo "  [ANALYSIS] Running analyze_results_v5.py for batch ${BATCH_NUM}..."
    mkdir -p "$BATCH_OUT"

    {
        echo "========================================================================"
        echo "  nm SWEEP BATCH ${BATCH_NUM} — ${B_LABEL}"
        echo "  alpha           = ${B_ALPHA}"
        echo "  sigma_px        = ${B_SIGMA}"
        echo "  dead_zone       = ${B_DEAD}"
        echo "  max_depth       = ${B_DEPTH}"
        echo "  max_tri_error   = ${B_TRI}"
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

    # ── Write params.txt ─────────────────────────────────────────────────
    cat > "$PARAMS_FILE" << EOF
batch_number      = ${BATCH_NUM}
label             = ${B_LABEL}
alpha             = ${B_ALPHA}
sigma_px          = ${B_SIGMA}
dead_zone         = ${B_DEAD}
max_depth         = ${B_DEPTH}
max_tri_error     = ${B_TRI}
runs_per_scenario = ${RUNS_PER_SCENARIO}
dilation_kernel   = ${DILATION}
max_mask_fraction = ${MAX_MASK}
min_disparity     = ${MIN_DISP}
min_features      = ${MIN_FEAT}
orb_nfeatures     = ${ORB_NFEAT}
EOF

    # ── Archive 12 scenario folders ──────────────────────────────────────
    echo "  [ARCHIVE] Copying scenario folders to batch_${BATCH_NUM}/..."
    ARCHIVED=0
    for dataset in "${DATASETS[@]}"; do
        for level in ${DENSITY_LEVELS[$dataset]}; do
            scenario_name="masked-${dataset}-${level}"
            src="${RESULTS_BASE}/${scenario_name}"
            if [[ -d "$src" ]]; then
                cp -r "$src" "${BATCH_OUT}/${scenario_name}"
                ARCHIVED=$((ARCHIVED + 1))
            else
                echo "    [WARN] Scenario folder missing: ${scenario_name}"
            fi
        done
    done
    echo "  [ARCHIVE] Archived ${ARCHIVED}/12 folders."

    # Copy trajectory analysis plots
    for env in "parking_lot" "city_day" "city_night"; do
        plot="${RESULTS_BASE}/Trajectory_Analysis_${env}.png"
        [[ -f "$plot" ]] && cp "$plot" "${BATCH_OUT}/Trajectory_Analysis_${env}.png"
    done

    # ── Clean up scenario folders from sim_results/ ──────────────────────
    echo "  [CLEANUP] Removing scenario folders from sim_results/..."
    for dataset in "${DATASETS[@]}"; do
        for level in ${DENSITY_LEVELS[$dataset]}; do
            src="${RESULTS_BASE}/masked-${dataset}-${level}"
            [[ -d "$src" ]] && rm -rf "$src"
        done
    done
    echo "  [CLEANUP] sim_results/ cleared for next batch."

    echo ""
    echo "  [BATCH ${BATCH_NUM} COMPLETE] Results at: ${BATCH_OUT}"
done

# Remove temp sweep config from the install dir
rm -f "$SWEEP_CONFIG"

# ==========================================
# ── FINAL SUMMARY ─────────────────────────
# ==========================================
echo ""
echo "=========================================================="
echo "  All ${TOTAL_BATCHES} batches complete."
echo ""
echo "  Results archived to:"
for batch_idx in "${!BATCH_DEFS[@]}"; do
    BATCH_NUM=$((START_BATCH + batch_idx))
    IFS='|' read -r B_LABEL B_ALPHA B_SIGMA B_DEAD B_DEPTH B_TRI <<< "${BATCH_DEFS[$batch_idx]}"
    B_LABEL="${B_LABEL// /}"; B_ALPHA="${B_ALPHA// /}"; B_SIGMA="${B_SIGMA// /}"
    B_DEAD="${B_DEAD// /}"; B_DEPTH="${B_DEPTH:-0.0}"; B_TRI="${B_TRI:-0.0}"
    echo "    batch_${BATCH_NUM}/  [${B_LABEL}]  α=${B_ALPHA} σ=${B_SIGMA} τ=${B_DEAD} d=${B_DEPTH} tri=${B_TRI}"
done
echo ""
echo "  Each batch_N/ contains:"
echo "    params.txt          — all parameter values"
echo "    analysis.log        — analyze_results output for all 3 envs"
echo "    masked-*/           — 12 scenario folders with all CSVs"
echo "=========================================================="
