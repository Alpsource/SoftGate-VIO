#!/bin/bash
# =============================================================================
# run_kaist_evaluation.sh
# KAIST Complex Urban — four-condition Phase 1 ablation.
#
# Conditions (same as VIODE run_phase1_evaluation.sh):
#   unmasked  — force_empty mask, use_imu_residual: false  → U-VIO baseline
#   imu_only  — force_empty mask, use_imu_residual: true   → IMU-VIO
#   yolo      — YOLO mask,        use_imu_residual: false  → M-VIO-YOLO
#   yolo_imu  — YOLO mask,        use_imu_residual: true   → M+IMU-VIO
#
# KAIST bags used: urban38_10min and urban39_10min (~10 min each, 10Hz stereo).
# yolo_masker input topics are remapped from /cam0/image_raw → /stereo/left/image_raw
# (KAIST bags use stereo/left|right/image_raw — raw images, no pre-rectification).
#
# Ground truth:
#   KAIST bags have no GT topic. GT is pre-extracted from global_pose.csv
#   (3x4 SE3 matrices in UTM, provided with each raw sequence) into relative
#   trajectory CSVs, then copied per-run. Since the bag content is fixed, GT
#   is identical across runs.
#
# Results saved to:  ${RESULTS_BASE}/kaist_eval/batch_N/
# Folder naming:     {mode}-kaist-{seq}  (e.g. yolo-kaist-urban38)
#   → compatible with analyze_kaist_results.py for ATE comparison.
#
# Usage:
#   ./run_kaist_evaluation.sh          # 5 runs per sequence (default)
#   ./run_kaist_evaluation.sh 3        # 3 runs per sequence
# =============================================================================

set -euo pipefail

WORKSPACE="/media/neurolab/60a72ba2-3a9d-47d0-88e3-852b0f67f283"
OPENVINS_WS="${WORKSPACE}/openvins_ws"
KAIST_DIR="${WORKSPACE}/Downloads_Ext/KAIST Urban"
RESULTS_BASE="${WORKSPACE}/sim_results"
BATCH_ROOT="${RESULTS_BASE}/kaist_eval"
MODEL_PATH="${OPENVINS_WS}/models/yolo26s-seg.pt"
VENV_PYTHON="${RESULTS_BASE}/env/bin/python3"
GT_CONVERTER="${OPENVINS_WS}/tools/kaist_gt_to_csv.py"
ANALYZE_SCRIPT="${WORKSPACE}/sim_results/analyze_kaist_results.py"
DOV_POSTPROCESSOR="${WORKSPACE}/sim_results/dov_postprocessor.py"
KAIST_CALIB="${OPENVINS_WS}/src/open_vins/config/kaist_urban/kalibr_imucam_chain.yaml"

CONFIG_BASE="${OPENVINS_WS}/install/ov_msckf/share/ov_msckf/config/kaist_urban"
CONFIG_PLAIN="${CONFIG_BASE}/estimator_config.yaml"
CONFIG_IMU="${CONFIG_BASE}/estimator_config_imu_residual.yaml"

RUNS_PER_SEQ="${1:-5}"

# ── YOLO parameters ───────────────────────────────────────────────────────────
DILATION_KERNEL=5
MAX_MASK_FRACTION=0.50
CONF_THRESHOLD=0.25
FLOW_DYNAMIC_THRESHOLD=5.0
FLOW_MIN_FEATURES=5
USE_FLOW_CLASSIFIER=true
YOLO_DEVICE="${YOLO_DEVICE:-cuda}"

# ── IMU residual parameters (same as VIODE batch_7) ──────────────────────────
IMU_RESIDUAL_ALPHA=1.5
IMU_RESIDUAL_SIGMA_PX=10.0
IMU_RESIDUAL_INIT_DELAY=5.0

# ── Sequences and bag metadata ────────────────────────────────────────────────
# Bag start/end timestamps (nanoseconds) — from: ros2 bag info <bag>
declare -A BAG_START_NS
declare -A BAG_END_NS
BAG_START_NS["urban38"]=1559193232373910975
BAG_END_NS["urban38"]=1559195394405655837
BAG_START_NS["urban39"]=1559195795611174799
BAG_END_NS["urban39"]=1559197662137509957

SEQUENCES=("urban38" "urban39")
MASK_MODES=("unmasked" "imu_only" "yolo" "yolo_imu")

BATCH_LABEL="anchored_imd_rawbags_fullseq_alpha${IMU_RESIDUAL_ALPHA}_sigma${IMU_RESIDUAL_SIGMA_PX}_delay${IMU_RESIDUAL_INIT_DELAY}"

KILL_LIST=(
    "ov_msckf"
    "yolo_masker"
    "hybrid_speed_estimator"
    "path_recorder"
    "static_transform_publisher"
)

# ── Auto-number batch ─────────────────────────────────────────────────────────
BATCH_NUM=1
while [[ -d "${BATCH_ROOT}/batch_${BATCH_NUM}" ]]; do
    BATCH_NUM=$((BATCH_NUM + 1))
done
BATCH_OUT="${BATCH_ROOT}/batch_${BATCH_NUM}"
ANALYSIS_LOG="${BATCH_OUT}/analysis.log"
PARAMS_FILE="${BATCH_OUT}/params.txt"

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
    timeout 15 bash -c 'until ros2 node list > /dev/null 2>&1; do sleep 0.5; done' || true
    echo "  [CLEANUP] Done."
}

# ── GT PRE-GENERATION ─────────────────────────────────────────────────────────
gen_gt_csv() {
    local seq="$1"
    local gt_csv="${BATCH_ROOT}/gt_${seq}.csv"
    if [[ -f "$gt_csv" ]]; then
        echo "  [GT] Reusing: $gt_csv"
    else
        echo "  [GT] Generating ground truth for ${seq}..."
        python3 "$GT_CONVERTER" \
            --seq_dir "${KAIST_DIR}/${seq}-pankyo" \
            --out_csv "$gt_csv" \
            --t_start_ns "${BAG_START_NS[$seq]}" \
            --t_end_ns   "${BAG_END_NS[$seq]}"
        echo "  [GT] Done: $gt_csv"
    fi
}

# ── CORE: RUN ONE SCENARIO ────────────────────────────────────────────────────
run_scenario() {
    local mask_mode="$1"
    local seq="$2"

    local scenario_name="${mask_mode}-kaist-${seq}"
    local dest_folder="${RESULTS_BASE}/${scenario_name}"
    local bag_path="${KAIST_DIR}/${seq}_10min_raw"
    local gt_csv="${BATCH_ROOT}/gt_${seq}.csv"

    if [[ ! -d "$bag_path" ]]; then
        echo "  [SKIP] Bag not found: ${bag_path}"
        return 0
    fi

    echo ""
    echo "  ============================================================"
    echo "  SCENARIO : ${scenario_name}  (${RUNS_PER_SEQ} runs)"
    echo "  ============================================================"
    # Remove any stale data from previous crashed batches before writing new results
    rm -rf "$dest_folder"
    mkdir -p "$dest_folder"

    echo "  [PRE-FLIGHT] Clearing stale processes..."
    for target in "${KILL_LIST[@]}"; do
        pkill -9 -f "$target" > /dev/null 2>&1 || true
    done
    sleep 2

    for ((i=1; i<=RUNS_PER_SEQ; i++)); do
        echo ""
        echo "    ── Run ${i} / ${RUNS_PER_SEQ} ──────────────────"

        # ── Select config ────────────────────────────────────────────────────
        local config_path
        if [[ "$mask_mode" == "yolo_imu" || "$mask_mode" == "imu_only" ]]; then
            config_path="$CONFIG_IMU"
        else
            config_path="$CONFIG_PLAIN"
        fi

        # ── 1. OpenVINS ──────────────────────────────────────────────────────
        ros2 launch ov_msckf subscribe.launch.py \
            config_path:="$config_path" \
            use_sim_time:=true > /dev/null 2>&1 &
        timeout 15 bash -c \
            'until ros2 node list 2>/dev/null | grep -q "ov_msckf"; do sleep 0.3; done' || true
        sleep 2

        # ── 2. YOLO masker (with KAIST topic remapping) ──────────────────────
        # KAIST bags publish on /stereo/left|right/image_raw; yolo_masker
        # subscribes to /cam0|1/image_raw — remap at launch time.
        if [[ "$mask_mode" == "yolo" || "$mask_mode" == "yolo_imu" ]]; then
            ros2 run yolo_masker yolo_masker \
                --ros-args \
                -r /cam0/image_raw:=/stereo/left/image_raw \
                -r /cam1/image_raw:=/stereo/right/image_raw \
                -p model_path:="${MODEL_PATH}" \
                -p device:="${YOLO_DEVICE}" \
                -p confidence_threshold:="${CONF_THRESHOLD}" \
                -p dilation_kernel:="${DILATION_KERNEL}" \
                -p max_mask_fraction:="${MAX_MASK_FRACTION}" \
                -p use_flow_classifier:="${USE_FLOW_CLASSIFIER}" \
                -p flow_dynamic_threshold:="${FLOW_DYNAMIC_THRESHOLD}" \
                -p flow_min_features:="${FLOW_MIN_FEATURES}" \
                > /dev/null 2>&1 &
        else
            # force_empty: publishes zero-masks to maintain 4-topic sync
            ros2 run yolo_masker yolo_masker \
                --ros-args \
                -r /cam0/image_raw:=/stereo/left/image_raw \
                -r /cam1/image_raw:=/stereo/right/image_raw \
                -p model_path:="${MODEL_PATH}" \
                -p device:="${YOLO_DEVICE}" \
                -p force_empty:=true \
                > /dev/null 2>&1 &
        fi

        echo "      -> Waiting for yolo_masker ready (GPU warmup)..."
        timeout 90 bash -c \
            'until ros2 topic echo --once /yolo_masker/ready 2>/dev/null | grep -q "data: true"; do sleep 0.5; done' \
            || echo "      [WARN] /yolo_masker/ready not seen — proceeding anyway"
        sleep 2

        # ── 3. hybrid_speed_estimator (yolo/yolo_imu only — DOV object CSV) ──
        if [[ "$mask_mode" == "yolo" || "$mask_mode" == "yolo_imu" ]]; then
            ros2 run ov_softgate hybrid_speed_estimator -- "$i" \
                --ros-args \
                -r /cam0/image_raw:=/stereo/left/image_raw \
                -r /cam1/image_raw:=/stereo/right/image_raw \
                -p calib_file:="${KAIST_CALIB}" \
                -p output_dir:="${RESULTS_BASE}" \
                -p mask_source:=yolo \
                -p min_features:=8 \
                -p orb_nfeatures:=100 \
                -p min_disparity:=1.2 \
                > /dev/null 2>&1 &
            sleep 1
        fi

        # ── 4. path_recorder (VIO path only; GT is injected offline) ─────────
        ros2 run ov_softgate path_recorder -- "$i" \
            --ros-args -p output_dir:="${RESULTS_BASE}" > /dev/null 2>&1 &
        sleep 1

        # ── 5. Play bag ──────────────────────────────────────────────────────
        # YOLO conditions use 1x: GPU inference can't keep up at 2x — delayed masks corrupt VIO.
        # Non-YOLO conditions use 2x safely (no mask processing bottleneck).
        local play_rate=2.0
        if [[ "$mask_mode" == "yolo" || "$mask_mode" == "yolo_imu" ]]; then
            play_rate=1.0
        fi
        echo "      -> Playing: $(basename "$bag_path") at ${play_rate}x"
        ros2 bag play "$bag_path" --clock --read-ahead-queue-size 10000 --rate "$play_rate"
        echo "      -> Bag finished."

        # ── 6. Flush + kill ──────────────────────────────────────────────────
        sleep 1
        cleanup_nodes
        sleep 1

        # ── 7. Move VIO CSV, dynamic_objects CSV, and inject GT CSV ──────────
        mkdir -p "$dest_folder"
        local vio_src="${RESULTS_BASE}/vio_path_run_${i}.csv"
        if [[ -f "$vio_src" ]]; then
            mv "$vio_src" "${dest_folder}/vio_path_run_${i}.csv"
            echo "    [MOVE] vio_path_run_${i}.csv -> $(basename "$dest_folder")/"
        else
            echo "    [WARN] vio_path_run_${i}.csv not found at ${RESULTS_BASE}"
        fi

        local dov_src="${RESULTS_BASE}/dynamic_objects_run_${i}.csv"
        if [[ -f "$dov_src" ]]; then
            mv "$dov_src" "${dest_folder}/dynamic_objects_run_${i}.csv"
            echo "    [MOVE] dynamic_objects_run_${i}.csv -> $(basename "$dest_folder")/"
        fi

        # Copy pre-generated GT (identical across runs — same bag, same GT)
        if [[ -f "$gt_csv" ]]; then
            cp "$gt_csv" "${dest_folder}/gt_path_run_${i}.csv"
            echo "    [GT]   gt_path_run_${i}.csv injected from pre-generated GT"
        else
            echo "    [WARN] GT CSV not found: ${gt_csv}"
        fi

        sleep 3
        echo "    Run ${i} complete."
    done

    # ── DOV postprocessor (yolo/yolo_imu only) ────────────────────────────────
    if [[ "$mask_mode" == "yolo" || "$mask_mode" == "yolo_imu" ]]; then
        if [[ -f "$DOV_POSTPROCESSOR" ]]; then
            echo "  [DOV] Running dov_postprocessor on ${scenario_name}..."
            (cd "${RESULTS_BASE}" && "${VENV_PYTHON}" dov_postprocessor.py --folder "${scenario_name}" 2>&1) \
                || echo "  [WARN] DOV postprocessor failed for ${scenario_name}"
        else
            echo "  [WARN] dov_postprocessor.py not found at ${DOV_POSTPROCESSOR}"
        fi
    fi

    echo "  [DONE] ${scenario_name}"
}

# ── SOURCE ROS ────────────────────────────────────────────────────────────────
echo "Sourcing ROS 2 environment..."
set +u
source "${OPENVINS_WS}/install/setup.bash"
set -u

trap cleanup_nodes INT TERM
mkdir -p "$BATCH_OUT"

# ── Pre-flight: verify config files exist in install/ ────────────────────────
for cfg in "$CONFIG_PLAIN" "$CONFIG_IMU"; do
    if [[ ! -f "$cfg" ]]; then
        echo "[ERROR] Config not found in install/: $cfg"
        echo "        Fix: ln -s <src_path> $cfg"
        echo "        Or:  colcon build --symlink-install --packages-select ov_msckf"
        exit 1
    fi
done
echo "  [OK] Both config files found in install/."

echo ""
echo "=========================================================="
echo "  KAIST Phase 1 — Four-Condition Ablation"
echo "  Batch         : batch_${BATCH_NUM} (${BATCH_LABEL})"
echo "  Sequences     : urban38, urban39"
echo "  Runs/seq      : ${RUNS_PER_SEQ}"
echo "  Output        : ${BATCH_OUT}"
echo "=========================================================="
echo ""
echo "  Conditions:"
echo "    unmasked : force_empty + use_imu_residual=false  → U-VIO"
echo "    imu_only : force_empty + use_imu_residual=true   → IMU-VIO"
echo "    yolo     : YOLO mask   + use_imu_residual=false  → M-VIO-YOLO"
echo "    yolo_imu : YOLO mask   + use_imu_residual=true   → M+IMU-VIO"
echo ""
echo "  KAIST-specific:"
echo "    yolo_masker remaps /cam0/image_raw → /stereo/left/image_raw"
echo "    GT injected offline from global_pose.csv (UTM → relative SE3)"
echo "=========================================================="

# ── Pre-generate GT CSVs ──────────────────────────────────────────────────────
echo ""
echo "Pre-generating ground truth CSVs..."
for seq in "${SEQUENCES[@]}"; do
    gen_gt_csv "$seq"
done

# ── Run all scenarios ─────────────────────────────────────────────────────────
for mask_mode in "${MASK_MODES[@]}"; do
    for seq in "${SEQUENCES[@]}"; do
        run_scenario "$mask_mode" "$seq"
    done
done

# ── Analyze ───────────────────────────────────────────────────────────────────
echo ""
echo "  [ANALYSIS] Running analyze_kaist_results.py..."

{
    echo "=========================================================================="
    echo "  KAIST Batch ${BATCH_NUM} — ${BATCH_LABEL}"
    echo "  Conditions: unmasked | imu_only | yolo | yolo_imu"
    echo "  imu_residual_alpha = ${IMU_RESIDUAL_ALPHA}"
    echo "  imu_residual_sigma_px = ${IMU_RESIDUAL_SIGMA_PX}"
    echo "  imu_residual_init_delay = ${IMU_RESIDUAL_INIT_DELAY}"
    echo "  yolo_conf = ${CONF_THRESHOLD}  dilation = ${DILATION_KERNEL}"
    echo "  runs_per_seq = ${RUNS_PER_SEQ}"
    echo "=========================================================================="
    echo ""
} > "$ANALYSIS_LOG"

if [[ -f "$ANALYZE_SCRIPT" ]]; then
    python3 "$ANALYZE_SCRIPT" --batch "$RESULTS_BASE" \
        | tee -a "$ANALYSIS_LOG"
else
    echo "    [WARN] analyze_kaist_results.py not found at ${ANALYZE_SCRIPT}"
    echo "    [INFO] Run manually: python3 ${ANALYZE_SCRIPT} --batch ${RESULTS_BASE}"
fi

# ── params.txt ────────────────────────────────────────────────────────────────
cat > "$PARAMS_FILE" << EOF
batch_number          = ${BATCH_NUM}
label                 = ${BATCH_LABEL}
sequences             = urban38, urban39
conditions            = unmasked | imu_only | yolo | yolo_imu
use_imu_residual      = true (imu_only and yolo_imu)
imu_residual_alpha    = ${IMU_RESIDUAL_ALPHA}
imu_residual_sigma_px = ${IMU_RESIDUAL_SIGMA_PX}
imu_residual_init_delay = ${IMU_RESIDUAL_INIT_DELAY}
feat_rep_msckf        = ANCHORED_MSCKF_INVERSE_DEPTH
yolo_conf             = ${CONF_THRESHOLD}
dilation_kernel       = ${DILATION_KERNEL}
max_mask_fraction     = ${MAX_MASK_FRACTION}
use_flow_clf          = ${USE_FLOW_CLASSIFIER}
flow_threshold        = ${FLOW_DYNAMIC_THRESHOLD}
dov_postprocessor     = enabled (yolo and yolo_imu)
runs_per_seq          = ${RUNS_PER_SEQ}
EOF

# ── Archive scenario folders ──────────────────────────────────────────────────
echo "  [ARCHIVE] Archiving to batch_${BATCH_NUM}/..."
ARCHIVED=0
for mask_mode in "${MASK_MODES[@]}"; do
    for seq in "${SEQUENCES[@]}"; do
        src="${RESULTS_BASE}/${mask_mode}-kaist-${seq}"
        if [[ -d "$src" ]]; then
            cp -r "$src" "${BATCH_OUT}/${mask_mode}-kaist-${seq}"
            ARCHIVED=$((ARCHIVED + 1))
        else
            echo "  [WARN] Missing: ${mask_mode}-kaist-${seq}"
        fi
    done
done
# Archive GT CSVs into batch folder too
cp "${BATCH_ROOT}/gt_urban38.csv" "${BATCH_OUT}/" 2>/dev/null || true
cp "${BATCH_ROOT}/gt_urban39.csv" "${BATCH_OUT}/" 2>/dev/null || true
echo "  [ARCHIVE] ${ARCHIVED}/8 scenario folders archived."

# ── Clean up sim_results/ ─────────────────────────────────────────────────────
echo "  [CLEANUP] Clearing scenario folders from sim_results/..."
for mask_mode in "${MASK_MODES[@]}"; do
    for seq in "${SEQUENCES[@]}"; do
        rm -rf "${RESULTS_BASE}/${mask_mode}-kaist-${seq}"
    done
done

echo ""
echo "=========================================================="
echo "  KAIST evaluation complete."
echo "  Results : ${BATCH_OUT}"
echo "  Analysis: ${ANALYSIS_LOG}"
echo "=========================================================="
