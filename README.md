# SoftGate-VIO

**Continuous per-feature measurement-noise weighting for visual-inertial odometry in dynamic environments.**

Standard MSCKF-based VIO systems apply a binary chi-squared gate: features either pass or are hard-rejected. SoftGate-VIO replaces that gate with a continuous noise multiplier *nm* derived from each triangulated feature's IMU reprojection residual. Dynamic features receive *nm* >> 1 — their contribution to the covariance update is down-weighted, not discarded — allowing the estimator to remain robust under partial dynamic-scene coverage without sacrificing track continuity.

> **Paper:** *SoftGate-VIO: Continuous Per-Feature Measurement-Noise Weighting for Visual-Inertial Odometry in Dynamic Environments* — under review.

---

## Repository layout

```
SoftGate-VIO/
├── .repos                 vcs import → Alpsource/open_vins (SoftGate nm lives here)
├── src/
│   ├── ov_softgate/       GT semantic masker · stereo DOV tracker · path recorder
│   └── yolo_masker/       YOLOv26s segmentation masker + ego-motion flow classifier
├── scripts/
│   ├── run_experiments.sh       canonical VIODE experiment runner
│   ├── run_ablation.sh          parameter sweep (masker + estimator)
│   ├── run_yolo_experiments.sh  YOLO-masked batch runner
│   ├── run_phase1_evaluation.sh Phase 1 IMU-residual three-condition comparison
│   ├── run_kaist_evaluation.sh  KAIST Complex Urban evaluation runner
│   ├── run_timing_all.sh        per-component latency sweep (all conditions)
│   ├── run_timing_test.sh       single-scenario timing test
│   ├── run_timing_subcomp.sh    sub-component breakdown (verbosity:=ALL)
│   ├── run_timing_basecfg.sh    city timing with 600-feat base config
│   ├── parse_timing.py          timing log parser → latency table
│   ├── make_paper_timing_table.py  aggregates timing folders → paper tables + LaTeX
│   ├── run_zedx_live.sh         live pipeline for ZED X stereo camera
│   ├── debug_run.sh             interactive single-run with RViz
│   └── debug_kaist_bag.sh       KAIST bag playback monitor
├── analysis/
│   ├── dov_postprocessor.py     offline world-frame EKF DOV correction
│   ├── analyze_results_v5.py    ATE tables and summary statistics
│   └── analyze_ablation_batches.py  cross-batch ablation comparison
├── models/                YOLO segmentation weights (≤23 MB each)
└── tools/
    ├── convert_kaist_to_ros2bag.py  KAIST Complex Urban → ROS2 bag converter
    ├── compare_masks_offline.py     offline YOLO-vs-GT mask metric tool
    └── test_yolo_speed.py           YOLO GPU latency benchmark (20 Hz budget check)
```

The SoftGate *nm* mechanism is implemented in the OpenVINS fork (`Alpsource/open_vins`, branch `feature/dynamic-masking-viode`) inside `ov_msckf/src/update/UpdaterMSCKF.cpp`. This workspace contains the masking pipeline, DOV tracker, and experiment infrastructure that surrounds it.

---

## System architecture

```
VIODE / KAIST ROS2 bag
  │
  ├─► ov_softgate: semantic_masker  ──────────────────► /cam0/masked
  │     (GT colour IDs 241–251)                         /cam1/masked
  │   ─ or ─
  ├─► yolo_masker                   ──────────────────► /cam0/masked  (VIO mask)
  │     (YOLOv26s + flow classifier)                    /cam0/objects (DOV label map)
  │
  ├─► OpenVINS ov_msckf  ◄── SoftGate nm inflation (IMU residual gate)
  │     MSCKF stereo VIO
  │     /ov_msckf/poseimu · /ov_msckf/pathimu
  │
  ├─► ov_softgate: hybrid_speed_estimator
  │     stereo ORB + centroid fallback → dynamic_objects_run_N.csv
  │
  └─► ov_softgate: path_recorder
        → vio_path_run_N.csv · gt_path_run_N.csv

  Post-processing (offline):
    vio_path + dynamic_objects → dov_postprocessor.py
      world-frame EKF per object → velocity-weighted ego correction
      → dov_path_run_N.csv
```

---

## Results (VIODE dataset, 10 runs per scenario)

YOLOv26s masker (`yolo26s-seg.pt`, confidence 0.25, flow classifier threshold 2.0 px).
Mask Δ = ATE improvement of YOLO-masked VIO over unmasked VIO (positive = better).

| Environment  | None   | Low    | Mid    | High     |
|--------------|-------:|-------:|-------:|---------:|
| Parking lot  | +43.6% | +26.3% | +35.1% | **+66.5%** |
| City day     |  −6.4% | −15.6% | +26.4% |  +4.5%   |
| City night   | −19.3% | +32.9% | −12.2% |  +9.0%   |

Phase 1 IMU-residual gate (α = 3.0): **8 / 12** VIODE scenarios improve over Phase 0 baseline.

The DOV post-processor provides additional ATE reduction in parking lot scenarios where quasi-static parked cars supply stable stereo anchors.

---

## Requirements

- **ROS2 Humble** on Ubuntu 22.04
- **Python 3.10+** with `opencv-python`, `pandas`, `numpy`
- **YOLO masker only:** `pip install ultralytics` + CUDA-capable GPU

---

## Setup

```bash
# 1. Clone — the repo root is your ROS2 workspace
git clone https://github.com/Alpsource/SoftGate-VIO.git
cd SoftGate-VIO

# 2. Pull the OpenVINS fork (SoftGate nm lives there)
vcs import src < .repos

# 3. Build
source /opt/ros/humble/setup.bash
colcon build

# 4. Source
source install/setup.bash
```

---

## Running experiments

Edit the **three path variables** at the top of the script for your machine:

```bash
WORKSPACE="/your/data/root"          # parent of openvins_ws and sim_results
RESULTS_BASE="${WORKSPACE}/sim_results"
VIODE_DATASET="${WORKSPACE}/VIODE_Dataset"
```

Then:

```bash
# GT masker — one environment, both masked and unmasked, 10 runs
./scripts/run_experiments.sh parking_lot both 10

# YOLO masker
./scripts/run_yolo_experiments.sh

# Ablation parameter sweep (edit BATCH_DEFS in the script first)
./scripts/run_ablation.sh

# Phase 1 three-condition comparison (baseline / imu_residual / gate)
./scripts/run_phase1_evaluation.sh
```

### Analysing results

```bash
cd $RESULTS_BASE

# ATE tables for one environment
python3 analyze_results_v5.py --env parking_lot

# DOV correction on existing scenario folder
python3 dov_postprocessor.py --folder masked-parking_lot-high

# Cross-batch ablation comparison
python3 analyze_ablation_batches.py
```

### Timing analysis

Measures per-component latency by replaying a bag and parsing timestamped log output.

```bash
# Single scenario (choose any env / density / mask mode)
./scripts/run_timing_test.sh parking_lot high yolo

# Full sweep — 3 conditions × 3 environments × 4 densities = 36 runs
./scripts/run_timing_all.sh all

# Sub-component breakdown (verbosity:=ALL, parking_lot only, ~7 min)
./scripts/run_timing_subcomp.sh

# City environments with 600-feature base config (real-time baseline)
./scripts/run_timing_basecfg.sh

# Aggregate all three folders into paper tables + LaTeX
python3 scripts/make_paper_timing_table.py \
    --sweep   timings/<sweep_folder> \
    --subcomp timings/<subcomp_folder> \
    --basecfg timings/<basecfg_folder> \
    --latex
```

Each run saves logs to `timings/<label>_TIMESTAMP/`. The `parse_timing.py` script
is called automatically at the end of every timing script and prints a latency table.
Frozen workstation numbers are in `timings/20260819_130134/` (main sweep),
`timings/subcomp_20260820_125408/` (sub-components), and
`timings/basecfg_20260820_121652/` (city base config).

### ZEDX live experiments

For on-robot experiments with a ZED X stereo camera (no bag replay):

```bash
# Minimal — pipeline only, no RViz, no logging
./scripts/run_zedx_live.sh

# With RViz visualization
./scripts/run_zedx_live.sh --rviz

# With timing logs saved to timings/zedx_TIMESTAMP/
./scripts/run_zedx_live.sh --timing

# Both
./scripts/run_zedx_live.sh --rviz --timing
```

Press **Ctrl+C** to stop all nodes cleanly. With `--timing`, a latency table is
printed on exit and logs are saved to `timings/zedx_TIMESTAMP/`.

**Before first use**, edit the `TODO: ZEDX` block at the top of the script:

1. Install the [ZED ROS2 wrapper](https://github.com/stereolabs/zed-ros2-wrapper)
2. Confirm the camera namespace and topic names (defaults match ZED SDK v4)
3. Create `src/open_vins/config/zedx_config/estimator_config_zedx.yaml`
   and `kalibr_imucam_chain_zedx.yaml` with ZEDX intrinsics / extrinsics
4. Uncomment the `ros2 launch zed_wrapper zedx.launch.py` line in the script
5. Start the ZED wrapper (or let the script launch it), then run the script above

The pipeline uses the same VIODE internal topic convention (`/cam0/image_raw`,
`/cam1/image_raw`) — only the YOLO masker remaps its subscriptions to the ZED topics,
so all other nodes remain unchanged.

---

## Packages

### `ov_softgate`

| Node | Executable | Role |
|---|---|---|
| `SemanticMasker` | `masker` | VIODE GT segmentation → binary VIO mask |
| `FastHybridSpeedEstimator` | `hybrid_speed_estimator` | Stereo ORB + centroid DOV tracker |
| `PathRecorder` | `path_recorder` | VIO / GT / DOV trajectory CSV export |

Key parameters for `hybrid_speed_estimator` (injectable via `--ros-args`):

| Parameter | Default | Description |
|---|---|---|
| `calib_file` | `""` | Path to `kalibr_imucam_chain.yaml` |
| `output_dir` | `~/ov_results` | CSV output directory |
| `mask_source` | `gt` | `gt` or `yolo` |
| `min_features` | `8` | ORB features required per object |
| `min_disparity` | `1.2` | Min stereo disparity (~15.7 m max depth) |
| `orb_nfeatures` | `100` | ORB detector budget |

### `yolo_masker`

YOLOv26s-based masker with ego-motion-compensated optical flow classifier. Publishes two topics per camera: `/cam0/masked` (VIO mask, dynamic objects only) and `/cam0/objects` (all-detection label map for DOV).

Key parameters:

| Parameter | Default | Description |
|---|---|---|
| `model_path` | `""` | Path to `.pt` segmentation model **(required)** |
| `use_flow_classifier` | `true` | Static/dynamic discrimination via KLT flow |
| `force_empty` | `false` | Publish all-zeros mask (unmasked control) |
| `confidence_threshold` | `0.25` | YOLO detection confidence cutoff |

Launch:

```bash
ros2 launch yolo_masker yolo_masker.launch.py \
    model_path:=$(pwd)/models/yolo26s-seg.pt
```

### OpenVINS fork (`Alpsource/open_vins`)

The SoftGate nm mechanism adds three parameters to `estimator_config.yaml`:

```yaml
use_imu_residual: true       # enable SoftGate nm inflation
imu_residual_alpha: 3.0      # nm = 1 + alpha * (1 - exp(-r_eff / sigma_px))
imu_residual_sigma_px: 5.0   # decay constant (pixels)
imu_residual_max_depth: 15.0 # depth gate — suppress nm beyond this depth (m)
```

---

## Datasets

- **VIODE** — [https://github.com/matsuren/viode](https://github.com/matsuren/viode)  
  Stereo + IMU + semantic segmentation. 3 environments × 4 traffic densities. Native ROS2 bags.

- **KAIST Complex Urban** — [https://sites.google.com/view/complex-urban-dataset](https://sites.google.com/view/complex-urban-dataset)  
  Convert raw sequences to ROS2 bags with `tools/convert_kaist_to_ros2bag.py`.

---

## YOLO model weights

Pre-trained weights are in `models/`. All models are standard Ultralytics segmentation checkpoints trained on COCO. No custom training was performed.

| File | Size | Use |
|---|---|---|
| `yolo26s-seg.pt` | 23 MB | **Used in all reported experiments** |
| `yolo11n-seg.pt` | 5.9 MB | Lightweight alternative for resource-constrained deployment |
| `yolo11s-seg.pt` | 20 MB | Alternative mid-size variant |

---

## Citation

If you use this work, please cite:

```bibtex
@article{demirel2026softgatevio,
  title   = {SoftGate-VIO: Continuous Per-Feature Measurement-Noise Weighting
             for Visual-Inertial Odometry in Dynamic Environments},
  author  = {Demirel, Alp},
  journal = {under review},
  year    = {2026}
}
```

---

## License

MIT — see [LICENSE](LICENSE).
