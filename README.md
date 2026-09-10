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
│   ├── analyze_zedx_run.py      ZED X run report (trajectory plot + timing)
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

### ZED X live experiments

For on-robot experiments with a ZED X stereo camera (no bag replay):

```bash
# Full pipeline (YOLO masking + VIO + path recorder)
./scripts/run_zedx_live.sh

# With RViz visualization
./scripts/run_zedx_live.sh --rviz

# Unmasked VIO baseline (force_empty — for comparison)
./scripts/run_zedx_live.sh --no-mask

# With timing logs saved into the run folder
./scripts/run_zedx_live.sh --timing
```

Press **Ctrl+C** to stop all nodes. Each run writes its own folder under
`~/ov_results_zedx/`, so nothing is overwritten between runs:

```
~/ov_results_zedx/zedx_<TIMESTAMP>/
├── vio_path_run_1.csv   trajectory          (always)
├── gt_path_run_1.csv    empty for live runs (no ground truth)
├── bag/                 rosbag              (--bag)
└── combined.log         timing log          (--timing)
```

#### Analysing a run

`scripts/analyze_zedx_run.py` reads one run folder and reports the trajectory,
motion statistics, and — when the run was recorded with `--timing` — the same
latency table `parse_timing.py` prints. It reuses that parser, so any pattern
added there shows up here too.

```bash
# Newest run under ~/ov_results_zedx, picked automatically
python3 scripts/analyze_zedx_run.py

# A specific run
python3 scripts/analyze_zedx_run.py ~/ov_results_zedx/zedx_20260910_143022

# Write the report elsewhere, without opening a window
python3 scripts/analyze_zedx_run.py --out ~/reports --no-show
```

| Option | Effect |
|---|---|
| *(positional)* | Run folder to analyse. Omit to use the newest one. |
| `--results-dir` | Where run folders live (default: `~/ov_results_zedx`) |
| `--out` | Where to write the report (default: the run folder itself) |
| `--no-show` | Save without opening a plot window |

It writes two files into the run folder:

| File | Contents |
|---|---|
| `trajectory.png` | Top-down path, height vs time, speed vs time, and mean latency per component against the 50 ms frame budget |
| `analysis.txt` | Pose count, duration, pose rate, 3D/XY path length, start→end displacement, mean/max speed, vertical drift |

Components the live pipeline does not run (`hybrid_speed_estimator`) show as `–`
in the table. Over SSH there is no display, so the plot is saved rather than
shown — copy the PNG off the Jetson to view it.

**Before first use — edit the USER CONFIG block at the top of `scripts/run_zedx_live.sh`:**

| Variable | What to set |
|---|---|
| `CONFIG_PATH` | Path to `estimator_config_zedx.yaml` (default: `src/open_vins/config/zedx_config/`) |
| `MODEL_PATH` | Path to YOLO weights (default: `models/yolo26s-seg.pt`; use `yolo11n-seg.pt` on low-memory Jetson) |
| `ZED_LEFT_TOPIC` / `ZED_RIGHT_TOPIC` | ZED ROS2 wrapper image topics — check with `ros2 topic list` |
| `IMU_TOPIC` | `/imu/data` for Xsens MTi; `/zed/zed_node/imu/data` for ZED X internal IMU |
| `OUTPUT_DIR` | Parent directory for run folders (each run creates `zedx_<TIMESTAMP>/` inside it) |

**Before first use — fill the three kalibr template files in `src/open_vins/config/zedx_config/`:**

| File | What to fill |
|---|---|
| `kalibr_imucam_chain_zedx.yaml` | Camera intrinsics, `T_imu_cam`, `rostopic` for each camera |
| `kalibr_imu_chain_zedx.yaml` | IMU noise params, `rostopic`, `update_rate` |
| `estimator_config_zedx.yaml` | `track_frequency` to match ZED framerate; everything else is ready |

Get intrinsics from your Kalibr calibration output or directly from the ZED SDK (`getCameraInformation().camera_configuration.calibration_parameters`). For rectified images (`image_rect_gray`) set `distortion_coeffs: [0,0,0,0]`.

**Note on ZED wrapper:** the script expects the ZED ROS2 wrapper to be already running. To have the script launch it, uncomment the `ros2 launch zed_wrapper zed_camera.launch.py` block near the top of the script.

---

## Jetson Deployment (ZED X + Xsens)

Tested on JetPack 5.x / 6.x (Ubuntu 20.04 / 22.04) with Jetson AGX Orin or Orin NX.

### 1 — System prerequisites

```bash
# ROS2 Humble (skip if already installed)
sudo apt install ros-humble-desktop ros-humble-rmw-cyclonedds-cpp
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Python deps for YOLO masker
pip3 install ultralytics

# Build deps for ROS packages
sudo apt install python3-colcon-common-extensions python3-vcstool
```

### 2 — ZED ROS2 wrapper

Install ZED SDK for Jetson from [Stereolabs downloads](https://www.stereolabs.com/developers/release/),
then build the ROS2 wrapper:

```bash
mkdir -p ~/zed_ws/src && cd ~/zed_ws/src
git clone --recursive https://github.com/stereolabs/zed-ros2-wrapper.git
cd ..
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 3 — Xsens MTi driver (skip if using ZED X internal IMU)

```bash
mkdir -p ~/xsens_ws/src && cd ~/xsens_ws/src
git clone https://github.com/bluespace-ai/bluespace_ai_xsens_ros_mti_driver.git
cd ..
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

Launch: `ros2 launch bluespace_ai_xsens_ros_mti_driver xsens_mti_node.launch.py`
Default IMU topic: `/imu/data` at 100 Hz.

### 4 — Clone and build this repo

```bash
git clone https://github.com/Alpsource/SoftGate-VIO.git
cd SoftGate-VIO

# Pull the OpenVINS fork (SoftGate nm lives there)
vcs import src < .repos

source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 5 — Add your calibration files

Copy your Kalibr output into `src/open_vins/config/zedx_config/` and fill in the
three template files (see TODO comments inside each):

```
src/open_vins/config/zedx_config/
├── estimator_config_zedx.yaml          ← set track_frequency to match ZED framerate
├── kalibr_imucam_chain_zedx.yaml       ← intrinsics + T_imu_cam + rostopic per camera
└── kalibr_imu_chain_zedx.yaml          ← noise params + rostopic + update_rate
```

### 6 — Run

Terminal 1 — ZED wrapper:
```bash
source ~/zed_ws/install/setup.bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedx
```

Terminal 2 — Xsens (skip if using ZED X internal IMU):
```bash
source ~/xsens_ws/install/setup.bash
ros2 launch bluespace_ai_xsens_ros_mti_driver xsens_mti_node.launch.py
```

Terminal 3 — SoftGate-VIO:
```bash
cd SoftGate-VIO
source install/setup.bash
./scripts/run_zedx_live.sh
```

Optional with RViz: `./scripts/run_zedx_live.sh --rviz`

The trajectory is saved to `~/ov_results_zedx/zedx_<TIMESTAMP>/vio_path_run_1.csv` when you
press Ctrl+C, alongside the rosbag (`--bag`) and timing log (`--timing`) for that same run.

### Jetson performance notes

| Model | GPU inference | Recommendation |
|---|---|---|
| `yolo26s-seg.pt` (23 MB) | ~30 ms on Orin NX | Good for 30 Hz ZED framerate |
| `yolo11n-seg.pt` (5.9 MB) | ~12 ms on Orin NX | Use if VIO is CPU-bound |
| `yolo11s-seg.pt` (20 MB) | ~25 ms on Orin NX | Alternative mid-size |

Set `MODEL_PATH` in `run_zedx_live.sh` to switch models without rebuilding.

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
