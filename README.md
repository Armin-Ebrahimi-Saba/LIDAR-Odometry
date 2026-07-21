# GLIM (CPU-only) — Ouster OS0-32

CPU-only build of [GLIM](https://github.com/koide3/glim) (LiDAR-Inertial SLAM)
for the Sensor Systems project (Task 3: LIDAR-based Positioning and Mapping). 
No NVIDIA GPU required (pure CPU). Tested on WSL2, Ubuntu 24.04, ROS2 Jazzy.

## Repository structure
```
LIDAR-based-Positioning/
├── config/                          # GLIM config (CPU mode, tuned noise params)
├── data/                             # gitignored (regenerate locally)
│   ├── Test1_data/
│   │   ├── rosbag/                  # original course bag
│   │   └── rosbag_glim/             # OUTPUT of bag_converter.py -> what GLIM reads
│   └── xtrack_gnss_corrected/       # ground truth CSV for evaluation
├── ros2_packages/aspn_msgs/         # reconstructed custom message package
├── scripts/
│   ├── bag_converter.py             # aspn_msgs -> sensor_msgs/Imu, sign fix, timeoffset fix
│   ├── utils.py                     # shared: load traj/GNSS, ENU<->LatLon, SE3 alignment
│   ├── export_trajectory.py         # Deliverable 1: LatLon + NED velocity CSV
│   ├── compute_rmse.py              # Deliverable 3: RMSE/ATE + error plots
│   ├── ply2pcd.py                   # Deliverable 2: PLY -> PCD + topdown map
│   ├── plot_raw_gnss.py             # diagnostic: raw GNSS sanity check
│   └── find_timeoffset.py           # diagnostic: IMU/GNSS clock-sync check
├── results/
│   ├── dump/<run_name>/             # raw GLIM output per run
│   └── deliverables/                # final CSV/PCD/plots for submission
└── README.md
```
`~/src/` (GTSAM, gtsam_points, Iridescence) and `~/ros2_ws/` (glim, glim_ros2)
live outside this repo (see setup below to rebuild). `results/dump/` is used
instead of GLIM's `/tmp/dump` default, since WSL2's `/tmp` is wiped on reboot.

## System architecture

```mermaid
flowchart TD
    A[Raw course rosbag<br/>aspn_msgs IMU, timeoffset field] -->|bag_converter.py| B[Converted rosbag<br/>sensor_msgs/Imu, deskew-ready points]
    B --> C[glim_rosbag]
    subgraph GLIM Pipeline
        C --> D[Odometry Estimation<br/>CPU: GICP + iVox, fixed-lag smoothing]
        D --> E[Local/Sub Mapping<br/>keyframe submaps]
        E --> F[Global Mapping<br/>submap-pair VGICP factors + IMU]
    end
    F --> G[results/dump/run2/<br/>traj_lidar.txt, submaps, graph.bin]
    G -->|export_trajectory.py --offset -307.00| H[run2_trajectory_latlon_ned.csv]
    G -->|compute_rmse.py --offset -307.00| I[RMSE + error plots]
    G -->|offline_viewer + ply2pcd.py| J[map_run2.pcd + topdown images]
    K[GNSS ground truth CSV] --> H
    K --> I
```

The raw bag can't be fed to GLIM directly: custom IMU message type, inverted
IMU sign convention, non-standard per-point timestamp field. `bag_converter.py`
would fix all three. GLIM's pipeline itself is unmodified upstream GLIM (see
[koide3/glim](https://github.com/koide3/glim), [paper](https://arxiv.org/abs/2407.10344)). 
This repository focuses on the conversion layer, CPU-only build, sensor-specific
config tuning, clock synchronization calibration and the evaluation scripts producing the three required deliverables:
- 2D trajectory (LatLon) and velocity (NED-frame) estimation result (e.g. saved as .csv file);
- Error plot: Estimate vs. Ground Truth (GNSS) using RMSE as metrics;
- 3D point cloud map saved as .pcd.

## Choice of algorithms / system design

- **GLIM** chosen here over LIO-SAM/KISS-ICP to test its fixed-lag smoothing +
  keyframe odometry, which is robust to brief LiDAR degeneracy, and global submap-pair
  registration to avoids pose-graph Gaussian-covariance approximation errors.
- **CPU path** (`libodometry_estimation_cpu.so`, GICP+iVox): CPU only. This is GLIM's 
  fallback path, not its primary design point (see Results discussion).
- **IMU source**: Ouster's own embedded IMU (`/ouster/imu_meas`), not Pixhawk. Since 
  its axes already match the LiDAR frame, `T_lidar_imu` uses GLIM's manufacturer-default Ouster OS0 value
  (`[0.006, -0.012, 0.008, 0,0,0,1]`), tested against identity as a control (see Results).
- **No online GNSS fusion.** GLIM operates as a pure LiDAR-Inertial Odometry (LIO) framework. GNSS is used strictly 
  as post-hoc for rigid SE(3) spatial alignment and ground truth evaluation. Camera fusion is natively supported 
  but this needs real D435i calibration.
- **Trajectory-to-GNSS alignment and Clock Sync**: GLIM's local frame has no absolute heading reference and its bag timestamps are
  offset by `-307.00 s` relative to GNSS UTC time. Alignment to GNSS local-ENU is performed via SE(3) Umeyama least squares 
  (no scaling) after applying the clock offset. This should aligns with `evo`'s ATE metric, which the GLIM paper itself relies on.
- **Noise parameters** (`imu_acc_noise`, `imu_gyro_noise`, `imu_bias_noise`) tuned via manual 
  parameter sweeps.

---

# Setup

## 1. Install ROS2 Jazzy
```bash
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-jazzy-desktop python3-colcon-common-extensions python3-rosdep
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && source ~/.bashrc
```
> TLS error on `packages.ros.org`? Use `http://` instead (antivirus HTTPS
> inspection is a common cause; apt's real security is the GPG keyring).
> GUI file dialogs not appearing? `sudo apt install -y zenity`.

## 2. GTSAM 4.3a0 (from source — PPA build is ABI-broken as of July 2026,
see footnote¹)
```bash
sudo apt install -y build-essential cmake git libboost-all-dev libeigen3-dev \
  libomp-dev libmetis-dev libfmt-dev libspdlog-dev libglm-dev libglfw3-dev libpng-dev libjpeg-dev
mkdir -p ~/src && cd ~/src && git clone https://github.com/borglab/gtsam && cd gtsam
git checkout 4.3a0 && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_BUILD_TESTS=OFF \
  -DGTSAM_WITH_TBB=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF
make -j$(nproc) && sudo make install && sudo ldconfig
```

## 3. gtsam_points (CPU-only)
```bash
cd ~/src && git clone https://github.com/koide3/gtsam_points
mkdir gtsam_points/build && cd gtsam_points/build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_WITH_CUDA=OFF -DBUILD_WITH_TBB=OFF \
  -DBUILD_WITH_OPENMP=ON -DBUILD_WITH_MARCH_NATIVE=OFF
make -j$(nproc) && sudo make install && sudo ldconfig
```

## 4. Iridescence (viewer)
```bash
cd ~/src && git clone https://github.com/koide3/iridescence --recursive
mkdir iridescence/build && cd iridescence/build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc) && sudo make install && sudo ldconfig
```

## 5. GLIM via colcon
```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/koide3/glim && git clone https://github.com/koide3/glim_ros2
cd ~/ros2_ws
colcon build --cmake-args -DBUILD_WITH_CUDA=OFF -DBUILD_WITH_VIEWER=ON -DBUILD_WITH_MARCH_NATIVE=OFF
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc && source ~/.bashrc
```

## 6. Custom message packages
```bash
cd ~/ros2_ws/src
git clone -b release/1.17 https://github.com/PX4/px4_msgs.git
ln -s ~/LIDAR-based-Positioning/ros2_packages/aspn_msgs ~/ros2_ws/src/aspn_msgs
cd ~/ros2_ws && colcon build --packages-select px4_msgs aspn_msgs && source install/setup.bash
```

¹ *`glim_rosbag` failed with `undefined symbol:
_ZTVN5gtsam28PreintegratedImuMeasurementsE` (reproducible across multiple
version pins, a build inconsistency in the PPA itself, not fixable by
version selection. Source build should avoid it.)*

---

# Pipeline: raw bag → deliverables

### 0. Bag on WSL filesystem
Copy `metadata.yaml` + `rosbag_0.db3` into `data/Test1_data/rosbag/`. Avoid
`/mnt/c/...` and use native WSL filesystem for performance.

### 1. Convert the bag
```bash
cd ~/LIDAR-based-Positioning
python3 scripts/bag_converter.py data/Test1_data/rosbag data/Test1_data/rosbag_glim
```
This fixes costum IMU message types, negates accelerometer values to conform to the standard gravity (+9.81m/s^2),
and converts point timestamps to seconds for proper LIDAR deskewing.

### 2. Run GLIM
```bash
mkdir -p results/dump/test1
ros2 run glim_ros glim_rosbag $(realpath data/Test1_data/rosbag_glim) \
  --ros-args -p config_path:=$(realpath config) \
  -p auto_quit:=true -p dump_path:=$(realpath results/dump/test1)
```
**Always set `auto_quit:=true`**, otherwise `glim_rosbag` finishes reading
the bag and then idles forever waiting for live messages (`rclcpp::spin()`),
which would look like a hang. **Always set `dump_path`** under `results/dump/`.
For faster runs, comment out `libstandard_viewer.so`/`librviz_viewer.so` in
`config/config_ros.json` (GUI rendering competes with GLIM's own CPU).

### 3. Export trajectory — LatLon + NED velocity (Deliverable 1)
```bash
python3 scripts/export_trajectory.py \
  results/dump/dump_test2/traj_lidar.txt \
  data/xtrack_gnss_corrected/xtrack_global_position_t12.csv \
  results/deliverables/run2_trajectory_latlon_ned.csv \
  --offset -307.00
```
Exports the GLIM's local trajectory transformed into global lat-lon-alt coordinates and calculates
finite-difference velocities in the North-East-Down (NED) frame.

### 4. RMSE + error plots (Deliverable 2)
```bash
python3 scripts/compute_rmse.py \
  results/dump/dump_test2/traj_lidar.txt \
  data/xtrack_gnss_corrected/xtrack_global_position_t12.csv \
  results/deliverables/run2 "run2" \
  --offset -307.00
```
Applies the `-307.0s`timestamp calibration offset, performs Umeyama SE(3) alignment against the GNSS ground truth,
calculates RMSE position error, and generates spatial trajectory and temporal error plots.

### 5. Point cloud map as PCD (Deliverable 3)
```bash
# 1. Open offline viewer to save map
ros2 run glim_ros offline_viewer $(realpath results/dump/dump_test2)
# In GUI viewer: File -> Save -> Export Points -> save as e.g. results/deliverables/run2/map_run2.ply

# 2. Convert PLY to PCD and render 2D maps
python3 scripts/ply2pcd.py \
  results/deliverables/run2/map_run2.ply \
  results/deliverables/run2/map_run2.pcd \
  0.05 \
  results/deliverables/run2/map_run2.png
```
Downsamples the point cloud with a `0.05m` voxel grid, exports `map_run2.pcd` and generates 2D top-down
occupancy `map_run2_gray.png` and height map `map_run2_height.png` images.

## Data conversion details (`bag_converter.py`)

| Fix | Problem | Solution |
|---|---|---|
| IMU message type | `aspn_msgs/MeasurementIMU` (custom, GLIM can't subscribe) | Convert to `sensor_msgs/msg/Imu` |
| Accelerometer sign | At rest read `[0.15, -0.12, **-9.6**]`. GTSAM expects **+9.81** on the up-axis at rest (per GLIM docs) | Perform transformation into the 3 axes |
| Per-point timestamps | Field named `timeoffset` (ms), not `t`/`time`/`timestamp` GLIM recognizes → fell back to pseudo-timestamps, degrading deskewing | Rename to `time`, convert ms→s |

`aspn_msgs` has no public ROS2 package, since it's an own implementation of the 
[ASPN 2023 ICD](https://github.com/Open-PNT/ASPN-ICD) spec (YAML, not code) plus a `std_msgs/Header`. 
Here, the `.msg` files are reconstructed directly from the spec (`ros2_packages/aspn_msgs/`).
*(verified correct against real data (decoded gravity magnitude ≈9.6-9.7 m/s², `scripts/verify_aspn_imu.py`.)*

---

# Results / Evaluation

## Parameter sweep (RMSE, full Test1 bag, ~816s processed)

| Run | acc/gyro noise | bias noise | Other | RMSE [m] | Notes |
|---|---|---|---|---|---|
| run1 | 0.05 / 0.02 | 1e-5 (default) | — | - | Baseline |
| **run2** | **0.01 / 0.005** | **1e-5** | — | **24.071** | **Final config** |
| run3 | 0.05 / 0.02 | 0.01 | — | - | - |
| run4 | 0.1 / 0.05 | 1e-3 | — | - | - |
| run5 | 1e-5 / 1e-5 | 1e-5 | — | - | - |
| run6 | 0.01 / 0.005 | 1e-5 | `T_lidar_imu`=identity | - | - |
| run7 | 0.01 / 0.005 | 1e-5 | deskew off | - | - |

<p align="center">
  <img src="results/deliverables/raw_gnss_check.png" width="500">
  <br>
  <em>Raw GNSS check</em>
</p>

<p align="center">
  <img src="results/deliverables/run2/run2_error_plot.png" width="500">
  <br>
  <em>Error plot of run2</em>
</p>

<p align="center">
  <img src="results/deliverables/run2/run2_satellite.png" width="500">
  <br>
  <em>GLIM trajectory of run2 and GNSS in satellite view</em>
</p>

<p align="center">
  <img src="results/deliverables/run2/map_run2_topdown_gray.png" width="500">
  <br>
  <em>Generated map of run2 in top-down view (grayscale)</em>
</p>

<p align="center">
  <img src="results/deliverables/run2/map_run2_topdown_height.png" width="500">
  <br>
  <em>Generated map of run2 in top-down view (colored height map)</em>
</p>

[![Run 2 demonstration](https://img.youtube.com/vi/KeU6mBA8BLE/maxresdefault.jpg)](https://www.youtube.com/watch?v=KeU6mBA8BLE)

## Technical Analysis of Error Curve Dynamics
As seen in the figures before, the positional error exhibits possible specific behaviors:
- **Initial high error (~60m dropping to <8m at t=140s)**. GLIM began recording 136 seconds before the GNSS receiver started logging. During
  this period, GLIM possibly tracked the vehicle driving 60m toward the initial GNSS fix location. At t ~ 140s, the vehicle arrived
  at the GNSS start location (`GNSS index 0`), causing the positional error to drop below 8 meters.
- **Periodic Error Oscillations (5 to 30 meters)**. As the vehicle completed loops, unconstrained yaw (heading) drift caused the
  estimated path to periodically diverge from the ground truth. Every time the vehicle loop crossed the GNSS trajectory in 2D space,
  error dropped to <5m.
- **Global SE(3) Alignment Behavior**. Umeyama alignment minimizes total squared error across all 6802 poses simulaneously. It is possible
  that it intentionally avoids pinning the initial pose to `0m` error, doing so would force all accumulated drift into the end of the run,
  blowing up overall RMSE beyond `80m`. 

## Why results are ~24m RMSE, which are worse than LIO-SAM's and KISS-ICP's results
While **24.071m RMSE** represents substantial improvement over initial uncalibrated runs (>72m, not in the report), several
architectural and environmental factors could prevent the accuracy:

- **Unbounded Heading (Yaw) Drift in Pure LIO**
  GLIM runs as a pure LiDAR-Inertial Odometry system without compass/magnetometer or online GNSS factor integration. Over an
  ~816s driving covering ~1.5km, an uncorrected heading drift of even just 1°-2° accumulates into 20-30m of spatial displacement
  at the far ends of the loop.
- **No GPU**
  Here, the CPU path (GICP+iVox) is a lower-fidelity approximation of GLIM's intended 
  VGICP_GPU pipeline, not just a slower version of it.
- **Sparse LiDAR resolution (Ouster OS0-32)??**
  The 32-beam sensor provides lower vertical point density compared to 64- or 128-beam units. It open roadside environments with
  featureless geometry, scan matching exhibits weak constraints along the direction of travel.
- **Drift in the embedded IMU**
  GLIM's tight coupling assumes a well-behaved IMU, while LIO-SAM's looser coupling or KISS-ICP's 
  IMU-independence may be more robust to this specifically.
- **Weak Implicit Loop Closures**
  GLIM's implicit overlap-based closure vs. LIO-SAM's explicit detection (radius search + ICP verify) may trigger more reliably 
  at weaker overlap.
- **Manual tuning**: 7 manual tuning of configurations is used rather than 
  for example Allan-variance-based calibration.

## Known limitations

- **Pedestrian / Dynamic Object "Ghosting"**: Trailing point cloud artifacts exist near sidewalks due to the lack of dedicated
  dynamic object filtering (standard for raw point cloud SLAM)
- **Evaluated primarily only on Test1 dataset**
