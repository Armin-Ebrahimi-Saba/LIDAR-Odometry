# GLIM (CPU-only) — Ouster OS0-32 + Pixhawk

CPU-only build of [GLIM](https://github.com/koide3/glim) (LiDAR-Inertial SLAM) for
the Sensor Systems project (Task 3: LIDAR-based Positioning and Map Generation).
No NVIDIA GPU required. Tested on WSL2, Ubuntu 24.04, ROS2 Jazzy.

## Workspace layout

- **This repo** = the working directory. `config/` holds GLIM's config (CPU
  variants already selected for odometry/sub_mapping/global_mapping). `data/`
  (gitignored) is where datasets/bags go locally.
- **`~/src/`** (outside this repo). This contains source checkouts of GTSAM, gtsam_points, and
  Iridescence, built and installed system-wide via `make install`.
- **`~/ros2_ws/`** (outside this repo). This is a colcon workspace containing the `glim`
  and `glim_ros2` repos, built via `colcon build`. To rebuild it locally please refer to the following steps below.

## Repository file structure
```
LIDAR-based-Positioning/
├── config/                          # GLIM config (CPU mode, tuned for our sensors)
├── data/                            # gitignored -- download/regenerate locally
│   ├── Test1_data/
│   │   ├── rosbag/                  # original course bag (metadata.yaml + .db3)
│   │   ├── rosbag_data/             # small extras (camera_infos.txt, plotjuggler csv, laz_clouds)
│   │   └── rosbag_glim/             # OUTPUT of bag_converter.py -- what GLIM actually reads
│   ├── demo/                        # official GLIM demo dataset (sanity-check bag)
│   └── xtrack_gnss_corrected/       # ground truth CSVs + KML for evaluation
├── ros2_packages/
│   └── aspn_msgs/                   # our reconstructed custom message package
├── scripts/
│   ├── bag_converter.py             # aspn_msgs -> sensor_msgs/Imu + timeoffset fix
│   └── verify_aspn_imu.py           # sanity-check script for IMU decoding
└── README.md
```

`rosbag_glim/` is the directory that `glim_rosbag` actually points to. Regenerate it with `bag_converter.py`
if the conversion logic changes.

## Why build from source instead of the PPA

koide3 provides prebuilt packages via a PPA (`ros-jazzy-glim-ros`,
`libgtsam-points-dev`), which is much faster to install. We tried the
CPU-only PPA packages for Ubuntu 24.04 but were broken (per July 2026). Running `glim_rosbag` failed
with `undefined symbol: _ZTVN5gtsam28PreintegratedImuMeasurementsE`, which is an ABI
mismatch between the packaged `libgtsam-notbb-dev 4.3.0` and the `gtsam_points`/
`glim_ros` binaries. This was reproducible even after pinning to older,
previously-matched package versions (1.2.0), so it isn't a version-skew issue we
could fix by picking a different combination. It is a build inconsistency in the
PPA itself. Building the full stack from source, all against the same GTSAM
headers, avoids it entirely. 

```bash
curl -s https://koide3.github.io/ppa/setup_ppa.sh | sudo bash
sudo apt install -y libiridescence-dev libboost-all-dev libglfw3-dev libmetis-dev
sudo apt install -y libgtsam-points-dev ros-jazzy-glim-ros
```

If `ros2 run glim_ros glim_rosbag ...` throws a `symbol lookup error`, skip to the
source build below.

## 1. Install ROS2 Jazzy

```bash
sudo apt update && sudo apt install -y curl gnupg lsb-release

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-jazzy-desktop python3-colcon-common-extensions python3-rosdep
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 2. Build GTSAM 4.3a0 from source

```bash
sudo apt install -y build-essential cmake git \
  libboost-all-dev libeigen3-dev \
  libomp-dev libmetis-dev libfmt-dev libspdlog-dev \
  libglm-dev libglfw3-dev libpng-dev libjpeg-dev

mkdir -p ~/src && cd ~/src
git clone https://github.com/borglab/gtsam
cd gtsam
git checkout 4.3a0
mkdir build && cd build
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
  -DGTSAM_BUILD_TESTS=OFF \
  -DGTSAM_WITH_TBB=OFF \
  -DGTSAM_USE_SYSTEM_EIGEN=ON \
  -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF
make -j$(nproc)
sudo make install
sudo ldconfig
```

## 3. Build gtsam_points from source (CPU-only)

```bash
cd ~/src
git clone https://github.com/koide3/gtsam_points
mkdir gtsam_points/build && cd gtsam_points/build
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_WITH_CUDA=OFF \
  -DBUILD_WITH_TBB=OFF \
  -DBUILD_WITH_OPENMP=ON \
  -DBUILD_WITH_MARCH_NATIVE=OFF
make -j$(nproc)
sudo make install
sudo ldconfig
```

## 4. Build Iridescence (map viewer)

```bash
cd ~/src
git clone https://github.com/koide3/iridescence --recursive
mkdir iridescence/build && cd iridescence/build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
sudo ldconfig
```

## 5. Build GLIM via colcon

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/koide3/glim
git clone https://github.com/koide3/glim_ros2

cd ~/ros2_ws
colcon build --cmake-args \
  -DBUILD_WITH_CUDA=OFF \
  -DBUILD_WITH_VIEWER=ON \
  -DBUILD_WITH_MARCH_NATIVE=OFF

echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Verify:
```bash
ros2 pkg list | grep glim
# should list: glim, glim_ros
```

## 6. Config

`config/` in this repo already has CPU variants selected in `config.json`:
```json
"config_odometry": "config_odometry_cpu.json",
"config_sub_mapping": "config_sub_mapping_cpu.json",
"config_global_mapping": "config_global_mapping_cpu.json",
```

Run GLIM against a rosbag, pointing explicitly at this config folder:
```bash
cd ~/LIDAR-based-Positioning   # or wherever this repo is cloned
ros2 run glim_ros glim_rosbag <path_to_bag> --ros-args -p config_path:=$(realpath config)
```

Output (trajectory files, submaps, factor graph) is saved to `/tmp/dump`.

## Test dataset (demo / sanity check)

Official GLIM demo bag, used to verify if this whole pipeline works before pointing
it at the project data (Test1_data):

```bash
mkdir -p data && cd data
curl -L -o os1_128_01_downsampled.tar.gz "https://zenodo.org/record/7233945/files/os1_128_01_downsampled.tar.gz?download=1"
tar -xzf os1_128_01_downsampled.tar.gz
```

(~406 MB, ROS2 `.db3` format.) Note: the link on GLIM's own docs page
(`staff.aist.go.jp`) was found to serve a truncated/corrupt file as of July 2026. Use the Zenodo link above instead.

Run with:
```bash
ros2 run glim_ros glim_rosbag $(realpath data/os1_128_01_downsampled) --ros-args -p config_path:=$(realpath config)
```

## Custom ROS2 message packages

This project depends on two non-standard ROS2 message packages that aren't part of
a normal ROS2 install. Both could be found in `ros2_packages/` in this repo and get
symlinked into the colcon workspace (`~/ros2_ws/src/`) at build time.

## Data conversion pipeline (`scripts/bag_converter.py`)
Converts the raw course bag into something GLIM can consume correctly. Currently
performs three transformations on `/ouster/points` and `/ouster/imu_meas`:

1. **`aspn_msgs/MeasurementIMU` → `sensor_msgs/msg/Imu`** (topic renamed to
   `/imu/data`)

2. **Accelerometer sign flip.** At rest, the raw IMU reported `linear_acceleration
   ≈ [0.15, -0.12, -9.6]` -- negative on the gravity axis. GTSAM's IMU
   preintegration (used internally by GLIM) expects the standard "specific
   force" convention, positive along the up axis when stationary. Left
   uncorrected, GLIM bootstraps the wrong "up" direction from the first IMU
   reading, corrupting orientation from the very first scan (visually: the map
   looked upside-down, structures appeared in negative-z). Fix: negate all three
   accelerometer axes during conversion.

3. **`timeoffset` field rename + unit conversion.** `/ouster/points` carries a
   per-point time field named `timeoffset` (not one of GLIM's recognized names:
   `t`, `time`, `time_stamp`, `timestamp`), so GLIM silently fell back to
   pseudo-timestamps (uniformly spaced across the estimated scan duration)
   instead of real per-point timing -- degrading motion-distortion correction
   (deskewing), especially during turns. Per course staff (email, July 2026):
   `timeoffset = (timestamp - first_timestamp) / 1e6`, i.e. **milliseconds**
   relative to the first point in each scan. Fix: rename the field to `time` and
   divide values by 1000 (ms -> s) during conversion, so GLIM's
   `autoconf_perpoint_times` auto-detection picks it up correctly.

Regenerate the converted bag after any change to the script:
```bash
rm -rf data/Test1_data/rosbag_glim
python3 scripts/bag_converter.py data/Test1_data/rosbag data/Test1_data/rosbag_glim
```

### `px4_msgs`

Official PX4 message definitions (release/1.17), required to read the
`/fmu/out/*` topics in the course rosbags. Not vendored in this repo. This should be cloned
directly:

```bash
cd ~/ros2_ws/src
git clone -b release/1.17 https://github.com/PX4/px4_msgs.git
cd ~/ros2_ws
colcon build --packages-select px4_msgs
```

### `aspn_msgs` (reconstructed, vendored in this repo under `ros2_packages/aspn_msgs`)

The given project rosbags publish IMU data (`/ouster/imu_meas`) and attitude
(`/ouster/imu_att`) using a **custom, non-standard message type**
(`aspn_msgs/msg/MeasurementIMU`, `aspn_msgs/msg/MeasurementAttitude3D`) instead
of the standard `sensor_msgs/msg/Imu`. GLIM cannot subscribe to these directly.

**There is no public `aspn_msgs` ROS2 package to install.** `aspn_msgs` is not a
piece of software, rather an own implementation of the [ASPN 2023 ICD](https://github.com/Open-PNT/ASPN-ICD) 
(a data schema *specification* published as YAML) with a `std_msgs/Header` added on
top. We reconstructed the `.msg` files ourselves directly from that spec, matching:

- `measurements/measurement_IMU.yaml` → `MeasurementIMU.msg`
- `measurements/measurement_attitude_3d.yaml` → `MeasurementAttitude3D.msg`
- `types/type_header.yaml` → `TypeHeader.msg`
- `types/type_timestamp.yaml` → `TypeTimestamp.msg`
- `types/type_integrity.yaml` → `TypeIntegrity.msg`

Design choices made where the YAML spec is ambiguous about ROS2-specific
serialization details:
- ASPN enums (`imu_type`, `reference_frame`, `error_model`, `integrity_method`) →
  `uint8`
- Variable-length arrays (`type_integrity[num_integrity]`) → ROS2 unbounded
  array syntax (`TypeIntegrity[]`)
- Optional fields (`float64?`) → plain `float64` (ROS2 has no true optional
  primitive; reads as `0.0` when unused — doesn't affect `meas_accel`/`meas_gyro`)
- 3×3 matrix (`tilt_error_covariance`) → flattened `float64[9]`, row-major
- The added `std_msgs/Header` is assumed to come first, with the original ASPN
  header field renamed `aspn_header` to avoid a name collision

**Verified correct** against real bag data (`scripts/verify_aspn_imu.py`): decoded
accelerometer magnitude consistently reads ~9.6-9.7 m/s² (≈ gravity) and gyro
values are small and smooth, confirming the field layout matches what the
recording pipeline actually used.

Build it:
```bash
cd ~/ros2_ws
colcon build --packages-select aspn_msgs
source install/setup.bash
```

Verify against a bag at any time:
```bash
python3 scripts/verify_aspn_imu.py data/Test1_data/rosbag
```

### LIDAR–IMU extrinsics

We use `/ouster/imu_meas` (raw IMU) as GLIM's IMU source, not a Pixhawk topic.
This raw IMU is physically internal to the Ouster LiDAR unit, and its axes already correspond to the LiDAR's own frame.
**`T_lidar_imu` = identity**.

### Ground truth

Use `data/xtrack_gnss_corrected/xtrack_global_position_t12.csv` as ground truth
for RMSE evaluation, not the raw `/fmu/out/vehicle_gps_position` bag topic.

### Reference: PX4 message definitions

Not used as core SLAM input in this pipeline, but documented here for
reference (e.g. cross-checking against PX4's own odometry estimate later):
https://docs.px4.io/v1.16/en/msg_docs/

**Verified working** (2026-07-11): full SLAM pipeline (odometry → local mapping →
global mapping) ran end-to-end on CPU only, viewer displayed live map + trajectory,
output written to `/tmp/dump`.

## Known issues / investigation log
**Resolved:**
- GTSAM PPA ABI mismatch -> built from source (see top of README)
- `aspn_msgs` missing -> reconstructed from ASPN-ICD spec (see above)
- Accelerometer sign convention -> fixed in `bag_converter.py`
- Missing per-point deskewing timestamps -> fixed in `bag_converter.py`
  (`timeoffset` field rename/rescale)
- `imu_bias_noise`: tested `1e-3`/`1e-4`/`1e-5`. `1e-3` looked like it helped
  *before* the accel-sign fix (likely compensating for that deeper bug rather
  than a real noise-model improvement). Reverted to the documented default
  (`1e-5`) once the accel-sign fix was in place.

**Open / unresolved:**
- **"Tornado" scattering artifact**: point clouds scatter heavily around the
  platform's own position, particularly noticeable when the platform is
  stationary for an extended period. Not yet root-caused but the candidate
  explanations include residual drift accumulation or submap loop-closure
  weakness during long stationary segments. Moving object "ghosting" (trailing 
  point-cloud copies e.g. behind pedestrians) occurs, especially for a point-cloud SLAM system without
  dedicated dynamic-object filtering.
- **Long-run performance on CPU**: global mapping cost grows over the course of
  the ~13.7 min recording (denser submap-pair factor graph, which is consistent with
  the GLIM paper's own reported behavior), causing the GUI to become slow/
  unresponsive in the back half of a run on this hardware. Also not a hang, confirmed via `top` (high CPU%, not 0%) 
  during an apparently "stuck" period. Closing the GUI mid-run triggers a partial save (whatever was
  processed up to that point) and does not corrupt output.
- **GNSS/camera fusion**: considered but not pursued. GLIM has no built-in GNSS
  factor (would require writing a custom extension module via its "global
  callback slot" mechanism). Visual constraints are natively supported but require real D435i 
  calibration (currently placeholder values in `config_sensors.json`).

## Status / Next steps
- [x] CPU-only GLIM build working
- [x] Verified against official demo dataset
- [x] Custom `aspn_msgs` package reconstructed and verified
- [x] Bag conversion pipeline (IMU type + accel sign + deskewing timestamps)
- [x] Run against Test1 course dataset (partial)
- [ ] Run Test1 to full completion
- [ ] Export trajectory to LatLon CSV
- [ ] Compute RMSE vs GNSS ground truth (`xtrack_global_position_t12.csv`)
- [ ] Export point cloud map as `.pcd`
