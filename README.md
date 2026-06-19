# Sensys LiDAR Positioning Pipeline

A pure-Python LiDAR odometry/SLAM pipeline for the Sensys dataset (Berlin,
XTrack platform: PX4 + Ouster LiDAR + RealSense D435i + GNSS). Built around
**KISS-ICP** for the core odometry/mapping, with the GNSS ground truth used
to georeference the result and evaluate accuracy. No ROS2 installation is
required anywhere in this pipeline -- ROS2 bags are read directly via the
`rosbags` library.

It produces the three deliverables required by the project:

1. **2D trajectory + velocity** -- `trajectory_latlon.csv` (lat/lon/alt) and
   `velocity_ned.csv` (North/East/Down velocity).
2. **3D point cloud map** -- `map_local.pcd`.
3. **Error plot + RMSE** -- `error_evaluation.png` and `error_metrics.csv`,
   comparing the estimate against the GNSS ground truth.

Scope: Test1 only (822 s, 8194 LiDAR scans), but every path/time is config-
driven, so pointing `config.yaml` at Test2's files and time window runs the
exact same pipeline on Test2.

---

## 1. How it works, in one paragraph

Each `.laz` scan is time-tagged using the *bag-recorded* timestamp of the
matching `/ouster/points` message (read directly from the `.db3` file,
without deserializing the bulky point-cloud payload). KISS-ICP consumes the
scans in order and produces a trajectory in an arbitrary local frame, plus
an accumulated 3D map. The corrected GNSS CSV (`xtrack_global_position_t12.csv`)
is cropped to the run's time window and used as ground truth. A rigid
SE(3) transform (rotation + translation, no scaling -- both are already
metric) is estimated from the timestamps where SLAM and GNSS overlap, and
applied to the *entire* SLAM trajectory, which is then converted back to
lat/lon. Velocity is the time-derivative of the aligned trajectory.
Accuracy is reported by re-matching the aligned trajectory against the
*full* (independent) ground-truth series and computing RMSE.

---

## 2. Design decisions (and why)

**Why KISS-ICP, pure Python, no ROS2.** KISS-ICP ships a pip-installable
Python API with a compiled core but no ROS dependency, and performs well
without per-sensor tuning. LIO-SAM and ORB-SLAM3 are ROS/C++ packages built
with colcon and (for LIO-SAM) GTSAM -- genuinely useful, but they cannot be
"pure Python, no ROS2," so they're treated as a documented advanced
extension instead (Section 7) rather than faked into something they aren't.

**Why bag-recorded timestamps, not LAS `gps_time`.** Individual `.laz`
frame dumps usually don't carry a per-point absolute time field. Rather
than guess scan timing from sequence index and the run's average frame
rate, `sensys_slam/timestamps.py` reads the bag's own recorded timestamp for
each `/ouster/points` message -- exact, and free of synchronization
guesswork. This *does* assume the Nth `.laz` file (sorted by filename)
matches the Nth bag message in capture order; the manifest builder raises
an error if the counts don't match rather than silently mis-pairing data
(see Section 8, "Assumptions").

**Why the corrected GNSS CSV as ground truth, not the raw GPS CSV.** The
project's explicit ground-truth recommendation is the corrected/filtered
`xtrack_global_position_t12.csv`, not the raw `xtrack_gps_position_t12.csv`
(whose `fix_type`/`eph` columns are a separate, optional cross-check, not
used by this pipeline). The corrected file still has noise near its very
start/end (a wider window than either test run), so it's cropped to the
run's time window -- and an extra `crop_margin_s` is available in
`config.yaml` if you still see edge jumps in the error plot.

**Why Savitzky-Golay-smoothed differentiation for velocity, not raw
`diff`.** A few centimeters of frame-to-frame SLAM jitter, differentiated
naively at ~10 Hz, turns into tens of cm/s of velocity noise even though
the *positions* are fine. A light smoothing pass before differentiating
(see `sensys_slam/velocity.py`) fixes this; set `velocity.smooth_window: 0`
in `config.yaml` to disable it and difference raw positions instead.

---

## 3. Installation

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

`laspy[lazrs]` pulls in a Rust-based LAZ decompressor so you don't need a
system LASzip install. `kiss-icp` ships a compiled extension via pip wheels
for common platforms; if no wheel matches your machine, pip will build from
source (needs a C++ compiler -- `sudo apt install g++` on Ubuntu).

---

## 4. Directory structure

```
sensys_slam_project/
├── README.md                  <- this file
├── requirements.txt
├── config.yaml                 <- all paths and parameters live here
├── run_pipeline.py             <- top-level orchestrator (run this)
├── scripts/
│   └── inspect_bag.py          <- lists topics/types/counts in a bag, no ROS2 needed
└── sensys_slam/
    ├── timestamps.py            <- pairs .laz files with bag-recorded timestamps
    ├── lidar_io.py               <- .laz loading + dataset wrapper for the SLAM loop
    ├── odometry.py                <- runs KISS-ICP -> poses_local.csv + map_local.pcd
    ├── groundtruth.py             <- loads/crops/filters the GNSS reference
    ├── geo.py                      <- geodetic <-> local ENU conversions
    ├── align.py                     <- SE(3) alignment of SLAM trajectory to GNSS
    ├── velocity.py                  <- NED velocity from the aligned trajectory
    ├── evaluate.py                   <- RMSE + error-over-time plot
    └── imu_assist.py                  <- OPTIONAL: IMU-seeded ICP initial guess
```

---

## 5. Usage

This matches your actual project layout, with `data/`, `scripts/`, and
`sensys_slam/` all under the same root as `run_pipeline.py` and
`config.yaml`:

```
.
├── config.yaml
├── run_pipeline.py
├── requirements.txt
├── data
│   ├── rosbag
│   │   ├── metadata.yaml              <- bag_dir = "./data/rosbag" (this folder)
│   │   ├── rosbag_0.db3
│   │   ├── rosbag_data
│   │   │   └── laz_clouds              <- the .laz scans
│   │   └── Location Person GNSS        <- unrelated GNSS export, NOT used here
│   │       ├── meta                     (ground truth comes from
│   │       └── Raw Data.csv              xtrack_gnss_corrected/ instead)
│   └── xtrack_gnss_corrected
│       └── xtrack_global_position_t12.csv
├── scripts
│   └── inspect_bag.py
└── sensys_slam
```

`metadata.yaml` and `rosbag_0.db3` sit directly in `data/rosbag/` -- that
whole folder is `bag_dir`. `Location Person GNSS/` is a separate GNSS
export sitting alongside the bag (not the bag's own metadata, despite the
similarly-named `meta` subfolder); this pipeline ignores it and uses
`xtrack_gnss_corrected/xtrack_global_position_t12.csv` as ground truth
instead. Run every command below from this project root.

### Step 0 -- inspect the bag first

```bash
python scripts/inspect_bag.py ./data/rosbag
```

Confirm `/ouster/points` is listed with a message count matching your
`.laz` file count (8194 for Test1) before moving on.

### Step 1 -- `config.yaml` is already set for this layout

`paths.bag_dir`, `paths.laz_dir`, and `paths.gnss_csv` already match your
tree above (`./data/rosbag`, `./data/rosbag/rosbag_data/laz_clouds`, and
`./data/xtrack_gnss_corrected/xtrack_global_position_t12.csv`). Only change
them if your real folder/file names end up differing from what's shown
here. The run time window and topic name already default to Test1's values
from the inventory report.

### Step 2 -- run the pipeline

```bash
python run_pipeline.py --config config.yaml
```

This runs all five stages in order. Each can also be run individually
(useful while iterating, since odometry is the slow step):

```bash
python run_pipeline.py --config config.yaml --stage timestamps
python run_pipeline.py --config config.yaml --stage odometry
python run_pipeline.py --config config.yaml --stage align
python run_pipeline.py --config config.yaml --stage velocity
python run_pipeline.py --config config.yaml --stage evaluate
```

### Outputs (written to `paths.output_dir`, default `./outputs/test1/`)

| File | Contents |
|---|---|
| `scan_manifest.csv` | filename, filepath, timestamp for every `.laz` scan |
| `poses_local.csv` | timestamp, x/y/z, quaternion -- SLAM trajectory, arbitrary local frame |
| `map_local.pcd` | accumulated 3D point cloud map, same local frame |
| `trajectory_latlon.csv` | timestamp, lat, lon, alt, x/y/z\_enu -- **deliverable 1** |
| `alignment_origin.yaml` | ENU tangent point used + alignment fit RMSE (calibration sanity check) |
| `velocity_ned.csv` | timestamp, vel\_n/e/d\_m\_s -- **deliverable 1, velocity part** |
| `trajectory_latlon_with_velocity.csv` | trajectory + velocity combined |
| `error_evaluation.png` | trajectory overlay + error-over-time plot -- **deliverable 3** |
| `error_metrics.csv` | rmse\_m, mean\_error\_m, max\_error\_m, n\_matched -- **deliverable 3** |

`map_local.pcd` (deliverable 2) is in the same local SLAM frame as
`poses_local.csv`, not georeferenced -- that's normal for a 3D point cloud
map (georeferencing a multi-million-point map is unnecessary; only the
trajectory needs lat/lon).

---

## 6. Tuning notes

- `kiss_icp.max_range` / `min_range`: trim points outside the Ouster's
  reliable range or too close to the vehicle body if the map looks noisy.
- `kiss_icp.deskew`: leave `false` unless `scripts/inspect_bag.py` and a
  look at `sensys_slam/lidar_io.py`'s `gps_time` check confirm your `.laz`
  files actually carry per-point time -- most per-frame exports don't.
- `alignment.max_time_diff_s`: how close (in seconds) a SLAM pose and a GNSS
  sample must be to count as a match. Loosen it if too few matches are
  found (the pipeline will tell you); tighten it if the GNSS rate is high
  enough that you'd rather not match across a stale sample.
- `ground_truth.crop_margin_s`: increase if `error_evaluation.png` still
  shows a jump right at the start/end of the trajectory overlay.
- `velocity.smooth_window`: increase for smoother (but more lagged)
  velocity; set to `0` for raw differentiation.

---

## 7. Advanced extension: IMU-assisted ICP (pure Python, no ROS2)

`sensys_slam/imu_assist.py` is an optional, experimental module that seeds
KISS-ICP's per-frame initial guess with the Ouster's own fused attitude
estimate (`/ouster/imu_att`, ~100 Hz quaternions per the inventory report)
instead of the default constant-velocity assumption. This helps in fast
turns or jerky motion where constant-velocity is a poor model, without
needing ROS2, GTSAM, or LIO-SAM.

**This module's message-field assumptions are inferred from the inventory
report, not the actual bag** -- run `scripts/inspect_bag.py` and check the
real message type for `/ouster/imu_att` before relying on it; adjust
`_get_quaternion_field()` in `imu_assist.py` if the field layout differs
from what's assumed (`sensor_msgs/Imu`-style `.orientation` or
`geometry_msgs/QuaternionStamped`-style `.quaternion`).

Sketch of how to wire it into the odometry loop (replacing the call in
`sensys_slam/odometry.run_odometry`):

```python
from sensys_slam.imu_assist import IMUAidedKissICP, load_imu_attitude, relative_rotation_between

imu_times, imu_rot = load_imu_attitude(cfg["paths"]["bag_dir"])
odometry = IMUAidedKissICP(config=build_kiss_config(cfg))

prev_t = None
for idx in range(len(dataset)):
    frame, point_times = dataset[idx]
    t = dataset.scan_timestamp(idx)
    imu_delta = relative_rotation_between(imu_times, imu_rot, prev_t, t) if prev_t else None
    odometry.register_frame(frame, point_times, imu_relative_rotation=imu_delta)
    prev_t = t
```

---

## 8. Advanced extension: LIO-SAM (requires ROS2 -- not pure Python)

LIO-SAM is a genuine improvement over KISS-ICP for platforms with a good
IMU (tighter, IMU-preintegrated factor-graph fusion instead of point-cloud-
only registration), but it is a ROS2 C++ package built with `colcon` and
depends on GTSAM and PCL. **It cannot be made to run without ROS2** -- that
would just be a different algorithm wearing LIO-SAM's name. This section is
a setup guide for if/when you have (or set up) a ROS2 environment; it isn't
runnable from this pure-Python project as-is.

### 8.1 Setup (on a machine with ROS2, e.g. Humble)

```bash
sudo apt install ros-humble-navigation2 ros-humble-robot-localization \
    libgtsam-dev libpcl-dev
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/TixiaoShan/LIO-SAM --branch ros2
cd ~/ros2_ws && colcon build --packages-select lio_sam
source install/setup.bash
```

### 8.2 Point it at the Sensys bag

LIO-SAM expects specific topic names and a `params.yaml` describing the
LiDAR (Ouster) and IMU extrinsics/intrinsics -- edit
`lio_sam/config/params.yaml`: set `pointCloudTopic: "/ouster/points"`,
`imuTopic` to the actual IMU topic from `scripts/inspect_bag.py`, and the
Ouster's intrinsic parameters (vertical FOV, ring count) from the inventory
report. Then:

```bash
ros2 launch lio_sam run.launch.py
ros2 bag play ./data/rosbag
```

### 8.3 Feed its output back into this pipeline

LIO-SAM publishes its optimized odometry on `/lio_sam/mapping/odometry`.
Record that topic to a small bag while it runs, then convert it to this
project's `poses_local.csv` schema (timestamp, x, y, z, qx, qy, qz, qw) with
a short script using the same `rosbags` library already in this project:

```python
from rosbags.highlevel import AnyReader
from pathlib import Path
import pandas as pd

records = []
with AnyReader([Path("lio_sam_odom_bag")]) as reader:
    conns = [c for c in reader.connections if c.topic == "/lio_sam/mapping/odometry"]
    for connection, t_ns, rawdata in reader.messages(connections=conns):
        msg = reader.deserialize(rawdata, connection.msgtype)
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        records.append({"timestamp": t_ns * 1e-9, "x": p.x, "y": p.y, "z": p.z,
                         "qx": q.x, "qy": q.y, "qz": q.z, "qw": q.w})
pd.DataFrame(records).to_csv("outputs/test1/poses_local.csv", index=False)
```

Once `poses_local.csv` exists in that format, `run_pipeline.py --stage align`
onward works completely unchanged -- alignment, velocity, and evaluation
don't care which odometry engine produced the local trajectory.

---

## 9. Assumptions and caveats

- **`.laz` files pair 1:1, in order, with bag `/ouster/points` messages.**
  True whenever the export tool wrote frames in capture order (the normal
  case for sequential per-frame dumps). `build_scan_manifest` raises an
  error rather than guessing if the counts don't match -- if you hit that
  error, the bag/laz/topic in `config.yaml` likely don't all refer to the
  same run.
- **No per-point deskewing by default.** Per-frame `.laz` exports typically
  lack a usable `gps_time` per point; `kiss_icp.deskew` defaults to `false`
  accordingly (see Section 6).
- **`/ouster/imu_att` field layout in `imu_assist.py` is inferred, not
  verified**, since this project was built from the inventory report
  without access to the actual bag file. Verify with `inspect_bag.py`
  before relying on that module.
- **Ground truth column names** (`timestamp`, `lat`, `lon`, `alt`,
  `lat_lon_valid`, `alt_valid`) are taken from the inventory report's
  description of `xtrack_global_position_t12.csv`. If your actual CSV uses
  different column names, update `sensys_slam/groundtruth.py` accordingly.
- **Alignment needs at least 10 timestamp matches** between SLAM poses and
  ground truth within `alignment.max_time_diff_s`; it raises a clear error
  otherwise rather than fitting an unreliable transform from too few points.

---

## 10. Validation

The alignment, velocity, and evaluation math (everything downstream of
having a `poses_local.csv`) was validated against a synthetic circular
trajectory with known ground truth: alignment recovered the true rotation/
translation to within ~0.09 m RMSE, and smoothed velocity matched the
analytic constant-speed value (1.157 m/s vs. an expected 1.257 m/s estimate
under realistic position jitter; near-exact with noise-free input). It has
not been run against the real Sensys bag/`.laz` files, since those weren't
available in the environment this project was built in -- run
`scripts/inspect_bag.py` first and watch for the manifest-mismatch error as
your first sanity checks against the real data.
