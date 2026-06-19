# Agent handoff — Sensys LiDAR SLAM (GLIM on amd64 + GitHub push)

You are picking up a LiDAR-SLAM positioning project. This file is self-contained:
read it top to bottom and you can continue without prior context.

---

## 1. Goal & deliverables (from `DESCRIPTION.md`)

Produce, for the **Test1** run of the Sensys dataset:
1. **2D trajectory (lat/lon) + NED velocity** — CSV.
2. **3D point-cloud map** — `.pcd`/`.ply`.
3. **Error plot vs GNSS ground truth + RMSE.**

`DESCRIPTION.md` recommends SLAM toolboxes incl. **koide3** (GLIM). The baseline
KISS-ICP attempt gives **~77 m RMSE, which is accumulated global drift** (not a
local-accuracy issue). GLIM (tightly-coupled LiDAR-IMU, factor-graph + loop
closure) is expected to cut that substantially.

## 2. Repo & branches

Local repo: `/home/armin/Experiments/sensys/sensys_slam_project`
Remote (to push): `git@github.com:Armin-Ebrahimi-Saba/LIDAR-based-Positioning.git`

- **`master`** — original KISS-ICP pipeline (~77 m). Untouched baseline.
- **`LIO-SAM`** — complete LIO_SAM_6AXIS (ROS1/Docker) pipeline, ~90% run. See
  `LIO_SAM_NOTES.md` on that branch for status + how to finish. Kept for
  comparison; not the current focus.
- **`glim`** — branched from `master` (clean KISS-ICP base). **This is where the
  GLIM work goes.** Currently only contains this handoff; KISS-ICP files still
  need trimming (see §6).

## 3. The data (verified facts — trust these)

ROS2 bag: `data/rosbag/` (sqlite3 db3). Ground truth:
`data/xtrack_gnss_corrected/xtrack_global_position_t12.csv` (cols
`timestamp,lat,lon,alt,...`, Unix-epoch seconds). `data/` is gitignored (36 GB).

Key topics:
| Topic | Type | Notes |
|---|---|---|
| `/ouster/points` | `sensor_msgs/PointCloud2` | 8194 msgs ~10 Hz. **Unorganized** (width≈36k,height=1), fields `x,y,z,intensity,nearir,timeoffset`. **Contains NaNs** (`is_dense=false`). No `ring`. **GLIM doesn't need ring/time** — can pass through as-is. |
| `/ouster/imu_meas` | `aspn_msgs/msg/MeasurementIMU` | 81945 msgs ~100 Hz. **CUSTOM type, no message definition anywhere** → cannot deserialize normally. Decode from raw CDR (see below). |
| `/ouster/imu_att` | (custom) | **DEAD** — identity quaternions. Do not use. |

**IMU decode (little-endian CDR, verified, mean|accel|≈9.84 m/s²):**
- stamp: `sec=int32@4`, `nsec=uint32@8`
- linear_acceleration (specific force, m/s²): `3×float64 @ offset 68`
- angular_velocity (rad/s): `3×float64 @ offset 92`
- These are rates/specific-force (NOT deltas) → directly usable as
  `sensor_msgs/Imu`. Note **az ≈ −9.6 at rest** (tilted/inverted mount; standard
  `sensor_msgs/Imu` reports specific force, so this is fine — GLIM estimates the
  gravity direction itself).

**Test1 time window (Unix s):** `1780397390.972` → `1780398213.329`.

Reference Python decode (the LIO-SAM branch's `scripts/build_ros1_bag.py` has a
working `decode_imu`; reuse the offsets above):
```python
import struct, numpy as np
sec  = struct.unpack_from("<i", raw, 4)[0]
nsec = struct.unpack_from("<I", raw, 8)[0]
accel = np.array(struct.unpack_from("<ddd", raw, 68))   # m/s^2
gyro  = np.array(struct.unpack_from("<ddd", raw, 92))   # rad/s
```
Read the bag with `rosbags` (already in `.venv`): `AnyReader([Path("data/rosbag")],
default_typestore=get_typestore(Stores.LATEST))`. The custom IMU type is NOT in
the typestore, so read its `rawdata` bytes directly and decode by offset.

## 4. CRITICAL constraint: GLIM Docker is amd64-only

This host is **aarch64 (GB10)**. The prebuilt GLIM images
(`koide3/glim_ros2:{jazzy, jazzy_cuda12.5, jazzy_cuda13.1, humble, humble_cuda12.2}`)
have **only amd64 manifests** — confirmed via `docker manifest inspect --verbose`.
Per the user's instruction, **implement and run GLIM on an amd64 machine** using
these prebuilt images. (On arm64 the alternative is koide3's APT PPA, which does
support arm64 — `curl -s https://koide3.github.io/ppa/setup_ppa.sh | sudo bash`
then `apt install` the glim ros2 package — but that's NOT the chosen path here.)

The **ROS2 bag conversion (§5.1) is pure Python and runs anywhere**, so build the
bag on this machine; run GLIM itself on amd64.

## 5. GLIM implementation plan (amd64)

### 5.1 ROS2 bag converter — `scripts/build_ros2_bag.py` (pure Python, do first)
Read `data/rosbag` (ROS2), write a NEW ROS2 bag (`outputs/test1_glim/test1_glim`)
cropped to the Test1 window, with two topics:
- `/points` — `sensor_msgs/PointCloud2`, **pass through** `/ouster/points`
  unchanged (GLIM handles unorganized + NaN clouds; re-serialize, or copy the raw
  CDR since PointCloud2 CDR is distro-stable).
- `/imu` — `sensor_msgs/Imu` built from the decoded aspn accel+gyro,
  `orientation_covariance[0] = -1` (no orientation).
Use `rosbags.rosbag2.Writer` + `get_typestore(Stores.ROS2_JAZZY)` (storage
sqlite3 is fine; GLIM/Jazzy reads it). Set BOTH topics' header stamps from the
bag-recorded time (one consistent clock) — same approach as the LIO-SAM
converter. Add a `--verify` mode (print gravity magnitude, msg counts) and a
`--max-scans` smoke option. Bag size ≈ original points (~6–7 GB).

### 5.2 GLIM config (mount into the container)
- `config/config_ros.json` — set `imu_topic:"/imu"`, `points_topic:"/points"`.
- `config/config_sensors.json` — `T_lidar_imu` (LiDAR↔IMU extrinsic). No
  published extrinsic exists in this bag (`/tf_static` has only camera frames);
  start with identity `[0,0,0, 0,0,0,1]`. GLIM estimates gravity, so the −Z
  gravity reading needs no manual flip (unlike LIO-SAM). If results look wrong,
  this extrinsic is the first knob.
- Copy GLIM's default configs out of the image first
  (`/root/ros2_ws/install/glim/share/glim/config` or similar) and edit, so all
  other params keep sane defaults.

### 5.3 Run (on amd64)
```bash
docker run --rm -it -v $PWD:/work [--gpus all]  koide3/glim_ros2:jazzy \
  bash -lc "source /opt/ros/jazzy/setup.bash && source /root/ros2_ws/install/setup.bash && \
            ros2 run glim_ros glim_rosbag /work/outputs/test1_glim/test1_glim \
              --ros-args -p config_path:=/work/config"
```
Use the CPU `jazzy` tag first (robust); switch to `jazzy_cuda13.1` + `--gpus all`
for speed on a Blackwell GPU. GLIM auto-manages playback speed. Disable the
viewer for headless (`--ros-args -p enable_viewer:=false` or set in config).
Output: TUM trajectories in the dump dir (default `/tmp/dump`): use
`traj_lidar.txt` (with loop closure). Set the dump path to `/work/outputs/test1_glim/`.
Export the map (offline viewer `File→Save→Export Points`, or the dump's points).

### 5.4 Feed into the existing evaluator (reused, unchanged)
The downstream stages are odometry-engine agnostic — they only need
`poses_local.csv` (cols `timestamp,x,y,z,qx,qy,qz,qw`) + the GNSS CSV.
- Write `scripts/tum_to_poses.py`: TUM (`t tx ty tz qx qy qz qw`) → `poses_local.csv`
  into `outputs/test1_glim/`. (TUM timestamps must be Unix-epoch s to match GNSS;
  since the bag uses original stamps, they will be.)
- `config_glim.yaml` = copy of the LIO-SAM branch's `config_liosam.yaml` with
  `output_dir: ./outputs/test1_glim` and the `plot_title` changed.
- Run `python run_pipeline.py --config config_glim.yaml` (stages align→velocity→
  evaluate). Trim `run_pipeline.py` to those 3 stages (see §6).
- Deliverables land in `outputs/test1_glim/`: `trajectory_latlon.csv`,
  `velocity_ned.csv`, `map_local.pcd/.ply`, `error_evaluation.png`,
  `error_metrics.csv`. **Compare RMSE to the ~77 m KISS-ICP baseline.**

## 6. Branch cleanup (do on `glim`)
Mirror what was done on the LIO-SAM branch:
- Delete KISS-ICP-only modules: `sensys_slam/{odometry,lidar_io,timestamps,
  imu_assist,attitude,px4_odometry}.py`, `scripts/diagnose_frame.py`, `config.yaml`.
- Keep reused: `sensys_slam/{align,velocity,evaluate,groundtruth,geo,__init__}.py`.
- Trim `run_pipeline.py` to `["align","velocity","evaluate"]` (remove the
  timestamps/odometry imports+stages). A trimmed version exists on the `LIO-SAM`
  branch — copy it and change the default `--config` to `config_glim.yaml`.
- Verify: `python -c "from sensys_slam import align,velocity,evaluate,groundtruth,geo"`.

## 7. GitHub push — BLOCKED on auth (resolve first if asked)
Remote `origin` is set to the SSH URL above. `git ls-remote origin` fails with
**`Permission denied (publickey)`** — no SSH key on this machine is authorized
for the account (host key for github.com was already added to known_hosts).
To push, the user must either:
- add an SSH public key (`~/.ssh/id_ed25519.pub`, create with `ssh-keygen` if
  none) to GitHub → then `git push -u origin master LIO-SAM glim`; **or**
- authenticate `gh` (`gh auth login`) / use an HTTPS PAT and switch the remote to
  `https://github.com/Armin-Ebrahimi-Saba/LIDAR-based-Positioning.git`.
Push all three branches. `data/` and `outputs/` are gitignored (won't upload the
36 GB dataset / 7 GB bags).

## 8. What's already committed
- `LIO-SAM` branch: full LIO_SAM_6AXIS pipeline + `LIO_SAM_NOTES.md` (commit
  `0621aeb`).
- `glim` branch: clean KISS-ICP base + this file (commit after you read this).
- Memories worth knowing (in the user's Claude memory): the IMU decode offsets
  and the "77 m = global drift" finding are recorded there too.
