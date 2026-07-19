# Sensys LiDAR Positioning Pipeline

LiDAR odometry/SLAM on the XTrack dataset (Ouster OS0 + PX4), evaluated against
the filtered GNSS ground truth. See `DESCRIPTION.md` for the task brief.

Run everything from the repository root so the relative paths in `config.yaml`
resolve. The active run window (Test1 vs Test2) and all input/output paths are
configured in [`config.yaml`](config.yaml).

## Preprocessing

### Extract the ground truth for the run

The provided ground truth, `xtrack_global_position_t12.csv`, is PX4's filtered
`vehicle_global_position` (the recommended GNSS reference) and covers **both**
datasets back-to-back. It must be cropped to a single run's time window before
use. This step writes a per-run CSV cropped on `timestamp_sample` (the key
shared with the rosbag), preserving all columns. It reads the run window
straight from `config.yaml`, so switching to Test2 is just a config edit.

```bash
python scripts/extract_ground_truth.py            # uses config.yaml
# options: --config <path>  --output <path>
```

Output (Test1): `data/xtrack_gnss_corrected/xtrack_global_position_t12_test1.csv`
— 4112 rows over 822 s (~5 Hz), all `lat_lon_valid`/`alt_valid`.

## Running the pipeline

From the repository root, with the virtualenv active:

```bash
python run_pipeline.py --config config.yaml              # all stages in order
python run_pipeline.py --config config.yaml --stage odometry   # a single stage
```

Valid stages: `timestamps`, `odometry`, `align`, `velocity`, `evaluate`, `map`,
`map3d`, `colormaps`. They are sequential — each stage reads the previous stage's
output from `paths.output_dir`, so a stage can be re-run alone as long as its
inputs already exist (`--stage X` runs `X` and every stage after it). With
`lidar.source: bag` the `timestamps` stage is optional (it only builds the `.laz`
manifest needed by `lidar.source: laz`).

Process a **range of scans** with `--frames START END` (closed range, `END`
inclusive), e.g. `--frames 1000 2000`. This overrides `run.frame_start` /
`run.frame_end` in the config; the legacy `run.max_frames` (first-N-scans cap) is
still honoured when `frame_end` is null. Test1 has **8,194** scans (Test2:
6,008); bounds are clamped to what exists. A full run is ~5–7 min in pure Python.

## Pipeline stages

```
bag / .laz ─▶ timestamps ─▶ odometry ─▶ align ─▶ velocity ─▶ evaluate ─▶ map ─▶ map3d ─▶ colormaps
                            (KISS-ICP)   (georef)  (NED)      (RMSE)     (OSM)  (3D)    (coloured maps)
```

1. **timestamps** — pair each `.laz` scan with its bag-recorded timestamp
   → `scan_manifest.csv` (only needed for `lidar.source: laz`).
2. **odometry** — run the KISS-ICP package over the scans, seeded at
   the first GNSS ground-truth point → `poses_local.csv`, `map_local.pcd`.
3. **align** — global least-squares (Umeyama) SE(3) georeference of the trajectory
   to GNSS ENU, re-expressed as lat/lon → `trajectory_latlon.csv`, `alignment_origin.yaml`.
4. **velocity** — smoothed finite-difference of the trajectory → `velocity_ned.csv`.
5. **evaluate** — re-match to GNSS, compute RMSE/mean/max error, plot trajectory +
   error-over-time → `error_evaluation.png`, `error_metrics.csv`.
6. **map** — render odometry + GNSS on an OpenStreetMap basemap → `trajectory_map.html`.
7. **map3d** — render the accumulated 3D point cloud interactively → `map_3d.html`.
8. **colormaps** — colour the 3D map by elevation (`map_local_height.pcd`, an
   instant post-process of `map_local.pcd`) and by the LiDAR's per-point return
   (`map_local_intensity.pcd`, which re-reads the bag) → both `.pcd` files.

All outputs land in `paths.output_dir` (default `outputs/test1/`).

## Module reference (`sensys_slam/`)

| Module | Responsibility |
| --- | --- |
| `timestamps.py` | `build_scan_manifest` — pair `.laz` files with bag-recorded `/ouster/points` timestamps. |
| `lidar_io.py` | Scan loaders: `BagScanDataset` (streams `/ouster/points` with per-point sweep time for deskew), `LazScanDataset` (`.laz`). Includes a self-contained numpy `PointCloud2` parser (no `kiss-icp` dependency). Both yield `(timestamp, points, point_times)`. |
| `odometry.py` | Drives the **KISS-ICP package** (PRBonn `kiss-icp`). `build_kiss_config` maps the `kiss_icp` config block → `kiss_icp.config.KISSConfig`; `run_odometry` seeds the engine at the first GT pose, registers each frame scan-to-map, and writes `poses_local.csv` + `map_local.pcd` (accumulating the registered downsamples into a global map, since the package's own local map is pruned to `max_range`). |
| `groundtruth.py` | Load `xtrack_global_position_t12.csv`, crop to the run window on `timestamp_sample`, drop invalid rows. |
| `geo.py` | WGS84 ↔ local ENU conversions (via `pymap3d`). |
| `frames.py` | Body-frame definition. `build_lidar_to_body` returns the LiDAR→Pixhawk-FRD rotation used to express scans in the body frame (`lidar.body_frame`). |
| `align.py` | `umeyama_alignment` (global least-squares SE(3) best fit, scale fixed to 1, optionally eph-weighted), `nearest_time_match` (time-pair SLAM poses to GT samples), `match_weights` (1/eph² inverse-variance weights), `align_and_georeference` (drives the full georeference). |
| `velocity.py` | `compute_ned_velocity` — Savitzky-Golay-smoothed finite differences of ENU position → NED velocity. |
| `evaluate.py` | `evaluate_against_ground_truth` — RMSE/mean/max vs GNSS, trajectory + error plot. |
| `attitude.py` | **Optional** (`lidar.imu_deskew`). `AttitudeDeskewer`/`load_attitude_deskewer` — reads `/fmu/out/vehicle_attitude`, SLERP-interpolates, and rotation-deskews each sweep to the orientation at the sweep end. Replaces only the *rotational* part of KISS-ICP's constant-velocity deskew; off by default. |

## Alignment: georeferencing the trajectory to GNSS

KISS-ICP produces poses in a *local* world frame — metric, but with an arbitrary
initial heading and origin. The `align` stage (`sensys_slam/align.py`,
`align_and_georeference`) ties that track to absolute coordinates by fitting **one
rigid SE(3) transform** onto the GNSS ENU frame. It is a **global least-squares
(Umeyama) best fit, not a start-anchored one**: every matched sample contributes,
so the residual is spread across the whole run rather than forced to zero at the
start. (This is why the stationary start still shows tens of metres of error in
the Results below — a start-anchored fit would instead pin `t = 0` to ~0.)

Steps:

1. **GNSS → ENU.** Ground-truth lat/lon/alt is converted to local ENU metres
   (`geo.py`) about a tangent origin `ref_origin` — the run's first GT sample,
   the same point the odometry was seeded at, so the odometry world frame already
   starts at ENU ≈ (0, 0, 0). The origin is stored in `alignment_origin.yaml`.
2. **Time-match.** Each SLAM pose is paired with its nearest-in-time GT sample by
   binary search (`nearest_time_match`); any pair separated by more than
   `alignment.max_time_diff_s` (0.15 s) is dropped.
3. **Inverse-variance weights.** Each pair is weighted by
   `1 / max(eph, eph_floor)²` from PX4's own horizontal uncertainty `eph`
   (`alignment.eph_weighting`, `eph_floor_m = 0.3`). This down-weights the
   several-metre GNSS wander during EKF initialisation so it can't dominate the fit.
4. **Umeyama SE(3).** Weighted centroids → weighted cross-covariance → SVD yields
   the rotation `R` (with a determinant-sign guard against reflections) and the
   translation `t = μ_dst − R·μ_src`. **Scale is fixed to 1** — the fit is rigid
   and metric-preserving, because LiDAR odometry is already metric; only the
   arbitrary heading and origin need correcting.
5. **Apply & re-project.** `R, t` are applied to the *entire* trajectory, which is
   converted back to lat/lon (`enu_to_geodetic`) → `trajectory_latlon.csv`. The
   reported fit RMSE is the eph-weighted residual — the exact quantity minimised.

Because the fit is global *and* rigid (no per-segment warping, no scale freedom),
it cannot mask odometry drift: fitting one part of the run well necessarily leaves
large residuals wherever the odometry has drifted. That is what makes the
error-over-time curve a fair picture of the odometry rather than an artefact of
the alignment.

## Coordinate frames

Per `DESCRIPTION.md`, the **Pixhawk coordinate system is taken as the body
frame**: FRD — X forward, Y right, Z down. Expressing every sensor in this one
frame is what lets the LiDAR clouds and the PX4 attitude (`vehicle_attitude`,
which is body→NED) be combined consistently.

`sensys_slam/frames.py` builds the LiDAR→body rotation. The Ouster sensor frame
is FLU (X-fwd, Y-left, Z-up); assuming the Ouster is mounted axis-aligned with
the Pixhawk, the default LiDAR→body rotation is the FLU→FRD flip (180° about X).
Enable with `lidar.body_frame: true`; override the rotation with
`lidar.extrinsic_rpy_deg` (intrinsic XYZ degrees) if the true mounting is known.
Only rotation is modelled; the lever-arm translation is unknown and assumed zero.

> **Empirical caveat (this bag).** For *pure* LiDAR odometry the body-frame
> rotation is a constant and is absorbed by the global least-squares alignment (RMSE
> unchanged, ±0.03 m). It only has a real effect when combined with the PX4
> attitude — and there the default FLU→FRD guess **does not help**: it makes the
> attitude-deskew worse on Test1 (RMSE 3.08 m vs 2.01 m without it). That means
> the true Ouster↔Pixhawk extrinsic differs from the convention flip and would
> need calibration before LiDAR+attitude fusion is trustworthy. Until then, keep
> `imu_deskew` off (or supply a calibrated `extrinsic_rpy_deg`).

## How the IMU / attitude is used

The IMU contributes to exactly **one** thing in this pipeline — motion-compensating
(deskewing) each LiDAR sweep — and nothing else. It does **not** feed the pose
estimate, the heading, or the velocity. Those come entirely from KISS-ICP's
scan-to-map registration.

**Mechanism** (`sensys_slam/attitude.py`, enabled by `lidar.imu_deskew`):

- The Ouster spins over ~100 ms per sweep, so points captured early vs. late in
  the sweep are seen from different orientations; on a rotating platform that
  smears the cloud.
- For each sweep `AttitudeDeskewer.deskew` takes the per-point sweep times
  (`timeoffset`), **SLERP-interpolates the measured attitude** to each point's
  timestamp, and rotates every point to the orientation at the *sweep end*.
- This **replaces only the rotational part** of KISS-ICP's own deskew. KISS-ICP
  normally assumes the platform keeps rotating at the rate implied by the last
  two scans (constant velocity); with `imu_deskew` on we use the *actually
  measured* rotation instead, and `kiss_icp.data.deskew` is forced off
  (`run_pipeline.py`). Translation within the sweep is not compensated
  (negligible here at ~0.08 m/sweep), and the LiDAR↔FCU extrinsic is assumed
  identity (a single-sweep rotation is small, so the residual is second order).

**Which data source, and why** — the choice was verified against *this* bag, not
assumed:

| Source (topic) | Rate | In pipeline? | How / why |
|---|---|---|---|
| `/fmu/out/vehicle_attitude` (PX4 EKF fused attitude, body-FRD→NED quaternion) | ~100 Hz | ✅ **the only IMU input** | Drives the deskew. Chosen because it is the real fused attitude: unit-norm, and its **yaw tracks the GNSS course** over the run. px4_msgs is not registered in the bag, so the quaternion is read straight from the CDR payload (`float32[4]` at byte offset 20, PX4 `[w,x,y,z]`) — reverse-engineered and validated before use. |
| `/ouster/imu_att` | — | ❌ rejected | **All-identity for the entire recording** — dead/unusable. This is the "obvious" LiDAR-attitude source and it is broken, which is exactly why the PX4 stream is used instead. |
| `/ouster/imu_meas` (raw Ouster accel + gyro) | 100 Hz | ⚠️ diagnostic only | Used by `scripts/imu_pure_speed.py` for a pure-inertial strapdown speed check — not by the odometry. |
| `/fmu/out/vehicle_local_position` / `vehicle_odometry` (PX4 EKF velocity, IMU-propagated) | — | ⚠️ diagnostic only | Used by `scripts/imu_speed.py` to report EKF speed at a frame — not by the odometry. |

To make the attitude apply cleanly the scans are first rotated into the **Pixhawk
FRD body frame** (`lidar.body_frame`, see "Coordinate frames"), so they share the
PX4 attitude's convention.

> **Where this leaves accuracy.** At this platform's crawl speed the within-sweep
> rotation is small, so deskew is a second-order cleanup — it sharpens each cloud
> but does **not** bound the trajectory drift. The single most valuable fact —
> that `vehicle_attitude`'s yaw already tracks the GNSS course — is currently
> spent only on deskewing, *not* fused into the pose/heading. Wiring that
> attitude (or GNSS) into the pose solution, rather than just the sweep
> correction, is the natural next step against the heading-driven drift reported
> below. (See also the empirical caveat under "Coordinate frames": with the
> uncalibrated FLU→FRD extrinsic guess, `imu_deskew` currently makes Test1
> *slightly worse* — 3.08 m vs 2.01 m on the moving segment — so it is off by
> default until the true extrinsic is calibrated.)

## Known limitation: the stationary start

For the first ~100 s of Test1 the platform is essentially stationary —
consecutive `/ouster/points` scans are identical to ~5 mm (vs ~60 mm once it is
genuinely moving), so there is **no ego-motion for scan matching to recover** and
the trajectory stays put. Meanwhile the GNSS reference drifts ~16 m over the same
window (`eph` 2–3 m, several `lat_lon_reset_counter` increments, `dead_reckoning`
episodes) as the EKF settles. So the apparent error over the opening is dominated
by GNSS startup drift, not odometry error — KISS-ICP's frozen output is correct
for identical inputs.

Tuning (voxel size, range, adaptive threshold) does **not** help here; the data
simply contains no motion. The honest options are to start the run window after
the platform begins moving, or to add a zero-velocity / external-prior step for
the stationary segment.

## Results — accuracy, and why KISS-ICP is not good enough here

Test1 (`outputs/test1/error_metrics.csv`, over 6,519 GNSS-matched samples):

| RMSE | eph-weighted RMSE | mean | max | GNSS `eph` (median / max) |
|---|---|---|---|---|
| **52.46 m** | 47.94 m | 46.75 m | **118.42 m** | 0.55 m / 3.44 m |

On a ~573 m out-and-back that is ~9 % of the path RMSE and ~21 % at worst — far
from usable. The `error_evaluation.png` error-over-time curve (right panel) tells
the story better than the aggregate:

- **t = 0–110 s — pinned at ~65–80 m, but not KISS-ICP's fault.** The platform is
  stationary (see "the stationary start"); consecutive scans are identical, so
  odometry correctly stays put while the GNSS EKF is still settling. This segment
  inflates the raw RMSE and is exactly what the eph-weighting discounts.
- **KISS-ICP *can* track locally.** Once motion begins the error collapses to
  ~10 m at **t ≈ 160 s** and again at **t ≈ 390 s**, and to just **~1–3 m at
  t ≈ 520 s** (the platform is back near the start — a near-loop-closure landing
  in the well-fit origin region of the global alignment). Scan-to-map registration is working
  frame-to-frame.
- **But drift is unbounded, and it runs away on the final leg.** From
  **t ≈ 525 s the error balloons from ~2 m to its 118 m maximum at t ≈ 655 s**
  (end of run) — a monotonic blow-up over the last ~130 s with nothing to arrest
  it. Earlier the same mechanism produces the intermediate peaks (**~72 m at
  t ≈ 310 s**, **~51 m at t ≈ 435 s**): the error is essentially *accumulated
  heading error × distance-from-anchor*, so it shrinks whenever the platform
  passes near the start/turnaround and grows as it moves away. That oscillation
  is the fingerprint of pure dead-reckoning, not measurement noise.

**Why it happens.** KISS-ICP here is pure scan-to-map dead-reckoning with **no
absolute reference** — no GNSS fusion, no IMU-heading fusion, no loop closure. Two
dataset properties make that fatal:

1. **Geometric degeneracy.** The route is a narrow corridor (~223 m × 96 m,
   ~8:1 aspect) and `max_range` is 20 m, so along-track structure is starved —
   the very direction that constrains heading. A small per-scan yaw error is
   unobservable locally and integrates into the large end-of-run offset seen
   above. The trajectory panel shows the symptom: the orange odometry collapses
   the wide GNSS loop onto a nearly straight line.
2. **A starved motion model.** Median speed is ~0.82 m/s (max ~1.96 m/s, ~7 cm per
   scan); ~62 % of scans move less than `min_motion_th` (0.1 m) and ~38 % are
   essentially still (<2 cm). KISS-ICP's constant-velocity prediction and adaptive
   threshold have almost no signal to work with, so registration leans hardest on
   the weakly-constrained corridor geometry precisely when it is least reliable.

The fix is **not** more KISS-ICP tuning (voxel/range/threshold do not create the
missing constraint) but adding an absolute reference — fuse the GNSS, or the PX4
attitude whose yaw already tracks the GNSS course (see "How the IMU / attitude is
used"), into the pose estimate, and/or close the loop at the start/turnaround.

## Utility scripts (`scripts/`)

All are run from the repo root and read `config.yaml` unless noted. Outputs go
to `outputs/` (plots) or alongside the source (extractions).

| Script | What it does | Example |
| --- | --- | --- |
| `inspect_bag.py` | List a bag's topics, message types, counts, and time span (no ROS2 needed). Run first to sanity-check topic names. | `python scripts/inspect_bag.py ./data/rosbag` |
| `extract_ground_truth.py` | Crop the combined GNSS CSV to the configured run window (on `timestamp_sample`) → per-run ground-truth CSV. | `python scripts/extract_ground_truth.py` |
| `extract_attitude.py` | Extract PX4 `/fmu/out/vehicle_attitude` quaternions over the run window (±margin) → `data/imu_attitude_<run>.csv`. Standalone inspection/export; the `imu_deskew` path reads attitude straight from the bag and does not need this. | `python scripts/extract_attitude.py --margin 1.0` |
| `plot_ground_truth.py` | Plot the GNSS ground-truth trajectory (ENU bird's-eye + components). `--full` plots both datasets. Mark positions with `--times "t1 t2"` (seconds since run start, or absolute epoch) or `--frames "f1 f2"` (LiDAR frame indices, mapped to time via `/ouster/points`). Out-of-window marks warn instead of silently snapping to the endpoint. | `python scripts/plot_ground_truth.py --frames "1000 1500"` |
| `plot_headings.py` | Overlay IMU-heading (blue) vs GNSS-course (red) arrows along the GNSS trajectory every `--interval` s — shows where the platform *points* vs where it *goes*. | `python scripts/plot_headings.py --interval 20` |
| `plot_scans.py` | Plot two scans top-down, raw vs de-registered, and print the before/after overlap — a diagnostic for whether the clouds carry ego-motion. `--box MIN MAX` keeps only points with `\|x\|` AND `\|y\|` in `[MIN, MAX]` (the four corner regions). Uses `sensys_slam/deregister.py` (a diagnostic-only module, not part of the pipeline). | `python scripts/plot_scans.py 1000 1500 --box 5 20` |
| `read_first_gps.py` | Decode and print the first `/fmu/out/vehicle_gps_position` (raw GPS) message. Inspection only — this is the *noisy* raw GNSS, not the ground truth. | `python scripts/read_first_gps.py` |
| `imu_speed.py` | Report the IMU-based speed at a given LiDAR frame — the PX4 EKF velocity (IMU-propagated, GPS-corrected) from `vehicle_local_position` (default) or `vehicle_odometry`, read at that frame's time. Needs the `px4_msgs` clone (`--px4-msgs-dir`, default `./px4_msgs`). | `python scripts/imu_speed.py 1500 --source odometry` |
| `imu_pure_speed.py` | **Pure-inertial** speed over a LiDAR frame range: strapdown-integrates only the raw Ouster IMU (`/ouster/imu_meas`, accel+gyro) — no GPS/EKF. Gravity and gyro bias are calibrated on the stationary start (`--calib`), initial velocity assumed 0. Drifts with no aiding; use `--validate-static A B` to quantify the drift on a known-still segment. | `python scripts/imu_pure_speed.py 1000 1500 --validate-static 200 700` |
| `diagnose_frame.py` | Check whether `/ouster/points` is sensor-frame or a fixed world frame (frame_id + centroid shift over ~10 s). | `python scripts/diagnose_frame.py ./data/rosbag` |

## Configuration

Key fields in `config.yaml`:

- `paths.*` — bag, LiDAR `.laz`, GNSS CSV, and output directory.
- `run.start_time` / `run.end_time` — the run window (Test1 and Test2 values
  are both listed in comments; the active pair selects which dataset is used).
- `lidar.source` — `bag` (per-point sweep time, deskew-capable) or `laz`.
- `lidar.body_frame` / `lidar.extrinsic_rpy_deg` — express scans in the Pixhawk
  FRD body frame; `extrinsic_rpy_deg` (intrinsic XYZ degrees, `null` = default
  FLU→FRD flip) sets the LiDAR→body rotation. See "Coordinate frames" above.
- `lidar.imu_deskew` — optional, default `false`. Deskew sweeps with measured PX4
  attitude (`lidar.attitude_topic`) instead of KISS-ICP's constant-velocity
  model; corrects rotation only and forces `kiss_icp.data.deskew` off. Requires
  `lidar.source: bag`.
- `kiss_icp.*` — mirrors KISS-ICP's own config (`data`, `mapping`,
  `registration`, `adaptive_threshold`). `kiss_icp.data.deskew` enables the
  package's constant-velocity deskew (needs `lidar.source: bag`).
- `evaluation.time_tick_s` — on `error_evaluation.png`, both trajectories are
  coloured by time and get a labelled marker every N seconds (`0` = off).

## Outputs / deliverables

A full run writes these to `paths.output_dir` (default
[`outputs/test1/`](outputs/test1/)). The three items required by
[`DESCRIPTION.md`](DESCRIPTION.md) — trajectory + velocity, a 3D point-cloud map,
and an error plot vs. GNSS — map onto the files below.

**Task-3 deliverables**

| Requirement (`DESCRIPTION.md`) | File |
|---|---|
| 2D trajectory (LatLon) | [`outputs/test1/trajectory_latlon.csv`](outputs/test1/trajectory_latlon.csv) |
| Velocity (NED frame) | [`outputs/test1/velocity_ned.csv`](outputs/test1/velocity_ned.csv) |
| Trajectory + velocity, combined | [`outputs/test1/trajectory_latlon_with_velocity.csv`](outputs/test1/trajectory_latlon_with_velocity.csv) |
| 3D point-cloud map (`.pcd`) | [`outputs/test1/map_local.pcd`](outputs/test1/map_local.pcd) |
| Error plot: estimate vs GNSS (RMSE) | [`outputs/test1/error_evaluation.png`](outputs/test1/error_evaluation.png) |
| Error metrics (RMSE / mean / max) | [`outputs/test1/error_metrics.csv`](outputs/test1/error_metrics.csv) |

**Point-cloud maps** (same 3D geometry, different per-point colour)

| File | Colour | Built by |
|---|---|---|
| [`outputs/test1/map_local.pcd`](outputs/test1/map_local.pcd) | none (XYZ only) | `odometry` stage |
| [`outputs/test1/map_local_height.pcd`](outputs/test1/map_local_height.pcd) | by elevation (Z), `turbo` | `colormaps` stage (instant post-process) |
| [`outputs/test1/map_local_intensity.pcd`](outputs/test1/map_local_intensity.pcd) | by LiDAR return strength | `colormaps` stage (re-reads the bag) |

View any of them with:

```bash
python -c "import open3d as o3d; o3d.visualization.draw_geometries([o3d.io.read_point_cloud('outputs/test1/map_local_intensity.pcd')])"
```

**Interactive HTML views**

| File | Contents |
|---|---|
| [`outputs/test1/trajectory_map.html`](outputs/test1/trajectory_map.html) | odometry vs GNSS on an OpenStreetMap basemap |
| [`outputs/test1/map_3d.html`](outputs/test1/map_3d.html) | the accumulated 3D point-cloud map, interactive |
