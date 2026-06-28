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

Valid stages: `timestamps`, `odometry`, `align`, `velocity`, `evaluate`. They are
sequential — each stage reads the previous stage's output from `paths.output_dir`,
so a stage can be re-run alone as long as its inputs already exist. With
`lidar.source: bag` the `timestamps` stage is optional (it only builds the `.laz`
manifest needed by `lidar.source: laz`).

Process a **range of scans** with `--frames START END` (closed range, `END`
inclusive), e.g. `--frames 1000 2000`. This overrides `run.frame_start` /
`run.frame_end` in the config; the legacy `run.max_frames` (first-N-scans cap) is
still honoured when `frame_end` is null. Test1 has **8,194** scans (Test2:
6,008); bounds are clamped to what exists. A full run is ~5–7 min in pure Python.

## Pipeline stages

```
bag / .laz ─▶ timestamps ─▶ odometry ─▶ align ─▶ velocity ─▶ evaluate
                            (KISS-ICP)   (georef)  (NED)      (RMSE + plot)
```

1. **timestamps** — pair each `.laz` scan with its bag-recorded timestamp
   → `scan_manifest.csv` (only needed for `lidar.source: laz`).
2. **odometry** — run the KISS-ICP package over the scans, seeded at
   the first GNSS ground-truth point → `poses_local.csv`, `map_local.pcd`.
3. **align** — start-anchored SE(3) georeference of the trajectory to GNSS ENU,
   re-expressed as lat/lon → `trajectory_latlon.csv`, `alignment_origin.yaml`.
4. **velocity** — smoothed finite-difference of the trajectory → `velocity_ned.csv`.
5. **evaluate** — re-match to GNSS, compute RMSE/mean/max error, plot trajectory +
   error-over-time → `error_evaluation.png`, `error_metrics.csv`.

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
| `align.py` | `anchored_alignment` (pin the start to the first GT point, fit rotation about it — error 0 at t=0), `umeyama_alignment` (global best-fit alternative), `nearest_time_match`, `align_and_georeference`. |
| `velocity.py` | `compute_ned_velocity` — Savitzky-Golay-smoothed finite differences of ENU position → NED velocity. |
| `evaluate.py` | `evaluate_against_ground_truth` — RMSE/mean/max vs GNSS, trajectory + error plot. |
| `attitude.py` | **Optional** (`lidar.imu_deskew`). `AttitudeDeskewer`/`load_attitude_deskewer` — reads `/fmu/out/vehicle_attitude`, SLERP-interpolates, and rotation-deskews each sweep to the orientation at the sweep end. Replaces only the *rotational* part of KISS-ICP's constant-velocity deskew; off by default. |

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
> rotation is a constant and is absorbed by the start-anchored alignment (RMSE
> unchanged, ±0.03 m). It only has a real effect when combined with the PX4
> attitude — and there the default FLU→FRD guess **does not help**: it makes the
> attitude-deskew worse on Test1 (RMSE 3.08 m vs 2.01 m without it). That means
> the true Ouster↔Pixhawk extrinsic differs from the convention flip and would
> need calibration before LiDAR+attitude fusion is trustworthy. Until then, keep
> `imu_deskew` off (or supply a calibrated `extrinsic_rpy_deg`).

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

## Utility scripts (`scripts/`)

All are run from the repo root and read `config.yaml` unless noted. Outputs go
to `outputs/` (plots) or alongside the source (extractions).

| Script | What it does | Example |
| --- | --- | --- |
| `inspect_bag.py` | List a bag's topics, message types, counts, and time span (no ROS2 needed). Run first to sanity-check topic names. | `python scripts/inspect_bag.py ./data/rosbag` |
| `extract_ground_truth.py` | Crop the combined GNSS CSV to the configured run window (on `timestamp_sample`) → per-run ground-truth CSV. | `python scripts/extract_ground_truth.py` |
| `extract_attitude.py` | Extract PX4 `/fmu/out/vehicle_attitude` quaternions over the run window (±margin) → `data/imu_attitude_<run>.csv`. Standalone inspection/export; the `imu_deskew` path reads attitude straight from the bag and does not need this. | `python scripts/extract_attitude.py --margin 1.0` |
| `plot_ground_truth.py` | Plot the GNSS ground-truth trajectory (ENU bird's-eye + components). `--full` plots both datasets. Mark positions at given times with `--times "t1 t2"` (seconds since run start, or absolute epoch). | `python scripts/plot_ground_truth.py --times "100 300 500"` |
| `plot_headings.py` | Overlay IMU-heading (blue) vs GNSS-course (red) arrows along the GNSS trajectory every `--interval` s — shows where the platform *points* vs where it *goes*. | `python scripts/plot_headings.py --interval 20` |
| `plot_scans.py` | Plot two scans top-down, raw vs de-registered, and print the before/after overlap — a diagnostic for whether the clouds carry ego-motion. `--box MIN MAX` keeps only points with `\|x\|` AND `\|y\|` in `[MIN, MAX]` (the four corner regions). Uses `sensys_slam/deregister.py` (a diagnostic-only module, not part of the pipeline). | `python scripts/plot_scans.py 1000 1500 --box 5 20` |
| `read_first_gps.py` | Decode and print the first `/fmu/out/vehicle_gps_position` (raw GPS) message. Inspection only — this is the *noisy* raw GNSS, not the ground truth. | `python scripts/read_first_gps.py` |
| `imu_speed.py` | Report the IMU-based speed at a given LiDAR frame — the PX4 EKF velocity (IMU-propagated, GPS-corrected) from `vehicle_local_position` (default) or `vehicle_odometry`, read at that frame's time. Needs the `px4_msgs` clone (`--px4-msgs-dir`, default `./px4_msgs`). | `python scripts/imu_speed.py 1500 --source odometry` |
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
