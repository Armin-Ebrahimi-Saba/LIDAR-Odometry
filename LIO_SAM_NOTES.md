# LIO-SAM branch — progress & how to continue

Status as of 2026-06-19. This branch runs **LIO_SAM_6AXIS** (JokerJohn fork,
ROS1 Noetic) on the Sensys Test1 data via Docker, then reuses the existing
`align`/`velocity`/`evaluate` stages to georeference and score against GNSS.

## Where I left off (≈90% done)

LIO-SAM **runs and initializes** on a 60-scan smoke bag: it produces odometry
(`outputs/test1_liosam/lio_odom.bag`) and the save_map service completes. The
**one remaining fix** is making the saved map land on the host mount (see below),
then doing the full 14-minute run and the RMSE comparison.

## What works (verified)

- **IMU decode** (`scripts/build_ros1_bag.py`): `/ouster/imu_meas` is the custom
  `aspn_msgs/MeasurementIMU` with no message def. Decoded from raw CDR bytes
  (little-endian): stamp `sec=int32@4`, `nsec=uint32@8`; accel (specific force,
  m/s²) = `3×float64@68`; gyro (rad/s) = `3×float64@92`. mean|accel|≈9.84 ✓.
- **Cloud conversion**: `/ouster/points` is unorganized, no `ring`, has NaNs.
  Converter drops NaNs, synthesizes `ring` (32 beams via elevation binning) and
  `time` (`timeoffset` ms→s), emits Velodyne `PointXYZIRT` layout (point_step=32).
- **Full ROS1 bag built**: `outputs/test1_liosam/test1_lio.bag` (7.4 GB, 8194
  clouds + 81944 IMU) with topics `/points_raw`, `/imu_raw`.
- **Docker image** `liosam6axis:noetic` builds (Noetic + GTSAM 4.0.3 from source
  with `GTSAM_BUILD_UNSTABLE=ON` — required by `imuPreintegration`; only the
  `lio_sam_6axis` package is built via `CATKIN_WHITELIST_PACKAGES`, skipping the
  OGRE-dependent `rviz_satellite`).

## Gotchas already solved (don't re-discover)

1. `useGPS: true` (vlp default) makes `mapOptimization` gate init on a synced GPS
   msg and spin forever on `"sysyem need to be initialized"`. **Set `useGPS:false`**
   (done in `docker/lio_sam_overrides.yaml`) to take the IMU-only init branch.
2. `save_map.srv` request is **empty** (all fields commented out). Call it as
   `rosservice call /lio_sam_6axis/save_map "{}"` (done).
3. Raw accel reads gravity as **−9.6 on Z**; LIO-SAM needs +g on Z or it diverges.
   `extrinsicRot`/`extrinsicRPY` set to 180° about X (flip Y,Z) in the overrides.
   If heading/map look wrong, try 180° about Y: `[-1,0,0, 0,1,0, 0,0,-1]`.

## THE remaining fix (do this first)

The map is NOT saved via `savePCDDirectory`. This fork's `DataSaver` writes to
`save_directory = <saveDirectory> + <sequence> + "/"`, read as **global** params
(no `lio_sam_6axis/` prefix), defaulting to `/Downloads/LOAM/map/` (inside the
container, lost on `--rm`). The single-arg `savePointCloudMap` writes
**`globalmap_lidar_feature.pcd`** there.

Fix: add global params to `docker/lio_sam.launch` so the map lands on the mount:
```xml
<param name="saveDirectory" value="/work/outputs/test1_liosam/" />
<param name="sequence" value="lio" />
```
→ map at `outputs/test1_liosam/lio/globalmap_lidar_feature.pcd`, plus TUM
trajectory `optimized_odom_tum.txt` in the same dir (note: `DataSaver` does
`rm -r` on that dir at node startup, so keep `sequence` a dedicated subdir).
Then update `run_liosam_pipeline.sh` `run_poses()` to copy
`.../lio/globalmap_lidar_feature.pcd` → `map_local.pcd`.

## Then: full run + evaluate

```bash
docker/build.sh                         # one-time (~done)
./run_liosam_pipeline.sh convert        # build the ROS1 bag (~done: test1_lio.bag)
./run_liosam_pipeline.sh slam           # docker run, ~14 min at rate 1.0
./run_liosam_pipeline.sh poses          # lio_odom.bag -> poses_local.csv + map
./run_liosam_pipeline.sh evaluate       # align + velocity + RMSE plot
```
Compare RMSE in `outputs/test1_liosam/error_metrics.csv` against the KISS-ICP
baseline (~77 m global drift, see memory `test1-rmse-is-global-drift`).

## File map (this branch)

- `scripts/build_ros1_bag.py` — ROS2→ROS1 converter (IMU decode + ring/time).
- `scripts/run_lio_sam.sh` — in-container: launch, play, record odom, save map.
- `scripts/odom_to_poses.py` — odom bag → `poses_local.csv`.
- `docker/{Dockerfile,build.sh,run_in_docker.sh,lio_sam.launch,lio_sam_overrides.yaml}`
- `config_liosam.yaml`, `run_liosam_pipeline.sh`
- Reused downstream: `sensys_slam/{align,velocity,evaluate,groundtruth,geo}.py`,
  `run_pipeline.py` (trimmed to align/velocity/evaluate stages).
