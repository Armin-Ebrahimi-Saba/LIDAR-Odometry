#!/usr/bin/env bash
# End-to-end LIO_SAM_6AXIS pipeline for Sensys Test1.
#
# Prereq (one-time): docker/build.sh   (builds the Noetic+GTSAM+LIO_SAM image)
#
# Stages:
#   1. convert  ROS2 bag -> ROS1 bag (/points_raw, /imu_raw)   [pure Python]
#   2. slam     run LIO_SAM_6AXIS in Docker -> lio_odom.bag + GlobalMap.pcd
#   3. poses    lio_odom.bag -> poses_local.csv  (+ map_local.pcd)
#   4. evaluate align to GNSS, velocity, RMSE + plot  [reuses run_pipeline.py]
#
#   ./run_liosam_pipeline.sh [all|convert|slam|poses|evaluate] [PLAY_RATE]
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
PY=.venv/bin/python
CFG=config_liosam.yaml
OUT=outputs/test1_liosam
RATE="${2:-1.0}"
STAGE="${1:-all}"

run_convert() { $PY scripts/build_ros1_bag.py --config "$CFG"; }

run_slam() {
  docker image inspect liosam6axis:noetic >/dev/null 2>&1 || {
    echo "Image liosam6axis:noetic missing -- run docker/build.sh first." >&2; exit 1; }
  docker/run_in_docker.sh /work/$OUT/test1_lio.bag "$RATE"
}

run_poses() {
  $PY scripts/odom_to_poses.py "$OUT/lio_odom.bag" --out "$OUT/poses_local.csv"
  MAP="$OUT/lio/globalmap_lidar_feature.pcd"
  if [[ -f "$MAP" ]]; then
    cp -f "$MAP" "$OUT/map_local.pcd"
    echo "[poses] map_local.pcd <- lio/globalmap_lidar_feature.pcd"
  else
    echo "[poses] WARN: $MAP not found (map not saved)"
  fi
}

run_evaluate() {
  $PY run_pipeline.py --config "$CFG" --stage align
  $PY run_pipeline.py --config "$CFG" --stage velocity
  $PY run_pipeline.py --config "$CFG" --stage evaluate
}

case "$STAGE" in
  convert)  run_convert ;;
  slam)     run_slam ;;
  poses)    run_poses ;;
  evaluate) run_evaluate ;;
  all)      run_convert; run_slam; run_poses; run_evaluate ;;
  *) echo "Unknown stage: $STAGE" >&2; exit 1 ;;
esac
echo "Stage '$STAGE' done. Outputs in $OUT/"
