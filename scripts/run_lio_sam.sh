#!/usr/bin/env bash
# Runs INSIDE the liosam6axis:noetic container (project bind-mounted at /work).
# Starts LIO_SAM_6AXIS, plays the converted ROS1 bag on sim time, records the
# optimized odometry, and saves the dense map. Invoked by run_in_docker.sh.
#
#   bash scripts/run_lio_sam.sh [BAG] [PLAY_RATE]
set -euo pipefail

BAG="${1:-/work/outputs/test1_liosam/test1_lio.bag}"
RATE="${2:-1.0}"
OUTDIR=/work/outputs/test1_liosam
ODOM_TOPIC=/lio_sam_6axis/mapping/odometry

# mapOptmization prepends $HOME to savePCDDirectory; point it at the mount.
export HOME=/work
mkdir -p "$OUTDIR/lio_map"

source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

cleanup() {
  set +e
  rosnode kill /odom_recorder 2>/dev/null
  sleep 2
  rosnode kill -a 2>/dev/null
  sleep 1
  [[ -n "${LAUNCH_PID:-}" ]] && kill "$LAUNCH_PID" 2>/dev/null
  [[ -n "${CORE_PID:-}" ]] && kill "$CORE_PID" 2>/dev/null
}
trap cleanup EXIT

roscore & CORE_PID=$!
sleep 4
rosparam set /use_sim_time true

echo "[run] launching LIO_SAM_6AXIS nodes"
roslaunch /work/docker/lio_sam.launch & LAUNCH_PID=$!
sleep 8

echo "[run] recording $ODOM_TOPIC"
rm -f "$OUTDIR/lio_odom.bag"
rosbag record -O "$OUTDIR/lio_odom.bag" "$ODOM_TOPIC" __name:=odom_recorder &
sleep 3

echo "[run] playing $BAG at rate $RATE"
rosbag play --clock -r "$RATE" "$BAG"

echo "[run] playback done; letting optimization settle"
sleep 12

echo "[run] saving map"
rosservice call /lio_sam_6axis/save_map "{}" || \
  echo "[run] WARN: save_map service call failed"
sleep 5

echo "[run] finished. Outputs in $OUTDIR (lio_odom.bag, lio_map/GlobalMap.pcd)"
