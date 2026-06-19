#!/usr/bin/env bash
# Host-side wrapper: run LIO_SAM_6AXIS in the container with the project root
# bind-mounted at /work. Assumes docker/build.sh has been run and the converted
# bag exists (scripts/build_ros1_bag.py).
#
#   docker/run_in_docker.sh [BAG_INSIDE_CONTAINER] [PLAY_RATE]
set -euo pipefail
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BAG="${1:-/work/outputs/test1_liosam/test1_lio.bag}"
RATE="${2:-1.0}"

docker run --rm -t \
  -v "$PROJECT_ROOT:/work" \
  --shm-size=4g \
  liosam6axis:noetic \
  bash -lc "bash /work/scripts/run_lio_sam.sh '$BAG' '$RATE'"
