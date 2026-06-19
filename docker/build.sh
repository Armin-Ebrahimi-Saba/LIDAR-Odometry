#!/usr/bin/env bash
# Build the LIO_SAM_6AXIS ROS1 Noetic image (arm64). One-time, ~20-40 min
# (GTSAM is compiled from source). Re-run only if docker/Dockerfile changes.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
docker build -t liosam6axis:noetic -f "$HERE/Dockerfile" "$HERE"
echo "Built image: liosam6axis:noetic"
