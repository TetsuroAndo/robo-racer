#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/humble/setup.bash
set -u

if [ $# -lt 1 ]; then
  echo "usage: $0 <bag_path> [rate]"
  exit 1
fi

BAG_PATH=$1
RATE=${2:-1.0}

ros2 bag play "$BAG_PATH" --rate "$RATE" --clock
