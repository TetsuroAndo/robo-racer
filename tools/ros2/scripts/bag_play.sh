#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/humble/setup.bash

if [ $# -lt 1 ]; then
  echo "usage: $0 <bag_path> [rate]"
  exit 1
fi

BAG_PATH=$1
RATE=${2:-1.0}

ros2 bag play "$BAG_PATH" --rate "$RATE"
