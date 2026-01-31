#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/humble/setup.bash
set -u

WS=/ws/ros2_ws
mkdir -p "$WS/src"

cd "$WS"
colcon build --symlink-install
