#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/humble/setup.bash
set -u

WS=/ws/rpi/ros2_ws
mkdir -p "$WS/src"

cd "$WS"
colcon build --symlink-install --install-base "$WS/install"
