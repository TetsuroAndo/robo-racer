#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/humble/setup.bash

WS=/ws/ros2_ws
mkdir -p "$WS/src"

cd "$WS"
colcon build --symlink-install
