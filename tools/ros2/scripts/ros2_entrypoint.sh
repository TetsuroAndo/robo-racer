#!/usr/bin/env bash
set -euo pipefail

XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR:-/tmp/runtime-root}
export XDG_RUNTIME_DIR

install -d -m 700 "${XDG_RUNTIME_DIR}"

if [ -d /opt/ros/humble ]; then
  export AMENT_PREFIX_PATH="/opt/ros/humble${AMENT_PREFIX_PATH:+:${AMENT_PREFIX_PATH}}"
  export COLCON_PREFIX_PATH="/opt/ros/humble${COLCON_PREFIX_PATH:+:${COLCON_PREFIX_PATH}}"
fi

if [ -d /ws/rpi/ros2_ws/install ]; then
  export AMENT_PREFIX_PATH="/ws/rpi/ros2_ws/install${AMENT_PREFIX_PATH:+:${AMENT_PREFIX_PATH}}"
  export COLCON_PREFIX_PATH="/ws/rpi/ros2_ws/install${COLCON_PREFIX_PATH:+:${COLCON_PREFIX_PATH}}"
fi

exec /ros_entrypoint.sh "$@"
