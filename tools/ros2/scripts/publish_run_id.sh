#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/humble/setup.bash
set -u

if [ $# -ne 1 ]; then
  echo "usage: $0 <run_id>"
  exit 1
fi

RUN_ID="$1"

ros2 topic pub -1 \
  --wait-matching-subscriptions 0 \
  --qos-reliability reliable \
  --qos-durability transient_local \
  --qos-history keep_last \
  --qos-depth 1 \
  /mc/run_id std_msgs/msg/String \
  "data: \"${RUN_ID}\"" >/dev/null
