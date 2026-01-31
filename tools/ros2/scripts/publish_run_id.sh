#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/humble/setup.bash
set -u

usage() {
  echo "usage: $0 [--keepalive] [--rate <hz>] <run_id>"
}

KEEPALIVE=0
RATE=1

while [ $# -gt 0 ]; do
  case "$1" in
    --keepalive)
      KEEPALIVE=1
      shift
      ;;
    --rate)
      RATE="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      break
      ;;
    -*)
      echo "[ERROR] unknown option: $1"
      usage
      exit 1
      ;;
    *)
      break
      ;;
  esac
done

if [ $# -ne 1 ]; then
  usage
  exit 1
fi

RUN_ID="$1"

if [ "${KEEPALIVE}" = "1" ]; then
  exec ros2 topic pub \
    --rate "${RATE}" \
    --wait-matching-subscriptions 0 \
    --qos-reliability reliable \
    --qos-durability transient_local \
    --qos-history keep_last \
    --qos-depth 1 \
    /mc/run_id std_msgs/msg/String \
    "data: \"${RUN_ID}\"" >/dev/null
fi

ros2 topic pub -1 \
  --wait-matching-subscriptions 0 \
  --qos-reliability reliable \
  --qos-durability transient_local \
  --qos-history keep_last \
  --qos-depth 1 \
  /mc/run_id std_msgs/msg/String \
  "data: \"${RUN_ID}\"" >/dev/null
