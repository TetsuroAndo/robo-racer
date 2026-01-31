#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/humble/setup.bash
set -u

if [ "${1:-}" = "--help" ] || [ "${1:-}" = "-h" ]; then
  echo "usage: $0 [--] <command...>"
  echo "  RUN_ID           : optional, force run_id"
  echo "  SESSION_WAIT_SEC : optional, wait before recording (default 1.0)"
  echo "  PUBLISH_RUN_ID   : optional, 1 to publish /mc/run_id (default 1)"
  echo "  SESSION_CMD      : optional, command string to run (bash -lc)"
  exit 0
fi

if [ "${1:-}" = "--" ]; then
  shift
fi

RUN_ID=${RUN_ID:-$(python3 - <<'PY'
import uuid
print(uuid.uuid4())
PY
)}
export RUN_ID

SESSION_WAIT_SEC=${SESSION_WAIT_SEC:-1.0}
PUBLISH_RUN_ID=${PUBLISH_RUN_ID:-1}
SESSION_CMD=${SESSION_CMD:-}

if [ "${PUBLISH_RUN_ID}" = "1" ]; then
  if /ws/tools/ros2/scripts/publish_run_id.sh "$RUN_ID"; then
    echo "[INFO] published /mc/run_id=${RUN_ID}"
  else
    echo "[WARN] failed to publish /mc/run_id (continuing)"
  fi
fi

if [ $# -gt 0 ] && [ -n "${SESSION_CMD}" ]; then
  echo "[ERROR] both command args and SESSION_CMD are set"
  exit 1
fi

SESSION_PID=""
SESSION_PGID=""
cleanup() {
  if [ -n "${SESSION_PID}" ]; then
    if [ -n "${SESSION_PGID}" ]; then
      kill -- -"${SESSION_PGID}" 2>/dev/null || true
    fi
    kill "${SESSION_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

start_session_cmd() {
  if command -v setsid >/dev/null 2>&1; then
    RUN_ID="${RUN_ID}" setsid bash -lc "${SESSION_CMD}" &
    SESSION_PID=$!
    SESSION_PGID="${SESSION_PID}"
  else
    echo "[WARN] setsid not found; child processes may remain on exit"
    RUN_ID="${RUN_ID}" bash -lc "${SESSION_CMD}" &
    SESSION_PID=$!
  fi
}

start_session_args() {
  if command -v setsid >/dev/null 2>&1; then
    RUN_ID="${RUN_ID}" setsid -- "$@" &
    SESSION_PID=$!
    SESSION_PGID="${SESSION_PID}"
  else
    echo "[WARN] setsid not found; child processes may remain on exit"
    RUN_ID="${RUN_ID}" "$@" &
    SESSION_PID=$!
  fi
}

if [ -n "${SESSION_CMD}" ]; then
  start_session_cmd
elif [ $# -gt 0 ]; then
  start_session_args "$@"
fi

if [ "${SESSION_WAIT_SEC}" != "0" ]; then
  sleep "${SESSION_WAIT_SEC}"
fi

if [ -n "${SESSION_PID}" ] && ! kill -0 "${SESSION_PID}" 2>/dev/null; then
  echo "[ERROR] session command exited before recording started"
  exit 1
fi

exec /ws/tools/ros2/scripts/bag_record.sh
