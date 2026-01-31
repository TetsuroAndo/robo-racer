#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/humble/setup.bash
set -u

RUN_ID=${RUN_ID:-$(python3 - <<'PY'
import uuid
print(uuid.uuid4())
PY
)}
STAMP=$(date +%Y%m%d_%H%M%S)
OUT_DIR=${OUT_DIR:-/ws/training/data/bags/${STAMP}_${RUN_ID}}

PROFILE=${PROFILE:-core}
WAIT_SEC=${WAIT_SEC:-2.0}
PUBLISH_RUN_ID=${PUBLISH_RUN_ID:-1}
RUN_ID_PUB_KEEPALIVE=${RUN_ID_PUB_KEEPALIVE:-1}
RUN_ID_PUB_RATE=${RUN_ID_PUB_RATE:-1}
REQUIRE_AT_LEAST_ONE=${REQUIRE_AT_LEAST_ONE:-1}
RUN_ID_PUB_PID=""

cleanup() {
  if [ -n "${RUN_ID_PUB_PID}" ]; then
    kill "${RUN_ID_PUB_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

mkdir -p "$OUT_DIR"

GIT_SHA=$(git -C /ws rev-parse --short HEAD 2>/dev/null || echo unknown)

DESIRED_TOPICS=()
RECORD_ALL=0
TOPICS_FILE=""

if [ -n "${TOPICS:-}" ]; then
  # NOTE:
  # - ROS2のトピック名にはスペースを含めない前提
  # - TOPICSにはスペース区切りのトピック一覧を渡す想定
  # - グロブ展開を防ぐために、ダブルクォート付きでread -aを使用する
  IFS=' ' read -r -a DESIRED_TOPICS <<< "$TOPICS"
elif [ "${PROFILE}" = "all" ]; then
  RECORD_ALL=1
else
  TOPICS_FILE="/ws/tools/ros2/topics/${PROFILE}.txt"
  if [ ! -f "$TOPICS_FILE" ]; then
    echo "[ERROR] topics profile not found: $TOPICS_FILE"
    echo "        set PROFILE=all or provide TOPICS=..."
    exit 1
  fi
  while IFS= read -r line; do
    line="${line%%#*}"
    line="$(echo "$line" | xargs)"
    [ -z "$line" ] && continue
    DESIRED_TOPICS+=("$line")
  done < "$TOPICS_FILE"
fi

if [ "${PUBLISH_RUN_ID}" = "1" ]; then
  if [ "${RUN_ID_PUB_KEEPALIVE}" = "1" ]; then
    /ws/tools/ros2/scripts/publish_run_id.sh --keepalive --rate "${RUN_ID_PUB_RATE}" "$RUN_ID" &
    RUN_ID_PUB_PID=$!
    echo "[INFO] keepalive publishing /mc/run_id=${RUN_ID}"
  else
    if /ws/tools/ros2/scripts/publish_run_id.sh "$RUN_ID"; then
      echo "[INFO] published /mc/run_id=${RUN_ID}"
    else
      echo "[WARN] failed to publish /mc/run_id (continuing)"
    fi
  fi
fi

cat > "$OUT_DIR/meta.txt" <<META
run_id=${RUN_ID}
git_sha=${GIT_SHA}
host=$(hostname)
created_at=${STAMP}
profile=${PROFILE}
record_all=${RECORD_ALL}
topics_env=${TOPICS:-}
topics_file=${TOPICS_FILE}
META

if [ "${RECORD_ALL}" = "1" ]; then
  echo "[INFO] ros2 bag record --all -> ${OUT_DIR}/bag"
  ros2 bag record -o "$OUT_DIR/bag" --all
  exit 0
fi

if [ ${#DESIRED_TOPICS[@]} -eq 0 ]; then
  echo "[ERROR] no topics resolved; set TOPICS=... or PROFILE=all"
  exit 1
fi

deadline=$(python3 - <<PY
import time
print(time.time() + float("${WAIT_SEC}"))
PY
)

present_topics=()
while :; do
  available="$(ros2 topic list 2>/dev/null || true)"

  present_topics=()
  for t in "${DESIRED_TOPICS[@]}"; do
    if echo "$available" | grep -Fxq "$t"; then
      present_topics+=("$t")
    fi
  done

  if [ "${REQUIRE_AT_LEAST_ONE}" != "1" ] || [ ${#present_topics[@]} -ge 1 ]; then
    break
  fi

  now=$(python3 - <<PY
import time
print(time.time())
PY
)
  if python3 - <<PY | grep -q true
import sys
now=float("${now}")
dl=float("${deadline}")
print("true" if now>=dl else "false")
PY
  then
    break
  fi
  sleep 0.1
done

if [ ${#present_topics[@]} -eq 0 ]; then
  echo "[WARN] none of desired topics are currently visible; recording will start anyway."
  echo "visible_topics=0" >> "$OUT_DIR/meta.txt"
else
  echo "visible_topics=${#present_topics[@]}" >> "$OUT_DIR/meta.txt"
fi

{
  echo "present_topics_begin"
  for t in "${present_topics[@]}"; do echo "$t"; done
  echo "present_topics_end"
  echo "topics_resolved_begin"
  for t in "${DESIRED_TOPICS[@]}"; do echo "$t"; done
  echo "topics_resolved_end"
} >> "$OUT_DIR/meta.txt"

echo "[INFO] ros2 bag record topics -> ${OUT_DIR}/bag"
ros2 bag record -o "$OUT_DIR/bag" "${DESIRED_TOPICS[@]}"
