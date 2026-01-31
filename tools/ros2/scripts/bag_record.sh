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

mkdir -p "$OUT_DIR"

GIT_SHA=$(git -C /ws rev-parse --short HEAD 2>/dev/null || echo unknown)
cat > "$OUT_DIR/meta.txt" <<META
run_id=${RUN_ID}
git_sha=${GIT_SHA}
host=$(hostname)
created_at=${STAMP}
META

if [ -n "${TOPICS:-}" ]; then
  ros2 bag record -o "$OUT_DIR/bag" ${TOPICS}
else
  ros2 bag record -o "$OUT_DIR/bag" --all
fi
