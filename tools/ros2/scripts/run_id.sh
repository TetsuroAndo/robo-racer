#!/usr/bin/env bash
set -euo pipefail

STAMP=$(date +%Y%m%d_%H%M%S)
SHORT=$(python3 - <<'PY'
import uuid
print(uuid.uuid4().hex[:8])
PY
)

echo "${STAMP}_${SHORT}"
