#!/usr/bin/env bash
set -euo pipefail

RPI_HOST=${RPI_HOST:-}
RPI_USER=${RPI_USER:-pi}
PORT=${PORT:-22}
RUN_ID=${RUN_ID:-}
SRC_PATH=${SRC_PATH:-}
DEST_DIR=${DEST_DIR:-./training/data/bags}

if [ -z "$RPI_HOST" ]; then
  echo "Error: RPI_HOST is required. Example: RPI_HOST=100.102.92.54"
  exit 1
fi

if [ -z "$RUN_ID" ] && [ -z "$SRC_PATH" ]; then
  echo "Error: RUN_ID or SRC_PATH is required."
  echo "  RUN_ID example: RUN_ID=20240205_123456_abcd"
  echo "  SRC_PATH example: SRC_PATH=/ws/training/data/bags/<run_id>"
  exit 1
fi

if [ -z "$SRC_PATH" ]; then
  SRC_PATH="/ws/training/data/bags/${RUN_ID}"
fi

mkdir -p "$DEST_DIR"

echo "[INFO] scp -r -P $PORT ${RPI_USER}@${RPI_HOST}:${SRC_PATH} ${DEST_DIR}/"
scp -r -P "$PORT" "${RPI_USER}@${RPI_HOST}:${SRC_PATH}" "${DEST_DIR}/"
