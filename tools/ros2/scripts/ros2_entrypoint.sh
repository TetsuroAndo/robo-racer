#!/usr/bin/env bash
set -euo pipefail

XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR:-/tmp/runtime-root}
export XDG_RUNTIME_DIR

install -d -m 700 "${XDG_RUNTIME_DIR}"

exec /ros_entrypoint.sh "$@"
