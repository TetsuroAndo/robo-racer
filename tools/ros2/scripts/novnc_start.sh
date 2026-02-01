#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/humble/setup.bash
set -u

NOVNC_DISPLAY=${NOVNC_DISPLAY:-:1}
NOVNC_RESOLUTION=${NOVNC_RESOLUTION:-1280x800x24}
NOVNC_PORT=${NOVNC_PORT:-6080}
NOVNC_VNC_PORT=${NOVNC_VNC_PORT:-5900}
NOVNC_WEB_DIR=${NOVNC_WEB_DIR:-/usr/share/novnc}

export DISPLAY=${NOVNC_DISPLAY}
export LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE:-1}
export QT_XCB_GL_INTEGRATION=${QT_XCB_GL_INTEGRATION:-none}
export XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR:-/tmp/runtime-root}

mkdir -p "${XDG_RUNTIME_DIR}"
chmod 700 "${XDG_RUNTIME_DIR}"

cleanup() {
  for pid in ${WEBSOCKIFY_PID:-} ${X11VNC_PID:-} ${FLUXBOX_PID:-} ${XVFB_PID:-}; do
    if [ -n "${pid}" ]; then
      kill "${pid}" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT INT TERM

Xvfb "${NOVNC_DISPLAY}" -screen 0 "${NOVNC_RESOLUTION}" -ac +extension GLX +render -noreset &
XVFB_PID=$!

fluxbox -display "${NOVNC_DISPLAY}" >/tmp/fluxbox.log 2>&1 &
FLUXBOX_PID=$!

x11vnc -display "${NOVNC_DISPLAY}" -rfbport "${NOVNC_VNC_PORT}" -forever -shared -nopw -quiet &
X11VNC_PID=$!

websockify --web="${NOVNC_WEB_DIR}" "${NOVNC_PORT}" "localhost:${NOVNC_VNC_PORT}" >/tmp/novnc.log 2>&1 &
WEBSOCKIFY_PID=$!

echo "[INFO] noVNC ready: http://localhost:${NOVNC_PORT}/vnc.html"

action=("${@}")
if [ ${#action[@]} -eq 0 ]; then
  action=(bash)
fi

exec "${action[@]}"
