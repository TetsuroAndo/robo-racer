#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONF_DIR="${SCRIPT_DIR}/daemons"
TEMPLATE="${SCRIPT_DIR}/systemd/service.in"
UNIT_DST_DIR="/etc/systemd/system"

ACTION="${1:-}"
shift || true

NAME_FILTER=""

die(){ echo "ERROR: $*" >&2; exit 1; }
need(){ command -v "$1" >/dev/null 2>&1 || die "command not found: $1"; }
is_root(){ [[ "${EUID:-$(id -u)}" -eq 0 ]]; }

usage() {
  cat <<EOF
Usage:
  sudo ./tools/setupd.sh list
  sudo ./tools/setupd.sh install [--name <daemon>]
  sudo ./tools/setupd.sh uninstall [--name <daemon>]
  sudo ./tools/setupd.sh start|stop|restart|status|logs [--name <daemon>]

Notes:
- Daemon definitions: tools/daemons/*.conf (bash-style key=value)
- Unit template: tools/systemd/service.in
EOF
}

# args
while [[ $# -gt 0 ]]; do
  case "$1" in
    --name) NAME_FILTER="${2:-}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) die "unknown argument: $1" ;;
  esac
done

need systemctl
need sed
need install
need mkdir
need rm

# repo root 推定
if command -v git >/dev/null 2>&1 && git -C "${SCRIPT_DIR}" rev-parse --show-toplevel >/dev/null 2>&1; then
  PROJ_ROOT="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel)"
else
  PROJ_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
fi

require_root() {
  is_root || die "run with sudo (systemd system service install/control)"
}

get_conf_files() {
  [[ -d "${CONF_DIR}" ]] || die "missing conf dir: ${CONF_DIR}"
  local files=()
  while IFS= read -r -d '' f; do files+=("$f"); done < <(find "${CONF_DIR}" -maxdepth 1 -type f -name "*.conf" -print0 | sort -z)
  ((${#files[@]} > 0)) || die "no daemon conf found in ${CONF_DIR}"
  printf "%s\n" "${files[@]}"
}

load_conf() {
  NAME=""; DESC=""; BIN_REL=""; ARGS=""; USER=""; GROUP=""; WORKDIR_REL=""; SUPP_GROUPS=""; ENABLED_BY_DEFAULT="yes"

  # shellcheck disable=SC1090
  source "$1"

  [[ -n "${NAME}" ]] || die "NAME is required in $1"
  [[ -n "${BIN_REL}" ]] || die "BIN_REL is required in $1"
  [[ -n "${USER}" ]] || USER="pi"
  [[ -n "${GROUP}" ]] || GROUP="${USER}"
  [[ -n "${WORKDIR_REL}" ]] || WORKDIR_REL="."
  [[ -n "${DESC}" ]] || DESC="${NAME}"
}

unit_path_for() {
  echo "${UNIT_DST_DIR}/${NAME}.service"
}

bin_abs_for() {
  echo "$(cd "${PROJ_ROOT}" && pwd)/${BIN_REL}"
}

workdir_abs_for() {
  echo "$(cd "${PROJ_ROOT}" && pwd)/${WORKDIR_REL}"
}

should_target() {
  if [[ -z "${NAME_FILTER}" ]]; then
    [[ "${ENABLED_BY_DEFAULT}" != "no" ]]
  else
    [[ "${NAME_FILTER}" == "${NAME}" ]]
  fi
}

render_unit() {
  [[ -f "${TEMPLATE}" ]] || die "missing template: ${TEMPLATE}"
  local unit_out; unit_out="$(unit_path_for)"
  local bin_abs; bin_abs="$(bin_abs_for)"
  local workdir_abs; workdir_abs="$(workdir_abs_for)"

  [[ -x "${bin_abs}" ]] || die "binary not found or not executable: ${bin_abs}"

  local exec_start="${bin_abs}"
  if [[ -n "${ARGS}" ]]; then
    exec_start="${exec_start} ${ARGS}"
  fi

  local supp_line=""
  if [[ -n "${SUPP_GROUPS}" ]]; then
    supp_line="SupplementaryGroups=${SUPP_GROUPS}"
  else
    supp_line="# (no supplementary groups)"
  fi

  sed \
    -e "s|@DESC@|${DESC}|g" \
    -e "s|@USER@|${USER}|g" \
    -e "s|@GROUP@|${GROUP}|g" \
    -e "s|@WORKDIR@|${workdir_abs}|g" \
    -e "s|@EXEC_START@|${exec_start}|g" \
    -e "s|@SUPP_GROUPS_LINE@|${supp_line}|g" \
    "${TEMPLATE}" > "${unit_out}"

  chmod 0644 "${unit_out}"
  echo "Generated: ${unit_out}"
}

install_all() {
  require_root
  mkdir -p "${UNIT_DST_DIR}"

  local any=0
  while IFS= read -r conf; do
    load_conf "${conf}"
    if should_target; then
      render_unit
      any=1
    fi
  done < <(get_conf_files)

  [[ "${any}" -eq 1 ]] || die "no target daemons matched (name filter or all disabled)"

  systemctl daemon-reload

  while IFS= read -r conf; do
    load_conf "${conf}"
    if should_target; then
      systemctl enable --now "${NAME}.service"
      echo "Enabled+Started: ${NAME}.service"
    fi
  done < <(get_conf_files)
}

uninstall_all() {
  require_root
  local any=0

  while IFS= read -r conf; do
    load_conf "${conf}"
    if should_target; then
      any=1
      systemctl disable --now "${NAME}.service" >/dev/null 2>&1 || true
      rm -f "$(unit_path_for)"
      echo "Removed: ${NAME}.service"
    fi
  done < <(get_conf_files)

  [[ "${any}" -eq 1 ]] || die "no target daemons matched (name filter or all disabled)"

  systemctl daemon-reload
  systemctl reset-failed >/dev/null 2>&1 || true
}

ctl_all() {
  require_root
  local verb="$1"
  local any=0
  while IFS= read -r conf; do
    load_conf "${conf}"
    if should_target; then
      any=1
      systemctl "${verb}" "${NAME}.service" || true
    fi
  done < <(get_conf_files)
  [[ "${any}" -eq 1 ]] || die "no target daemons matched"
}

status_all() {
  require_root
  local any=0
  while IFS= read -r conf; do
    load_conf "${conf}"
    if should_target; then
      any=1
      echo "== ${NAME}.service =="
      systemctl status "${NAME}.service" --no-pager || true
      echo
    fi
  done < <(get_conf_files)
  [[ "${any}" -eq 1 ]] || die "no target daemons matched"
}

logs_all() {
  require_root
  need journalctl
  local any=0
  while IFS= read -r conf; do
    load_conf "${conf}"
    if should_target; then
      any=1
      echo "== logs: ${NAME}.service =="
      journalctl -u "${NAME}.service" -n 200 --no-pager || true
      echo
    fi
  done < <(get_conf_files)
  [[ "${any}" -eq 1 ]] || die "no target daemons matched"
}

list_all() {
  while IFS= read -r conf; do
    load_conf "${conf}"
    printf "%-16s  bin=%s  args=%s  user=%s  enabled=%s\n" \
      "${NAME}" "${BIN_REL}" "${ARGS:-<none>}" "${USER}" "${ENABLED_BY_DEFAULT}"
  done < <(get_conf_files)
}

case "${ACTION}" in
  list) list_all ;;
  install) install_all ;;
  uninstall) uninstall_all ;;
  start|stop|restart) ctl_all "${ACTION}" ;;
  status) status_all ;;
  logs) logs_all ;;
  ""|-h|--help) usage ;;
  *) die "unknown action: ${ACTION}" ;;
esac
