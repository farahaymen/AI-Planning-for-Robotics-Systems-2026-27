#!/usr/bin/env bash
# Shared helpers. Every provisioning stage sources this.
#
# Stages are idempotent: re-running a completed stage is a no-op, so a failure
# halfway through stage 30 is fixed by re-running stage 30, not by rebuilding
# the machine.
set -euo pipefail

ARC_RELEASE="2026.1"
ROS_DISTRO_NAME="jazzy"
STAMP_DIR="/var/lib/arc-provision"

c_ok=$'\033[32m'; c_warn=$'\033[33m'; c_err=$'\033[31m'; c_off=$'\033[0m'

log()  { echo "${c_ok}[arc]${c_off} $*"; }
warn() { echo "${c_warn}[arc]${c_off} $*"; }
die()  { echo "${c_err}[arc] FAILED:${c_off} $*" >&2; exit 1; }

require_root() { [[ $EUID -eq 0 ]] || die "run this with sudo"; }

stamp_done() { sudo mkdir -p "$STAMP_DIR"; sudo touch "$STAMP_DIR/$1"; }
is_done()    { [[ -f "$STAMP_DIR/$1" ]]; }

stage() {
  local name="$1"
  if is_done "$name"; then
    log "stage $name already complete, skipping"
    return 1
  fi
  log "stage $name starting"
  return 0
}

# The course user. Provisioning runs as root but the workspace belongs to them.
ARC_USER="${SUDO_USER:-${ARC_USER:-student}}"
ARC_HOME="$(getent passwd "$ARC_USER" | cut -d: -f6)"
