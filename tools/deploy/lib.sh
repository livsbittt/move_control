#!/usr/bin/env bash
# Shared helpers for the on-robot deploy scripts. Sourced, never run directly.
#
# Layout on the robot (approach A -- versioned source dirs, symlink swap):
#   $RELEASES/<version>/        unpacked bundle (one dir per version)
#   $RELEASES/current           -> the version the workspace source points at
#   $RELEASES/previous          -> the version to roll back to
#   $RELEASES/deploy.log        append-only history: who shipped what, and how it went
#   $RELEASES/.deploy.lock/     mkdir-based mutex (holder identity inside)
#   $WS/src/move_control        symlink -> $RELEASES/<version>

PINKY_HOME="${PINKY_HOME:-$HOME}"
WS="${PINKY_WS:-$PINKY_HOME/dev_ws/wj}"
RELEASES="${PINKY_RELEASES:-$PINKY_HOME/releases/move_control}"
# Consumed by the scripts that source this file, not here.
# shellcheck disable=SC2034
SRC_LINK="$WS/src/move_control"
DEPLOY_LOG="$RELEASES/deploy.log"
LOCK_DIR="$RELEASES/.deploy.lock"
LOCK_STALE_SECONDS="${PINKY_LOCK_STALE_SECONDS:-1800}"

die()  { echo "deploy: $*" >&2; exit 1; }
info() { echo "deploy: $*"; }

now_utc() { date -u +%Y-%m-%dT%H:%M:%SZ; }

sha256_of() {
  if command -v sha256sum >/dev/null 2>&1; then sha256sum "$1" | awk '{print $1}'
  else python3 -c "import hashlib,sys;print(hashlib.sha256(open(sys.argv[1],'rb').read()).hexdigest())" "$1"
  fi
}

# Read one field out of a manifest. python3 is always present on the robot --
# it is a ROS 2 python package -- so no jq dependency.
manifest_get() {
  python3 -c 'import json,sys;print(json.load(open(sys.argv[1])).get(sys.argv[2],""))' "$1" "$2"
}

log_deploy() {  # log_deploy <event> <version> <by> <detail>
  mkdir -p "$RELEASES"
  printf '%s\t%s\t%s\t%s\t%s\t%s\n' \
    "$(now_utc)" "$1" "$2" "$3" "$(id -un)@$(hostname)" "$4" >> "$DEPLOY_LOG"
}

# --- lock ------------------------------------------------------------------
# Every operator reaches the robot as the same `pinky` unix user, so the lock
# records WHO claimed it; that is the only way "who is deploying right now?"
# has an answer.
lock_acquire() {  # lock_acquire <deployed_by> [force]
  local by="$1" force="${2:-0}"
  mkdir -p "$RELEASES"
  if ! mkdir "$LOCK_DIR" 2>/dev/null; then
    local holder age
    holder="$(cat "$LOCK_DIR/holder" 2>/dev/null || echo unknown)"
    age=$(( $(date +%s) - $(stat -c %Y "$LOCK_DIR" 2>/dev/null || date +%s) ))
    if [ "$age" -gt "$LOCK_STALE_SECONDS" ]; then
      info "breaking stale lock (${age}s old, held by $holder)"
      rm -rf "$LOCK_DIR"; mkdir "$LOCK_DIR" || die "cannot create lock"
    elif [ "$force" = 1 ]; then
      info "forcing lock held by $holder (${age}s old)"
      rm -rf "$LOCK_DIR"; mkdir "$LOCK_DIR" || die "cannot create lock"
    else
      die "another deploy is in progress: $holder (${age}s ago). Wait, or pass --force-lock."
    fi
  fi
  printf '%s pid=%s at=%s from=%s\n' "$by" "$$" "$(now_utc)" "$(id -un)@$(hostname)" > "$LOCK_DIR/holder"
  trap lock_release EXIT INT TERM
}

lock_release() { rm -rf "$LOCK_DIR" 2>/dev/null || true; }

# --- stopped gate ----------------------------------------------------------
# Source updates must never happen under a moving robot, and updating must
# never start driving. Returns 0 only when we can positively tell it is idle.
robot_is_stopped() {
  if ! command -v ros2 >/dev/null 2>&1; then
    info "ros2 not on PATH -- treating robot as stopped"; return 0
  fi
  local nodes
  nodes="$(timeout 10 ros2 node list 2>/dev/null || true)"
  if [ -z "$nodes" ]; then
    info "no ROS nodes running -- robot is stopped"; return 0
  fi
  if ! printf '%s\n' "$nodes" | grep -q 'wander_node'; then
    info "wander_node not running -- nothing is driving"; return 0
  fi
  local state
  state="$(timeout 10 ros2 topic echo /wander/state std_msgs/msg/String --once 2>/dev/null \
           | sed -n 's/^data: *//p' | tr -d "'\"" | tr -d '\r')"
  case "$state" in
    stop|wait|"") info "wander state '${state:-<none>}' -- stopped"; return 0 ;;
    *)            info "wander state '$state' -- robot is ACTIVE"; return 1 ;;
  esac
}
