#!/usr/bin/env bash
# Runs ON THE ROBOT, once, to move the deploy state from the old `move_control`
# package name to `rosy_control`. Run this BEFORE the first rosy_control
# release is installed -- pinky_update.sh already looks under the new names and
# would otherwise start an empty release tree, orphaning the version history.
#
#   ~/releases/move_control/         -> ~/releases/rosy_control/
#   ~/dev_ws/wj/src/move_control     -> ~/dev_ws/wj/src/rosy_control
#   ~/.local/state/move_control/     -> ~/.local/state/rosy_control/
#
# The calibration result and the adc lock live under .local/state, so moving
# that directory is what keeps the robot's measured calibration across the
# rename instead of silently re-calibrating from scratch.
#
# ROLLBACK: every release already unpacked under the old directory has
# `move_control/` inside it and a package.xml naming `move_control`. Those
# versions stay readable but are NOT roll-back targets for the renamed
# updater -- this script clears `current`/`previous` so a half-renamed tree
# cannot be swapped into. The first rosy_control release is the new floor.
#
#   ./migrate_to_rosy_control.sh [--dry-run] [--force-lock]
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib.sh
. "$HERE/lib.sh"

DRY=0
FORCE_LOCK=0
for arg in "$@"; do
  case "$arg" in
    --dry-run)    DRY=1 ;;
    --force-lock) FORCE_LOCK=1 ;;
    -h|--help)    sed -n '2,20p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *)            die "unknown argument: $arg" ;;
  esac
done

# lib.sh already resolved the NEW names; the old ones are what we migrate from.
LEGACY_RELEASES="${PINKY_LEGACY_RELEASES:-$PINKY_HOME/releases/move_control}"
LEGACY_SRC_LINK="$WS/src/move_control"
LEGACY_STATE="$PINKY_HOME/.local/state/move_control"
NEW_STATE="$PINKY_HOME/.local/state/rosy_control"

run() {
  if [ "$DRY" = 1 ]; then echo "dry-run: $*"; else "$@"; fi
}

info "migrating deploy state: move_control -> rosy_control"
if [ "$DRY" = 1 ]; then info "DRY RUN -- nothing will be changed"; fi

# --- gate ------------------------------------------------------------------
# Same rule as an update: never rearrange the source tree under a moving robot.
if ! robot_is_stopped; then
  die "robot is driving -- stop it first (ros2 topic pub /wander/cmd std_msgs/msg/String \"data: stop\")"
fi

# --- lock ------------------------------------------------------------------
# The lock must be taken inside whichever tree currently holds the state.
# lock_acquire does `mkdir -p "$RELEASES"`, so pointing it at the destination
# before the move would CREATE the destination and trip the "both exist" guard
# below on every real migration. Point it at the legacy tree instead; the lock
# directory then travels with the tree when we move it.
NEW_RELEASES="$RELEASES"
if [ -d "$LEGACY_RELEASES" ]; then
  RELEASES="$LEGACY_RELEASES"
  LOCK_DIR="$RELEASES/.deploy.lock"
  DEPLOY_LOG="$RELEASES/deploy.log"
fi

if [ "$DRY" = 0 ]; then
  lock_acquire "migrate:$(id -un)" "$FORCE_LOCK"
fi

# --- 1. release tree -------------------------------------------------------
if [ -d "$LEGACY_RELEASES" ] && [ ! -d "$NEW_RELEASES" ]; then
  info "moving $LEGACY_RELEASES -> $NEW_RELEASES"
  run mv "$LEGACY_RELEASES" "$NEW_RELEASES"
elif [ -d "$NEW_RELEASES" ] && [ -d "$LEGACY_RELEASES" ]; then
  die "both $LEGACY_RELEASES and $NEW_RELEASES exist -- resolve by hand, this script will not merge them"
elif [ -d "$NEW_RELEASES" ]; then
  info "$NEW_RELEASES already exists -- release tree already migrated"
else
  info "no release tree at $LEGACY_RELEASES -- nothing to move (fresh robot)"
fi

# The held lock moved with the tree; follow it so the EXIT trap releases the
# right directory and the deploy log lands under the new name. Both are read by
# lock_release and log_deploy over in lib.sh, which shellcheck cannot see.
RELEASES="$NEW_RELEASES"
# shellcheck disable=SC2034
LOCK_DIR="$RELEASES/.deploy.lock"
# shellcheck disable=SC2034
DEPLOY_LOG="$RELEASES/deploy.log"

# --- 2. current/previous pointers ------------------------------------------
# These are absolute symlinks created with `ln -sfn "$(readlink -f ...)"`, so
# after the directory move they still point into the old path. Every target
# they could name is a pre-rename bundle, so clear them rather than repair
# them: the next pinky_update.sh writes both correctly.
for link in current previous; do
  if [ -L "$RELEASES/$link" ]; then
    info "clearing stale pointer $link -> $(readlink "$RELEASES/$link")"
    run rm -f "$RELEASES/$link"
  fi
done

# --- 3. workspace source symlink -------------------------------------------
if [ -L "$LEGACY_SRC_LINK" ]; then
  info "removing old source symlink $LEGACY_SRC_LINK -> $(readlink "$LEGACY_SRC_LINK")"
  run rm -f "$LEGACY_SRC_LINK"
elif [ -d "$LEGACY_SRC_LINK" ]; then
  MIGRATED="$LEGACY_SRC_LINK.premigrate.$(date -u +%Y%m%d%H%M%S)"
  info "$LEGACY_SRC_LINK is a real directory -- preserving it as $MIGRATED"
  run mv "$LEGACY_SRC_LINK" "$MIGRATED"
else
  info "no source symlink at $LEGACY_SRC_LINK -- nothing to remove"
fi
if [ -e "$SRC_LINK" ]; then
  info "note: $SRC_LINK already exists -- pinky_update.sh will swap it, not recreate it"
fi

# --- 4. runtime state (calibration!) ---------------------------------------
if [ -d "$LEGACY_STATE" ] && [ ! -d "$NEW_STATE" ]; then
  info "moving runtime state $LEGACY_STATE -> $NEW_STATE (keeps calibration.json)"
  run mkdir -p "$(dirname "$NEW_STATE")"
  run mv "$LEGACY_STATE" "$NEW_STATE"
elif [ -d "$NEW_STATE" ] && [ -d "$LEGACY_STATE" ]; then
  info "WARNING: both state dirs exist -- leaving $LEGACY_STATE in place, $NEW_STATE wins"
elif [ -d "$NEW_STATE" ]; then
  info "$NEW_STATE already exists -- state already migrated"
else
  info "no runtime state at $LEGACY_STATE -- calibration will be taken on first run"
fi

# --- 5. stale colcon artifacts ---------------------------------------------
# A leftover build/ or install/ entry keeps the old package discoverable, so
# `ros2 launch move_control ...` would still half-work and hide the rename.
for d in "$WS/build/move_control" "$WS/install/move_control"; do
  if [ -e "$d" ]; then
    info "removing stale colcon artifact $d"
    run rm -rf "$d"
  fi
done

if [ "$DRY" = 0 ] && [ -d "$RELEASES" ]; then
  log_deploy MIGRATED "-" "migrate:$(id -un)" "move_control -> rosy_control; current/previous cleared"
fi

info ""
info "migration done. The workspace has NO rosy_control source yet -- install one:"
info ""
info "    ~/pinky-deploy/pinky_update.sh --from-github --tag <first rosy_control tag>"
info ""
info "then verify and start it yourself:"
info ""
info "    ros2 pkg executables rosy_control"
info "    ros2 launch rosy_control robot.launch.py"
