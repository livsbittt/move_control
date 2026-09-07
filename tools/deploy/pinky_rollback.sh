#!/usr/bin/env bash
# Runs ON THE ROBOT. Manual rollback, for when a version builds and passes its
# tests but misbehaves once you actually drive it -- the automatic rollback
# inside pinky_update.sh only covers build/verify failures.
#
#   pinky_rollback.sh                 # back to the previous version
#   pinky_rollback.sh --to <version>  # back to a specific one
#   pinky_rollback.sh --list          # what is installed, and the history
set -euo pipefail
. "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib.sh"

TO=""; DEPLOYED_BY="${DEPLOYED_BY:-}"; FORCE_LOCK=0; ALLOW_ACTIVE=0; DO_BUILD=1; LIST=0
while [ $# -gt 0 ]; do
  case "$1" in
    --to)           TO="$2"; shift 2 ;;
    --list)         LIST=1; shift ;;
    --deployed-by)  DEPLOYED_BY="$2"; shift 2 ;;
    --force-lock)   FORCE_LOCK=1; shift ;;
    --allow-active) ALLOW_ACTIVE=1; shift ;;
    --no-build)     DO_BUILD=0; shift ;;
    -h|--help)      sed -n '2,10p' "$0"; exit 0 ;;
    *) die "unknown arg $1" ;;
  esac
done

if [ "$LIST" = 1 ]; then
  echo "installed versions in $RELEASES:"
  for d in "$RELEASES"/*/; do
    [ -d "$d" ] || continue
    case "$(basename "$d")" in current|previous) continue ;; esac
    printf '  %s\n' "$(basename "$d")"
  done
  echo "current : $(readlink -f "$SRC_LINK" 2>/dev/null || echo '<not a symlink>')"
  echo "previous: $(readlink -f "$RELEASES/previous" 2>/dev/null || echo '<none>')"
  if [ -f "$DEPLOY_LOG" ]; then
    echo "last 10 deploy log entries (utc / event / version / by / from / detail):"
    tail -10 "$DEPLOY_LOG" | sed 's/^/  /'
  fi
  exit 0
fi

[ -n "$DEPLOYED_BY" ] || DEPLOYED_BY="$(id -un)@$(hostname)"

if [ -n "$TO" ]; then
  DEST="$RELEASES/$TO"
else
  DEST="$(readlink -f "$RELEASES/previous" 2>/dev/null || true)"
fi
[ -n "$DEST" ] || die "no previous version recorded. Pick one explicitly: --to <version> (see --list)"
[ -d "$DEST" ] || die "no such version: $DEST"

CURRENT="$(readlink -f "$SRC_LINK" 2>/dev/null || true)"
[ "$DEST" != "$CURRENT" ] || die "already on $(basename "$DEST")"

lock_acquire "$DEPLOYED_BY" "$FORCE_LOCK"

if robot_is_stopped; then
  :
elif [ "$ALLOW_ACTIVE" = 1 ]; then
  info "robot is ACTIVE but --allow-active was given -- continuing"
else
  die "robot is driving. Stop it first:
       ros2 topic pub --once /wander/cmd std_msgs/msg/String \"{data: stop}\""
fi

info "rolling back $(basename "${CURRENT:-none}") -> $(basename "$DEST")"
ln -sfn "$DEST" "$RELEASES/.swap.$$"
mv -Tf "$RELEASES/.swap.$$" "$SRC_LINK"
ln -sfn "$DEST" "$RELEASES/current"
# The version we just left becomes the thing to go back to, so a rollback can
# itself be undone.
[ -n "$CURRENT" ] && ln -sfn "$CURRENT" "$RELEASES/previous"

if [ "$DO_BUILD" = 1 ]; then
  info "colcon build --packages-select move_control"
  ( cd "$WS" && colcon build --packages-select move_control ) \
    || die "rebuild failed after rollback -- inspect $WS by hand"
fi

log_deploy MANUAL_ROLLBACK "$(basename "$DEST")" "$DEPLOYED_BY" "from=$(basename "${CURRENT:-none}")"
info "now on $(basename "$DEST")"
info "driving is NOT started -- run 'ros2 launch move_control robot.launch.py' when you are ready."
