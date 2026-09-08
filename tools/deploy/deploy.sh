#!/usr/bin/env bash
# Runs ON THE DEVELOPER PC. Builds a bundle, copies it to the robot over ssh,
# and runs the on-robot updater with your identity attached.
#
#   ./tools/deploy/deploy.sh                     # build from HEAD, then deploy
#   ./tools/deploy/deploy.sh --ref v0.2.0
#   ./tools/deploy/deploy.sh --bundle dist/move_control-0.2.0.tar.gz
#   ./tools/deploy/deploy.sh --rollback          # roll the robot back one version
#   ./tools/deploy/deploy.sh --list              # what the robot has installed
#
# Robot connection details live in tools/deploy/deploy.local.env, which is
# git-ignored -- this is a public repo, so the robot's address never gets
# committed. Copy deploy.env.example to start.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO_ROOT"

REF="HEAD"; BUNDLE=""; ACTION="deploy"; PASSTHRU=()
while [ $# -gt 0 ]; do
  case "$1" in
    --ref)      REF="$2"; shift 2 ;;
    --bundle)   BUNDLE="$2"; shift 2 ;;
    --rollback) ACTION="rollback"; shift ;;
    --list)     ACTION="list"; shift ;;
    -h|--help)  sed -n '2,13p' "$0"; exit 0 ;;
    *)          PASSTHRU+=("$1"); shift ;;   # e.g. --allow-active, --force-lock, --to
  esac
done

# Parsed args first so --help answers without needing a configured robot.
ENV_FILE="$REPO_ROOT/tools/deploy/deploy.local.env"
if [ -f "$ENV_FILE" ]; then
  # shellcheck disable=SC1090
  . "$ENV_FILE"
else
  echo "deploy: $ENV_FILE not found." >&2
  echo "deploy: cp tools/deploy/deploy.env.example tools/deploy/deploy.local.env and fill it in." >&2
  exit 2
fi

: "${PINKY_HOST:?set PINKY_HOST in deploy.local.env}"
PINKY_SSH_USER="${PINKY_SSH_USER:-pinky}"
PINKY_SSH_PORT="${PINKY_SSH_PORT:-22}"
PINKY_REMOTE_DEPLOY_DIR="${PINKY_REMOTE_DEPLOY_DIR:-/home/$PINKY_SSH_USER/pinky-deploy}"

SSH_OPTS=(-p "$PINKY_SSH_PORT")
SCP_OPTS=(-P "$PINKY_SSH_PORT")
if [ -n "${PINKY_SSH_KEY:-}" ]; then
  SSH_OPTS+=(-i "$PINKY_SSH_KEY")
  SCP_OPTS+=(-i "$PINKY_SSH_KEY")
fi
TARGET_SSH="$PINKY_SSH_USER@$PINKY_HOST"

# Identity travels with the command: on the robot every operator is the same
# `pinky` unix user, so this string is the only record of who actually did it.
WHO="$(git config user.email 2>/dev/null || true)"
if [ -z "$WHO" ]; then
  WHO="${USER:-${USERNAME:-unknown}}"
  echo "deploy: WARNING git user.email is unset -- logging you as '$WHO'." >&2
  echo "deploy: set it so the robot's deploy log names a person. This repo is" >&2
  echo "deploy: public, so prefer your GitHub noreply address over a real one:" >&2
  echo "deploy:   git config user.email <id>+<user>@users.noreply.github.com" >&2
fi
DEPLOYED_BY="$WHO@$(hostname 2>/dev/null || echo unknown-host)"

sync_scripts() {
  # The updater must live OUTSIDE the versioned tree: it replaces the symlink
  # that tree hangs from, and a script cannot have its own directory swapped
  # out from under it mid-run.
  echo "deploy: syncing updater scripts to $TARGET_SSH:$PINKY_REMOTE_DEPLOY_DIR"
  ssh "${SSH_OPTS[@]}" "$TARGET_SSH" "mkdir -p '$PINKY_REMOTE_DEPLOY_DIR'"
  scp "${SCP_OPTS[@]}" -q \
    tools/deploy/lib.sh tools/deploy/pinky_update.sh tools/deploy/pinky_rollback.sh \
    "$TARGET_SSH:$PINKY_REMOTE_DEPLOY_DIR/"
  ssh "${SSH_OPTS[@]}" "$TARGET_SSH" "chmod +x '$PINKY_REMOTE_DEPLOY_DIR'/*.sh"
}

remote() { ssh "${SSH_OPTS[@]}" "$TARGET_SSH" "$@"; }

case "$ACTION" in
  list)
    sync_scripts
    remote "DEPLOYED_BY='$DEPLOYED_BY' '$PINKY_REMOTE_DEPLOY_DIR/pinky_rollback.sh' --list"
    exit 0 ;;
  rollback)
    sync_scripts
    remote "DEPLOYED_BY='$DEPLOYED_BY' '$PINKY_REMOTE_DEPLOY_DIR/pinky_rollback.sh' ${PASSTHRU[*]+${PASSTHRU[*]}}"
    exit 0 ;;
esac

# --- build ------------------------------------------------------------------
if [ -z "$BUNDLE" ]; then
  echo "deploy: building bundle from $REF"
  ./tools/ci/make_bundle.sh --ref "$REF"
  BUNDLE="$(ls -t dist/move_control-*.tar.gz | head -1)"
fi
[ -f "$BUNDLE" ] || { echo "deploy: bundle not found: $BUNDLE" >&2; exit 2; }
BASE="$(basename "$BUNDLE")"
STEM="${BUNDLE%.tar.gz}"
for f in "$BUNDLE" "$BUNDLE.sha256" "$STEM.manifest.json"; do
  [ -f "$f" ] || { echo "deploy: missing $f -- rebuild with tools/ci/make_bundle.sh" >&2; exit 2; }
done

# --- ship -------------------------------------------------------------------
sync_scripts
REMOTE_TMP="/tmp/move_control-deploy"
echo "deploy: copying $BASE to $TARGET_SSH:$REMOTE_TMP"
remote "mkdir -p '$REMOTE_TMP'"
scp "${SCP_OPTS[@]}" -q "$BUNDLE" "$BUNDLE.sha256" "$STEM.manifest.json" "$TARGET_SSH:$REMOTE_TMP/"

echo "deploy: running the updater on $PINKY_HOST as '$DEPLOYED_BY'"
remote "DEPLOYED_BY='$DEPLOYED_BY' '$PINKY_REMOTE_DEPLOY_DIR/pinky_update.sh' --bundle '$REMOTE_TMP/$BASE' ${PASSTHRU[*]+${PASSTHRU[*]}}"

echo "deploy: done. Source is updated; the robot is NOT driving."
echo "deploy: start it when ready:  ssh $TARGET_SSH 'ros2 launch move_control robot.launch.py'"
