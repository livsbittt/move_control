#!/usr/bin/env bash
# Runs ON THE ROBOT. Installs one bundle, switching only while stopped, and
# restores the previous version if the new one fails to build or verify.
#
# This updates SOURCE ONLY and never starts driving -- bring the stack up
# yourself afterwards: ros2 launch move_control robot.launch.py
#
#   pinky_update.sh --bundle /tmp/move_control-0.2.0.tar.gz
#   pinky_update.sh --from-github               # latest release
#   pinky_update.sh --from-github --tag v0.2.0
set -euo pipefail
. "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib.sh"

BUNDLE=""; MANIFEST=""; FROM_GITHUB=0; TAG=""
REPO="${PINKY_REPO:-livsbittt/move_control}"
DEPLOYED_BY="${DEPLOYED_BY:-}"
FORCE_LOCK=0; ALLOW_ACTIVE=0; DO_BUILD=1

while [ $# -gt 0 ]; do
  case "$1" in
    --bundle)       BUNDLE="$2"; shift 2 ;;
    --from-github)  FROM_GITHUB=1; shift ;;
    --tag)          TAG="$2"; FROM_GITHUB=1; shift 2 ;;
    --repo)         REPO="$2"; shift 2 ;;
    --deployed-by)  DEPLOYED_BY="$2"; shift 2 ;;
    --force-lock)   FORCE_LOCK=1; shift ;;
    --allow-active) ALLOW_ACTIVE=1; shift ;;
    --no-build)     DO_BUILD=0; shift ;;
    -h|--help)      sed -n '2,12p' "$0"; exit 0 ;;
    *) die "unknown arg $1" ;;
  esac
done

# Everyone reaches the robot as the same `pinky` unix user, so identity has to
# be carried in, not read off the login. deploy.sh passes the operator through.
[ -n "$DEPLOYED_BY" ] || DEPLOYED_BY="$(id -un)@$(hostname)"
[ -n "$BUNDLE" ] || [ "$FROM_GITHUB" = 1 ] || die "need --bundle FILE or --from-github"

lock_acquire "$DEPLOYED_BY" "$FORCE_LOCK"
mkdir -p "$RELEASES"

WORK="$(mktemp -d)"
cleanup_work() { rm -rf "$WORK"; }

# --- 1. obtain the bundle --------------------------------------------------
if [ "$FROM_GITHUB" = 1 ]; then
  # A public repo needs no credential. Export GH_TOKEN and the same code path
  # works if the repo is made private later.
  AUTH=()
  [ -n "${GH_TOKEN:-}" ] && AUTH=(-H "Authorization: Bearer $GH_TOKEN")
  API="https://api.github.com/repos/$REPO/releases/latest"
  [ -n "$TAG" ] && API="https://api.github.com/repos/$REPO/releases/tags/$TAG"

  info "querying $API"
  curl -fsSL "${AUTH[@]+"${AUTH[@]}"}" -H 'Accept: application/vnd.github+json' \
       "$API" > "$WORK/release.json" \
    || { cleanup_work; die "cannot read release metadata from $REPO (tag=${TAG:-latest})"; }

  python3 - "$WORK/release.json" "$WORK/urls" <<'PYEOF' || { cleanup_work; die "release is missing required assets"; }
import json, sys
rel = json.load(open(sys.argv[1]))
want = {}
for a in rel.get("assets", []):
    name, url = a["name"], a["browser_download_url"]
    if name.endswith(".tar.gz"):          want["tar"] = (name, url)
    elif name.endswith(".tar.gz.sha256"): want["sum"] = (name, url)
    elif name.endswith(".manifest.json"): want["man"] = (name, url)
missing = [k for k in ("tar", "sum", "man") if k not in want]
if missing:
    sys.exit("release %s lacks assets: %s" % (rel.get("tag_name"), ", ".join(missing)))
with open(sys.argv[2], "w") as fh:
    for key in ("tar", "sum", "man"):
        fh.write("%s\t%s\t%s\n" % ((key,) + want[key]))
PYEOF

  while IFS="$(printf '\t')" read -r kind name url; do
    [ -n "$kind" ] || continue
    info "downloading $name"
    curl -fsSL "${AUTH[@]+"${AUTH[@]}"}" -o "$WORK/$name" "$url" \
      || { cleanup_work; die "download failed: $name"; }
    [ "$kind" = tar ] && BUNDLE="$WORK/$name"
    [ "$kind" = man ] && MANIFEST="$WORK/$name"
  done < "$WORK/urls"
else
  [ -f "$BUNDLE" ] || { cleanup_work; die "bundle not found: $BUNDLE"; }
  BUNDLE="$(cd "$(dirname "$BUNDLE")" && pwd)/$(basename "$BUNDLE")"
  MANIFEST="${BUNDLE%.tar.gz}.manifest.json"
fi

# --- 2. verify integrity before anything touches the workspace -------------
SUMFILE="${BUNDLE}.sha256"
[ -f "$SUMFILE" ] || { cleanup_work; die "missing checksum file: $SUMFILE"; }
EXPECT="$(awk '{print $1}' < "$SUMFILE")"
ACTUAL="$(sha256_of "$BUNDLE")"
[ "$EXPECT" = "$ACTUAL" ] || { cleanup_work; die "checksum mismatch: expected $EXPECT, got $ACTUAL"; }
info "checksum ok ($ACTUAL)"

[ -f "$MANIFEST" ] || { cleanup_work; die "missing manifest: $MANIFEST"; }
VERSION="$(manifest_get "$MANIFEST" version)"
COMMIT="$(manifest_get "$MANIFEST" commit)"
BUILT_BY="$(manifest_get "$MANIFEST" built_by)"
[ -n "$VERSION" ] || { cleanup_work; die "manifest has no version"; }
info "version $VERSION (commit ${COMMIT:0:8}, built by ${BUILT_BY:-unknown})"

# --- 3. stopped gate -------------------------------------------------------
if robot_is_stopped; then
  :
elif [ "$ALLOW_ACTIVE" = 1 ]; then
  info "robot is ACTIVE but --allow-active was given -- continuing"
else
  cleanup_work
  log_deploy REFUSED "$VERSION" "$DEPLOYED_BY" "robot active"
  die "robot is driving. Stop it first:
       ros2 topic pub --once /wander/cmd std_msgs/msg/String \"{data: stop}\""
fi

# --- 4. unpack into its own version directory ------------------------------
TARGET="$RELEASES/$VERSION"
if [ -d "$TARGET" ]; then
  info "version $VERSION already unpacked -- reusing it"
else
  STAGE="$RELEASES/.staging-$VERSION.$$"
  rm -rf "$STAGE"; mkdir -p "$STAGE"
  tar -xzf "$BUNDLE" -C "$STAGE" || { rm -rf "$STAGE"; cleanup_work; die "extract failed"; }
  [ -d "$STAGE/move_control" ] || { rm -rf "$STAGE"; cleanup_work; die "bundle has no move_control/ root"; }
  cp "$MANIFEST" "$STAGE/move_control/.bundle-manifest.json"
  # Rename into place so a version dir is never half-written.
  mv "$STAGE/move_control" "$TARGET"
  rm -rf "$STAGE"
fi
cleanup_work

# --- 5. remember what we are leaving, so rollback has a target -------------
mkdir -p "$(dirname "$SRC_LINK")"
PREVIOUS=""
if [ -L "$SRC_LINK" ]; then
  PREVIOUS="$(readlink -f "$SRC_LINK")"
elif [ -d "$SRC_LINK" ]; then
  # First run: the workspace still holds the unzip-era real directory. Keep it
  # as a version so even the first switch has something to fall back to.
  MIGRATED="$RELEASES/pre-cicd-$(date -u +%Y%m%d.%H%M%S)"
  info "migrating the existing source directory to $MIGRATED"
  mv "$SRC_LINK" "$MIGRATED"
  PREVIOUS="$MIGRATED"
fi
if [ -n "$PREVIOUS" ] && [ "$PREVIOUS" != "$TARGET" ]; then
  ln -sfn "$PREVIOUS" "$RELEASES/previous"
fi

swap_to() {  # atomic: build the new link beside the old one, then rename over it
  ln -sfn "$1" "$RELEASES/.swap.$$"
  mv -Tf "$RELEASES/.swap.$$" "$SRC_LINK"
  ln -sfn "$1" "$RELEASES/current"
}

build_ws() { ( cd "$WS" && colcon build --packages-select move_control ); }

rollback() {  # rollback <reason>
  local reason="$1"
  if [ -z "$PREVIOUS" ] || [ ! -d "$PREVIOUS" ]; then
    log_deploy FAILED "$VERSION" "$DEPLOYED_BY" "$reason; no previous version to restore"
    die "$reason -- and there is no previous version to restore. Workspace still points at $VERSION."
  fi
  info "rolling back to $(basename "$PREVIOUS")"
  swap_to "$PREVIOUS"
  if [ "$DO_BUILD" = 1 ] && ! build_ws >/dev/null 2>&1; then
    info "WARNING: rebuilding the previous version also failed -- inspect $WS by hand"
  fi
  log_deploy ROLLBACK "$VERSION" "$DEPLOYED_BY" "$reason; restored $(basename "$PREVIOUS")"
  die "$reason -- rolled back to $(basename "$PREVIOUS")"
}

# --- 6. switch -------------------------------------------------------------
info "switching $SRC_LINK -> $TARGET"
swap_to "$TARGET"

# --- 7. build and verify, rolling back on the first failure ----------------
if [ "$DO_BUILD" = 1 ]; then
  info "colcon build --packages-select move_control"
  build_ws || rollback "colcon build failed"
fi

info "running the test suite against the new version"
( cd "$TARGET" && ./tools/ci/run_tests.sh --system ) || rollback "test suite failed"

if [ "$DO_BUILD" = 1 ] && command -v ros2 >/dev/null 2>&1; then
  # setup.bash is what makes the freshly built entry points visible.
  # shellcheck disable=SC1091
  [ -f "$WS/install/setup.bash" ] && . "$WS/install/setup.bash"
  EXPECTED_EXECUTABLES="${PINKY_EXPECTED_EXECUTABLES:-8}"
  COUNT="$(ros2 pkg executables move_control 2>/dev/null | grep -c . || true)"
  [ "${COUNT:-0}" -ge "$EXPECTED_EXECUTABLES" ] \
    || rollback "only ${COUNT:-0}/$EXPECTED_EXECUTABLES entry points registered after build"
  info "$COUNT entry points registered"
fi

log_deploy SUCCESS "$VERSION" "$DEPLOYED_BY" \
  "commit=${COMMIT:0:8} built_by=${BUILT_BY:-unknown} prev=$(basename "${PREVIOUS:-none}")"
info "now on $VERSION"
info "source updated. Driving is NOT started -- run 'ros2 launch move_control robot.launch.py' when you are ready."
