#!/usr/bin/env bash
# Build one deployable bundle: tarball + sha256 + manifest.
# The bundle is the ONLY deploy unit -- the robot updater takes the same file
# whether it was downloaded from a GitHub release or copied over SSH.
#
#   ./tools/ci/make_bundle.sh                    # from HEAD
#   ./tools/ci/make_bundle.sh --ref v0.2.0       # from a tag
#   ./tools/ci/make_bundle.sh --allow-dirty      # include uncommitted work (never in CI)
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO_ROOT"

REF="HEAD"; OUT_DIR="$REPO_ROOT/dist"; VERSION=""; ALLOW_DIRTY=0
while [ $# -gt 0 ]; do
  case "$1" in
    --ref)         REF="$2"; shift 2 ;;
    --out)         OUT_DIR="$2"; shift 2 ;;
    --version)     VERSION="$2"; shift 2 ;;
    --allow-dirty) ALLOW_DIRTY=1; shift ;;
    -h|--help)     sed -n '2,10p' "$0"; exit 0 ;;
    *) echo "make_bundle: unknown arg $1" >&2; exit 2 ;;
  esac
done

command -v git >/dev/null || { echo "make_bundle: git required" >&2; exit 2; }

DIRTY=0
git diff --quiet HEAD -- 2>/dev/null || DIRTY=1
if [ "$DIRTY" = 1 ] && [ "$ALLOW_DIRTY" = 0 ]; then
  echo "make_bundle: working tree has uncommitted changes." >&2
  echo "make_bundle: deploy what is committed and pushed -- commit first, or pass --allow-dirty." >&2
  exit 2
fi

COMMIT="$(git rev-parse "$REF")"
SHORT="$(git rev-parse --short=8 "$REF")"
if [ -z "$VERSION" ]; then
  # Prefer a tag so releases read as versions; fall back to date+sha so every
  # untagged build is still uniquely identifiable on the robot.
  if VERSION="$(git describe --tags --exact-match "$REF" 2>/dev/null)"; then :
  else VERSION="0+$(date -u +%Y%m%d.%H%M%S).${SHORT}"; fi
fi
[ "$DIRTY" = 1 ] && VERSION="${VERSION}.dirty"

PKG_VERSION="$(sed -n 's:.*<version>\(.*\)</version>.*:\1:p' package.xml | head -1)"
BUILT_BY="$(git config user.email 2>/dev/null || true)"
[ -n "$BUILT_BY" ] || BUILT_BY="${USER:-${USERNAME:-unknown}}"
BUILD_HOST="$(hostname 2>/dev/null || echo unknown)"
BUILT_AT="$(date -u +%Y-%m-%dT%H:%M:%SZ)"

mkdir -p "$OUT_DIR"
BASE="move_control-${VERSION}"
TAR="$OUT_DIR/${BASE}.tar.gz"

if [ "$ALLOW_DIRTY" = 1 ]; then
  # Tracked files as they currently sit on disk. One tar process fed from
  # git ls-files -- a copy-per-file loop costs minutes on Windows.
  git ls-files -z | tar --null -T - -czf "$TAR" --transform='s,^,move_control/,'
else
  git archive --format=tar.gz --prefix=move_control/ -o "$TAR" "$REF"
fi

sha256_of() {
  if command -v sha256sum >/dev/null 2>&1; then sha256sum "$1" | awk '{print $1}'
  elif command -v shasum >/dev/null 2>&1; then shasum -a 256 "$1" | awk '{print $1}'
  else python3 -c "import hashlib,sys;print(hashlib.sha256(open(sys.argv[1],'rb').read()).hexdigest())" "$1"
  fi
}
SUM="$(sha256_of "$TAR")"
echo "${SUM}  ${BASE}.tar.gz" > "$OUT_DIR/${BASE}.tar.gz.sha256"

# Provenance travels with the bundle: on the robot everyone is the `pinky` unix
# user, so the manifest -- not the ssh login -- is what says who shipped this.
cat > "$OUT_DIR/${BASE}.manifest.json" <<JSON
{
  "name": "move_control",
  "version": "${VERSION}",
  "package_version": "${PKG_VERSION}",
  "commit": "${COMMIT}",
  "ref": "${REF}",
  "tree_dirty": $( [ "$DIRTY" = 1 ] && echo true || echo false ),
  "built_at": "${BUILT_AT}",
  "built_by": "${BUILT_BY}",
  "build_host": "${BUILD_HOST}",
  "bundle": "${BASE}.tar.gz",
  "sha256": "${SUM}"
}
JSON

echo "bundle : $TAR"
echo "sha256 : $SUM"
echo "version: $VERSION"
