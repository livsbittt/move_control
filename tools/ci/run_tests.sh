#!/usr/bin/env bash
# Run the pure-logic test suite (test/). One entry point for the developer PC,
# GitHub Actions and the robot, so CI can never drift from what you ran locally.
#
#   ./tools/ci/run_tests.sh            # venv mode: build .venv-ci, install deps, run
#   ./tools/ci/run_tests.sh --system   # system mode: use python3 as-is (the robot)
#
# Env: PYTHON=/path/to/python3 picks the interpreter (default: python3, then python).
#
# ROS coverage split -- read this before trusting a green run:
#   Most of test/ is ROS-free by design (sensing/, control/, planning/, watch.py
#   import no ROS). A few modules do reach rosy_control.safety / rosy_control.wander,
#   whose __init__ pulls in rclpy and the message packages. Off-robot -- your PC and
#   GitHub Actions -- those cannot import, so this script skips exactly those files
#   and says so loudly. On the robot (--system) ROS is present and the whole suite
#   runs, which is what the deploy gate checks before switching versions.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO_ROOT"

MODE="venv"
[ "${1:-}" = "--system" ] && { MODE="system"; shift; }

pick_python() {
  if [ -n "${PYTHON:-}" ]; then echo "$PYTHON"; return; fi
  for c in python3 python; do command -v "$c" >/dev/null 2>&1 && { echo "$c"; return; }; done
  echo "ci: no python3 on PATH" >&2; exit 2
}

# The robot gets numpy/opencv from apt (python3-numpy / python3-opencv, see
# package.xml) and ROS from /opt/ros. A venv there would hide both, so --system
# uses the interpreter as-is and installs nothing.
if [ "$MODE" = "system" ]; then
  PY="$(pick_python)"
else
  VENV="$REPO_ROOT/.venv-ci"
  BASE_PY="$(pick_python)"
  if [ ! -d "$VENV" ]; then
    echo "ci: creating venv at .venv-ci ($("$BASE_PY" --version 2>&1))"
    "$BASE_PY" -m venv "$VENV"
  fi
  # Windows (Git Bash) lays a venv out as Scripts/, POSIX as bin/.
  if   [ -x "$VENV/bin/python" ];          then PY="$VENV/bin/python"
  elif [ -x "$VENV/Scripts/python.exe" ];  then PY="$VENV/Scripts/python.exe"
  else echo "ci: venv at $VENV has no interpreter" >&2; exit 2
  fi
  echo "ci: installing tools/ci/requirements-dev.txt"
  "$PY" -m pip install --quiet --upgrade pip
  if ! "$PY" -m pip install --quiet -r tools/ci/requirements-dev.txt; then
    echo "ci: dependency install failed for $("$PY" --version 2>&1)." >&2
    echo "ci: usually means no wheels for this CPython yet -- retry with e.g. PYTHON=python3.12" >&2
    exit 2
  fi
fi

echo "ci: interpreter $("$PY" --version 2>&1) at $PY"

"$PY" - <<'PYEOF'
import importlib.util, sys
missing = [m for m in ("pytest", "numpy", "cv2") if importlib.util.find_spec(m) is None]
if missing:
    print("ci: missing modules: " + ", ".join(missing), file=sys.stderr)
    raise SystemExit(2)
PYEOF

# Decide the ROS split by asking the interpreter, then scanning the tests for
# the packages that need it -- scanning rather than a hardcoded filename list so
# a newly added ROS-touching test is classified automatically instead of
# failing collection for everyone.
SKIP_FILE="$(mktemp)"
trap 'rm -f "$SKIP_FILE"' EXIT
"$PY" - "$SKIP_FILE" <<'PYEOF'
import importlib.util, pathlib, re, sys

ROS_PKGS = ("rclpy", "sensor_msgs", "geometry_msgs", "nav_msgs",
            "std_msgs", "tf2_ros", "visualization_msgs")

def importable(name):
    try:
        return importlib.util.find_spec(name) is not None
    except (ImportError, ValueError):
        return False

absent = [p for p in ROS_PKGS if not importable(p)]
out = pathlib.Path(sys.argv[1])
if not absent:
    print("ci: ROS present -- running the whole suite")
    out.write_text("", encoding="utf-8")
    raise SystemExit(0)

# rosy_control.safety / rosy_control.wander re-export their ROS node modules
# from __init__, so importing anything under them needs ROS.
needle = re.compile(r"rosy_control\.(safety|wander)\b")
skip = sorted(p.as_posix() for p in pathlib.Path("test").glob("test_*.py")
              if needle.search(p.read_text(encoding="utf-8")))
out.write_text("\n".join(skip), encoding="utf-8")

print("ci: ROS not importable here (missing: %s)" % ", ".join(absent))
if skip:
    print("ci: SKIPPING %d ROS-dependent test file(s) -- they run on the robot:" % len(skip))
    for s in skip:
        print("ci:   - %s" % s)
else:
    print("ci: no ROS-dependent test files found")
PYEOF

IGNORE=()
# `|| [ -n "$f" ]` so the final line still counts when the file has no
# trailing newline -- otherwise the skip list silently does nothing.
while IFS= read -r f || [ -n "$f" ]; do
  [ -n "$f" ] && IGNORE+=("--ignore=$f")
done < "$SKIP_FILE"

echo "ci: pytest test/ ${IGNORE[*]:-}"
"$PY" -m pytest test/ -q "${IGNORE[@]+"${IGNORE[@]}"}"
