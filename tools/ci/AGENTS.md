<!-- Generated: 2026-09-07 -->

# tools/ci — test runner and bundle builder

## Purpose
The PC-side half of the delivery pipeline. `run_tests.sh` is the single test
command shared by your machine, GitHub Actions and the robot, so a green tick
in CI means exactly what a green local run means. `make_bundle.sh` turns a
commit into the one artifact the robot installs.

## Key Files
| File | Description |
|------|-------------|
| `run_tests.sh` | Runs `test/`. Builds `.venv-ci` on a dev PC; `--system` uses the interpreter as-is (the robot). |
| `make_bundle.sh` | Builds `dist/move_control-<version>.tar.gz` + `.sha256` + `.manifest.json` from a git ref. |
| `requirements-dev.txt` | Test dependencies for the PC venv only. The robot gets numpy/opencv from apt. |

## The ROS coverage split — read before trusting a green run

`sensing/`, `control/`, `planning/` and `watch.py` import no ROS, which is what
makes most of `test/` runnable anywhere. `move_control.safety` and
`move_control.wander` re-export their ROS node modules from `__init__`, so any
test reaching into those needs `rclpy` and the message packages.

`run_tests.sh` probes for ROS, and when it is absent it skips exactly those
files and prints each one it skipped. Off-robot that is currently
`test/test_scale.py`. The classification is a source scan, not a hardcoded
filename, so a newly added ROS-touching test is handled automatically instead
of breaking collection for everyone.

The full suite does run — on the robot, as the deploy gate, before a version
switch is allowed to stand. A PC or Actions run alone never proves the ROS-side
tests pass.

## Commands
```bash
./tools/ci/run_tests.sh                  # PC: venv + full runnable subset
./tools/ci/run_tests.sh --system         # robot: system python, whole suite
PYTHON=python3.12 ./tools/ci/run_tests.sh   # pin the interpreter

./tools/ci/make_bundle.sh                # from HEAD
./tools/ci/make_bundle.sh --ref v0.2.0   # from a tag
./tools/ci/make_bundle.sh --allow-dirty  # include uncommitted work (never in CI)
```

## For AI Agents

- `make_bundle.sh` **refuses a dirty working tree** by default. That is
  deliberate: you deploy what is committed and pushed, so the commit in the
  manifest actually describes what is on the robot. `--allow-dirty` marks the
  version `.dirty` and is for local experiments only.
- Version is `git describe` when the ref is tagged, otherwise
  `0+<UTC date>.<short sha>` — every untagged build stays uniquely
  identifiable once it is sitting on a robot.
- The manifest carries provenance (`commit`, `built_by`, `build_host`,
  `built_at`). On the robot everyone logs in as `pinky`, so this file, not the
  ssh session, is what records who shipped a version.
- **Keep these scripts LF.** `deploy.sh` copies them to the robot verbatim and
  a CRLF makes them fail there with a bare "bad interpreter". `.gitattributes`
  pins `*.sh text eol=lf` and CI rejects a CR outright, but a Windows editor
  can still dirty your working copy — check with `tr -cd '\r' < f | wc -c`.

## Dependencies
- PC: python3 with venv support; `git`, `tar`, `curl`.
- Robot: system python3 with `python3-numpy` / `python3-opencv` from apt.
- CI: `.github/workflows/ci.yml` calls these same scripts and adds shellcheck.
