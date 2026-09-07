<!-- Generated: 2026-09-07 -->

# tools/deploy — robot update and rollback

## Purpose
Installs one bundle onto Pinky, switching versions only while the robot is
stopped and restoring the previous version when a new one fails. Updating
source and starting to drive are deliberately separate actions: nothing here
ever brings the stack up.

## Key Files
| File | Description |
|------|-------------|
| `lib.sh` | Sourced by the on-robot scripts: paths, lock, deploy log, checksum, the stopped check. |
| `pinky_update.sh` | Runs **on the robot**. Verify → gate → unpack → swap → build → verify → roll back on failure. |
| `pinky_rollback.sh` | Runs **on the robot**. Manual rollback and `--list` for installed versions plus history. |
| `deploy.sh` | Runs **on the PC**. Builds a bundle, ships it over ssh, invokes the updater with your identity. |
| `deploy.env.example` | Template for `deploy.local.env` (git-ignored — this repo is public). |

## Layout on the robot
```
~/releases/move_control/<version>/   unpacked bundle, one directory per version
~/releases/move_control/current      -> the version in use
~/releases/move_control/previous     -> the rollback target
~/releases/move_control/deploy.log   append-only: utc, event, version, by, from, detail
~/releases/move_control/.deploy.lock mkdir mutex, holder identity inside
~/pinky-deploy/                      the updater scripts themselves
~/dev_ws/wj/src/move_control         symlink -> the current version
```

The updater lives in `~/pinky-deploy/`, **outside** the versioned tree, because
it replaces the symlink that tree hangs from — a script cannot have its own
directory swapped out from under it mid-run.

## Operator runbook (Korean summary)

```bash
# PC에서: 최초 1회 설정
cp tools/deploy/deploy.env.example tools/deploy/deploy.local.env   # 로봇 IP 입력

# PC에서: 커밋된 HEAD를 빌드해 로봇에 배포
./tools/deploy/deploy.sh

# PC에서: 로봇 상태 확인 / 롤백
./tools/deploy/deploy.sh --list
./tools/deploy/deploy.sh --rollback

# 로봇에서: GitHub 릴리스를 직접 받아 설치
~/pinky-deploy/pinky_update.sh --from-github --tag v0.2.0
~/pinky-deploy/pinky_rollback.sh --list

# 업데이트는 주행을 시작하지 않는다. 준비되면 직접:
ros2 launch move_control robot.launch.py
```

## What the gates actually check

1. **Integrity** — sha256 against the sidecar, before anything touches the
   workspace. A mismatch aborts with the workspace untouched.
2. **Stopped** — refuses to switch while `wander_node` reports a driving state.
   No running nodes, or no `wander_node`, counts as stopped. `--allow-active`
   overrides it explicitly; there is no silent bypass.
3. **Build** — `colcon build --packages-select move_control`.
4. **Tests** — the whole suite via `run_tests.sh --system`; ROS is present here,
   so this is the run that covers the files a PC or Actions run has to skip.
5. **Entry points** — `ros2 pkg executables move_control` must list at least 8
   (`PINKY_EXPECTED_EXECUTABLES` to change), which catches a build that
   "succeeded" without registering console scripts.

Failing 3, 4 or 5 rolls back to `previous` and rebuilds. Failing 1 or 2 never
switches at all.

## For AI Agents

- **Identity is carried, not inferred.** Every operator reaches the robot as the
  same `pinky` unix user, so `deploy.sh` passes `DEPLOYED_BY` (git email + your
  hostname) into the updater, and the log records both that and the actual
  login. If `git config user.email` is unset, `deploy.sh` warns and falls back
  to the OS username — set it so the log names a person.
- **The lock records its holder.** Concurrent deploys are refused with the
  holder's name and the lock's age. Older than `PINKY_LOCK_STALE_SECONDS`
  (default 1800) it is broken automatically; `--force-lock` is the explicit
  override.
- **Credentials.** `move_control` is a public repo, so `--from-github` needs no
  token. If it is ever made private, export `GH_TOKEN` — the same code path
  sends it as a bearer token. Never commit the robot's address or keys:
  `deploy.local.env` is git-ignored for exactly that reason.
- **Rollback has limits.** It restores source and rebuilds; it does not restore
  a `config/*.yaml` you hand-edited on the robot, and it cannot undo anything
  the new version wrote outside the package. `calib_node` writes
  `config/auto_calib.yaml` into the *current* version directory, so a rollback
  leaves the older version's calibration in place.
- The first run migrates the existing real `src/move_control` directory to
  `pre-cicd-<timestamp>` so even the first switch has a rollback target.

## Dependencies
- Robot: `bash`, `tar`, `curl`, `python3`, `colcon`, ROS 2 Jazzy.
- PC: `ssh`, `scp`, `git`, plus `tools/ci/make_bundle.sh`.
