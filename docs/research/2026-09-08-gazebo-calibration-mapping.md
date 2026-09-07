# Gazebo auto-calibration and mapping verification

Status: GO for the isolated Gazebo velocity-plant mapping algorithm. HOLD for wheel drivetrain and physical robot validation.

Saved result: [`map/calibrated_2026-09-08/map.yaml`](../../map/calibrated_2026-09-08/map.yaml), with PGM, immutable snapshot hashes, overlay, live applied calibration evidence and `acceptance.json`. At simulation time 2780.2 s the rounded interior unknown fraction was 0.000, corridor purity 0.966, all 48 wall objects were detected, and phantom fraction was 0.000. Full-run sampled travel was 22.719 m, minimum center-to-wall distance 0.12071 m against configured radius 0.105 m. The sole `/cmd_vel` publisher and coherent applied calibration were verified at 2807.965 s, and navigation was stopped normally.

## Isolation and evidence scope

- Worktree: `move_control_rotation`, branch `codex/rotation-calibration`.
- WSL Ubuntu / ROS 2 Jazzy / Gazebo, ROS domain 227, Gazebo partition `pinky_calmap227`.
- Domain 13 / partition `pinky_rig13` is another session and was not changed.
- Maze: repository `tools/gz/pinky_maze.sdf`, 3.9 m square, 30 cm passages.
- Scan: real Gazebo GPU lidar, 720 beams, 10 Hz; SLAM consumes it at 2 cm map resolution.
- IMU and odometry share Gazebo ground-truth pose; US derives from the scan. Camera and IR are synthetic flat-floor inputs. This does not validate independent sensors, real cliff detection, physical stopping response, or camera perception.
- Wheel plant failed the bounded translation trial at 1x (too little displacement); saved `wheel-calibration-failure.json` and `wheel-stack.log` under `/tmp/pinky-calmap227`.
- Algorithm-loop experiment uses Gazebo VelocityControl with wheel links/joints removed and base gravity disabled. It retains base collisions, maze walls and raycast lidar. This cannot validate wheel traction, motor gains or physical robot commissioning.
- VelocityControl API: https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1VelocityControl.html
- Production scan acceptance is unchanged. Only the explicitly isolated tool process accepts Gazebo scans.

## Acceptance

`calibration_mapping_audit.py` measures the complete SDF maze, including space outside the current SLAM raster. Required: interior unknown <= 1%, per-wall recall >= 98%, known corridor purity >= 95%, phantom walls < 2%. The legacy checker permits 40% interior unknown and is insufficient to claim mapping completion.

Final PGM/YAML, measured quality, calibration profile/ack, execution logs and remaining limitations are saved together. Calibration ready or movement alone is not acceptance.

The legacy `wall_recall` is detection per SDF wall object within 10 cm, not a percentage of wall-surface coverage. Map-raster gates alone do not establish collision-free motion or physical drivetrain validity. Final evidence identifies its run, source revision, plant and sampled motion interval. The second read-only audit started at 1817.45 s; its clearance covers that interval, while the original full-run metrics cover earlier motion too. The runner now archives old results and initializes pending evidence before a new run; immutable snapshot hashes bind metrics to saved maps. Coherent live profile authority is checked separately from advisory status.

## Reproduced issues and changes

1. Normal route rotation revoked completed calibration because the IMU callback applied stationary gyro limits at runtime. Fixed while preserving finite-value, gravity, tilt and source-age checks (`99f8d11`; sibling equivalent `77cc7d5`).
2. Front-wall detection prohibited turning away. Integrated sibling `e1a360e` as `06dae20`; final safety gate remains the only motor publisher.
3. Integrated bounded route replanning from sibling `1c2bc03` as `44b8bc5`, then distinct-route entry exclusion from `16db8fb` / `f29f30f` as `fa5c35e`.
4. Review reproduced a permanent recovery latch: planner waiting consumed all physical recovery attempts before temporary exclusions expired. Waiting now holds at zero without consuming another attempt; three actually accepted and failed alternatives still exhaust the push budget. Repeated empty replans no longer extend the same exit exclusion.
5. Coverage selection now evaluates failed-exit avoidance while trying its waypoint candidates, so a blocked first candidate does not starve other directions.
6. Actual scan maps contained only 2–5-cell staircase frontier fragments, all below the 6-cell cluster threshold. Fallback diagonal clustering preserves these when no ordinary cluster survives. Existing orthogonal approach faces and A* no-corner-cut behavior remain intact.
7. A single-executor test adapter starved map TF while planning; split each production node into its own process. The failed trial is preserved in `single-executor-tf-failure.json`.
8. Accelerated execution and concurrent verification load reproduced missed final-command deadlines and a camera source-time rejection. The current retry calibrated at 0.5x with other tests finished, then navigation runs at 1x. No deadline was relaxed. Earlier failed attempts remain in the archive; this is not a first-attempt success.
9. Slow but measured convergence toward a fixed turn bearing now counts as progress. Fresh executor feedback disables the duplicate XY-only planner watchdog (`3de1cce`).
10. Measured lidar translation participates in the safety profile revision and turn envelope. Removed the second legacy clamp that could overwrite validated configured distances. Invalid or lost mount evidence still stops motion (`5e03ab8`).
11. Runtime map availability belongs to route eligibility, while calibration retains physical sensor and geometry checks. Map source timestamps cannot be replayed to renew freshness. Both planner and executor validate TF quaternions. Localization holds pause, but do not replenish, physical progress budgets (`5e03ab8`).
12. At simulation time 443 s a frontier update replaced a westbound observation point with an eastbound one before arrival, producing expensive backtracking. Fresh executor feedback now retains a reachable observation point until arrival; each plan still rechecks map clearance. Recovery, new obstacles, reset, manual selection and expired feedback release retention (`cdbb3d0`). Applied by an explicit goal/wander reload around 640 s while preserving SLAM and calibration. The run is therefore incremental integration evidence, not an untouched first-attempt run. It reached the next observation point and increased known interior to about 25% by 708 s.

## Reproduction

From the worktree in WSL with ROS installed:

```bash
RIG_PLANT=velocity bash tools/gz/run_calibration_mapping.sh
```

The runner owns its partition with flock and separate process groups. Output is `/tmp/pinky-calmap227`. Run `calibration_mapping_audit.py` in domain 227 for independent map capture/quality; `map_run_monitor.py` provides trail rendering. `rig_command.py` can issue operator commands only in the isolated domain and never publishes motor output. Debug navigation reload preserves the current simulator and map but restarts route planning; do not present such an incremental diagnostic as an untouched end-to-end run.

## Verification so far

- At `5e03ab8`: 288 Python/pure/ROS tests passed, including startup, recovery, goal-route, safety, wander, web calibration and parameter preservation.
- Dashboard calibration/mapping: 9 Node tests passed.
- Isolated colcon package build passed.
- Offline planner simulator: 75/75 known, zero unseen; this is not Gazebo completion evidence.
- Read-only review found no blocking issue in mount, map freshness, quaternion or localization-hold changes.
- Final isolated source-copy verification: 293 Python/pure/ROS/tool tests passed in 28.52 s. A bind-mounted rerun was abandoned after confirmed filesystem I/O wait; the same suite plus the new runner regression passed from the container filesystem. Dashboard tests: 9 passed.
- Gazebo velocity-plant mapping acceptance passed. Wheel/physical validation remains HOLD.
