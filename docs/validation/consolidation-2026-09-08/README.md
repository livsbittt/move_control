# Local main consolidation verification

Integration parents: main `e87707f` and rotation `8e51b75`. The other session branch tips were already ancestors of main. Rotation had 12 commits outside main's ancestry, including one patch-equivalent commit; its unique behavior was reconciled rather than replaying all older files.

Preserved main's directional mesh clearance, WallTracker precision, adaptive translation stroke, persisted calibration restoration, optional ultrasound, advisory camera, manual goals, map export, reachable coverage, and saved-map localization. Added bilateral rotation validation, atomic gain lease/ack, evidence freshness, final command validation, retained exploration viewpoints, and localization-aware progress timers. Default YAML values are initial parameters; measured calibration saves merge only measured fields atomically.

Verification:

- Windows pure suite: 317 passed, 1 skipped (ROS-only test); dashboard suites: 22 passed.
- Linux pure and goal/recovery/TF/calibration-save/web/manifest suite: 363 passed; see consolidation-tests.log.
- Final Linux safety/startup/wander/atomic adapter suites: 61 passed; see consolidation-final-adapters.log.
- Final colcon build: one package succeeded; see consolidation-build.log.
- Gazebo domain 228, partition pinky_localization228: initial localization, commanded motion, translation teleport, yaw teleport, and scan outage/restoration passed; 19 routes, safety_node the only final command publisher. Ground truth was used for scoring only. See acceptance.json and run_manifest.json for exact run-source hashes.

The Gazebo run began before the final mixed-command tilt reversal guard was tightened; the final guard was subsequently covered by the 61-test adapter run and build. This run validates localization regression, not a fresh complete eight-leg physical calibration or full remapping. Existing saved map artifacts are unchanged. No physical robot deployment or remote push was performed.

Review fixes include pinned geometry identity after calibration, mixed-command trajectory changes stopping rather than reversing, and fresh finite US evidence required to release an existing contact latch. Legacy translation-only certificates remain translation-only and may bind runtime geometry once after their configuration fingerprint passes; new certificates retain runtime geometry identity.

While waiting for a distinct executable alternative, the robot stays stopped and the planner continues periodic replanning. Planning-only waiting does not consume the bounded physical-attempt budget and has no terminal timeout.

Untracked main planning/research documents matched incoming tracked files exactly. Other untracked rotation/release documents and candidate evidence were copied into docs/archive/2026-09-08-session-wip without modifying their originals. Candidate screenshots and paused draft tests are explicitly not acceptance evidence.
