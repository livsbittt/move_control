# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

`rosy_control` — a ROS 2 Jazzy ament_python package for the **Pinky Pro** robot: a ~11 cm desk-maze robot (Raspberry Pi, RPLidar C1, US-016 ultrasonic, 3-channel IR cliff sensors, BNO055 IMU, OV5647 camera). It provides the wander autonomy, the safety velocity gate, camera look-ahead, auto-calibration, SLAM mapping, and a node-graph health monitor.

This repo is deployed to the robot at `/home/pinky/dev_ws/wj/src/rosy_control` (see `STEPS.txt`). Production rejects remote gazebo `/scan` (`lidar.is_robot_scan` — beam count + range_max + wall-clock stamp). Isolated Gazebo rigs call `enable_simulation_scans()` so the same predicate feeds bumper, frontiers, routes and the dashboard.

## Commands

Environment (each new terminal, on the robot):
```bash
source /opt/ros/jazzy/setup.bash
source /home/pinky/pinky_pro/install/setup.bash
source /home/pinky/dev_ws/wj/install/setup.bash
```

Build (in the robot workspace `/home/pinky/dev_ws/wj`):
```bash
colcon build --packages-select rosy_control lcd_control
```

Tests — pure-logic unittest suite (89 tests, no ROS needed; run from repo root, `python3-numpy`/`python3-opencv` required):
```bash
python3 -m pytest test/ -q                          # all
python3 -m pytest test/test_recover.py -q           # one file
python3 -m pytest test/test_recover.py -k backup    # one test
```

Run (bringup order matters — bringup+ADC first):
```bash
ros2 launch pinky_bringup bringup_robot.launch.xml   # bringup (lidar+motors)
ros2 run pinky_sensor_adc main_node                  # ADC (IR cliff / US)
ros2 launch rosy_control robot.launch.py             # full stack: imu+camera→safety→wander→lcd+web+watch
# or wander.launch.py (imu+camera+safety+wander only)
ros2 launch rosy_control map.launch.py               # slam_toolbox mapping
ros2 launch rosy_control goal.launch.py              # goal node: /map -> /goal_point + /route
ros2 launch rosy_control web.launch.py               # web_node: map + control dashboard on :28161 (api :28162)
python3 tools/explore_sim.py --quiet                  # ASCII sim: frontier explore -> zigzag coverage
ros2 launch rosy_control calib.launch.py             # calibration node
```

Drive the calibrator and wanderer by topic:
```bash
ros2 topic pub --once /calib/step  std_msgs/msg/String "{data: auto}"    # auto|lidar|floor|cliff|compute|abort
ros2 topic pub --once /wander/cmd  std_msgs/msg/String "{data: stop}"    # stop|start — motors only, node stays up
ros2 topic pub --once /goal_distance std_msgs/msg/Float64 "{data: 0.2}"  # control_node straight test
ros2 topic pub --once /goal_rotate   std_msgs/msg/Float64 "{data: 90.0}" # control_node rotate test
```

Observe: `/wander/state` (FSM verb), `/robot/mode` (canonical fused label), `/safety/mode` (deprecated alias, same label), `/robot/health`, `/camera/debug`, `/goal_point`, `/route`, `/goal/options`, `/goal_node/state`, `ros2 pkg executables rosy_control`. The `web_node` dashboard (:28161, `web.launch.py`) draws the live map + trail and these labels, with goal/wander/estop buttons and gate-safe teleop.

There is no linter configured. LCD/LED live in **other packages** (`lcd_control`, `pinky_web`) — not in this repo; the web dashboard (`web_node`, :28161) is part of this package, and `robot.launch.py` starts it in place of pinky_web.

## Architecture

### Command chain: wander → /cmd_vel_raw → safety gate → /cmd_vel

`wander_node` and `control_node` never publish to the motor topic. They publish **semantic** velocity (positive x = nose-forward) on `/cmd_vel_raw`; `safety_node` is the **only** publisher of `/cmd_vel` and re-publishes the last raw command every 20 ms tick unless it halts (obstacle, cliff, tilt, pickup, e-stop, stale command > 0.5 s). Safety also applies the physical `cmd_linear_sign` flip. If safety dies, no one publishes `/cmd_vel` — the robot stops. `/estop` (latched QoS) or `/estop/cmd` kill everything immediately.

### Nodes and their subjects

Each ROS node is an `rclpy.Node` composed of **subject mixins**, one concern each:

- `wander_node` = `Senses | Judge | Contact | Motion` + FSM in `wander/node.py`. States: `wait forward pause look calc recon wall backup turn escape stop` (one 20 ms `tick()` dispatcher). Behavior: IR cliff → pause → back until IR clears → turn; wall/camera block → `look` (median L/R/F samples) → `calc` (score openings) → locked turn or backup; `escape` for maze corners — steered to the full-circle exit safety publishes on `/safety/exit_yaw`/`exit_range` (shortest-way spin, failed bearings benched near the stuck spot; `exit_steering:=false` restores the legacy fixed-sign spin + timeout flips). Narrow mode scales behavior to the measured clearance (`/safety/narrow`): speed caps toward think speed, frontier gate floors at `wall_front`. Wander subscribes latched `/estop/state` for the **label only** — safety owns the e-stop decision. State and mode are always published together (`_announce`); FSM entries go through shared helpers (`_hold` / `_start_backup` / `_start_escape` / `_start_turn` / `_resume_forward`).
- `safety_node` = `Bumper | Hazard | Gate | Scale`. Fuses lidar sectors, US, IR, IMU, camera; publishes ~20 `/safety/*` range/bool topics that wander consumes; auto-scales the narrow-maze HUD (`map_range`/`open_max`) from live corridor width L+R, and publishes `/safety/narrow` = corridor − 2×robot_radius (negative = open) for wander's narrow-mode scaling.
- `calib_node` — `/calib/step` FSM: stable floor IR → slow ±x nudge to solve `cmd_linear_sign` + lidar nose yaw → wait for a real IR cliff. Writes `config/auto_calib.yaml` **and** pushes params into the running safety_node via the `SetParameters` service.
- `camera_detect_node` — OV5647 via picamera2, treated as **BGR8** (libcamera RGB888 is BGR in memory), frame rotated 180°. Settles AE/AWB then **freezes** `ExposureTime`/`AnalogueGain`/`ColourGains` (`sensing/camera_controls.py`) — the ISP's Bayesian AWB re-decides every 10 frames and its documented weakness is "large objects of a uniform but non-grey colour", i.e. the floor; unfrozen, a 1.3× gain step on one unchanged frame flipped `blocked`. HSV floor/foreground classification in `camera.py:classify_frame` (stateless, one frame in), hysteresis/warmup/blindness-hold in `sensing/camera_policy.py` (shared with `tools/gz/rendered_camera_adapter.py`, which used to keep its own and diverge). Publishes `/camera/blocked`, `/camera/side`, `/camera/observation` (compact evidence, schema in `sensing/camera_evidence.py`), `/camera/controls`. **`/camera/cliff` is always false**: a single camera cannot separate a dark wall from a dark hole at the same ground line — both appearance tests were measured wrong on the recorded frames — so floor IR keeps the cliff decision and the darkness is published as evidence only. Region `distance_m` stays `null` until the six `camera_*` calibration parameters are measured (`docs/camera-ground-calibration.md`); `sensing/camera_ground.py` ranges a region at its **bottom** edge and refuses anything at the horizon or past `camera_max_range_m`.
- `control_node` — odom-P-controller for straight/rotate goals (published as raw `Float64` on `/goal_distance`, `/goal_rotate`).
- `goal_node` — map-driven point to go: `/map` + TF map→base_link (odom fallback) → `/goal_point` (PoseStamped), `/route` (Path), `/goal_node/state`; `/goal/cmd` explore|coverage|stop. explore = frontier goal scored by unknown-gain ÷ route length; `/goal/options` MarkerArray shows the ranked alternatives (marker 0 = chosen); no frontier left → coverage = zigzag waypoint queue. Stall watchdog: <`progress_m` approach over `stall_plans` plan calls benches a frontier (`blacklist_plans` memory) and the next-best option becomes the goal, planned wide-first (`escape_clear_m` 2-cell inflation seals 15 cm gaps) until the robot leaves the stuck area. `/goal/eta` (Float32 s) = route length ÷ odom-derived speed. Thin I/O over `planning.GoalBrain` (explore→coverage FSM lives in `planning/goals.py`). Advisory only — never touches `/cmd_vel_raw`; motors stay with wander/control behind the safety gate.
- `web_node` — browser view + command relay, no decision logic: /map → PNG + /camera/front → JPEG (both gen-counted, stale-while-revalidate on the page), pose/trail from /odom, labels + safety-fused sensor values polled as JSON at 333 ms; posts relay to the documented surfaces (`/goal/cmd` incl. `x,y` manual goals, `/wander/cmd`, `/estop/cmd`); teleop always `/cmd_vel_raw` (never around the gate). Isolated Gazebo uses the same gauges; `/robot/evidence_scope` only labels synthetic auxiliary sensors. Map canvas has wheel zoom / drag pan / fit (overlays screen-constant).
- `watch_node` — graph health: required node set, **exclusive topic ownership** (`/cmd_vel`→safety_node, `/cmd_vel_raw`→wander_node, `/scan`→sllidar_node), foreign-node detection (gazebo bridges). Publishes `/robot/ok|health|interrupt`; `once:=true` exits non-zero on interrupts.

### Pure-logic modules (no ROS imports — what the tests cover)

Decision logic lives in subject subpackages, each with a facade `__init__`, mirroring the data flow sensor → geometry → policy → map:

- `sensing/` — raw data → robot-frame geometry: `lidar` (C1 scan geometry, the nose-yaw trap, sector ranges, frontiers), `filt` (median/low-pass), `body` (URDF footprint constants, chassis-hit ignore), `camera` (HSV floor/foreground classification, stateless), `camera_policy` (hysteresis + warmup + blindness hold), `camera_controls` (the AE/AWB freeze, pure — no picamera2 import), `camera_evidence` (the one `/camera/observation` serialiser), `camera_ground` (image row → metres on the floor; returns `None`, never a guess, without a measured calibration).
- `control/` — geometry → motion policy: `recover` (stuck/backup/escape policy + `hazard_action` — tilt is always trusted, cliff only after the first forward drive, rear clear → backup else spin, one cliff/tilt answer for every FSM state; also `wall_first_move` and the turn-sign rules `side_sign`/`ratio_sign`), `route` (longest free straight line), `modes` (the one canonical `/robot/mode` label: hazard > wander action > contact bands; wander publishes it — and `/safety/mode` only as a deprecated same-value alias for the LCD — so ESTOP is reachable via the latched `/estop/state` subscription; state = what the FSM is doing, mode = why).
- `planning/` — map → goals: `gridmap`/`astar`/`frontier`/`zigzag` + `goals` (GoalBrain explore→coverage FSM shared by goal_node and the sim; `test_planning.py`).
- `watch.py` — graph health inspect (single consumer, stays top-level).

Keep new decision logic in these and it stays unit-testable; tests mirror the modules (`test_recover.py`, `test_planning.py`, ...).

### Lidar geometry — the one non-obvious trap

The C1 lidar is mounted rotated: **scan angle 0 is the robot's rear; the nose is ≈190°** (`NOSE_YAW = π + 10°` in `lidar.py`, param `lidar_yaw_offset`). Every heading must go through `robot_yaw()` / `wrap_pi()`. Sector ranges use a 10th percentile (`scan_pctl`) to kill single-beam spikes and `ignore_m` (`body.py`) to drop chassis hits.

### Parameters

`config/robot.yaml` is the single shared source (stop distances, yaw offsets, cliff thresholds, drive sign, robot radius) — loaded **first** by every launch with the `/**` wildcard so all nodes get the same numbers. Per-node yaml (`safety.yaml`, `wander.yaml`, `control.yaml`, `camera.yaml`) loads after and overrides. `cliff_calib.yaml` holds measured cliff IR thresholds (4095 = ADC saturation when lifted, never a cliff); `auto_calib.yaml` is machine-written by calib_node. Speeds are deliberately tiny (cruise 1.4 cm/s, think 3 mm/s) for a desk maze; stop distances (1.8–2 cm) are **sensor clearance, not map size**.

## Conventions

- Docs/README/STEPS.txt are in Korean; code comments and log messages are English.
- Commit messages are short, imperative summaries of the behavioral fix (see `git log`).
- Comments explain *why* against measured hardware limits (lidar 5 cm min, US-016 2 cm blind zone, IR 4095 saturation) — keep that style when touching tuning constants.
