# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

`move_control` — a ROS 2 Jazzy ament_python package for the **Pinky Pro** robot: a ~11 cm desk-maze robot (Raspberry Pi, RPLidar C1, US-016 ultrasonic, 3-channel IR cliff sensors, BNO055 IMU, OV5647 camera). It provides the wander autonomy, the safety velocity gate, camera look-ahead, auto-calibration, SLAM mapping, and a node-graph health monitor.

This repo is deployed to the robot at `/home/pinky/dev_ws/wj/src/move_control` (see `STEPS.txt`). Real-hardware only: remote gazebo `/scan` messages are detected and deliberately rejected (`lidar.is_robot_scan` — beam count + range_max + wall-clock stamp).

## Commands

Environment (each new terminal, on the robot):
```bash
source /opt/ros/jazzy/setup.bash
source /home/pinky/pinky_pro/install/setup.bash
source /home/pinky/dev_ws/wj/install/setup.bash
```

Build (in the robot workspace `/home/pinky/dev_ws/wj`):
```bash
colcon build --packages-select move_control lcd_control
```

Tests — pure-logic unittest suite (36 tests, no ROS needed; run from repo root, `python3-numpy`/`python3-opencv` required):
```bash
python3 -m pytest test/ -q                          # all
python3 -m pytest test/test_recover.py -q           # one file
python3 -m pytest test/test_recover.py -k backup    # one test
```

Run (bringup order matters — bringup+ADC first):
```bash
ros2 launch pinky_bringup bringup_robot.launch.xml   # bringup (lidar+motors)
ros2 run pinky_sensor_adc main_node                  # ADC (IR cliff / US)
ros2 launch move_control robot.launch.py             # full stack: imu+camera→safety→wander→lcd+web+watch
# or wander.launch.py (imu+camera+safety+wander only)
ros2 launch move_control map.launch.py               # slam_toolbox mapping
ros2 launch move_control calib.launch.py             # calibration node
```

Drive the calibrator and wanderer by topic:
```bash
ros2 topic pub --once /calib/step  std_msgs/msg/String "{data: auto}"    # auto|lidar|floor|cliff|compute|abort
ros2 topic pub --once /wander/cmd  std_msgs/msg/String "{data: stop}"    # stop|start — motors only, node stays up
ros2 topic pub --once /goal_distance std_msgs/msg/Float64 "{data: 0.2}"  # control_node straight test
ros2 topic pub --once /goal_rotate   std_msgs/msg/Float64 "{data: 90.0}" # control_node rotate test
```

Observe: `/wander/state`, `/safety/mode`, `/robot/mode` (same label), `/robot/health`, `/camera/debug`, `ros2 pkg executables move_control`.

There is no linter configured. LCD/LED/web live in **other packages** (`lcd_control`, `pinky_web`) — not in this repo.

## Architecture

### Command chain: wander → /cmd_vel_raw → safety gate → /cmd_vel

`wander_node` and `control_node` never publish to the motor topic. They publish **semantic** velocity (positive x = nose-forward) on `/cmd_vel_raw`; `safety_node` is the **only** publisher of `/cmd_vel` and re-publishes the last raw command every 20 ms tick unless it halts (obstacle, cliff, tilt, pickup, e-stop, stale command > 0.5 s). Safety also applies the physical `cmd_linear_sign` flip. If safety dies, no one publishes `/cmd_vel` — the robot stops. `/estop` (latched QoS) or `/estop/cmd` kill everything immediately.

### Nodes and their subjects

Each ROS node is an `rclpy.Node` composed of **subject mixins**, one concern each:

- `wander_node` = `Senses | Judge | Contact | Motion` + FSM in `wander/node.py`. States: `wait forward pause look calc recon wall backup turn escape stop` (one 20 ms `tick()` dispatcher). Behavior: IR cliff → pause → back until IR clears → turn; wall/camera block → `look` (median L/R/F samples) → `calc` (score openings) → locked turn or backup; `escape` for maze corners; stall detection flips the escape sign after 2 failures.
- `safety_node` = `Bumper | Hazard | Gate | Scale`. Fuses lidar sectors, US, IR, IMU, camera; publishes ~20 `/safety/*` range/bool topics that wander consumes; auto-scales the narrow-maze HUD (`map_range`/`open_max`) from live corridor width L+R.
- `calib_node` — `/calib/step` FSM: stable floor IR → slow ±x nudge to solve `cmd_linear_sign` + lidar nose yaw → wait for a real IR cliff. Writes `config/auto_calib.yaml` **and** pushes params into the running safety_node via the `SetParameters` service.
- `camera_detect_node` — OV5647 via picamera2, treated as **BGR8** (libcamera RGB888 is BGR in memory), frame rotated 180°. HSV floor/void/obstacle classification in `camera.py:classify_frame` → `/camera/cliff`, `/camera/blocked`, `/camera/side`.
- `control_node` — odom-P-controller for straight/rotate goals (published as raw `Float64` on `/goal_distance`, `/goal_rotate`).
- `watch_node` — graph health: required node set, **exclusive topic ownership** (`/cmd_vel`→safety_node, `/cmd_vel_raw`→wander_node, `/scan`→sllidar_node), foreign-node detection (gazebo bridges). Publishes `/robot/ok|health|interrupt`; `once:=true` exits non-zero on interrupts.

### Pure-logic modules (no ROS imports — what the tests cover)

`recover.py` (stuck/backup/escape policy), `route.py` (longest free straight line), `modes.py` (the one canonical `/robot/mode` label: hazard > wander action > contact bands; wander publishes it, LCD only displays), `lidar.py`, `body.py`, `filt.py`, `camera.py`, `watch.py`. Keep new decision logic here and it stays unit-testable.

### Lidar geometry — the one non-obvious trap

The C1 lidar is mounted rotated: **scan angle 0 is the robot's rear; the nose is ≈190°** (`NOSE_YAW = π + 10°` in `lidar.py`, param `lidar_yaw_offset`). Every heading must go through `robot_yaw()` / `wrap_pi()`. Sector ranges use a 10th percentile (`scan_pctl`) to kill single-beam spikes and `ignore_m` (`body.py`) to drop chassis hits.

### Parameters

`config/robot.yaml` is the single shared source (stop distances, yaw offsets, cliff thresholds, drive sign, robot radius) — loaded **first** by every launch with the `/**` wildcard so all nodes get the same numbers. Per-node yaml (`safety.yaml`, `wander.yaml`, `control.yaml`, `camera.yaml`) loads after and overrides. `cliff_calib.yaml` holds measured cliff IR thresholds (4095 = ADC saturation when lifted, never a cliff); `auto_calib.yaml` is machine-written by calib_node. Speeds are deliberately tiny (cruise 1.4 cm/s, think 3 mm/s) for a desk maze; stop distances (1.8–2 cm) are **sensor clearance, not map size**.

## Conventions

- Docs/README/STEPS.txt are in Korean; code comments and log messages are English.
- Commit messages are short, imperative summaries of the behavioral fix (see `git log`).
- Comments explain *why* against measured hardware limits (lidar 5 cm min, US-016 2 cm blind zone, IR 4095 saturation) — keep that style when touching tuning constants.
