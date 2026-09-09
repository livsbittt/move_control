<!-- Parent: ../AGENTS.md -->
<!-- Generated: 2026-09-06 | Updated: 2026-09-06 -->

# rosy_control/ (Python package)

## Purpose
The package's Python code: executable ROS node wrappers at the top level and the pure-logic "subject" modules grouped into `control/`, `planning/`, `sensing/` subpackages plus `safety/` and `wander/` node packages. Data flow: sensor → geometry (`sensing/`) → policy (`control/`) → map goals (`planning/`).

## Key Files
| File | Description |
|------|-------------|
| `wander_node.py` | Entry wrapper — re-exports `WanderNode`/`main` from `wander/` |
| `safety_node.py` | Entry wrapper for the safety gate node |
| `calib_node.py` | `/calib/step` FSM: floor IR → nudge to solve drive sign + lidar nose yaw → real IR cliff; writes `config/auto_calib.yaml` and pushes params into the running safety node |
| `camera_detect_node.py` | OV5647 capture (BGR8, rotated 180°) → `camera.classify_frame` → `/camera/cliff|blocked|side` |
| `control_node.py` | odom-P straight/rotate controller driven by `/goal_distance`, `/goal_rotate` |
| `goal_node.py` | Map point-to-go: `/map`+TF → `/goal_point`, `/route`; thin I/O over `planning.GoalBrain`; advisory only, never touches `/cmd_vel_raw` |
| `watch_node.py` | Graph health: required nodes, exclusive topic ownership, foreign-node detection → `/robot/ok|health|interrupt` |
| `watch.py` | Pure graph-inspect logic (ROS-free), covered by `test_watch.py` |

## Subdirectories
| Directory | Purpose |
|-----------|---------|
| `control/` | Pure motion policy + the canonical mode label (see `control/AGENTS.md`) |
| `planning/` | Occupancy-grid planning: A*, frontiers, zigzag, GoalBrain (see `planning/AGENTS.md`) |
| `safety/` | The safety gate node subjects (see `safety/AGENTS.md`) |
| `sensing/` | Raw sensor → robot-frame geometry (see `sensing/AGENTS.md`) |
| `wander/` | The wander FSM node (see `wander/AGENTS.md`) |

## For AI Agents

### Working In This Directory
- Top-level `*_node.py` files are thin entry wrappers — put logic in the subject modules, not here.
- Node files are `#!/usr/bin/env python3` executables wired in `setup.py` entry points; keep the `main()` + `rclpy.spin` + `KeyboardInterrupt`-safe shutdown shape.
- On shutdown, every node must leave motors stopped and status topics truthful (see `wander/node.py:stop_motors`).

### Testing Requirements
- Node wrappers are not unit-tested (no ROS in CI-style tests); keep their logic thin.
- Pure modules here (`watch.py`) must stay importable without ROS and tested in `test/`.

### Common Patterns
- Docstring convention: `"""Subject: <one concern>. …"""` — one subject per module/mixin.
- English comments/logs; explain *why* against hardware limits.

## Dependencies

### Internal
- `config/` (loaded via launch, `/**` wildcard first: `robot.yaml`), `launch/` bringup order: bringup+ADC → safety (+1.5 s) → wander (+3 s) → lcd/web/watch (+3.5 s).

### External
- `rclpy`, `std_msgs`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `picamera2` (camera node), `tf_transformations`.

<!-- MANUAL: -->
