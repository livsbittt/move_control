<!-- Generated: 2026-09-06 | Updated: 2026-09-06 -->

# move_control (Pinky Pro)

## Purpose
ROS 2 Jazzy ament_python package for the **Pinky Pro** desk-maze robot (~11 cm; RPi, RPLidar C1, US-016, 3× IR cliff, BNO055 IMU, OV5647). Provides wander autonomy, the safety velocity gate, camera look-ahead, auto-calibration, SLAM mapping, map-driven goals, and a node-graph health monitor. Real hardware only — remote gazebo `/scan` messages are detected and rejected (`sensing/lidar.is_robot_scan`).

## Key Files
| File | Description |
|------|-------------|
| `CLAUDE.md` | Deep architecture guide (command chain, lidar trap, parameters) — read first |
| `STEPS.txt` | Korean operator runbook: bringup order, tuning log, LCD/web notes |
| `package.xml` / `setup.py` | ament_python package manifest and entry points |
| `resource/move_control` | ament resource marker (generated, do not edit) |

## Subdirectories
| Directory | Purpose |
|-----------|---------|
| `move_control/` | Python package: ROS nodes + pure-logic subjects (see `move_control/AGENTS.md`) |
| `config/` | Shared and per-node ROS parameters (see `config/AGENTS.md`) |
| `launch/` | Launch files; bringup order matters (see `launch/AGENTS.md`) |
| `test/` | Pure-logic unittest suite, no ROS needed (see `test/AGENTS.md`) |
| `tools/` | Offline ASCII simulator (see `tools/AGENTS.md`) |
| `web/` | web_node dashboard UI — served from share/move_control/web (see `web/AGENTS.md`) |
| `map/` | Gazebo world asset for the desk maze |

## For AI Agents

### Working In This Directory
- Keep new **decision logic in the pure-logic subjects** (`move_control/control/`, `move_control/planning/`, `move_control/sensing/`, `move_control/watch.py`) — no ROS imports there, that is what the tests cover.
- `config/robot.yaml` is the single shared parameter source; per-node yamls override after it.
- The lidar is mounted rotated: **scan 0° = rear, nose ≈ 190°**. Every heading goes through `robot_yaw()` / `wrap_pi()`.
- Docs/STEPS.txt are Korean; code comments and logs are English. Comments explain *why* against measured hardware limits (lidar 5 cm min, US 2 cm blind zone, IR 4095 = ADC saturation, never a cliff).
- Commits: short imperative behavioral summaries.

### Testing Requirements
- `python3 -m pytest test/ -q` from repo root (75 tests; needs `python3-numpy`/`python3-opencv`, no ROS).
- Keep the suite green before committing; add tests for new pure-logic decisions.

### Common Patterns
- Subject mixins: each node = `rclpy.Node` + one-concern mixins (wander = Senses/Judge/Contact/Motion; safety = Bumper/Hazard/Gate/Scale).
- Command chain: wander → `/cmd_vel_raw` → **safety gate (only `/cmd_vel` publisher)** → motors. Never publish `/cmd_vel` directly.
- One fused status label (`control/modes.pick_mode`) on `/robot/mode`; `/safety/mode` is a deprecated same-value alias.

## Dependencies

### External
- ROS 2 Jazzy (`rclpy`, `std_msgs`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `tf_transformations`), `slam_toolbox` for mapping.

<!-- MANUAL: -->
