<!-- Parent: ../AGENTS.md -->
<!-- Generated: 2026-09-06 | Updated: 2026-09-06 -->

# launch/ (bringup order matters)

## Purpose
Launch files for the full stack. Bringup order is load-bearing: bringup+ADC first, then safety (+1.5 s) → wander (+3 s) → lcd/web/watch (+3.5 s), all respawn=True.

## Key Files
| File | Description |
|------|-------------|
| `robot.launch.py` | Full stack: imu+camera → safety → wander → lcd+web+watch, all `respawn=True`; loads `robot.yaml` under `/**` first, then per-node yamls |
| `wander.launch.py` | imu+camera+safety+wander only (no lcd/web/watch) |
| `map.launch.py` | slam_toolbox mapping with wander driving |
| `goal.launch.py` | goal_node alone: `/map` + TF → `/goal_point` + `/route` |
| `calib.launch.py` | calibration node |

## For AI Agents

### Working In This Directory
- Preserve the timed sequencing (safety before wander) — safety halts on stale commands; wander before safety would publish raw commands to a dead gate.
- Every launch loads `config/robot.yaml` under the `/**` wildcard **first**, then per-node yamls — keep that order.
- Keep `respawn=True` on the stack nodes.

### Testing Requirements
- Launch files are verified on-robot only; after editing, run `ros2 launch rosy_control robot.launch.py` and watch `/robot/health` + `/robot/mode`.

## Dependencies

### Internal
- `config/` (robot.yaml + per-node), node executables from `setup.py` entry points.

<!-- MANUAL: -->
