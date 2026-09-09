<!-- Parent: ../AGENTS.md -->
<!-- Generated: 2026-09-06 | Updated: 2026-09-06 -->
# tools/ (offline simulator)

## Purpose
ASCII simulator of the full explore→coverage pipeline (frontier explore → zigzag coverage) without ROS. Reuses `rosy_control/planning` directly.

## Key Files
| File | Description |
|---|---|
| `explore_sim.py` | ASCII sim: `--quiet` mode for CI-style checks; exits non-zero on failure |
| `synth_rig.py` | Synthetic web-test rig: maze /map + animated /odom + /scan (inf beams included) on plain rclpy — no gz/bridge, survives the dev machine's process reaper. Integrates /cmd_vel and follows /route so the :28161 web can be tested end-to-end (run goal_node beside it) |
| `__pycache__/` | generated |

## For AI Agents

### Working In This Directory
- The sim must stay import-side-effect-free; it imports `rosy_control.planning` (pure logic) only.
- Use it to sanity-check planning changes: `python3 tools/explore_sim.py --quiet`.

## Dependencies

### Internal
- `rosy_control/planning` — shares GoalBrain with goal_node.

<!-- MANUAL: -->
