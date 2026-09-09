<!-- Parent: ../AGENTS.md -->
<!-- Generated: 2026-09-06 | Updated: 2026-09-06 -->

# config/ (ROS parameters)

## Purpose
All ROS parameters. `robot.yaml` is the **single shared source** loaded first by every launch with the `/**` wildcard so all nodes get the same numbers; per-node yamls load after and override.

## Key Files
| File | Description |
|------|-------------|
| `robot.yaml` | Shared source: stop distances, yaw offsets, cliff thresholds, drive sign, robot radius, speeds |
| `safety.yaml` / `wander.yaml` / `control.yaml` / `camera.yaml` / `goal.yaml` / `mapper.yaml` | Per-node overrides (load after `robot.yaml`) |
| `cliff_calib.yaml` | Measured IR cliff thresholds; 4095 = ADC saturation when lifted — never a cliff |
| `auto_calib.yaml` | Machine-written by `calib_node` — do not hand-edit |

## For AI Agents

### Working In This Directory
- Changing a shared number: edit `robot.yaml`, not a per-node yaml, unless the node must differ.
- Speeds are deliberately tiny (cruise 1.4 cm/s, think 3 mm/s) for a desk maze; stop distances (1.8–2 cm) are **sensor clearance, not map size**.
- Machine-written files (`auto_calib.yaml`) carry timestamps from real calibration runs.

### Testing Requirements
- No unit tests cover yaml; after param changes, rebuild (`colcon build --packages-select rosy_control lcd_control`) and re-run the on-robot smoke echo checks.

### Common Patterns
- Launch order: `robot.yaml` first (`/**`), then per-node yaml.

<!-- MANUAL: -->
