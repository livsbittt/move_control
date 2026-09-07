<!-- Parent: ../AGENTS.md -->
<!-- Generated: 2026-09-06 | Updated: 2026-09-06 -->

# safety/ (the only /cmd_vel publisher)

## Purpose
The safety velocity gate: fuses lidar sectors, US, IR, IMU, camera into ~20 `/safety/*` topics, halts/reverses `/cmd_vel` on hazard, applies the physical drive-sign flip, and auto-scales the narrow-maze HUD. If this node dies, nobody publishes `/cmd_vel` — the robot stops.

## Key Files
| File | Description |
|------|-------------|
| `node.py` | SafetyNode(Node, Bumper, Hazard, Gate, Scale), 20 ms tick; latched `/estop/state` (depth 1, RELIABLE, TRANSIENT_LOCAL); dual same-value aliases `/safety/can_reverse` ≡ `/safety/rear_clear` (deprecated pattern, like `/safety/mode`) |
| `bumper.py` | Lidar sector ranges (10th-percentile to kill single-beam spikes), US, rear sectors, full-circle exit judge (best all-around gap) → `/safety/*` range topics incl. `/safety/exit_yaw`/`exit_range` |
| `hazard.py` | ESTOP/CLIFF/TILT/PICK detection: latched e-stop (`/estop`, `/estop/cmd`), `ir_looks_like_cliff` (4095-saturation ignore + hysteresis); on engage zeros commands and cross-publishes `/wander/cmd 'stop'` + `/calib/step 'abort'`
| `gate.py` | The command gate: halt on obstacle/cliff/tilt/pickup/estop/stale>0.5 s, tilt-reverse creep, `cmd_linear_sign` flip, re-publishes last raw command every 20 ms |
| `scale.py` | Narrow-map auto-cal: corridor width L+R → `map_range`/`open_max` HUD scaling |

## For AI Agents

### Working In This Directory
- Safety is the **only** publisher of `/cmd_vel`; wander/control publish semantic `/cmd_vel_raw`. Never add another `/cmd_vel` publisher.
- Hazard *detection* lives here; hazard *response policy* lives in `control/recover.hazard_action` — don't duplicate.
- The e-stop chain crosses nodes by design (engage → wander stop + calib abort); keep it intact when touching `hazard.py`.
- Latched QoS (TRANSIENT_LOCAL depth 1) is required on `/estop/state` so late joiners see the latch.

### Testing Requirements
- `python3 -m pytest test/test_scale.py -q` covers `scale.py`; the gate/hazard logic is hardware-coupled and verified on-robot.
- Thresholds come from `config/robot.yaml` + `config/cliff_calib.yaml` (4095 = ADC saturation, never a cliff).

### Common Patterns
- 10th-percentile sector ranges (`scan_pctl`) to kill single-beam spikes; `ignore_m` drops chassis hits.
- Comments cite measured hardware limits (lidar 5 cm min, US 2 cm blind zone).

## Dependencies

### Internal
- Publishes the `/safety/*` vocabulary wander consumes (`control/` policies make decisions from it).
- Cross-publishes `/wander/cmd`, `/calib/step` on e-stop.

### External
- `rclpy.qos` (latched), `sensor_msgs` LaserScan/Range/Imu.

<!-- MANUAL: -->
