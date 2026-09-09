<!-- Parent: ../AGENTS.md -->
<!-- Generated: 2026-09-06 | Updated: 2026-09-06 -->

# wander/ (the autonomy FSM)

## Purpose
The wander autonomy: a 20 ms FSM (`wait forward pause look calc recon wall backup turn escape stop`) over subject mixins. Publishes semantic velocity (positive x = nose-forward) to `/cmd_vel_raw`, never `/cmd_vel`.

## Key Files
| File | Description |
|------|-------------|
| `node.py` | `WanderNode(Node, Senses, Judge, Contact, Motion)` — FSM tick dispatcher, `_announce` pairs `/wander/state` + `/robot/mode` (one fused label from `control/modes.pick_mode`; `/safety/mode` is a deprecated same-value alias), shared FSM entries (`_hold`/`_start_backup`/`_start_escape`/`_start_turn`/`_resume_forward`), `stop_motors()` publishes the fused label so shutdown never leaves a stale label |
| `senses.py` | Topic callbacks + geometry helpers: `_on_wall` (delegates to `modes.nose_on_wall`), `_pick_turn_sign` (via `recover.side_sign`/`ratio_sign`), `_ir_ready` floor-band check |
| `judge.py` | LOOK/CALC/RECON/PAUSE: look sampling → `_calc_plan` (kind/sign/why), `_commit_plan` enters the plan; hazard answers come from `recover.hazard_action` |
| `contact.py` | WALL/BACK: `_can_reverse`, `_back_cmd`, `_tick_wall` (policy via `wall_first_move`), `_tick_backup` (raw hazard persistence → disable) |
| `motion.py` | FWD/ESCAPE/TURN + stuck detection: speeds/blends, `_recover_stuck` (`recover.stuck_kind`/`stuck_flip`, failed-exit bench), escape steers to the latched full-circle exit (`recover.ExitSteer`; `exit_steering=false` = legacy fixed-sign spin), escape auto-sensitivity |

## For AI Agents

### Working In This Directory
- Cliff/tilt decisions go through `control/recover.hazard_action` — do not re-branch on `self.cliff`/`self.tilt` in tick handlers.
- State and mode are always published together (`_announce`/`_publish_mode`); `/safety/mode` fan-out is a deprecated alias line — keep it single-site.
- Cliff only counts after the first forward drive (`seen_forward` gate lives inside `hazard_action`); raw `self.cliff` remains only where the comment says so (backup persistence, forward interrupt).
- Enter states via the shared helpers; the only forward entry is `_resume_forward`.

### Testing Requirements
- The FSM is hardware-coupled (not unit-tested); keep new decisions in `control/recover.py`/`control/modes.py` where they are.
- After FSM changes, on-robot smoke: echo `/wander/state` + `/robot/mode` through drive → wall → cliff → estop → pickup; Ctrl+C must leave `STOP`.

### Common Patterns
- One concern per mixin, `"""Subject: …"""` docstrings; transitions via `_enter` (logs + per-state resets).

## Dependencies

### Internal
- `control/modes.pick_mode`, `control/recover.*`, `sensing/lidar.wrap_pi`.

### External
- `rclpy.qos` (latched `/estop/state` subscription — label only; safety owns the e-stop decision).

<!-- MANUAL: -->
