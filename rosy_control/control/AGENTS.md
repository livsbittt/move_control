<!-- Parent: ../AGENTS.md -->
<!-- Generated: 2026-09-06 | Updated: 2026-09-06 -->

# control/ (motion policy + mode label)

## Purpose
Robot-frame geometry → motion policy. Pure logic, **no ROS imports** — the unit-test-covered decision layer shared by wander/safety. One concern per module.

## Key Files
| File | Description |
|------|-------------|
| `modes.py` | The one canonical `/robot/mode` label: 16 labels, precedence hazard > wander action > contact bands; `pick_mode()` fuses hazard bools + FSM state + `nose_on_wall()`; `NON_FORWARD_STATES` single-sources the state list; `US_NOSE_MAX_M`/`LIDAR_NOSE_MAX_M` hold the hardware cutoffs |
| `recover.py` | Stuck/backup/escape policy + `hazard_action` (the one cliff/tilt answer: tilt always trusted, cliff only after first forward drive, rear clear → backup else spin) + `wall_first_move` + turn-sign rules `side_sign`/`ratio_sign` + `ExitSteer`/`turn_toward_sign` (escape spins the shortest way to the full-circle exit, benches failed bearings) |
| `route.py` | Longest free straight line through sector ranges (line route, not circular) |

## For AI Agents

### Working In This Directory
- Functions take floats/bools and return strings/floats/bools — no `rclpy`, no topics, no node state. If you need node state, the caller passes it in.
- `hazard_action` is the only cliff/tilt decision home: **do not re-branch on `self.cliff`/`self.tilt` in FSM code** — call the policy.
- Label list lives once in `modes.MODES`; `WANDER_TO_MODE` maps FSM state names → labels.
- New labels must keep the taxonomy: hazard > action > contact > idle, subjects do not overlap.

### Testing Requirements
- `python3 -m pytest test/test_modes.py test/test_recover.py test/test_route.py -q`
- New policy → new pure test with boundary values (see `test_recover.py` style).

### Common Patterns
- Thresholds as module constants with hardware rationale comments (`US_NOSE_MAX_M = 0.80` — beyond ~0.8 m a US-016 echo is noise).
- NaN/None-safe float handling (`_hit`, `ratio_sign` treat no-echo as no-opinion).

## Dependencies

### Internal
- Consumed by `rosy_control/wander/` (FSM + label), `rosy_control/safety/` (gate/hazard respond to the same thresholds).

### External
- `dataclasses`, `math` only.

<!-- MANUAL: -->
