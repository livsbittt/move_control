<!-- Parent: ../AGENTS.md -->
<!-- Generated: 2026-09-06 | Updated: 2026-09-06 -->

# test/ (pure-logic suite)

## Purpose
ROS-free unittest suite covering every pure-logic decision module. No ROS on the dev machine — numpy/opencv only.

## Key Files
| File | Covers |
|------|--------|
| `test_modes.py` | `control/modes`: label precedence, ESTOP reachable, WARN band edges, `nose_on_wall`, US no-echo/0.80 cap |
| `test_recover.py` | `control/recover`: `hazard_action` matrix, `wall_first_move`, turn-sign rules, stuck/backup policy |
| `test_planning.py` | `planning` (GoalBrain): unreachable waypoint skip, degenerate goals, explore→coverage |
| `test_frontier.py` | `planning/frontier` |
| `test_body.py` / `test_filt.py` | `sensing/body`, `sensing/filt` |
| `test_route.py` | `control/route` |
| `test_scale.py` | `safety/scale` corridor auto-scale |
| `test_watch.py` | `watch.py` graph inspect |

## For AI Agents

### Working In This Directory
- Mirrors the module tree: a new decision module gets `test_<module>.py`.
- Unittest style (`unittest.TestCase`), plain asserts with boundary comments (see `test_recover.py`).

### Testing Requirements
- `python3 -m pytest test/ -q` (all, ~75 tests) — keep green before committing.
- `python3 -m pytest test/test_recover.py -k backup` to run one test.

### Common Patterns
- Tests encode *hardware rationale*: think-speed crawls can't cover 8 mm in 1.2 s (not stuck); 4095 IR = saturation, never a cliff.

<!-- MANUAL: -->
