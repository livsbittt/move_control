<!-- Parent: ../AGENTS.md -->
<!-- Generated: 2026-09-06 | Updated: 2026-09-06 -->

# planning/ (map → goals)

## Purpose
Occupancy-grid planning: map check → point to go, best route, coverage. Pure logic (no ROS) shared by `goal_node` and the offline sim `tools/explore_sim.py`. GoalBrain is the explore→coverage FSM.

## Key Files
| File | API |
|------|-----|
| `gridmap.py` | `OccupancyMap` — OccupancyGrid-shaped grid (−1 unknown / 0..64 free / ≥65 occ), world↔grid transforms, flood fill, inflate (OCC-only, never unknown), ASCII render |
| `astar.py` | `best_route(map, start, goal, clear_m)` — 8-connected A*, no corner cutting, unknown blocks, returns world points + length |
| `frontier.py` | `frontier_points(map)`, `pick_goal(map)` — free cells touching unknown, clusters, snaps centroid to free, biggest first |
| `zigzag.py` | `ZigzagPlanner` — boustrophedon coverage lanes along the longer axis; covered cells excluded before lane build so replanning advances |
| `goals.py` | `GoalBrain` — explore (frontier goal) → coverage (zigzag waypoints) FSM; map+pose in, point-to-go + route out |

## For AI Agents

### Working In This Directory
- Inflation grows only from OCC cells: frontier cells hug unknown space, so inflating unknown would close every frontier; unknown still blocks A* by absence.
- Robot radius via `clear_m`/inflation so the 5 cm grid can't hug corners (~15 cm robot).
- `ZigzagPlanner(covered=...)` semantics: pass a fresh covered set per replan to advance coverage.

### Testing Requirements
- `python3 -m pytest test/test_planning.py test/test_frontier.py -q`
- Grid fixtures are small ASCII maps; keep tests ROS-free.

### Common Patterns
- Facade `__init__.py` re-exports the planning API (`gridmap`, `best_route`, `frontier_points`, `pick_goal`, `ZigzagPlanner`, `GoalBrain`).
- One subject per module with a `"""Subject: …"""` docstring.

## Dependencies

### Internal
- Fed by `sensing/lidar` frontiers (safety topics `/safety/frontier_*`, `/safety/route_*`) and `/map` from slam_toolbox via `goal_node`.
- Used by `rosy_control/goal_node.py` (ROS I/O) and `tools/explore_sim.py` (offline sim).

### External
- `collections.deque`, `heapq`, `math` only.

<!-- MANUAL: -->
