#!/usr/bin/env python3
"""ASCII sim: SLAM-style exploring, then zigzag coverage. No ROS needed.

python3 tools/explore_sim.py                # animated, 8 fps
python3 tools/explore_sim.py --quiet        # summary + final frame only

Builds the known map by raycast (C1 lidar cap 0.45 m), checks the map and
makes the point to go with planner.pick_goal (frontier), drives it with
planner.best_route. When no frontier is left, ZigzagPlanner zigzag-covers
the known free space. Speed is 6x the real robot so it stays watchable.
"""
import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from move_control.planning import (FREE, OCC, UNKNOWN, GoalBrain,
                                   OccupancyMap, cover_ring)

RES = 0.05          # m/cell, same as slam_toolbox resolution
LIDAR_M = 0.45      # C1 usable range in the desk maze (route.py cap)
V = 0.06            # m/s (real cruise is 0.014; 6x so the sim is watchable)
W = 1.2             # rad/s turn rate
DT = 0.1            # s per tick
REACH_TOL = 0.03    # m, waypoint reached
REPLAN_EVERY = 300  # ticks; replan mainly on arrival, not mid-route

# 13x13 desk maze: 3-cell (15 cm) corridors, doors punched through walls.
# Top line is r=h-1. '#' wall, '.' floor.
MAZE = [
    "#############",
    "#...........#",
    "#.###.###.#.#",
    "#.#.....#...#",
    "#.#.###.#.###",
    "#...#...#...#",
    "###.#.#####.#",
    "#...#.....#.#",
    "#.#######.#.#",
    "#.#.....#.#.#",
    "#.#.###...#.#",
    "#.....#...#.#",
    "#############",
]


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def reveal(sim, true_free, x, y, beams=120):
    """Lidar reveal: free along each beam, wall at the hit, 0.45 m cap."""
    for k in range(beams):
        a = k * (2.0 * math.pi / beams)
        d = 0.02
        while d <= LIDAR_M:
            px = x + math.cos(a) * d
            py = y + math.sin(a) * d
            c, r = sim.world_to_grid(px, py)
            if (c, r) not in true_free:
                sim.set_cell(c, r, OCC)
                break
            sim.set_cell(c, r, FREE)
            d += 0.02
        else:
            continue
    return


def step(x, y, yaw, tx, ty):
    """One dt of rotate-then-drive motion. Returns (x, y, yaw, moved)."""
    err = wrap(math.atan2(ty - y, tx - x) - yaw)
    if abs(err) > 0.35:
        return x, y, yaw + W * DT * (1.0 if err > 0 else -1.0), False
    x += V * DT * math.cos(yaw)
    y += V * DT * math.sin(yaw)
    return x, y, yaw, True


def frame(sim, x, y, route, ri, goal, covered, tick, phase):
    marks = {(c, r): 'o' for c, r in covered}
    if route:
        for pt in route['points'][ri:]:
            marks[sim.world_to_grid(*pt)] = '+'
    if goal is not None:
        marks[sim.world_to_grid(*goal)] = '*'
    marks[sim.world_to_grid(x, y)] = 'R'
    nxt = route['points'][ri] if route and ri < len(route['points']) else None
    head = (f't={tick} {phase} next={nxt} '
            f'known={len(sim.free_cells())} covered={len(covered)}')
    print('\033[2J\033[H')
    print(head)
    print(sim.render(marks=marks))


def run(quiet=False, max_ticks=4000, render_every=25, fps=8):
    h = len(MAZE)
    w = len(MAZE[0])
    true_free = {(c, h - 1 - row) for row, line in enumerate(MAZE)
                 for c, ch in enumerate(line) if ch == '.'}
    sim = OccupancyMap(w, h, RES, fill=UNKNOWN)
    x, y = sim.grid_to_world(1, 1)
    yaw = 0.0
    brain = GoalBrain(min_size=1, clear_m=0.06, retry_clear_m=0.0)
    covered = brain.covered
    phase = brain.mode
    route, ri, goal = None, 0, None
    since_plan = 0
    for tick in range(1, max_ticks + 1):
        reveal(sim, true_free, x, y)
        if route is None or ri >= len(route['points']) or since_plan >= REPLAN_EVERY:
            goal, route, status = brain.plan(sim, (x, y))
            since_plan = 0
            phase = brain.mode
            if status == 'coverage done':
                break  # zigzag empty -> everything swept
            if goal is None:
                route = None  # skip / at-robot: retry next tick
            else:
                ri = 1  # route points[0] is the robot's own cell
        if route is None or ri >= len(route['points']):
            route = None  # force replan next tick
        if route is not None:
            tx, ty = route['points'][ri]
            if math.hypot(tx - x, ty - y) < REACH_TOL:
                ri += 1
            else:
                x, y, yaw, moved = step(x, y, yaw, tx, ty)
        else:
            moved = False
        if moved:
            cover_ring(covered, sim, x, y)
        since_plan += 1
        if not quiet and tick % render_every == 0:
            frame(sim, x, y, route, ri, goal, covered, tick, phase)
            time.sleep(1.0 / fps)
    if quiet:
        frame(sim, x, y, route, ri, goal, covered, tick, phase)
    left = true_free - set(sim.free_cells())
    uncov = len(set(sim.free_cells()) - covered)
    print(f'done={phase} ticks={tick} known={len(sim.free_cells())}/{len(true_free)} '
          f'unseen={len(left)} covered={len(covered)} uncovered={uncov}')
    # Exit reflects exploration; uncovered nooks are informational
    # (best-effort sweep — lanes can't center in every corner).
    return 1 if left else 0


def main():
    ap = argparse.ArgumentParser(description='ASCII explore + zigzag sim')
    ap.add_argument('--quiet', action='store_true', help='final frame only')
    ap.add_argument('--max-ticks', type=int, default=4000)
    ap.add_argument('--render-every', type=int, default=25)
    ap.add_argument('--fps', type=float, default=8.0,
                    help='animation delay; ignored with --quiet')
    a = ap.parse_args()
    sys.exit(run(quiet=a.quiet, max_ticks=a.max_ticks,
                 render_every=a.render_every, fps=a.fps))


if __name__ == '__main__':
    main()
