#!/usr/bin/env python3
"""ASCII sim: SLAM-style exploring, then zigzag coverage. No ROS needed.

python3 tools/explore_sim.py                # animated, 8 fps
python3 tools/explore_sim.py --quiet        # summary + final frame only

Four classes, one job each:
  Maze       — the true world: ASCII art -> free-cell set.
  Robot      — differential toy: rotate-then-drive toward a point.
  SimLidar   — C1 stand-in: raycast beams paint the known OccupancyMap.
  ExploreSim — the tick loop: reveal, GoalBrain plan, drive, render.

Driven by the same planning.GoalBrain as goal_node, so sim and node share
one decision path. Speed is 6x the real robot so it stays watchable.
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


class Maze:
    """The true world: ASCII art -> free-cell set (grid coords)."""

    def __init__(self, art):
        self.h = len(art)
        self.w = len(art[0])
        self.free = {(c, self.h - 1 - row) for row, line in enumerate(art)
                     for c, ch in enumerate(line) if ch == '.'}


class Robot:
    """Differential toy: rotate in place past 0.35 rad error, then drive."""

    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw

    def step_toward(self, tx, ty):
        """One DT of motion; True when the robot drove this tick."""
        err = wrap(math.atan2(ty - self.y, tx - self.x) - self.yaw)
        if abs(err) > 0.35:
            self.yaw += W * DT * (1.0 if err > 0 else -1.0)
            return False
        self.x += V * DT * math.cos(self.yaw)
        self.y += V * DT * math.sin(self.yaw)
        return True


class SimLidar:
    """C1 stand-in: 2 cm beam steps, wall at the hit, capped range."""

    def __init__(self, beams=120, max_m=LIDAR_M):
        self.beams = beams
        self.max_m = max_m

    def paint(self, sim, true_free, x, y):
        """Reveal: free along each beam, wall at the hit."""
        for k in range(self.beams):
            a = k * (2.0 * math.pi / self.beams)
            d = 0.02
            while d <= self.max_m:
                px = x + math.cos(a) * d
                py = y + math.sin(a) * d
                c, r = sim.world_to_grid(px, py)
                if (c, r) not in true_free:
                    sim.set_cell(c, r, OCC)
                    break
                sim.set_cell(c, r, FREE)
                d += 0.02


class ExploreSim:
    """Wires world + known map + robot + brain and runs the tick loop."""

    def __init__(self, quiet=False, max_ticks=4000, render_every=25,
                 fps=8.0):
        self.world = Maze(MAZE)
        self.sim = OccupancyMap(self.world.w, self.world.h, RES, fill=UNKNOWN)
        self.robot = Robot(*self.sim.grid_to_world(1, 1), 0.0)
        self.lidar = SimLidar()
        self.brain = GoalBrain(min_size=1, clear_m=0.06, retry_clear_m=0.0)
        self.covered = self.brain.covered
        self.phase = self.brain.mode
        self.quiet = quiet
        self.max_ticks = max_ticks
        self.render_every = render_every
        self.fps = fps
        self.route, self.ri, self.goal = None, 0, None
        self.since_plan = 0
        self.tick = 0

    def run(self):
        for self.tick in range(1, self.max_ticks + 1):
            self.lidar.paint(self.sim, self.world.free,
                             self.robot.x, self.robot.y)
            if self._maybe_plan():
                break  # brain says coverage done
            self._drive()
            self.since_plan += 1
            if not self.quiet and self.tick % self.render_every == 0:
                self._frame()
                time.sleep(1.0 / self.fps)
        if self.quiet:
            self._frame()
        return self._summary()

    def _maybe_plan(self):
        """Plan when idle. True when the brain says coverage is done."""
        if self.route is None or self.ri >= len(self.route['points']) \
                or self.since_plan >= REPLAN_EVERY:
            self.goal, self.route, status = self.brain.plan(
                self.sim, (self.robot.x, self.robot.y))
            self.since_plan = 0
            self.phase = self.brain.mode
            if status == 'coverage done':
                return True
            if self.goal is None:
                self.route = None  # skip / at-robot: retry next tick
            else:
                self.ri = 1  # route points[0] is the robot's own cell
        return False

    def _drive(self):
        if self.route is None or self.ri >= len(self.route['points']):
            self.route = None  # force replan next tick
            return
        tx, ty = self.route['points'][self.ri]
        if math.hypot(tx - self.robot.x, ty - self.robot.y) < REACH_TOL:
            self.ri += 1
            return
        if self.robot.step_toward(tx, ty):
            cover_ring(self.covered, self.sim, self.robot.x, self.robot.y)

    def _frame(self):
        marks = {(c, r): 'o' for c, r in self.covered}
        if self.route:
            for pt in self.route['points'][self.ri:]:
                marks[self.sim.world_to_grid(*pt)] = '+'
        if self.goal is not None:
            marks[self.sim.world_to_grid(*self.goal)] = '*'
        marks[self.sim.world_to_grid(self.robot.x, self.robot.y)] = 'R'
        nxt = (self.route['points'][self.ri]
               if self.route and self.ri < len(self.route['points']) else None)
        head = (f't={self.tick} {self.phase} next={nxt} '
                f'known={len(self.sim.free_cells())} covered={len(self.covered)}')
        print('\033[2J\033[H')
        print(head)
        print(self.sim.render(marks=marks))

    def _summary(self):
        left = self.world.free - set(self.sim.free_cells())
        uncov = len(set(self.sim.free_cells()) - self.covered)
        print(f'done={self.phase} ticks={self.tick} '
              f'known={len(self.sim.free_cells())}/{len(self.world.free)} '
              f'unseen={len(left)} covered={len(self.covered)} '
              f'uncovered={uncov}')
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
    sys.exit(ExploreSim(quiet=a.quiet, max_ticks=a.max_ticks,
                        render_every=a.render_every,
                        fps=a.fps).run())


if __name__ == '__main__':
    main()
