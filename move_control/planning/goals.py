"""Subject: goal FSM. Map + pose in, point-to-go + route out. No ROS.

explore: pick_goal (frontier) while frontiers last, then coverage:
ZigzagPlanner waypoints with covered-cell tracking. Unreachable coverage
waypoints are skipped instead of stalling the loop. goal_node and
tools/explore_sim.py are thin drivers around this brain.
"""
import math

from .astar import best_route
from .frontier import pick_goal
from .zigzag import ZigzagPlanner, cover_ring


class GoalBrain:
    """Decide the next point to go. Owns mode + covered set, not the map."""

    def __init__(self, min_size=6, clear_m=0.06, retry_clear_m=0.0,
                 lane_width=0.12, lane_step=0.20, reach_tol=0.05,
                 max_options=3, stall_plans=6, progress_m=0.03,
                 stall_min_dist=0.15, blacklist_plans=20):
        self.max_options = max(1, int(max_options))
        # Stall watchdog: if the robot gets nowhere for this many plan calls,
        # bench that frontier and take the next-best option.
        self.stall_plans = max(2, int(stall_plans))
        self.progress_m = float(progress_m)
        self.stall_min_dist = float(stall_min_dist)
        self.blacklist_plans = max(1, int(blacklist_plans))
        self.min_size = int(min_size)
        self.clear_m = float(clear_m)
        self.retry_clear_m = retry_clear_m
        self.lane_width = float(lane_width)
        self.lane_step = float(lane_step)
        self.reach_tol = float(reach_tol)
        self.mode = 'explore'
        self.covered = set()
        self.last_options = []  # ranked frontier candidates for display
        self._plan_n = 0
        self._blacklist = {}    # cell -> plan_n when it may be tried again
        self._target = None     # current goal cell
        self._first = self._best = 0.0
        self._n = 0

    def _watchdog(self, m, pose, g):
        """Bench a frontier the robot fails to approach.

        Same target for stall_plans plan calls with under progress_m
        improvement (target farther than stall_min_dist, so near-goals are
        exempt) => blacklist it for blacklist_plans plan calls and re-pick
        from the remaining options. Returns (goal | None, status prefix).
        """
        cell = m.world_to_grid(g['x'], g['y'])
        dist = math.hypot(g['x'] - pose[0], g['y'] - pose[1])
        if self._target != cell:
            self._target, self._first, self._best, self._n = cell, dist, dist, 0
            return g, ''
        self._n += 1
        self._best = min(self._best, dist)
        if (self._n < self.stall_plans or dist <= self.stall_min_dist
                or self._first - self._best >= self.progress_m):
            return g, ''
        self._blacklist[cell] = self._plan_n + self.blacklist_plans
        tried = self._n
        self._target = None
        alt = pick_goal(m, pose, min_size=self.min_size,
                        clear_m=self.clear_m,
                        retry_clear_m=self.retry_clear_m,
                        exclude=set(self._blacklist))
        if alt is None:
            return None, (f'stall: frontier {cell} benched, '
                          f'no alternative left')
        cell2 = m.world_to_grid(alt['x'], alt['y'])
        self._target, d2 = cell2, math.hypot(alt['x'] - pose[0],
                                             alt['y'] - pose[1])
        self._first = self._best = d2
        self._n = 0
        return alt, (f'stall: {cell} benched after {tried} plans '
                     f'(<{self.progress_m:.2f}m gain), switched -> ')

    def _gc_blacklist(self):
        """Drop benched cells whose plan-call timeout has passed."""
        for cell in [c for c, exp in self._blacklist.items()
                     if exp <= self._plan_n]:
            del self._blacklist[cell]

    def plan(self, m, pose):
        """Next point to go: (goal_xy | None, route | None, status str).

        goal and route are None for transitional statuses (skipped waypoint,
        done) — the driver publishes nothing then.
        """
        self._plan_n += 1
        self._gc_blacklist()
        if self.mode == 'explore':
            g = pick_goal(m, pose, min_size=self.min_size,
                          clear_m=self.clear_m,
                          retry_clear_m=self.retry_clear_m,
                          exclude=set(self._blacklist))
            if g is not None:
                g, prefix = self._watchdog(m, pose, g)
                if g is None:
                    return None, None, prefix
                self.last_options = g.get('options', [])
                st = (f"explore goal=({g['x']:.2f},{g['y']:.2f}) "
                      f"score={g.get('score', 0):.1f} "
                      f"size={g['size']} "
                      f"route={g['route']['length']:.2f}m "
                      f"alts={max(0, len(self.last_options) - 1)}")
                return (g['x'], g['y']), g['route'], prefix + st
            self.mode = 'coverage'  # map fully frontiered
        cover_ring(self.covered, m, pose[0], pose[1],
                   radius_m=self.lane_width / 2 + 0.08)
        zz = ZigzagPlanner(m, start=pose, covered=self.covered,
                           lane_width=self.lane_width,
                           lane_step=self.lane_step)
        if not zz.region:
            # Pose sits on unknown space (map still filling, or odom~map
            # drift): nothing is drivable yet. done would be a lie — the
            # robot hasn't swept anything. Idle and retry next plan.
            return None, None, 'coverage idle (no known space)'
        wps = zz.waypoints()
        if not wps:
            return None, None, 'coverage done'
        goal = wps[0]
        route = best_route(m, pose, goal, clear_m=self.clear_m)
        if route is None:
            self.covered.add(m.world_to_grid(*goal))
            return None, None, 'coverage wp unreachable, skip'
        if math.hypot(goal[0] - pose[0], goal[1] - pose[1]) < self.reach_tol:
            return None, None, 'coverage wp at robot'
        return goal, route, (f'coverage goal=({goal[0]:.2f},{goal[1]:.2f}) '
                             f'left={len(wps)}')
