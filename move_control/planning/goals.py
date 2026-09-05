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
                 lane_width=0.12, lane_step=0.20, reach_tol=0.05):
        self.min_size = int(min_size)
        self.clear_m = float(clear_m)
        self.retry_clear_m = retry_clear_m
        self.lane_width = float(lane_width)
        self.lane_step = float(lane_step)
        self.reach_tol = float(reach_tol)
        self.mode = 'explore'
        self.covered = set()

    def plan(self, m, pose):
        """Next point to go: (goal_xy | None, route | None, status str).

        goal and route are None for transitional statuses (skipped waypoint,
        done) — the driver publishes nothing then.
        """
        if self.mode == 'explore':
            g = pick_goal(m, pose, min_size=self.min_size,
                          clear_m=self.clear_m,
                          retry_clear_m=self.retry_clear_m)
            if g is not None:
                st = (f"explore goal=({g['x']:.2f},{g['y']:.2f}) "
                      f"size={g['size']} route={g['route']['length']:.2f}m")
                return (g['x'], g['y']), g['route'], st
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
