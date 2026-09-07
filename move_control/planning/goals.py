"""Subject: goal FSM. Map + pose in, point-to-go + route out. No ROS.

explore: pick_goal (frontier) while frontiers last, then coverage:
ZigzagPlanner waypoints with covered-cell tracking. Unreachable coverage
waypoints are skipped instead of stalling the loop. When coverage is done
and probe_when_done is set, a latched farthest-cell probe keeps pushing
the sensor envelope outward while the map may still be growing. goal_node
and tools/explore_sim.py are thin drivers around this brain.
"""
import math

from .astar import best_route
from .escape import start_escape
from .frontier import pick_goal
from .zigzag import ZigzagPlanner, cover_ring


def parse_goal_cmd(text):
    """'x,y' or 'x y' -> (x, y) world metres; None when not coordinates.

    The dashboard's send button lands here over /goal/cmd; verbs still take
    the old path. Non-finite floats are rejected (float('nan') parses!).
    """
    try:
        a, b = str(text).replace(',', ' ').split()
        x, y = float(a), float(b)
    except ValueError:
        return None
    return (x, y) if (math.isfinite(x) and math.isfinite(y)) else None


class GoalBrain:
    """Decide the next point to go. Owns mode + covered set, not the map."""

    def __init__(self, min_size=6, clear_m=0.06, retry_clear_m=0.0,
                 lane_width=0.12, lane_step=0.20, reach_tol=0.05,
                 max_options=3, stall_plans=6, progress_m=0.03,
                 stall_min_dist=0.15, blacklist_plans=20,
                 escape_clear_m=0.08, probe_when_done=False,
                 retry_unreachable_wp=False, manual_ttl_plans=120,
                 start_escape_clear_m=0.0, start_escape_distance_m=.08):
        self.start_escape_clear_m = float(start_escape_clear_m)
        self.start_escape_distance_m = float(start_escape_distance_m)
        self.max_options = max(1, int(max_options))
        # Wide-first escape: after a stall, routes are planned with this
        # clearance (2-cell inflation seals 15 cm gaps) until the robot has
        # left the stuck neighborhood — the way out avoids tight obstacles.
        self.escape_clear_m = float(escape_clear_m)
        self.probe_when_done = bool(probe_when_done)
        # Sim-rig knob: retry an inflation-sealed coverage wp on the raw
        # map instead of burning it as covered. On the 0.3 m sim corridors
        # clear_m inflation seals the whole lane set and coverage starved
        # to 'coverage done' on a 526-cell map. Off by default: a 5 cm gap
        # that is raw-open is still not drivable for real hardware.
        self.retry_unreachable_wp = bool(retry_unreachable_wp)
        # Latched probe target (world xy) + rest window (plan_n counter).
        # The farthest cell shifts as the robot closes in and probe mode
        # has no stall watchdog, so an unlatched goal ping-pongs between
        # far corners forever.
        self._probe = None
        self._probe_deadline = 0
        # Probe stall bench (see _probe_goal): drive-at-but-unreachable
        # probes get benched after stall_plans plans without approach.
        self._probe_n = 0
        self._probe_first = self._probe_best = math.inf
        # Latched external (dashboard) goal: set_manual wins over explore/
        # coverage until reached, unreachable for stall_plans plans, TTL
        # expiry, or a mode verb clears it. Same shape as the probe latch.
        self.manual_ttl_plans = max(1, int(manual_ttl_plans))
        self._manual = None
        self._manual_n = 0
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
        self._wide_cell = None  # stuck pose: demand wide routes near it
        self._target = None     # current goal cell
        self._first = self._best = 0.0
        self._n = 0
        self._coverage_deferred = {}
        self._map_lattice = None
        self._failed_goals = []
        self._failed_exits = []
        self._route_tick = 0

    def avoid_route_exit(self, point):
        if point is not None and all(math.isfinite(v) for v in point):
            self._failed_exits.append((tuple(point), self._route_tick+self.blacklist_plans))

    def avoid_goal(self, goal):
        if goal is not None and all(math.isfinite(v) for v in goal):
            self._failed_goals.append((tuple(goal), self._plan_n+self.blacklist_plans))
        self.clear_manual()
        self._target = self._probe = None

    def reset(self):
        """Clear map-session memory without changing configured geometry."""
        self.covered.clear()
        self._failed_goals.clear()
        self._failed_exits.clear()
        self._coverage_deferred.clear()
        self._blacklist.clear()
        self.last_options = []
        self.clear_manual()
        self._probe = self._target = self._wide_cell = None
        self._map_lattice = None
        self._probe_deadline = self._plan_n = self._n = 0

    def _track_map_lattice(self, m):
        lattice = (m.res, m.ox, m.oy)
        if self._map_lattice is not None and lattice != self._map_lattice:
            res, ox, oy = self._map_lattice
            remapped = set()
            for c, r in self.covered:
                nc = math.floor((ox + (c + .5) * res - m.ox) / m.res)
                nr = math.floor((oy + (r + .5) * res - m.oy) / m.res)
                if 0 <= nc < m.w and 0 <= nr < m.h and m.is_free(nc, nr):
                    remapped.add((nc, nr))
            self.covered = remapped
            self._coverage_deferred.clear()
            self._blacklist.clear()
            self._target = None
        self._map_lattice = lattice

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
        if self._first-self._best >= self.progress_m:
            self._first = self._best = dist
            self._n = 0  # start a new window; early progress cannot excuse a later stall
        if (self._n < self.stall_plans or dist <= self.stall_min_dist
                or self._first - self._best >= self.progress_m):
            return g, ''
        self._blacklist[cell] = self._plan_n + self.blacklist_plans
        # Bench the WHOLE cluster: a single-cell bench churned inside the
        # same unreachable frontier — re-snapping picked a neighbour cell
        # of the same cluster next plan (measured: 600 s grinding one
        # size-15 cluster while the map froze).
        for cc in g.get('cells', ()):
            self._blacklist[cc] = self._plan_n + self.blacklist_plans
        tried = self._n
        self._target = None
        self._wide_cell = (pose[0], pose[1])  # escape wide from here
        alt = pick_goal(m, pose, min_size=self.min_size,
                        clear_m=self.escape_clear_m,
                        retry_clear_m=self.clear_m,
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

    def set_manual(self, x, y):
        """Latch an external goal; replaces any previous one."""
        self._manual = (float(x), float(y))
        self._manual_n = 0

    def clear_manual(self):
        self._manual = None

    def _manual_plan(self, m, pose):
        """Route to the latched external goal. Mirrors _probe_goal: same
        sealed-corridor route retry, same stall_plans bench for a
        drive-at-but-unreachable point."""
        x, y = self._manual
        self._manual_n += 1
        if self._manual_n > self.manual_ttl_plans:
            self._manual = None
            return None                      # expired -> normal planning
        if math.hypot(x - pose[0], y - pose[1]) < self.reach_tol:
            self._manual = None
            return None, None, 'manual goal reached'
        route = None
        margins = [self.clear_m]
        if self.retry_unreachable_wp:
            margins.append(self.retry_clear_m)
        for cm in margins:
            route = best_route(m, pose, (x, y), clear_m=cm)
            if route:
                break
        if route is None:
            if self._manual_n >= self.stall_plans:
                self._manual = None
                return None, None, 'manual goal unreachable, cleared'
            return None, None,                 f'manual goal unreachable ({self._manual_n}/{self.stall_plans})'
        self.last_options = []
        x, y = route['points'][-1]
        self._manual = (x, y)
        return (x, y), route, \
            f'manual goal=({x:.2f},{y:.2f}) route={route["length"]:.2f}m'

    def _probe_goal(self, m, pose, zz):
        """Coverage-done probe: one latched farthest-cell target.

        Returns (goal, route, status); goal None -> caller falls through
        to 'coverage done'. The latch holds one target until it is
        reached, the map takes the cell back, circling sweeps it, or a
        rolling stall_plans-plan window shows no approach — bench that
        cell (into covered, so re-picks skip it) and take the next
        farthest. Without the bench the robot circled one cell for 8
        min with the map frozen (measured on the gz rig).
        """
        d = math.hypot(self._probe[0] - pose[0],
                       self._probe[1] - pose[1]) if self._probe else None
        if self._probe is not None:
            cell = m.world_to_grid(*self._probe)
            if d < self.reach_tol:
                self._probe = None  # reached
            elif not m.is_free(*cell):
                self._probe = None  # noisy SLAM took the cell back
            elif cell in self.covered:
                self._probe = None  # circling it swept the cell
            elif d < self._probe_best:
                self._probe_best = d
            self._probe_n += 1
            if self._probe is not None and self._probe_n >= self.stall_plans:
                if self._probe_first - self._probe_best < self.progress_m:
                    self.covered.add(cell)
                    self._probe = None  # bench: no approach in the window
                self._probe_n = 0
                self._probe_first = self._probe_best = d
        if self._probe is None:
            self._probe = zz.probe_point(pose, covered=self.covered)
            self._probe_n = 0
            self._probe_first = self._probe_best = \
                math.hypot(self._probe[0] - pose[0],
                           self._probe[1] - pose[1]) if self._probe \
                else math.inf
        if self._probe is None:
            return None, None, ''  # nothing reachable >= walk_min yet
        margins = [self.clear_m]
        if self.retry_unreachable_wp:
            margins.append(self.retry_clear_m)
        for cm in margins:
            route = best_route(m, pose, self._probe, clear_m=cm)
            if route and route['length'] >= 0.05:
                self._probe = route['points'][-1]
                return self._probe, route, (
                    f'probe goal=({self._probe[0]:.2f},{self._probe[1]:.2f}) '
                    f'route={route["length"]:.2f}m')
        self._probe = None
        self._probe_deadline = self._plan_n + self.blacklist_plans
        return None, None, ''

    def plan(self, m, pose):
        self._route_tick += 1
        self._failed_exits = [(xy, expiry) for xy, expiry in self._failed_exits if expiry > self._route_tick]
        goal, route, status = self._plan_candidate(m, pose)
        if route and self._failed_exits:
            candidates = [(goal, route)] + [((opt['x'], opt['y']), opt['route']) for opt in self.last_options]
            self.last_options = []
            route = None
            for candidate, original in candidates:
                route = best_route(m, pose, candidate, clear_m=original.get('clearance_m', self.clear_m),
                                   avoid_points=[xy for xy, _ in self._failed_exits])
                if route:
                    goal = route['points'][-1]
                    break
            if route is None:
                return None, None, 'replanning: no route outside failed exits'
            status = 'alternative exit: ' + status
        return goal, route, status

    def _plan_candidate(self, m, pose):
        start = m.world_to_grid(*pose)
        if m.is_free(*start) and not m.inflate(self.clear_m / m.res).is_free(*start):
            route = start_escape(m, pose, self.clear_m, self.start_escape_clear_m,
                                 self.start_escape_distance_m,
                                 [xy for xy,expiry in self._failed_goals if expiry>self._plan_n])
            if route:
                self.last_options = []
                return route['points'][-1], route, 'escape: moving to preferred clearance'
            minimum = self.start_escape_clear_m
            if 0 < minimum < self.clear_m and m.inflate(minimum/m.res).is_free(*start):
                # A long narrow corridor may have no nearby wide escape.
                # Replan against the explicit hard footprint margin; never raw map.
                preferred, retry = self.clear_m, self.retry_clear_m
                self.clear_m = self.retry_clear_m = minimum
                try:
                    goal, route, status = self._plan(m, pose)
                    return goal, route, 'narrow passage: ' + status
                finally:
                    self.clear_m, self.retry_clear_m = preferred, retry
            self.last_options = []
            return None, None, 'planning blocked: robot inside obstacle clearance'
        return self._plan(m, pose)

    def _plan(self, m, pose):
        """Next point to go: (goal_xy | None, route | None, status str).

        goal and route are None for transitional statuses (skipped waypoint,
        done) — the driver publishes nothing then.
        """
        self._plan_n += 1
        self.last_options = []
        self._track_map_lattice(m)
        self._coverage_deferred = {c: expiry for c, expiry in self._coverage_deferred.items()
                                   if expiry > self._plan_n}
        self._gc_blacklist()
        self._failed_goals = [(xy,expiry) for xy,expiry in self._failed_goals if expiry>self._plan_n]
        failed = {cell for cell in m.free_cells() if any(
            math.dist(m.grid_to_world(*cell),xy)<=.10 for xy,_ in self._failed_goals)}
        for cell in failed:
            self._coverage_deferred[cell] = self._plan_n+1
            self._blacklist[cell] = max(self._blacklist.get(cell,0),self._plan_n+1)
        # External (dashboard) goal wins while valid; _manual_plan clears it
        # on expiry/unreachable and returns None -> fall through normally.
        if self._manual is not None:
            out = self._manual_plan(m, pose)
            if out is not None:
                return out
        # Wide-first latch: near the last stuck pose, plan with extra
        # clearance until the robot is clear of that obstacle pocket.
        wide = False
        if self._wide_cell is not None:
            if math.hypot(pose[0] - self._wide_cell[0],
                          pose[1] - self._wide_cell[1]) <= 0.30:
                wide = True
            else:
                self._wide_cell = None
        if self.mode == 'explore':
            clear_first = self.escape_clear_m if wide else self.clear_m
            clear_retry = self.clear_m if wide else self.retry_clear_m
            g = pick_goal(m, pose, min_size=self.min_size,
                          clear_m=clear_first,
                          retry_clear_m=clear_retry,
                          exclude=set(self._blacklist) | failed)
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
                return (g['x'], g['y']), g['route'], \
                    ('wide-first ' if wide else '') + prefix + st
            # No frontier this tick: sweep coverage below, but keep the
            # explore request — a growing map will re-open frontiers and
            # this branch picks them up automatically on a later plan.
            if self.mode != 'explore':
                self.mode = 'explore'
        cover_ring(self.covered, m, pose[0], pose[1],
                   radius_m=self.lane_width / 2)
        zz = ZigzagPlanner(m, start=pose,
                           covered=self.covered | set(self._coverage_deferred),
                           lane_width=self.lane_width,
                           lane_step=self.lane_step)
        if not zz.region:
            # Pose sits on unknown space (map still filling, or odom~map
            # drift): nothing is drivable yet. done would be a lie — the
            # robot hasn't swept anything. Idle and retry next plan.
            return None, None, 'coverage idle (no known space)'
        wps = zz.waypoints()
        if not wps:
            if self._coverage_deferred:
                return None, None, f'coverage waiting ({len(self._coverage_deferred)} unreachable cells)'
            if self.probe_when_done and self._plan_n >= self._probe_deadline:
                goal, route, status = self._probe_goal(m, pose, zz)
                if goal is not None:
                    return goal, route, status
            return None, None, 'coverage done'
        # Skip a bounded batch of blocked lane endpoints in this plan. A
        # target snapped back onto the robot must not stall every replan.
        for requested in wps[:16]:
            route = best_route(m, pose, requested, clear_m=self.clear_m)
            if route is None and self.retry_unreachable_wp and \
                    self.retry_clear_m < self.clear_m:
                route = best_route(m, pose, requested, clear_m=self.retry_clear_m)
            goal = route['points'][-1] if route else None
            if (goal is None or m.world_to_grid(*goal) in self.covered or
                    math.hypot(goal[0] - pose[0], goal[1] - pose[1]) < self.reach_tol):
                self._coverage_deferred[m.world_to_grid(*requested)] = self._plan_n + self.blacklist_plans
                continue
            return goal, route, (f'coverage goal=({goal[0]:.2f},{goal[1]:.2f}) '
                                 f'left={len(wps)}')
        return None, None, 'coverage wp unreachable, deferred'
