#!/usr/bin/env python3
import math
import unittest
from unittest import mock

from rosy_control.planning import (FREE, OCC, UNKNOWN, GoalBrain,
                                   OccupancyMap, ZigzagPlanner, best_route,
                                   frontier, frontier_points,
                                   parse_goal_cmd, pick_goal)


class RoomCase(unittest.TestCase):
    """Shared fixture: walled rooms, standard poses, wall builders.

    Every test class below builds its world from these so a change to the
    grid resolution or the maze geometry stays a one-line edit.
    """

    RES = 0.05
    START = (0.125, 0.125)   # cell (2, 2) of the default 9x7 room
    EAST = (0.325, 0.125)    # cell (6, 2), across a mid-room wall
    CORNER = (0.075, 0.075)  # cell (1, 1)
    LANE_TOL = (0.12 / 2 + 0.20 / 2) / 0.05  # cells: half lane + half step

    def room(self, w=9, h=7, pockets=((5, 3, 6, 4),)):
        """Walled room; pockets are (c0, r0, c1, r1) unknown rectangles."""
        m = OccupancyMap(w, h, self.RES, fill=FREE)
        for c in range(w):
            m.set_cell(c, 0, OCC)
            m.set_cell(c, h - 1, OCC)
        for r in range(h):
            m.set_cell(0, r, OCC)
            m.set_cell(w - 1, r, OCC)
        for c0, r0, c1, r1 in pockets:
            for c in range(c0, c1 + 1):
                for r in range(r0, r1 + 1):
                    m.set_cell(c, r, UNKNOWN)
        return m

    def open_room(self, w=12, h=10):
        return self.room(w=w, h=h, pockets=())

    def dark_room(self, w=9, h=7):
        """All unknown — the map before the first scan."""
        return OccupancyMap(w, h, self.RES, fill=UNKNOWN)

    def known_room(self, w=9, h=7):
        """Fully mapped room — no unknown space left."""
        return self.room(w=w, h=h, pockets=())

    def wall_col(self, m, c, rows, v=OCC):
        for r in rows:
            m.set_cell(c, r, v)
        return m

    def gap_room(self):
        """9x11 room, mid wall with a 3-cell (15 cm) gap at rows 3-5."""
        m = self.room(w=9, h=11, pockets=())
        return self.wall_col(m, 4, (1, 2, 6, 7, 8, 9))


class GridMapTest(RoomCase):
    def setUp(self):
        self.m = self.room()

    def test_roundtrip(self):
        c, r = self.m.world_to_grid(0.225, 0.125)
        x, y = self.m.grid_to_world(c, r)
        self.assertAlmostEqual(x, 0.225, places=2)
        self.assertAlmostEqual(y, 0.125, places=2)

    def test_outside_is_wall(self):
        self.assertEqual(self.m.cell(-1, 0), OCC)
        self.assertEqual(self.m.cell(99, 99), OCC)

    def test_free_bands(self):
        m = self.room(pockets=())
        m.set_cell(2, 2, UNKNOWN)
        m.set_cell(2, 3, 50)
        m.set_cell(2, 4, 70)
        self.assertFalse(m.is_free(2, 2))  # unknown never free
        self.assertTrue(m.is_free(2, 3))   # 0..64 free-ish
        self.assertFalse(m.is_free(2, 4))  # >= 65 is a wall

    def test_inflate_keeps_unknown(self):
        # 13x11 room so (7,6) sits >2 cells from every wall and the ring.
        m = self.room(w=13, h=11, pockets=())
        m.set_cell(4, 3, OCC)
        m.set_cell(3, 6, UNKNOWN)  # >2 cells from walls and the ring
        inf = m.inflate(2)
        self.assertEqual(inf.cell(4, 3), OCC)
        self.assertEqual(inf.cell(5, 4), OCC)   # diagonal inside the ring
        self.assertEqual(inf.cell(7, 6), FREE)  # outside the ring
        self.assertEqual(inf.cell(3, 6), UNKNOWN)  # unknown never inflates


class AstarTest(RoomCase):
    def setUp(self):
        self.m = self.known_room()

    def test_around_wall(self):
        m = self.wall_col(self.m, 4, (1, 2, 4, 5))
        r = best_route(m, self.START, self.EAST, clear_m=0.0)
        self.assertIsNotNone(r)
        self.assertGreater(r['length'], 0.2)  # detour, not the straight line

    def test_closed_wall_no_route(self):
        m = self.wall_col(self.m, 4, (1, 2, 3, 4, 5))
        self.assertIsNone(best_route(m, self.START, self.EAST))

    def test_route_never_touches_wall(self):
        # clear_m 0.0: cells on the route must all be free on the raw map.
        m = self.wall_col(self.m, 4, (1, 2, 4, 5))
        r = best_route(m, self.START, self.EAST, clear_m=0.0)
        self.assertIsNotNone(r)
        for c, rr in r['cells']:
            self.assertTrue(m.is_free(c, rr), (c, rr))

    def test_goal_on_wall_snaps(self):
        # Clicking a wall goes to the nearest free cell (5-7 cm away),
        # so a sloppy map tap still yields a drivable point.
        r = best_route(self.m, self.START, (0.025, 0.025))
        self.assertIsNotNone(r)
        self.assertEqual(r['cells'][-1], (2, 2))

    def test_route_length_matches_polyline(self):
        # A requested goal inside inflation must stop at a safe endpoint;
        # appending the raw cell bypassed the planner's collision margin.
        m = self.known_room()
        r = best_route(m, (0.125, 0.125), (0.075, 0.125))
        self.assertIsNotNone(r)
        self.assertNotEqual(r['cells'][-1], m.world_to_grid(0.075, 0.125))
        inflated = m.inflate(1)
        self.assertTrue(all(inflated.is_free(*cell) for cell in r['cells']))
        pl = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                 for a, b in zip(r['points'], r['points'][1:]))
        self.assertAlmostEqual(r['length'], pl, places=6)
        self.assertAlmostEqual(r['length'], 0.0, places=6)

    def test_start_inside_inflation_does_not_teleport_to_safe_cell(self):
        self.assertIsNone(best_route(self.known_room(), self.CORNER, self.EAST))

    def test_three_cell_gap_passes(self):
        # 15 cm gap survives clear_m 0.05 (2*0.05 + 1 cell = robot diameter).
        m = self.gap_room()
        r = best_route(m, self.START, (0.325, 0.225), clear_m=0.05)
        self.assertIsNotNone(r)
        for c, rr in r['cells']:
            self.assertTrue(m.is_free(c, rr), (c, rr))


class FrontierTest(RoomCase):
    def setUp(self):
        self.m = self.room()
        self.start = self.START

    def test_pick_goal_reachable(self):
        g = pick_goal(self.m, self.start, min_size=2)
        self.assertIsNotNone(g)
        self.assertEqual(g['kind'], 'frontier')
        c, r = self.m.world_to_grid(g['x'], g['y'])
        self.assertTrue(self.m.is_free(c, r), (c, r))
        self.assertGreaterEqual(g['route']['length'], 0.08)

    def test_pick_goal_none_when_mapped(self):
        self.assertIsNone(pick_goal(self.known_room(), self.start))

    def test_pick_goal_retry_sealed_corridor(self):
        # 5 cm gap: raw-open, sealed at clear_m 0.06. The retry pass at 0.0
        # still finds an approach to the frontier behind it.
        m = self.wall_col(self.room(w=9, h=7, pockets=((6, 2, 7, 3),)),
                          4, (1, 2, 4, 5))
        self.assertIsNone(pick_goal(m, self.start, min_size=2))
        g = pick_goal(m, self.start, min_size=2, retry_clear_m=0.0)
        self.assertIsNotNone(g)
        self.assertEqual(g['clear_m'], 0.0)

    def test_pick_goal_skips_degenerate(self):
        self.assertIsNone(pick_goal(self.m, self.start, min_size=2,
                                    min_route_m=99.0))
        self.assertIsNotNone(pick_goal(self.m, self.start, min_size=2,
                                       min_route_m=0.0))

    def test_pick_goal_best_ratio_not_biggest(self):
        # Ratio beat raw size: a small frontier next door outscores a big
        # one behind a wall (detour kills its gain-per-metre).
        m = self.room(w=13, h=7, pockets=((3, 2, 3, 3), (9, 1, 11, 4)))
        m = self.wall_col(m, 6, (1, 2, 4, 5))  # gap only at row 3
        g = pick_goal(m, self.START, min_size=2, retry_clear_m=0.0)
        self.assertIsNotNone(g)
        c, _ = m.world_to_grid(g['x'], g['y'])
        self.assertLess(c, 6, g)  # winner is on the near side of the wall
        opts = g['options']
        self.assertGreaterEqual(len(opts), 2)
        # Chosen first (marker 0 = the published goal), then alternatives by
        # score: the chosen frontier's raw-pass twin scores higher (zero-
        # clearance route), and that is fine — marker 0 is the chosen route.
        self.assertEqual(opts[0]['clear_m'], g['clear_m'])
        self.assertEqual((opts[0]['x'], opts[0]['y']), (g['x'], g['y']))
        self.assertGreaterEqual(opts[1]['score'], opts[2]['score'])
        self.assertLessEqual(len(opts), 3)
        # The winner is NOT the biggest candidate — the ratio decided.
        self.assertLess(opts[0]['size'], max(o['size'] for o in opts))

    def test_pick_goal_options_chosen_first(self):
        # Review finding: options were sorted by score ACROSS passes, so a
        # raw-clearance pass candidate (shorter route -> higher score) sat
        # at index 0 while the published goal was the safe-pass winner:
        # marker 0 on /goal/options showed a route never chosen.
        m = self.wall_col(
            self.room(w=13, h=7, pockets=((1, 4, 1, 5), (8, 2, 9, 3))),
            4, (1, 2, 4, 5))
        start = self.START
        real = frontier.best_route

        def routes(mm, s, goal, clear_m=0.06):
            r = real(mm, s, goal, clear_m=clear_m)
            if r is None:
                return None
            if clear_m == 0.0:
                # Zero clearance takes the shortest line: model it as the
                # same route shorter (x0.85 — kept above min_route_m so the
                # raw twin still qualifies) so a raw-pass candidate
                # provably outscores the safe winner.
                return {**r, 'length': r['length'] * 0.85}
            return r

        with mock.patch.object(frontier, 'best_route',
                               side_effect=routes):
            g = pick_goal(m, start, min_size=2, retry_clear_m=0.0)
        self.assertIsNotNone(g)
        opts = g['options']
        self.assertGreaterEqual(len(opts), 3)
        # options[0] IS the chosen goal: same point, same clearance pass.
        self.assertEqual(opts[0]['clear_m'], 0.06)
        self.assertEqual((opts[0]['x'], opts[0]['y']), (g['x'], g['y']))
        # A raw-pass candidate with a STRICTLY higher score sits behind the
        # chosen goal, and the alternatives are score-ordered.
        self.assertEqual(opts[1]['clear_m'], 0.0)
        self.assertGreater(opts[1]['score'], opts[0]['score'])
        self.assertGreaterEqual(opts[1]['score'], opts[2]['score'])


class ZigzagTest(RoomCase):
    def setUp(self):
        self.m = self.open_room()
        self.zz = ZigzagPlanner(self.m, start=self.CORNER, lane_width=0.12,
                                lane_step=0.20)
        self.wps = self.zz.wps  # cells; waypoints() is the world-metre view

    def test_coverage_completes(self):
        for wc, wr in self.wps:
            self.assertTrue(self.m.is_free(wc, wr), (wc, wr))
        # Every free cell within half a lane + half a step of a waypoint.
        for c, r in self.m.free_cells():
            d = min(math.hypot(c - wc, r - wr) for wc, wr in self.wps)
            self.assertLessEqual(d, self.LANE_TOL + 1e-9, (c, r))

    def test_covered_lane_dropped(self):
        cov = {(c, 1) for c in range(1, 11)}
        zz = ZigzagPlanner(self.m, start=self.CORNER, covered=cov)
        self.assertNotIn(1, [lane['key'] for lane in zz.lanes])
        self.assertIn(1, [lane['key'] for lane in self.zz.lanes])

    def test_serpentine_alternates(self):
        keys = list(dict.fromkeys(r for _c, r in self.wps))
        self.assertGreaterEqual(len(keys), 3)
        dirs = []
        for key in keys:
            lane_wps = [(c, r) for c, r in self.wps if r == key]
            dirs.append(lane_wps[0][0] < lane_wps[-1][0])
        for a, b in zip(dirs, dirs[1:]):
            self.assertNotEqual(a, b)

    def test_probe_farthest_by_walk(self):
        # Farthest by walk distance lands in the far column (8-conn ties
        # across the column, walk_min 0.10 is far below the walk there).
        probe = self.zz.probe_point(self.CORNER)
        self.assertIsNotNone(probe)
        c, r = self.m.world_to_grid(*probe)
        self.assertTrue(self.m.is_free(c, r), (c, r))
        self.assertEqual(c, 10)  # far column of the 12x10 room

    def test_probe_no_corner_cutting(self):
        # The probe flood shares best_route's connectivity: chambers joined
        # only by corner-cut diagonals stay separate, so a raw-map route to
        # the probe always exists (the coverage-done deadlock regression).
        m = self.room(w=9, h=5, pockets=())
        self.wall_col(m, 4, (1, 2, 3))  # full wall, gap freed below
        m.set_cell(4, 2, FREE)          # gap cell in the wall
        m.set_cell(3, 2, OCC)           # pinch: (3,1)<->(4,2) is cut-only
        zz = ZigzagPlanner(m, start=self.START)
        probe = zz.probe_point(self.START)
        self.assertIsNotNone(probe)
        c, r = m.world_to_grid(*probe)
        self.assertTrue(m.is_free(c, r), (c, r))
        self.assertLessEqual(c, 3, (c, r))  # cut-only diagonal can't cross
        self.assertIsNotNone(
            best_route(m, self.START, probe, clear_m=0.0))

    def test_probe_walk_min_filters_near_cells(self):
        # Cells closer than walk_min_m (driver arrival band) never win, so
        # standing on the target can't re-pick it and re-deadlock.
        m = self.room(w=3, h=3, pockets=())  # single free cell (1, 1)
        zz = ZigzagPlanner(m, start=self.CORNER)
        self.assertIsNone(zz.probe_point(self.CORNER))
        self.assertEqual(
            m.world_to_grid(*zz.probe_point(self.CORNER, walk_min_m=0.0)),
            (1, 1))

    def test_probe_skips_covered_keeps_flood(self):
        # Covered cells stay traversable but unpickable: the flood must
        # cross a fully covered column and still find the far side. A
        # rejected candidate that stopped the expansion once truncated the
        # search behind the covered wall (probe landed short of it).
        m = self.open_room()
        covered = {(5, r) for r in range(1, 9)}
        zz = ZigzagPlanner(m, start=self.CORNER)
        probe = zz.probe_point(self.CORNER, covered=covered)
        self.assertIsNotNone(probe)
        c, r = m.world_to_grid(*probe)
        self.assertEqual(c, 10)  # far side of the covered wall
        self.assertNotIn((c, r), covered)

    def test_probe_walk_min_is_metres(self):
        # walk_min_m is metres: 0.35 m rejects every cell of a 10x5 cm
        # room (max walk 1 cell); 0.04 m admits the neighbour. A cell-count
        # comparison once let a 5 cm neighbour pass as "far".
        m = self.room(w=4, h=3, pockets=())  # 2 free cells
        zz = ZigzagPlanner(m, start=self.CORNER)
        self.assertIsNone(zz.probe_point(self.CORNER, walk_min_m=0.35))
        self.assertEqual(
            m.world_to_grid(*zz.probe_point(self.CORNER, walk_min_m=0.04)),
            (2, 1))


class GoalBrainTest(RoomCase):
    def setUp(self):
        self.brain = GoalBrain(min_size=2)
        self.m = self.room()
        self.known = self.known_room()

    def test_explore_returns_frontier_goal(self):
        goal, route, status = self.brain.plan(self.m, self.START)
        self.assertIsNotNone(goal)
        self.assertEqual(self.brain.mode, 'explore')
        self.assertTrue(status.startswith('explore'))

    def test_active_exploration_finishes_viewpoint_despite_frontier_fragment_changes(self):
        self.brain.execution_feedback = True
        goal, _, _ = self.brain.plan(self.m, self.START)
        # A scan fills small frontier fragments before the robot reaches its
        # selected observation point. Replanning must not reverse its heading.
        again, route, status = self.brain.plan(self.known, self.START)
        self.assertEqual(again, goal)
        self.assertTrue(status.startswith('explore'))
        self.assertIsNotNone(route)
        _, _, status = self.brain.plan(self.known, goal)
        self.assertTrue(status.startswith('coverage'))

    def test_execution_failure_releases_committed_exploration_viewpoint(self):
        self.brain.execution_feedback = True
        goal, _, _ = self.brain.plan(self.m, self.START)
        self.brain.avoid_goal(goal)
        again, _, _ = self.brain.plan(self.known, self.START)
        self.assertNotEqual(again, goal)

    def test_new_obstacle_releases_committed_viewpoint(self):
        self.brain.execution_feedback = True
        goal, _, _ = self.brain.plan(self.m, self.START)
        self.known.set_cell(*self.known.world_to_grid(*goal), OCC)
        again, _, _ = self.brain.plan(self.known, self.START)
        self.assertNotEqual(again, goal)

    def test_expired_executor_feedback_releases_viewpoint_latch(self):
        self.brain.execution_feedback = True
        goal, _, _ = self.brain.plan(self.m, self.START)
        self.brain.execution_feedback = False
        again, _, status = self.brain.plan(self.known, self.START)
        self.assertTrue(status.startswith('coverage'))
        self.assertNotEqual(again, goal)

    def test_sweeps_coverage_when_no_frontiers(self):
        # No frontier this tick -> the brain falls through to a coverage
        # waypoint for the sweep, while staying ready to explore again.
        goal, route, status = self.brain.plan(self.known, self.START)
        self.assertTrue(status.startswith('coverage'))
        self.assertIsNotNone(goal)

    def test_coverage_done_when_all_covered(self):
        self.brain.mode = 'coverage'
        self.brain.covered = set(self.known.free_cells())
        goal, route, status = self.brain.plan(self.known, self.START)
        self.assertIsNone(goal)
        self.assertEqual(status, 'coverage done')

    def test_unreachable_wp_skipped(self):
        # 5 cm gap is raw-open but sealed at clear_m 0.06: the coverage
        # waypoint behind it cannot be planned to and gets skipped.
        m = self.wall_col(self.known, 4, (1, 2, 4, 5))
        self.brain.mode = 'coverage'
        self.brain.covered = {(c, r) for (c, r) in m.free_cells() if c <= 3}
        before = set(self.brain.covered)
        goal, route, status = self.brain.plan(m, self.START)
        self.assertIsNone(goal)
        self.assertIn('coverage done', status)  # inaccessible lanes are not candidates
        self.assertEqual(self.brain.covered, before)
        self.assertFalse(self.brain._coverage_deferred)

    def test_idle_not_done_on_unknown_map(self):
        # Regression: pose on unknown space (map still filling) must not
        # report coverage done — nothing was swept yet.
        goal, route, status = self.brain.plan(self.dark_room(), (0.225, 0.175))
        self.assertIsNone(goal)
        self.assertIn('idle', status)
        self.assertNotEqual(status, 'coverage done')

    def test_stall_switches_to_alternative(self):
        # Robot pinned at one pose: after stall_plans plans with no
        # progress the frontier is benched and the next-best option wins.
        brain = GoalBrain(min_size=2, stall_plans=2, progress_m=0.05,
                          stall_min_dist=0.05, blacklist_plans=50)
        m = self.room(w=13, h=7, pockets=((3, 2, 3, 3), (9, 1, 11, 4)))
        goal1, _route, st1 = brain.plan(m, self.START)
        self.assertIsNotNone(goal1)
        for _ in range(2):  # same pose again and again = stuck
            goal2, _route, st2 = brain.plan(m, self.START)
        self.assertIn('stall', st2)
        c1 = m.world_to_grid(*goal1)
        c2 = m.world_to_grid(*goal2)
        self.assertNotEqual(c1, c2)
        self.assertIn(c1, brain._blacklist)

    def test_blacklist_expires(self):
        # A benched cell is retried after blacklist_plans plan calls —
        # and a still-stuck robot simply re-benches it (expiry moves up).
        brain = GoalBrain(min_size=2, stall_plans=2, progress_m=0.05,
                          stall_min_dist=0.05, blacklist_plans=3)
        m = self.room(w=13, h=7, pockets=((3, 2, 3, 3), (9, 1, 11, 4)))
        brain.plan(m, self.START)
        for _ in range(2):
            brain.plan(m, self.START)
        first = max(brain._blacklist.values())
        for _ in range(5):
            brain.plan(m, self.START)
        still = max(brain._blacklist.values()) if brain._blacklist else 0
        self.assertGreater(still, first)  # old entry expired, fresh one set

    def test_wide_first_until_clear(self):
        # After a stall the following plans demand wide routes (2-cell
        # inflation, 15 cm gaps sealed) while near the stuck pose.
        brain = GoalBrain(min_size=2, stall_plans=2, progress_m=0.05,
                          stall_min_dist=0.05, blacklist_plans=50)
        m = self.room(w=13, h=7, pockets=((3, 2, 3, 3), (9, 1, 11, 4)))
        brain.plan(m, self.START)
        st = ''
        for _ in range(2):
            st = brain.plan(m, self.START)[2]
        self.assertIn('stall', st)
        st_next = brain.plan(m, self.START)[2]
        self.assertTrue(st_next.startswith('wide-first'), st_next)

    def test_returns_to_explore_when_frontier_reappears(self):
        # A growing map re-opens frontiers: after coverage the brain must
        # try explore again instead of staying 'coverage done'.
        brain = GoalBrain(min_size=2)
        m = self.known_room()
        brain.covered = set(m.free_cells())
        goal, route, status = brain.plan(m, self.START)
        self.assertEqual(status, 'coverage done')
        m.set_cell(6, 3, UNKNOWN)
        m.set_cell(6, 4, UNKNOWN)
        goal, route, status = brain.plan(m, self.START)
        self.assertEqual(brain.mode, 'explore')
        self.assertTrue(status.startswith('explore'), status)

    def test_probe_moves_when_done(self):
        # probe_when_done: all covered + no frontiers -> drive to the
        # farthest reachable cell instead of declaring done (fresh sim maps
        # otherwise deadlock on 'coverage done' at plan #1).
        brain = GoalBrain(min_size=2, probe_when_done=True)
        m = self.known_room()
        brain.covered = set(m.free_cells())
        goal, route, status = brain.plan(m, self.START)
        self.assertIsNotNone(goal)
        self.assertTrue(status.startswith('probe'), status)

    def test_probe_latch_until_reached(self):
        # One probe target holds across plan ticks (probe mode has no stall
        # watchdog) and releases only when reached, so the goal cannot
        # ping-pong between far corners as the robot closes in.
        brain = GoalBrain(min_size=2, probe_when_done=True)
        m = self.known_room()
        brain.covered = set(m.free_cells())
        g1, _r, _s = brain.plan(m, self.START)
        g2, _r, _s = brain.plan(m, self.START)  # same pose: same target
        self.assertEqual(m.world_to_grid(*g2), m.world_to_grid(*g1))
        # Reached (within reach_tol): latch releases, new farthest wins.
        near = m.grid_to_world(*m.world_to_grid(*g1))
        g3, _r, _s = brain.plan(m, near)
        self.assertNotEqual(m.world_to_grid(*g3), m.world_to_grid(*g1))

    def test_probe_rests_after_dead_target(self):
        # A probe with no route at either clearance rests for blacklist_plans
        # plan calls instead of retrying the same dead cell every tick;
        # after the rest a fresh probe is attempted again.
        brain = GoalBrain(min_size=2, probe_when_done=True, blacklist_plans=3)
        m = self.known_room()
        brain.covered = set(m.free_cells())
        with mock.patch('rosy_control.planning.goals.best_route',
                        return_value=None):
            g1, _r, s1 = brain.plan(m, self.START)
            self.assertIsNone(g1)
            self.assertEqual(s1, 'coverage done')
            g2, _r, s2 = brain.plan(m, self.START)  # resting
            self.assertIsNone(g2)
            self.assertGreater(brain._probe_deadline, brain._plan_n)
        for _ in range(2):
            g, _r, s = brain.plan(m, self.START)
        self.assertIsNotNone(g)
        self.assertTrue(s.startswith('probe'), s)


class ManualGoalTest(RoomCase):
    """Dashboard click-to-send: latched external goal in the brain."""

    def setUp(self):
        self.brain = GoalBrain(min_size=2)
        self.m = self.room()

    def test_parse_goal_cmd(self):
        self.assertEqual(parse_goal_cmd('1.5,2.0'), (1.5, 2.0))
        self.assertEqual(parse_goal_cmd('1 2'), (1.0, 2.0))
        self.assertIsNone(parse_goal_cmd('explore'))
        self.assertIsNone(parse_goal_cmd('nan,1'))
        self.assertIsNone(parse_goal_cmd('1,inf'))
        self.assertIsNone(parse_goal_cmd('1,2,3'))
        self.assertIsNone(parse_goal_cmd(''))

    def test_manual_goal_wins_over_explore(self):
        self.brain.set_manual(*self.EAST)
        goal, route, status = self.brain.plan(self.m, self.START)
        self.assertTrue(status.startswith('manual goal='))
        self.assertEqual(goal, self.EAST)
        self.assertIsNotNone(route)

    def test_manual_cleared_on_arrival(self):
        self.brain.set_manual(*self.START)
        goal, route, status = self.brain.plan(self.m, self.START)
        self.assertEqual(status, 'manual goal reached')
        self.assertIsNone(goal)
        self.assertIsNone(self.brain._manual)
        goal, route, status = self.brain.plan(self.m, self.START)
        self.assertTrue(status.startswith('explore'))

    def test_manual_cleared_by_verb(self):
        self.brain.set_manual(*self.EAST)
        self.brain.clear_manual()
        goal, route, status = self.brain.plan(self.m, self.START)
        self.assertTrue(status.startswith('explore'))

    def test_manual_unreachable_cleared_after_stall(self):
        brain = GoalBrain(min_size=2, stall_plans=2)
        brain.set_manual(*self.EAST)
        # best_route snaps to nearest free cells, so no fixture goal is
        # unroutable — mock the router to exercise the unreachable branch.
        with mock.patch('rosy_control.planning.goals.best_route',
                        return_value=None):
            for _ in range(2):
                goal, route, status = brain.plan(self.m, self.START)
                self.assertIn('unreachable', status)
        goal, route, status = brain.plan(self.m, self.START)
        self.assertIsNone(brain._manual)
        self.assertTrue(status.startswith('explore'))

    def test_manual_ttl_expiry(self):
        brain = GoalBrain(min_size=2, manual_ttl_plans=2)
        brain.set_manual(*self.EAST)
        for _ in range(2):
            goal, route, status = brain.plan(self.m, self.START)
            self.assertTrue(status.startswith('manual goal='))
        goal, route, status = brain.plan(self.m, self.START)
        self.assertIsNone(brain._manual)
        self.assertTrue(status.startswith('explore'))


if __name__ == '__main__':
    unittest.main()


class PlannerBlockedStartRegression(unittest.TestCase):
    def test_failed_goal_expires_even_while_start_is_blocked(self):
        m = OccupancyMap(20, 20, .02, fill=FREE)
        m.set_cell(5, 5, OCC)
        brain = GoalBrain(clear_m=.12, start_escape_clear_m=.096, blacklist_plans=3)
        pose = m.grid_to_world(6, 5)
        brain.avoid_goal(m.grid_to_world(12, 12))
        for _ in range(3):
            goal, route, status = brain.plan(m, pose)
            self.assertIsNone(route)
            self.assertIn('obstacle clearance', status)
        self.assertEqual(brain._failed_goals, [])

    def test_occupied_start_is_not_coverage_done_or_snapped(self):
        m = OccupancyMap(20, 20, .02, fill=FREE)
        m.set_cell(5, 5, OCC)
        brain = GoalBrain(clear_m=.04)
        goal, route, status = brain.plan(m, m.grid_to_world(5, 5))
        self.assertIsNone(route)
        self.assertIn('robot cell occupied', status)

    def test_coverage_candidates_use_executable_clearance(self):
        m = OccupancyMap(30, 20, .02, fill=FREE)
        for c in range(m.w):
            m.set_cell(c, 0, OCC); m.set_cell(c, m.h-1, OCC)
        for r in range(m.h):
            m.set_cell(0, r, OCC); m.set_cell(m.w-1, r, OCC)
        brain = GoalBrain(clear_m=.08)
        brain.mode = 'coverage'
        pose = m.grid_to_world(10, 10)
        goal, route, status = brain.plan(m, pose)
        self.assertIsNotNone(route, status)
        self.assertTrue(all(m.inflate(.08/m.res).is_free(*c) for c in route['cells']))


class NarrowFrontierRouteRegression(unittest.TestCase):
    def test_wide_start_can_reach_frontier_through_minimum_clearance_corridor(self):
        m = OccupancyMap(40, 25, .02, fill=OCC)
        for r in range(1, 24):
            for c in range(1, 39):
                m.set_cell(c, r, FREE)
        # Divider doorway is open at the explicit hard radius, closed at preferred.
        for r in range(1, 24):
            if not 8 <= r <= 16:
                m.set_cell(20, r, OCC)
        for r in range(1, 24):
            m.set_cell(38, r, UNKNOWN)
        pose = m.grid_to_world(10, 12)
        brain = GoalBrain(min_size=3, clear_m=.12, retry_clear_m=.12,
                          start_escape_clear_m=.08)
        self.assertTrue(m.inflate(.12/m.res).is_free(*m.world_to_grid(*pose)))
        goal, route, status = brain.plan(m, pose)
        self.assertIsNotNone(route, status)
        self.assertTrue(status.startswith('explore'), status)
        self.assertGreater(goal[0], m.grid_to_world(20, 12)[0])
        self.assertEqual(route['clearance_m'], .08)
        self.assertTrue(all(m.inflate(.08/m.res).is_free(*c) for c in route['cells']))


class ManualOnlyRegression(unittest.TestCase):
    def test_expiry_never_selects_autonomous_target(self):
        m = OccupancyMap(20, 20, .02, fill=FREE)
        brain = GoalBrain(clear_m=.02, manual_ttl_plans=1)
        brain.mode = 'manual'
        brain.set_manual(.3, .3)
        brain.plan(m, (.1, .1))
        goal, route, status = brain.plan(m, (.1, .1))
        self.assertIsNone(route)
        self.assertEqual(status, 'manual goal expired')
        goal, route, status = brain.plan(m, (.1, .1))
        self.assertIsNone(route)
        self.assertEqual(status, 'manual goal finished')

    def test_reached_goal_does_not_resume_exploration_on_next_tick(self):
        m = OccupancyMap(20, 20, .02, fill=FREE)
        brain = GoalBrain(clear_m=.02)
        brain.mode = 'manual'
        brain.set_manual(.1, .1)
        self.assertEqual(brain.plan(m, (.1, .1))[2], 'manual goal reached')
        self.assertIsNone(brain.plan(m, (.1, .1))[1])


class NarrowCoverageRouteRegression(unittest.TestCase):
    def test_coverage_does_not_finish_before_reachable_narrow_room(self):
        m = OccupancyMap(40, 25, .02, fill=OCC)
        for r in range(1, 24):
            for c in range(1, 39):
                m.set_cell(c, r, FREE)
        for r in range(1, 24):
            if not 8 <= r <= 16:
                m.set_cell(20, r, OCC)
        brain = GoalBrain(clear_m=.12, retry_clear_m=.12, start_escape_clear_m=.08)
        brain.mode = 'coverage'
        brain.covered = {c for c in m.free_cells() if c[0] <= 20}
        goal, route, status = brain.plan(m, m.grid_to_world(10, 12))
        self.assertIsNotNone(route, status)
        self.assertGreater(goal[0], m.grid_to_world(20, 12)[0])
        self.assertEqual(route['clearance_m'], .08)


class ArrivalProgressionRegression(unittest.TestCase):
    def test_coverage_arrival_selects_next_point_without_failure_blacklist(self):
        m = OccupancyMap(30, 30, .02, fill=FREE)
        brain = GoalBrain(clear_m=.04)
        brain.mode = 'coverage'
        goal, route, _ = brain.plan(m, (.3, .3))
        self.assertIsNotNone(goal)
        brain.complete_goal(m, goal, goal)
        nxt, route, status = brain.plan(m, goal)
        self.assertIsNotNone(nxt, status)
        self.assertGreater(math.dist(goal, nxt), brain.reach_tol)
        self.assertEqual(brain._failed_goals, [])
        self.assertEqual(brain._failed_exits, [])
        brain.reset()
        self.assertEqual(brain._completed_goals, [])

    def test_completed_frontier_neighborhood_excluded_on_immediate_replan(self):
        m = OccupancyMap(30, 30, .02, fill=FREE)
        for r in range(30):
            m.set_cell(29, r, UNKNOWN)
        brain = GoalBrain(clear_m=.02, min_size=3)
        target, route, _ = brain.plan(m, (.2, .3))
        brain.complete_goal(m, target, target)
        with mock.patch('rosy_control.planning.goals.pick_goal', return_value=None) as pick:
            brain.plan(m, target)
            self.assertIn(m.world_to_grid(*target), pick.call_args.kwargs['exclude'])


class RecoveryRestartRegression(unittest.TestCase):
    def test_explicit_restart_discards_failed_exits_but_preserves_visits(self):
        brain = GoalBrain()
        brain.covered.add((2,3))
        brain.avoid_route_exit((.2,.3));brain.avoid_goal((.4,.5))
        brain.restart_recovery()
        self.assertEqual(brain._failed_exits, [])
        self.assertEqual(brain._failed_goals, [])
        self.assertEqual(brain.covered, {(2,3)})
