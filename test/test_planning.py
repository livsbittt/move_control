#!/usr/bin/env python3
import math
import unittest

from move_control.planning import (FREE, OCC, UNKNOWN, GoalBrain,
                                   OccupancyMap, ZigzagPlanner, best_route,
                                   frontier_points, pick_goal)


def room(w=9, h=7, pockets=((5, 3, 6, 4),)):
    """Walled room at 5 cm res; optional unknown pockets (c0, r0, c1, r1)."""
    m = OccupancyMap(w, h, 0.05, fill=FREE)
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


class GridMapTest(unittest.TestCase):
    def test_roundtrip(self):
        m = room()
        c, r = m.world_to_grid(0.225, 0.125)
        x, y = m.grid_to_world(c, r)
        self.assertAlmostEqual(x, 0.225, places=2)
        self.assertAlmostEqual(y, 0.125, places=2)

    def test_outside_is_wall(self):
        m = room()
        self.assertEqual(m.cell(-1, 0), OCC)
        self.assertEqual(m.cell(99, 99), OCC)

    def test_free_bands(self):
        m = room(pockets=())
        m.set_cell(2, 2, UNKNOWN)
        m.set_cell(2, 3, 50)
        m.set_cell(2, 4, 70)
        self.assertFalse(m.is_free(2, 2))  # unknown never free
        self.assertTrue(m.is_free(2, 3))   # 0..64 free-ish
        self.assertFalse(m.is_free(2, 4))  # >= 65 is a wall

    def test_inflate_keeps_unknown(self):
        # 13x11 room so (7,6) sits >2 cells from every wall and the ring.
        m = room(w=13, h=11, pockets=())
        m.set_cell(4, 3, OCC)
        m.set_cell(3, 6, UNKNOWN)  # >2 cells from walls and the ring
        inf = m.inflate(2)
        self.assertEqual(inf.cell(4, 3), OCC)
        self.assertEqual(inf.cell(5, 4), OCC)   # diagonal inside the ring
        self.assertEqual(inf.cell(7, 6), FREE)  # outside the ring
        self.assertEqual(inf.cell(3, 6), UNKNOWN)  # unknown never inflates


class AstarTest(unittest.TestCase):
    def wall_col(self, m, c, rows, v=OCC):
        for r in rows:
            m.set_cell(c, r, v)
        return m

    def test_around_wall(self):
        m = self.wall_col(room(pockets=()), 4, (1, 2, 4, 5))
        r = best_route(m, (0.125, 0.125), (0.325, 0.125), clear_m=0.0)
        self.assertIsNotNone(r)
        self.assertGreater(r['length'], 0.2)  # detour, not the straight line

    def test_closed_wall_no_route(self):
        m = self.wall_col(room(pockets=()), 4, (1, 2, 3, 4, 5))
        self.assertIsNone(best_route(m, (0.125, 0.125), (0.325, 0.125)))

    def test_route_never_touches_wall(self):
        # clear_m 0.0: cells on the route must all be free on the raw map.
        m = self.wall_col(room(pockets=()), 4, (1, 2, 4, 5))
        r = best_route(m, (0.125, 0.125), (0.325, 0.125), clear_m=0.0)
        self.assertIsNotNone(r)
        for c, rr in r['cells']:
            self.assertTrue(m.is_free(c, rr), (c, rr))

    def test_goal_on_wall_snaps(self):
        # Clicking a wall goes to the nearest free cell (5-7 cm away),
        # so a sloppy map tap still yields a drivable point.
        m = room(pockets=())
        r = best_route(m, (0.125, 0.125), (0.025, 0.025))
        self.assertIsNotNone(r)
        self.assertEqual(r['cells'][-1], (2, 2))

    def test_three_cell_gap_passes(self):
        # 15 cm gap survives clear_m 0.05 (2*0.05 + 1 cell = robot diameter).
        m = room(w=9, h=11, pockets=())
        for rr in (1, 2, 6, 7, 8, 9):
            m.set_cell(4, rr, OCC)
        r = best_route(m, (0.125, 0.225), (0.325, 0.225), clear_m=0.05)
        self.assertIsNotNone(r)
        for c, rr in r['cells']:
            self.assertTrue(m.is_free(c, rr), (c, rr))


class FrontierTest(unittest.TestCase):
    def test_pick_goal_reachable(self):
        m = room()
        g = pick_goal(m, (0.125, 0.125), min_size=2)
        self.assertIsNotNone(g)
        self.assertEqual(g['kind'], 'frontier')
        c, r = m.world_to_grid(g['x'], g['y'])
        self.assertTrue(m.is_free(c, r), (c, r))
        self.assertGreaterEqual(g['route']['length'], 0.08)

    def test_pick_goal_none_when_mapped(self):
        self.assertIsNone(pick_goal(room(pockets=()), (0.125, 0.125)))

    def test_pick_goal_retry_sealed_corridor(self):
        # 5 cm gap: raw-open, sealed at clear_m 0.06. The retry pass at 0.0
        # still finds an approach to the frontier behind it.
        m = room(w=9, h=7, pockets=((6, 2, 7, 3),))
        for rr in (1, 2, 4, 5):
            m.set_cell(4, rr, OCC)
        self.assertIsNone(pick_goal(m, (0.125, 0.125), min_size=2))
        g = pick_goal(m, (0.125, 0.125), min_size=2, retry_clear_m=0.0)
        self.assertIsNotNone(g)
        self.assertEqual(g['clear_m'], 0.0)

    def test_pick_goal_skips_degenerate(self):
        m = room()
        self.assertIsNone(pick_goal(m, (0.125, 0.125), min_size=2,
                                    min_route_m=99.0))
        self.assertIsNotNone(pick_goal(m, (0.125, 0.125), min_size=2,
                                       min_route_m=0.0))


class ZigzagTest(unittest.TestCase):
    def test_coverage_completes(self):
        m = room(w=12, h=10, pockets=())
        zz = ZigzagPlanner(m, start=(0.075, 0.075), lane_width=0.12,
                           lane_step=0.20)
        wps = zz.wps  # cells; waypoints() is the world-metre view
        self.assertTrue(wps)
        for wc, wr in wps:
            self.assertTrue(m.is_free(wc, wr), (wc, wr))
        # Every free cell within half a lane + half a step of a waypoint.
        tol = (0.12 / 2 + 0.20 / 2) / 0.05
        for c, r in m.free_cells():
            d = min(math.hypot(c - wc, r - wr) for wc, wr in wps)
            self.assertLessEqual(d, tol + 1e-9, (c, r))

    def test_covered_lane_dropped(self):
        m = room(w=12, h=10, pockets=())
        cov = {(c, 1) for c in range(1, 11)}
        zz = ZigzagPlanner(m, start=(0.075, 0.075), covered=cov)
        self.assertNotIn(1, [lane['key'] for lane in zz.lanes])
        zz_all = ZigzagPlanner(m, start=(0.075, 0.075))
        self.assertIn(1, [lane['key'] for lane in zz_all.lanes])

    def test_serpentine_alternates(self):
        m = room(w=12, h=10, pockets=())
        zz = ZigzagPlanner(m, start=(0.075, 0.075))
        keys = list(dict.fromkeys(r for _c, r in zz.wps))
        self.assertGreaterEqual(len(keys), 3)
        dirs = []
        for key in keys:
            lane_wps = [(c, r) for c, r in zz.wps if r == key]
            dirs.append(lane_wps[0][0] < lane_wps[-1][0])
        for a, b in zip(dirs, dirs[1:]):
            self.assertNotEqual(a, b)


class GoalBrainTest(unittest.TestCase):
    def test_explore_returns_frontier_goal(self):
        brain = GoalBrain(min_size=2)
        goal, route, status = brain.plan(room(), (0.125, 0.125))
        self.assertIsNotNone(goal)
        self.assertEqual(brain.mode, 'explore')
        self.assertTrue(status.startswith('explore'))

    def test_transitions_to_coverage_when_mapped(self):
        brain = GoalBrain()
        goal, route, status = brain.plan(room(pockets=()), (0.125, 0.125))
        self.assertEqual(brain.mode, 'coverage')
        self.assertTrue(status.startswith('coverage'))

    def test_coverage_done_when_all_covered(self):
        m = room(pockets=())
        brain = GoalBrain()
        brain.mode = 'coverage'
        brain.covered = set(m.free_cells())
        goal, route, status = brain.plan(m, (0.125, 0.125))
        self.assertIsNone(goal)
        self.assertEqual(status, 'coverage done')

    def test_unreachable_wp_skipped(self):
        # 5 cm gap is raw-open but sealed at clear_m 0.06: the coverage
        # waypoint behind it cannot be planned to and gets skipped.
        m = room(pockets=())
        for rr in (1, 2, 4, 5):
            m.set_cell(4, rr, OCC)
        brain = GoalBrain()
        brain.mode = 'coverage'
        brain.covered = {(c, r) for (c, r) in m.free_cells() if c <= 3}
        before = len(brain.covered)
        goal, route, status = brain.plan(m, (0.125, 0.125))
        self.assertIsNone(goal)
        self.assertIn('skip', status)
        self.assertGreater(len(brain.covered), before)  # wp marked swept
