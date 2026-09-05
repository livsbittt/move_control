#!/usr/bin/env python3
import unittest

from move_control.gridmap import FREE, OCC, UNKNOWN, OccupancyMap
from move_control.astar import best_route
from move_control.frontier import frontier_points, pick_goal
from move_control.zigzag import ZigzagPlanner


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
