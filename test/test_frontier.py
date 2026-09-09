#!/usr/bin/env python3
import math
import unittest

from rosy_control.sensing.lidar import find_frontiers, robot_yaw
from rosy_control.planning.gridmap import OccupancyMap, FREE, OCC, UNKNOWN
from rosy_control.planning.frontier import pick_goal, _reachable_costs


class _Stamp:
    sec = 2_000_000_000
    nanosec = 0


class _Hdr:
    stamp = _Stamp()


class FakeScan:
    def __init__(self, ranges, angle_min=-math.pi, inc=None, range_max=40.0):
        self.header = _Hdr()
        self.angle_min = float(angle_min)
        self.angle_increment = float(inc if inc is not None else (2.0 * math.pi / len(ranges)))
        self.range_max = float(range_max)
        self.ranges = list(ranges)


def _maze_scan():
    n = 720
    amin = -math.pi
    inc = 2.0 * math.pi / n
    ranges = []
    for i in range(n):
        ang = amin + i * inc
        yaw = robot_yaw(ang, math.pi, False)
        if abs(yaw) < math.radians(12.0):
            ranges.append(0.42)
        elif abs(yaw) < math.radians(35.0):
            ranges.append(0.13)
        else:
            ranges.append(0.12)
    return FakeScan(ranges, amin, inc)


class FrontierTest(unittest.TestCase):
    def test_finds_front_opening(self):
        fr = find_frontiers(_maze_scan(), yaw_offset=math.pi, occ=0.16, free=0.22, max_r=0.45)
        self.assertTrue(fr, 'expected a frontier at the front gap')
        best = fr[0]
        self.assertLess(abs(best['yaw']), math.radians(20.0), best)
        self.assertGreater(best['depth'], 0.25)

    def test_empty_scan(self):
        self.assertEqual(find_frontiers(None), [])


class MapFrontierTest(unittest.TestCase):
    def divided_frontier(self):
        m = OccupancyMap(21, 12, .02, fill=OCC)
        for r in range(1, 10):
            for c in range(1, 20):
                m.set_cell(c, r, FREE)
        for c in range(1, 20):
            m.set_cell(c, 10, UNKNOWN)
        for r in range(1, 9):
            m.set_cell(7, r, OCC)
        return m, m.grid_to_world(3, 4)

    def test_unreachable_centroid_keeps_reachable_cluster_portion(self):
        m, start = self.divided_frontier()
        goal = pick_goal(m, start, min_size=2, clear_m=.02)
        self.assertIsNotNone(goal)
        self.assertLess(goal['route']['cells'][-1][0], 7)
        self.assertEqual(goal['clear_m'], .02)
        self.assertTrue(all(m.inflate(1).is_free(*cell)
                            for cell in goal['route']['cells']))

    def test_excluded_target_keeps_other_cluster_members(self):
        m, start = self.divided_frontier()
        first = pick_goal(m, start, min_size=2, clear_m=.02)
        excluded = {first['route']['cells'][-1], (10, 9)}
        alternative = pick_goal(m, start, min_size=2, clear_m=.02,
                                exclude=excluded)
        self.assertIsNotNone(alternative)
        self.assertNotIn(alternative['route']['cells'][-1], excluded)

    def test_identical_clearance_pass_has_no_duplicate_alternative(self):
        m, start = self.divided_frontier()
        goal = pick_goal(m, start, min_size=2, clear_m=.02,
                         retry_clear_m=.02)
        self.assertEqual(len(goal['options']), 1)
        self.assertEqual(goal['clear_m'], .02)

    def test_reachability_does_not_cut_unknown_corners(self):
        m = OccupancyMap(3, 3, .02, fill=UNKNOWN)
        m.set_cell(0, 0, FREE)
        m.set_cell(1, 1, FREE)
        self.assertEqual(set(_reachable_costs(m, m.grid_to_world(0, 0))),
                         {(0, 0)})

    def test_inflated_start_is_not_snapped(self):
        m, _ = self.divided_frontier()
        self.assertIsNone(pick_goal(m, m.grid_to_world(6, 4),
                                   min_size=2, clear_m=.02))


if __name__ == '__main__':
    unittest.main()
