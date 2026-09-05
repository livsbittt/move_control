#!/usr/bin/env python3
import math
import unittest

from move_control.lidar import MOUNT_YAW_DEG, NOSE_YAW, robot_yaw
from move_control.route import line_route
from test.test_frontier import FakeScan


def _front_open_scan():
    n = 720
    amin = -math.pi
    inc = 2.0 * math.pi / n
    ranges = []
    for i in range(n):
        ang = amin + i * inc
        yaw = robot_yaw(ang, math.pi, False)
        if abs(yaw) < math.radians(10.0):
            ranges.append(0.40)
        else:
            ranges.append(0.11)
    return FakeScan(ranges, amin, inc)


class RouteTest(unittest.TestCase):
    def test_nose_yaw_is_front(self):
        self.assertAlmostEqual(MOUNT_YAW_DEG, 10.0)
        self.assertAlmostEqual(robot_yaw(NOSE_YAW, NOSE_YAW), 0.0, places=6)
        self.assertAlmostEqual(math.degrees(robot_yaw(NOSE_YAW, math.pi)), 10.0, places=5)

    def test_picks_straight_ahead(self):
        line = line_route(_front_open_scan(), yaw_offset=math.pi, occ=0.12, max_r=0.45)
        self.assertIsNotNone(line)
        self.assertLess(abs(line['yaw']), math.radians(12.0), line)
        self.assertGreater(line['length'], 0.20)

    def test_rejects_empty(self):
        self.assertIsNone(line_route(None))


if __name__ == '__main__':
    unittest.main()
