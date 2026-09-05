#!/usr/bin/env python3
import math
import unittest

from move_control.lidar import find_frontiers, robot_yaw


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


if __name__ == '__main__':
    unittest.main()
