"""Isolated Gazebo scans share the C1 heading path without weakening production."""
import math
import unittest
from types import SimpleNamespace

from rosy_control.control.route import line_route
from rosy_control.sensing import lidar
from rosy_control.sensing.lidar import find_frontiers, is_robot_scan, is_simulation_scan


def _scan(ranges, *, stamp=2_000_000_000, range_max=40.0, frame='laser'):
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=stamp, nanosec=0),
            frame_id=frame,
        ),
        angle_min=-math.pi,
        angle_increment=2.0 * math.pi / len(ranges),
        range_min=0.05,
        range_max=range_max,
        ranges=list(ranges),
    )


def _open_front(n=720, **kwargs):
    ranges = []
    inc = 2.0 * math.pi / n
    for i in range(n):
        yaw = lidar.robot_yaw(-math.pi + i * inc, math.pi)
        ranges.append(0.40 if abs(yaw) < math.radians(12.0) else 0.12)
    return _scan(ranges, **kwargs)


class ScanAcceptanceTest(unittest.TestCase):
    def tearDown(self):
        lidar.enable_simulation_scans(False)

    def test_production_rejects_sim_time_gpu_lidar(self):
        scan = _open_front(stamp=12, range_max=8.0, frame='pinky/base/lidar')
        self.assertTrue(is_simulation_scan(scan))
        self.assertFalse(is_robot_scan(scan))
        self.assertEqual(find_frontiers(scan, yaw_offset=math.pi), [])
        self.assertIsNone(line_route(scan, yaw_offset=math.pi))

    def test_production_still_accepts_c1_wall_clock_scan(self):
        scan = _open_front()
        self.assertTrue(is_robot_scan(scan))
        self.assertTrue(find_frontiers(scan, yaw_offset=math.pi, occ=0.16, free=0.22))

    def test_isolated_rig_enables_the_same_frontier_and_route_path(self):
        scan = _open_front(stamp=12, range_max=8.0, frame='pinky/base/lidar')
        lidar.enable_simulation_scans(True)
        self.assertTrue(is_robot_scan(scan))
        frontiers = find_frontiers(scan, yaw_offset=math.pi, occ=0.16, free=0.22)
        self.assertTrue(frontiers)
        self.assertLess(abs(frontiers[0]['yaw']), math.radians(20.0))
        line = line_route(scan, yaw_offset=math.pi, occ=0.12)
        self.assertIsNotNone(line)
        self.assertGreater(line['length'], 0.20)

    def test_simulation_policy_still_rejects_foreign_scan_shape(self):
        lidar.enable_simulation_scans(True)
        wrong_count = _open_front(n=640, stamp=12, range_max=8.0, frame='pinky/base/lidar')
        wrong_frame = _open_front(stamp=12, range_max=8.0, frame='gazebo/lidar')
        self.assertFalse(is_robot_scan(wrong_count))
        self.assertFalse(is_robot_scan(wrong_frame))

    def test_rebinding_a_copied_name_does_not_open_find_frontiers(self):
        scan = _open_front(stamp=12, range_max=8.0, frame='pinky/base/lidar')
        copied = lambda msg: True
        self.assertTrue(copied(scan))
        self.assertFalse(is_robot_scan(scan))
        self.assertEqual(find_frontiers(scan, yaw_offset=math.pi), [])
