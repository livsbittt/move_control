import math
import unittest
from move_control.sensing.lidar_mount import nose_from_quaternion


class LidarMountTest(unittest.TestCase):
    def test_measured_half_turn_mount_has_nose_180(self):
        self.assertAlmostEqual(nose_from_quaternion(0, 0, 1, 0), math.pi)

    def test_tf_170_corresponds_to_scan_nose_190_not_170(self):
        yaw = math.radians(170)
        nose = nose_from_quaternion(0, 0, math.sin(yaw/2), math.cos(yaw/2))
        self.assertAlmostEqual(nose, math.radians(190))

    def test_invalid_rotation_rejected(self):
        for q in ((0, 0, 0, 0), (0, 0, math.nan, 1)):
            with self.assertRaises(ValueError):
                nose_from_quaternion(*q)
