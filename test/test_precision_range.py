import math
from types import SimpleNamespace
import unittest

from move_control.sensing.precision_range import precision_axis_range


def wall(distance=.3, slope=math.tan(math.radians(60)), nose=math.pi):
    scan = SimpleNamespace(angle_min=-math.pi, angle_increment=math.pi/360,
                           range_min=.05, range_max=40., ranges=[])
    for i in range(720):
        angle = scan.angle_min+i*scan.angle_increment-nose
        denominator = math.cos(angle)-slope*math.sin(angle)
        scan.ranges.append(distance/denominator if denominator > 0 else math.inf)
    return scan


class PrecisionRangeTest(unittest.TestCase):
    def test_oblique_wall_recovers_forward_translation_across_angle_wrap(self):
        for nose in (math.pi, math.pi+math.radians(10), 0.):
            before = precision_axis_range(wall(.3, nose=nose), nose)
            after = precision_axis_range(wall(.27, nose=nose), nose)
            self.assertAlmostEqual(before-after, .03, places=7)

    def test_missing_center_is_not_interpolated(self):
        scan = wall()
        scan.ranges[0] = math.inf
        self.assertTrue(math.isinf(precision_axis_range(scan, math.pi)))

    def test_isolated_return_and_mixed_wall_are_rejected(self):
        scan = wall()
        scan.ranges[2] += .025
        self.assertTrue(math.isinf(precision_axis_range(scan, math.pi)))
        scan = wall()
        for i in range(1, 9):
            scan.ranges[i] += .025
        self.assertTrue(math.isinf(precision_axis_range(scan, math.pi)))

    def test_steep_grazing_wall_and_one_sided_points_are_rejected(self):
        self.assertTrue(math.isinf(precision_axis_range(wall(slope=3.), math.pi)))
        scan = wall()
        scan.ranges = scan.ranges[:8]
        self.assertTrue(math.isinf(precision_axis_range(scan, math.pi)))

    def test_small_sensor_noise_preserves_axis_distance(self):
        scan = wall()
        scan.ranges = [v + (i % 3-1)*.0005 for i, v in enumerate(scan.ranges)]
        self.assertAlmostEqual(precision_axis_range(scan, math.pi), .3, delta=.001)
