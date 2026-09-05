#!/usr/bin/env python3
import unittest

from move_control.sensing.body import (
    URDF_RADIUS,
    ignore_m,
    turn_clear_m,
    urdf_radius,
    use_radius,
)


class BodyTest(unittest.TestCase):
    def test_urdf_is_rear_caster(self):
        r = urdf_radius()
        self.assertAlmostEqual(r, URDF_RADIUS)
        self.assertGreater(r, 0.068)
        self.assertLess(r, 0.090)
        self.assertAlmostEqual(r, 0.076, places=3)

    def test_calib_param_wins(self):
        self.assertAlmostEqual(use_radius(0.080), 0.080)
        self.assertAlmostEqual(use_radius(0.076), 0.076)

    def test_bad_calib_falls_back(self):
        self.assertAlmostEqual(use_radius(None), URDF_RADIUS)
        self.assertAlmostEqual(use_radius(0.0), URDF_RADIUS)
        self.assertAlmostEqual(use_radius(1.5), URDF_RADIUS)
        self.assertAlmostEqual(use_radius(float('nan')), URDF_RADIUS)

    def test_turn_clear_bigger_than_body(self):
        c = turn_clear_m(0.076)
        self.assertGreater(c, 0.076)
        self.assertAlmostEqual(c, 0.086, places=3)

    def test_ignore_near_four_cm(self):
        lo = ignore_m(0.076)
        self.assertGreaterEqual(lo, 0.035)
        self.assertLessEqual(lo, 0.055)


if __name__ == '__main__':
    unittest.main()
