import unittest
from rosy_control.control.rotation_clearance import rotation_clearance_allowed


class RotationClearanceTest(unittest.TestCase):
    def test_partial_scan_retains_conservative_swept_body_check(self):
        self.assertTrue(rotation_clearance_allowed(True, False, True, None, (.2,)*6))
        self.assertFalse(rotation_clearance_allowed(False, False, True, .1, (.2,)*6))
        self.assertFalse(rotation_clearance_allowed(True, False, False, .1, (.2,)*6))

    def test_full_scan_pivot_optimization_requires_actual_margin(self):
        self.assertTrue(rotation_clearance_allowed(False, True, True, .02, (.2,)*6))
        for margin in (None, .005, float('nan')):
            self.assertFalse(rotation_clearance_allowed(True, True, True, margin, (.2,)*6))
        self.assertFalse(rotation_clearance_allowed(True, True, True, .02, (.2,float('inf'))))
