import unittest
from rosy_control.control.space_speed import limit_for_space
from rosy_control.control.safety_profile import SafetyProfile


class SpaceSpeedTest(unittest.TestCase):
    def test_gap_reduces_pair_without_minimum_creep(self):
        profile = SafetyProfile.build()
        v, w, reason = limit_for_space(.014, .08, [.123, .5, .5, .5], profile)
        self.assertLess(v, .014)
        self.assertAlmostEqual(w/v, .08/.014)
        self.assertEqual(reason, 'space_limit')
        self.assertEqual(limit_for_space(.014, 0., [.12, .5, .5, .5], profile)[:2], (0., 0.))

    def test_unknown_side_and_reverse_clearance_are_not_free_space(self):
        profile = SafetyProfile.build()
        self.assertEqual(limit_for_space(.01, 0., [.5, .5, float('inf'), .5], profile)[:2], (0., 0.))
        self.assertEqual(limit_for_space(-.01, 0., [.5, .12, .5, .5], profile)[:2], (0., 0.))
