import math
import unittest

from move_control.control.safety_profile import SafetyProfile, bounded_command


class ProfileTest(unittest.TestCase):
    def test_legacy_distances_cannot_shrink_nominal_physical_limits(self):
        profile = SafetyProfile.build(radius=.04, stop=.018, clear=.028, half_width_deg=8.)
        self.assertGreaterEqual(profile.radius, .076)
        self.assertGreaterEqual(profile.stop, .12)
        self.assertGreaterEqual(profile.clear, .14)
        self.assertGreaterEqual(profile.half_width_deg, 45.)
        self.assertEqual(profile.revision, SafetyProfile.build(radius=.04, stop=.018, clear=.028, half_width_deg=8.).revision)
        self.assertFalse(profile.report()['commissioned'])

    def test_larger_body_increases_limits_and_changes_revision(self):
        a, b = SafetyProfile.build(), SafetyProfile.build(radius=.12)
        self.assertGreater(b.stop, a.stop)
        self.assertGreater(b.turn_clear, a.turn_clear)
        self.assertNotEqual(a.revision, b.revision)
        with self.assertRaises(ValueError):
            SafetyProfile.build(radius=float('nan'))

    def test_post_gain_limits_preserve_curvature_and_reject_nan(self):
        self.assertEqual(bounded_command(float('nan'), .1, .014, .1), (0., 0., 'invalid_command'))
        v, w, reason = bounded_command(.028, .2, .014, .1)
        self.assertAlmostEqual(v, .014)
        self.assertAlmostEqual(w, .1)
        self.assertEqual(reason, 'speed_limit')
        v, w, _ = bounded_command(-.028, .05, .014, .1)
        self.assertAlmostEqual(w/v, .05/-.028)
        self.assertTrue(math.isfinite(v))
