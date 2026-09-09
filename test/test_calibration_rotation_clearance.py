import math
import unittest

from rosy_control.control.calibration_rotation_clearance import calibration_rotation_clearance


class CalibrationRotationClearanceTests(unittest.TestCase):
    def report(self, ranges=None, **kwargs):
        args = dict(ranges=[.13]*720 if ranges is None else ranges, increment=math.tau/720, scan_age=.1,
                    gate={'can_rotate': True, 'geometry_revision': 'current'}, gate_age=.02,
                    geometry_revision='current')
        args.update(kwargs)
        return calibration_rotation_clearance(**args)

    def test_final_gate_replaces_ad_hoc_extra_five_centimeters(self):
        self.assertTrue(self.report()['clear'])
        self.assertEqual(self.report()['authority'], 'safety_motion_limits')

    def test_partial_scan_remains_partial_while_final_gate_owns_clearance(self):
        ranges = [.4]*720
        ranges[100:120] = [math.inf]*20
        report = self.report(ranges)
        self.assertTrue(report['clear'])
        self.assertFalse(report['fully_observed'])
        self.assertEqual(report['missing_bins'], 20)
        self.assertAlmostEqual(report['maximum_gap_deg'], 10.5)
        self.assertFalse(self.report(ranges, gate={'can_rotate': False, 'geometry_revision': 'current'})['clear'])

    def test_missing_stale_or_wrong_geometry_gate_cannot_authorize(self):
        for options in ({'gate': None}, {'gate_age': .251}, {'gate_age': -1.},
                        {'scan_age': .251}, {'gate': {'can_rotate': True, 'geometry_revision': 'old'}}):
            self.assertFalse(self.report(**options)['clear'])

    def test_invalid_scan_and_seam_gap_are_reported_truthfully(self):
        for ranges, increment in [([], .01), ([math.inf]*720, math.tau/720), ([.4]*720, math.nan),
                                   ([.4]*720, .01)]:
            self.assertFalse(self.report(ranges, increment=increment)['clear'])
        ranges = [.4]*720
        ranges[:3] = ranges[-3:] = [math.inf]*3
        self.assertAlmostEqual(self.report(ranges)['maximum_gap_deg'], 3.5)
