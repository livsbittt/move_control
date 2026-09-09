"""Localization confidence must not be inferred from covariance alone."""
import unittest
import math
from pathlib import Path
import numpy as np

from rosy_control.sensing.localization import MapAgreement, Confidence, lease_ready, planar_yaw


class LocalizationTests(unittest.TestCase):
    def test_recorded_gazebo_corner_is_not_lost_between_coarse_candidates(self):
        d = np.load(Path(__file__).parent / 'fixtures/gazebo_localization_corner.npz')
        m = MapAgreement(d['grid'], float(d['resolution']), d['origin'])
        result = m.global_match(d['ranges'], d['angles'], .105)
        self.assertTrue(result['unique'], result)
        self.assertLess(math.dist(result['pose'][:2], d['truth'][:2]), .02)
        self.assertTrue(m.footprint_clear(*result['pose'][:2], .105))
    def test_invalid_or_tilted_rotation_cannot_authorize_planar_pose(self):
        self.assertAlmostEqual(planar_yaw(0., 0., math.sin(.2), math.cos(.2)), .4)
        for values in ((0., 0., 0., 0.), (0., 0., 0., 2.), (math.nan, 0., 0., 1.),
                       (1., 0., 0., 0.)):
            with self.assertRaises(ValueError):
                planar_yaw(*values)

    def test_footprint_clearance_checks_edges_and_unknown(self):
        grid = np.zeros((50, 50), dtype=np.int8)
        grid[:, 40] = 100
        m = MapAgreement(grid, .02, (0., 0.))
        self.assertTrue(m.footprint_clear(.5, .5, .1))
        self.assertFalse(m.footprint_clear(.75, .5, .1))
        self.assertFalse(m.footprint_clear(.01, .5, .1))
        grid[25, 25] = -1
        self.assertFalse(MapAgreement(grid, .02, (0., 0.)).footprint_clear(.5, .5, .1))
    def test_lease_is_stamped_and_expires(self):
        self.assertTrue(lease_ready({'ready': True, 'stamp_ns': 1000000000}, 1.2))
        for value in (None, {}, {'ready': True}, {'ready': 'true', 'stamp_ns': 1000000000},
                      {'ready': True, 'stamp_ns': 3000000000}):
            self.assertFalse(lease_ready(value, 1.2))
        self.assertFalse(lease_ready({'ready': True, 'stamp_ns': 1000000000}, 2.))

    def test_confidence_requires_distinct_good_scans_and_revokes_immediately(self):
        c = Confidence(stable_scans=3)
        for t in (1., 1., 1.1):
            self.assertFalse(c.observe(t, True))
        self.assertTrue(c.observe(1.2, True))
        self.assertFalse(c.observe(1.3, False))
        self.assertFalse(c.observe(1.4, True))
        self.assertFalse(c.observe(.1, True))  # Time rewind cannot renew a lease.

    def test_scan_fit_uses_sensor_pose_and_rejects_unknown_and_wrong_pose(self):
        grid = np.zeros((50, 50), dtype=np.int8)
        grid[:, 40] = 100
        m = MapAgreement(grid, .02, (0., 0.))
        angles = np.linspace(-.4, .4, 60)
        ranges = .4 / np.cos(angles)
        self.assertGreater(m.score((.4, .5, 0.), ranges, angles), .95)
        self.assertLess(m.score((.2, .5, 0.), ranges, angles), .1)
        self.assertLess(m.score((.4, .5, 3.14), ranges, angles), .1)
        self.assertEqual(m.score((.4, .5, 0.), [], []), 0.)
        grid[:, 40] = -1
        self.assertEqual(MapAgreement(grid, .02, (0., 0.)).score((.4, .5, 0.), ranges, angles), 0.)

    def test_endpoints_behind_a_wall_are_not_a_valid_match(self):
        grid = np.zeros((50, 50), dtype=np.int8)
        grid[:, 30] = grid[:, 40] = 100
        m = MapAgreement(grid, .02, (0., 0.))
        angles = np.linspace(-.4, .4, 60)
        self.assertLess(m.score((.4, .5, 0.), .4 / np.cos(angles), angles), .1)

    def test_global_match_rejects_rotationally_symmetric_room(self):
        grid = np.zeros((51, 51), dtype=np.int8)
        grid[[0, -1], :] = 100
        grid[:, [0, -1]] = 100
        angles = np.linspace(-math.pi, math.pi, 72, endpoint=False)
        ranges = .5 / np.maximum(np.abs(np.cos(angles)), np.abs(np.sin(angles)))
        result = MapAgreement(grid, .02, (0., 0.)).global_match(ranges, angles, .08)
        self.assertFalse(result['unique'])

    def test_global_match_finds_unknown_pose_in_asymmetric_map(self):
        grid = np.zeros((80, 100), dtype=np.int8)
        grid[[0, -1], :] = 100
        grid[:, [0, -1]] = 100
        grid[40:43, 40:90] = 100
        grid[12:20, 12:17] = 100
        angles = np.linspace(-math.pi, math.pi, 180, endpoint=False)
        truth = (.7, .5, .37)
        ranges = []
        for a in angles + truth[2]:
            for r in np.arange(.01, 3., .005):
                x, y = int((truth[0]+r*math.cos(a))/.02), int((truth[1]+r*math.sin(a))/.02)
                if not (0 <= x < 100 and 0 <= y < 80) or grid[y, x] >= 65:
                    ranges.append(r)
                    break
        result = MapAgreement(grid, .02, (0., 0.)).global_match(ranges, angles, .08)
        self.assertTrue(result['unique'], result)
        self.assertLess(math.dist(result['pose'][:2], truth[:2]), .03)
        self.assertLess(abs(math.atan2(math.sin(result['pose'][2]-.37), math.cos(result['pose'][2]-.37))), .03)
