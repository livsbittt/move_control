import gzip
import json
import math
from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np

from rosy_control.sensing.scan_rotation import scan_rotation, percentile80


class ScanRotationVisibilityTests(unittest.TestCase):
    def test_dropout_diagnostic_identifies_failure_without_granting_evidence(self):
        reference,increment=self.capture()
        diagnostic={'old':True}
        self.assertIsNotNone(scan_rotation(reference,reference,increment,diagnostic))
        self.assertEqual(diagnostic['reason'],'ok')
        self.assertNotIn('old',diagnostic)
        sparse=reference.copy();sparse[:200]=math.inf
        self.assertIsNone(scan_rotation(reference,sparse,increment,diagnostic))
        self.assertEqual(diagnostic['reason'],'insufficient_observed_returns')
        self.assertLess(diagnostic['current_observed'],diagnostic['minimum_observed'])
        uniform=np.ones(720)
        self.assertIsNone(scan_rotation(uniform,uniform,2*math.pi/720,diagnostic))
        self.assertEqual(diagnostic['reason'],'ambiguous_alignment')
        self.assertEqual(diagnostic['separation_m'],0.)

    def test_partition_percentile_matches_numpy_linear_interpolation(self):
        rng = np.random.default_rng(9381)
        for size in (1, 2, 3, 5, 6, 10, 360, 671, 720, 1440):
            for values in (rng.random(size)*8, np.zeros(size), np.round(rng.random(size), 2)):
                self.assertAlmostEqual(percentile80(values), float(np.quantile(values, .8)), places=14)

    def test_replay_and_random_scan_decisions_match_generic_quantile(self):
        reference, increment = self.capture()
        rng = np.random.default_rng(185)
        cases = [(reference, np.roll(reference, shift), increment) for shift in (-20, 0, 12, 20)]
        for count in (360, 720, 1440):
            original = rng.uniform(.2, 3., count)
            cases.append((original, np.roll(original, 8)+rng.normal(0., .002, count), 2*math.pi/count))
            cases.append((original, np.ones(count), 2*math.pi/count))
        for first, second, step in cases:
            actual = scan_rotation(first, second, step)
            with patch('rosy_control.sensing.scan_rotation.percentile80', side_effect=lambda x: float(np.quantile(x, .8))):
                expected = scan_rotation(first, second, step)
            if expected is None:
                self.assertIsNone(actual)
            else:
                self.assertIsNotNone(actual)
                for key in expected:
                    self.assertAlmostEqual(actual[key], expected[key], places=14)

    def capture(self):
        path = Path(__file__).parent / 'fixtures/rotation_scan_missing_returns_20260909.json.gz'
        with gzip.open(path, 'rt') as stream:
            frame = json.load(stream)
        return np.array([math.inf if v is None else v for v in frame['ranges']]), frame['increment']

    def test_real_missing_returns_still_allow_unique_self_alignment(self):
        reference, increment = self.capture()
        result = scan_rotation(reference, reference, increment)
        self.assertIsNotNone(result)
        self.assertEqual(result['yaw'], 0.)

    def test_real_scan_with_known_rotation_and_missing_returns(self):
        reference, increment = self.capture()
        for shift in (-20, 12, 20):
            result = scan_rotation(reference, np.roll(reference, -shift), increment)
            self.assertIsNotNone(result)
            self.assertAlmostEqual(result['yaw'], shift*increment)

    def test_missing_returns_cannot_make_a_uniform_scene_observable(self):
        reference, increment = self.capture()
        uniform = np.where(np.isfinite(reference), 1., math.inf)
        self.assertIsNone(scan_rotation(uniform, uniform, increment))

    def test_sparse_scan_or_insufficient_common_observed_support_is_rejected(self):
        reference, increment = self.capture()
        sparse = reference.copy()
        sparse[:200] = math.inf
        self.assertIsNone(scan_rotation(sparse, sparse, increment))
        complete = np.linspace(.3, 2., 720)
        first, second = complete.copy(), complete.copy()
        first[:71] = math.inf
        second[300:371] = math.inf
        self.assertIsNone(scan_rotation(first, second, increment))
