#!/usr/bin/env python3
import unittest

from move_control.safety.scale import corridor_width, fit_map, fit_open_max


class ScaleTest(unittest.TestCase):
    def test_corridor_maze(self):
        w = corridor_width(0.13, 0.16)
        self.assertAlmostEqual(w, 0.29, places=3)

    def test_corridor_rejects_room(self):
        self.assertIsNone(corridor_width(1.2, 1.5))
        self.assertIsNone(corridor_width(0.02, 0.03))
        self.assertIsNone(corridor_width(float('inf'), 0.16))

    def test_map_narrower_than_50cm(self):
        m = fit_map(0.29)
        self.assertLessEqual(m, 0.40)
        self.assertGreaterEqual(m, 0.18)
        self.assertAlmostEqual(m, 0.29 * 1.35, places=3)

    def test_open_max_not_120cm(self):
        o = fit_open_max(0.29)
        self.assertLess(o, 0.55)
        self.assertGreater(o, 0.20)

    def test_clamps(self):
        self.assertEqual(fit_map(0.05, lo=0.18, hi=0.40), 0.18)
        self.assertEqual(fit_map(2.0, lo=0.18, hi=0.40), 0.40)


if __name__ == '__main__':
    unittest.main()
