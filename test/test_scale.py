#!/usr/bin/env python3
import unittest

from rosy_control.safety.scale import (
    corridor_width,
    fit_map,
    fit_open_max,
    narrow_clearance,
)


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

    def test_narrow_clearance_is_width_minus_machine(self):
        # 24 cm corridor, 15.2 cm machine → 8.8 cm total clearance
        self.assertAlmostEqual(narrow_clearance(0.24, 0.076), 0.088, places=3)
        self.assertAlmostEqual(narrow_clearance(0.30, 0.076), 0.148, places=3)
        # corridor narrower than the machine clamps to 0.0 (speed floor,
        # escape is the FSM's job) — never negative.
        self.assertEqual(narrow_clearance(0.12, 0.076), 0.0)
        self.assertIsNone(narrow_clearance(None, 0.076))
        self.assertIsNone(narrow_clearance(-0.24, 0.076))
        self.assertIsNone(narrow_clearance(float('nan'), 0.076))


if __name__ == '__main__':
    unittest.main()
