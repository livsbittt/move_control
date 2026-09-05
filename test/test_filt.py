#!/usr/bin/env python3
import unittest

from move_control.sensing.filt import MedianLp


class FiltTest(unittest.TestCase):
    def test_holds_then_drops_stale(self):
        lp = MedianLp(median_n=3, fc_hz=10.0, dt=0.05, miss_n=3)
        self.assertAlmostEqual(lp.push(0.20), 0.20, places=3)
        self.assertAlmostEqual(lp.push(None), 0.20, places=3)
        self.assertAlmostEqual(lp.push(None), 0.20, places=3)
        self.assertIsNone(lp.push(None))
        self.assertIsNone(lp.value())

    def test_new_hit_after_drop(self):
        lp = MedianLp(median_n=3, fc_hz=10.0, dt=0.05, miss_n=2)
        lp.push(0.20)
        lp.push(None)
        self.assertIsNone(lp.push(None))
        y = lp.push(0.08)
        self.assertIsNotNone(y)
        self.assertAlmostEqual(y, 0.08, places=3)


if __name__ == '__main__':
    unittest.main()
