#!/usr/bin/env python3
import math
import unittest

from move_control.recover import (
    ESCAPE_MIN_TURN,
    backup_limit_m,
    escape_may_abort,
    escape_may_desense,
    gate_keep_angular,
    have_turn_space,
    is_stuck_motion,
    need_space_to_turn,
    stuck_flip,
    stuck_kind,
)


class RecoverTest(unittest.TestCase):
    def test_stuck_forces_backup_or_escape_not_forward(self):
        self.assertEqual(stuck_kind(True), 'backup')
        self.assertEqual(stuck_kind(False), 'escape')
        self.assertNotEqual(stuck_kind(True), 'forward')
        self.assertNotEqual(stuck_kind(False), 'look')

    def test_stuck_flips_sign_on_second(self):
        self.assertFalse(stuck_flip(1))
        self.assertTrue(stuck_flip(2))
        self.assertTrue(stuck_flip(3))

    def test_backup_uses_full_rear_gap(self):
        # Old 0.45 * (0.12-0.018) = 4.6cm. Now the whole gap, capped.
        self.assertAlmostEqual(backup_limit_m(0.12, stop=0.018, cap=0.12), 0.102, places=3)
        self.assertAlmostEqual(backup_limit_m(0.05, stop=0.018, cap=0.12), 0.032, places=3)
        self.assertAlmostEqual(backup_limit_m(0.40, stop=0.018, cap=0.12), 0.12, places=3)
        self.assertAlmostEqual(backup_limit_m(float('inf'), cap=0.12), 0.02, places=3)

    def test_turn_space_includes_tail(self):
        clear = 0.086
        self.assertTrue(have_turn_space(0.20, 0.20, 0.20, clear))
        self.assertFalse(have_turn_space(0.20, 0.20, 0.04, clear))
        self.assertTrue(
            need_space_to_turn(0.20, 0.20, 0.04, clear, True, 0, 2)
        )
        self.assertFalse(
            need_space_to_turn(0.20, 0.20, 0.20, clear, True, 0, 2)
        )
        self.assertFalse(
            need_space_to_turn(0.04, 0.04, 0.04, clear, False, 0, 2)
        )
        self.assertFalse(
            need_space_to_turn(0.04, 0.04, 0.04, clear, True, 2, 2)
        )

    def test_crawl_is_not_stuck(self):
        stuck_m, stuck_sec = 0.008, 1.2
        # think 3 mm/s cannot cover 8 mm in 1.2 s
        self.assertFalse(is_stuck_motion(0.0036, 1.2, 0.003, stuck_m, stuck_sec))
        # cruise commanded, chassis did not move
        self.assertTrue(is_stuck_motion(0.001, 1.2, 0.014, stuck_m, stuck_sec))
        # actually moved
        self.assertFalse(is_stuck_motion(0.010, 1.2, 0.014, stuck_m, stuck_sec))
        # window not elapsed
        self.assertFalse(is_stuck_motion(0.0, 0.5, 0.014, stuck_m, stuck_sec))

    def test_escape_abort_needs_45deg_and_not_on_wall(self):
        self.assertFalse(escape_may_abort(math.radians(13.0), False, False))
        self.assertTrue(escape_may_abort(ESCAPE_MIN_TURN, False, False))
        self.assertFalse(escape_may_abort(ESCAPE_MIN_TURN, True, False))
        self.assertFalse(escape_may_abort(ESCAPE_MIN_TURN, False, True))
        self.assertFalse(
            escape_may_abort(ESCAPE_MIN_TURN, False, False, pinched=True)
        )

    def test_timeout_does_not_desense_stuck(self):
        # 5s / 13° used to call _escape_false. Must not, especially when stuck.
        self.assertFalse(escape_may_desense(5.0, math.radians(13.0), True))
        self.assertFalse(escape_may_desense(5.0, math.radians(13.0), False))
        self.assertTrue(escape_may_desense(0.50, math.radians(5.0), False))
        self.assertFalse(escape_may_desense(0.50, math.radians(5.0), True))

    def test_think_rate_cannot_pass_15deg_in_5s(self):
        w_think = 0.045
        self.assertLess(w_think * 5.0, math.radians(15.0))
        wturn = 0.10
        self.assertGreater(wturn * 8.0, ESCAPE_MIN_TURN)

    def test_cliff_tilt_keep_spin(self):
        self.assertEqual(gate_keep_angular(0.10, vx=0.0, cliff=True, tilt=False), 0.10)
        self.assertEqual(gate_keep_angular(-0.10, vx=0.0, cliff=False, tilt=True), -0.10)


if __name__ == '__main__':
    unittest.main()
