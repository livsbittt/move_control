import math
import unittest

from move_control.control.path_follow import ProgressGuard, follow_path


class PathFollowTest(unittest.TestCase):
    def command(self, route=None, pose=(0, 0, 0), **kwargs):
        args = dict(route_age=0.1, tf_age=0.1)
        args.update(kwargs)
        return follow_path([(0, 0), (.3, 0)] if route is None else route, pose, **args)

    def test_valid_route_forward_respects_hardware_speed(self):
        v, w, reason = self.command()
        self.assertEqual(reason, 'forward')
        self.assertGreater(v, 0)
        self.assertLessEqual(v, .014)
        self.assertEqual(w, 0)

    def test_heading_alignment_never_advances(self):
        v, w, reason = self.command(pose=(0, 0, math.pi / 2))
        self.assertEqual((v, reason), (0, 'align'))
        self.assertLess(w, 0)
        self.assertGreaterEqual(w, -.10)

    def test_missing_empty_stale_and_future_inputs_stop(self):
        cases = [dict(route=[]), dict(pose=None), dict(route_age=6),
                 dict(tf_age=2), dict(route_age=-1), dict(tf_age=-1),
                 dict(route=[(math.nan, 0)]), dict(pose=(math.inf, 0, 0))]
        for values in cases:
            with self.subTest(values=values):
                self.assertEqual(self.command(**values)[:2], (0, 0))

    def test_hazard_always_stops_even_turn(self):
        self.assertEqual(self.command(pose=(0, 0, 2), blocked=True), (0, 0, 'hazard'))

    def test_arrival_stops(self):
        self.assertEqual(self.command(pose=(.29, 0, 0)), (0, 0, 'arrived'))

    def test_localization_jump_cannot_drive_across_to_distant_route(self):
        self.assertEqual(self.command(pose=(.1, .081, 0)), (0, 0, 'off_route'))
        self.assertGreater(self.command(pose=(.15, 0, 0))[0], 0)

    def test_brief_empty_route_cannot_replenish_stall_budget(self):
        guard = ProgressGuard(timeout=8)
        self.assertFalse(guard.check(0, (0, 0, 0), True))
        self.assertFalse(guard.check(7, (0, 0, 0), False))
        self.assertTrue(guard.check(8, (0, 0, 0), True))

    def test_corner_target_does_not_cut_diagonal(self):
        route = [(0, 0), (.1, 0), (.1, .1)]
        v, w, _ = self.command(route=route, pose=(.04, 0, 0), lookahead=.20)
        self.assertGreater(v, 0)
        self.assertEqual(w, 0)

    def test_stall_latches_across_new_route_and_requires_reset(self):
        guard = ProgressGuard(timeout=8)
        self.assertFalse(guard.check(0, (0, 0, 0), True))
        self.assertTrue(guard.check(8, (0, 0, 0), True))
        self.assertTrue(guard.check(9, (.3, 0, 0), False))
        guard.reset()
        self.assertFalse(guard.check(10, (.3, 0, 0), True))

    def test_slow_real_motion_and_turns_are_progress(self):
        guard = ProgressGuard(timeout=8)
        for i in range(30):
            self.assertFalse(guard.check(i, (i * .001, 0, 0), True))
        guard.reset()
        for i in range(30):
            self.assertFalse(guard.check(i, (0, 0, i * .02), True))
