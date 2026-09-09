import unittest
from rosy_control.control.navigation_session import NavigationSession, validate_options


class NavigationSessionTest(unittest.TestCase):
    def test_deadline_is_not_renewed_by_repeated_start_or_motion(self):
        session = NavigationSession()
        options = dict(strategy='nearest', duration_s=20, stall_s=10)
        self.assertTrue(session.start(options, 100))
        self.assertFalse(session.start(options, 105))
        session.tick(101, (0, 0), True)
        session.tick(109, (.03, 0), True)
        session.tick(118, (.06, 0), True)
        self.assertEqual(session.tick(120, (.1, 0)), 'time_limit')
        self.assertFalse(session.active)
        self.assertEqual(session.snapshot()['remaining_s'], 0)

    def test_missing_pose_and_tiny_oscillation_do_not_renew_stall(self):
        for pose in (None, (.005, 0)):
            session = NavigationSession()
            session.start(dict(strategy='gain', duration_s=60, stall_s=10), 0)
            session.tick(0, (0, 0))
            self.assertIsNone(session.tick(9, pose))
            self.assertEqual(session.tick(10, pose), 'no_translation')

    def test_clock_reversal_fails_closed_and_restart_is_explicit(self):
        session = NavigationSession()
        options = dict(strategy='coverage', duration_s=60, stall_s=10)
        session.start(options, 20)
        self.assertEqual(session.tick(19), 'clock_invalid')
        self.assertIsNone(session.tick(21))
        self.assertTrue(session.start(options, 22))

    def test_invalid_options(self):
        valid = dict(strategy='gain', duration_s=60, stall_s=10)
        for patch in ({'strategy':'astar'}, {'duration_s':True}, {'duration_s':float('nan')},
                      {'stall_s':61}, {'extra':1}, {'duration_s':3601}):
            with self.assertRaises(ValueError):
                validate_options({**valid, **patch})

    def test_offset_pivot_spin_cannot_renew_translation_deadline(self):
        import math
        session = NavigationSession()
        session.start(dict(strategy='gain', duration_s=60, stall_s=10), 0)
        for second in range(10):
            yaw = second*.1
            pose = (.041*(1-math.cos(yaw)), -.041*math.sin(yaw))
            self.assertIsNone(session.tick(second, pose, translating=False))
        self.assertEqual(session.tick(10, (.04, -.03), translating=False), 'no_translation')

    def test_short_translations_across_alignment_pauses_count_as_real_progress(self):
        session = NavigationSession()
        session.start(dict(strategy='gain', duration_s=30, stall_s=10), 0)
        session.tick(0, (0., 0.), True)
        session.tick(3, (.012, 0.), True)
        session.tick(4, (.014, .004), False)  # Pivot/settling motion is excluded.
        session.tick(6, (.015, .006), True)
        session.tick(8, (.025, .006), True)
        self.assertEqual(session.progress_at, 8)
        self.assertIsNone(session.tick(10, (.025, .006), False))
        self.assertEqual(session.tick(18, (.025, .006), False), 'no_translation')

    def test_short_forward_reverse_oscillation_does_not_renew_progress(self):
        session = NavigationSession()
        session.start(dict(strategy='gain', duration_s=30, stall_s=10), 0)
        for second, pose, moving in ((0, (0.,0.), True), (2,(.008,0.),True),
                (3,(.008,0.),False), (4,(.008,0.),True), (6,(0.,0.),True),
                (7,(0.,0.),False), (8,(0.,0.),True), (9,(.008,0.),True)):
            self.assertIsNone(session.tick(second, pose, moving))
        self.assertEqual(session.tick(10, (.008,0.), False), 'no_translation')
