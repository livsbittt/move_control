import unittest

from rosy_control.control.recovery_budget import RecoveryBudget


class RecoveryBudgetTest(unittest.TestCase):
    def test_repeated_contact_in_same_pocket_stops(self):
        b = RecoveryBudget()
        self.assertTrue(b.attempt(0, 0))
        self.assertTrue(b.attempt(.02, 0))
        self.assertTrue(b.attempt(0, .02))
        self.assertFalse(b.attempt(0, 0))

    def test_back_and_forth_distance_does_not_erase_failures(self):
        b = RecoveryBudget()
        for x in (0, .03, -.03):
            self.assertTrue(b.attempt(x, 0))
        self.assertFalse(b.attempt(.03, 0))

    def test_real_displacement_allows_a_new_pocket(self):
        b = RecoveryBudget()
        for _ in range(3):
            b.attempt(0, 0)
        self.assertTrue(b.attempt(.20, 0))
        self.assertEqual(b.count, 1)

    def test_unknown_pose_cannot_reset_budget(self):
        b = RecoveryBudget()
        for _ in range(3):
            b.attempt(0, 0)
        self.assertFalse(b.attempt(float('nan'), 0))
