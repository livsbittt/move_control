import unittest

from move_control.control.pursuit import pursuit_index, pursuit_speed


class PursuitTest(unittest.TestCase):
    def test_aligns_before_driving_across_a_corner(self):
        self.assertEqual(pursuit_speed(0.18, 0.15, -0.65), 0.0)
        self.assertGreater(pursuit_speed(0.18, 0.15, 0.1), 0.0)

    def test_stationary_robot_does_not_skip_corner_on_later_ticks(self):
        route = [(0, 0), (0.2, 0), (0.2, 0.2), (0.2, 0.4)]
        for _ in range(30):
            self.assertEqual(pursuit_index(route, 0, 0), 1)

    def test_advances_after_robot_reaches_corner(self):
        route = [(0, 0), (0.2, 0), (0.2, 0.2), (0.2, 0.4)]
        self.assertEqual(pursuit_index(route, 0.2, 0.19), 3)

    def test_nearest_corner_is_not_yet_reached(self):
        route = [(0, 0), (0.2, 0), (0.2, 0.2), (0.2, 0.4)]
        self.assertEqual(pursuit_index(route, 0.11, 0), 1)
