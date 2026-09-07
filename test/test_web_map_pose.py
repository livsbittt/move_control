import math
import unittest

from move_control.sensing.map_pose import update_pose


class MapPoseTest(unittest.TestCase):
    def test_rotation_translation_and_heading(self):
        state = {}
        update_pose(state, 1, 0, math.pi, (2, 3, math.pi / 2))
        self.assertEqual(state['pose'][:2], [2, 4])
        self.assertAlmostEqual(state['pose'][2], -math.pi / 2, places=3)

    def test_missing_transform_hides_overlays_but_keeps_distance(self):
        state = {}
        update_pose(state, 0, 0, 0, (0, 0, 0))
        update_pose(state, .1, 0, 0, None)
        self.assertIsNone(state['pose'])
        self.assertEqual(state['trail'], [])
        self.assertFalse(state['pose_available'])
        self.assertEqual(state['path_m'], .1)

    def test_slow_motion_does_not_round_each_increment_to_zero(self):
        state = {}
        for i in range(101):
            update_pose(state, i * .0007, 0, 0, (0, 0, 0))
        self.assertEqual(state['path_m'], .07)

    def test_slam_correction_moves_trail_without_inflating_distance(self):
        state = {}
        update_pose(state, 0, 0, 0, (0, 0, 0))
        update_pose(state, .1, 0, 0, (2, 0, 0))
        self.assertEqual(state['trail'], [[2, 0], [2.1, 0]])
        self.assertEqual(state['path_m'], .1)
