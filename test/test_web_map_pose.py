import math
import unittest

from move_control.sensing.map_pose import display_pose, record_odom, update_pose


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

    def test_authoritative_base_tf_does_not_assume_odom_child_is_base(self):
        state = {}
        record_odom(state, 1, 0)
        display_pose(state, (2.03, 3, .2), (1, 3, 0))
        self.assertEqual(state['pose'], [2.03, 3, .2])
        self.assertEqual(state['trail'], [[2, 3]])
        self.assertEqual(state['pose_reason'], 'ready')

    def test_periodic_invalidation_needs_no_new_odom_sample(self):
        state = {}
        record_odom(state, 1, 0)
        display_pose(state, (1, 0, 0), (0, 0, 0))
        display_pose(state, (1, 0, 0), (0, 0, 0), odom_age=1.01)
        self.assertIsNone(state['pose'])
        self.assertEqual(state['pose_reason'], 'stale_odom')
        self.assertEqual(state['trail_odom'], [(1, 0)])

    def test_missing_stale_and_invalid_tf_cannot_leave_available_pose(self):
        cases = [(None, 0, 'missing_map_tf'),
                 ((1, 0, 0), 1.01, 'stale_map_tf'),
                 ((1, 0, 0), -1, 'stale_map_tf'),
                 ((math.nan, 0, 0), 0, 'invalid_map_tf')]
        for pose, age, reason in cases:
            state = {}
            display_pose(state, pose, (0, 0, 0), tf_age=age)
            self.assertFalse(state['pose_available'])
            self.assertIsNone(state['pose'])
            self.assertEqual(state['pose_reason'], reason)
