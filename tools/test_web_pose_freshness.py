"""ROS-backed callback tests without creating nodes or commanding hardware."""
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from geometry_msgs.msg import TransformStamped
from rosy_control import web_node as web


class WebPoseFreshnessTest(unittest.TestCase):
    def setUp(self):
        web.STATE.clear()
        web.STATE['trail_odom'] = [(0, 0)]
        self.now = 100.0
        self.base = TransformStamped()
        self.base.header.stamp.sec = 100
        self.base.transform.translation.x = 2.03
        self.base.transform.rotation.w = 1.0
        self.odom = TransformStamped()
        self.odom.header.stamp.sec = 100
        self.odom.transform.translation.x = 2.0
        self.odom.transform.rotation.w = 1.0
        self.node = SimpleNamespace(
            map_frame='map', odom_frame='odom', odom_received=100.0, odom_stamp=100.0,
            tf=SimpleNamespace(lookup_transform=lambda target, source, stamp:
                               self.base if source == 'base_link' else self.odom),
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
                nanoseconds=int(self.now * 1e9))),
            get_parameter=lambda name: SimpleNamespace(value=1.0),
            _tf_pose=web.WebNode._tf_pose)

    def refresh(self):
        with patch.object(web.time, 'monotonic', return_value=self.now):
            web.WebNode.refresh_pose(self.node)

    def test_base_link_tf_is_authority_not_odom_child(self):
        self.refresh()
        self.assertEqual(web.STATE['pose'], [2.03, 0, 0])
        self.assertEqual(web.STATE['trail'], [[2, 0]])

    def test_timer_invalidates_without_another_odometry_callback(self):
        self.refresh()
        self.now = 101.1
        self.refresh()
        self.assertIsNone(web.STATE['pose'])
        self.assertEqual(web.STATE['pose_reason'], 'stale_odom')

    def test_fresh_odometry_does_not_hide_stale_tf(self):
        self.base.header.stamp.sec = 98
        self.refresh()
        self.assertEqual(web.STATE['pose_reason'], 'stale_map_tf')

    def test_future_tf_cannot_be_hidden_by_other_fresh_transform(self):
        self.base.header.stamp.sec = 102
        self.refresh()
        self.assertEqual(web.STATE['pose_reason'], 'stale_map_tf')

    def test_slam_transform_timeout_future_stamp_is_valid(self):
        self.odom.header.stamp.nanosec = 500000000
        lookup = Mock(wraps=self.node.tf.lookup_transform)
        self.node.tf.lookup_transform = lookup
        self.refresh()
        self.assertTrue(web.STATE['pose_available'])
        self.assertEqual(lookup.call_args_list[1].args[2].nanoseconds, 100000000000)

    def test_future_odometry_cannot_be_hidden_by_recent_receipt(self):
        self.node.odom_stamp = 102
        self.refresh()
        self.assertEqual(web.STATE['pose_reason'], 'stale_odom')

    def test_missing_tf_clears_previously_valid_pose(self):
        self.refresh()
        self.node.tf.lookup_transform = Mock(side_effect=web.TransformException('missing'))
        self.refresh()
        self.assertFalse(web.STATE['pose_available'])
        self.assertEqual(web.STATE['pose_reason'], 'missing_map_tf')

    def test_paused_reset_does_not_restore_previous_map_pose(self):
        web.STATE['map_control'] = {'paused': True}
        self.refresh()
        self.assertIsNone(web.STATE['pose'])
        self.assertEqual(web.STATE['pose_reason'], 'mapping_paused')


if __name__ == '__main__':
    unittest.main()
