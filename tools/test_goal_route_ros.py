"""Planner route revocation tests in an isolated local ROS domain."""
import time
import unittest
from unittest.mock import Mock

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from move_control.goal_node import GoalNode, grid_clearance


class GoalRouteTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(domain_id=218)

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = GoalNode()
        self.node.route_pub = Mock()
        self.node.tf = Mock()
        self.node.tf.lookup_transform.side_effect = RuntimeError('no transform')

    def tearDown(self):
        self.node.destroy_node()

    def assert_empty_route(self):
        self.assertEqual(self.node.route_pub.publish.call_args.args[0].poses, [])

    def known_map(self):
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.info.resolution = .05
        msg.info.width = msg.info.height = 20
        msg.data = [0] * 400
        self.node.on_map(msg)
        return msg

    def test_replayed_map_does_not_renew_source_deadline(self):
        msg = self.known_map()
        received = self.node._map_received
        self.node.on_map(msg)
        self.assertEqual(self.node._map_received, received)
        msg.header.stamp.sec -= 30
        self.node.on_map(msg)
        self.assertIsNone(self.node.map_obj)
        self.assert_empty_route()

    def test_fresh_invalid_tf_quaternion_cannot_authorize_route(self):
        transform = TransformStamped()
        transform.transform.rotation.w = 0.
        transform.header.stamp = self.node.get_clock().now().to_msg()
        self.node.tf.lookup_transform.side_effect = None
        self.node.tf.lookup_transform.return_value = transform
        self.assertEqual(self.node.pose(), ((None, None), 'invalid-tf'))

    def test_default_stop_and_explicit_stop_revoke_route_immediately(self):
        self.assertEqual(self.node.mode, 'stop')
        self.node.plan()
        self.assert_empty_route()
        self.node.on_cmd(String(data='explore'))
        self.node.on_cmd(String(data='stop'))
        self.assert_empty_route()
        self.node.stop()
        self.assert_empty_route()

    def test_missing_map_and_failed_tf_never_reuse_odom_coordinates(self):
        self.node.on_cmd(String(data='explore'))
        self.node.plan()
        self.assert_empty_route()
        self.known_map()
        self.node.have_odom = True
        self.node.ox = self.node.oy = .5
        self.node.plan()
        self.assert_empty_route()
        self.assertEqual(self.node.pose(), ((None, None), 'none'))

    def test_stale_tf_and_stale_map_revoke_route(self):
        self.node.on_cmd(String(data='coverage'))
        self.known_map()
        transform = TransformStamped()
        transform.transform.translation.x = transform.transform.translation.y = .5
        self.node.tf.lookup_transform.side_effect = None
        self.node.tf.lookup_transform.return_value = transform
        self.node.plan()
        self.assert_empty_route()
        self.assertEqual(self.node.pose()[1], 'stale-tf')
        transform.header.stamp = self.node.get_clock().now().to_msg()
        self.node._map_received = time.monotonic() - 100
        self.node.plan()
        self.assert_empty_route()

    def test_no_plan_clears_previous_route_and_reset_clears_session(self):
        self.node.on_cmd(String(data='coverage'))
        self.known_map()
        self.node.pose = lambda: ((.5, .5), 'tf')
        self.node.brain.plan = Mock(return_value=(None, None, 'coverage waiting'))
        self.node.plan()
        self.assert_empty_route()
        self.assertGreaterEqual(self.node.brain.clear_m, .12)
        self.assertGreaterEqual(self.node.brain.retry_clear_m, self.node.brain.clear_m)
        self.node.brain.covered.add((2, 2))
        self.node.on_cmd(String(data='reset'))
        self.assert_empty_route()
        self.assertEqual(self.node.brain.covered, set())
        self.assertIsNone(self.node.map_obj)

    def test_float32_grid_does_not_add_an_unnecessary_cell(self):
        self.assertAlmostEqual(grid_clearance(.12, .019999999552965164), .12)
        self.assertGreaterEqual(grid_clearance(.121, .019999999552965164), .139)
