"""Planner route revocation tests in an isolated local ROS domain."""
import time
import json
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
        msg.info.resolution = .05
        msg.info.width = msg.info.height = 20
        msg.data = [0] * 400
        self.node.on_map(msg)

    def test_default_stop_and_explicit_stop_revoke_route_immediately(self):
        self.assertEqual(self.node.mode, 'stop')
        self.node.plan()
        self.assert_empty_route()
        self.node.on_cmd(String(data='explore'))
        self.node.on_cmd(String(data='stop'))
        self.assert_empty_route()
        self.node.stop()
        self.assert_empty_route()

    def test_environment_profile_applies_only_when_ready_fresh_and_same_map_resolution(self):
        from move_control.control.navigation_calibration import environment_profile
        p = environment_profile(.076, .05, [(.14,.13,None,None)]*30)
        self.node.on_calibration_profile(String(data=json.dumps({'ready':True,'navigation_profile':p})))
        self.known_map()
        self.node.pose = Mock(return_value=((.5,.5),'tf'))
        self.node.brain.plan = Mock(return_value=(None,None,'test'))
        self.node.mode = 'explore'
        self.node.plan()
        self.assertAlmostEqual(self.node.brain.clear_m,p['preferred_clearance_m'])
        self.assertAlmostEqual(self.node.brain.start_escape_clear_m,p['minimum_clearance_m'])
        self.node.navigation_profile_received -= 10
        self.node.plan()
        self.assertNotEqual(self.node.brain.start_escape_clear_m,p['minimum_clearance_m'])
        self.node.on_calibration_profile(String(data=json.dumps({'ready':False,'navigation_profile':p})))
        self.assertIsNone(self.node.navigation_profile)

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
