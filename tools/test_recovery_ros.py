"""Execution failure reaches planning without publishing physical motor commands."""
import unittest
from unittest.mock import Mock, patch
import rclpy
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from move_control.goal_node import GoalNode
from move_control.wander.node import WanderNode
from move_control.planning import OccupancyMap


class RecoveryIntegrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):rclpy.init(domain_id=223)
    @classmethod
    def tearDownClass(cls):rclpy.shutdown()

    def test_executor_feedback_expires_and_restores_planner_watchdog(self):
        node = GoalNode()
        try:
            node.mode = 'explore'
            node.map_obj = OccupancyMap(50, 50, .02, fill=0)
            node._map_received = 100.
            node.pose = Mock(return_value=((.5, .5), 'tf'))
            node.brain.plan = Mock(return_value=(None, None, 'idle'))
            node._pub_status = node._pub_options = node._clear_route = Mock()
            with patch('move_control.goal_node.time.monotonic', return_value=100.) as clock:
                node.on_navigation_feedback(String(data='route_explore:align'))
                node.plan()
                self.assertTrue(node.brain.execution_feedback)
                clock.return_value = 102.
                node.plan()
                self.assertFalse(node.brain.execution_feedback)
        finally:
            node.destroy_node()

    def test_planner_excludes_failed_target_and_replans_but_never_starts_from_stop(self):
        node=GoalNode()
        try:
            node.plan=Mock()
            node.last_executable_goal=(1.,2.)
            node.last_executable_exit=(.06,0.)
            node.on_cmd(String(data='replan'))
            node.plan.assert_not_called()
            node.mode='explore'
            node.on_cmd(String(data='replan'))
            node.plan.assert_called_once()
            self.assertEqual(node.brain._failed_goals[0][0],(1.,2.))
            self.assertEqual(node.brain._failed_exits[0][0],(.06,0.))
            node.on_cmd(String(data='replan'))
            self.assertEqual(len(node.brain._failed_exits),1)
        finally:node.destroy_node()

    def test_stationary_camera_hold_requests_replan_and_only_new_goal_unlocks(self):
        node=WanderNode()
        try:
            node.pub=Mock()
            node.navigation_goal_pub=Mock()
            node.navigation_mode='explore'
            node.navigation_route=[(0.,0.),(1.,0.)]
            node._ir_ready=Mock(return_value=True)
            node._on_wall=Mock(return_value=False)
            node._can_reverse=Mock(return_value=True)
            node.cam_block=True
            node.estop=node.pickup=node.tilt=node.cliff=False
            node.navigation_tf=Mock()
            def tick(t,goal):
                node.now=Mock(return_value=Time(seconds=t))
                transform=TransformStamped()
                transform.header.stamp=Time(seconds=t).to_msg()
                transform.transform.rotation.w=1.
                node.navigation_tf.lookup_transform.return_value=transform
                node.navigation_route=[(0.,0.),goal]
                node.navigation_received=node.navigation_stamp=t
                node._tick_navigation()
            tick(100,(1.,0.))
            tick(106,(1.,0.))
            self.assertEqual(node.navigation_goal_pub.publish.call_args.args[0].data,'replan')
            tick(107,(1.,0.))
            self.assertTrue(node.navigation_recovery.waiting)
            tick(108,(0.,1.))
            self.assertFalse(node.navigation_recovery.waiting)
            self.assertEqual(node.pub.publish.call_args.args[0].linear.x,0.)
        finally:node.destroy_node()
