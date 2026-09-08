"""Execution failure reaches planning without publishing physical motor commands."""
import unittest
from unittest.mock import Mock
import rclpy
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from move_control.goal_node import GoalNode
from move_control.wander.node import WanderNode


class RecoveryIntegrationTest(unittest.TestCase):
    def test_blocked_turn_uses_bounded_straight_sensor_suggestion(self):
        node=WanderNode()
        try:
            node.pub=Mock(); node.navigation_goal_pub=Mock()
            node.navigation_mode='explore'
            node.navigation_route=[(0.,0.),(0.,1.)]
            node._ir_ready=Mock(return_value=True)
            node._on_wall=Mock(return_value=False)
            node._can_reverse=Mock(return_value=True)
            node._odom_fresh=Mock(return_value=True)
            node._motion_limits_fresh=Mock(return_value=True)
            node.blocked=node.estop=node.pickup=node.tilt=node.cliff=False
            node.motion_limits={'can_rotate':False,'rotation_recovery_m':-.02,
                'rotation_scan_observed':True,'rotation_pivot_clearance_m':.009,
                'rotation_translation_limits_m':[.03,.03]}
            node.navigation_tf=Mock()
            def tick(t,x):
                node.now=Mock(return_value=Time(seconds=t))
                transform=TransformStamped()
                transform.header.stamp=Time(seconds=t).to_msg()
                transform.transform.rotation.w=1.
                transform.transform.translation.x=x
                node.odom_x=x; node.odom_y=node.odom_yaw=0.
                node.navigation_tf.lookup_transform.return_value=transform
                node.navigation_received=node.navigation_stamp=t
                node._tick_navigation()
                return node.pub.publish.call_args.args[0]
            out=tick(100,0.)
            self.assertEqual((out.linear.x,out.angular.z),(-.006,0.))
            node.motion_limits.update(can_rotate=True,rotation_recovery_m=None,
                                      rotation_pivot_clearance_m=.011)
            self.assertEqual(tick(101,-.005).linear.x,-.006)
            node.motion_limits['rotation_pivot_clearance_m']=.014
            self.assertEqual(tick(102,-.015).linear.x,0.)
            self.assertGreater(tick(103,-.015).angular.z,0.)
            node.motion_limits.update(can_rotate=False,rotation_recovery_m=-.02)
            tick(104,-.015)
            self.assertFalse(node.rotation_relocation.active)
            node.rotation_relocation.reset()
            self.assertEqual(tick(200,0.).linear.x,-.006)
            node.motion_limits.update(can_rotate=True,rotation_recovery_m=None,
                                      rotation_pivot_clearance_m=.011,
                                      rotation_translation_limits_m=[.03,.001])
            self.assertEqual(tick(201,-.005).linear.x,0.)
            self.assertFalse(node.rotation_relocation.active)
        finally:node.destroy_node()

    @classmethod
    def setUpClass(cls):rclpy.init(domain_id=223)
    @classmethod
    def tearDownClass(cls):rclpy.shutdown()

    def test_planner_excludes_failed_target_and_replans_but_never_starts_from_stop(self):
        node=GoalNode()
        try:
            node.plan=Mock()
            node.last_executable_goal=(1.,2.)
            node.last_executable_exit=(.06, 0.)
            node.on_cmd(String(data='replan'))
            node.plan.assert_not_called()
            node.mode='explore'
            node.on_cmd(String(data='replan'))
            node.plan.assert_called_once()
            self.assertEqual(node.brain._failed_goals[0][0],(1.,2.))
            self.assertEqual(node.brain._failed_exits[0][0],(.06,0.))
        finally:node.destroy_node()

    def test_stationary_range_hold_requests_replan_and_only_new_goal_unlocks(self):
        node=WanderNode()
        try:
            node.pub=Mock()
            node.navigation_goal_pub=Mock()
            node.navigation_mode='explore'
            node.navigation_route=[(0.,0.),(1.,0.)]
            node._ir_ready=Mock(return_value=True)
            node._on_wall=Mock(return_value=False)
            node._can_reverse=Mock(return_value=True)
            node.blocked=True
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
