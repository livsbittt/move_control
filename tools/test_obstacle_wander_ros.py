import json
import unittest
from unittest.mock import Mock
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from rosy_control.wander.node import WanderNode


class ObstacleWanderTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(domain_id=215)

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_static_replan_turns_without_translation_but_moving_object_waits(self):
        node = WanderNode()
        try:
            node.pub = Mock()
            node.navigation_mode = 'explore'
            node.navigation_route = [(0.,0.),(.5,.3)]
            node._ir_ready = Mock(return_value=True)
            node._on_wall = Mock(return_value=False)
            node._can_reverse = Mock(return_value=False)
            node.blocked = node.estop = node.pickup = node.cliff = node.tilt = False
            tf = TransformStamped()
            tf.header.stamp = node.now().to_msg()
            tf.transform.rotation.w = 1.
            node.navigation_tf = Mock()
            node.navigation_tf.lookup_transform.return_value = tf
            node.navigation_received = node.navigation_stamp = node.now().nanoseconds*1e-9
            node.path_follower.update = Mock(return_value=(.014,.1,'forward'))
            node._obstacle_wait = Mock(return_value='replan:predicted_obstacle')
            node._tick_navigation()
            cmd = node.pub.publish.call_args.args[0]
            self.assertEqual(cmd.linear.x, 0.)
            self.assertGreater(cmd.angular.z, 0.)
            node._obstacle_wait.return_value = 'wait:predicted_obstacle'
            node._tick_navigation()
            cmd = node.pub.publish.call_args.args[0]
            self.assertEqual((cmd.linear.x,cmd.angular.z), (0.,0.))
        finally:
            node.destroy_node()

    def test_obstacle_wait_uses_intended_speed_without_erasing_progress_budget(self):
        node = WanderNode()
        try:
            node.set_parameters([Parameter('obstacle_tracking_enabled', value=True)])
            node._odom_fresh = Mock(return_value=True)
            node.odom_x = node.odom_y = node.odom_yaw = 0.
            now = node.now().nanoseconds*1e-9
            future = String(data=json.dumps(dict(stamp=now+1000., frame='odom', tracks=[], blocked=True)))
            node._on_obstacle_camera(future)
            node._on_obstacle_tracks(future)
            self.assertIsNone(node.navigation_camera)
            self.assertIsNone(node.navigation_obstacles)
            node._on_obstacle_camera(String(data=json.dumps(dict(stamp=now, blocked=False))))
            node._on_obstacle_tracks(String(data=json.dumps(dict(stamp=now, frame='odom', tracks=[
                dict(id=1, position=[.2,0.], velocity=[-.1,0.], radius=.03, state='moving', observed=True, age=0.)]))))
            self.assertIsNotNone(node._obstacle_wait(.014,0.))
            node.navigation_obstacles['tracks'][0].update(position=[.1,0.], velocity=[0.,0.], state='stationary')
            self.assertTrue(node._obstacle_wait(0.,0.).startswith('replan:'))
            self.assertIsNone(node._obstacle_wait(0.,.1))
            guard = node.navigation_progress
            guard.check(0.,(0.,0.,0.),True)
            guard.pause(3.,True)
            guard.pause(103.,False)
            self.assertFalse(guard.check(107.,(0.,0.,0.),True))
            self.assertTrue(guard.check(109.,(0.,0.,0.),True))
        finally:
            node.destroy_node()
