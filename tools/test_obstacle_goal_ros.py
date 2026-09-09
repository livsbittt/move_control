import json
import time
import unittest
from unittest.mock import Mock

import rclpy
from rclpy.parameter import Parameter
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from tf2_ros import Buffer
from rosy_control.goal_node import GoalNode
from rosy_control.planning.gridmap import OccupancyMap


class ObstacleGoalTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(domain_id=214)

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_future_dated_slam_tf_is_queried_at_observation_time(self):
        node = GoalNode()
        try:
            node.set_parameters([Parameter('obstacle_tracking_enabled', value=True),
                                 Parameter('start_escape_clear_m', value=.05)])
            node.mode = 'explore'
            node.map_obj = OccupancyMap(20,20,.05,fill=0)
            node._map_received = time.monotonic()
            node.pose = lambda: ((.25,.25), 'tf')
            node.tf = Buffer()
            now = node.get_clock().now().nanoseconds*1e-9
            node.on_obstacle_tracks(String(data=json.dumps(dict(stamp=now+1000., frame='odom', tracks=[]))))
            self.assertIsNone(node.obstacle_observation)
            for stamp in (now-1., now+.2):
                tf = TransformStamped()
                tf.header.frame_id, tf.child_frame_id = 'map', 'odom'
                tf.header.stamp = Time(seconds=stamp).to_msg()
                tf.transform.rotation.w = 1.
                node.tf.set_transform(tf, 'test_slam')
            node.on_obstacle_tracks(String(data=json.dumps(dict(stamp=now-.1, frame='odom',
                tracks=[dict(position=[.7,.7], radius=.02)]))))
            node.brain.plan = Mock(return_value=(None,None,'test'))
            node._pub_status = Mock()
            node._pub_options = Mock()
            node.plan()
            node.brain.plan.assert_called_once()
            self.assertIsNot(node.brain.plan.call_args.args[0], node.map_obj)
            # Even the explicit smaller escape clearance must retain the
            # tracking envelope when no fresh calibration profile is present.
            self.assertEqual(node.brain.plan.call_args.args[0].cell(15,14), 100)
        finally:
            node.destroy_node()
