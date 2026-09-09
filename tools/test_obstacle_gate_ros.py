import json
import unittest
from unittest.mock import Mock

import rclpy
from rclpy.parameter import Parameter
from geometry_msgs.msg import TransformStamped, Twist
from std_msgs.msg import String
from rosy_control.safety.node import SafetyNode


class ObstacleGateTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(domain_id=232)

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = SafetyNode()
        self.node.set_parameters([Parameter('obstacle_tracking_enabled', value=True)])
        transform = TransformStamped()
        transform.header.stamp = self.node.get_clock().now().to_msg()
        transform.transform.rotation.w = 1.
        self.node.lidar_tf = Mock()
        self.node.lidar_tf.lookup_transform.return_value = transform

    def tearDown(self):
        self.node.destroy_node()

    def observe(self, tracks=(), blocked=False):
        stamp = self.node.get_clock().now().nanoseconds*1e-9
        self.node.on_camera_observation(String(data=json.dumps(dict(stamp=stamp, blocked=blocked))))
        self.node.on_obstacle_tracks(String(data=json.dumps(dict(stamp=stamp, frame='odom', tracks=list(tracks)))))

    def test_missing_camera_and_tracks_hold(self):
        self.assertEqual(self.node.obstacle_tracking_hold(), 'camera_observation_unavailable')
        self.observe()
        self.assertIsNone(self.node.obstacle_tracking_hold())

    def test_camera_only_obstacle_holds_without_metric_guess(self):
        self.observe(blocked=True)
        self.assertEqual(self.node.obstacle_tracking_hold(), 'camera_obstacle_unranged')

    def test_approaching_object_holds_even_before_existing_bumper_distance(self):
        self.observe([dict(id=1, position=[.3,0.], velocity=[-.15,0.], state='moving',
                           radius=.03, age=0., observed=True)])
        self.assertTrue(self.node.obstacle_tracking_hold().startswith('obstacle_wait'))

    def test_repeated_packet_does_not_refresh_source_time(self):
        self.observe()
        old = self.node.obstacle_observation.copy()
        self.node.on_obstacle_tracks(String(data=json.dumps(dict(old, tracks=[{}]))))
        self.assertEqual(self.node.obstacle_observation, old)

    def test_future_packet_cannot_block_later_valid_camera_or_tracks(self):
        future = self.node.now().nanoseconds*1e-9+1000.
        message = String(data=json.dumps(dict(stamp=future, frame='odom', tracks=[], blocked=True)))
        self.node.on_camera_observation(message)
        self.node.on_obstacle_tracks(message)
        self.assertIsNone(self.node.camera_observation)
        self.assertIsNone(self.node.obstacle_observation)
        self.observe()
        self.assertIsNone(self.node.obstacle_tracking_hold())

    def test_disabled_feature_does_not_claim_tracking_readiness(self):
        self.node.set_parameters([Parameter('obstacle_tracking_enabled', value=False)])
        self.assertIsNone(self.node.obstacle_tracking_hold())

    def test_final_publisher_stops_actual_raw_command_for_approaching_track(self):
        n = self.node
        n.pub = Mock()
        n.release_estop()
        n.set_parameters([Parameter('lidar_use_tf', value=False)])
        for name in ('lidar', 'imu', 'ir'):
            n.observe(name)
        n.last_scan_time = n.last_ir_time = n.last_imu_time = n.now()
        n.ir_raw = (2100, 2100, 2100)
        for field in ('lidar_front', 'lidar_rear', 'lidar_left', 'lidar_right', 'lidar_rear_left', 'lidar_rear_right'):
            setattr(n, field, .5)
        n._refresh_distances()
        n.refresh_profile()
        self.observe()
        cmd = Twist()
        cmd.linear.x = .01
        n.on_cmd(cmd)
        n.tick()
        self.assertGreater(abs(n.pub.publish.call_args.args[0].linear.x), 0.)
        self.observe([dict(id=1, position=[.3,0.], velocity=[-.15,0.], state='moving',
                           radius=.03, age=0., observed=True)])
        n.on_cmd(cmd)
        n.tick()
        output = n.pub.publish.call_args.args[0]
        self.assertEqual((output.linear.x, output.angular.z), (0.,0.))
        self.assertTrue(n.last_decision['reason'].startswith('obstacle_wait'))
