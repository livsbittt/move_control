"""Local ROS safety-node tests. A dedicated domain never joins the robot."""
import unittest
from unittest.mock import Mock

import rclpy
from geometry_msgs.msg import Twist
from move_control.safety.node import SafetyNode


class SafetyGateTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(domain_id=217)

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = SafetyNode()
        self.node.pub = Mock()
        self.node.block_pub = Mock()
        self.node.range_pub = Mock()

    def tearDown(self):
        self.node.destroy_node()

    def test_restart_is_estopped_but_still_publishes_obstacle_telemetry(self):
        self.assertTrue(self.node.estop)
        self.node.tick()
        self.node.range_pub.publish.assert_called_once()
        self.assertTrue(self.node.block_pub.publish.call_args.args[0].data)
        for call in self.node.pub.publish.call_args_list:
            self.assertEqual(call.args[0].linear.x, 0.0)
            self.assertEqual(call.args[0].angular.z, 0.0)

    def test_missing_scan_blocks_forward_reverse_and_rotation(self):
        self.node.release_estop()
        for speed in (.02, -.02):
            command = Twist()
            command.linear.x = speed
            command.angular.z = .2
            self.node.on_cmd(command)
            self.node.tick()
            published = self.node.pub.publish.call_args.args[0]
            self.assertEqual(published.linear.x, 0.0)
            self.assertEqual(published.angular.z, 0.0)

    def test_measured_nine_cm_wall_blocks_command_despite_distant_filter_history(self):
        self.node.release_estop()
        self.node.last_scan_time = self.node.now()
        self.node.lidar_front = .09
        for field in ('lidar_rear', 'lidar_left', 'lidar_right',
                      'lidar_rear_left', 'lidar_rear_right'):
            setattr(self.node, field, .5)
        for _ in range(8):
            self.node._lp['front'].push(.5)
        command = Twist()
        command.linear.x = .02
        command.angular.z = .2
        self.node.on_cmd(command)
        self.node.tick()
        published = self.node.pub.publish.call_args.args[0]
        self.assertEqual(published.linear.x, 0.0)
        self.assertEqual(published.angular.z, 0.0)
