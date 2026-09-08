import math
import unittest
from unittest.mock import Mock
import rclpy
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
from move_control.obstacle_observer_node import ObstacleObserver


class ObstacleObserverTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(domain_id=216)

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_only_fresh_horizontal_scan_reaches_mapping_without_rewriting_ranges(self):
        node = ObstacleObserver()
        try:
            node.output = Mock()
            node.mapping_scan = Mock()
            node.tf = Mock()
            tf = TransformStamped()
            tf.transform.rotation.w = 1.
            node.tf.lookup_transform.return_value = tf
            msg = LaserScan()
            msg.header.frame_id = 'laser'
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.angle_min, msg.angle_increment = -.01, .01
            msg.range_min, msg.range_max = .05, 8.
            msg.ranges = [.5,.5,.5]
            node.on_scan(msg)
            node.tick()
            node.mapping_scan.publish.assert_called_once_with(msg)
            self.assertEqual(list(msg.ranges), [.5,.5,.5])
            node.tick()
            self.assertEqual(node.mapping_scan.publish.call_count, 1)
            node.mapping_scan.reset_mock()
            node.output.reset_mock()
            msg.header.stamp = Time(nanoseconds=node.processed_stamp-1).to_msg()
            node.tick()
            node.mapping_scan.publish.assert_not_called()
            node.output.publish.assert_not_called()
            msg.header.stamp = node.get_clock().now().to_msg()
            tf.transform.rotation.y = math.sin(math.radians(10.))
            tf.transform.rotation.w = math.cos(math.radians(10.))
            node.tick()
            node.mapping_scan.publish.assert_not_called()
            node.output.publish.assert_not_called()
            tf.transform.rotation.y, tf.transform.rotation.w = 0., 1.
            msg.ranges = [float('nan')]*3
            node.tick()
            node.mapping_scan.publish.assert_not_called()
            node.output.publish.assert_not_called()
        finally:
            node.destroy_node()
