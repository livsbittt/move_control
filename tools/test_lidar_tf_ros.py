"""Mount TF contract tests in an isolated ROS domain; output is mocked."""
import math
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformException
from move_control.safety.node import SafetyNode
from move_control import web_node as web


class LidarTfTest(unittest.TestCase):
    def test_blind_wedge_cannot_remove_a_rotation_obstacle(self):
        import json
        from std_msgs.msg import String
        from move_control.control.rotation_envelope import RotationEnvelope
        from move_control.control.calibration_profile import make_profile
        n = self.node
        n.release_estop()
        self.scan_once()
        n._refresh_distances()
        n.refresh_profile()
        envelope = RotationEnvelope(n.robot_r)
        for yaw in (.17, -.17, .18, -.18):
            envelope.add((0.,0.,yaw), (0.,0.,yaw), yaw, .0005)
        rotation = dict(done=True,error=None,max_angular_rad_s=.06,legs=[{}]*8,
                        angular_gains=[1.,1.],envelope=envelope.report())
        packet = make_profile('wedge',1,n.now().nanoseconds*1e-9,True,(1.,1.),n.profile.revision,rotation)
        n.on_calibration_profile(String(data=json.dumps(packet)))
        for missing in (.07, math.nan, math.inf, 0.):
            self.scan.header.stamp = n.now().to_msg()
            ranges = [.5]*720
            ranges[200:240] = [missing]*40
            self.scan.ranges = ranges
            self.scan_once()
            n.last_imu_time = n.last_ir_time = n.now()
            n.ir_raw = (2100,2100,2100)
            for name in ('imu','ir'):
                n.observe(name)
            command = Twist()
            command.angular.z = .04
            n.on_cmd(command)
            n.tick()
            self.assertEqual(n.pub.publish.call_args.args[0].angular.z, 0., str(missing))

    @classmethod
    def setUpClass(cls):
        rclpy.init(domain_id=218)

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = SafetyNode()
        self.node.pub = Mock()
        self.scan = LaserScan()
        self.scan.header.frame_id = 'rplidar_link'
        self.scan.header.stamp = self.node.now().to_msg()
        self.scan.angle_min = -math.pi
        self.scan.angle_increment = 2 * math.pi / 720
        self.scan.range_min = .05
        self.scan.range_max = 12.0
        self.scan.ranges = [.5] * 720
        self.tf = TransformStamped()
        self.tf.transform.rotation.z = 1.0
        self.tf.transform.rotation.w = 0.0
        self.node.lidar_tf = Mock()
        self.node.lidar_tf.lookup_transform.return_value = self.tf

    def tearDown(self):
        self.node.destroy_node()

    def scan_once(self):
        with patch('move_control.safety.bumper.is_robot_scan', return_value=True):
            self.node.on_scan(self.scan)

    def test_tf_mount_wins_over_machine_calibration_and_refresh(self):
        self.node.set_parameters([Parameter('lidar_yaw_offset', value=math.radians(190))])
        self.scan_once()
        self.node._refresh_distances()
        self.assertAlmostEqual(self.node.lidar_yaw, math.pi)
        self.assertIsNotNone(self.node.last_scan_time)
        call = self.node.lidar_tf.lookup_transform.call_args.args
        self.assertEqual(call[:2], ('base_link', 'rplidar_link'))
        self.assertEqual(call[2].nanoseconds, self.scan.header.stamp.sec * 10**9 +
                         self.scan.header.stamp.nanosec)

    def test_missing_tf_invalidates_previous_scan_and_zeros_motion(self):
        self.scan_once()
        self.node.lidar_tf.lookup_transform.side_effect = TransformException('missing')
        self.scan_once()
        self.assertIsNone(self.node.last_scan_time)
        self.node.release_estop()
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = .02, .2
        self.node.on_cmd(cmd)
        self.node.tick()
        out = self.node.pub.publish.call_args.args[0]
        self.assertEqual((out.linear.x, out.angular.z), (0, 0))

    def test_explicit_legacy_mode_uses_parameter_without_tf(self):
        self.node.set_parameters([Parameter('lidar_use_tf', value=False),
                                  Parameter('lidar_yaw_offset', value=math.radians(190))])
        self.scan_once()
        self.node.lidar_tf.lookup_transform.assert_not_called()
        self.assertAlmostEqual(self.node.lidar_yaw, math.radians(190))

    def test_web_uses_identical_tf_nose_and_drops_missing_mount(self):
        web.STATE.clear()
        viewer = SimpleNamespace(tf=self.node.lidar_tf, scan_step=4)
        web.WebNode.on_scan(viewer, self.scan)
        self.assertAlmostEqual(web.STATE['scan']['nose_yaw'], math.pi)
        viewer.tf.lookup_transform.side_effect = TransformException('missing')
        web.WebNode.on_scan(viewer, self.scan)
        self.assertIsNone(web.STATE['scan'])
        self.assertEqual(web.STATE['scan_reason'], 'missing_mount_tf')


if __name__ == '__main__':
    unittest.main()
