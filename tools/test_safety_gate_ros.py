"""Local ROS safety-node tests. A dedicated domain never joins the robot."""
import unittest
import json
from unittest.mock import Mock

import rclpy
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32MultiArray, String
from move_control.control.calibration_profile import make_profile
from move_control.safety.node import SafetyNode


class SafetyGateTest(unittest.TestCase):
    def test_verified_translation_uses_body_clearance_but_turning_keeps_circle(self):
        from rclpy.parameter import Parameter
        self.node.set_parameters([Parameter('footprint_guard_enabled', value=True),
                                  Parameter('lidar_use_tf', value=True)])
        self.node.release_estop()
        self.node.lidar_mount = (-.017, 0.)
        self.node.translation_clearance = (.03, .03)
        self.fresh_sensors()
        self.node.lidar_measurement_time = self.node.now()
        for field in ('lidar_front', 'lidar_rear', 'lidar_left', 'lidar_right',
                      'lidar_rear_left', 'lidar_rear_right'):
            setattr(self.node, field, .1)
        command = Twist()
        command.linear.x = .008
        self.node.on_cmd(command)
        self.node.tick()
        self.assertGreater(self.node.pub.publish.call_args.args[0].linear.x, 0.)
        measured = self.node.lidar_measurement_time
        self.node.lidar_measurement_time = None
        self.node.tick()
        self.assertEqual(self.node.pub.publish.call_args.args[0].linear.x, 0.)
        self.node.lidar_measurement_time = measured
        self.apply_profile((1.25, 1.25))
        command.linear.x = .014
        self.node.on_cmd(command)
        self.node.tick()
        self.assertEqual(self.node.pub.publish.call_args.args[0].linear.x, 0.)
        command.linear.x = .008
        self.node.on_cmd(command)
        self.node.translation_clearance = (0., .03)
        self.node.tick()
        self.assertEqual(self.node.pub.publish.call_args.args[0].linear.x, 0.)
        self.node.translation_clearance = (.03, .03)
        command.angular.z = .1
        self.node.on_cmd(command)
        self.node.tick()
        self.assertEqual(self.node.pub.publish.call_args.args[0].linear.x, 0.)

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

    def fresh_sensors(self):
        n = self.node
        n.last_scan_time = n.last_imu_time = n.last_ir_time = n.now()
        n.ir_raw = (2100, 2100, 2100)
        for name in ('lidar', 'imu', 'ir'):
            n.observe(name)

    def apply_profile(self, gains, enabled=True, sequence=1):
        n = self.node
        n._refresh_distances()
        n.refresh_profile()
        packet = make_profile('gate-test', sequence, n.now().nanoseconds*1e-9,
                              enabled, gains, n.profile.revision)
        n.on_calibration_profile(String(data=json.dumps(packet)))

    def test_verified_fresh_gains_apply_only_to_low_speed_and_are_revocable(self):
        self.node.on_drive_scale(Float32MultiArray(data=[1.1, .9]))
        self.node.on_drive_ready(Bool(data=True))
        self.assertEqual(self.node.corrected_drive_speed(.008), .008)
        self.apply_profile((1.1, .9))
        self.assertAlmostEqual(self.node.corrected_drive_speed(.008), .0088)
        self.assertAlmostEqual(self.node.corrected_drive_speed(-.008), -.0072)
        self.assertEqual(self.node.corrected_drive_speed(.05), .05)
        self.apply_profile((1.1, .9), enabled=False, sequence=2)
        self.assertEqual(self.node.corrected_drive_speed(.008), .008)
        self.apply_profile((1.1, .9), sequence=3)
        self.node.calibration_lease.deadline = 0.
        self.assertEqual(self.node.corrected_drive_speed(.008), .008)
        self.apply_profile((1.1, .9), sequence=4)
        self.node.on_calibration_profile(String(data='{}'))
        self.assertEqual(self.node.corrected_drive_speed(.008), .008)

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
        self.fresh_sensors()
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

    def test_camera_observations_cannot_block_clear_lidar(self):
        self.node.release_estop()
        self.fresh_sensors()
        self.node.last_cam_time = self.node.now()
        self.node.cam_block = self.node.cam_cliff = True
        for field in ('lidar_front', 'lidar_rear', 'lidar_left', 'lidar_right',
                      'lidar_rear_left', 'lidar_rear_right'):
            setattr(self.node, field, .5)
        command = Twist()
        command.linear.x = .02
        self.node.on_cmd(command)
        self.node.tick()
        self.assertFalse(self.node.block_pub.publish.call_args.args[0].data)
        self.assertGreater(self.node.pub.publish.call_args.args[0].linear.x, 0.)

    def test_tf_rear_limit_is_used_by_tick_and_reported_to_calibration(self):
        self.node.set_parameters([Parameter('lidar_use_tf', value=True)])
        self.node.lidar_mount = (-.017, 0.)
        self.fresh_sensors()
        self.node.lidar_front = .5
        self.node.lidar_rear = .117
        self.node.can_rev_pub = Mock()
        self.node.motion_limits_pub = Mock()
        self.node.tick()
        self.assertTrue(self.node.can_rev_pub.publish.call_args.args[0].data)
        data = json.loads(self.node.motion_limits_pub.publish.call_args.args[0].data)
        self.assertAlmostEqual(data['front_stop_m'], .12)
        self.assertLess(data['rear_stop_m'], .1)
        self.assertAlmostEqual(data['rear_m'], .117)
        self.node.lidar_mount = None
        self.node.tick()
        self.assertFalse(self.node.can_rev_pub.publish.call_args.args[0].data)
