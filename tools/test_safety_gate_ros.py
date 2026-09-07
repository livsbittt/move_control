"""Local ROS safety-node tests. A dedicated domain never joins the robot."""
import json
import math
import time
import unittest
from unittest.mock import Mock

import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Range
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, Float32MultiArray, String
from move_control.control.calibration_profile import make_profile
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

    def healthy(self):
        self.node.release_estop()
        for name in ('lidar', 'ir', 'imu', 'camera_block'):
            self.node.observations.add(name, time.monotonic())
        self.node.last_scan_time = self.node.now()
        for field in ('lidar_front', 'lidar_rear', 'lidar_left', 'lidar_right',
                      'lidar_rear_left', 'lidar_rear_right'):
            setattr(self.node, field, .5)

    def test_nan_command_is_rejected_before_becoming_last_command(self):
        self.node.release_estop()
        command = Twist()
        command.linear.x = float('nan')
        self.node.on_cmd(command)
        self.assertEqual(self.node.last_cmd.linear.x, 0.)
        self.assertIsNone(self.node.last_cmd_time)

    def test_final_limits_apply_to_every_request_and_preserve_curve(self):
        self.healthy()
        command = Twist()
        command.linear.x, command.angular.z = .028, .2
        self.node.on_cmd(command)
        self.node.tick()
        output = self.node.pub.publish.call_args.args[0]
        self.assertAlmostEqual(output.linear.x, .014)
        self.assertAlmostEqual(output.angular.z, .1)

    def test_blocked_spin_does_not_turn_curve_into_unvalidated_straight(self):
        self.healthy()
        self.node.lidar_left = .09
        command = Twist()
        command.linear.x, command.angular.z = .008, .1
        self.node.on_cmd(command)
        self.node.tick()
        output = self.node.pub.publish.call_args.args[0]
        self.assertEqual((output.linear.x, output.angular.z), (0., 0.))

    def test_missing_ir_is_unknown_and_stops_final_output(self):
        self.healthy()
        self.node.observations.add('ir', time.monotonic(), valid=False)
        command = Twist()
        command.linear.x = .008
        self.node.on_cmd(command)
        self.node.tick()
        self.assertEqual(self.node.pub.publish.call_args.args[0].linear.x, 0.)
        self.assertEqual(self.node.last_decision['reason'], 'ir_unavailable')

    def test_old_scan_cannot_refresh_safety_and_duplicates_do_not_refilter(self):
        self.node.set_parameters([Parameter('lidar_use_tf', value=False)])
        scan = LaserScan()
        scan.header.stamp.sec = 1700000000
        scan.header.frame_id = 'laser'
        scan.angle_increment = math.pi/360
        scan.range_min, scan.range_max = .05, 40.
        scan.ranges = [.3]*720
        self.node.on_scan(scan)
        self.assertIsNone(self.node.last_scan_time)
        scan.header.stamp = self.node.now().to_msg()
        self.node.on_scan(scan)
        for _ in range(8):
            self.node.tick()
        self.assertEqual(len(self.node._lp['front'].buf), 1)
        self.assertEqual(len(self.node._corr_buf), 1)
        self.node.on_scan(scan)
        self.assertEqual(self.node.observations.generation('lidar'), 1)

    def test_one_us_packet_cannot_count_as_three_hits(self):
        self.node.set_parameters([Parameter('us_stop_distance', value=.05)])
        msg = Range()
        msg.header.stamp = self.node.now().to_msg()
        msg.min_range, msg.max_range, msg.range = .02, 3., .03
        self.node.on_us(msg)
        for _ in range(8):
            self.node.tick()
        self.assertEqual(self.node._us_hits, 1)

    def test_us_no_echo_does_not_release_contact_latch(self):
        self.healthy()
        self.node.us_blocked = True
        msg = Range()
        msg.header.stamp = self.node.now().to_msg()
        msg.min_range, msg.max_range, msg.range = .02, 3., .01
        self.node.on_us(msg)
        self.node.tick()
        self.assertTrue(self.node.us_blocked)

    def test_impossible_corridor_is_not_open_space(self):
        self.node.narrow_pub = Mock()
        self.node._scale_update(.055, .055)
        self.assertEqual(self.node.narrow_pub.publish.call_args.args[0].data, 0.)

    def test_straight_trial_gain_does_not_change_requested_curve(self):
        self.healthy()
        self.node.on_drive_scale(Float32MultiArray(data=[1.25, 1.]))
        self.node.on_drive_ready(Bool(data=True))
        command = Twist()
        command.linear.x, command.angular.z = .008, .08
        self.node.on_cmd(command)
        self.node.tick()
        output = self.node.pub.publish.call_args.args[0]
        self.assertAlmostEqual(output.linear.x, .008)
        self.assertAlmostEqual(output.angular.z, .08)

    def test_camera_block_does_not_refresh_camera_cliff(self):
        self.node.on_cam_block(Bool(data=False))
        self.assertFalse(self.node.observations.fresh('camera_cliff', time.monotonic()))

    def test_profile_reports_effective_values_instead_of_legacy_override(self):
        self.node.profile_pub = Mock()
        self.node.set_parameters([Parameter('stop_distance', value=.018),
                                 Parameter('clear_distance', value=.028),
                                 Parameter('robot_radius', value=.04)])
        self.node.tick()
        report = json.loads(self.node.profile_pub.publish.call_args.args[0].data)
        self.assertEqual(report['effective']['stop'], self.node.stop_d)
        self.assertGreaterEqual(self.node.stop_d, .12)
        self.assertGreaterEqual(self.node.robot_r, .076)

    def test_restart_is_estopped_but_still_publishes_obstacle_telemetry(self):
        self.assertTrue(self.node.estop)
        self.node.tick()
        self.node.range_pub.publish.assert_called_once()
        self.assertTrue(self.node.block_pub.publish.call_args.args[0].data)
        for call in self.node.pub.publish.call_args_list:
            self.assertEqual(call.args[0].linear.x, 0.0)
            self.assertEqual(call.args[0].angular.z, 0.0)

    def test_verified_fresh_gains_apply_only_to_low_speed_and_are_revocable(self):
        self.node.on_drive_scale(Float32MultiArray(data=[1.1, .9]))
        self.assertEqual(self.node.corrected_drive_speed(.008), .008)
        self.node.on_drive_ready(Bool(data=True))
        self.assertEqual(self.node.corrected_drive_speed(.008), .008)
        packet = make_profile('test', 1, self.node.now().nanoseconds*1e-9,
                              True, [1.1, .9], self.node.profile.revision)
        self.node.on_calibration_profile(String(data=json.dumps(packet)))
        self.assertAlmostEqual(self.node.corrected_drive_speed(.008), .0088)
        self.assertAlmostEqual(self.node.corrected_drive_speed(-.008), -.0072)
        self.assertEqual(self.node.corrected_drive_speed(.05), .05)
        self.node.on_drive_ready(Bool(data=False))
        self.assertAlmostEqual(self.node.corrected_drive_speed(.008), .0088)
        self.node.on_drive_ready(Bool(data=True))
        self.node.calibration_lease.deadline = 0.
        self.assertEqual(self.node.corrected_drive_speed(.008), .008)
        self.node.on_drive_scale(Float32MultiArray(data=[float('nan'), 1.]))
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
