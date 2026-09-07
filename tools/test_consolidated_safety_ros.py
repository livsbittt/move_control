"""ROS integration contracts for consolidated final safety authority."""
import json
import time
import unittest
from unittest.mock import Mock

import rclpy
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import Bool, Float32MultiArray, String
from move_control.control.calibration_profile import make_profile
from move_control.safety.node import SafetyNode


class ConsolidatedSafetyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(domain_id=219)

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = SafetyNode()
        self.node.pub = Mock()
        self.node.release_estop()
        self.node.set_parameters([Parameter('lidar_use_tf', value=False)])
        self.fresh()
        self.node._refresh_distances()
        self.node.refresh_profile()

    def tearDown(self):
        self.node.destroy_node()

    def fresh(self):
        n = self.node
        for name in ('lidar', 'imu', 'ir'):
            n.observe(name)
        n.last_scan_time = n.last_ir_time = n.last_imu_time = n.now()
        n.ir_raw = (2100, 2100, 2100)
        for field in ('lidar_front', 'lidar_rear', 'lidar_left', 'lidar_right',
                      'lidar_rear_left', 'lidar_rear_right'):
            setattr(n, field, .5)

    def command(self, v=.01, w=0.):
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = v, w
        self.node.on_cmd(cmd)
        self.node.tick()
        return self.node.pub.publish.call_args.args[0]

    def test_split_topics_have_no_authority_atomic_lease_does(self):
        n = self.node
        n.on_drive_scale(Float32MultiArray(data=[1.25, 1.25]))
        n.on_drive_ready(Bool(data=True))
        self.assertAlmostEqual(self.command().linear.x, .01)
        packet = make_profile('test', 1, n.now().nanoseconds*1e-9, True,
                              (1.25, 1.25), n.profile.revision)
        n.on_calibration_profile(String(data=json.dumps(packet)))
        self.assertAlmostEqual(self.command().linear.x, .0125)
        n.calibration_lease.deadline = time.monotonic()-1
        self.assertAlmostEqual(self.command().linear.x, .01)

    def test_caps_preserve_requested_curvature(self):
        out = self.command(.028, .2)
        self.assertAlmostEqual(out.linear.x, .014)
        self.assertAlmostEqual(out.angular.z, .1)

    def test_unknown_required_sensor_stops_and_discards_command(self):
        self.assertGreater(self.command().linear.x, 0.)
        self.node.observe('imu', valid=False)
        self.node.tick()
        self.assertEqual(self.node.pub.publish.call_args.args[0].linear.x, 0.)
        self.fresh()
        self.node.tick()
        self.assertEqual(self.node.pub.publish.call_args.args[0].linear.x, 0.)

    def test_nonfinite_unused_twist_axis_rejected(self):
        cmd = Twist()
        cmd.linear.x = .01
        cmd.angular.x = float('nan')
        self.node.on_cmd(cmd)
        self.assertIsNone(self.node.last_cmd_time)
        self.assertEqual(self.node.last_decision['reason'], 'invalid_command')

    def test_profile_fingerprint_stable_and_geometry_revokes_lease(self):
        n = self.node
        rev = n.profile.revision
        n.refresh_profile()
        self.assertEqual(n.profile.revision, rev)
        packet = make_profile('test', 1, n.now().nanoseconds*1e-9, True, (1.2, 1.2), rev)
        n.on_calibration_profile(String(data=json.dumps(packet)))
        n.set_parameters([Parameter('cmd_linear_sign', value=-1.)])
        n._refresh_distances()
        n.refresh_profile()
        self.assertNotEqual(n.profile.revision, rev)
        self.assertFalse(n.calibration_lease.live(time.monotonic()))

    def test_rgb_and_missing_optional_ultrasound_do_not_gate(self):
        n = self.node
        n.on_cam_cliff(Bool(data=True))
        n.on_cam_block(Bool(data=True))
        self.assertGreater(self.command().linear.x, 0.)

    def test_duplicate_source_stamp_does_not_renew_observation(self):
        n = self.node
        scan = LaserScan()
        scan.header.stamp = n.now().to_msg()
        self.assertTrue(n.observe('lidar', scan))
        row = n.observations.rows['lidar']
        generation, received = row.generation, row.received
        self.assertFalse(n.observe('lidar', scan))
        self.assertEqual(row.generation, generation)
        self.assertEqual(row.received, received)

    def test_nonfinite_geometry_stops_before_serializing_telemetry(self):
        n = self.node
        n.set_parameters([Parameter('us_stop_distance', value=float('nan'))])
        self.command()
        self.assertEqual(n.last_decision['reason'], 'invalid_geometry')
        self.assertEqual(n.pub.publish.call_args.args[0].linear.x, 0.)

    def test_no_echo_cannot_release_confirmed_ultrasonic_contact(self):
        n = self.node
        n.set_parameters([Parameter('us_stop_distance', value=.03),
                          Parameter('us_clear_distance', value=.04)])
        def ping(value):
            msg = Range()
            msg.header.stamp = n.now().to_msg()
            msg.range, msg.min_range, msg.max_range = value, .02, 3.
            n.on_us(msg)
            self.command()
        for _ in range(3):
            ping(.025)
        self.assertTrue(n.us_blocked)
        for _ in range(8):
            ping(float('nan'))
        self.assertTrue(n.us_blocked)
        self.assertEqual(n.pub.publish.call_args.args[0].linear.x, 0.)

    def test_no_echo_after_valid_callback_same_tick_cannot_release_contact(self):
        n = self.node
        n.us_blocked = True
        msg = Range()
        msg.header.stamp = n.now().to_msg()
        msg.range, msg.min_range, msg.max_range = .5, .02, 3.
        n.on_us(msg)
        msg.range = float('nan')
        n.on_us(msg)
        self.command()
        self.assertTrue(n.us_blocked)
        self.assertEqual(n.pub.publish.call_args.args[0].linear.x, 0.)

    def test_impossibly_narrow_corridor_is_unknown_not_open(self):
        n = self.node
        n.narrow_pub = Mock()
        n._scale_update(.051, .051)
        self.assertEqual(n.narrow_pub.publish.call_args.args[0].data, 0.)

    def test_blocked_mixed_curve_cannot_become_straight_or_spin(self):
        n = self.node
        n.lidar_left = .09
        out = self.command(.008, .06)
        self.assertEqual((out.linear.x, out.angular.z), (0., 0.))
        self.assertEqual(n.last_decision['reason'], 'trajectory_changed')
        self.fresh()
        n.lidar_front = .115
        out = self.command(.008, .06)
        self.assertEqual((out.linear.x, out.angular.z), (0., 0.))

    def test_tilt_reverse_cannot_change_mixed_forward_curve(self):
        n = self.node
        n.tilt = True
        out = self.command(.008, .05)
        self.assertEqual((out.linear.x, out.angular.z), (0., 0.))
        self.assertEqual(n.last_decision['reason'], 'trajectory_changed')

    def test_timer_ticks_do_not_invent_filter_samples(self):
        n = self.node
        n._filt('front', .5)
        first = n._filtered_values['front']
        for _ in range(10):
            n._filt('front', .1)
        self.assertEqual(n._filtered_values['front'], first)


if __name__ == '__main__':
    unittest.main()
