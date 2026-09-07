"""No physical devices: validate calibration lifecycle in isolated ROS domain."""
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan, Imu

from move_control.startup_calibration_node import StartupCalibrationNode
from move_control.control.calibration import StationaryBaseline
from test.test_calibration import VALUES


class StartupCalibrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(domain_id=219)

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        path = Path(self.tmp.name) / 'calibration.json'
        path.write_text('{"ready": true, "phase": "ready"}')
        self.node = StartupCalibrationNode(parameter_overrides=[Parameter('result_path', value=str(path))])
        self.node.raw_pub = Mock()
        self.node.ready_pub = Mock()
        self.node.read_tf = Mock()
        self.clock = patch('move_control.startup_calibration_node.time.monotonic', return_value=100.)
        self.now = self.clock.start()

    def tearDown(self):
        self.clock.stop()
        self.node.destroy_node()
        self.tmp.cleanup()

    def refresh(self, when, moving=False):
        self.now.return_value = when
        for name, value in VALUES.items():
            if moving and name in ('odom', 'lidar', 'us', 'map_tf'):
                value = {'odom': (.03, 0., 0., 0.), 'lidar': (.62,), 'us': (.62,), 'map_tf': (.03, 0., 0.)}[name]
            self.node.baseline.add(name, value, when)
        for topic in ('/safety/blocked', '/safety/cliff', '/safety/tilt', '/safety/pickup', '/camera/blocked', '/camera/cliff'):
            self.node.hazards[topic] = (when, False)
        self.node.wander_state = ('stop', when)

    def arm(self):
        self.node.baseline = StationaryBaseline()
        for i in range(21):
            self.refresh(96. + i * .2)
        self.node.phase = 'waiting_motion'
        self.node.estop = False
        self.node.on_command(String(data='validate_motion'))
        self.refresh(100.6)
        self.node.tick()

    def test_driver_degree_units_are_converted_without_hiding_real_rotation(self):
        msg = Imu()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.orientation.w = 1.
        msg.linear_acceleration.z = 9.86
        msg.angular_velocity.x = .9375
        self.node.set_parameters([Parameter('imu_angular_velocity_unit', value='deg_s')])
        self.node.on_imu(msg)
        self.assertAlmostEqual(self.node.baseline.latest('imu')[1], math.radians(.9375))
        msg.angular_velocity.x = 90.
        self.node.on_imu(msg)
        self.assertIsNone(self.node.baseline.latest('imu'))
        self.node.set_parameters([Parameter('imu_angular_velocity_unit', value='rad_s')])
        msg.angular_velocity.x = .2
        self.node.on_imu(msg)
        self.assertIsNone(self.node.baseline.latest('imu'))

    def test_boot_and_estopped_request_never_publish_positive_velocity(self):
        self.assertEqual(self.node.phase, 'collecting')
        saved = json.loads((Path(self.tmp.name) / 'calibration.json').read_text())
        self.assertFalse(saved['ready'])
        self.node.tick()
        self.node.estop = True
        self.node.on_command(String(data='validate_motion'))
        self.assertNotEqual(self.node.phase, 'validating_motion')
        self.assertTrue(all(call.args[0].linear.x == 0 for call in self.node.raw_pub.publish.call_args_list))
        self.assertFalse(self.node.ready_pub.publish.call_args.args[0].data)

    def test_explicit_trial_is_bounded_stops_and_persists_only_after_agreement(self):
        self.arm()
        self.assertEqual(self.node.raw_pub.publish.call_args.args[0].linear.x, .008)
        self.refresh(104.7, moving=True)
        self.node.tick()
        self.assertEqual(self.node.phase, 'ready')
        self.assertEqual(self.node.raw_pub.publish.call_args.args[0].linear.x, 0.)
        report = json.loads((Path(self.tmp.name) / 'calibration.json').read_text())
        self.assertTrue(report['ready'])
        self.assertTrue(all(report['motion']['checks'].values()))
        self.assertAlmostEqual(report['estimates']['imu_gyro_bias_rad_s'][0], .001)
        self.assertFalse(report['settings_applied'])

    def test_hazard_or_stale_data_aborts_trial_with_zero(self):
        self.arm()
        self.node.hazards['/safety/blocked'] = (100.6, True)
        self.node.tick()
        self.assertEqual(self.node.phase, 'failed')
        self.assertEqual(self.node.raw_pub.publish.call_args.args[0].linear.x, 0.)
        self.assertFalse(self.node.ready_pub.publish.call_args.args[0].data)

    def test_ready_is_revoked_when_required_map_or_sensor_disappears(self):
        self.arm()
        self.refresh(104.7, moving=True)
        self.node.tick()
        self.now.return_value = 111.
        self.node.tick()
        self.assertEqual(self.node.phase, 'failed')
        self.assertFalse(self.node.ready_pub.publish.call_args.args[0].data)

    def test_lidar_nose_uses_actual_tf_instead_of_legacy_parameter(self):
        transform = TransformStamped()
        transform.transform.rotation.z = 1.
        transform.transform.rotation.w = 0.
        self.node.tf = Mock()
        self.node.tf.lookup_transform.return_value = transform
        scan = LaserScan()
        scan.header.frame_id = 'laser'
        scan.header.stamp = self.node.get_clock().now().to_msg()
        scan.angle_increment = math.pi / 360
        scan.range_max = 40.
        ranges = [math.inf] * 720
        ranges[340] = .65  # 170deg is in the true180 cone, not legacy190 cone.
        scan.ranges = ranges
        self.node.on_scan(scan)
        self.assertAlmostEqual(self.node.lidar_nose, math.pi)
        self.assertAlmostEqual(self.node.baseline.latest('lidar')[0], .65, places=5)
