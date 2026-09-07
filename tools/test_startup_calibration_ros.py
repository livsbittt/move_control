"""No physical devices: validate calibration lifecycle in isolated ROS domain."""
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch
from types import SimpleNamespace
import numpy as np

import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan, Imu

from move_control.startup_calibration_node import StartupCalibrationNode
from move_control.control.calibration import StationaryBaseline
from move_control.control.safety_profile import SafetyProfile
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
        self.node = StartupCalibrationNode(parameter_overrides=[Parameter('result_path', value=str(path)),
            Parameter('calibration_rotation', value=False)])
        self.node.raw_pub = Mock()
        self.node.ready_pub = Mock()
        self.node.profile_pub = Mock()
        self.node.read_tf = Mock()
        self.clock = patch('move_control.startup_calibration_node.time.monotonic', return_value=100.)
        self.now = self.clock.start()

    def tearDown(self):
        self.clock.stop()
        self.node.destroy_node()
        self.tmp.cleanup()

    def refresh(self, when, moving=False):
        self.now.return_value = when
        profile = SafetyProfile.build().report()
        profile['valid'] = True
        self.node.on_safety_profile(String(data=json.dumps(profile)))
        previous = self.node.raw_pub.publish.call_args
        v = previous.args[0].linear.x if previous else 0.
        w = previous.args[0].angular.z if previous else 0.
        self.node.on_gate_decision(String(data=json.dumps({
            'issued_s': self.node.get_clock().now().nanoseconds*1e-9,
            'requested_v': v, 'requested_omega': w, 'safe_v': v, 'safe_omega': w})))
        for name, value in VALUES.items():
            if moving and name in ('odom', 'lidar', 'us', 'map_tf'):
                value = {'odom': (.03, 0., 0., 0.), 'lidar': (.62,), 'us': (.62,), 'map_tf': (.03, 0., 0.)}[name]
            self.node.baseline.add(name, value, when)
            self.node.observations.add(name, when)
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

    def test_verified_turn_preserves_calibration_but_invalid_imu_does_not(self):
        msg = Imu()
        msg.orientation.w = 1.
        msg.linear_acceleration.z = 9.86
        msg.angular_velocity.z = .2
        self.node.set_parameters([Parameter('imu_angular_velocity_unit', value='rad_s')])
        for phase, tilted, gyro, valid in (
                ('ready', False, .2, True), ('collecting', False, .2, False),
                ('ready', True, .2, False), ('ready', False, float('nan'), False)):
            self.node.phase = phase
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.orientation.x = math.sin(math.radians(30)/2) if tilted else 0.
            msg.orientation.w = math.cos(math.radians(30)/2) if tilted else 1.
            msg.angular_velocity.z = gyro
            self.node.on_imu(msg)
            self.assertEqual(self.node.baseline.latest('imu') is not None, valid)

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
        msg.header.stamp = self.node.get_clock().now().to_msg()
        self.node.on_imu(msg)
        self.assertIsNone(self.node.baseline.latest('imu'))
        self.node.set_parameters([Parameter('imu_angular_velocity_unit', value='rad_s')])
        msg.angular_velocity.x = .2
        msg.header.stamp = self.node.get_clock().now().to_msg()
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

    def test_automatic_flow_waits_for_estop_release_then_completes_without_command(self):
        from std_msgs.msg import Bool
        for i in range(21):
            self.refresh(96. + i * .2)
        self.node.estop = True
        self.node.tick()
        self.assertEqual(self.node.phase, 'waiting_motion')
        self.assertFalse(any(c.args[0].linear.x for c in self.node.raw_pub.publish.call_args_list))
        self.node.on_estop(Bool(data=False))
        self.node.tick()
        self.assertEqual(self.node.phase, 'validating_motion')
        self.refresh(100.6)
        self.node.tick()
        self.assertEqual(self.node.raw_pub.publish.call_args.args[0].linear.x, .008)
        self.refresh(104.7, moving=True)
        self.node.tick()
        self.assertEqual(self.node.phase, 'ready')
        self.assertEqual(self.node.raw_pub.publish.call_args.args[0].linear.x, 0.)
        self.assertFalse(json.loads((Path(self.tmp.name) / 'calibration.json').read_text())['ready'])

    def test_auto_motion_never_retries_after_abort_or_failure(self):
        for final_phase in ('aborted', 'failed'):
            self.node.finish(False, 'test', final_phase)
            for i in range(21):
                self.refresh(96. + i * .2)
            self.node.estop = False
            self.node.tick()
            self.assertEqual(self.node.phase, final_phase)
            self.assertEqual(self.node.raw_pub.publish.call_args.args[0].linear.x, 0.)

    def test_tf_collection_time_is_not_mistaken_for_future_sensor_data(self):
        for i in range(21):
            self.refresh(96. + i * .2)
        self.node.estop = False
        def collect():
            self.refresh(100.01)
        self.node.read_tf.side_effect = collect
        self.node.tick()
        self.assertEqual(self.node.phase, 'validating_motion')

    def test_round_trip_requires_rear_clearance_and_persists_corrections(self):
        self.node.set_parameters([Parameter('calibration_round_trip', value=True)])
        for i in range(21):
            self.refresh(96. + i*.2)
        self.node.estop = False
        self.node.tick()
        self.assertEqual(self.node.phase, 'waiting_motion')
        self.node.rear_clear = (100., True)
        self.node.tick()
        x = speed = 0.
        for i in range(1, 700):
            now = 100. + i*.05
            x += speed*.05*.92
            self.refresh(now)
            self.node.rear_clear = (now, True)
            for name, value in {'odom': (x,0.,0.,0.), 'map_tf': (x,0.,0.), 'lidar': (.65-x,), 'us': (.65-x,)}.items():
                self.node.baseline.add(name, value, now)
            self.node.tick()
            speed = self.node.raw_pub.publish.call_args.args[0].linear.x
            if self.node.phase in ('ready', 'failed'):
                break
        self.assertEqual(self.node.phase, 'ready', self.node.message)
        self.assertEqual(speed, 0.)
        saved = json.loads((Path(self.tmp.name) / 'calibration.json').read_text())
        self.assertFalse(saved['settings_applied'])  # Published is not acknowledged by safety.
        self.assertEqual(len(saved['motion']['legs']), 4)

    def test_rotation_requires_observed_space_and_never_claims_translation_is_rotation(self):
        self.node.set_parameters([Parameter('calibration_rotation', value=True)])
        self.arm()
        self.refresh(104.7, moving=True)
        self.node.tick()
        self.assertEqual(self.node.phase, 'validating_rotation')
        self.node.tick()
        self.assertEqual(self.node.phase, 'failed')
        self.assertEqual(self.node.raw_pub.publish.call_args.args[0].angular.z, 0.)

    def test_duplicate_imu_packet_does_not_manufacture_baseline_samples(self):
        msg = Imu()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.orientation.w, msg.linear_acceleration.z = 1., 9.86
        for _ in range(25):
            self.node.on_imu(msg)
        self.assertEqual(len(self.node.baseline.samples['imu']), 1)

    def test_round_trip_rear_clearance_loss_aborts_before_reverse(self):
        self.node.set_parameters([Parameter('calibration_round_trip', value=True)])
        self.node.rear_clear = (100., True)
        self.arm()
        self.node.rear_clear = (100.6, False)
        self.node.tick()
        self.assertEqual(self.node.phase, 'failed')
        self.assertEqual(self.node.raw_pub.publish.call_args.args[0].linear.x, 0.)

    def test_explicit_trial_is_bounded_stops_and_persists_only_after_agreement(self):
        self.arm()
        self.assertEqual(self.node.raw_pub.publish.call_args.args[0].linear.x, .008)
        self.refresh(104.7, moving=True)
        self.node.tick()
        self.assertEqual(self.node.phase, 'ready')
        self.assertEqual(self.node.raw_pub.publish.call_args.args[0].linear.x, 0.)
        report = json.loads((Path(self.tmp.name) / 'calibration.json').read_text())
        self.assertFalse(report['ready'])  # Safety acknowledgement has not arrived.
        self.assertTrue(all(report['motion']['checks'].values()))
        self.assertAlmostEqual(report['estimates']['imu_gyro_bias_rad_s'][0], .001)
        self.assertFalse(report['settings_applied'])

    def test_only_matching_safety_ack_enables_ready(self):
        self.arm()
        self.refresh(104.7, moving=True)
        self.node.tick()
        ack = {'applied': True, 'revision': 'other', 'session': self.node.profile_session}
        self.node.on_applied(String(data=json.dumps(ack)))
        self.node.publish()
        self.assertFalse(self.node.ready_pub.publish.call_args.args[0].data)
        ack['revision'] = self.node.profile_revision
        self.node.on_applied(String(data=json.dumps(ack)))
        self.node.publish()
        self.assertTrue(self.node.ready_pub.publish.call_args.args[0].data)

    def test_geometry_change_revokes_without_relabelling_trial(self):
        self.arm()
        original = self.node.trial_geometry_revision
        changed = SafetyProfile.build(radius=.12).report()
        changed['valid'] = True
        self.node.on_safety_profile(String(data=json.dumps(changed)))
        self.assertEqual(self.node.phase, 'failed')
        self.assertEqual(self.node.profile_packet()['geometry_revision'], original)
        self.assertFalse(self.node.profile_packet()['enabled'])

    def test_source_age_is_not_extended_by_receipt_time(self):
        self.arm()
        self.node.observations.add('imu', 100.6, source=50., source_now=50.95)
        self.now.return_value = 100.7
        self.node.tick()
        self.assertEqual(self.node.phase, 'failed')

    def test_safety_reduction_is_not_learned_as_motor_gain(self):
        self.arm()
        self.node.on_gate_decision(String(data=json.dumps({
            'issued_s': self.node.get_clock().now().nanoseconds*1e-9,
            'requested_v': .008, 'requested_omega': 0., 'safe_v': .007, 'safe_omega': 0.})))
        self.node.tick()
        self.assertEqual(self.node.phase, 'failed')
        self.assertIn('modified', self.node.message)

    def test_wrong_json_shapes_do_not_crash_callbacks(self):
        for value in ('[]', 'null', '1'):
            self.node.on_safety_profile(String(data=value))
            self.node.on_applied(String(data=value))
            self.node.on_gate_decision(String(data=value))
            self.assertFalse(self.node.settings_applied())
            self.assertIsNone(self.node.geometry_profile)

    def test_bilateral_rotation_persists_final_evidence_and_safety_ack(self):
        from move_control.safety.node import SafetyNode
        self.node.set_parameters([Parameter('calibration_rotation', value=True)])
        self.arm()
        self.refresh(104.7, moving=True)
        self.node.tick()
        increment = math.pi/360
        angles = np.arange(720)*increment
        reference = .7+.15*np.sin(3*angles)+.1*np.cos(7*angles)
        yaw = speed = 0.
        for i in range(1, 1200):
            now = 104.7+i*.05
            yaw += speed*.05*.92
            self.refresh(now)
            self.node.baseline.add('odom', (0., 0., yaw, 0.), now)
            self.node.rotation_imu_yaw = yaw
            self.node.rotation_scan_sample(SimpleNamespace(
                ranges=np.roll(reference, -round(yaw/increment)), angle_increment=increment), True)
            self.node.tick()
            speed = self.node.raw_pub.publish.call_args.args[0].angular.z
            if self.node.phase in ('ready', 'failed'):
                break
        self.assertEqual(self.node.phase, 'ready', self.node.message)
        self.assertEqual(speed, 0.)
        saved = json.loads((Path(self.tmp.name)/'calibration.json').read_text())
        self.assertEqual(len(saved['rotation']['legs']), 8)
        self.assertEqual(saved['profile']['revision'], self.node.profile_revision)
        self.assertFalse(self.node.ready_pub.publish.call_args.args[0].data)
        safety = SafetyNode()
        try:
            safety.calibration_applied_pub = Mock()
            safety.on_calibration_profile(self.node.profile_pub.publish.call_args.args[0])
            self.node.on_applied(safety.calibration_applied_pub.publish.call_args.args[0])
            self.node.publish()
            self.assertTrue(self.node.ready_pub.publish.call_args.args[0].data)
            self.assertNotEqual(safety.calibration_lease.angular_gains(now), (1., 1.))
        finally:
            safety.destroy_node()

    def test_hazard_or_stale_data_aborts_trial_with_zero(self):
        self.arm()
        self.node.hazards['/safety/blocked'] = (100.6, True)
        self.node.tick()
        self.assertEqual(self.node.phase, 'failed')
        self.assertEqual(self.node.raw_pub.publish.call_args.args[0].linear.x, 0.)
        self.assertFalse(self.node.ready_pub.publish.call_args.args[0].data)

    def test_close_raw_range_stops_even_when_median_still_looks_clear(self):
        self.arm()
        self.node.raw_ranges['lidar'] = (100.6, .1, True)
        self.node.tick()
        self.assertEqual(self.node.phase, 'failed')
        self.assertEqual(self.node.raw_pub.publish.call_args.args[0].linear.x, 0.)

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
