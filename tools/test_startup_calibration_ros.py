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
from sensor_msgs.msg import LaserScan, Imu, Range

from move_control.startup_calibration_node import StartupCalibrationNode
from move_control.control.calibration import StationaryBaseline
from move_control.control.round_trip import RoundTrip
from test.test_calibration import VALUES
from test.test_calibration_certificate import complete_motion
from move_control.control.calibration_certificate import make_certificate


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
        self.node = StartupCalibrationNode(parameter_overrides=[Parameter('result_path', value=str(path)), Parameter('calibration_rotation', value=False)])
        self.node.raw_pub = Mock()
        self.node.ready_pub = Mock()
        self.node.read_tf = Mock()
        self.clock = patch('move_control.startup_calibration_node.time.monotonic', return_value=100.)
        self.now = self.clock.start()
        original_tick = self.node.tick
        def tick_with_gate_ack():
            original_tick()
            # Simulated final gate acknowledges the freshly published atomic
            # packet; persisted completion remains evidence, not a live lease.
            if self.node.phase == 'ready':
                self.node.publish()
                packet = self.node.profile_packet()
                self.node.on_applied(String(data=json.dumps(dict(applied=packet['enabled'],
                    revision=self.node.profile_revision, session=self.node.profile_session))))
                self.node.publish()
        self.node.tick = tick_with_gate_ack

    def tearDown(self):
        self.clock.stop()
        self.node.destroy_node()
        self.tmp.cleanup()

    def refresh(self, when, moving=False):
        self.now.return_value = when
        self.node.geometry_revision = 'fixture-geometry'
        self.node.geometry_profile = {'effective': {'turn_clear': .1}}
        self.node.geometry_received = when
        request = getattr(self.node, '_trial_request', (when, 0., 0.))
        self.node.gate_decision = (when+.25, dict(requested_v=request[1], safe_v=request[1],
            requested_omega=request[2], safe_omega=request[2]))
        self.node.safety_limits = (when, dict(front_m=.65, rear_m=.65,
            front_stop_m=.12, rear_stop_m=.091, us_stop_m=.02))
        self.node.raw_ranges = {'lidar': (when,.65,True), 'us': (when,.65,True)}
        self.node.us_source_valid = True
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

    def test_invalid_precision_sample_during_motion_stops_without_crashing(self):
        self.arm()
        self.node.motion_start = (100.6, self.node.snapshot())
        self.node.phase = 'validating_motion'
        self.node.baseline.add('lidar', (math.inf,), 100.6, False)
        self.node.tick()
        self.assertEqual(self.node.phase, 'failed')
        self.assertIn('stale or invalid', self.node.message)
        self.assertEqual(self.node.raw_pub.publish.call_args.args[0].linear.x, 0.)

    def test_certificate_restores_only_after_fresh_health_and_explicit_retry_invalidates(self):
        motion = complete_motion()
        self.node.write_json(self.node.certificate_path(),
            make_certificate(self.node.certificate_configuration(), motion))
        self.node.restore_certificate()
        self.assertTrue(self.node.report()['calibration_verified'])
        self.assertFalse(self.node.report()['ready'])
        self.assertEqual(self.node.round_trip.scales[0], motion['forward_scale'])
        self.refresh(100.)
        self.node.tick()
        self.refresh(101.1)
        self.node.tick()
        self.assertTrue(self.node.report()['ready'])
        self.node.on_command(String(data='sensor_check'))
        self.assertFalse(self.node.report()['ready'])
        self.assertTrue(self.node.certificate_path().exists())
        self.node.on_command(String(data='retry'))
        self.assertFalse(self.node.certificate_path().exists())
        self.assertEqual(self.node.phase, 'collecting')

    def test_optional_echo_absence_allows_lidar_evidence_but_stale_source_does_not(self):
        self.node.set_parameters([Parameter('calibration_require_us_agreement', value=False)])
        self.arm()
        self.refresh(100.7)
        self.node.baseline.add('us', (math.inf,), 100.7, False)
        self.node.raw_ranges['us'] = (100.7, .97, False)
        self.assertIsNone(self.node.safe_motion(100.7))
        self.assertIsNone(self.node.motion_clearance['available_us_m'])
        self.node.us_source_valid = False
        self.assertIsNotNone(self.node.safe_motion(100.7))
        self.node.tick()
        self.assertEqual(self.node.phase, 'failed')
        self.assertIn('stale or invalid', self.node.message)
        self.assertEqual(self.node.raw_pub.publish.call_args.args[0].linear.x, 0.)

    def test_precision_pause_holds_zero_then_resumes_on_fresh_wall(self):
        self.arm()
        self.node.motion_start = (100.6, self.node.snapshot())
        self.node.round_trip = RoundTrip(100.6, self.node.snapshot())
        self.node.wall_tracker.diagnostic = {'reason': 'Tracked wall missing or ambiguous'}
        self.node.baseline.add('lidar', (math.inf,), 100.6, False)
        self.node.tick()
        self.assertEqual(self.node.phase, 'validating_motion', self.node.message)
        self.assertEqual(self.node.raw_pub.publish.call_args.args[0].linear.x, 0.)
        self.refresh(100.8)
        self.node.tick()
        self.assertGreater(self.node.raw_pub.publish.call_args.args[0].linear.x, 0.)
        self.assertAlmostEqual(self.node.precision_pause_total, .2)

    def test_short_reacquisition_does_not_fail_due_to_prior_accumulated_pauses(self):
        self.arm()
        self.node.motion_start = (100.6, self.node.snapshot())
        self.node.round_trip = RoundTrip(100.6, self.node.snapshot())
        self.node.precision_pause_total = 2.05
        self.node.wall_tracker.diagnostic = {'reason': 'Tracked wall missing or ambiguous'}
        self.node.baseline.add('lidar', (math.inf,), 100.6, False)
        self.node.tick()
        self.assertEqual(self.node.phase, 'validating_motion')
        self.assertEqual(self.node.raw_pub.publish.call_args.args[0].linear.x, 0.)

    def test_precision_pause_cannot_hide_real_hazard_or_wait_indefinitely(self):
        self.arm()
        self.node.motion_start = (100.6, self.node.snapshot())
        self.node.round_trip = RoundTrip(100.6, self.node.snapshot())
        self.node.wall_tracker.diagnostic = {'reason': 'Tracked wall missing or ambiguous'}
        self.node.baseline.add('lidar', (math.inf,), 100.6, False)
        self.node.hazards['/safety/cliff'] = (100.6, True)
        self.assertFalse(self.node.pause_precision(100.6))
        for i in range(23):
            now = 100.6+i*.05
            self.refresh(now)
            self.node.baseline.add('lidar', (math.inf,), now, False)
            self.node.tick()
            if self.node.phase == 'failed':
                break
        self.assertEqual(self.node.phase, 'failed')
        self.assertEqual(self.node.raw_pub.publish.call_args.args[0].linear.x, 0.)

    def test_collecting_displays_raw_values_and_clearance_without_motion(self):
        self.refresh(100.)
        self.node.phase = 'collecting'
        self.node.tick()
        self.assertIn('raw=0.650m', self.node.sensors['lidar']['detail'])
        self.assertIn('valid=1/1', self.node.sensors['lidar']['detail'])
        self.assertTrue(self.node.motion_clearance)
        self.assertEqual(self.node.phase, 'collecting')

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

    def test_verified_calibration_accepts_turning_but_rejects_tilt_and_invalid_imu(self):
        msg = Imu()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.orientation.w = 1.
        msg.linear_acceleration.z = 9.86
        self.node.set_parameters([Parameter('imu_angular_velocity_unit', value='rad_s')])
        msg.angular_velocity.z = .2
        self.node.phase = 'ready'
        self.node.on_imu(msg)
        self.assertIsNotNone(self.node.baseline.latest('imu'))
        self.node.phase = 'collecting'
        self.node.on_imu(msg)
        self.assertIsNone(self.node.baseline.latest('imu'))

        self.node.phase = 'ready'
        msg.orientation.x = math.sin(math.radians(30)/2)
        msg.orientation.w = math.cos(math.radians(30)/2)
        self.node.on_imu(msg)
        self.assertIsNone(self.node.baseline.latest('imu'))
        msg.orientation.x, msg.orientation.w = 0., 1.
        msg.angular_velocity.z = float('nan')
        self.node.on_imu(msg)
        self.assertIsNone(self.node.baseline.latest('imu'))

    def test_environment_is_measured_while_estop_still_blocks_motion(self):
        from move_control.planning import OccupancyMap
        for i in range(21):
            self.refresh(96.+i*.2)
        self.node.environment_map = OccupancyMap(20,20,.02,fill=0)
        self.node.environment_samples = [(100.,(.14,.13,None,None))]*30
        self.node.estop = True
        self.node.tick()
        self.assertEqual(self.node.phase,'waiting_motion')
        self.assertIsNotNone(self.node.report()['navigation_profile'])
        self.assertFalse(self.node.report()['ready'])

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
        self.assertTrue(self.node.report()['ready'])

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
        self.assertFalse(saved['settings_applied'])
        self.assertTrue(self.node.report()['settings_applied'])
        self.assertEqual(len(saved['motion']['legs']), 4)

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
        self.assertFalse(report['ready'])
        self.assertTrue(self.node.report()['ready'])
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

    def test_close_raw_range_stops_even_when_median_still_looks_clear(self):
        self.arm()
        self.node.raw_ranges['lidar'] = (100.6, .1, True)
        self.node.tick()
        self.assertEqual(self.node.phase, 'failed')
        self.assertEqual(self.node.raw_pub.publish.call_args.args[0].linear.x, 0.)

    def test_actual_safety_clearance_allows_short_stroke_below_twenty_cm(self):
        self.refresh(100.)
        self.node.estop = False
        self.node.safety_limits[1]['front_m'] = .16
        self.assertIsNone(self.node.safe_motion(100.))
        self.assertAlmostEqual(self.node.motion_clearance['target_m'], .032)
        self.node.safety_limits = (98., self.node.safety_limits[1])
        self.assertIn('fresh safety', self.node.safe_motion(100.))

    def test_wide_corridor_jamb_blocks_even_when_precision_reference_is_far(self):
        self.refresh(100.)
        self.node.estop = False
        self.node.safety_limits[1]['front_m'] = .135
        self.assertIsNotNone(self.node.safe_motion(100.))
        self.assertLess(self.node.motion_clearance['target_m'], .02)

    def test_sensor_loss_holds_driving_but_recovers_without_recalibration(self):
        self.arm()
        self.refresh(104.7, moving=True)
        self.node.tick()
        self.node.round_trip = Mock(done=True, scales=[1.1, .95])
        self.node.scale_pub = Mock()
        self.now.return_value = 111.
        self.node.tick()
        self.assertEqual(self.node.phase, 'ready')
        self.assertEqual(self.node.report()['phase'], 'sensor_hold')
        self.assertTrue(self.node.report()['calibration_verified'])
        self.assertFalse(self.node.report()['settings_applied'])
        self.assertAlmostEqual(self.node.scale_pub.publish.call_args.args[0].data[0], 1.1, places=6)
        self.assertFalse(self.node.ready_pub.publish.call_args.args[0].data)
        self.refresh(112., moving=True)
        self.node.tick()
        self.assertFalse(self.node.report()['ready'])
        self.refresh(113.1, moving=True)
        self.node.tick()
        self.assertTrue(self.node.report()['ready'])
        self.assertEqual(self.node.raw_pub.publish.call_args.args[0].linear.x, 0.)

    def test_driving_motion_and_optional_ultrasonic_echo_do_not_erase_calibration(self):
        self.refresh(100., moving=True)
        self.node.phase = 'ready'
        self.node.runtime_ready = True
        self.node.set_parameters([Parameter('calibration_require_us_agreement', value=False)])
        self.node.baseline.add('us', (math.inf,), 100., False)
        self.node.tick()
        self.assertTrue(self.node.report()['ready'])
        self.assertEqual(self.node.sensors['us']['status'], 'advisory')
        self.node.baseline.add('imu', (math.nan,), 100., False)
        self.node.tick()
        self.assertFalse(self.node.report()['ready'])
        self.assertTrue(self.node.report()['calibration_verified'])

    def test_optional_echo_cannot_bypass_ultrasonic_source_timestamp(self):
        self.refresh(100.)
        self.node.phase = 'ready'
        self.node.set_parameters([Parameter('calibration_require_us_agreement', value=False)])
        msg = Range()
        msg.max_range = 3.
        msg.range = 1.
        self.node.on_us(msg)  # Zero source stamp is stale despite a fresh callback.
        self.node.tick()
        self.assertFalse(self.node.report()['ready'])
        self.assertTrue(self.node.report()['calibration_verified'])

    def test_lidar_nose_uses_actual_tf_instead_of_legacy_parameter(self):
        self.refresh(100.)
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
        # A continuous wall around actual180; legacy190 has no usable plane.
        for index in range(350, 371):
            ranges[index] = .65 / math.cos((index-360)*scan.angle_increment)
        scan.ranges = ranges
        for i in range(3):
            self.now.return_value = 100. + i * .05
            self.node.on_scan(scan)
        self.assertAlmostEqual(self.node.lidar_nose, math.pi)
        self.assertEqual(self.node.wall_tracker.diagnostic['status'], 'ok')
        self.assertAlmostEqual(self.node.baseline.latest('lidar')[0], .65, places=5)

    def test_ready_scan_health_does_not_require_calibration_wall(self):
        transform = TransformStamped()
        transform.transform.rotation.w = 1.
        self.node.tf = Mock()
        self.node.tf.lookup_transform.return_value = transform
        self.node.phase = 'ready'
        scan = LaserScan()
        scan.header.frame_id = 'laser'
        scan.header.stamp = self.node.get_clock().now().to_msg()
        scan.angle_increment = math.pi / 360
        scan.range_max = 40.
        scan.ranges = [.4] + [math.inf] * 719
        self.node.on_scan(scan)
        self.assertAlmostEqual(self.node.baseline.latest('lidar')[0], .4)
