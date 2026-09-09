"""ROS adapter relocation lifecycle; no physical robot graph."""
import math
import os
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
import rclpy
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from rosy_control.startup_calibration_node import StartupCalibrationNode


class RelocationAdapterTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls): rclpy.init(domain_id=223)
    @classmethod
    def tearDownClass(cls): rclpy.shutdown()
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.node = StartupCalibrationNode(parameter_overrides=[
            Parameter('result_path', value=str(Path(self.folder.name)/'cal.json'))])
        self.node.raw_pub = Mock()
        self.node.publish = Mock()
    def tearDown(self):
        self.node.destroy_node()
        self.folder.cleanup()

    def test_actual_scan_callback_preserves_full_polygon_separate_from_registration(self):
        n = self.node
        n.rotation_mount = (0.,0.,0.)
        scan = LaserScan()
        scan.header.stamp = n.get_clock().now().to_msg()
        scan.angle_min = -math.pi
        scan.angle_increment = math.tau/720
        scan.ranges = [.8]*720
        with patch('rosy_control.calibration_rotation.time.monotonic', return_value=100):
            n.rotation_scan_sample(scan, True)
        self.assertEqual(len(n.rotation_points), 180)
        self.assertEqual(len(n.relocation_points), 720)
        n.phase = 'relocating_calibration'
        n.relocation_wait = 99.
        n.rotation_eligibility = Mock(return_value=None)
        n.wander_state = ('stop',100.)
        n.requested = 99.
        n.geometry_profile = {'effective': {'radius':.1144, 'turn_clear':.14}}
        n.baseline.add('odom',(0.,0.,0.,0.),100.,True)
        n.round_trip = SimpleNamespace(done=True)
        n.relocation = Mock()
        n.relocation.update.return_value = (0.,'confirming_rotation_clearance')
        n.relocation.report.return_value = {'done':False,'error':None}
        n.tick_relocation(100.)
        self.assertTrue(n.relocation.update.call_args.kwargs['full_scan_observed'])
        self.assertEqual(n.phase, 'relocating_calibration')
        n.rotation_scan_sample(scan, False)
        self.assertIsNone(n.relocation_points)

    def _prepare_relocation(self):
        n = self.node
        n.get_clock = Mock(return_value=SimpleNamespace(now=lambda: Time(seconds=1000)))
        n.phase = 'relocating_calibration'
        n.relocation_wait = n.return_wait = 99.
        n.requested = 99.
        n.wander_state = ('stop',100.)
        n.geometry_profile = {'effective': {'radius':.1144,'turn_clear':.14}}
        n.rotation_mount = (0.,0.,0.)
        n.round_trip = SimpleNamespace(done=True)
        n.finish = Mock()
        n.relocation = Mock()
        n.relocation.update.return_value = (0.,'confirming_rotation_clearance')
        n.relocation.report.return_value = {'done':False,'error':None}
        n.baseline.add('odom',(0.,0.,0.,0.),100.,True)
        return n

    def test_disabled_rotation_or_round_trip_blocks_bootstrap_and_return_relocation(self):
        n = self.node
        n.set_parameters([Parameter('calibration_relocation_enabled',value=True),
                          Parameter('use_sim_time',value=True)])
        n.after_relocation = 'return_origin'
        n.round_trip = SimpleNamespace(done=True)
        n.rotation_eligibility = Mock(return_value=None)
        n.baseline.add('odom',(0.,0.,0.,0.),100.,True)
        with patch.dict(os.environ, {'ROS_DOMAIN_ID':'227','GZ_PARTITION':'pinky_calmap227'}):
            for rotation,round_trip in ((False,True),(True,False)):
                with self.subTest(rotation=rotation,round_trip=round_trip):
                    n.set_parameters([Parameter('calibration_rotation',value=rotation),
                                      Parameter('calibration_round_trip',value=round_trip)])
                    self.assertFalse(n.relocation_enabled())
                    self.assertFalse(n.begin_relocation(100.,before_translation=True))
                    self.assertFalse(n.begin_relocation(100.))
                    self.assertIsNone(n.relocation)

    def test_fresh_receipt_does_not_authorize_stale_source_odometry_or_imu(self):
        n = self._prepare_relocation()
        n.trial_gate_reason = Mock(return_value=None)
        n.estop = False
        n.hazards = {key:(100.,False) for key in
                     ('/safety/blocked','/safety/cliff','/safety/tilt','/safety/pickup')}
        for phase in ('relocating_calibration','returning_calibration'):
            for name in ('odom','imu'):
                with self.subTest(phase=phase,sensor=name):
                    n.phase = phase
                    n.relocation_sensor_deadlines = {'odom':100.2, 'imu':100.2}
                    for stream in ('odom','imu','ir','tf'):
                        n.baseline.add(stream,(0.,0.,0.,0.),100.,True)
                    msg = Odometry() if name=='odom' else Imu()
                    msg.header.stamp = Time(seconds=999.1).to_msg()
                    if name=='odom':
                        msg.pose.pose.orientation.w = 1.
                    else:
                        msg.orientation.w = 1.
                        msg.linear_acceleration.z = 9.81
                    with patch('rosy_control.startup_calibration_node.time.monotonic',return_value=100.):
                        (n.on_odom if name=='odom' else n.on_imu)(msg)
                    self.assertEqual(n.baseline.samples[name][-1][0],100.)
                    self.assertFalse(n.baseline.samples[name][-1][2])
                    self.assertIn(name,n.rotation_eligibility(100.,allow_front_blocked=True))
                    n.finish.reset_mock()
                    n.return_motion = Mock()
                    (n.tick_relocation if phase=='relocating_calibration' else n.tick_return)(100.)
                    self.assertFalse(n.finish.call_args.args[0])
                    n.relocation.update.assert_not_called()
                    n.return_motion.update.assert_not_called()

    def test_full_scan_with_fresh_receipt_but_stale_source_cannot_authorize(self):
        n = self._prepare_relocation()
        n.rotation_eligibility = Mock(return_value=None)
        scan = LaserScan()
        scan.header.stamp = Time(seconds=999.5).to_msg()
        scan.angle_min = -math.pi
        scan.angle_increment = math.tau/720
        scan.ranges = [.8]*720
        with patch('rosy_control.calibration_rotation.time.monotonic',return_value=100.):
            n.rotation_scan_sample(scan,True)
        self.assertEqual(n.rotation_scan[0],100.)
        self.assertEqual(len(n.relocation_points),720)
        self.assertIsNone(n.relocation_scan_deadline)
        n.tick_relocation(100.)
        self.assertFalse(n.relocation.update.call_args.kwargs['full_scan_observed'])
        n.phase = 'returning_calibration'
        n.return_motion = Mock()
        n.return_motion.update.return_value = (0.,'observation_unavailable')
        n.return_motion.report.return_value = {'done':False,'error':'observation_unavailable'}
        n.rotation_report = Mock(return_value={'done':True})
        n.tick_return(100.)
        self.assertFalse(n.return_motion.update.call_args.kwargs['full_scan_observed'])
        self.assertFalse(n.finish.call_args.args[0])

    def test_bootstrap_requires_translation_clearance_as_well_as_rotation_clearance(self):
        n = self._prepare_relocation()
        n.rotation_eligibility = Mock(return_value=None)
        n.relocation_before_translation = True
        n.round_trip = None
        n.safety_limits = (100.,{'front_stop_m':.14,'rear_stop_m':.14,'us_stop_m':.02})
        n.rotation_clear = Mock(return_value=True)
        n.safe_motion = Mock(return_value='Insufficient translation clearance')
        n.rotation_scan = (100., [.8]*720, math.tau/720)
        n.relocation_points = [(0.,0.)]*720
        n.relocation_scan_deadline = 100.2
        n.tick_relocation(100.)
        self.assertFalse(n.relocation.update.call_args.kwargs['rotation_clear_current'])
        self.assertEqual(n.phase,'relocating_calibration')
        n.finish.assert_not_called()
        n.safe_motion.return_value = None
        n.tick_relocation(100.)
        self.assertTrue(n.relocation.update.call_args.kwargs['rotation_clear_current'])

    def test_bootstrap_without_scan_never_reports_clear_or_raises(self):
        n = self._prepare_relocation()
        n.rotation_eligibility = Mock(return_value=None)
        n.relocation_before_translation = True
        n.safety_limits = (100.,{'front_stop_m':.14,'rear_stop_m':.14,'us_stop_m':.02})
        n.rotation_clear = Mock(return_value=True)
        n.safe_motion = Mock(return_value=None)
        n.tick_relocation(100.)
        self.assertFalse(n.relocation.update.call_args.kwargs['full_scan_observed'])
        self.assertFalse(n.relocation.update.call_args.kwargs['rotation_clear_current'])

    def test_station_reserves_rotation_center_travel_before_stopping(self):
        n = self._prepare_relocation()
        n.rotation_eligibility = Mock(return_value=None)
        n.rotation_clear = Mock(return_value=True)
        n.relocation_points = [(0.,0.)]*720
        n.relocation_scan_deadline = 100.2
        n.rotation_scan = (100.,[.195]*720,math.tau/720)
        n.tick_relocation(100.)
        self.assertFalse(n.relocation.update.call_args.kwargs['rotation_clear_current'])
        n.rotation_scan = (100.,[.202]*720,math.tau/720)
        n.tick_relocation(100.)
        self.assertTrue(n.relocation.update.call_args.kwargs['rotation_clear_current'])

    def test_source_age_consumes_remaining_receipt_freshness_budget(self):
        n = self._prepare_relocation()
        msg = Odometry()
        msg.header.stamp = Time(seconds=999.85).to_msg()
        with patch('rosy_control.startup_calibration_node.time.monotonic',return_value=100.):
            self.assertAlmostEqual(n.relocation_source_deadline(msg),100.05)
        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.angle_min = -math.pi
        scan.angle_increment = math.tau/720
        scan.ranges = [.8]*720
        with patch('rosy_control.calibration_rotation.time.monotonic',return_value=100.):
            n.rotation_scan_sample(scan,True)
        n.rotation_eligibility = Mock(return_value=None)
        n.tick_relocation(100.1)
        self.assertFalse(n.relocation.update.call_args.kwargs['full_scan_observed'])

    def test_relocation_requires_verified_translation_and_is_single_attempt(self):
        n = self.node
        n.relocation_enabled = Mock(return_value=True)
        n.rotation_eligibility = Mock(return_value=None)
        n.baseline.add('odom',(0.,0.,0.,0.),100.,True)
        self.assertFalse(n.begin_relocation(100.))
        n.round_trip = SimpleNamespace(done=False)
        self.assertFalse(n.begin_relocation(100.))
        n.round_trip.done = True
        self.assertTrue(n.begin_relocation(100.))
        self.assertFalse(n.begin_relocation(101.))

    def test_blocked_front_can_enter_observed_reverse_relocation_but_other_guards_cannot(self):
        n = self.node
        n.wander_state = ('stop',100.)
        n.requested = 99.
        n.rotation_eligibility = Mock(side_effect=lambda now, allow_front_blocked=False:
                                      None if allow_front_blocked else 'front blocked')
        n.rotation_clear = Mock(return_value=False)
        n.begin_relocation = Mock(return_value=True)
        n.tick_rotation(100.)
        n.begin_relocation.assert_called_once()
        n.begin_relocation.reset_mock()
        n.rotation_eligibility = Mock(return_value='stale imu')
        n.finish = Mock()
        n.tick_rotation(100.)
        n.begin_relocation.assert_not_called()
        n.finish.assert_called_once_with(False, 'stale imu')
