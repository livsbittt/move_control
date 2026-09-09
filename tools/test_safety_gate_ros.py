"""Local ROS safety-node tests. A dedicated domain never joins the robot."""
import unittest
import json
from unittest.mock import Mock, patch

import rclpy
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32MultiArray, String
from rosy_control.control.calibration_profile import make_profile
from rosy_control.safety.node import SafetyNode


class SafetyGateTest(unittest.TestCase):
    def test_escape_prediction_is_cached_and_never_reuses_stale_geometry(self):
        from rosy_control.control.rotation_envelope import RotationEnvelope
        from rosy_control.control.escape_space import escape_space_plan
        n = self.node
        n.set_parameters([Parameter('footprint_guard_enabled', value=True),
                          Parameter('lidar_use_tf', value=True)])
        n.release_estop(); n._refresh_distances(); n.refresh_profile()
        estimator = RotationEnvelope(.083)
        for yaw in (.17, -.17, .18, -.18):
            estimator.add((0., 0., yaw), (0., 0., yaw), yaw, .0005)
        n.calibration_lease.rotation_envelope = Mock(return_value=estimator.report())
        n.motion_limits_pub = Mock()
        self.fresh_sensors()
        n.lidar_mount = (-.017, 0.)
        n.translation_clearance = (.05, .1)
        n.lidar_measurement_time = n.now()
        n.lidar_rotation_points = [(0., .10), (.3, .2), (-.3, .2)]
        n.lidar_rotation_clearance = .10
        n.lidar_rotation_observed = False
        for field in ('lidar_front', 'lidar_rear', 'lidar_left', 'lidar_right',
                      'lidar_rear_left', 'lidar_rear_right'):
            setattr(n, field, .2)
        n.on_cmd(Twist())
        with patch('rosy_control.safety.node.escape_space_plan', wraps=escape_space_plan) as prediction:
            n.tick(); n.tick()
            self.assertEqual(prediction.call_count, 1)
        packet = json.loads(n.motion_limits_pub.publish.call_args.args[0].data)
        self.assertEqual(packet['execution_escape']['geometry_revision'], n.profile.revision)
        self.assertLessEqual(packet['execution_escape']['scan_age_s'], .2)
        self.assertEqual(n.pub.publish.call_args.args[0].linear.x, 0.)
        n.last_scan_time = None
        n.tick()
        packet = json.loads(n.motion_limits_pub.publish.call_args.args[0].data)
        self.assertIsNone(packet['execution_escape'])

    def test_translation_retry_retains_sweep_but_enforces_straight_domain(self):
        with patch.dict('os.environ',{'ROS_DOMAIN_ID':'227','GZ_PARTITION':'pinky_calmap227'}), \
                patch('rosy_control.safety.node.bounded_sweep_clearance',return_value=.01) as sweep:
            self.prepare_bounded_sweep()
            n=self.node
            packet=make_profile('bounded-test',n.calibration_lease.sequence+1,
                n.now().nanoseconds*1e-9,False,(1.25,1.25),n.profile.revision,translation_trial=True)
            n.on_calibration_profile(String(data=json.dumps(packet)))
            actual=self.bounded_command(.008,0.)
            self.assertAlmostEqual(actual.linear.x,n.cmd_linear_sign*.008)
            self.assertAlmostEqual(sweep.call_args.args[4],.008)
            self.assertTrue(n.calibration_lease.rotation_estimate_required())
            for v,w in ((.014001,0.),(-.014001,0.),(.008,1e-12),(.008,-.001)):
                actual=self.bounded_command(v,w)
                self.assertEqual((actual.linear.x,actual.angular.z),(0.,0.))
            actual=self.bounded_command(-.014,0.)
            self.assertAlmostEqual(actual.linear.x,-n.cmd_linear_sign*.014)
            n.calibration_lease.deadline=0.
            sweep.reset_mock()
            actual=self.bounded_command(.008,0.)
            self.assertEqual((actual.linear.x,actual.angular.z),(0.,0.))
            sweep.assert_not_called()

    def prepare_bounded_sweep(self, enabled=True, simulation=True, estimate=True, footprint=()):
        from rosy_control.control.rotation_envelope import RotationEnvelope
        n=self.node
        n.set_parameters([Parameter('simulation_motion_sweep_enabled',value=enabled),
                          Parameter('use_sim_time',value=simulation),
                          Parameter('rotation_footprint_xy',value=[float(v) for p in footprint for v in p])])
        n.release_estop(); n._refresh_distances(); n.refresh_profile()
        if estimate:
            estimator=RotationEnvelope(n.robot_r,footprint)
            for yaw in (.17,-.17,.18,-.18):
                estimator.add((0.,0.,yaw),(0.,0.,yaw),yaw,.0005)
            rotation=dict(done=True,error=None,max_angular_rad_s=.06,legs=[{}]*8,
                          angular_gains=[1.,1.],envelope=estimator.report())
            packet=make_profile('bounded-test',max(1,n.calibration_lease.sequence+1),n.now().nanoseconds*1e-9,
                                True,(1.25,1.25),n.profile.revision,rotation)
            n.on_calibration_profile(String(data=json.dumps(packet)))
            self.assertTrue(n.calibration_lease.live(__import__('time').monotonic()))
        self.fresh_sensors()
        n.lidar_measurement_time=n.now()
        for field in ('lidar_front','lidar_rear','lidar_left','lidar_right','lidar_rear_left','lidar_rear_right'):
            setattr(n,field,.4)
        n.lidar_rotation_points=[(.4,0.),(-.4,0.),(0.,.4),(0.,-.4)]
        n.lidar_rotation_clearance=.08
        n.lidar_rotation_observed=True

    def bounded_command(self,v=.014,w=.04):
        command=Twist(); command.linear.x=v; command.angular.z=w
        self.node.on_cmd(command); self.node.tick()
        return self.node.pub.publish.call_args.args[0]

    def test_trusted_polygon_checks_final_command_and_revokes_on_evidence_loss(self):
        radius=self.node.robot_r
        shape=[(x*.8*radius,y*.6*radius) for x,y in ((1,1),(-1,1),(-1,-1),(1,-1))]
        with patch.dict('os.environ',{'ROS_DOMAIN_ID':'227','GZ_PARTITION':'pinky_calmap227'}), \
                patch('rosy_control.safety.node.footprint_sweep_clearance',return_value=.02) as sweep:
            self.prepare_bounded_sweep(footprint=shape)
            actual=self.bounded_command(.008,0.)
            self.assertAlmostEqual(sweep.call_args.args[5],.010)
            self.assertGreater(actual.linear.x,0.)
            for result in (None,0.,-.001):
                sweep.return_value=result
                actual=self.bounded_command(.008,.04)
                self.assertEqual((actual.linear.x,actual.angular.z),(0.,0.))
            sweep.return_value=.02
            self.node.last_scan_time=None
            sweep.reset_mock()
            self.assertEqual(self.bounded_command(.008,.04).linear.x,0.)
            sweep.assert_not_called()
            self.fresh_sensors()
            self.node.calibration_lease.deadline=0.
            self.assertEqual(self.bounded_command(.008,.04).linear.x,0.)
            sweep.assert_not_called()

    def test_same_radius_different_footprint_cannot_authorize_polygon_motion(self):
        radius=self.node.robot_r
        shape=[(x*.8*radius,y*.6*radius) for x,y in ((1,1),(-1,1),(-1,-1),(1,-1))]
        with patch.dict('os.environ',{'ROS_DOMAIN_ID':'227','GZ_PARTITION':'pinky_calmap227'}):
            self.prepare_bounded_sweep(footprint=shape)
            self.node.set_parameters([Parameter('rotation_footprint_xy',value=[v for x,y in shape for v in (-y,x)])])
            actual=self.bounded_command(.008,.04)
            self.assertEqual((actual.linear.x,actual.angular.z),(0.,0.))

    def test_bounded_translation_is_not_vetoed_by_a_clear_side_return(self):
        with patch.dict('os.environ',{'ROS_DOMAIN_ID':'227','GZ_PARTITION':'pinky_calmap227'}):
            self.prepare_bounded_sweep()
            n=self.node
            n.lidar_front=(.04**2+.1**2)**.5
            n.lidar_rotation_points=[(.04,.1),(.4,0.),(-.4,0.)]
            actual=self.bounded_command(.008,0.)
            self.assertGreater(actual.linear.x,0.)
            self.assertFalse(n.block_pub.publish.call_args.args[0].data)

    def test_bounded_translation_keeps_current_collision_and_ultrasound_stops(self):
        with patch.dict('os.environ',{'ROS_DOMAIN_ID':'227','GZ_PARTITION':'pinky_calmap227'}):
            self.prepare_bounded_sweep()
            n=self.node
            n.lidar_rotation_points=[(.08,0.),(.4,0.),(-.4,0.)]
            self.assertEqual(self.bounded_command(.008,0.).linear.x,0.)
            self.prepare_bounded_sweep()
            n.us_blocked=True
            n.us_distance=Mock(return_value=.01)
            self.assertEqual(self.bounded_command(.008,0.).linear.x,0.)

    def test_bounded_sweep_requires_opt_in_and_simulation(self):
        with patch.dict('os.environ',{'ROS_DOMAIN_ID':'227','GZ_PARTITION':'pinky_calmap227'}), \
                patch('rosy_control.safety.node.bounded_sweep_clearance',return_value=.01) as sweep:
            self.prepare_bounded_sweep(enabled=False)
            self.node.lidar_rotation_points=[(.08,0.)]
            actual=self.bounded_command()
            self.assertEqual((actual.linear.x,actual.angular.z),(0.,0.))
            sweep.assert_not_called()
            self.node.set_parameters([Parameter('simulation_motion_sweep_enabled',value=True),
                                      Parameter('use_sim_time',value=False)])
            self.fresh_sensors()
            actual=self.bounded_command()
            self.assertEqual((actual.linear.x,actual.angular.z),(0.,0.))
            sweep.assert_not_called()

    def test_first_calibration_retains_existing_straight_gate_before_estimate(self):
        with patch.dict('os.environ',{'ROS_DOMAIN_ID':'227','GZ_PARTITION':'pinky_calmap227'}), \
                patch('rosy_control.safety.node.bounded_sweep_clearance',return_value=.01) as sweep:
            self.prepare_bounded_sweep(estimate=False)
            actual=self.bounded_command(.008,0.)
            self.assertGreater(actual.linear.x,0.)
            sweep.assert_not_called()

    def test_bounded_sweep_checks_corrected_limited_command_and_stops_whole_arc(self):
        with patch.dict('os.environ',{'ROS_DOMAIN_ID':'227','GZ_PARTITION':'pinky_calmap227'}), \
                patch('rosy_control.safety.node.bounded_sweep_clearance',return_value=.01) as sweep:
            self.prepare_bounded_sweep()
            actual=self.bounded_command(.008,0.)
            self.assertAlmostEqual(sweep.call_args.args[4],.010)
            self.assertAlmostEqual(sweep.call_args.args[5],0.)
            self.assertAlmostEqual(actual.linear.x,self.node.cmd_linear_sign*.010)
            actual=self.bounded_command(.028,.2)
            self.assertAlmostEqual(sweep.call_args.args[4],.014)
            self.assertAlmostEqual(sweep.call_args.args[5],.1)
            self.assertAlmostEqual(actual.linear.x,self.node.cmd_linear_sign*.014)
            self.assertAlmostEqual(actual.angular.z,.1)
            sweep.return_value=0.
            actual=self.bounded_command()
            self.assertEqual((actual.linear.x,actual.angular.z),(0.,0.))

    def test_bounded_sweep_rejects_missing_estimate_and_incomplete_or_stale_scan(self):
        with patch.dict('os.environ',{'ROS_DOMAIN_ID':'227','GZ_PARTITION':'pinky_calmap227'}), \
                patch('rosy_control.safety.node.bounded_sweep_clearance',return_value=.01) as sweep:
            self.prepare_bounded_sweep()
            self.node.calibration_lease.deadline=0.
            actual=self.bounded_command()
            self.assertEqual((actual.linear.x,actual.angular.z),(0.,0.))
            sweep.assert_not_called()
            self.prepare_bounded_sweep()
            self.node.lidar_rotation_observed=False
            actual=self.bounded_command()
            self.assertEqual((actual.linear.x,actual.angular.z),(0.,0.))
            sweep.assert_not_called()
            self.node.lidar_rotation_observed=True
            original_age=self.node.age
            old_source=object()
            self.node.lidar_measurement_time=old_source
            with patch.object(self.node,'age',side_effect=lambda stamp: .3 if stamp is old_source else original_age(stamp)):
                actual=self.bounded_command()
                self.assertEqual((actual.linear.x,actual.angular.z),(0.,0.))
            sweep.assert_not_called()
            self.node.lidar_measurement_time=self.node.now()
            self.node.last_scan_time=None
            actual=self.bounded_command()
            self.assertEqual((actual.linear.x,actual.angular.z),(0.,0.))
            sweep.assert_not_called()

    def test_full_pivot_shape_allows_pure_spin_despite_circle_approximation(self):
        with patch.dict('os.environ',{'ROS_DOMAIN_ID':'227','GZ_PARTITION':'pinky_calmap227'}), \
                patch('rosy_control.safety.node.bounded_sweep_clearance',return_value=0.), \
                patch('rosy_control.safety.node.pivot_clearance',return_value=.03):
            self.prepare_bounded_sweep()
            actual=self.bounded_command(0.,.04)
            self.assertEqual(actual.linear.x,0.)
            self.assertGreater(actual.angular.z,0.)

    def test_pure_spin_still_requires_latency_padded_pivot_clearance(self):
        with patch.dict('os.environ',{'ROS_DOMAIN_ID':'227','GZ_PARTITION':'pinky_calmap227'}), \
                patch('rosy_control.safety.node.bounded_sweep_clearance',return_value=0.) as sweep, \
                patch('rosy_control.safety.node.pivot_clearance',return_value=.011):
            self.prepare_bounded_sweep()
            actual=self.bounded_command(0.,.04)
            self.assertEqual((actual.linear.x,actual.angular.z),(0.,0.))
            sweep.assert_called()

    def test_tiny_final_translation_cannot_use_full_spin_shortcut(self):
        with patch.dict('os.environ',{'ROS_DOMAIN_ID':'227','GZ_PARTITION':'pinky_calmap227'}), \
                patch('rosy_control.safety.node.bounded_sweep_clearance',return_value=0.) as sweep, \
                patch('rosy_control.safety.node.pivot_clearance',return_value=.03):
            self.prepare_bounded_sweep()
            actual=self.bounded_command(1e-7,.04)
            self.assertEqual((actual.linear.x,actual.angular.z),(0.,0.))
            self.assertGreater(sweep.call_args.args[4],0.)

    def test_full_spin_shortcut_never_authorized_by_stale_or_partial_scan(self):
        with patch.dict('os.environ',{'ROS_DOMAIN_ID':'227','GZ_PARTITION':'pinky_calmap227'}), \
                patch('rosy_control.safety.node.bounded_sweep_clearance',return_value=0.), \
                patch('rosy_control.safety.node.pivot_clearance',return_value=.03):
            self.prepare_bounded_sweep()
            self.node.lidar_rotation_observed=False
            actual=self.bounded_command(0.,.04)
            self.assertEqual((actual.linear.x,actual.angular.z),(0.,0.))
            self.node.lidar_rotation_observed=True
            self.node.lidar_measurement_time=None
            actual=self.bounded_command(0.,.04)
            self.assertEqual((actual.linear.x,actual.angular.z),(0.,0.))

    def test_full_spin_shortcut_stays_within_measured_angular_domain(self):
        with patch.dict('os.environ',{'ROS_DOMAIN_ID':'227','GZ_PARTITION':'pinky_calmap227'}), \
                patch('rosy_control.safety.node.bounded_sweep_clearance',return_value=None) as sweep, \
                patch('rosy_control.safety.node.pivot_clearance',return_value=.03):
            self.prepare_bounded_sweep()
            n=self.node
            # Inject a wider downstream cap to ensure this optimization has
            # its own measured-domain bound even if profile rules change.
            n.profile.max_angular=.2
            rotation=n.calibration_lease.active['rotation']
            with patch.object(n,'refresh_profile'):
                packet=make_profile('bounded-test',n.calibration_lease.sequence+1,
                    n.now().nanoseconds*1e-9,True,(1.,1.),n.profile.revision,rotation)
                n.on_calibration_profile(String(data=json.dumps(packet)))
                self.assertTrue(n.calibration_lease.live(__import__('time').monotonic()))
                actual=self.bounded_command(0.,.2)
            self.assertEqual((actual.linear.x,actual.angular.z),(0.,0.))
            self.assertAlmostEqual(sweep.call_args.args[5],.2)

    def test_recalibration_retains_sweep_restriction_without_restoring_gains(self):
        from types import SimpleNamespace
        from rosy_control.calibration_atomic import CalibrationAtomic
        from rosy_control.control.rotation_envelope import RotationEnvelope
        n = self.node
        n.release_estop(); n._refresh_distances(); n.refresh_profile()
        envelope = RotationEnvelope(n.robot_r)
        for yaw in (.17,-.17,.18,-.18):
            envelope.add((0.,0.,yaw),(0.,0.,yaw),yaw,.0005)
        rotation = dict(done=True,error=None,max_angular_rad_s=.06,legs=[{}]*8,
                        angular_gains=[.9,.9],envelope=envelope.report())
        calibration = SimpleNamespace(phase='ready', runtime_ready=True,
            geometry_fresh=lambda now: True, round_trip=None, profile_session='retry-lifecycle',
            profile_sequence=0, get_clock=n.get_clock, trial_geometry_revision=n.profile.revision,
            geometry_revision=n.profile.revision, rotation_report=lambda:rotation)
        def phase(value):
            calibration.phase = value
            packet = CalibrationAtomic.profile_packet(calibration)
            calibration.profile_sequence += 1
            n.on_calibration_profile(String(data=json.dumps(packet)))
        def turn(speed=.04, clearance=.4):
            self.fresh_sensors()
            for field in ('lidar_front','lidar_rear','lidar_left','lidar_right',
                          'lidar_rear_left','lidar_rear_right'):
                setattr(n,field,.4)
            n.lidar_rotation_clearance=clearance
            command=Twist(); command.angular.z=speed
            n.on_cmd(command); n.tick()
            return n.pub.publish.call_args.args[0].angular.z
        n.lidar_rotation_points=[(.4,0.)]; n.lidar_rotation_observed=True
        phase('ready')
        self.assertGreater(turn(),0.)
        phase('collecting')
        self.assertEqual(turn(),0.)
        phase('validating_motion')
        self.assertEqual(turn(),0.)
        phase('validating_rotation')
        self.assertAlmostEqual(turn(),.04)
        self.assertEqual(n.calibration_lease.angular_gains(__import__('time').monotonic()),(1.,1.))
        self.assertEqual(turn(.07),0.)
        command=Twist(); command.linear.x=.01
        n.on_cmd(command); n.tick()
        self.assertEqual(n.pub.publish.call_args.args[0].linear.x,0.)
        n.lidar_rotation_points=[(.08,0.)]
        self.assertEqual(turn(),0.)
        n.lidar_rotation_points=[(.4,0.)]; n.lidar_rotation_observed=False
        # Partial angular coverage retains the conservative full swept radius;
        # it cannot use the tighter exact-pivot optimization.
        self.assertEqual(turn(), .04)
        self.assertEqual(turn(clearance=rotation['envelope']['required_radius_m']+.009), 0.)
        n.lidar_rotation_observed=True
        n.calibration_lease.deadline=0.
        self.assertEqual(turn(),0.)

    def test_learned_rotation_restriction_survives_lease_loss_and_missing_scan(self):
        import math
        from rosy_control.control.rotation_envelope import RotationEnvelope
        n = self.node
        n.release_estop()
        n._refresh_distances()
        n.refresh_profile()
        estimator = RotationEnvelope(n.robot_r)
        for yaw in (.17, -.17, .18, -.18):
            delta = ((1-math.cos(yaw))*.04, -math.sin(yaw)*.04, yaw)
            self.assertTrue(estimator.add(delta, delta, yaw, .0005))
        rotation = dict(done=True, error=None, max_angular_rad_s=.06, legs=[{}]*8,
                        angular_gains=[1.,1.], envelope=estimator.report())
        def apply(sequence):
            packet = make_profile('pivot-test', sequence, n.now().nanoseconds*1e-9,
                                  True, (1.,1.), n.profile.revision, rotation)
            n.on_calibration_profile(String(data=json.dumps(packet)))
        def turn():
            self.fresh_sensors()
            for field in ('lidar_front','lidar_rear','lidar_left','lidar_right',
                          'lidar_rear_left','lidar_rear_right'):
                setattr(n, field, .3)
            n.lidar_rotation_clearance = .13
            command = Twist()
            command.angular.z = .04
            n.on_cmd(command)
            n.tick()
            return n.pub.publish.call_args.args[0].angular.z
        n.lidar_rotation_points = [(.13,0.)]
        n.lidar_rotation_observed = True
        apply(1)
        self.assertEqual(turn(), 0.)
        n.calibration_lease.deadline = 0.
        self.assertEqual(turn(), 0.)
        apply(2)
        n.on_calibration_profile(String(data='{}'))
        self.assertEqual(turn(), 0.)
        apply(3)
        n.lidar_rotation_points = None
        self.assertEqual(turn(), 0.)
        n.lidar_rotation_points = [(.3,0.)]
        apply(4)
        self.assertGreater(turn(), 0.)

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
