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
    def test_recalibration_retains_sweep_restriction_without_restoring_gains(self):
        from types import SimpleNamespace
        from move_control.calibration_atomic import CalibrationAtomic
        from move_control.control.rotation_envelope import RotationEnvelope
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
        def turn(speed=.04):
            self.fresh_sensors()
            for field in ('lidar_front','lidar_rear','lidar_left','lidar_right',
                          'lidar_rear_left','lidar_rear_right'):
                setattr(n,field,.4)
            n.lidar_rotation_clearance=.4
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
        self.assertEqual(turn(),0.)
        n.lidar_rotation_observed=True
        n.calibration_lease.deadline=0.
        self.assertEqual(turn(),0.)

    def test_learned_rotation_restriction_survives_lease_loss_and_missing_scan(self):
        import math
        from move_control.control.rotation_envelope import RotationEnvelope
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
