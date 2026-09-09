import time
import json
import unittest
from unittest.mock import Mock

import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, String
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Path
from rosy_control.wander.node import WanderNode


class CalibrationGateTest(unittest.TestCase):
    def test_bounded_escape_uses_fresh_directional_limits_without_renewing_session(self):
        from unittest.mock import patch
        self.node.navigation_session.start(dict(strategy='gain',duration_s=30,stall_s=20),100.)
        self.node.navigation_recovery.waiting=True
        self.node.navigation_recovery.failed_exit=(.1,0.)
        self.node.motion_limits=dict(translation_mode=True,reverse_travel_m=.1,
            geometry_revision='test',can_rotate=False,rotation_scan_observed=False,
            execution_escape=dict(geometry_revision='test',issued_s=100.,scan_age_s=.05,
                rotation_restored=False,candidates=[dict(direction=-1,target_m=.023,
                    available_m=.1,predicted_rotation_clearance_m=.015)]))
        self.node.odom_x=self.node.odom_y=self.node.odom_yaw=0.
        self.node.tilt=self.node.cliff=False
        self.node._odom_fresh=Mock(return_value=True)
        self.node._motion_limits_fresh=Mock(return_value=True)
        self.node._can_reverse=Mock(return_value=True)
        self.node._obstacle_wait=Mock(return_value=None)
        for now, expected in ((100.,-.005),(105.,-.005)):
            self.node.motion_limits_received=self.node.odom_received=now
            self.node.odom_stamp_ns=int(now*1e9)
            self.node.motion_limits['execution_escape']['issued_s']=now
            with patch('rosy_control.wander.navigator.time.monotonic',return_value=now), \
                    patch.object(self.node,'_session_now',return_value=now):
                velocity,reason=self.node._execution_escape_step(now,(0.,0.,0.),True)
            self.assertEqual(velocity,expected)
        self.assertEqual(self.node.navigation_session.started,100.)
        self.assertEqual(self.node.navigation_session.progress_at,100.)
        with patch('rosy_control.wander.navigator.time.monotonic',return_value=106.), \
                patch.object(self.node,'_session_now',return_value=106.):
            velocity,reason=self.node._execution_escape_step(106.,(0.,0.,0.),True)
        self.assertEqual((velocity,reason),(0.,'execution_escape_stopped'))

    def test_session_budget_stops_raw_and_planner_without_browser(self):
        from unittest.mock import patch
        self.node.on_calibration(Bool(data=True))
        self.node.estop = False
        self.node._set_enabled(False)
        self.node.navigation_goal_pub = Mock()
        self.node.pub = Mock()
        payload = 'session:' + json.dumps(dict(strategy='nearest', duration_s=10, stall_s=10))
        with patch('rosy_control.wander.node.time.monotonic', return_value=100), patch.object(self.node, '_session_now', return_value=100):
            self.node.calibration_received = 100
            self.node.on_cmd(String(data=payload))
        self.assertTrue(self.node.navigation_session.active)
        self.assertEqual(self.node.navigation_goal_pub.publish.call_args.args[0].data, 'explore_nearest')
        self.node.on_cmd(String(data='coverage'))
        self.assertEqual(self.node.navigation_session.options['strategy'], 'nearest')
        with patch('rosy_control.wander.node.time.monotonic', return_value=110), patch.object(self.node, '_session_now', return_value=110):
            self.node.calibration_received = 110
            self.node._tick_session()
        self.assertFalse(self.node.enabled)
        self.assertEqual(self.node.navigation_session.reason, 'time_limit')
        self.assertEqual(self.node.navigation_goal_pub.publish.call_args.args[0].data, 'stop')
        output = self.node.pub.publish.call_args.args[0]
        self.assertEqual((output.linear.x, output.angular.z), (0., 0.))

    def test_arrival_retries_without_flooding_and_new_goal_rearms(self):
        self.node.on_calibration(Bool(data=True))
        self.node.estop = False
        self.node.on_cmd(String(data='explore'))
        self.node._ir_ready = Mock(return_value=True)
        self.node._on_wall = Mock(return_value=False)
        self.node._can_reverse = Mock(return_value=True)
        self.node.blocked = self.node.cliff = self.node.tilt = self.node.pickup = False
        self.node.navigation_arrival_pub = Mock()
        self.node.navigation_goal_pub = Mock()
        self.node.navigation_tf = Mock()

        def arrive(x):
            stamp = self.node.now().to_msg()
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.transform.translation.x = x
            tf.transform.rotation.w = 1.
            self.node.navigation_tf.lookup_transform.return_value = tf
            route = Path()
            route.header.frame_id = 'map'
            route.header.stamp = stamp
            point = PoseStamped()
            point.pose.position.x = x
            route.poses = [point]
            self.node._on_navigation_route(route)
            self.node.navigation_progress.stalled = True
            self.node._tick_navigation()
            return stamp.sec * 1_000_000_000 + stamp.nanosec

        stamp_ns = arrive(.2)
        self.assertFalse(self.node.navigation_progress.stalled)
        first = json.loads(self.node.navigation_arrival_pub.publish.call_args.args[0].data)
        self.assertEqual(first, {'route_stamp_ns': stamp_ns, 'target': [.2, 0.],
                                 'pose': [.2, 0.]})
        arrive(.2)
        self.assertEqual(self.node.navigation_arrival_pub.publish.call_count, 1)
        self.node.navigation_goal_pub.publish.assert_not_called()
        self.node.navigation_arrival_sent_at -= .5
        retry_stamp = arrive(.2)
        self.assertEqual(self.node.navigation_arrival_pub.publish.call_count, 2)
        retry = json.loads(self.node.navigation_arrival_pub.publish.call_args.args[0].data)
        self.assertEqual(retry['route_stamp_ns'], retry_stamp)
        arrive(.3)
        self.assertEqual(self.node.navigation_arrival_pub.publish.call_count, 3)
        cmd = self.node.pub.publish.call_args.args[0]
        self.assertEqual((cmd.linear.x, cmd.angular.z), (0., 0.))
        self.node.estop = True
        arrive(.4)
        self.assertEqual(self.node.navigation_arrival_pub.publish.call_count, 3)

    def test_live_safety_limits_replace_old_wall_band_and_missing_values_do_not(self):
        self.node.front_range = .15
        self.node.us_range = .08
        self.node.blocked = False
        self.node.set_parameters([Parameter('wall_front', value=.14)])
        self.node.on_motion_limits(String(data='{"front_m":0.15,"rear_m":0.12,"front_stop_m":0.07,"rear_stop_m":0.07}'))
        self.assertFalse(self.node._on_wall())
        self.node.blocked = True
        self.assertTrue(self.node._on_wall())
        self.node.blocked = False
        self.node.on_motion_limits(String(data='{}'))
        self.assertFalse(self.node._motion_limits_fresh())
        self.assertTrue(self.node._on_wall())

    @classmethod
    def setUpClass(cls):
        rclpy.init(domain_id=222)

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = WanderNode()
        self.node.pub = Mock()
        self.node.set_parameters([Parameter('calibration_required', value=True)])

    def tearDown(self):
        self.node.destroy_node()

    def test_start_and_map_modes_cannot_bypass_initial_calibration(self):
        for command in ('start', 'explore', 'coverage', 'manual:0.2,0.1'):
            self.node.on_cmd(String(data=command))
            self.assertFalse(self.node.enabled)
        self.node.on_enable(Bool(data=True))
        self.assertFalse(self.node.enabled)

    def test_lost_ready_heartbeat_stops_active_autonomy(self):
        self.node.on_calibration(Bool(data=True))
        self.node.on_cmd(String(data='start'))
        self.assertTrue(self.node.enabled)
        self.node.calibration_received = time.monotonic() - 4
        self.node.tick()
        self.assertFalse(self.node.enabled)
        self.assertEqual(self.node.stop_reason, 'calibration_required')

    def test_ready_does_not_start_a_mode_until_a_command_arrives(self):
        self.node.on_calibration(Bool(data=True))
        self.node.tick()
        self.assertFalse(self.node.enabled)
        self.assertIsNone(self.node.navigation_mode)
        self.node.on_cmd(String(data='coverage'))
        self.assertTrue(self.node.enabled)
        self.assertEqual(self.node.navigation_mode, 'coverage')
        self.node.on_cmd(String(data='stop'))
        self.assertFalse(self.node.enabled)

    def test_manual_start_orders_cancel_before_coordinates_without_explore(self):
        self.node.on_calibration(Bool(data=True))
        self.node.estop = False
        self.node.navigation_mode = 'coverage'
        self.node.navigation_goal_pub = Mock()
        self.node.on_cmd(String(data='manual:0.2,0.1'))
        self.assertTrue(self.node.enabled)
        self.assertEqual(self.node.navigation_mode, 'manual')
        self.assertEqual([c.args[0].data for c in
                          self.node.navigation_goal_pub.publish.call_args_list],
                         ['stop', '0.200,0.100'])

    def test_manual_start_rejected_while_estopped(self):
        self.node.on_calibration(Bool(data=True))
        self.node.estop = True
        self.node.on_cmd(String(data='manual:0.2,0.1'))
        self.assertFalse(self.node.enabled)

    def test_manual_terminal_rejects_old_transaction_then_stops_current(self):
        self.node.on_calibration(Bool(data=True))
        self.node.estop = False
        self.node.on_cmd(String(data='manual:0.2,0.1'))
        result = {'started_ns': self.node.navigation_started_ns - 1,
                  'target': [.2, .1], 'status': 'manual goal reached'}
        self.node._on_manual_result(String(data=json.dumps(result)))
        self.assertTrue(self.node.enabled)
        result['started_ns'] = self.node.navigation_started_ns
        result['target'] = [.3, .1]
        self.node._on_manual_result(String(data=json.dumps(result)))
        self.assertTrue(self.node.enabled)
        result['target'] = [.2, .1]
        self.node._on_manual_result(String(data=json.dumps(result)))
        self.assertFalse(self.node.enabled)
        self.assertIsNone(self.node.navigation_mode)

    def test_front_wall_allows_route_alignment_but_estop_still_halts_every_axis(self):
        self.node.on_calibration(Bool(data=True))
        self.node.on_cmd(String(data='explore'))
        self.node._ir_ready = Mock(return_value=True)
        self.node._on_wall = Mock(return_value=True)
        self.node._can_reverse = Mock(return_value=False)
        self.node.blocked = True
        self.node.cam_block = self.node.cliff = self.node.tilt = self.node.pickup = False
        self.node.estop = False
        t = TransformStamped()
        t.header.stamp = self.node.now().to_msg()
        t.transform.rotation.w = 1.
        self.node.navigation_tf = Mock()
        self.node.navigation_tf.lookup_transform.return_value = t
        self.node.navigation_route = [(0.,0.),(-.3,0.)]
        self.node.navigation_received = self.node.navigation_stamp = self.node.now().nanoseconds*1e-9
        self.node._tick_navigation()
        cmd = self.node.pub.publish.call_args.args[0]
        self.assertEqual(cmd.linear.x,0.)
        self.assertNotEqual(cmd.angular.z,0.)
        self.node.estop = True
        self.node._tick_navigation()
        cmd = self.node.pub.publish.call_args.args[0]
        self.assertEqual((cmd.linear.x,cmd.angular.z),(0.,0.))

    def test_camera_observation_cannot_override_lidar_turn_or_forward_route(self):
        self.node.cam_block = True
        self.node.cam_side = -1.
        self.node.left_range, self.node.right_range = .4, .1
        self.assertEqual(self.node._pick_turn_sign(), 1.)
        self.node.on_calibration(Bool(data=True))
        self.node.on_cmd(String(data='explore'))
        self.node._ir_ready = Mock(return_value=True)
        self.node._on_wall = Mock(return_value=False)
        self.node._can_reverse = Mock(return_value=True)
        self.node.blocked = self.node.cliff = self.node.tilt = self.node.pickup = False
        self.node.estop = False
        t = TransformStamped()
        t.header.stamp = self.node.now().to_msg()
        t.transform.rotation.w = 1.
        self.node.navigation_tf = Mock()
        self.node.navigation_tf.lookup_transform.return_value = t
        self.node.navigation_route = [(0.,0.),(.3,0.)]
        self.node.navigation_received = self.node.navigation_stamp = self.node.now().nanoseconds*1e-9
        self.node._tick_navigation()
        cmd = self.node.pub.publish.call_args.args[0]
        self.assertGreater(cmd.linear.x,0.)
