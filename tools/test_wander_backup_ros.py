"""Reactive recovery transitions on an isolated ROS domain, no hardware."""
import unittest
from unittest.mock import Mock
import rclpy
from rclpy.time import Time
from rosy_control.wander.node import WanderNode


class BackupTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(domain_id=218)

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.n = WanderNode()
        self.n._publish = Mock()
        self.n._motion_limits_fresh = Mock(return_value=True)
        self.n.motion_limits = dict(translation_mode=True,reverse_travel_m=.04,
            front_m=.3,rear_m=.13,front_stop_m=.07,rear_stop_m=.07)
        self.n.rear_clear = True

    def tearDown(self):
        self.n.destroy_node()

    def test_reverse_uses_fresh_actual_guard_and_straight_stroke(self):
        self.n.rear_range = .09
        self.assertTrue(self.n._can_reverse())
        self.n._rear_steer_wz = Mock(return_value=.1)
        command = self.n._back_cmd()
        self.assertLess(command.linear.x,0.)
        self.assertEqual(command.angular.z,0.)
        self.assertTrue(self.n._wall_backed)
        self.assertAlmostEqual(self.n._backup_entry_limit,.04)
        self.n.rear_clear = False
        self.assertFalse(self.n._can_reverse())

    def test_remaining_gap_does_not_halve_entry_stroke(self):
        self.n._back_cmd()
        self.n.motion_limits['reverse_travel_m'] = .015
        self.n.elapsed = Mock(return_value=1.)
        self.n._traveled = Mock(return_value=.025)
        self.n.have_odom = True
        self.n._have_turn_space = Mock(return_value=False)
        self.n._finish_backup = Mock()
        self.n._tick_backup()
        self.n._finish_backup.assert_not_called()
        self.assertLess(self.n._publish.call_args.args[0].linear.x,0.)

    def test_clear_aligned_path_resumes_after_backup_but_hazard_cannot(self):
        self.n.estop = self.n.pickup = self.n.blocked = False
        self.n._ir_ready = Mock(return_value=True)
        self.n._on_wall = self.n._front_pinched = Mock(return_value=False)
        self.n._route_aligned = Mock(return_value=True)
        self.n._resume_forward = Mock()
        self.n._finish_backup()
        self.n._resume_forward.assert_called_once_with(use_line=True)
        self.n._resume_forward.reset_mock()
        self.n.pickup = True
        self.n._finish_backup()
        self.n._resume_forward.assert_not_called()

    def test_stuck_backup_keeps_escape_before_retrying_forward(self):
        self.n._from_stuck = True
        self.n._start_escape = Mock()
        self.n._resume_forward = Mock()
        self.n._finish_backup()
        self.n._start_escape.assert_called_once()
        self.n._resume_forward.assert_not_called()

    def test_blocked_rotation_backs_once_then_exhausts_shared_budget(self):
        self.n._odom_fresh = Mock(return_value=True)
        self.n.t0 = Time(seconds=100)
        self.n.now = Mock(return_value=Time(seconds=100))
        self.n._start_backup = Mock(side_effect=self.n._back_cmd)
        for start in (100, 110, 120, 130):
            self.n.now.return_value = Time(seconds=start)
            self.assertFalse(self.n._escape_rotation_stalled())
            self.n.now.return_value = Time(seconds=start+9)
            self.assertTrue(self.n._escape_rotation_stalled())
        self.n._start_backup.assert_called_once()
        self.assertAlmostEqual(self.n._backup_entry_limit,.03)
        self.assertEqual(self.n.stop_reason,'escape_rotation_exhausted')

    def test_real_rotation_progress_does_not_spend_recovery_budget(self):
        self.n._odom_fresh = Mock(return_value=True)
        self.n.t0 = Time(seconds=100)
        self.n.now = Mock()
        for i in range(30):
            self.n.now.return_value = Time(seconds=100+i)
            self.n.odom_yaw = i*.06
            self.assertFalse(self.n._escape_rotation_stalled())
        self.assertIsNone(getattr(self.n,'_recovery_budget',None))

    def test_stale_odometry_never_authorizes_recovery_motion(self):
        self.n._odom_fresh = Mock(return_value=False)
        self.n._start_backup = Mock()
        self.assertTrue(self.n._escape_rotation_stalled())
        self.assertEqual(self.n.stop_reason,'escape_odometry_stale')
        self.n._start_backup.assert_not_called()

    def test_legacy_escape_timer_reset_cannot_replenish_stall_window(self):
        self.n._odom_fresh = Mock(return_value=True)
        self.n.now = Mock(return_value=Time(seconds=100))
        self.n._start_backup = Mock()
        self.assertFalse(self.n._escape_rotation_stalled())
        self.n.t0 = Time(seconds=107)
        self.n.now.return_value = Time(seconds=109)
        self.assertTrue(self.n._escape_rotation_stalled())
        self.n._start_backup.assert_called_once()
