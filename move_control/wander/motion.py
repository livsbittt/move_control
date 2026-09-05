"""Subject: motion. FWD / ESCAPE / TURN — drive and spin."""
import math

from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter

from ..lidar import wrap_pi
from ..recover import (
    ESCAPE_MIN_TURN,
    STUCK_CLEAR_M,
    escape_may_abort,
    escape_may_desense,
    stuck_flip,
    stuck_kind,
)


class Motion:

    def _line_wz(self) -> float:
        """Hold a straight line toward route_yaw. No circling."""
        yaw = float(getattr(self, 'route_yaw', 0.0) or 0.0)
        length = float(getattr(self, 'route_range', 0.0) or 0.0)
        if not self._finite(length) or length <= float(self.get_parameter('wall_front').value):
            return self._steer_wz()
        if abs(yaw) < math.radians(4.0):
            return 0.0
        wmax = float(self.get_parameter('steer_wmax').value)
        return max(-wmax, min(wmax, 1.4 * yaw))

    def _steer_wz(self) -> float:
        """Ratio steer: (L-R)/(L+R). Hard away when hugging the nearer wall."""
        wmax = float(self.get_parameter('steer_wmax').value)
        if not (self._finite(self.left_range) and self._finite(self.right_range)):
            return 0.0
        s = self.left_range + self.right_range
        if s <= 1e-4:
            return 0.0
        frac = (self.left_range - self.right_range) / s
        if self._hugging():
            return wmax if frac >= 0.0 else -wmax
        if abs(frac) < 0.12:
            return 0.0
        k = float(self.get_parameter('steer_k').value)
        return max(-wmax, min(wmax, k * frac))

    def _pinched_held(self) -> bool:
        if not self._front_pinched():
            self._pinch_t = None
            return False
        if self.blocked:
            return True
        if self._pinch_t is None:
            self._pinch_t = self.now()
            return False
        hold = float(self.get_parameter('escape_hold_sec').value)
        return (self.now() - self._pinch_t).nanoseconds * 1e-9 >= hold

    def _want_lidar_escape(self) -> bool:
        """Spin only at a corner: front held pinched and a side is relatively open."""
        if not self._pinched_held():
            return False
        if self.blocked:
            return True
        if not self._finite(self.front_range):
            return self.cam_block
        far = self._far_side()
        if far is None:
            return self.cam_block
        f = max(self.front_range, 1e-4)
        return far / f >= self.open_ratio or self.cam_block

    def _escape_can_adjust(self) -> bool:
        if not bool(self.get_parameter('auto_escape').value):
            return False
        if self._escape_adjust_t is None:
            return True
        return (self.now() - self._escape_adjust_t).nanoseconds * 1e-9 >= 2.0

    def _commit_escape_params(self, why: str):
        lo = float(self.get_parameter('escape_front_min').value)
        hi = float(self.get_parameter('escape_front_max').value)
        self.escape_front = max(lo, min(hi, self.escape_front))
        self.pinch_ratio = max(1.05, min(2.0, self.pinch_ratio))
        self.open_ratio = max(1.4, min(3.2, self.open_ratio))
        self._escape_adjust_t = self.now()
        self.set_parameters([
            Parameter('escape_front', Parameter.Type.DOUBLE, float(self.escape_front)),
            Parameter('pinch_ratio', Parameter.Type.DOUBLE, float(self.pinch_ratio)),
            Parameter('open_ratio', Parameter.Type.DOUBLE, float(self.open_ratio)),
        ])
        self.get_logger().warn(
            f'escape auto {why} F<{self.escape_front*100:.1f}cm '
            f'pinch={self.pinch_ratio:.2f} open={self.open_ratio:.2f}'
        )

    def _escape_false(self):
        """Escaped while the corridor was still driveable — desensitize."""
        if not self._escape_can_adjust():
            return
        self.escape_front *= 0.88
        self.pinch_ratio *= 0.94
        self.open_ratio *= 1.08
        self._commit_escape_params('too sensitive')

    def _escape_need(self):
        """Stuck without escaping — allow a slightly earlier spin next time."""
        if not self._escape_can_adjust():
            return
        self.escape_front *= 1.10
        self.pinch_ratio *= 1.04
        self.open_ratio *= 0.96
        if self._finite(self.front_range):
            self.escape_front = max(self.escape_front, self.front_range * 1.05)
        self._commit_escape_params('stuck, need escape')

    def _is_stuck(self) -> bool:
        if not self.seen_forward or not self._sensors_ready():
            return False
        if self.motion_t is None or not self.have_odom:
            return False
        dt = (self.now() - self.motion_t).nanoseconds * 1e-9
        if dt < float(self.get_parameter('stuck_sec').value):
            return False
        moved = math.hypot(self.odom_x - self.motion_x, self.odom_y - self.motion_y)
        return moved < float(self.get_parameter('stuck_m').value)

    def _note_motion(self):
        if self.motion_t is None:
            self.motion_t = self.now()
            self.motion_x, self.motion_y = self.odom_x, self.odom_y
            return
        moved = math.hypot(self.odom_x - self.motion_x, self.odom_y - self.motion_y)
        if moved >= float(self.get_parameter('stuck_m').value):
            self.motion_t = self.now()
            self.motion_x, self.motion_y = self.odom_x, self.odom_y

    def _mark_pose(self):
        self.seg_x, self.seg_y, self.seg_yaw = self.odom_x, self.odom_y, self.odom_yaw

    def _traveled(self) -> float:
        if not self.have_odom:
            return 0.0
        return math.hypot(self.odom_x - self.seg_x, self.odom_y - self.seg_y)

    def _turned(self) -> float:
        if not self.have_odom:
            return 0.0
        return abs(wrap_pi(self.odom_yaw - self.seg_yaw))

    def _blend_speed(self, dist, v_think, v_open, d_stop, d_slow):
        """Crawl when close (thinking); cruise only if the path is open."""
        if dist < 0.0 or not math.isfinite(dist):
            return v_think
        if dist <= d_stop:
            return v_think
        if dist >= d_slow:
            return v_open
        t = (dist - d_stop) / max(1e-4, d_slow - d_stop)
        return v_think + t * (v_open - v_think)

    def _safe_speed(self, dist, v_think, v_open, d_stop, d_slow) -> float:
        """Blend by remaining gap, then cap with odom so we can still stop."""
        v_blend = self._blend_speed(dist, v_think, v_open, d_stop, d_slow)
        horizon = max(0.05, float(self.get_parameter('think_horizon').value))
        if not math.isfinite(dist) or dist < 0.0:
            return v_think
        gap = dist - d_stop
        if gap <= 0.0:
            return v_think
        v_gap = gap / horizon
        v = min(v_blend, v_gap, v_open)
        v = max(v, v_think)
        if self.have_odom and abs(self.odom_vx) * horizon > gap:
            v = v_think
        return v

    def _fwd_speed(self) -> float:
        return self._safe_speed(
            self.front_range,
            float(self.get_parameter('vmax_think').value),
            self.vmax,
            float(self.get_parameter('stop_front').value),
            float(self.get_parameter('slow_front').value),
        )

    def _back_speed(self) -> float:
        return self._safe_speed(
            self.rear_range,
            float(self.get_parameter('vback_think').value),
            self.vback,
            float(self.get_parameter('stop_front').value),
            float(self.get_parameter('slow_rear').value),
        )

    def _spin_wz(self, sign=None) -> float:
        s = self.turn_sign if sign is None else sign
        return float(self.wturn) * s

    def _from_stuck_now(self) -> bool:
        return bool(getattr(self, '_from_stuck', False))

    def _clear_stuck(self):
        self._from_stuck = False
        self._stuck_n = 0

    def _recover_stuck(self):
        """Do not look-then-forward. Backup if the tail is free, else spin."""
        self._stuck_n = int(getattr(self, '_stuck_n', 0)) + 1
        self._from_stuck = True
        self._escape_need()
        if self._stuck_n == 1:
            self.turn_sign = self._pick_turn_sign()
        elif stuck_flip(self._stuck_n):
            self.turn_sign = -self.turn_sign
        kind = stuck_kind(self._can_reverse())
        self.get_logger().warn(
            f'stuck #{self._stuck_n} → {kind} sign={self.turn_sign:.0f} '
            f'F={self.front_range:.2f} rear={self.rear_range:.2f}'
        )
        if kind == 'backup':
            self._enter('backup')
            cmd = Twist()
            cmd.linear.x = -self._back_speed()
            cmd.angular.z = self._rear_steer_wz()
            self._publish(cmd, 'backup')
            return
        self._enter('escape')
        cmd = Twist()
        cmd.angular.z = self._spin_wz()
        self._publish(cmd, 'escape')

    def _tick_forward(self):
        self._note_motion()
        if self._from_stuck_now() and self._traveled() >= STUCK_CLEAR_M:
            self._clear_stuck()
        if self.tilt or self.cliff or self.blocked or self._on_wall():
            self._enter('pause')
            self._publish(Twist(), 'pause')
            return
        if self._is_stuck():
            self._recover_stuck()
            return
        if self._want_lidar_escape():
            self._enter('look')
            self._publish(Twist(), 'look')
            return
        cmd = Twist()
        cmd.linear.x = self._fwd_speed()
        cmd.angular.z = self._line_wz()
        self.seen_forward = True
        self._publish(cmd, 'forward')

    def _tick_turn(self):
        if (self.cliff or self.tilt) and self.seen_forward and self._can_reverse():
            self._enter('backup')
            cmd = Twist()
            cmd.linear.x = -self._back_speed()
            cmd.angular.z = self._rear_steer_wz()
            self._publish(cmd, 'backup')
            return
        if (self.cliff or self.tilt) and self.seen_forward and not self._can_reverse():
            cmd = Twist()
            cmd.angular.z = self._spin_wz()
            self._publish(cmd, 'turn')
            return
        if self.blocked or self._on_wall():
            self._enter('wall')
            self._publish(Twist(), 'wall')
            return
        self._look_accum()
        if self._try_turn_backup('turn objects close'):
            return
        cmd = Twist()
        w = float(self.wturn)
        cmd.angular.z = self._spin_wz()
        if self._aligned_to_open():
            self._enter('forward')
            cmd = Twist()
            cmd.linear.x = self._fwd_speed()
            cmd.angular.z = self._steer_wz()
            self.seen_forward = True
            self._publish(cmd, 'forward')
            return
        turn_rad = math.radians(float(self.get_parameter('turn_deg').value))
        need_sec = turn_rad / max(0.02, abs(w)) + 0.25
        cap_sec = max(self.turn_sec, need_sec)
        turned_enough = (
            (self.have_odom and self._turned() >= turn_rad)
            or self.elapsed() >= cap_sec
        )
        if turned_enough:
            if self.blocked or self._on_wall():
                self._enter('wall')
                self._publish(Twist(), 'wall')
                return
            else:
                self._enter('forward')
                cmd = Twist()
                cmd.linear.x = self._fwd_speed()
                cmd.angular.z = self._steer_wz()
                self.seen_forward = True
                self._publish(cmd, 'forward')
                return
        self._publish(cmd, 'turn')

    def _tick_escape(self):
        """Rotate in place toward the farthest lidar gap until it is in front."""
        if (self.cliff or self.tilt) and self.seen_forward and self._can_reverse():
            self._enter('backup')
            cmd = Twist()
            cmd.linear.x = -self._back_speed()
            cmd.angular.z = self._rear_steer_wz()
            self._publish(cmd, 'backup')
            return
        self._look_accum()
        if self._try_turn_backup('escape objects close'):
            return
        if (
            not self._from_stuck_now()
            and self._route_aligned()
            and not self._on_wall()
            and not self._front_pinched()
        ):
            self._enter('forward')
            out = Twist()
            out.linear.x = self._fwd_speed()
            out.angular.z = self._line_wz()
            self.seen_forward = True
            self._publish(out, 'forward')
            return
        cmd = Twist()
        cmd.angular.z = self._spin_wz()
        self.get_logger().info(
            f'turn objects F={self.front_range:.2f} L={self.left_range:.2f} '
            f'R={self.right_range:.2f} rear={self.rear_range:.2f} '
            f'side={self._turn_side_d():.2f} sign={self.turn_sign:.0f}',
            throttle_duration_sec=0.8,
        )
        if not self._from_stuck_now() and self._want_recon():
            self._enter('recon')
            self._publish(Twist(), 'recon')
            return
        turned = self._turned()
        aligned = (
            self.elapsed() >= 0.40
            and self._aligned_to_open()
            and not self._on_wall()
        )
        if aligned and (
            not self._from_stuck_now() or turned >= ESCAPE_MIN_TURN
        ):
            if escape_may_desense(self.elapsed(), turned, self._from_stuck_now()):
                self._escape_false()
            if self._from_stuck_now():
                self._clear_stuck()
            self._enter('forward')
            out = Twist()
            out.linear.x = self._fwd_speed()
            out.angular.z = self._steer_wz()
            self.seen_forward = True
            self._publish(out, 'forward')
            return
        if self.elapsed() > 2.5 and self._on_wall() and self._try_turn_backup('escape still on wall'):
            return
        if self._on_wall() and (turned >= math.radians(80.0) or self.elapsed() > 8.0):
            self.turn_sign = -self.turn_sign
            self._mark_pose()
            self.t0 = self.now()
            self.get_logger().warn(f'wall still — flip escape sign={self.turn_sign:.0f}')
        if escape_may_abort(turned, self._on_wall(), self.blocked):
            if self._from_stuck_now():
                self._clear_stuck()
            self._enter('forward')
            out = Twist()
            out.linear.x = self._fwd_speed()
            self.seen_forward = True
            self._publish(out, 'forward')
            return
        if self.elapsed() > 12.0:
            if self._from_stuck_now():
                self._clear_stuck()
            self._enter('forward')
            out = Twist()
            out.linear.x = self._fwd_speed()
            self.seen_forward = True
            self._publish(out, 'forward')
            return
        self._publish(cmd, 'escape')

    def _tick_wait(self):
        self._publish(Twist(), 'wait')
        if self._ir_ready() and (self._sensors_ready() or self.elapsed() > 2.0):
            self._enter('forward')
