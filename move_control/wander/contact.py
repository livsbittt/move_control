"""Subject: contact. WALL / BACK — bumper recover."""
from geometry_msgs.msg import Twist

from ..control.recover import backup_limit_m, hazard_action, wall_first_move


class Contact:

    def _can_reverse(self) -> bool:
        """Rear lidar must show a path. No scan → no reverse."""
        if not self.rear_clear:
            return False
        stop = float(self.get_parameter('stop_front').value)
        if not self._finite(self.rear_range) or self.rear_range <= stop:
            return False
        return True

    def _backup_limit(self) -> float:
        """How far we may reverse: the free rear gap, not a 45% snippet."""
        stop = float(self.get_parameter('stop_front').value)
        cap = float(self.get_parameter('backup_max_m').value)
        rear = self.rear_range if self._finite(self.rear_range) else float('nan')
        return backup_limit_m(rear, stop=stop, cap=cap)

    def _rear_steer_wz(self) -> float:
        """Steer the tail toward the more open rear side (sign flipped vs forward)."""
        wmax = float(self.get_parameter('steer_wmax').value)
        if not (self._finite(self.rear_left) and self._finite(self.rear_right)):
            return 0.0
        s = self.rear_left + self.rear_right
        if s <= 1e-4:
            return 0.0
        frac = (self.rear_left - self.rear_right) / s
        if abs(frac) < 0.12:
            return 0.0
        k = float(self.get_parameter('steer_k').value)
        return max(-wmax, min(wmax, -k * frac))

    def _back_cmd(self) -> Twist:
        """Standard reverse command: back speed with tail steering."""
        cmd = Twist()
        cmd.linear.x = -self._back_speed()
        cmd.angular.z = self._rear_steer_wz()
        return cmd

    def _try_turn_backup(self, why: str) -> bool:
        """Reverse to make turning room. Uses live F / turn-side object distance."""
        n = int(getattr(self, '_turn_backs', 0))
        max_n = int(self.get_parameter('turn_back_max').value)
        if n >= max_n or not self._need_space_to_turn():
            return False
        self._turn_backs = n + 1
        side = self._turn_side_d()
        self.get_logger().warn(
            f'back for turn #{self._turn_backs}/{max_n} {why} '
            f'F={self.front_range:.2f} side={side:.2f} '
            f'rear={self.rear_range:.2f} sign={self.turn_sign:.0f}'
        )
        self._start_backup()
        return True

    def _tick_wall(self):
        """Wall bumper: reverse off it if the tail is clear, else spin away."""
        if hazard_action(
            self.tilt, self.cliff, self.seen_forward, self._can_reverse()
        ) != 'none':
            self._hold('pause')
            return
        if not self._on_wall() and not self.blocked and self._aligned_to_open():
            self._resume_forward()
            return
        if wall_first_move(self._can_reverse(), self._wall_backed) == 'backup':
            self._wall_backed = True
            self._start_backup()
            return
        if self._try_turn_backup('wall still tight'):
            return
        self._start_escape()

    def _finish_backup(self):
        """Stuck recovery spins next. Otherwise look and re-plan."""
        if getattr(self, '_from_stuck', False):
            self._enter('escape')
            cmd = Twist()
            cmd.angular.z = self._spin_wz()
            self._publish(cmd, 'escape')
            return
        self._enter('look')
        self._publish(Twist(), 'look')

    def _tick_backup(self):
        if not self._can_reverse():
            self.get_logger().warn(
                f'rear not clear — no reverse lidar={self.rear_range:.3f}m '
                f'RL={self.rear_left:.2f} RR={self.rear_right:.2f}'
            )
            self._finish_backup()
            return
        cmd = Twist()
        cmd.linear.x = -self._back_speed()
        cmd.angular.z = self._rear_steer_wz()
        elapsed = self.elapsed()
        traveled = self._traveled()
        limit = self._backup_limit()
        too_far = elapsed >= self.backup_max_sec or (
            self.have_odom and traveled >= limit
        )

        if too_far:
            if self.tilt:
                self.get_logger().error('tilt still after backup — stop')
                self._set_enabled(False)
                self._publish(Twist(), 'stop')
                return
            if self.cliff:
                self.get_logger().error(
                    f'cliff still after {elapsed:.2f}s backup IR={self.ir} — stop'
                )
                self._set_enabled(False)
                self._publish(Twist(), 'stop')
                return
            self._finish_backup()
            return

        if self.cliff or self.tilt:
            self.cliff_clear_since = None
            self._publish(cmd, 'backup')
            return

        if self.cliff_clear_since is None:
            self.cliff_clear_since = self.now()
            self.get_logger().info(
                f'cliff clear, rear={self.rear_range:.3f}m limit={limit:.3f}m then turn'
            )
        clear_for = (self.now() - self.cliff_clear_since).nanoseconds * 1e-9
        if (
            elapsed >= self.backup_min_sec
            and clear_for >= self.backup_clear_sec
            and self._have_turn_space()
        ):
            self._finish_backup()
            return
        self._publish(cmd, 'backup')
