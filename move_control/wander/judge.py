"""Subject: judge. LOOK / CALC / RECON / PAUSE — decide, do not drive far."""
import math

from geometry_msgs.msg import Twist


class Judge:

    def _look_accum(self):
        self._look_n += 1
        if self._finite(self.left_range):
            self._look_L += self.left_range
            self._look_nL += 1
            self._samp_L.append(self.left_range)
            self._samp_L = self._samp_L[-40:]
        if self._finite(self.right_range):
            self._look_R += self.right_range
            self._look_nR += 1
            self._samp_R.append(self.right_range)
            self._samp_R = self._samp_R[-40:]
        if self._finite(self.front_range):
            self._look_F += self.front_range
            self._look_nF += 1
            self._samp_F.append(self.front_range)
            self._samp_F = self._samp_F[-40:]
        self._look_yaw += self.open_yaw
        self._look_cam += self.cam_side
        if self._finite(self.rear_range):
            self._samp_rear = getattr(self, '_samp_rear', [])
            self._samp_rear.append(self.rear_range)
            self._samp_rear = self._samp_rear[-40:]

    def _median(self, xs):
        if not xs:
            return None
        s = sorted(xs)
        return s[len(s) // 2]

    def _look_means(self):
        F = self._median(self._samp_F)
        L = self._median(self._samp_L)
        R = self._median(self._samp_R)
        return F, L, R

    def _look_sign(self) -> float:
        ry = float(getattr(self, 'route_yaw', 0.0) or 0.0)
        if abs(ry) > math.radians(8.0):
            return 1.0 if ry > 0.0 else -1.0
        fy = float(getattr(self, 'frontier_yaw', 0.0) or 0.0)
        if abs(fy) > math.radians(10.0):
            return 1.0 if fy > 0.0 else -1.0
        if self.cam_block and self._look_n > 0:
            side = self._look_cam / self._look_n
            if abs(side) >= 0.3:
                return 1.0 if side > 0.0 else -1.0
        L = self._median(self._samp_L)
        R = self._median(self._samp_R)
        if L is not None and R is not None:
            if L > R * 1.15:
                return 1.0
            if R > L * 1.15:
                return -1.0
        if self._look_n > 0:
            yaw = self._look_yaw / self._look_n
            if abs(yaw) > math.radians(8.0):
                return 1.0 if yaw > 0.0 else -1.0
        return self._pick_turn_sign()

    def _calc_plan(self):
        """Return (kind, sign, why). kind: forward, backup, escape, look."""
        F, L, R = self._look_means()
        sign = self._look_sign()
        if getattr(self, '_from_stuck', False):
            sign = self.turn_sign
            if self._can_reverse() and self._need_space_to_turn(sign):
                return 'backup', sign, 'stuck space'
            return 'escape', sign, 'stuck spin'
        if (self.tilt or (self.cliff and self.seen_forward)) and self._can_reverse():
            return 'backup', sign, 'cliff/tilt'
        if self.tilt or self.cliff:
            return 'turn', sign, 'cliff/tilt no rear'
        if self._on_wall():
            if self._can_reverse() and not self._wall_backed:
                return 'backup', sign, 'wall rear clear'
            if self._need_space_to_turn(sign):
                return 'backup', sign, 'space for turn'
            return 'escape', sign, 'wall spin'
        if self.blocked or self.cam_block:
            if self._need_space_to_turn(sign):
                return 'backup', sign, 'space for turn'
            return 'escape', sign, 'corner'
        ry = float(getattr(self, 'route_yaw', 0.0) or 0.0)
        rl = float(getattr(self, 'route_range', 0.0) or 0.0)
        if self._route_aligned() and not self._front_pinched() and not self._on_wall():
            return 'forward', 1.0 if ry >= 0.0 else sign, 'line route'
        if self._finite(rl) and rl > float(self.get_parameter('wall_front').value):
            if abs(ry) > math.radians(float(self.get_parameter('align_deg').value)):
                if self._need_space_to_turn(1.0 if ry > 0.0 else -1.0):
                    return 'backup', 1.0 if ry > 0.0 else -1.0, 'space for turn'
                return 'turn', 1.0 if ry > 0.0 else -1.0, 'face line'
        fy = float(getattr(self, 'frontier_yaw', 0.0) or 0.0)
        fd = float(getattr(self, 'frontier_range', float('inf')))
        if (
            self._finite(fd)
            and abs(fy) < math.radians(18.0)
            and not self._front_pinched()
            and not self._on_wall()
            and (F is None or fd >= max(0.16, (F or 0.0) * 1.05))
        ):
            return 'forward', 1.0 if fy >= 0.0 else sign, 'frontier ahead'
        if self._aligned_to_open() and not self._front_pinched():
            return 'forward', sign, 'front open'
        if L is not None and R is not None:
            wide = max(L, R)
            tight = min(L, R)
            if tight > 1e-4 and wide / tight < 1.12 and self._calc_tries < 2:
                return 'look', sign, 'L~R unsure'
        if self._need_space_to_turn(sign):
            return 'backup', sign, 'space for turn'
        return 'escape', sign, 'turn to opening'

    def _commit_plan(self, kind, sign, why):
        self.turn_sign = sign
        self.get_logger().info(f'calc {kind} sign={sign:.0f} ({why})')
        if kind == 'look':
            self._calc_tries += 1
            self._enter('look')
            self._publish(Twist(), 'look')
            return
        if kind == 'backup':
            if 'space' in why:
                self._turn_backs = int(getattr(self, '_turn_backs', 0)) + 1
            self._enter('backup')
            cmd = Twist()
            cmd.linear.x = -self._back_speed()
            cmd.angular.z = self._rear_steer_wz()
            self._publish(cmd, 'backup')
            return
        if kind == 'turn':
            self._enter('turn')
            cmd = Twist()
            cmd.angular.z = self._spin_wz(sign)
            self._publish(cmd, 'turn')
            return
        if kind == 'forward':
            self._enter('forward')
            cmd = Twist()
            cmd.linear.x = self._fwd_speed()
            cmd.angular.z = self._steer_wz()
            self.seen_forward = True
            self._publish(cmd, 'forward')
            return
        if kind == 'escape':
            if self._on_wall() and not getattr(self, '_from_stuck', False):
                self._enter('wall')
                self._publish(Twist(), 'wall')
                return
            self._enter('escape')
            cmd = Twist()
            cmd.angular.z = self._spin_wz(sign)
            self._publish(cmd, 'escape')
            return
        self._enter('escape')
        cmd = Twist()
        cmd.angular.z = self._spin_wz(sign)
        self._publish(cmd, 'escape')

    def _tick_look(self):
        """Stop and sample. Calc scores next."""
        if self.pickup:
            self._publish(Twist(), 'look')
            return
        self._look_accum()
        self._publish(Twist(), 'look')
        look_sec = float(self.get_parameter('look_sec').value)
        if self.elapsed() < look_sec:
            return
        if self._look_nF < 3 and self.elapsed() < look_sec + 1.2:
            return
        if self._look_nF < 1:
            self.get_logger().warn('look: no front range yet — wait')
            self._enter('wait')
            self._publish(Twist(), 'wait')
            return
        F, L, R = self._look_means()
        def _fmt(x):
            return f'{x:.2f}' if x is not None else 'nan'
        self.get_logger().info(
            f'look done n={self._look_n} F={_fmt(F)} L={_fmt(L)} R={_fmt(R)}'
        )
        self._enter('calc')
        self._publish(Twist(), 'calc')

    def _tick_calc(self):
        """Score look samples; look again if L and R are too close."""
        self._look_accum()
        self._publish(Twist(), 'calc')
        if self.elapsed() < float(self.get_parameter('calc_sec').value):
            return
        kind, sign, why = self._calc_plan()
        self._commit_plan(kind, sign, why)

    def _tick_recon(self):
        """Pause, then a fresh look."""
        self._publish(Twist(), 'recon')
        if self.elapsed() < 0.35:
            return
        self._enter('look')
        self._publish(Twist(), 'look')

    def _want_recon(self) -> bool:
        if self._recon_n >= int(self.get_parameter('recon_max').value):
            return False
        if self.elapsed() < float(self.get_parameter('recon_sec').value):
            return False
        if not (self._finite(self.left_range) and self._finite(self.right_range)):
            return False
        live = 1.0 if self.left_range > self.right_range else -1.0
        if live == self.turn_sign:
            return False
        wide = max(self.left_range, self.right_range)
        tight = min(self.left_range, self.right_range)
        if tight <= 1e-4 or wide / tight < 1.35:
            return False
        return True

    def _tick_pause(self):
        cmd = Twist()
        if self.elapsed() < self.pause_sec:
            self._publish(cmd, 'pause')
            return
        if (self.tilt or (self.cliff and self.seen_forward)) and self._can_reverse():
            self._enter('backup')
            cmd.linear.x = -self._back_speed()
            cmd.angular.z = self._rear_steer_wz()
            self._publish(cmd, 'backup')
        elif self.tilt or self.cliff:
            self._enter('look')
            self._publish(Twist(), 'look')
        elif self.blocked or self._on_wall():
            self._enter('look')
            self._publish(Twist(), 'look')
        else:
            self._enter('forward')
            cmd.linear.x = self._fwd_speed()
            self.seen_forward = True
            self._publish(cmd, 'forward')
