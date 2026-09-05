"""Subject: command gate. Halt, drive sign, publish /cmd_vel."""
import math

from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter


class Gate:

    def on_cmd(self, msg: Twist):
        if self.estop:
            self._publish_zero()
            return
        self.last_cmd = msg
        self.last_cmd_time = self.now()

    def _auto_linear_sign(self, us, lidar, vx_sem, wz):
        """If nose-forward makes a real front wall recede, the motor sign is inverted."""
        if not bool(self.get_parameter('auto_linear_sign').value):
            self._sign_t = None
            self._sign_hits = 0
            return
        front = None
        if math.isfinite(us) and 0.04 < us < 0.80:
            front = us
        elif math.isfinite(lidar) and 0.10 < lidar < 0.80:
            front = lidar
        if (
            abs(vx_sem) < 0.004
            or abs(wz) > 0.06
            or front is None
        ):
            self._sign_t = None
            self._sign_front0 = None
            self._sign_hits = 0
            return
        if self._sign_t is None:
            self._sign_t = self.now()
            self._sign_front0 = front
            self._sign_vx0 = vx_sem
            return
        dt = (self.now() - self._sign_t).nanoseconds * 1e-9
        if dt < 0.70:
            return
        dfront = front - self._sign_front0
        self._sign_t = None
        self._sign_front0 = None
        if abs(dfront) < 0.03:
            self._sign_hits = 0
            return
        want_fwd = self._sign_vx0 > 0.0
        wrong = (want_fwd and dfront > 0.0) or ((not want_fwd) and dfront < 0.0)
        if not wrong:
            self._sign_hits = 0
            return
        self._sign_hits += 1
        if self._sign_hits < 2:
            return
        if self._sign_flip_t is not None:
            if (self.now() - self._sign_flip_t).nanoseconds * 1e-9 < 20.0:
                return
        self._sign_hits = 0
        self.cmd_linear_sign = -self.cmd_linear_sign
        self._sign_flip_t = self.now()
        self.set_parameters([
            Parameter('cmd_linear_sign', Parameter.Type.DOUBLE, float(self.cmd_linear_sign)),
        ])
        why = (
            f'forward opened front {dfront*100:.1f}cm'
            if want_fwd else
            f'backup closed front {dfront*100:.1f}cm'
        )
        self.get_logger().error(
            f'drive sign auto-flip → {self.cmd_linear_sign:.0f} ({why})'
        )

    def _publish_zero(self):
        self.pub.publish(Twist())

    def stop_motors(self):
        try:
            self.engage_estop('shutdown')
        except Exception:
            try:
                self.pub.publish(Twist())
            except Exception:
                pass
