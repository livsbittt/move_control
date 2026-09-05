"""Subject: hazard. ESTOP / CLIFF / TILT / PICK."""
import math

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String, UInt16MultiArray

from ..lidar import wrap_pi


def roll_pitch(q):
    w, x, y, z = q.w, q.x, q.y, q.z
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    return roll, pitch


class Hazard:

    def on_estop(self, msg: Bool):
        if msg.data:
            self.engage_estop('topic')
        else:
            self.release_estop()

    def on_estop_cmd(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd in ('stop', 'estop', 'e-stop', 'emergency', 'kill', 'halt', 'on', '1', 'true'):
            self.engage_estop(cmd)
        elif cmd in ('release', 'clear', 'reset', 'off', '0', 'false', 'resume'):
            self.release_estop()
        else:
            self.get_logger().warn(f'unknown /estop/cmd {cmd!r} (stop|release)')

    def engage_estop(self, why: str = ''):
        if not self.estop:
            self.get_logger().error(f'EMERGENCY STOP {why}'.strip())
        self.estop = True
        self.last_cmd = Twist()
        self.wander_cmd.publish(String(data='stop'))
        self.calib_step.publish(String(data='abort'))
        self.raw_zero_pub.publish(Twist())
        self._publish_zero()
        self.estop_pub.publish(Bool(data=True))

    def release_estop(self):
        if self.estop:
            self.get_logger().warn('E-STOP released — motors stay 0 until a new command')
        self.estop = False
        self.last_cmd = Twist()
        self.last_cmd_time = None
        self._publish_zero()
        self.estop_pub.publish(Bool(data=False))

    def on_ir(self, msg: UInt16MultiArray):
        self.ir_raw = tuple(int(v) for v in msg.data[:3])
        self.last_ir_time = self.now()

    def on_cam_cliff(self, msg: Bool):
        self.cam_cliff = bool(msg.data)
        self.last_cam_time = self.now()

    def on_cam_block(self, msg: Bool):
        self.cam_block = bool(msg.data)
        self.last_cam_time = self.now()

    def on_imu(self, msg: Imu):
        ax, ay, az = (
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        )
        acc = math.sqrt(ax * ax + ay * ay + az * az)
        # Dead / uninit IMU (0xa0 fail → zeros). Do not treat as pickup or tilt.
        if acc < 1.0:
            self.tilt = False
            self.pickup = False
            return
        if 6.0 <= acc <= 14.0:
            self._imu_has_g = True
        if not self._imu_has_g:
            self.tilt = False
            self.pickup = False
            return
        self.last_imu_time = self.now()
        roll, pitch = roll_pitch(msg.orientation)
        self._imu_n += 1
        if self._imu_n < 40:
            self.tilt = False
            self.pickup = False
            return
        roll0 = math.radians(float(self.get_parameter('imu_roll0').value))
        pitch0 = math.radians(float(self.get_parameter('imu_pitch0').value))
        droll = abs(wrap_pi(roll - roll0))
        dpitch = abs(wrap_pi(pitch - pitch0))
        lim = math.radians(float(self.get_parameter('tilt_deg').value))
        gx = float(msg.angular_velocity.x)
        gy = float(msg.angular_velocity.y)
        gyro = math.hypot(gx, gy)
        gyro_dps = gyro if gyro > 20.0 else math.degrees(gyro)
        angled = (droll > lim) or (dpitch > lim)
        spinning = gyro_dps > float(self.get_parameter('gyro_dps').value)
        self.tilt = angled and (spinning or droll > lim * 1.2 or dpitch > lim * 1.2)
        self.pickup = acc < float(self.get_parameter('pickup_acc').value)

    def ir_looks_like_cliff(self) -> bool:
        if not self.get_parameter('cliff_enable').value:
            return False
        if self.age(self.last_ir_time) > self.timeout or len(self.ir_raw) < 3:
            # Hold last value if IR drops mid-edge; never invent a cliff from silence.
            return self.cliff
        ir_f = self._ir_f.push(self.ir_raw)
        # 4095 = lifted / ADC sat. Need 2 real sensors to declare a cliff.
        ir = tuple(v for v in ir_f if v < 4000)
        sat = sum(1 for v in ir_f if v >= 4000)
        need = int(self.get_parameter('cliff_hits').value)
        if sat >= 2:
            return False
        if len(ir) < need:
            return self.cliff
        lo = int(self.get_parameter('cliff_raw_max').value)
        hi = int(self.get_parameter('cliff_clear_raw').value)
        hits = need
        mode = str(self.get_parameter('cliff_mode').value)
        if mode == 'high':
            n_trig = sum(1 for v in ir if v > lo)
            n_hold = sum(1 for v in ir if v > hi)
        else:
            n_trig = sum(1 for v in ir if v < lo)
            n_hold = sum(1 for v in ir if v < hi)
        if self.cliff:
            return n_hold >= hits
        return n_trig >= hits
