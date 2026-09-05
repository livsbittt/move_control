#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan, Range
from std_msgs.msg import Bool, Float32, String, UInt16MultiArray


def wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def roll_pitch(q):
    w, x, y, z = q.w, q.x, q.y, q.z
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    return roll, pitch


def parse_us_range(msg: Range, scale: float = 1.0):
    """Convert a Range msg to metres, or None to hold the last value.

    Vendor formula is (adc/4096) - 0.03, so no-echo is <= 0 and the
    2 cm blind zone is 0 < r <= min_range. A single close ping must
    count; invalid/no-echo must not wipe it.
    """
    r = float(msg.range) * float(scale)
    lo = float(msg.min_range) if msg.min_range > 0.0 else 0.02
    hi = float(msg.max_range) if msg.max_range > lo else 3.0
    if not math.isfinite(r) or r <= 0.0 or r >= hi:
        return None
    # Blind zone / no-echo both sit at or below min_range — do not call that a wall.
    if r <= lo + 1e-4:
        return None
    return r


class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')
        self.declare_parameter('cmd_in', '/cmd_vel_raw')
        self.declare_parameter('cmd_out', '/cmd_vel')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('us_topic', '/us_sensor/range')
        self.declare_parameter('ir_topic', '/ir_sensor/range')
        self.declare_parameter('stop_distance', 0.025)
        self.declare_parameter('clear_distance', 0.035)
        self.declare_parameter('us_stop_distance', 0.022)
        self.declare_parameter('us_clear_distance', 0.032)
        self.declare_parameter('front_half_width_deg', 12.0)
        self.declare_parameter('lidar_yaw_offset', 0.0)
        self.declare_parameter('sensor_timeout', 1.0)
        self.declare_parameter('us_scale', 1.0)
        self.declare_parameter('us_invalid_hold', 5)
        self.declare_parameter('us_hits', 3)
        self.declare_parameter('cmd_linear_sign', -1.0)
        self.declare_parameter('camera_as_wall', False)
        self.declare_parameter('imu_topic', '/imu_raw')
        self.declare_parameter('tilt_deg', 20.0)
        self.declare_parameter('gyro_dps', 90.0)
        self.declare_parameter('pickup_acc', 3.5)
        self.declare_parameter('imu_roll0', 0.0)
        self.declare_parameter('imu_pitch0', 0.0)
        self.declare_parameter('cliff_enable', True)
        self.declare_parameter('cliff_raw_max', 800)
        self.declare_parameter('cliff_clear_raw', 1500)
        self.declare_parameter('cliff_mode', 'low')
        self.declare_parameter('cliff_hits', 2)
        self.declare_parameter('camera_cliff_topic', '/camera/cliff')
        self.declare_parameter('camera_block_topic', '/camera/blocked')

        self.stop_d = float(self.get_parameter('stop_distance').value)
        self.clear_d = float(self.get_parameter('clear_distance').value)
        self.us_stop = float(self.get_parameter('us_stop_distance').value)
        self.us_clear = float(self.get_parameter('us_clear_distance').value)
        self.half_w = math.radians(float(self.get_parameter('front_half_width_deg').value))
        self.lidar_yaw = float(self.get_parameter('lidar_yaw_offset').value)
        self.timeout = float(self.get_parameter('sensor_timeout').value)
        self.cmd_out = self.get_parameter('cmd_out').value

        self.pub = self.create_publisher(Twist, self.cmd_out, 10)
        self.raw_zero_pub = self.create_publisher(Twist, self.get_parameter('cmd_in').value, 10)
        self.block_pub = self.create_publisher(Bool, '/safety/blocked', 10)
        self.cliff_pub = self.create_publisher(Bool, '/safety/cliff', 10)
        self.tilt_pub = self.create_publisher(Bool, '/safety/tilt', 10)
        self.pickup_pub = self.create_publisher(Bool, '/safety/pickup', 10)
        self.range_pub = self.create_publisher(Float32, '/safety/min_range', 10)
        self.us_pub = self.create_publisher(Float32, '/safety/us_range', 10)
        self.rear_pub = self.create_publisher(Float32, '/safety/rear_range', 10)
        self.rear_clear_pub = self.create_publisher(Bool, '/safety/rear_clear', 10)
        self.wander_cmd = self.create_publisher(String, '/wander/cmd', 10)
        self.calib_step = self.create_publisher(String, '/calib/step', 10)
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.estop_pub = self.create_publisher(Bool, '/estop/state', latched)
        self.create_subscription(Twist, self.get_parameter('cmd_in').value, self.on_cmd, 10)
        self.create_subscription(Bool, '/estop', self.on_estop, latched)
        self.create_subscription(String, '/estop/cmd', self.on_estop_cmd, 10)
        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value, self.on_scan, qos_profile_sensor_data
        )
        self.create_subscription(
            Range, self.get_parameter('us_topic').value, self.on_us, 10
        )
        self.create_subscription(
            UInt16MultiArray, self.get_parameter('ir_topic').value, self.on_ir, 10
        )
        self.create_subscription(
            Bool, self.get_parameter('camera_cliff_topic').value, self.on_cam_cliff, 10
        )
        self.create_subscription(
            Bool, self.get_parameter('camera_block_topic').value, self.on_cam_block, 10
        )
        self.create_subscription(
            Imu, self.get_parameter('imu_topic').value, self.on_imu, qos_profile_sensor_data
        )
        self.create_timer(0.05, self.tick)

        self.last_cmd = Twist()
        self.last_cmd_time = None
        self.last_scan_time = None
        self.last_us_time = None
        self.last_ir_time = None
        self.last_cam_time = None
        self.last_imu_time = None
        self.lidar_front = float('inf')
        self.lidar_rear = float('inf')
        self.us_front = float('inf')
        self.rear_blocked = False
        self._imu_n = 0
        self.ir_raw = ()
        self.blocked = False
        self.us_blocked = False
        self.cliff = False
        self.cam_cliff = False
        self.cam_block = False
        self.tilt = False
        self.pickup = False
        self._cam_cliff_logged = False
        self._tilt_logged = False
        self._pickup_logged = False
        self._us_invalid = 0
        self._us_hits = 0
        self._imu_has_g = False
        self.estop = False
        self.us_scale = float(self.get_parameter('us_scale').value)
        self.us_invalid_hold = max(1, int(self.get_parameter('us_invalid_hold').value))
        self.cmd_linear_sign = float(self.get_parameter('cmd_linear_sign').value)
        if self.cmd_linear_sign >= 0.0:
            self.cmd_linear_sign = 1.0
        else:
            self.cmd_linear_sign = -1.0
        self.get_logger().info(
            f'safety_node ready | lidar_stop={self.stop_d:.2f}m us_stop={self.us_stop:.2f}m '
            f'linear_sign={self.cmd_linear_sign:.0f} cam_wall='
            f'{int(bool(self.get_parameter("camera_as_wall").value))} '
            f'cliff_mode={self.get_parameter("cliff_mode").value} '
            f'th={self.get_parameter("cliff_raw_max").value} '
            f'lidar_yaw={math.degrees(self.lidar_yaw):.0f}deg | '
            'E-STOP /estop Bool or /estop/cmd stop|release'
        )
        self.estop_pub.publish(Bool(data=False))

    def now(self):
        return self.get_clock().now()

    def age(self, stamp):
        if stamp is None:
            return 1e9
        return (self.now() - stamp).nanoseconds / 1e9

    def on_cmd(self, msg: Twist):
        if self.estop:
            self._publish_zero()
            return
        self.last_cmd = msg
        self.last_cmd_time = self.now()

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

    def on_us(self, msg: Range):
        self.last_us_time = self.now()
        parsed = parse_us_range(msg, self.us_scale)
        if parsed is None:
            self._us_invalid += 1
            if self._us_invalid >= self.us_invalid_hold:
                self.us_front = float('inf')
            return
        self._us_invalid = 0
        self.us_front = parsed

    def on_scan(self, msg: LaserScan):
        yaw = self.lidar_yaw
        self.lidar_front = sector_min(msg, yaw, self.half_w)
        self.lidar_rear = sector_min(msg, wrap_pi(yaw + math.pi), self.half_w)
        self.last_scan_time = self.now()

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
        # 4095 = lifted / ADC sat. Need 2 real sensors to declare a cliff.
        ir = tuple(v for v in self.ir_raw if v < 4000)
        sat = sum(1 for v in self.ir_raw if v >= 4000)
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

    def front_distance(self) -> float:
        """Debug min of fresh lidar and US. Not used for the lidar latch."""
        d = float('inf')
        if self.age(self.last_scan_time) < self.timeout:
            d = min(d, self.lidar_front)
        if self.age(self.last_us_time) < self.timeout:
            d = min(d, self.us_front)
        return d

    def lidar_distance(self) -> float:
        if self.age(self.last_scan_time) < self.timeout:
            return self.lidar_front
        return float('inf')

    def rear_distance(self) -> float:
        if self.age(self.last_scan_time) < self.timeout:
            return self.lidar_rear
        return float('inf')

    def sensors_ok(self) -> bool:
        return self.age(self.last_scan_time) < self.timeout

    def us_distance(self) -> float:
        if self.age(self.last_us_time) < self.timeout:
            return self.us_front
        return float('inf')

    def _refresh_distances(self):
        self.stop_d = float(self.get_parameter('stop_distance').value)
        self.clear_d = float(self.get_parameter('clear_distance').value)
        self.us_stop = float(self.get_parameter('us_stop_distance').value)
        self.us_clear = float(self.get_parameter('us_clear_distance').value)
        self.half_w = math.radians(float(self.get_parameter('front_half_width_deg').value))
        sign = float(self.get_parameter('cmd_linear_sign').value)
        self.cmd_linear_sign = 1.0 if sign >= 0.0 else -1.0
        self.lidar_yaw = float(self.get_parameter('lidar_yaw_offset').value)

    def tick(self):
        self._refresh_distances()
        if self.estop:
            self.raw_zero_pub.publish(Twist())
            self._publish_zero()
            self.estop_pub.publish(Bool(data=True))
            return
        lidar_d = self.lidar_distance()
        us = self.us_distance()
        # Speed/think uses lidar only. US flicker at 2 cm must not look like a wall 2 cm away.
        self.range_pub.publish(Float32(data=float(lidar_d if math.isfinite(lidar_d) else -1.0)))
        self.us_pub.publish(Float32(data=float(us if math.isfinite(us) else -1.0)))

        # Cliff is independent of lidar. Always evaluate IR even if /scan is missing.
        if self.age(self.last_ir_time) > self.timeout:
            self.get_logger().warn('no IR — cliff unknown', throttle_duration_sec=2.0)
        cliff_now = self.ir_looks_like_cliff()
        if cliff_now and not self.cliff:
            self.get_logger().error(f'CLIFF IR={self.ir_raw}')
        elif self.cliff and not cliff_now:
            self.get_logger().info(f'cliff clear IR={self.ir_raw}')
        self.cliff = cliff_now
        self.cliff_pub.publish(Bool(data=self.cliff))

        lidar_ok = self.sensors_ok()
        rear_d = self.rear_distance()
        self.rear_pub.publish(Float32(data=float(rear_d if math.isfinite(rear_d) else -1.0)))
        if not lidar_ok:
            self.get_logger().warn('no lidar — reverse disabled until scan', throttle_duration_sec=2.0)
            self.blocked = False
            self.rear_blocked = True
        elif self.stop_d > 1e-4:
            if lidar_d <= self.stop_d:
                if not self.blocked:
                    self.get_logger().warn(f'BLOCKED lidar={lidar_d:.2f} m')
                self.blocked = True
            elif lidar_d >= self.clear_d:
                self.blocked = False
            if rear_d <= self.stop_d:
                if not self.rear_blocked:
                    self.get_logger().warn(f'REAR blocked lidar={rear_d:.2f} m')
                self.rear_blocked = True
            elif rear_d >= self.clear_d:
                self.rear_blocked = False
        else:
            self.blocked = False
            self.rear_blocked = False
        self.rear_clear_pub.publish(Bool(data=lidar_ok and not self.rear_blocked))

        us_need = max(1, int(self.get_parameter('us_hits').value))
        if us <= self.us_stop:
            self._us_hits += 1
            if self._us_hits >= us_need:
                if not self.us_blocked:
                    self.get_logger().warn(f'WALL us={us:.3f} m (stop {self.us_stop:.3f})')
                self.us_blocked = True
        else:
            self._us_hits = 0
            if us >= self.us_clear and self.us_blocked:
                self.get_logger().info(f'wall clear us={us:.3f} m')
                self.us_blocked = False

        cam_ok = self.age(self.last_cam_time) < self.timeout
        use_cam = bool(self.get_parameter('camera_as_wall').value)
        cam_cliff = bool(self.cam_cliff) if cam_ok and use_cam else False
        cam_block = bool(self.cam_block) if cam_ok and use_cam else False
        if cam_cliff and not self._cam_cliff_logged:
            self.get_logger().warn('camera drop ahead — treat as wall (turn, no reverse)')
            self._cam_cliff_logged = True
        elif not cam_cliff:
            self._cam_cliff_logged = False

        imu_ok = self.age(self.last_imu_time) < self.timeout
        tilt = bool(self.tilt) if imu_ok else False
        pickup = bool(self.pickup) if imu_ok else False
        if tilt and not self._tilt_logged:
            self.get_logger().error('TILT/gyro — reverse then stop')
            self._tilt_logged = True
        elif not tilt:
            self._tilt_logged = False
        if pickup and not self._pickup_logged:
            self.get_logger().error('PICKUP — motors 0')
            self._pickup_logged = True
        elif not pickup:
            self._pickup_logged = False
        self.tilt_pub.publish(Bool(data=tilt))
        self.pickup_pub.publish(Bool(data=pickup))

        # IR cliff = over the edge (backup). Lidar/US = wall (turn).
        # Camera look-ahead is off unless camera_as_wall is set.
        obstacle = self.blocked or self.us_blocked or cam_block or cam_cliff
        self.block_pub.publish(Bool(data=obstacle))

        if pickup:
            self._publish_zero()
            return
        if self.age(self.last_cmd_time) > 0.5 and not tilt:
            self._publish_zero()
            return

        cmd = Twist()
        cmd.linear.x = self.last_cmd.linear.x
        cmd.angular.z = self.last_cmd.angular.z
        if tilt and not self.rear_blocked:
            cmd.linear.x = -0.003
            cmd.angular.z = 0.0
        elif tilt and self.rear_blocked:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        # Missing lidar must not freeze the robot; IR+US still work. Reverse still
        # needs a rear view when lidar is up.
        halt_fwd = obstacle or self.cliff or tilt
        if halt_fwd and cmd.linear.x > 0.0:
            cmd.linear.x = 0.0
        if cmd.linear.x < 0.0 and self.rear_blocked:
            cmd.linear.x = 0.0
        if (self.cliff or tilt) and cmd.linear.x >= 0.0:
            cmd.angular.z = 0.0
        # Hardware on this robot treats +x as reverse. Flip after the
        # semantic halt so "forward" still means drive toward the camera.
        cmd.linear.x *= self.cmd_linear_sign
        self.pub.publish(cmd)

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


def sector_min(scan: LaserScan, heading: float, half_width: float) -> float:
    angle = scan.angle_min
    best = float('inf')
    hi = scan.range_max if scan.range_max > 0.0 else 12.0
    for r in scan.ranges:
        # Keep hits inside range_min — C1 often reports 2–4 cm as < range_min=5 cm.
        if math.isfinite(r) and 0.0 < r < hi:
            if abs(wrap_pi(angle - heading)) <= half_width and r < best:
                best = r
        angle += scan.angle_increment
    return best


def main():
    rclpy.init()
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_motors()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
