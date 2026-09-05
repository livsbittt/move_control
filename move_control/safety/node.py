#!/usr/bin/env python3
"""Safety ROS node. Subjects: bumper, hazard, gate."""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan, Range
from std_msgs.msg import Bool, Float32, String, UInt16MultiArray

from ..filt import IrMedian, MedianLp
from ..body import URDF_RADIUS, use_radius
from ..lidar import NOSE_YAW
from .bumper import Bumper
from .gate import Gate
from .hazard import Hazard
from .scale import Scale


class SafetyNode(Node, Bumper, Hazard, Gate, Scale):

    def __init__(self):
        super().__init__('safety_node')
        self.declare_parameter('cmd_in', '/cmd_vel_raw')
        self.declare_parameter('cmd_out', '/cmd_vel')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('us_topic', '/us_sensor/range')
        self.declare_parameter('ir_topic', '/ir_sensor/range')
        self.declare_parameter('stop_distance', 0.018)
        self.declare_parameter('clear_distance', 0.028)
        self.declare_parameter('us_stop_distance', 0.020)
        self.declare_parameter('us_clear_distance', 0.028)
        self.declare_parameter('front_half_width_deg', 8.0)
        self.declare_parameter('lidar_yaw_offset', NOSE_YAW)
        self.declare_parameter('scan_pctl', 0.10)
        self.declare_parameter('scan_ignore_m', 0.04)
        self.declare_parameter('sensor_timeout', 1.0)
        self.declare_parameter('us_scale', 1.0)
        self.declare_parameter('us_invalid_hold', 5)
        self.declare_parameter('us_hits', 3)
        self.declare_parameter('cmd_linear_sign', 1.0)
        self.declare_parameter('auto_linear_sign', False)
        self.declare_parameter('camera_as_wall', False)
        self.declare_parameter('camera_block_as_wall', True)
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
        self.declare_parameter('filt_median', 5)
        self.declare_parameter('filt_hz', 1.5)
        self.declare_parameter('auto_map', True)
        self.declare_parameter('map_range', 0.28)
        self.declare_parameter('map_range_min', 0.18)
        self.declare_parameter('map_range_max', 0.40)
        self.declare_parameter('open_max', 0.40)
        self.declare_parameter('robot_radius', URDF_RADIUS)
        self.declare_parameter('wall_front', 0.08)
        self.declare_parameter('warn_front', 0.11)

        self.stop_d = float(self.get_parameter('stop_distance').value)
        self.clear_d = float(self.get_parameter('clear_distance').value)
        self.us_stop = float(self.get_parameter('us_stop_distance').value)
        self.us_clear = float(self.get_parameter('us_clear_distance').value)
        self.half_w = math.radians(float(self.get_parameter('front_half_width_deg').value))
        self.lidar_yaw = float(self.get_parameter('lidar_yaw_offset').value)
        self.timeout = float(self.get_parameter('sensor_timeout').value)
        self.cmd_out = self.get_parameter('cmd_out').value
        nmed = max(1, int(self.get_parameter('filt_median').value))
        fchz = float(self.get_parameter('filt_hz').value)
        dt = 0.05
        self._lp = {
            k: MedianLp(nmed, fchz, dt)
            for k in (
                'front', 'rear', 'us', 'left', 'right',
                'rear_left', 'rear_right',
            )
        }
        self._ir_f = IrMedian(nmed)

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
        self.left_pub = self.create_publisher(Float32, '/safety/left_range', 10)
        self.right_pub = self.create_publisher(Float32, '/safety/right_range', 10)
        self.rear_left_pub = self.create_publisher(Float32, '/safety/rear_left', 10)
        self.rear_right_pub = self.create_publisher(Float32, '/safety/rear_right', 10)
        self.can_rev_pub = self.create_publisher(Bool, '/safety/can_reverse', 10)
        self.open_pub = self.create_publisher(Float32, '/safety/open_range', 10)
        self.open_yaw_pub = self.create_publisher(Float32, '/safety/open_yaw', 10)
        self.map_pub = self.create_publisher(Float32, '/safety/map_range', 10)
        self.open_max_pub = self.create_publisher(Float32, '/safety/open_max', 10)
        self.corr_pub = self.create_publisher(Float32, '/safety/corridor', 10)
        self.frontier_yaw_pub = self.create_publisher(Float32, '/safety/frontier_yaw', 10)
        self.frontier_pub = self.create_publisher(Float32, '/safety/frontier_range', 10)
        self.route_yaw_pub = self.create_publisher(Float32, '/safety/route_yaw', 10)
        self.route_pub = self.create_publisher(Float32, '/safety/route_range', 10)
        self.radius_pub = self.create_publisher(Float32, '/safety/robot_radius', 10)
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
        self.lidar_left = float('inf')
        self.lidar_right = float('inf')
        self.lidar_rear_left = float('inf')
        self.lidar_rear_right = float('inf')
        self.open_range = float('inf')
        self.open_yaw = 0.0
        self.frontiers = []
        self.frontier_yaw = 0.0
        self.frontier_range = float('inf')
        self.route_yaw = 0.0
        self.route_range = float('inf')
        self.map_range = float(self.get_parameter('map_range').value)
        self.open_max = float(self.get_parameter('open_max').value)
        self.robot_r = use_radius(self.get_parameter('robot_radius').value)
        self._corr_buf = []
        self._scan_drop = 0
        self._scan_ok = 0
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
        self._cam_block_logged = False
        self._tilt_logged = False
        self._pickup_logged = False
        self._us_invalid = 0
        self._us_hits = 0
        self._imu_has_g = False
        self._sign_t = None
        self._sign_front0 = None
        self._sign_vx0 = 0.0
        self._sign_flip_t = None
        self._sign_hits = 0
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
            f'cam_block={int(bool(self.get_parameter("camera_block_as_wall").value))} '
            f'cliff_mode={self.get_parameter("cliff_mode").value} '
            f'th={self.get_parameter("cliff_raw_max").value} '
            f'lidar_yaw={math.degrees(self.lidar_yaw):.0f}deg '
            f'radius={self.robot_r*100:.1f}cm '
            f'filt={int(self.get_parameter("filt_median").value)}/'
            f'{float(self.get_parameter("filt_hz").value):.1f}Hz | '
            'E-STOP /estop Bool or /estop/cmd stop|release'
        )
        self.estop_pub.publish(Bool(data=False))

    def now(self):
        return self.get_clock().now()

    def age(self, stamp):
        if stamp is None:
            return 1e9
        return (self.now() - stamp).nanoseconds / 1e9

    def tick(self):
        self._refresh_distances()
        if self.estop:
            self.raw_zero_pub.publish(Twist())
            self._publish_zero()
            self.estop_pub.publish(Bool(data=True))
            return
        lidar_d = self._filt('front', self.lidar_distance())
        us = self._filt('us', self.us_distance())
        left = self._filt('left', self.lidar_left)
        right = self._filt('right', self.lidar_right)
        self._scale_update(left, right)
        rleft = self._filt('rear_left', self.lidar_rear_left)
        rright = self._filt('rear_right', self.lidar_rear_right)
        # Speed/think uses lidar only. US flicker at 2 cm must not look like a wall 2 cm away.
        self.range_pub.publish(Float32(data=float(lidar_d if math.isfinite(lidar_d) else -1.0)))
        self.us_pub.publish(Float32(data=float(us if math.isfinite(us) else -1.0)))
        def _m(v):
            return float(v if math.isfinite(v) else -1.0)
        self.left_pub.publish(Float32(data=_m(left)))
        self.right_pub.publish(Float32(data=_m(right)))
        self.rear_left_pub.publish(Float32(data=_m(rleft)))
        self.rear_right_pub.publish(Float32(data=_m(rright)))
        self.open_pub.publish(Float32(data=_m(self.open_range)))
        self.open_yaw_pub.publish(Float32(data=float(self.open_yaw)))
        self.frontier_yaw_pub.publish(Float32(data=float(self.frontier_yaw)))
        self.frontier_pub.publish(Float32(data=_m(self.frontier_range)))
        self.route_yaw_pub.publish(Float32(data=float(self.route_yaw)))
        self.route_pub.publish(Float32(data=_m(self.route_range)))
        self.radius_pub.publish(Float32(data=float(self.robot_r)))

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
        rear_d = self._filt('rear', self.rear_distance())
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
        can_rev = lidar_ok and not self.rear_blocked and rear_d > self.stop_d
        self.rear_clear_pub.publish(Bool(data=can_rev))
        self.can_rev_pub.publish(Bool(data=can_rev))

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
        use_cam_block = use_cam or bool(self.get_parameter('camera_block_as_wall').value)
        cam_cliff = bool(self.cam_cliff) if cam_ok and use_cam else False
        cam_block = bool(self.cam_block) if cam_ok and use_cam_block else False
        if cam_block and not self._cam_block_logged:
            self.get_logger().warn('camera obstacle / corner — turn, no reverse')
            self._cam_block_logged = True
        elif not cam_block:
            self._cam_block_logged = False
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

        # IR cliff = reverse. Lidar/US = bumper wall. Camera block = corner look-ahead (turn).
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
            if cmd.linear.x >= 0.0:
                cmd.linear.x = -0.003
        elif tilt and self.rear_blocked:
            cmd.linear.x = 0.0

        # Missing lidar must not freeze the robot; IR+US still work. Reverse still
        # needs a rear view when lidar is up. Cliff/tilt must not zero spin —
        # if reverse is illegal, in-place turn is the only move.
        halt_fwd = obstacle or self.cliff or tilt
        if halt_fwd and cmd.linear.x > 0.0:
            cmd.linear.x = 0.0
        if cmd.linear.x < 0.0 and self.rear_blocked:
            cmd.linear.x = 0.0
        if abs(cmd.linear.x) >= 0.004:
            self._auto_linear_sign(us, lidar_d, self.last_cmd.linear.x, self.last_cmd.angular.z)
        else:
            self._sign_t = None
            self._sign_front0 = None
            self._sign_hits = 0
        # Apply after semantic halt: +raw means nose-forward.
        cmd.linear.x *= self.cmd_linear_sign
        self.pub.publish(cmd)


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
