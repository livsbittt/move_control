"""Subject: contact sensing. Lidar sectors + US. Publish ranges."""
import math
from ..control.lidar_guard import scan_body_clearance

from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import Float32
from rclpy.time import Time
from tf2_ros import TransformException

from ..sensing.body import ignore_m, use_radius
from ..sensing.lidar import find_frontiers, is_robot_scan, opening_max, sector_range, wrap_pi
from ..control.route import line_route
from ..control.lidar_guard import directional_lidar_limits
from ..sensing.lidar_mount import nose_from_quaternion


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


class Bumper:

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
        if not is_robot_scan(msg):
            self._scan_drop += 1
            if self._scan_drop in (1, 20) or self._scan_drop % 200 == 0:
                self.get_logger().warn(
                    f'drop remote /scan ({self._scan_drop}) n={len(msg.ranges)} '
                    f'rmax={float(msg.range_max):.1f} stamp={msg.header.stamp.sec}'
                )
            return
        if bool(self.get_parameter('lidar_use_tf').value):
            try:
                if not msg.header.frame_id:
                    raise ValueError('Missing scan frame')
                tf = self.lidar_tf.lookup_transform(
                    'base_link', msg.header.frame_id, Time.from_msg(msg.header.stamp))
                q = tf.transform.rotation
                self.lidar_yaw = nose_from_quaternion(q.x, q.y, q.z, q.w)
                self.lidar_yaw_source = 'tf:' + msg.header.frame_id
                mount = tf.transform.translation
                self.lidar_mount = (mount.x, mount.y)
            except (TransformException, ValueError) as error:
                self.last_scan_time = None
                self.lidar_mount = None
                self.lidar_yaw_source = 'missing_tf'
                self.get_logger().warn(f'drop scan without mount TF: {error}',
                                       throttle_duration_sec=5.0)
                return
        else:
            self.lidar_yaw = float(self.get_parameter('lidar_yaw_offset').value)
            self.lidar_yaw_source = 'parameter'
            self.lidar_mount = None
        self._scan_ok += 1
        yaw = self.lidar_yaw
        lo = max(
            float(self.get_parameter('scan_ignore_m').value),
            ignore_m(getattr(self, 'robot_r', 0.076)),
        )
        # A percentile can discard the one beam touching a jamb. Bumper
        # braking uses the closest valid beam; display smoothing stays downstream.
        kw = dict(pctl=0.0, ignore_below=lo, max_r=12.0)
        self.lidar_front = sector_range(msg, yaw, self.half_w, **kw)
        self.lidar_rear = sector_range(msg, wrap_pi(yaw + math.pi), self.half_w, **kw)
        side = math.radians(70.0)
        side_w = math.radians(28.0)
        self.lidar_left = sector_range(msg, wrap_pi(yaw + side), side_w, **kw)
        self.lidar_right = sector_range(msg, wrap_pi(yaw - side), side_w, **kw)
        rear = wrap_pi(yaw + math.pi)
        self.lidar_rear_left = sector_range(msg, wrap_pi(rear + side), side_w, **kw)
        self.lidar_rear_right = sector_range(msg, wrap_pi(rear - side), side_w, **kw)
        self.lidar_rotation_clearance = None
        if bool(self.get_parameter('lidar_use_tf').value):
            mount = tf.transform.translation
            rotation = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            self.lidar_rotation_clearance = scan_body_clearance(
                msg.ranges, msg.angle_min, msg.angle_increment,
                mount.x, mount.y, rotation, max(lo,msg.range_min), min(12.,msg.range_max))
        cap = float(getattr(self, 'open_max', 0.40) or 0.40)
        self.open_range, self.open_yaw = opening_max(
            msg, yaw, math.radians(70.0), max_r=cap
        )
        occ = float(self.get_parameter('wall_front').value) if self.has_parameter('wall_front') else 0.08
        free = float(self.get_parameter('warn_front').value) if self.has_parameter('warn_front') else 0.11
        self.frontiers = find_frontiers(
            msg,
            yaw_offset=yaw,
            occ=max(0.10, occ),
            free=max(occ + 0.04, min(free, cap)),
            max_r=cap,
        )
        if self.frontiers:
            best = self.frontiers[0]
            self.frontier_yaw = float(best['yaw'])
            self.frontier_range = float(best['depth'])
        else:
            self.frontier_yaw = self.open_yaw
            self.frontier_range = self.open_range
        # Full-circle exit judge for the stuck robot: the same frontier runs
        # but all-around (front fan is ±110° — it cannot see the way the
        # robot came in, which is often the only way out of a pocket).
        # max_r 1.2 m: desk-maze scale, beyond that reads as runway.
        exits = find_frontiers(
            msg,
            yaw_offset=yaw,
            occ=max(0.10, occ),
            free=max(occ + 0.04, min(free, 1.2)),
            max_r=1.2,
            front_half=math.pi,
        )
        if exits:
            self.exit_yaw = float(exits[0]['yaw'])
            self.exit_range = float(exits[0]['depth'])
        else:
            self.exit_yaw, self.exit_range = self.open_yaw, 0.0
        line = line_route(
            msg,
            yaw_offset=yaw,
            occ=max(0.10, occ),
            max_r=cap,
        )
        if line is not None:
            self.route_yaw = float(line['yaw'])
            self.route_range = float(line['length'])
        else:
            self.route_yaw = self.frontier_yaw
            self.route_range = self.frontier_range
        self.last_scan_time = self.now()

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
        if not bool(self.get_parameter('lidar_use_tf').value):
            self.lidar_yaw = float(self.get_parameter('lidar_yaw_offset').value)
        if self.has_parameter('robot_radius'):
            self.robot_r = use_radius(self.get_parameter('robot_radius').value)
        # The 8-degree centre cone missed corners in the chassis path.
        self.half_w = max(math.pi / 4, self.half_w)
        front, rear = directional_lidar_limits(
            self.stop_d, self.clear_d, self.robot_r,
            getattr(self, 'lidar_mount', None) if self.get_parameter('lidar_use_tf').value else None,
            self.half_w)
        self.stop_d, self.clear_d = front
        self.rear_stop_d, self.rear_clear_d = rear
        fc = float(self.get_parameter('filt_hz').value)
        for lp in self._lp.values():
            lp.set_cutoff(fc, 0.05)

    def _filt(self, name, raw):
        v = raw if math.isfinite(raw) and raw > 0.0 else None
        y = self._lp[name].push(v)
        return y if y is not None else raw
