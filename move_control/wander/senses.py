"""Subject: sensing. Read lidar/US/IR/camera/odom. No motion."""
import math
import random

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, UInt16MultiArray

from ..sensing.body import turn_clear_m, use_radius
from ..control.modes import nose_on_wall
from ..control.recover import have_turn_space, need_space_to_turn, ratio_sign, side_sign


def yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Senses:

    def on_ir(self, msg: UInt16MultiArray):
        self.ir = tuple(int(v) for v in msg.data[:3])

    def on_cliff(self, msg: Bool):
        self.cliff = bool(msg.data)

    def on_block(self, msg: Bool):
        self.blocked = bool(msg.data)

    def on_tilt(self, msg: Bool):
        self.tilt = bool(msg.data)

    def on_pickup(self, msg: Bool):
        self.pickup = bool(msg.data)

    def on_estop(self, msg: Bool):
        # Label truth only. Safety owns the e-stop: it zeros /cmd_vel and
        # forces /wander/cmd 'stop'; wander only reflects the latch here.
        self.estop = bool(msg.data)

    def on_rear(self, msg: Bool):
        self.rear_clear = bool(msg.data)

    def on_front_range(self, msg: Float32):
        v = float(msg.data)
        self.front_range = v if v >= 0.0 else float('inf')

    def on_rear_range(self, msg: Float32):
        v = float(msg.data)
        self.rear_range = v if v >= 0.0 else float('inf')

    def on_left(self, msg: Float32):
        v = float(msg.data)
        self.left_range = v if v >= 0.0 else float('inf')

    def on_right(self, msg: Float32):
        v = float(msg.data)
        self.right_range = v if v >= 0.0 else float('inf')

    def on_rear_left(self, msg: Float32):
        v = float(msg.data)
        self.rear_left = v if v >= 0.0 else float('inf')

    def on_rear_right(self, msg: Float32):
        v = float(msg.data)
        self.rear_right = v if v >= 0.0 else float('inf')

    def on_open(self, msg: Float32):
        v = float(msg.data)
        self.open_range = v if v >= 0.0 else float('inf')

    def on_open_yaw(self, msg: Float32):
        self.open_yaw = float(msg.data)

    def on_open_max(self, msg: Float32):
        v = float(msg.data)
        if math.isfinite(v) and 0.10 < v < 1.0:
            self.open_max = v

    def on_frontier_yaw(self, msg: Float32):
        self.frontier_yaw = float(msg.data)

    def on_frontier_range(self, msg: Float32):
        v = float(msg.data)
        self.frontier_range = v if v >= 0.0 else float('inf')

    def on_route_yaw(self, msg: Float32):
        self.route_yaw = float(msg.data)

    def on_route_range(self, msg: Float32):
        v = float(msg.data)
        self.route_range = v if v >= 0.0 else float('inf')

    def _route_aligned(self) -> bool:
        lim = math.radians(float(self.get_parameter('align_deg').value))
        yaw = float(getattr(self, 'route_yaw', 0.0) or 0.0)
        length = float(getattr(self, 'route_range', 0.0) or 0.0)
        wall = float(self.get_parameter('wall_front').value)
        return abs(yaw) <= lim and self._finite(length) and length > wall

    def on_cam_side(self, msg: Float32):
        self.cam_side = float(msg.data)

    def on_cam_block(self, msg: Bool):
        self.cam_block = bool(msg.data)

    def on_us(self, msg: Float32):
        v = float(msg.data)
        self.us_range = v if v >= 0.0 else float('inf')

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self.odom_x, self.odom_y = p.x, p.y
        self.odom_yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.odom_vx = float(msg.twist.twist.linear.x)
        self.have_odom = True

    def _finite(self, d) -> bool:
        return math.isfinite(d) and d >= 0.0

    def _local_ranges(self):
        """Front = closer of lidar and sonar. Left/right from lidar."""
        vals = {}
        front = None
        if self._finite(self.front_range):
            front = self.front_range
        if self._finite(self.us_range):
            front = self.us_range if front is None else min(front, self.us_range)
        if front is not None:
            vals['F'] = front
        if self._finite(self.left_range):
            vals['L'] = self.left_range
        if self._finite(self.right_range):
            vals['R'] = self.right_range
        return vals

    def _local_open(self):
        """Best gap at corridor scale. Drop lidar max if it is many times local."""
        vals = self._local_ranges()
        if not vals:
            return None, 0.0
        local_max = max(vals.values())
        cap = float(getattr(self, 'open_max', 0.40) or 0.40)
        local_max = min(local_max, cap)
        far = float(self.get_parameter('far_scale').value)
        if (
            self._finite(self.open_range)
            and self.open_range <= local_max * far
            and self.open_range >= local_max
        ):
            return self.open_range, self.open_yaw
        return local_max, {'F': 0.0, 'L': 1.0, 'R': -1.0}.get(
            max(vals, key=vals.get), 0.0
        ) * math.radians(70.0)

    def _pick_turn_sign(self) -> float:
        # Camera side only at a real corner (center obstacle), not corridor walls.
        s = side_sign(self.cam_side) if self.cam_block else 0.0
        if s:
            return s
        s = ratio_sign(self.left_range, self.right_range)
        if s:
            return s
        vals = self._local_ranges()
        if vals:
            wide = max(vals, key=vals.get)
            if wide == 'L':
                return 1.0
            if wide == 'R':
                return -1.0
        return random.choice((-1.0, 1.0))

    def _near_side(self):
        sides = []
        if self._finite(self.left_range):
            sides.append(self.left_range)
        if self._finite(self.right_range):
            sides.append(self.right_range)
        return min(sides) if sides else None

    def _far_side(self):
        sides = []
        if self._finite(self.left_range):
            sides.append(self.left_range)
        if self._finite(self.right_range):
            sides.append(self.right_range)
        return max(sides) if sides else None

    def _aligned_to_open(self) -> bool:
        if self.blocked or self.cam_block:
            return False
        if not self._finite(self.front_range):
            return False
        near = self._near_side()
        far = self._far_side()
        if near is None or far is None:
            return False
        if self._front_pinched() or self._on_wall():
            return False
        # Corridor scale: ignore an 8 m room hit as "the open side".
        far_lim = max(near, self.front_range, 0.05) * float(
            self.get_parameter('far_scale').value
        )
        local_open = min(far, far_lim)
        return self.front_range >= local_open * 0.75

    def _front_pinched(self) -> bool:
        """Dead-end / corner: front is close AND tighter than the nearer wall.

        Do not compare to the far side — a corridor with a side gap is not
        a pinch (that used to fire escape at F=50–90 cm).
        """
        if self.blocked:
            return True
        if not self._finite(self.front_range):
            return bool(self.cam_block)
        if self.front_range > self.escape_front:
            return False
        near = self._near_side()
        if near is None:
            return self.cam_block
        return self.front_range <= near * self.pinch_ratio

    def _sensors_ready(self) -> bool:
        return self._finite(self.front_range) or (
            self._finite(self.left_range) and self._finite(self.right_range)
        )

    def _turn_side_d(self, sign=None) -> float:
        """Object distance on the side we are turning into."""
        s = self.turn_sign if sign is None else sign
        d = self.left_range if s > 0.0 else self.right_range
        return d if self._finite(d) else float('inf')

    def _turn_tail_d(self, sign=None) -> float:
        """Rear corner that swings while turning."""
        s = self.turn_sign if sign is None else sign
        d = self.rear_right if s > 0.0 else self.rear_left
        return d if self._finite(d) else float('inf')

    def _turn_clear(self) -> float:
        r = use_radius(
            self.get_parameter('robot_radius').value
            if self.has_parameter('robot_radius') else None
        )
        return max(
            float(self.get_parameter('turn_clear_m').value),
            turn_clear_m(r),
        )

    def _have_turn_space(self, sign=None) -> bool:
        clear = self._turn_clear()
        front = self.front_range if self._finite(self.front_range) else float('inf')
        return have_turn_space(
            front, self._turn_side_d(sign), self._turn_tail_d(sign), clear
        )

    def _need_space_to_turn(self, sign=None) -> bool:
        """Too tight to spin: reverse first if the rear is clear."""
        n = int(getattr(self, '_turn_backs', 0))
        max_n = int(self.get_parameter('turn_back_max').value)
        front = self.front_range if self._finite(self.front_range) else float('inf')
        return need_space_to_turn(
            front,
            self._turn_side_d(sign),
            self._turn_tail_d(sign),
            self._turn_clear(),
            self._can_reverse(),
            n,
            max_n,
        )

    def _on_wall(self) -> bool:
        """Bumper contact from lidar/US only. Camera obstacle is a corner (ESCAPE)."""
        wall_d = float(self.get_parameter('wall_front').value)
        return nose_on_wall(self.front_range, self.us_range, wall_d)

    def _hugging(self) -> bool:
        vals = self._local_ranges()
        if 'L' not in vals or 'R' not in vals:
            return False
        wide = max(vals['L'], vals['R'])
        tight = min(vals['L'], vals['R'])
        if wide <= 1e-4:
            return False
        return tight / wide < float(self.get_parameter('hug_ratio').value)

    def _ir_ready(self) -> bool:
        if len(self.ir) < 3:
            return False
        # 4095 = ADC sat / lifted. Need a real floor reading before we drive.
        return sum(1 for v in self.ir if 1500 <= v < 4000) >= 2
