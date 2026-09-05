#!/usr/bin/env python3
"""Desk wander with split recovery.

  forward  — drive ahead; crawl when lidar is close
  pause    — brief stop to tell cliff vs wall
  look     — stop, sample L/R/F
  calc     — score openings, lock a plan (or look again if unsure)
  recon    — plan was wrong while turning; look again
  wall     — bumper/US wall: backup if rear clear, else spin; no forward
  backup   — reverse only if rear lidar is clear
  turn     — slow spin (cliff), locked sign
  escape   — slow spin toward the locked open side until front is clear
  stop     — hold still until /wander/cmd start

Commands on /wander/cmd: stop | start | wander
"""
import math
import random

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, Float32, String, UInt16MultiArray

from .modes import pick_mode


def wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class WanderNode(Node):
    def __init__(self):
        super().__init__('wander_node')
        self.declare_parameter('cmd_topic', '/cmd_vel_raw')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('vmax', 0.014)
        self.declare_parameter('vmax_think', 0.003)
        self.declare_parameter('vback', 0.004)
        self.declare_parameter('vback_think', 0.002)
        self.declare_parameter('wturn', 0.10)
        self.declare_parameter('wturn_think', 0.045)
        self.declare_parameter('slow_front', 0.05)
        self.declare_parameter('slow_rear', 0.04)
        self.declare_parameter('stop_front', 0.018)
        self.declare_parameter('stop_distance', 0.018)
        self.declare_parameter('wall_front', 0.12)
        self.declare_parameter('warn_front', 0.25)
        self.declare_parameter('think_horizon', 0.50)
        self.declare_parameter('auto_start', True)
        self.declare_parameter('pause_sec', 0.25)
        self.declare_parameter('look_sec', 1.00)
        self.declare_parameter('calc_sec', 0.50)
        self.declare_parameter('recon_sec', 1.20)
        self.declare_parameter('recon_max', 2)
        self.declare_parameter('backup_m', 0.025)
        self.declare_parameter('backup_max_m', 0.050)
        self.declare_parameter('backup_min_sec', 0.15)
        self.declare_parameter('backup_clear_sec', 0.20)
        self.declare_parameter('backup_max_sec', 4.00)
        self.declare_parameter('turn_deg', 70.0)
        self.declare_parameter('turn_sec', 4.0)
        self.declare_parameter('open_ratio', 2.0)
        self.declare_parameter('hug_ratio', 0.50)
        self.declare_parameter('pinch_ratio', 1.55)
        self.declare_parameter('escape_front', 0.12)
        self.declare_parameter('escape_front_min', 0.035)
        self.declare_parameter('escape_front_max', 0.28)
        self.declare_parameter('escape_hold_sec', 0.40)
        self.declare_parameter('auto_escape', True)
        self.declare_parameter('align_deg', 22.0)
        self.declare_parameter('far_scale', 3.0)
        self.declare_parameter('stuck_sec', 1.2)
        self.declare_parameter('stuck_m', 0.008)
        self.declare_parameter('steer_k', 0.9)
        self.declare_parameter('steer_wmax', 0.05)

        self.vmax = float(self.get_parameter('vmax').value)
        self.vback = float(self.get_parameter('vback').value)
        self.wturn = float(self.get_parameter('wturn').value)
        self.pause_sec = float(self.get_parameter('pause_sec').value)
        self.backup_min_sec = float(self.get_parameter('backup_min_sec').value)
        self.backup_clear_sec = float(self.get_parameter('backup_clear_sec').value)
        self.backup_max_sec = float(self.get_parameter('backup_max_sec').value)
        self.turn_sec = float(self.get_parameter('turn_sec').value)

        self.pub = self.create_publisher(
            Twist, self.get_parameter('cmd_topic').value, 10
        )
        self.state_pub = self.create_publisher(String, '/wander/state', 10)
        self.mode_pub = self.create_publisher(String, '/robot/mode', 10)
        self.safety_mode_pub = self.create_publisher(String, '/safety/mode', 10)
        self.create_subscription(Bool, '/safety/cliff', self.on_cliff, 10)
        self.create_subscription(Bool, '/safety/blocked', self.on_block, 10)
        self.create_subscription(Bool, '/safety/tilt', self.on_tilt, 10)
        self.create_subscription(Bool, '/safety/pickup', self.on_pickup, 10)
        self.create_subscription(Bool, '/safety/rear_clear', self.on_rear, 10)
        self.create_subscription(Float32, '/safety/min_range', self.on_front_range, 10)
        self.create_subscription(Float32, '/safety/rear_range', self.on_rear_range, 10)
        self.create_subscription(Float32, '/safety/left_range', self.on_left, 10)
        self.create_subscription(Float32, '/safety/right_range', self.on_right, 10)
        self.create_subscription(Float32, '/safety/rear_left', self.on_rear_left, 10)
        self.create_subscription(Float32, '/safety/rear_right', self.on_rear_right, 10)
        self.create_subscription(Float32, '/safety/open_range', self.on_open, 10)
        self.create_subscription(Float32, '/safety/open_yaw', self.on_open_yaw, 10)
        self.create_subscription(UInt16MultiArray, '/ir_sensor/range', self.on_ir, 10)
        self.create_subscription(Float32, '/camera/side', self.on_cam_side, 10)
        self.create_subscription(Bool, '/camera/blocked', self.on_cam_block, 10)
        self.create_subscription(Float32, '/safety/us_range', self.on_us, 10)
        self.create_subscription(String, '/wander/cmd', self.on_cmd, 10)
        self.create_subscription(Bool, '/wander/enable', self.on_enable, 10)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self.on_odom, 10
        )
        self.create_timer(0.05, self.tick)

        self.cliff = False
        self.blocked = False
        self.tilt = False
        self.pickup = False
        self.rear_clear = False
        self.front_range = float('inf')
        self.rear_range = float('inf')
        self.left_range = float('inf')
        self.right_range = float('inf')
        self.open_range = float('inf')
        self.open_yaw = 0.0
        self.ir = ()
        self.cam_side = 0.0
        self.cam_block = False
        self.us_range = float('inf')
        self.rear_left = float('inf')
        self.rear_right = float('inf')
        self.turn_sign = 1.0
        self.t0 = None
        self.cliff_clear_since = None
        self._pinch_t = None
        self._escape_adjust_t = None
        self._escape_entry_f = float('inf')
        self._look_n = 0
        self._look_L = self._look_R = self._look_F = 0.0
        self._look_nL = self._look_nR = self._look_nF = 0
        self._look_yaw = 0.0
        self._look_cam = 0.0
        self._samp_L = []
        self._samp_R = []
        self._samp_F = []
        self._wall_backed = False
        self._calc_tries = 0
        self._recon_n = 0
        self.escape_front = float(self.get_parameter('escape_front').value)
        self.pinch_ratio = float(self.get_parameter('pinch_ratio').value)
        self.open_ratio = float(self.get_parameter('open_ratio').value)
        self.enabled = bool(self.get_parameter('auto_start').value)
        self.seen_forward = False
        self.have_odom = False
        self.odom_x = self.odom_y = self.odom_yaw = 0.0
        self.odom_vx = 0.0
        self.seg_x = self.seg_y = self.seg_yaw = 0.0
        self.motion_x = self.motion_y = 0.0
        self.motion_t = None
        self.state = 'wait' if self.enabled else 'stop'
        self.t0 = self.now()
        self.get_logger().info(
            f'wander ready vmax={self.vmax:.3f} think={float(self.get_parameter("vmax_think").value):.3f} '
            f'stop={float(self.get_parameter("stop_front").value)*100:.1f}cm '
            f'backup={float(self.get_parameter("backup_m").value)*100:.1f}cm '
            f'turn={float(self.get_parameter("turn_deg").value):.0f}deg | '
            f'escape F<{self.escape_front*100:.0f}cm pinch={self.pinch_ratio:.2f} '
            f'open={self.open_ratio:.2f} look={float(self.get_parameter("look_sec").value):.2f}s'
        )

    def now(self):
        return self.get_clock().now()

    def elapsed(self) -> float:
        if self.t0 is None:
            return 1e9
        return (self.now() - self.t0).nanoseconds * 1e-9

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

    def on_enable(self, msg: Bool):
        self._set_enabled(bool(msg.data))

    def on_cmd(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd in ('stop', 'halt', 'off'):
            self._set_enabled(False)
        elif cmd in ('start', 'go', 'wander', 'resume', 'on', 'forward'):
            self._set_enabled(True)
        else:
            self.get_logger().warn(f'unknown cmd {cmd!r} (use stop|start)')

    def _set_enabled(self, enabled: bool):
        if not enabled:
            self.enabled = False
            if self.state != 'stop':
                self._enter('stop')
            return
        already = self.enabled and self.state != 'stop'
        self.enabled = True
        if already:
            return
        # Always start by going forward. Backup only after we have driven.
        self._enter('wait')

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
        if self.cam_block and abs(self.cam_side) >= 0.3:
            return 1.0 if self.cam_side > 0.0 else -1.0
        if self._finite(self.left_range) and self._finite(self.right_range):
            if self.left_range > self.right_range * 1.15:
                return 1.0
            if self.right_range > self.left_range * 1.15:
                return -1.0
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

    def _on_wall(self) -> bool:
        """Nose contact: bumper flag, or F/US ≤ wall_front. Not camera, not no-echo."""
        if self.blocked:
            return True
        wall_d = float(self.get_parameter('wall_front').value)
        if self._finite(self.us_range) and 0.0 < self.us_range <= wall_d and self.us_range < 0.80:
            return True
        if self._finite(self.front_range) and 0.0 < self.front_range <= wall_d:
            return True
        return False

    def _hugging(self) -> bool:
        vals = self._local_ranges()
        if 'L' not in vals or 'R' not in vals:
            return False
        wide = max(vals['L'], vals['R'])
        tight = min(vals['L'], vals['R'])
        if wide <= 1e-4:
            return False
        return tight / wide < float(self.get_parameter('hug_ratio').value)

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

    def _enter(self, state: str):
        prev = self.state
        self.state = state
        self.t0 = self.now()
        self.cliff_clear_since = None
        keep = prev in ('look', 'calc', 'recon')
        if state == 'backup':
            if not keep:
                self.turn_sign = self._pick_turn_sign()
            self.get_logger().warn(
                f'backup rear={self.rear_range:.3f}m lim={self._backup_limit():.3f}m '
                f'RL={self.rear_left:.2f} RR={self.rear_right:.2f} IR={self.ir}'
            )
        elif state == 'turn':
            if not keep:
                self.turn_sign = self._pick_turn_sign()
            self.get_logger().info(
                f'turn sign={self.turn_sign:.0f} open={self.open_range:.2f}m '
                f'yaw={math.degrees(self.open_yaw):.0f} '
                f'L={self.left_range:.2f} R={self.right_range:.2f}'
            )
        elif state == 'escape':
            if not keep:
                self.turn_sign = self._pick_turn_sign()
            self._escape_entry_f = self.front_range
            self.get_logger().warn(
                f'lidar escape F={self.front_range:.2f} L={self.left_range:.2f} '
                f'R={self.right_range:.2f} US={self.us_range:.2f} '
                f'cam={int(self.cam_block)} side={self.cam_side:.0f} '
                f'sign={self.turn_sign:.0f} Fmax={self.escape_front:.2f}'
            )
        elif state == 'look':
            self._look_n = 0
            self._look_L = self._look_R = self._look_F = 0.0
            self._look_nL = self._look_nR = self._look_nF = 0
            self._look_yaw = 0.0
            self._look_cam = 0.0
            self.get_logger().info(
                f'look F={self.front_range:.2f} L={self.left_range:.2f} '
                f'R={self.right_range:.2f} US={self.us_range:.2f}'
            )
            self.pub.publish(Twist())
        elif state == 'wall':
            self.get_logger().warn(
                f'wall recover F={self.front_range:.2f} US={self.us_range:.2f} '
                f'rear={self.rear_range:.2f} can_rev={int(self._can_reverse())} '
                f'sign={self.turn_sign:.0f}'
            )
        elif state == 'pause':
            self.get_logger().info(
                f'pause evaluate cliff={int(self.cliff)} wall={int(self.blocked)}'
            )
        elif state == 'wait':
            self.get_logger().info('wait for IR, then forward')
            self.pub.publish(Twist())
        elif state == 'stop':
            self.get_logger().info('stop')
            self.pub.publish(Twist())
        else:
            self.get_logger().info('forward')
            self._wall_backed = False
        if state in ('backup', 'turn', 'forward', 'escape', 'wall'):
            self._mark_pose()
            self.motion_t = self.now()
            self.motion_x, self.motion_y = self.odom_x, self.odom_y
        self.state_pub.publish(String(data=state))

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

    def _can_reverse(self) -> bool:
        """Rear lidar must show a path. No scan → no reverse."""
        if not self.rear_clear:
            return False
        stop = float(self.get_parameter('stop_front').value)
        if not self._finite(self.rear_range) or self.rear_range <= stop:
            return False
        return True

    def _backup_limit(self) -> float:
        """How far we may reverse: a fraction of the free rear gap."""
        stop = float(self.get_parameter('stop_front').value)
        cap = float(self.get_parameter('backup_max_m').value)
        if not self._finite(self.rear_range):
            return min(cap, 0.02)
        gap = max(0.0, self.rear_range - stop)
        return max(0.01, min(cap, 0.45 * gap))

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

    def _turn_rate(self) -> float:
        thinking = (
            self.cliff or self.blocked or self.tilt
            or (0.0 < self.front_range < float(self.get_parameter('slow_front').value))
        )
        w_think = float(self.get_parameter('wturn_think').value)
        return w_think if thinking else self.wturn

    def tick(self):
        self.vmax = float(self.get_parameter('vmax').value)
        self.vback = float(self.get_parameter('vback').value)
        self.wturn = float(self.get_parameter('wturn').value)
        self.escape_front = float(self.get_parameter('escape_front').value)
        self.pinch_ratio = float(self.get_parameter('pinch_ratio').value)
        self.open_ratio = float(self.get_parameter('open_ratio').value)
        if not self.enabled or self.state == 'stop':
            self._publish(Twist(), 'stop')
            return
        if self.pickup:
            if self.state != 'stop':
                self.get_logger().error('pickup — stop')
                self._set_enabled(False)
            self._publish(Twist(), 'stop')
            return
        if self.state == 'wait':
            self._tick_wait()
        elif self.state == 'forward':
            self._tick_forward()
        elif self.state == 'pause':
            self._tick_pause()
        elif self.state == 'look':
            self._tick_look()
        elif self.state == 'wall':
            self._tick_wall()
        elif self.state == 'backup':
            self._tick_backup()
        elif self.state == 'turn':
            self._tick_turn()
        elif self.state == 'escape':
            self._tick_escape()
        else:
            self._enter('stop')
            self._publish(Twist(), 'stop')

    def _ir_ready(self) -> bool:
        if len(self.ir) < 3:
            return False
        # 4095 = ADC sat / lifted. Need a real floor reading before we drive.
        return sum(1 for v in self.ir if 1500 <= v < 4000) >= 2

    def _tick_wait(self):
        self._publish(Twist(), 'wait')
        if self._ir_ready() and (self._sensors_ready() or self.elapsed() > 2.0):
            self._enter('forward')

    def _tick_forward(self):
        self._note_motion()
        if self.tilt or self.cliff or self.blocked or self._on_wall():
            self._enter('pause')
            self._publish(Twist(), 'pause')
            return
        if self._is_stuck():
            self._escape_need()
            self._enter('look')
            self._publish(Twist(), 'look')
            return
        if self._want_lidar_escape():
            self._enter('look')
            self._publish(Twist(), 'look')
            return
        cmd = Twist()
        cmd.linear.x = self._fwd_speed()
        cmd.angular.z = self._steer_wz()
        self.seen_forward = True
        self._publish(cmd, 'forward')

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

    def _look_accum(self):
        self._look_n += 1
        if self._finite(self.left_range):
            self._look_L += self.left_range
            self._look_nL += 1
        if self._finite(self.right_range):
            self._look_R += self.right_range
            self._look_nR += 1
        if self._finite(self.front_range):
            self._look_F += self.front_range
            self._look_nF += 1
        self._look_yaw += self.open_yaw
        self._look_cam += self.cam_side

    def _look_sign(self) -> float:
        if self.cam_block and self._look_n > 0:
            side = self._look_cam / self._look_n
            if abs(side) >= 0.3:
                return 1.0 if side > 0.0 else -1.0
        L = self._look_L / self._look_nL if self._look_nL else None
        R = self._look_R / self._look_nR if self._look_nR else None
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

    def _tick_look(self):
        """Stop and average sensors, then lock a direction."""
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
        self.turn_sign = self._look_sign()
        L = self._look_L / self._look_nL if self._look_nL else float('nan')
        R = self._look_R / self._look_nR if self._look_nR else float('nan')
        F = self._look_F / self._look_nF if self._look_nF else float('nan')
        self.get_logger().info(
            f'look done n={self._look_n} F={F:.2f} L={L:.2f} R={R:.2f} '
            f'sign={self.turn_sign:.0f}'
        )
        if (self.tilt or (self.cliff and self.seen_forward)) and self._can_reverse():
            self._enter('backup')
            cmd = Twist()
            cmd.linear.x = -self._back_speed()
            cmd.angular.z = self._rear_steer_wz()
            self._publish(cmd, 'backup')
            return
        if self.tilt or self.cliff:
            self._enter('turn')
            cmd = Twist()
            cmd.angular.z = float(self.get_parameter('wturn_think').value) * self.turn_sign
            self._publish(cmd, 'turn')
            return
        if self._on_wall() or self.blocked:
            self._enter('wall')
            self._publish(Twist(), 'wall')
            return
        if (
            self._aligned_to_open()
            and not self._front_pinched()
        ):
            self._enter('forward')
            cmd = Twist()
            cmd.linear.x = self._fwd_speed()
            cmd.angular.z = self._steer_wz()
            self.seen_forward = True
            self._publish(cmd, 'forward')
            return
        self._enter('escape')
        cmd = Twist()
        cmd.angular.z = float(self.get_parameter('wturn_think').value) * self.turn_sign
        self._publish(cmd, 'escape')

    def _tick_wall(self):
        """Wall bumper: reverse off it if the tail is clear, else spin away."""
        if self.cliff or self.tilt:
            self._enter('pause')
            self._publish(Twist(), 'pause')
            return
        if not self._on_wall() and not self.blocked and self._aligned_to_open():
            self._enter('forward')
            cmd = Twist()
            cmd.linear.x = self._fwd_speed()
            cmd.angular.z = self._steer_wz()
            self.seen_forward = True
            self._publish(cmd, 'forward')
            return
        if self._can_reverse() and not self._wall_backed:
            self._wall_backed = True
            self._enter('backup')
            cmd = Twist()
            cmd.linear.x = -self._back_speed()
            cmd.angular.z = self._rear_steer_wz()
            self._publish(cmd, 'backup')
            return
        self._enter('escape')
        cmd = Twist()
        cmd.angular.z = float(self.get_parameter('wturn_think').value) * self.turn_sign
        self._publish(cmd, 'escape')

    def _tick_backup(self):
        if not self._can_reverse():
            self.get_logger().warn(
                f'rear not clear — no reverse lidar={self.rear_range:.3f}m '
                f'RL={self.rear_left:.2f} RR={self.rear_right:.2f}'
            )
            self._enter('look')
            self._publish(Twist(), 'look')
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
            self._enter('look')
            self._publish(Twist(), 'look')
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
        min_m = (not self.have_odom) or traveled >= min(limit, 0.012)
        if elapsed >= self.backup_min_sec and clear_for >= self.backup_clear_sec and min_m:
            self._enter('look')
            self._publish(Twist(), 'look')
            return
        self._publish(cmd, 'backup')

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
            cmd.angular.z = self._turn_rate() * self.turn_sign
            self._publish(cmd, 'turn')
            return
        if self.blocked or self._on_wall():
            self._enter('wall')
            self._publish(Twist(), 'wall')
            return
        cmd = Twist()
        w = float(self.get_parameter('wturn_think').value)
        cmd.angular.z = w * self.turn_sign
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
        cmd = Twist()
        w = float(self.get_parameter('wturn_think').value)
        if abs(self.open_yaw) < math.radians(12.0):
            w *= 0.45
        cmd.angular.z = w * self.turn_sign
        if self.elapsed() >= 0.40 and self._aligned_to_open() and not self._on_wall():
            if self.elapsed() < 0.85 and self._turned() < math.radians(20.0):
                self._escape_false()
            self._enter('forward')
            out = Twist()
            out.linear.x = self._fwd_speed()
            out.angular.z = self._steer_wz()
            self.seen_forward = True
            self._publish(out, 'forward')
            return
        if self.elapsed() > 2.5 and self._on_wall() and self._can_reverse() and not self._wall_backed:
            self._wall_backed = True
            self._enter('backup')
            out = Twist()
            out.linear.x = -self._back_speed()
            out.angular.z = self._rear_steer_wz()
            self._publish(out, 'backup')
            return
        if self.elapsed() > 6.0 and self._on_wall():
            self.turn_sign = -self.turn_sign
            self._mark_pose()
            self.t0 = self.now()
            self.get_logger().warn(f'wall still — flip escape sign={self.turn_sign:.0f}')
        if self.elapsed() > 5.0 and not self.blocked and not self._on_wall():
            if self._turned() < math.radians(15.0):
                self._escape_false()
            self._enter('forward')
            out = Twist()
            out.linear.x = self._fwd_speed()
            self.seen_forward = True
            self._publish(out, 'forward')
            return
        self._publish(cmd, 'escape')

    def _mode_label(self) -> str:
        return pick_mode(
            pickup=self.pickup,
            cliff=self.cliff,
            tilt=self.tilt,
            wander_state=self.state,
            on_wall=self._on_wall(),
            front=self.front_range,
            us=self.us_range,
            wall_front=float(self.get_parameter('wall_front').value),
            warn_front=float(self.get_parameter('warn_front').value),
        )

    def _publish(self, cmd: Twist, state: str):
        self.pub.publish(cmd)
        self.state_pub.publish(String(data=state))
        mode = String(data=self._mode_label())
        self.mode_pub.publish(mode)
        self.safety_mode_pub.publish(mode)

    def stop_motors(self):
        try:
            self.enabled = False
            self.state = 'stop'
            self.pub.publish(Twist())
            self.state_pub.publish(String(data='stop'))
        except Exception:
            pass


def main():
    rclpy.init()
    node = WanderNode()
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
