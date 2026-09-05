#!/usr/bin/env python3
"""Wander ROS node. Subjects: senses, judge, contact, motion, idle."""
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String, UInt16MultiArray

from ..body import URDF_RADIUS
from ..modes import pick_mode
from .contact import Contact
from .judge import Judge
from .motion import Motion
from .senses import Senses


class WanderNode(Node, Senses, Judge, Contact, Motion):

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
        self.declare_parameter('slow_front', 0.04)
        self.declare_parameter('slow_rear', 0.04)
        self.declare_parameter('stop_front', 0.018)
        self.declare_parameter('stop_distance', 0.018)
        self.declare_parameter('wall_front', 0.08)
        self.declare_parameter('warn_front', 0.11)
        self.declare_parameter('think_horizon', 0.50)
        self.declare_parameter('auto_start', True)
        self.declare_parameter('pause_sec', 0.25)
        self.declare_parameter('look_sec', 0.40)
        self.declare_parameter('calc_sec', 0.20)
        self.declare_parameter('recon_sec', 1.20)
        self.declare_parameter('recon_max', 2)
        self.declare_parameter('backup_m', 0.025)
        self.declare_parameter('backup_max_m', 0.12)
        self.declare_parameter('backup_min_sec', 0.15)
        self.declare_parameter('backup_clear_sec', 0.20)
        self.declare_parameter('backup_max_sec', 4.00)
        self.declare_parameter('turn_deg', 70.0)
        self.declare_parameter('turn_sec', 4.0)
        self.declare_parameter('open_ratio', 2.0)
        self.declare_parameter('hug_ratio', 0.50)
        self.declare_parameter('pinch_ratio', 1.55)
        self.declare_parameter('escape_front', 0.08)
        self.declare_parameter('escape_front_min', 0.040)
        self.declare_parameter('escape_front_max', 0.14)
        self.declare_parameter('escape_hold_sec', 0.40)
        self.declare_parameter('auto_escape', True)
        self.declare_parameter('align_deg', 22.0)
        self.declare_parameter('far_scale', 1.8)
        self.declare_parameter('open_max', 0.40)
        self.declare_parameter('stuck_sec', 1.2)
        self.declare_parameter('stuck_m', 0.008)
        self.declare_parameter('steer_k', 0.9)
        self.declare_parameter('steer_wmax', 0.05)
        self.declare_parameter('vacuum_follow', True)
        self.declare_parameter('turn_clear_m', 0.08)
        self.declare_parameter('turn_back_max', 2)
        self.declare_parameter('robot_radius', URDF_RADIUS)

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
        self.create_subscription(Float32, '/safety/open_max', self.on_open_max, 10)
        self.create_subscription(Float32, '/safety/frontier_yaw', self.on_frontier_yaw, 10)
        self.create_subscription(Float32, '/safety/frontier_range', self.on_frontier_range, 10)
        self.create_subscription(Float32, '/safety/route_yaw', self.on_route_yaw, 10)
        self.create_subscription(Float32, '/safety/route_range', self.on_route_range, 10)
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
        self.open_max = float(self.get_parameter('open_max').value)
        self.frontier_yaw = 0.0
        self.frontier_range = float('inf')
        self.route_yaw = 0.0
        self.route_range = float('inf')
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
        self._turn_backs = 0
        self._calc_tries = 0
        self._recon_n = 0
        self._samp_rear = []
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
        self._stuck_n = 0
        self._from_stuck = False
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
            self._stop_n = 0
            if self.state != 'stop':
                self._enter('stop')
            return
        already = self.enabled and self.state != 'stop'
        self.enabled = True
        if already:
            return
        # Always start by going forward. Backup only after we have driven.
        self._enter('wait')

    def _enter(self, state: str):
        prev = self.state
        self.state = state
        self.t0 = self.now()
        self.cliff_clear_since = None
        keep = prev in ('look', 'calc', 'recon') or getattr(self, '_from_stuck', False)
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
            self._samp_L = []
            self._samp_R = []
            self._samp_F = []
            self._samp_rear = []
            self.get_logger().info(
                f'look F={self.front_range:.2f} L={self.left_range:.2f} '
                f'R={self.right_range:.2f} US={self.us_range:.2f}'
            )
            self.pub.publish(Twist())
        elif state == 'calc':
            self.get_logger().info(
                f'calc n={self._look_n} F={self.front_range:.2f} '
                f'L={self.left_range:.2f} R={self.right_range:.2f}'
            )
            self.pub.publish(Twist())
        elif state == 'recon':
            self._recon_n += 1
            self.get_logger().warn(
                f'recon #{self._recon_n} F={self.front_range:.2f} '
                f'L={self.left_range:.2f} R={self.right_range:.2f} '
                f'old_sign={self.turn_sign:.0f}'
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
            self._turn_backs = 0
            self._calc_tries = 0
            self._recon_n = 0
        if state in ('backup', 'turn', 'forward', 'escape', 'wall'):
            self._mark_pose()
            self.motion_t = self.now()
            self.motion_x, self.motion_y = self.odom_x, self.odom_y
        self.state_pub.publish(String(data=state))

    def tick(self):
        self.vmax = float(self.get_parameter('vmax').value)
        self.vback = float(self.get_parameter('vback').value)
        self.wturn = float(self.get_parameter('wturn').value)
        self.escape_front = float(self.get_parameter('escape_front').value)
        self.pinch_ratio = float(self.get_parameter('pinch_ratio').value)
        self.open_ratio = float(self.get_parameter('open_ratio').value)
        if not self.enabled or self.state == 'stop':
            # A few zero cmds, then release /cmd_vel_raw so teleop can own it.
            n = getattr(self, '_stop_n', 0)
            if n < 4:
                self._stop_n = n + 1
                self._publish(Twist(), 'stop')
            else:
                self.state_pub.publish(String(data='stop'))
                mode = String(data=self._mode_label())
                self.mode_pub.publish(mode)
                self.safety_mode_pub.publish(mode)
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
        elif self.state == 'calc':
            self._tick_calc()
        elif self.state == 'recon':
            self._tick_recon()
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
