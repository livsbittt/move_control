#!/usr/bin/env python3
"""Desk wander with split recovery.

  forward  — drive ahead; crawl when lidar is close (thinking)
  pause    — brief stop to evaluate cliff vs wall
  backup   — reverse only if rear lidar is clear, slowly while deciding
  turn     — spin in place (walls, or after a cliff is clear)
  stop     — hold still until /wander/cmd start

Commands on /wander/cmd: stop | start | wander
"""
import math
import random

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String, UInt16MultiArray


class WanderNode(Node):
    def __init__(self):
        super().__init__('wander_node')
        self.declare_parameter('cmd_topic', '/cmd_vel_raw')
        self.declare_parameter('vmax', 0.018)
        self.declare_parameter('vmax_think', 0.005)
        self.declare_parameter('vback', 0.006)
        self.declare_parameter('vback_think', 0.002)
        self.declare_parameter('wturn', 0.12)
        self.declare_parameter('wturn_think', 0.06)
        self.declare_parameter('slow_front', 0.18)
        self.declare_parameter('slow_rear', 0.12)
        self.declare_parameter('stop_front', 0.04)
        self.declare_parameter('auto_start', True)
        self.declare_parameter('pause_sec', 0.40)
        self.declare_parameter('backup_min_sec', 0.20)
        self.declare_parameter('backup_clear_sec', 0.30)
        self.declare_parameter('backup_max_sec', 6.00)
        self.declare_parameter('turn_sec', 5.0)

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
        self.create_subscription(Bool, '/safety/cliff', self.on_cliff, 10)
        self.create_subscription(Bool, '/safety/blocked', self.on_block, 10)
        self.create_subscription(Bool, '/safety/tilt', self.on_tilt, 10)
        self.create_subscription(Bool, '/safety/pickup', self.on_pickup, 10)
        self.create_subscription(Bool, '/safety/rear_clear', self.on_rear, 10)
        self.create_subscription(Float32, '/safety/min_range', self.on_front_range, 10)
        self.create_subscription(Float32, '/safety/rear_range', self.on_rear_range, 10)
        self.create_subscription(UInt16MultiArray, '/ir_sensor/range', self.on_ir, 10)
        self.create_subscription(Float32, '/camera/side', self.on_cam_side, 10)
        self.create_subscription(String, '/wander/cmd', self.on_cmd, 10)
        self.create_subscription(Bool, '/wander/enable', self.on_enable, 10)
        self.create_timer(0.05, self.tick)

        self.cliff = False
        self.blocked = False
        self.tilt = False
        self.pickup = False
        self.rear_clear = False
        self.front_range = float('inf')
        self.rear_range = float('inf')
        self.ir = ()
        self.cam_side = 0.0
        self.turn_sign = 1.0
        self.t0 = None
        self.cliff_clear_since = None
        self.enabled = bool(self.get_parameter('auto_start').value)
        self.seen_forward = False
        self.state = 'wait' if self.enabled else 'stop'
        self.t0 = self.now()
        self.get_logger().info(
            f'wander ready vmax={self.vmax:.3f} think={float(self.get_parameter("vmax_think").value):.3f} | '
            'slow when deciding | rear lidar for backup | /wander/cmd stop|start'
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
        self.front_range = float(msg.data)

    def on_rear_range(self, msg: Float32):
        self.rear_range = float(msg.data)

    def on_cam_side(self, msg: Float32):
        self.cam_side = float(msg.data)

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

    def _pick_turn_sign(self) -> float:
        # Camera: positive side = danger on the right → turn left (+z).
        if self.cam_side >= 0.3:
            return 1.0
        if self.cam_side <= -0.3:
            return -1.0
        # low-mode IR: smaller = more cliff. Turn away from the darker side.
        if len(self.ir) >= 3:
            left, _, right = self.ir
            if left + 80 < right:
                return -1.0
            if right + 80 < left:
                return 1.0
        return random.choice((-1.0, 1.0))

    def _enter(self, state: str):
        prev = self.state
        self.state = state
        self.t0 = self.now()
        self.cliff_clear_since = None
        if state == 'backup':
            self.turn_sign = self._pick_turn_sign()
            self.get_logger().warn(
                f'backup (cliff IR={self.ir}) until clear, max {self.backup_max_sec:.2f}s'
            )
        elif state == 'turn':
            if prev not in ('backup', 'turn'):
                self.turn_sign = self._pick_turn_sign()
            self.get_logger().info(f'turn sign={self.turn_sign:.0f}')
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
        self.state_pub.publish(String(data=state))

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

    def _fwd_speed(self) -> float:
        return self._blend_speed(
            self.front_range,
            float(self.get_parameter('vmax_think').value),
            self.vmax,
            float(self.get_parameter('stop_front').value),
            float(self.get_parameter('slow_front').value),
        )

    def _back_speed(self) -> float:
        return self._blend_speed(
            self.rear_range,
            float(self.get_parameter('vback_think').value),
            self.vback,
            float(self.get_parameter('stop_front').value),
            float(self.get_parameter('slow_rear').value),
        )

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
        elif self.state == 'backup':
            self._tick_backup()
        elif self.state == 'turn':
            self._tick_turn()
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
        if self._ir_ready():
            self._enter('forward')

    def _tick_forward(self):
        if self.tilt or self.cliff or self.blocked:
            self._enter('pause')
            self._publish(Twist(), 'pause')
            return
        cmd = Twist()
        cmd.linear.x = self._fwd_speed()
        self.seen_forward = True
        self._publish(cmd, 'forward')

    def _tick_pause(self):
        cmd = Twist()
        if self.elapsed() < self.pause_sec:
            self._publish(cmd, 'pause')
            return
        if (self.tilt or (self.cliff and self.seen_forward)) and self.rear_clear:
            self._enter('backup')
            cmd.linear.x = -self._back_speed()
            self._publish(cmd, 'backup')
        elif self.tilt or self.cliff:
            # At start, turn away instead of reversing.
            self._enter('turn')
            cmd.angular.z = self._turn_rate() * self.turn_sign
            self._publish(cmd, 'turn')
        elif self.blocked:
            self._enter('turn')
            cmd.angular.z = self._turn_rate() * self.turn_sign
            self._publish(cmd, 'turn')
        else:
            self._enter('forward')
            cmd.linear.x = self._fwd_speed()
            self.seen_forward = True
            self._publish(cmd, 'forward')

    def _tick_backup(self):
        if not self.rear_clear:
            self.get_logger().warn('rear lidar blocked — stop reverse, turn')
            self._enter('turn')
            cmd = Twist()
            cmd.angular.z = self._turn_rate() * self.turn_sign
            self._publish(cmd, 'turn')
            return
        cmd = Twist()
        cmd.linear.x = -self._back_speed()
        elapsed = self.elapsed()

        if elapsed >= self.backup_max_sec:
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
            self._enter('turn')
            cmd = Twist()
            cmd.angular.z = self._turn_rate() * self.turn_sign
            self._publish(cmd, 'turn')
            return

        if self.cliff or self.tilt:
            self.cliff_clear_since = None
            self._publish(cmd, 'backup')
            return

        if self.cliff_clear_since is None:
            self.cliff_clear_since = self.now()
            self.get_logger().info(f'cliff clear, extra {self.backup_clear_sec:.2f}s then turn')
        clear_for = (self.now() - self.cliff_clear_since).nanoseconds * 1e-9
        if elapsed >= self.backup_min_sec and clear_for >= self.backup_clear_sec:
            self._enter('turn')
            cmd = Twist()
            cmd.angular.z = self._turn_rate() * self.turn_sign
            self._publish(cmd, 'turn')
            return
        self._publish(cmd, 'backup')

    def _tick_turn(self):
        if (self.cliff or self.tilt) and self.seen_forward and self.rear_clear:
            self._enter('backup')
            cmd = Twist()
            cmd.linear.x = -self._back_speed()
            self._publish(cmd, 'backup')
            return
        if (self.cliff or self.tilt) and self.seen_forward and not self.rear_clear:
            cmd = Twist()
            cmd.angular.z = self._turn_rate() * self.turn_sign
            self._publish(cmd, 'turn')
            return
        cmd = Twist()
        cmd.angular.z = self._turn_rate() * self.turn_sign
        if self.elapsed() >= self.turn_sec:
            if self.blocked:
                self._enter('turn')
            else:
                self._enter('forward')
                cmd = Twist()
                cmd.linear.x = self._fwd_speed()
                self.seen_forward = True
                self._publish(cmd, 'forward')
                return
        self._publish(cmd, 'turn')

    def _publish(self, cmd: Twist, state: str):
        self.pub.publish(cmd)
        self.state_pub.publish(String(data=state))

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
