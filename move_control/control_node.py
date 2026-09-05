#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64


def yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_raw')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('vmax', 0.014)
        self.declare_parameter('vmax_think', 0.003)
        self.declare_parameter('vmin', 0.003)
        self.declare_parameter('think_dist', 0.04)
        self.declare_parameter('wmax', 0.35)
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('kp_yaw', 1.4)
        self.declare_parameter('tolerance', 0.008)
        self.declare_parameter('yaw_tolerance_deg', 2.0)
        self.declare_parameter('odom_timeout', 0.5)
        cmd_topic = self.get_parameter('cmd_vel_topic').value
        self.pub = self.create_publisher(Twist, cmd_topic, 10)
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self.on_odom, 10)
        self.create_subscription(Float64, '/goal_distance', self.on_goal, 10)
        self.create_subscription(Float64, '/goal_rotate', self.on_rotate, 10)
        self.create_timer(0.05, self.tick)
        self.have_odom = False
        self.last_odom_time = self.get_clock().now()
        self.x = self.y = self.yaw = 0.0
        self.vx = 0.0
        self.mode = None
        self.goal = 0.0
        self.start_x = self.start_y = self.yaw_start = 0.0
        self.get_logger().info(f'control_node ready | cmd={cmd_topic}')

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self.x, self.y = p.x, p.y
        self.yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.vx = float(msg.twist.twist.linear.x)
        self.have_odom = True
        self.last_odom_time = self.get_clock().now()

    def on_goal(self, msg: Float64):
        if not self.have_odom:
            self.get_logger().warn('waiting odom')
            return
        self.mode = 'straight'
        self.goal = float(msg.data)
        self.start_x, self.start_y, self.yaw_start = self.x, self.y, self.yaw
        self.get_logger().info(f'straight {self.goal:.3f} m')

    def on_rotate(self, msg: Float64):
        if not self.have_odom:
            self.get_logger().warn('waiting odom')
            return
        self.mode = 'rotate'
        self.goal = math.radians(float(msg.data))
        self.yaw_start = self.yaw
        self.get_logger().info(f'rotate {msg.data:.1f} deg')

    def traveled(self) -> float:
        dx = self.x - self.start_x
        dy = self.y - self.start_y
        return dx * math.cos(self.yaw_start) + dy * math.sin(self.yaw_start)

    def tick(self):
        cmd = Twist()
        age = (self.get_clock().now() - self.last_odom_time).nanoseconds * 1e-9
        if (not self.have_odom) or age > float(self.get_parameter('odom_timeout').value):
            self.pub.publish(cmd)
            return
        vmax = float(self.get_parameter('vmax').value)
        v_think = float(self.get_parameter('vmax_think').value)
        vmin = float(self.get_parameter('vmin').value)
        think_d = float(self.get_parameter('think_dist').value)
        wmax = float(self.get_parameter('wmax').value)
        if self.mode == 'straight':
            rem = self.goal - self.traveled()
            if abs(rem) < float(self.get_parameter('tolerance').value):
                self.mode = None
                self.get_logger().info(f'arrived {self.traveled():.3f} m odom')
            else:
                cap = v_think if abs(rem) < think_d else vmax
                v = max(-cap, min(cap, float(self.get_parameter('kp').value) * rem))
                if abs(v) < vmin and abs(rem) > float(self.get_parameter('tolerance').value):
                    v = vmin if rem > 0 else -vmin
                if abs(self.vx) > cap * 1.25:
                    v = v_think if rem > 0 else -v_think
                cmd.linear.x = v
        elif self.mode == 'rotate':
            err = wrap_pi(self.goal - wrap_pi(self.yaw - self.yaw_start))
            tol = math.radians(float(self.get_parameter('yaw_tolerance_deg').value))
            if abs(err) < tol:
                self.mode = None
                self.get_logger().info(f'rotate done odom {math.degrees(wrap_pi(self.yaw - self.yaw_start)):.1f}deg')
            else:
                w_cap = 0.12 if abs(err) < math.radians(12.0) else wmax
                cmd.angular.z = max(-w_cap, min(w_cap, float(self.get_parameter('kp_yaw').value) * err))
        self.pub.publish(cmd)

    def stop(self):
        self.mode = None
        for _ in range(5):
            self.pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
