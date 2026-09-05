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
        self.declare_parameter('vmax', 0.08)
        self.declare_parameter('wmax', 0.6)
        self.declare_parameter('kp', 0.8)
        self.declare_parameter('kp_yaw', 1.2)
        self.declare_parameter('tolerance', 0.02)
        self.declare_parameter('yaw_tolerance_deg', 4.0)
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
        self.mode = None
        self.goal = 0.0
        self.start_x = self.start_y = self.yaw_start = 0.0
        self.get_logger().info(f'control_node ready | cmd={cmd_topic}')

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self.x, self.y = p.x, p.y
        self.yaw = yaw_from_quat(msg.pose.pose.orientation)
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
        wmax = float(self.get_parameter('wmax').value)
        if self.mode == 'straight':
            rem = self.goal - self.traveled()
            if abs(rem) < float(self.get_parameter('tolerance').value):
                self.mode = None
                self.get_logger().info(f'arrived {self.traveled():.3f} m')
            else:
                cmd.linear.x = max(-vmax, min(vmax, float(self.get_parameter('kp').value) * rem))
                if abs(cmd.linear.x) < 0.03 and abs(rem) > 0.01:
                    cmd.linear.x = 0.03 if rem > 0 else -0.03
        elif self.mode == 'rotate':
            err = wrap_pi(self.goal - wrap_pi(self.yaw - self.yaw_start))
            tol = math.radians(float(self.get_parameter('yaw_tolerance_deg').value))
            if abs(err) < tol:
                self.mode = None
                self.get_logger().info('rotate done')
            else:
                cmd.angular.z = max(-wmax, min(wmax, float(self.get_parameter('kp_yaw').value) * err))
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
