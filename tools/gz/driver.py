#!/usr/bin/env python3
"""gz test driver: follow /route, fix sim TF frames, report ETA vs actual.

The planning stack's first closed-loop test in Gazebo. Subscribes the
planner outputs and drives the diff-drive robot; publishes the TF glue the
sim needs (odom->base_link from /odom, lidar frame->base_link static once
the first scan names its frame).
"""
import math
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))  # tools/gz -> repo root

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32, String
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time
from rclpy.duration import Duration


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class Driver(Node):
    def __init__(self):
        super().__init__('gz_driver')
        self.declare_parameter('v', 0.08)
        self.declare_parameter('w', 0.6)
        self.declare_parameter('goal_tol', 0.08)
        self.v = float(self.get_parameter('v').value)
        self.w = float(self.get_parameter('w').value)
        self.goal_tol = float(self.get_parameter('goal_tol').value)
        self.stb_sent = False
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(Path, '/route', self.on_route, 10)
        self.create_subscription(
            PoseStamped, '/goal_point', self.on_goal, 10)
        self.create_subscription(Float32, '/goal/eta', self.on_eta, 10)
        self.create_subscription(String, '/goal_node/state', self.on_state, 10)
        from sensor_msgs.msg import LaserScan
        self.create_subscription(
            LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tfb = TransformBroadcaster(self)
        self.stb = StaticTransformBroadcaster(self)
        self.x = self.y = self.yaw = 0.0
        self.have_odom = False
        self.wps = []
        self.wi = 0
        self.goal = None
        self.eta = None
        self.state = ''
        self.t_start = None
        self.t_goal_set = None
        self.reached = 0
        self.timer = self.create_timer(0.05, self.tick)

    def on_scan(self, msg):
        frame = msg.header.frame_id
        if self.stb_sent:
            return
        self.get_logger().info(f'scan frame {frame!r} -> static tf')
        t = TransformStamped()
        t.header.stamp = Time().to_msg()
        t.header.frame_id = frame
        t.child_frame_id = 'base_link'  # lidar centred: identity is fine
        t.transform.rotation.w = 1.0
        self.stb.sendTransform(t)
        self.stb_sent = True

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.x, self.y = p.x, p.y
        self.yaw = yaw_from_quat(msg.pose.pose.orientation)
        if not self.have_odom:
            self.have_odom = True
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = msg.pose.pose.orientation
        self.tfb.sendTransform(t)

    def on_route(self, msg):
        self.wps = [(ps.pose.position.x, ps.pose.position.y)
                    for ps in msg.poses]
        self.wi = 0
        if self.wps and self.goal is None:
            pass

    def on_eta(self, msg):
        self.eta = msg.data

    def on_state(self, msg):
        self.state = msg.data
        if self.t_start is None and 'explore' in self.state:
            self.t_start = self.now()

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def tick(self):
        cmd = Twist()
        if not self.have_odom or not self.wps:
            self.pub.publish(cmd)
            return
        tx, ty = self.wps[self.wi]
        d = math.hypot(tx - self.x, ty - self.y)
        if d < 0.05 and self.wi < len(self.wps) - 1:
            self.wi += 1
            tx, ty = self.wps[self.wi]
            d = math.hypot(tx - self.x, ty - self.y)
        err = wrap(math.atan2(ty - self.y, tx - self.x) - self.yaw)
        if abs(err) > 0.4:
            cmd.angular.z = self.w * (1.0 if err > 0 else -1.0)
        else:
            cmd.linear.x = min(self.v, 1.2 * d)
            cmd.angular.z = max(-0.8, min(0.8, 1.5 * err))
        self.pub.publish(cmd)
        if d < 0.05 and self.wi == len(self.wps) - 1:
            self.arrive()

    def arrive(self):
        self.wps = []
        self.reached += 1
        t = self.now()
        dt = t - self.t_goal_set if self.t_goal_set else 0.0
        est = f'{self.eta:.0f}s' if self.eta else '?'
        self.get_logger().info(
            f'ARRIVED #{self.reached} in {dt:.0f}s (eta was {est}) '
            f'| {self.state[:60]}')
        self.t_goal_set = None
        self.eta = None

    def on_goal(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.t_goal_set = self.now()

    def on_cmd(self, msg):
        pass


def main():
    rclpy.init()
    n = Driver()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
