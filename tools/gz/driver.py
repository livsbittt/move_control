#!/usr/bin/env python3
"""gz test driver: follow /route, fix sim TF frames, report ETA vs actual.

The planning stack's first closed-loop test in Gazebo. Subscribes the
planner outputs and drives the diff-drive robot; publishes the TF glue the
sim needs (odom->base_link from /odom, base_link->lidar static republished
~1 Hz since slam_toolbox can clear its TF buffer on sim-time jumps).
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
from tf2_ros import Buffer as TfBuffer, TransformListener
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped


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
        # Below the brain's coverage reach_tol 0.05: arrive() must never
        # claim a waypoint the brain still counts unreached (the 0.05-0.08
        # annulus made driver and brain deadlock on the same waypoint).
        self.declare_parameter('goal_tol', 0.04)
        self.v = float(self.get_parameter('v').value)
        self.w = float(self.get_parameter('w').value)
        self.goal_tol = float(self.get_parameter('goal_tol').value)
        self.stb_sent = False
        self.stb_t = 0.0
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
        self.tf = TfBuffer()
        self.tfl = TransformListener(self.tf, self)
        self.x = self.y = self.yaw = 0.0
        self.have_odom = False
        self.wps = []
        self.wi = 0
        self.goal = None
        self.eta = None
        self.state = ''
        self.t_goal_set = None
        self.reached = 0
        # No-route escape (wander's bump-and-turn, sim stand-in): when the
        # planner stops publishing /route, rotate gently so SLAM re-scans
        # from turned poses; a route resumes the drive.
        self.last_route_t = None
        self.wig_sign = 1.0
        self.flip_t = 0.0
        # Wedge recovery (wander's BACK+ESCAPE stand-in): commanded forward
        # but odom flat for stuck_s -> brief reverse, then a spin, then
        # resume the route.
        self.spd = 0.0
        self.wz = 0.0  # actual yaw rate (odom twist): rotation-stall sensor
        self.stuck_t0 = None
        self.maneuver = None   # None | 'back' | 'spin'
        self.maneuver_until = 0.0
        self.maneuver_sign = 1.0
        self.timer = self.create_timer(0.05, self.tick)

    def on_scan(self, msg):
        frame = msg.header.frame_id
        # Republish ~1 Hz instead of one-shot: a one-shot static at stamp 0
        # left slam_toolbox with an empty static cache after a sim-time jump
        # cleared its TF buffer ("Static cache is empty" abort).
        if self.stb_sent and self.now() - self.stb_t < 1.0:
            return
        self.stb_t = self.now()
        if not self.stb_sent:
            self.get_logger().info(f'scan frame {frame!r} -> static tf')
            self.stb_sent = True
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = frame  # sensor frame mounted on the robot root
        t.transform.rotation.w = 1.0
        self.stb.sendTransform(t)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.x, self.y = p.x, p.y
        self.yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.spd = math.hypot(msg.twist.twist.linear.x,
                              msg.twist.twist.linear.y)
        self.wz = msg.twist.twist.angular.z
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
        if self.wps:
            self.last_route_t = self.now()

    def on_eta(self, msg):
        self.eta = msg.data

    def on_state(self, msg):
        self.state = msg.data

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def tick(self):
        cmd = Twist()
        if not self.have_odom:
            self.pub.publish(cmd)
            return
        if not self.wps:
            # Planner between goals: rotate gently, re-scanning from turned
            # poses until a route returns (wander's escape, sim stand-in).
            t = self.now()
            if self.last_route_t is None:
                self.last_route_t = t
            if t - self.last_route_t > 6.0:
                if t >= self.flip_t:
                    self.wig_sign = -self.wig_sign
                    self.flip_t = t + 1.5
                cmd.angular.z = 0.4 * self.wig_sign
            self.pub.publish(cmd)
            return
        # Pure-pursuit lookahead: aim at the farthest route point within
        # 0.25 m instead of the next 5 cm cell — cell-to-cell steering made
        # err flip on every grid line and wedged the robot at doorways.
        la = self.wi
        while la + 1 < len(self.wps):
            nx, ny = self.tf_odom(*self.wps[la + 1])
            if math.hypot(nx - self.x, ny - self.y) >= 0.25:
                break
            # A waypoint may only be skipped if the path bends gently
            # through it: pure-pursuit aiming across a sharp corner aims
            # THROUGH the wall corner the route turns around (measured:
            # goals landed 2 cm from the robot across a jamb).
            if la > self.wi:
                pv = self.tf_odom(*self.wps[la - 1])
                cv = self.tf_odom(*self.wps[la])
                a1 = math.atan2(cv[1] - pv[1], cv[0] - pv[0])
                a2 = math.atan2(ny - cv[1], nx - cv[0])
                if abs(wrap(a2 - a1)) > 0.6:
                    break
            la += 1
        self.wi = la
        tx, ty = self.tf_odom(*self.wps[la])
        d = math.hypot(tx - self.x, ty - self.y)
        err = wrap(math.atan2(ty - self.y, tx - self.x) - self.yaw)
        # P-steering: arcs instead of stop-and-spin — the 1 Hz replan churn
        # made stop-then-rotate waste ~4 s re-orienting between 1 s hops
        # (effective ~2-3 cm/s). Arcs keep the cruise alive between plans.
        if abs(err) > 0.45:
            cmd.angular.z = max(-self.w, min(self.w, 1.5 * err))
        else:
            cmd.linear.x = min(self.v, 1.2 * d)
            cmd.angular.z = max(-self.w, min(self.w, 1.5 * err))
        # Wedged: commanded forward but odom flat for 3 s -> reverse, then
        # spin, then resume — wander's BACK+ESCAPE as a sim stand-in.
        t = self.now()
        if self.maneuver:
            if t >= self.maneuver_until:
                if self.maneuver == 'back':
                    self.maneuver = 'spin'
                    self.maneuver_until = t + 2.0
                    self.maneuver_sign = -self.maneuver_sign
                else:
                    # Spin done: FLEE forward blindly for 1.5 s — a pure
                    # back+spin grind leaves the scan matcher matching the
                    # same ring for minutes (its map frame wandered 12 m /
                    # 110 deg doing that) — translation unseals SLAM.
                    self.maneuver = 'flee'
                    self.maneuver_until = t + 1.5
            else:
                cmd = Twist()
                cmd.linear.x = -0.10 if self.maneuver == 'back' else \
                    (0.12 if self.maneuver == 'flee' else 0.0)
                cmd.angular.z = 0.0 if self.maneuver in ('back', 'flee') \
                    else 1.0 * self.maneuver_sign
        elif cmd.linear.x > 0.04 and self.spd < 0.02:
            self._stuck(t)
        elif abs(cmd.angular.z) > 0.3 and abs(self.wz) < 0.05:
            # Wedged nose-first: the chassis pins the wheels and yaw
            # freezes under sustained wz commands (measured on this rig);
            # forward-only stall detection missed it entirely.
            self._stuck(t)
        else:
            # Healthy motion (or a slow final approach): drop stale evidence
            # so an old stall timestamp can't fire the maneuver off hours-old
            # data at the next transient stop.
            self.stuck_t0 = None
        self.pub.publish(cmd)
        if d < self.goal_tol and self.wi == len(self.wps) - 1:
            self.arrive(d)

    def tf_odom(self, x, y):
        """Route points are map-frame; the pose is odom-frame. Steering at
        the raw point noses the robot into walls once slam drifts map from
        odom (5 cm + 7 deg measured on this rig) — transform first."""
        try:
            t = self.tf.lookup_transform('odom', 'map', Time(0))
        except Exception:
            # Pre-slam (no map frame yet) identity is the right fallback —
            # but a PERSISTENT failure must stay visible or the correction
            # silently never happens again (a missing Time import once made
            # this NameError and hid behind the bare except).
            self.get_logger().warning(
                'map->odom TF unavailable; steering raw map points',
                throttle_duration_sec=10.0)
            return (x, y)
        tr = t.transform.translation
        q = t.transform.rotation
        yaw = yaw_from_quat(q)
        return (math.cos(yaw) * x - math.sin(yaw) * y + tr.x,
                math.sin(yaw) * x + math.cos(yaw) * y + tr.y)

    def _stuck(self, t):
        """Accumulate stall evidence; after 3 s fire BACK (2 s) then SPIN."""
        if self.stuck_t0 is None:
            self.stuck_t0 = t
        elif t - self.stuck_t0 > 3.0:
            self.maneuver = 'back'
            self.maneuver_until = t + 2.0
            self.stuck_t0 = None

    def arrive(self, d):
        self.get_logger().info(
            f'ARRIVE-DEBUG d={d:.3f} tol={self.goal_tol} wi={self.wi} '
            f'nwps={len(self.wps)}')
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
