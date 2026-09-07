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
# Sim box: rclpy removed RcutilsLogger.warn (the robot's build still has
# it) — any warn-level call crashes the node mid-run (measured, run 7:
# the grind branch fired and killed the driver). Rig-only compat shim;
# the robot's .warn calls elsewhere in the repo are valid on-robot.
import rclpy.impl.rcutils_logger as _rcutils_logger
if not hasattr(_rcutils_logger.RcutilsLogger, 'warn'):
    _rcutils_logger.RcutilsLogger.warn = _rcutils_logger.RcutilsLogger.warning
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32, String
from tf2_ros import TransformException, TransformBroadcaster, StaticTransformBroadcaster
from tf2_ros import Buffer as TfBuffer, TransformListener
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped

from move_control.control.recover import (
    escape_open,
    front_block,
    guard_speed,
    is_stuck_motion,
    ratio_sign,
)
from move_control.sensing.lidar import sector_min


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
        # Nose guard (the rig has no safety_node): never command forward
        # into a close nose arc. Measured wedge on this rig: the scan was
        # already showing 7 cm walls while the driver held full command —
        # the chassis then climbed the wall (CG at the axle) and the robot
        # turtle'd at -90 deg pitch, unrecoverable by back/spin/flee.
        self.declare_parameter('guard_clear', 0.12)
        # Escape resume gate: after back+spin, forward again only when the
        # nose arc shows a real opening (never a blind flee — it re-wedged
        # corners).
        self.declare_parameter('escape_resume', 0.20)
        self.v = float(self.get_parameter('v').value)
        self.w = float(self.get_parameter('w').value)
        self.goal_tol = float(self.get_parameter('goal_tol').value)
        self.guard_clear = float(self.get_parameter('guard_clear').value)
        self.escape_resume = float(self.get_parameter('escape_resume').value)
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
        self.maneuver_flips = 0
        self.scan = None  # latest sim scan: nose guard + escape look
        # Lateral-grind detector: nose clear but body pressed — commanded
        # forward while odom displacement ~0. The cmd-vs-2cm/s stall checks
        # miss sub-threshold grinds (measured: cmd 0.04-0.11 m/s against
        # 2.9 mm/s actual at a clear nose).
        self.grind_t0 = None
        self.grind_x = self.grind_y = 0.0
        self.timer = self.create_timer(0.05, self.tick)

    def on_scan(self, msg):
        self.scan = msg  # nose guard + escape look (the rig has no safety node)
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
        # Anchor the pursuit index at the waypoint nearest the robot instead
        # of 0: resetting to 0 made the aim point jump backward on every 1 Hz
        # replan, and the resulting err swings drove constant rotate-in-place
        # states (measured: effective 0.2-0.5 cm/s in the maze).
        if self.wps:
            self.wi = min(
                range(len(self.wps)),
                key=lambda i: math.hypot(self.wps[i][0] - self.x,
                                         self.wps[i][1] - self.y))
        else:
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
        # Pure-pursuit lookahead: aim at the FIRST route point at least
        # 0.25 m out. The old "farthest point within 0.25 m" aim landed d at
        # 6-13 cm on dense A* routes (measured stg: d=0.057 err=55, d=0.071
        # err=91) — rotate-only states in tight space, swing-guard churn,
        # back-cycle crawl. A waypoint may only be skipped if the path bends
        # gently through it: aiming across a sharp corner aims THROUGH the
        # wall corner the route turns around (measured: goals landed 2 cm
        # from the robot across a jamb).
        wi0 = self.wi
        la = self.wi
        while la + 1 < len(self.wps):
            nx, ny = self.tf_odom(*self.wps[la + 1])
            if la > self.wi:
                pv = self.tf_odom(*self.wps[la - 1])
                cv = self.tf_odom(*self.wps[la])
                a1 = math.atan2(cv[1] - pv[1], cv[0] - pv[0])
                a2 = math.atan2(ny - cv[1], nx - cv[0])
                if abs(wrap(a2 - a1)) > 0.6:
                    break
            if math.hypot(nx - self.x, ny - self.y) >= 0.25:
                la += 1  # aim AT the first point at/over the lookahead
                break
            la += 1
        self.wi = la
        tx, ty = self.tf_odom(*self.wps[la])
        d = math.hypot(tx - self.x, ty - self.y)
        err = wrap(math.atan2(ty - self.y, tx - self.x) - self.yaw)
        # Cross-track pull-back: pure-pursuit alone drifts to the corridor
        # wall side and grazes jamb corners — the wall-hug that dominated
        # the stall churn. Steer back toward the route line: signed
        # perpendicular offset from the aim segment (last skipped waypoint
        # -> aim point), 0.10 m of drift = 0.3 rad of correction.
        ax, ay = self.tf_odom(*self.wps[wi0])
        segx, segy = tx - ax, ty - ay
        seg_len = math.hypot(segx, segy)
        if seg_len > 1e-3:
            ux, uy = segx / seg_len, segy / seg_len
            lat = (self.x - ax) * -uy + (self.y - ay) * ux
            err = wrap(err - max(-0.5, min(0.5, 3.0 * lat)))
        # Arc toward the aim: stop-and-spin at |err|>0.45 pivoted in tight
        # corridors (swing-guard churn, measured x=0/wz=0 at d=0.06) —
        # alignment-scaled forward speed turns while moving instead.
        cmd.linear.x = min(self.v, 1.2 * d) * max(0.3, math.cos(err))
        cmd.angular.z = max(-self.w, min(self.w, 1.5 * err))
        # Wedged: commanded forward but odom flat (or yaw frozen) for 3 s
        # -> reverse, then spin-until-clear — wander's BACK+ESCAPE as a sim
        # stand-in. No blind flee: it re-wedged corners (and after the
        # gen_maze_world CG/mu fix a wall press can no longer lift the nose).
        t = self.now()
        front = self._front_min()
        swing = self._swing_min()
        if self.maneuver:
            if self.maneuver == 'back':
                if t >= self.maneuver_until:
                    self.maneuver = 'spin'
                    self.maneuver_until = t + 2.0
                    self.maneuver_sign = -self.maneuver_sign
            elif self.maneuver == 'spin':
                if escape_open(front, self.escape_resume):
                    self.get_logger().info(
                        f'escape clear F={front:.2f} — resume route')
                    self.maneuver = None
                elif front_block(swing, self.guard_clear * 0.75):
                    # The spin itself presses the corner: back off first.
                    self.maneuver = 'back'
                    self.maneuver_until = t + 1.5
                elif t >= self.maneuver_until:
                    # Keep looking while spinning: flip direction each 2 s
                    # window; every 3rd window reverse briefly instead (a
                    # short back re-aims the nose arc). Never a blind flee.
                    self.maneuver_flips += 1
                    if self.maneuver_flips % 3 == 0:
                        self.maneuver = 'back'
                        self.maneuver_until = t + 1.5
                    else:
                        self.maneuver_until = t + 2.0
                        self.maneuver_sign = -self.maneuver_sign
            if self.maneuver == 'back':
                cmd = Twist()
                cmd.linear.x = -0.10
            elif self.maneuver == 'spin':
                cmd = Twist()
                cmd.angular.z = 1.0 * self.maneuver_sign
        else:
            if cmd.linear.x > 0 and front_block(front, self.guard_clear):
                # Nose guard, hard band (rig has no safety_node): wall inside
                # the stop band — hold and steer toward the wider side.
                # Guarding IS stall evidence: a robot held nose-to-wall must
                # reach the back+spin escape (held-forever was a live-lock).
                s = self._open_side_sign()
                cmd.linear.x = 0.0
                if s:
                    cmd.angular.z = max(-self.w, min(self.w, s * self.w))
                self._stuck(t)
                self.get_logger().info(
                    f'guard F={front:.2f} — hold, turn {s:.0f}',
                    throttle_duration_sec=1.0)
            else:
                # Proportional nose cap: crawl as the nose arc closes so the
                # guard steering wins before contact — the binary block
                # fought pure-pursuit at full command (measured: mean cmd
                # 0.157 m/s against 3.7 cm/s actual, a standing wall-skim
                # grind on this rig).
                if cmd.linear.x > 0 and front is not None:
                    cap = guard_speed(front, self.guard_clear, cmd.linear.x)
                    if cap < cmd.linear.x:
                        cmd.linear.x = cap
                # Lateral grind: commanded forward but odom displacement ~0
                # for 3 s — the body is pressed even with a clear nose arc
                # (the cmd-vs-2cm/s stall checks miss sub-threshold grinds).
                if cmd.linear.x > 0.02 and self.have_odom:
                    if self.grind_t0 is None:
                        self.grind_t0 = t
                        self.grind_x, self.grind_y = self.x, self.y
                    else:
                        gdt = t - self.grind_t0
                        gmoved = math.hypot(self.x - self.grind_x,
                                            self.y - self.grind_y)
                        if is_stuck_motion(gmoved, gdt, 0.05, 0.01, 3.0):
                            self.get_logger().warning(
                                f'grind F={front:.2f} moved={gmoved:.3f}m '
                                f'in {gdt:.1f}s — back out')
                            self.maneuver = 'back'
                            self.maneuver_until = t + 2.0
                            self.grind_t0 = None
                        elif gmoved >= 0.02:
                            self.grind_t0 = None  # progressing — re-arm
                else:
                    self.grind_t0 = None
                if abs(cmd.angular.z) > 0.05 and front_block(
                        swing, self.guard_clear * 0.75):
                    # Rotation press: the corner swing arc is blocked — hold
                    # the spin. Guarding IS stall evidence: 3 s held at one
                    # pose fires the back+spin escape.
                    cmd.angular.z = 0.0
                    self._stuck(t)
                    self.get_logger().info(
                        f'swing guard S={swing:.2f} — hold rotation',
                        throttle_duration_sec=1.0)
                elif cmd.linear.x > 0.5 * self.v and self.spd < 0.02:
                    self._stuck(t)
                elif abs(cmd.angular.z) > 0.3 and abs(self.wz) < 0.05:
                    # Wedged nose-first: the chassis pins the wheels and yaw
                    # freezes under sustained wz commands (measured on this
                    # rig); forward-only stall detection missed it entirely.
                    self._stuck(t)
                else:
                    # Healthy motion (or a slow final approach): drop stale
                    # evidence so an old stall timestamp can't fire the
                    # maneuver off hours-old data at the next transient stop.
                    self.stuck_t0 = None
        self.pub.publish(cmd)
        if d < self.goal_tol and self.wi == len(self.wps) - 1:
            self.arrive()
        self.get_logger().info(
            f'stg d={d:.3f} err={math.degrees(err):.0f} x={cmd.linear.x:.3f} '
            f'wz={cmd.angular.z:.2f} F={front if front is not None else -1:.2f}',
            throttle_duration_sec=5.0)

    def _front_min(self):
        """Min range in the nose arc (±35 deg), or None before the first scan.

        Sim lidar is axis-aligned (scan 0 = +x = nose); the real C1's 190
        deg mount trap lives in safety_node, not in this rig-only driver.
        """
        scan = self.scan
        if scan is None:
            return None
        return sector_min(scan, 0.0, math.radians(35.0))

    def _swing_min(self):
        """Min range in the corner-swing arc (±60 deg).

        Rotating in place sweeps the chassis half-diagonal (8.6 cm on this
        rig) through the front corners — at the 0.9 cm spawn clearance a
        plain in-place turn ground the corner into the wall and the wz press
        tipped the robot even with zero forward command.
        """
        scan = self.scan
        if scan is None:
            return None
        return sector_min(scan, 0.0, math.radians(60.0))

    def _open_side_sign(self):
        """Turn sign toward the wider side arc (±90 deg), 0 = no call.

        ratio_sign: the wider side wins at 1.15x, near-equal = no opinion
        (dead end — keep the pursuit steering).
        """
        scan = self.scan
        if scan is None:
            return 0.0
        left = sector_min(scan, math.radians(90.0), math.radians(35.0))
        right = sector_min(scan, math.radians(-90.0), math.radians(35.0))
        return ratio_sign(left, right)

    def tf_odom(self, x, y):
        """Route points are map-frame; the pose is odom-frame. Steering at
        the raw point noses the robot into walls once slam drifts map from
        odom (5 cm + 7 deg measured on this rig) — transform first."""
        try:
            t = self.tf.lookup_transform('odom', 'map', Time())
        except TransformException:
            # Pre-slam (no map frame yet) identity is the right fallback —
            # but a PERSISTENT failure must stay visible or the correction
            # silently never happens again. The old bare `except Exception`
            # turned `Time(0)`'s TypeError into "TF unavailable" and the
            # map->odom correction silently never ran at all.
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
