#!/usr/bin/env python3
"""Goal node. Check /map, make the point to go, publish the best route.

Thin I/O around planning.GoalBrain: explore = frontier point at the edge of
the unknown; no frontier left -> coverage = zigzag waypoints until done.
All decision logic lives in move_control/planning/goals.py and is unit-tested
without ROS.

Publishes:
  /goal_point       PoseStamped (map)  the point to go
  /route            nav_msgs/Path      best route robot -> point
  /goal_node/state  String             human-readable status
Subscribe /goal/cmd: explore|coverage|stop to switch modes at runtime.

The robot is not driven from here; wander/control stay in charge of motors
(via the safety gate). If /goal/cmd says stop, only publishing stops.
"""
import math

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32, String
from tf2_ros import Buffer as TfBuffer
from tf2_ros import TransformListener

from .planning import GoalBrain, OccupancyMap


class GoalNode(Node):
    def __init__(self):
        super().__init__('goal_node')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('rate', 1.0)
        self.declare_parameter('mode', 'explore')
        self.declare_parameter('min_size', 6)
        self.declare_parameter('clear_m', 0.06)
        self.declare_parameter('retry_clear_m', 0.0)
        self.declare_parameter('lane_width', 0.12)
        self.declare_parameter('lane_step', 0.20)
        self.declare_parameter('reach_tol', 0.05)
        self.declare_parameter('max_options', 3)
        # Nominal cruise for the ETA when odom history is not in yet.
        self.declare_parameter('speed_mps', 0.014)
        self.declare_parameter('stall_plans', 6)
        self.declare_parameter('progress_m', 0.03)
        self.declare_parameter('stall_min_dist', 0.15)
        self.declare_parameter('blacklist_plans', 20)
        self.declare_parameter('escape_clear_m', 0.08)
        self.create_subscription(
            OccupancyGrid, self.get_parameter('map_topic').value,
            self.on_map, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self.on_odom, 10)
        self.create_subscription(String, '/goal/cmd', self.on_cmd, 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_point', 10)
        self.route_pub = self.create_publisher(Path, '/route', 10)
        self.state_pub = self.create_publisher(String, '/goal_node/state', 10)
        self.options_pub = self.create_publisher(MarkerArray, '/goal/options', 10)
        self.eta_pub = self.create_publisher(Float32, '/goal/eta', 10)
        self.tf = TfBuffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.map_obj = None
        self.ox = self.oy = 0.0
        self.have_odom = False
        self._hist = []  # (t, x, y) odom ring for the effective speed
        self.mode = str(self.get_parameter('mode').value)
        if self.mode not in ('explore', 'coverage', 'stop'):
            self.mode = 'explore'
        self.brain = GoalBrain(
            min_size=int(self.get_parameter('min_size').value),
            clear_m=float(self.get_parameter('clear_m').value),
            retry_clear_m=float(self.get_parameter('retry_clear_m').value),
            lane_width=float(self.get_parameter('lane_width').value),
            lane_step=float(self.get_parameter('lane_step').value),
            reach_tol=float(self.get_parameter('reach_tol').value),
            max_options=int(self.get_parameter('max_options').value),
            stall_plans=int(self.get_parameter('stall_plans').value),
            progress_m=float(self.get_parameter('progress_m').value),
            stall_min_dist=float(self.get_parameter('stall_min_dist').value),
            blacklist_plans=int(self.get_parameter('blacklist_plans').value),
            escape_clear_m=float(self.get_parameter('escape_clear_m').value))
        self.brain.mode = self.mode if self.mode != 'stop' else 'explore'
        self.timer = self.create_timer(
            1.0 / max(0.1, float(self.get_parameter('rate').value)), self.plan)
        self.get_logger().info(
            f'goal_node ready | mode={self.mode} '
            f'min_size={self.brain.min_size}')


    def on_map(self, msg):
        # Keep the latest map as a planner object; planning reads it at 1 Hz.
        self.map_obj = OccupancyMap.from_msg(msg)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.ox, self.oy = p.x, p.y
        self.have_odom = True
        now = self.get_clock().now().nanoseconds * 1e-9
        self._hist.append((now, p.x, p.y))
        cut = now - 5.0
        while self._hist and self._hist[0][0] < cut:
            self._hist.pop(0)

    def v_eff(self):
        """Mean speed over the last ~3 s of odom. 0.0 when not enough."""
        h = self._hist
        for i, (t0, _x, _y) in enumerate(h):
            if h[-1][0] - t0 >= 1.5 or i == len(h) - 1:
                dt = h[-1][0] - t0
                d = math.hypot(h[-1][1] - _x, h[-1][2] - _y)
                return d / dt if dt > 0.2 else 0.0
        return 0.0

    def on_cmd(self, msg):
        cmd = msg.data.strip().lower()
        if cmd in ('explore', 'coverage', 'stop'):
            self.mode = cmd
            self.brain.mode = 'explore' if cmd == 'stop' else cmd
            self.get_logger().info(f'mode -> {cmd}')
        else:
            self.get_logger().warn(f'unknown /goal/cmd {cmd!r} (explore|coverage|stop)')

    def pose(self):
        """Robot pose in the map frame. TF first, odom as fallback."""
        try:
            t = self.tf.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2))
            tr = t.transform.translation
            return (float(tr.x), float(tr.y)), 'tf'
        except Exception:
            if self.have_odom:
                return (self.ox, self.oy), 'odom~map'
            # Nested-unpack safe: (x, y), src = pose() must never see a
            # bare None (crashed goal_node within seconds on this exact line).
            return (None, None), 'none'

    def plan(self):
        m = self.map_obj
        (x, y), src = self.pose()
        if m is None or x is None:
            self.state_pub.publish(
                String(data=f'waiting map={m is not None} pose={src}'))
            return
        if self.mode == 'stop':
            self.state_pub.publish(String(data='stopped'))
            return
        goal, route, status = self.brain.plan(m, (x, y))
        self._pub_status(status, route, src)
        self._pub_options()
        if goal is not None and route is not None:
            self._pub_goal(goal[0], goal[1], route)

    def _pub_status(self, status, route, src):
        """State line + /goal/eta. eta is seconds at the odom-derived speed
        (nominal cruise fallback); 0.0 = no estimate. The stall watchdog in
        the brain is what actually swaps an unreachable goal out."""
        v = self.v_eff()
        if v <= 0.002:
            v = float(self.get_parameter('speed_mps').value)
            v_note = 'nom'
        else:
            v_note = 'odo'
        eta = 0.0
        if route is not None and route.get('length'):
            eta = min(999.0, route['length'] / max(v, 0.001))
        self.eta_pub.publish(Float32(data=eta))
        eta_txt = f'eta~{eta:.0f}s({v_note})' if eta > 0.0 else 'eta=?'
        self.state_pub.publish(
            String(data=f'{status} {eta_txt} v={v * 100:.1f}cm/s pose~{src}'))

    def _pub_options(self):
        """Show the frontier alternatives in RViz: /goal/options.

        The chosen route is marker 0 (thick green); alternatives follow in
        score order (thin orange). No options -> one DELETEALL marker.
        """
        arr = MarkerArray()
        opts = self.brain.last_options if self.mode != 'stop' else []
        if not opts:
            clear = Marker()
            clear.action = Marker.DELETEALL
            arr.markers.append(clear)
        for i, opt in enumerate(opts):
            mk = Marker()
            mk.header.frame_id = 'map'
            mk.header.stamp = self.get_clock().now().to_msg()
            mk.ns = 'goal_options'
            mk.id = i
            mk.type = Marker.LINE_STRIP
            mk.action = Marker.ADD
            mk.pose.orientation.w = 1.0
            mk.scale.x = 0.02 if i == 0 else 0.01
            if i == 0:
                mk.color.r, mk.color.g, mk.color.b, mk.color.a = \
                    0.2, 1.0, 0.2, 0.9
            else:
                mk.color.r, mk.color.g, mk.color.b, mk.color.a = \
                    1.0, 0.6, 0.1, 0.5
            mk.points = [Point(x=float(px), y=float(py), z=0.01)
                         for px, py in opt['route']['points']]
            arr.markers.append(mk)
        self.options_pub.publish(arr)

    def _pub_goal(self, x, y, route):
        stamp = self.get_clock().now().to_msg()
        gp = PoseStamped()
        gp.header.frame_id = 'map'
        gp.header.stamp = stamp
        gp.pose.position.x = float(x)
        gp.pose.position.y = float(y)
        self.goal_pub.publish(gp)
        path = Path()
        path.header = gp.header
        for (px, py) in route['points']:
            ps = PoseStamped()
            ps.header = gp.header
            ps.pose.position.x = float(px)
            ps.pose.position.y = float(py)
            path.poses.append(ps)
        self.route_pub.publish(path)

    def stop(self):
        self.mode = 'stop'
        self.state_pub.publish(String(data='stopped'))


def main():
    rclpy.init()
    node = GoalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
