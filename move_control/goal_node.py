#!/usr/bin/env python3
"""Goal node. Check /map, make the point to go, publish the best route.

explore: planner.pick_goal (frontier) — a point at the edge of the unknown.
When no frontier is left, coverage: ZigzagPlanner hands out the next zigzag
waypoint. When those run out: done, goal holds.

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
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from tf2_ros import Buffer as TfBuffer
from tf2_ros import TransformListener

from .planner import (OccupancyMap, ZigzagPlanner, best_route, cover_ring,
                      pick_goal)


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
        self.create_subscription(
            OccupancyGrid, self.get_parameter('map_topic').value,
            self.on_map, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self.on_odom, 10)
        self.create_subscription(String, '/goal/cmd', self.on_cmd, 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_point', 10)
        self.route_pub = self.create_publisher(Path, '/route', 10)
        self.state_pub = self.create_publisher(String, '/goal_node/state', 10)
        self.tf = TfBuffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.map_obj = None
        self.ox = self.oy = 0.0
        self.have_odom = False
        self.mode = str(self.get_parameter('mode').value)
        self.covered = set()
        self.timer = self.create_timer(
            1.0 / max(0.1, float(self.get_parameter('rate').value)), self.plan)
        ms = int(self.get_parameter('min_size').value)
        self.get_logger().info(
            f'goal_node ready | mode={self.mode} min_size={ms}')


    def on_map(self, msg):
        # Keep the latest map as a planner object; planning reads it at 1 Hz.
        self.map_obj = OccupancyMap.from_msg(msg)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.ox, self.oy = p.x, p.y
        self.have_odom = True

    def on_cmd(self, msg):
        cmd = msg.data.strip().lower()
        if cmd in ('explore', 'coverage', 'stop'):
            self.mode = cmd
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
            return None, 'none'

    def plan(self):
        m = self.map_obj
        (x, y), src = self.pose()
        if m is None or x is None:
            self.state_pub.publish(String(data=f'waiting map={m is not None} pose={src}'))
            return
        rate = float(self.get_parameter('rate').value)
        min_size = int(self.get_parameter('min_size').value)
        clear_m = float(self.get_parameter('clear_m').value)
        retry = float(self.get_parameter('retry_clear_m').value)
        lane_w = float(self.get_parameter('lane_width').value)
        lane_s = float(self.get_parameter('lane_step').value)
        tol = float(self.get_parameter('reach_tol').value)
        if self.mode == 'stop':
            self.state_pub.publish(String(data='stopped'))
            return
        if self.mode == 'explore':
            g = pick_goal(m, (x, y), min_size=min_size, clear_m=clear_m,
                          retry_clear_m=retry)
            if g is not None:
                self._pub_goal(g['x'], g['y'], g['route'])
                self.state_pub.publish(String(
                    data=f"explore goal=({g['x']:.2f},{g['y']:.2f}) "
                         f"size={g['size']} route={g['route']['length']:.2f}m "
                         f"pose~{src}"))
                return
            self.get_logger().info('no frontier left -> coverage mode')
            self.mode = 'coverage'
        # coverage mode
        cover_ring(self.covered, m, x, y, radius_m=lane_w / 2 + 0.08)
        zz = ZigzagPlanner(m, start=(x, y), covered=self.covered,
                           lane_width=lane_w, lane_step=lane_s)
        wps = zz.waypoints()
        if not wps:
            self.state_pub.publish(String(data='coverage done'))
            return
        goal = wps[0]
        route = best_route(m, (x, y), goal, clear_m=clear_m)
        if route is None:
            # Waypoint behind a wall/unknown: mark swept and move on.
            self.covered.add(m.world_to_grid(*goal))
            self.state_pub.publish(String(data='coverage wp unreachable, skip'))
            return
        if math.hypot(goal[0] - x, goal[1] - y) < tol:
            self.state_pub.publish(String(data='coverage wp at robot'))
            return
        self._pub_goal(goal[0], goal[1], route)
        self.state_pub.publish(String(
            data=f'coverage goal=({goal[0]:.2f},{goal[1]:.2f}) '
                 f'left={len(wps)} route={route["length"]:.2f}m pose~{src}'))

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
