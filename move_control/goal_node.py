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
(via the safety gate). /goal/cmd stop immediately revokes the published route.
"""
import math
import json
import time

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from std_msgs.msg import Float32, String
from tf2_ros import Buffer as TfBuffer
from tf2_ros import TransformListener
from tf2_ros import TransformException

from .planning import GoalBrain, OccupancyMap, parse_goal_cmd
from .sensing.localization import lease_ready
from .sensing.pose import planar_pose
from .planning.obstacle_overlay import obstacle_overlay
from .control.obstacle_risk import accept_observation, TRACK_UNCERTAINTY
from rclpy.time import Time
from .goal_escape import GoalEscape
from .control.escape_budget import EscapeBudget


def grid_clearance(distance, resolution):
    # OccupancyGrid resolution is float32: 0.02 arrives as 0.01999999955.
    # Do not inflate a whole extra cell for that representation error.
    return max(distance, math.ceil(distance / resolution - 1e-6) * resolution)


class GoalNode(Node, GoalEscape):
    def __init__(self):
        super().__init__('goal_node')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('rate', 1.0)
        self.declare_parameter('mode', 'stop')
        self.declare_parameter('map_timeout', 10.0)
        self.declare_parameter('static_map', False)
        self.declare_parameter('localization_required', False)
        self.declare_parameter('obstacle_tracking_enabled', False)
        self.declare_parameter('obstacle_tracking_margin', .02)
        self.obstacle_observation = None
        self.create_subscription(String, '/obstacles/tracks', self.on_obstacle_tracks, 10)
        self.localization_status = None
        self.localization_was_ready = False
        self.create_subscription(String, '/localization/status', self.on_localization, 10)
        self.declare_parameter('pose_timeout', 1.0)
        self.declare_parameter('min_size', 6)
        self.declare_parameter('clear_m', 0.12)
        self.declare_parameter('retry_clear_m', 0.12)
        self.declare_parameter('start_escape_clear_m', 0.0)
        self.declare_parameter('start_escape_distance_m', .08)
        self.declare_parameter('robot_radius', .076)
        self.init_escape()
        self.navigation_profile = None
        self.navigation_profile_received = None
        self.create_subscription(String, '/calibration/status', self.on_calibration_profile, 10)
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
        # Coverage-done fallback: when zigzag has no lanes left, keep sending
        # the robot to the farthest reachable cell. Fresh maps with narrow
        # corridors (sim maze) otherwise deadlock on 'coverage done' at plan
        # #1 and never move.
        self.declare_parameter('probe_when_done', False)
        # Sim-only: raw-map retry for inflation-sealed coverage waypoints
        # (0.3 m sim corridors seal under clear_m; real corridors don't).
        self.declare_parameter('retry_unreachable_wp', False)
        self.declare_parameter('debug', False)
        self.create_subscription(
            OccupancyGrid, self.get_parameter('map_topic').value,
            self.on_map, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            if self.get_parameter('static_map').value else qos_profile_sensor_data)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self.on_odom, 10)
        self.create_subscription(String, '/goal/cmd', self.on_cmd, 10)
        self.create_subscription(String, '/goal/arrival', self.on_arrival, 10)
        self.issued_routes = []
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_point', 10)
        self.manual_result_pub = self.create_publisher(String, '/goal/manual_result', 10)
        self.manual_started_ns = None
        self.manual_target = None
        self.route_pub = self.create_publisher(Path, '/route', 10)
        self.state_pub = self.create_publisher(String, '/goal_node/state', 10)
        self.options_pub = self.create_publisher(MarkerArray, '/goal/options', 10)
        self.eta_pub = self.create_publisher(Float32, '/goal/eta', 10)
        self.tf = TfBuffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.map_obj = None
        self._map_received = None
        self._map_source = None
        self._map_source_age = 0.
        self._map_reset_ns = 0
        self._n_options = 0  # last published /goal/options marker count
        self.last_executable_goal = None
        self.last_executable_exit = None
        self.navigation_feedback_received = None
        self.create_subscription(String, "/wander/state", self.on_navigation_feedback, 10)
        self.ox = self.oy = 0.0
        self.have_odom = False
        self._hist = []  # (t, x, y) odom ring for the effective speed
        self.mode = str(self.get_parameter('mode').value)
        if self.mode not in ('explore', 'coverage', 'stop'):
            self.mode = 'stop'
        self.brain = GoalBrain(
            start_escape_clear_m=float(self.get_parameter('start_escape_clear_m').value),
            start_escape_distance_m=float(self.get_parameter('start_escape_distance_m').value),
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
            escape_clear_m=float(self.get_parameter('escape_clear_m').value),
            probe_when_done=bool(
                self.get_parameter('probe_when_done').value),
            retry_unreachable_wp=bool(
                self.get_parameter('retry_unreachable_wp').value))
        self.brain.mode = self.mode if self.mode != 'stop' else 'explore'
        self.timer = self.create_timer(
            1.0 / max(0.1, float(self.get_parameter('rate').value)), self.plan)
        self.get_logger().info(
            f'goal_node ready | mode={self.mode} '
            f'min_size={self.brain.min_size}')


    def on_calibration_profile(self, msg):
        self.navigation_profile = None
        try:
            status = json.loads(msg.data)
            p = status.get('navigation_profile')
            radius = float(self.get_parameter('robot_radius').value)
            if not status.get('ready') or not isinstance(p, dict):
                return
            values = [p[k] for k in ('body_radius_m','map_resolution_m','minimum_clearance_m','preferred_clearance_m')]
            if (not all(math.isfinite(v) for v in values) or abs(values[0]-radius) > .001 or
                    not .001 <= values[1] <= .1 or
                    not radius+.01+values[1]/2-1e-6 <= values[2] <= radius+.05 or
                    not values[2] <= values[3] <= values[2]+.031):
                return
            self.navigation_profile = p
            self.navigation_profile_received = time.monotonic()
        except (ValueError, TypeError, KeyError):
            pass

    def on_map(self, msg):
        # Keep the latest map as a planner object; planning reads it at 1 Hz.
        stamp_ns = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        if self._map_reset_ns and stamp_ns <= self._map_reset_ns:
            return  # An old queued map cannot repopulate a reset session.
        source = stamp_ns * 1e-9
        source_age = 0.
        if not self.get_parameter('static_map').value:
            source_age = self.get_clock().now().nanoseconds * 1e-9 - source
            if source <= 0. or not -.1 <= source_age <= float(self.get_parameter('map_timeout').value):
                self.map_obj = None
                self._clear_route('waiting for fresh map source')
                return
            if self._map_source is not None and source <= self._map_source:
                return  # Replayed source data cannot renew dynamic map evidence.
        if (msg.header.frame_id != 'map' or msg.info.width <= 0 or
                msg.info.height <= 0 or not math.isfinite(msg.info.resolution) or
                msg.info.resolution <= 0 or
                not math.isfinite(msg.info.origin.position.x) or
                not math.isfinite(msg.info.origin.position.y) or
                abs(msg.info.origin.orientation.x) > 1e-6 or
                abs(msg.info.origin.orientation.y) > 1e-6 or
                abs(msg.info.origin.orientation.z) > 1e-6 or
                len(msg.data) != msg.info.width * msg.info.height):
            self.map_obj = None
            self._clear_route('waiting for valid map frame')
            return
        self.map_obj = OccupancyMap.from_msg(msg)
        self._map_received = time.monotonic()
        self._map_source, self._map_source_age = source, max(0., source_age)

    def on_localization(self, msg):
        try:
            self.localization_status = json.loads(msg.data)
        except (ValueError, TypeError):
            self.localization_status = None
        if not self.get_parameter('localization_required').value:
            return
        ready = lease_ready(self.localization_status, self.get_clock().now().nanoseconds * 1e-9)
        if not ready:
            self._clear_route('localization uncertain; route revoked')
        elif not self.localization_was_ready:
            # Losing pose is not evidence that the destination is unreachable.
            self.brain.restart_recovery()
            self.last_executable_goal = self.last_executable_exit = None
            self.plan()
        self.localization_was_ready = ready

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.ox, self.oy = p.x, p.y
        self.have_odom = True
        now = self.get_clock().now().nanoseconds * 1e-9
        self._hist.append((now, p.x, p.y))
        self.escape_budget.observe(now,(p.x,p.y))
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

    def on_navigation_feedback(self, msg):
        self.navigation_feedback_received = time.monotonic() if msg.data.startswith("route_") else None

    def on_arrival(self, msg):
        if self.mode not in ('explore', 'coverage') or self.map_obj is None:
            return
        try:
            event = json.loads(msg.data)
            stamp, target = event['route_stamp_ns'], event['target']
            if (not isinstance(stamp, int) or isinstance(stamp, bool)
                    or not isinstance(target, list) or len(target) != 2
                    or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in target)):
                return
            if not any(t == stamp and tuple(target) == xy for t, xy in self.issued_routes):
                return
            if (self.last_executable_goal is None or
                    math.dist(target, self.last_executable_goal) > .005):
                return
            pose, _ = self.pose()
            if pose[0] is None or math.dist(pose, target) > .04:
                return
        except (ValueError, KeyError, TypeError):
            return
        self.brain.complete_goal(self.map_obj, pose, target)
        self._clear_route('goal reached; selecting next target')
        self.plan()

    def on_cmd(self, msg):
        cmd = msg.data.strip().lower()
        if cmd == 'replan':
            if self.mode == 'stop':
                return
            if self.mode != 'manual':
                self.brain.avoid_goal(self.last_executable_goal)
            self.brain.avoid_route_exit(self.last_executable_exit)
            self.last_executable_goal = None
            self._clear_route('replanning: failed target excluded; seeking alternative')
            self.plan()
            return
        if cmd == 'reset':
            self.escape_intent=None
            self.escape_used=False
            self.escape_budget=EscapeBudget()
            self.publish_escape(None)
            self.mode = 'stop'
            self.brain.reset()
            self.map_obj = None
            self._map_received = None
            self._map_source = None
            self._map_source_age = 0.
            self._map_reset_ns = self.get_clock().now().nanoseconds
            self._clear_route('map reset; stopped')
            return
        if cmd in ('explore', 'explore_nearest', 'coverage', 'stop'):
            self.escape_intent=None
            self.escape_used=False
            self.escape_budget=EscapeBudget()
            self.publish_escape(None)
            if cmd in ('explore','explore_nearest'):
                self.brain.frontier_strategy='nearest' if cmd=='explore_nearest' else 'gain'
            if cmd=='explore_nearest':
                cmd='explore'
            if cmd != 'stop':
                self.brain.restart_recovery()
            self.mode = cmd
            self.brain.mode = 'explore' if cmd == 'stop' else cmd
            self.brain.clear_manual()
            self._clear_route('stopped' if cmd == 'stop' else f'{cmd} waiting for route')
            self.get_logger().info(f'mode -> {cmd}')
            return
        xy = parse_goal_cmd(cmd)
        if xy is not None:
            self.escape_intent=None
            self.escape_used=False
            self.escape_budget=EscapeBudget()
            self.publish_escape(None)
            self.mode = 'manual'
            self.brain.mode = 'manual'
            self.manual_started_ns = self.get_clock().now().nanoseconds
            self.manual_target = list(xy)
            self.brain.set_manual(*xy)
            self._clear_route('manual goal waiting for route')
            self.get_logger().info(
                f'manual goal -> ({xy[0]:.2f}, {xy[1]:.2f})')
            return
        self.get_logger().warn(
            f'unknown /goal/cmd {cmd!r} (explore|explore_nearest|coverage|stop|x,y)')

    def pose(self):
        """Only a fresh map->base transform authorizes map-frame routes."""
        if (self.get_parameter('localization_required').value and not lease_ready(
                self.localization_status, self.get_clock().now().nanoseconds * 1e-9)):
            return (None, None), 'localization-uncertain'
        try:
            t = self.tf.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2))
            tr = t.transform.translation
            age = (self.get_clock().now().nanoseconds -
                   (t.header.stamp.sec * 1000000000 + t.header.stamp.nanosec)) / 1e9
            if age < -0.5 or age > float(self.get_parameter('pose_timeout').value):
                return (None, None), 'stale-tf'
            q = t.transform.rotation
            if planar_pose(tr.x, tr.y, (q.x, q.y, q.z, q.w)) is None:
                return (None, None), 'invalid-tf'
            self.map_yaw=planar_pose(tr.x,tr.y,(q.x,q.y,q.z,q.w))[2]
            return (float(tr.x), float(tr.y)), 'tf'
        except Exception:
            # Nested-unpack safe: (x, y), src = pose() must never see a
            # bare None (crashed goal_node within seconds on this exact line).
            return (None, None), 'none'

    def on_obstacle_tracks(self, msg):
        try:
            value = json.loads(msg.data)
            if value['frame'] == 'odom':
                self.obstacle_observation = accept_observation(value, self.obstacle_observation,
                    self.get_clock().now().nanoseconds*1e-9, .3)
        except (ValueError, TypeError, KeyError):
            pass

    def plan(self):
        if self.mode == 'stop':
            self._clear_route('stopped')
            return
        m = self.map_obj
        if (m is None or self._map_received is None or
                (not self.get_parameter('static_map').value and
                 time.monotonic() - self._map_received + self._map_source_age >
                 float(self.get_parameter('map_timeout').value))):
            self._clear_route('waiting for fresh map')
            return
        (x, y), src = self.pose()
        if x is None:
            if src == 'localization-uncertain':
                self.localization_was_ready = False
            self._clear_route(f'waiting pose={src}')
            return
        if self.get_parameter('obstacle_tracking_enabled').value:
            try:
                observation = self.obstacle_observation
                now = self.get_clock().now().nanoseconds*1e-9
                if observation is None or not 0 <= now-observation['stamp'] <= .3:
                    raise ValueError('Missing obstacle observation')
                tf = self.tf.lookup_transform('map', 'odom', Time(seconds=observation['stamp']))
                ts = tf.header.stamp.sec+tf.header.stamp.nanosec*1e-9
                t, q = tf.transform.translation, tf.transform.rotation
                transform = planar_pose(t.x, t.y, (q.x, q.y, q.z, q.w))
                if transform is None or not 0 <= now-ts <= 1.:
                    raise ValueError('Missing obstacle map transform')
            except (TransformException, ValueError, TypeError, KeyError) as exc:
                self._clear_route('waiting obstacle evidence: '+str(exc))
                return
        # Generic grid A* rounds cell radii for compatibility with offline
        # rigs; executable routes always round physical clearance upward.
        self.brain.clear_m = grid_clearance(float(self.get_parameter('clear_m').value), m.res)
        self.brain.retry_clear_m = max(self.brain.clear_m, grid_clearance(
            float(self.get_parameter('retry_clear_m').value), m.res))
        self.brain.escape_clear_m = max(self.brain.clear_m, grid_clearance(
            float(self.get_parameter('escape_clear_m').value), m.res))
        self.brain.start_escape_clear_m = float(self.get_parameter('start_escape_clear_m').value)
        profile = self.navigation_profile
        if (profile is not None and self.navigation_profile_received is not None and
                time.monotonic()-self.navigation_profile_received <= 3. and
                abs(profile['map_resolution_m']-m.res) < 1e-6):
            self.brain.clear_m = profile['preferred_clearance_m']
            # Tight corridors may lose comfort clearance while retaining the
            # calibrated body-plus-uncertainty minimum; keep that second pass.
            self.brain.retry_clear_m = profile['minimum_clearance_m']
            self.brain.start_escape_clear_m = profile['minimum_clearance_m']
        seen = self.navigation_feedback_received
        if self.get_parameter('obstacle_tracking_enabled').value:
            try:
                # The calibrated planner clearance already includes body and
                # measurement margin. Add only the missing tracking margin.
                required = (float(self.get_parameter('robot_radius').value)+
                            float(self.get_parameter('obstacle_tracking_margin').value)+TRACK_UNCERTAINTY)
                clearances = [self.brain.clear_m, self.brain.retry_clear_m]
                if self.brain.start_escape_clear_m > 0:
                    clearances.append(self.brain.start_escape_clear_m)
                padding = max(0., required-min(clearances))
                m = obstacle_overlay(m, observation['tracks'], transform, padding=padding)
            except (ValueError, TypeError, KeyError) as exc:
                self._clear_route('waiting obstacle evidence: '+str(exc))
                return
        self.brain.execution_feedback = seen is not None and 0 <= time.monotonic() - seen <= 1.
        escape_pose=(x,y,getattr(self,'map_yaw',float('nan')))
        if self.escape_intent is not None and self.escape_plan(m,escape_pose):
            return
        goal, route, status = self.brain.plan(m, (x, y))
        if route is None and self.escape_plan(m,escape_pose,status):
            return
        if self.mode == 'manual' and self.brain._manual is None:
            self.mode = 'stop'
            self._clear_route(status)
            self.manual_result_pub.publish(String(data=json.dumps({
                'started_ns': self.manual_started_ns, 'status': status,
                'target': self.manual_target})))
            return
        if self.get_parameter('debug').value:
            self.get_logger().info(
                f'plan covered={len(self.brain.covered)} '
                f'blacklist={len(self.brain._blacklist)} :: {status}')
        self._pub_status(status, route, src)
        self._pub_options()
        if goal is not None and route is not None:
            self._pub_goal(goal[0], goal[1], route)
        else:
            self._clear_route()

    def _clear_route(self, status=None):
        if status != 'escape: fixed-heading translation to planning clearance' and self.escape_intent is not None:
            self.escape_intent=None
            self.escape_budget.failed=True
            self.publish_escape(None)
        """Revoke old routes immediately; silence is not a stop command."""
        self.issued_routes.clear()
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        self.route_pub.publish(path)
        self.brain.last_options = []
        self._pub_options()
        self.eta_pub.publish(Float32(data=0.0))
        if status is not None:
            self.state_pub.publish(String(data=status))

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
        # A shrink (stall bench dropping candidates) leaves ids above n-1
        # drawn forever — wipe first so RViz drops the removed routes
        # instead of showing bench-dead options.
        if not opts or len(opts) < self._n_options:
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
        self._n_options = len(opts)
        self.options_pub.publish(arr)

    def _pub_goal(self, x, y, route):
        self.last_executable_goal = (x,y)
        points = route['points']
        self.last_executable_exit = next((p for p in points if math.dist(p, points[0]) >= .06), points[-1])
        stamp = self.get_clock().now().to_msg()
        self.issued_routes.append((stamp.sec*1000000000+stamp.nanosec, (x, y)))
        self.issued_routes = self.issued_routes[-8:]
        gp = PoseStamped()
        gp.header.frame_id = 'map'
        gp.header.stamp = stamp
        gp.pose.position.x = float(x)
        gp.pose.position.y = float(y)
        gp.pose.orientation.w = 1.0
        self.goal_pub.publish(gp)
        path = Path()
        path.header = gp.header
        for (px, py) in route['points']:
            ps = PoseStamped()
            ps.header = gp.header
            ps.pose.position.x = float(px)
            ps.pose.position.y = float(py)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.route_pub.publish(path)

    def stop(self):
        self.mode = 'stop'
        self._clear_route('stopped')


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


if __name__ == '__main__':
    main()
