"""Subject: map route execution through wander's existing safety-gated output."""
import math
import json

from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from ..control.path_follow import ProgressGuard, follow_path
from ..control.recover import hazard_action
from ..control.route_recovery import RouteRecovery


class Navigator:
    def _init_navigator(self):
        self.declare_parameter('route_timeout', 5.0)
        self.declare_parameter('route_tf_timeout', 1.0)
        self.declare_parameter('route_lookahead', .06)
        self.navigation_mode = None
        self.navigation_manual_target = None
        self.navigation_route = []
        self.navigation_received = None
        self.navigation_stamp = None
        self.navigation_stamp_ns = None
        self.navigation_arrived_target = None
        self.navigation_progress = ProgressGuard()
        self.navigation_recovery = RouteRecovery()
        self.navigation_tf = Buffer()
        self.navigation_listener = TransformListener(self.navigation_tf, self)
        self.navigation_goal_pub = self.create_publisher(String, '/goal/cmd', 10)
        self.navigation_arrival_pub = self.create_publisher(String, '/goal/arrival', 10)
        self.create_subscription(Path, '/route', self._on_navigation_route, 10)
        self.create_subscription(String, '/goal/manual_result', self._on_manual_result, 10)

    def _cancel_navigation(self):
        if self.navigation_mode:
            self.navigation_goal_pub.publish(String(data='stop'))
        self.navigation_mode = None
        self.navigation_manual_target = None
        self.navigation_route = []
        self.navigation_received = None
        self.navigation_stamp = None
        self.navigation_stamp_ns = None
        self.navigation_arrived_target = None
        self.navigation_progress.reset()
        self.navigation_recovery.reset()

    def _start_navigation(self, mode, goal_command=None):
        self._cancel_navigation()
        self.navigation_mode = mode
        self._recovery_budget = None
        self.stop_reason = None
        self.navigation_started = self.now().nanoseconds * 1e-9
        self.navigation_started_ns = self.now().nanoseconds
        self.navigation_manual_target = (
            [float(v) for v in goal_command.split(',')] if mode == 'manual' else None)
        self.enabled = True
        self.state = 'wait'
        self.navigation_goal_pub.publish(String(data=goal_command or mode))
        self._publish(Twist(), f'route_{mode}:no_route')

    def _on_manual_result(self, msg):
        if self.navigation_mode != 'manual':
            return
        try:
            result = json.loads(msg.data)
            started = result['started_ns']
            if (not isinstance(started, int) or isinstance(started, bool)
                    or started < self.navigation_started_ns
                    or started > self.now().nanoseconds
                    or result['target'] != self.navigation_manual_target):
                return
            status = result['status']
            if not isinstance(status, str) or not status.endswith((
                    'manual goal reached', 'manual goal finished',
                    'manual goal expired', 'manual goal unreachable, cleared')):
                return
        except (ValueError, TypeError, KeyError):
            return
        self._set_enabled(False)
        self._publish(Twist(), 'route_manual:finished')

    def _on_navigation_route(self, msg):
        if not self.navigation_mode:
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        valid = msg.header.frame_id == 'map' and stamp >= self.navigation_started
        valid = valid and all(p.header.frame_id in ('', 'map') for p in msg.poses)
        self.navigation_route = [(p.pose.position.x, p.pose.position.y)
                                 for p in msg.poses] if valid else []
        self.navigation_received = self.now().nanoseconds * 1e-9
        self.navigation_stamp = stamp
        self.navigation_stamp_ns = (msg.header.stamp.sec * 1_000_000_000
                                    + msg.header.stamp.nanosec) if valid else None
        if (self.navigation_route and self.navigation_arrived_target is not None
                and math.dist(self.navigation_route[-1], self.navigation_arrived_target) > .005):
            self.navigation_arrived_target = None

    def _tick_navigation(self):
        now = self.now().nanoseconds * 1e-9
        pose, tf_age = None, math.inf
        try:
            tf = self.navigation_tf.lookup_transform('map', 'base_link', Time())
            t, q = tf.transform.translation, tf.transform.rotation
            pose = (t.x, t.y, math.atan2(2 * (q.w*q.z + q.x*q.y),
                                        1 - 2 * (q.y*q.y + q.z*q.z)))
            tf_age = now - (tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9)
        except TransformException:
            pass
        hazard = hazard_action(self.tilt, self.cliff, self.seen_forward,
                               self._can_reverse()) != 'none'
        blocked = (hazard or self.estop or self.pickup or not self._ir_ready())
        age = math.inf if self.navigation_received is None else max(
            now - self.navigation_received, now - self.navigation_stamp)
        v, w, reason = follow_path(
            self.navigation_route, pose, route_age=age, tf_age=tf_age,
            blocked=blocked, speed=self.vmax, turn=self.wturn,
            max_age=float(self.get_parameter('route_timeout').value),
            max_tf_age=float(self.get_parameter('route_tf_timeout').value),
            lookahead=float(self.get_parameter('route_lookahead').value))
        # A front wall blocks translation, not a turn away from it. The sole
        # motor publisher still requires fresh all-around rotation clearance.
        if (self.blocked or self._on_wall()) and v > 0:
            v = 0.0
            reason = 'turn_away' if w else 'front_blocked'
        if reason == 'arrived' and self.navigation_mode in ('explore', 'coverage'):
            # Success is not a stuck episode. Let the planner retire this
            # endpoint once; periodic same-goal routes must not flood it.
            target = self.navigation_route[-1]
            if (self.navigation_arrived_target is None
                    and self.navigation_stamp_ns is not None):
                self.navigation_arrival_pub.publish(String(data=json.dumps({
                    'route_stamp_ns': self.navigation_stamp_ns,
                    'target': list(target), 'pose': list(pose[:2])})))
                self.navigation_arrived_target = target
            self.navigation_progress.reset()
            self.navigation_recovery.reset()
            self.state = 'wait'
            self._publish(Twist(), f'route_{self.navigation_mode}:awaiting_next_goal')
            return
        if self.navigation_progress.check(now, pose, bool(v or w)):
            v, w, reason = 0.0, 0.0, 'stalled_restart_required'
        recoverable = (not hazard and not self.estop and not self.pickup and self._ir_ready()
                       and 0 <= tf_age <= float(self.get_parameter('route_tf_timeout').value))
        exit_point = None
        if pose is not None and self.navigation_route:
            exit_point = next((point for point in self.navigation_route
                               if math.dist(point,pose[:2])>=.06),self.navigation_route[-1])
        recovery = self.navigation_recovery.update(
            now, pose, reason, self.navigation_route[-1] if self.navigation_route else None,
            self.navigation_stamp, recoverable, exit_point)
        if recovery == 'replan':
            v = w = 0.0
            self.navigation_goal_pub.publish(String(data='replan'))
            reason = 'replanning_' + str(self.navigation_recovery.attempts)
        elif recovery in ('waiting', 'exhausted'):
            v = w = 0.0
            reason = 'recovery_' + recovery
        elif recovery == 'alternative':
            self.navigation_progress.reset()
            v = w = 0.0  # resume the accepted alternative on the next control tick
            reason = 'alternative_accepted'
        self.state = 'forward' if v else ('turn' if w else 'wait')
        if v > 0:
            self.seen_forward = True
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = v, w
        self._publish(cmd, f'route_{self.navigation_mode}:{reason}')
