"""Subject: map route execution through wander's existing safety-gated output."""
import math
import json
import os

from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from ..control.path_follow import ProgressGuard, PathFollower
from ..control.recover import hazard_action
from ..control.route_recovery import RouteRecovery
from ..control.rotation_relocation import RotationRelocation
from ..control.safe_trail import SafeTrail
from ..control.trail_retreat import TrailRetreat
from ..control.straight_escape import StraightEscape
from ..sensing.pose import planar_pose


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
        self.path_follower = PathFollower()
        self.navigation_recovery = RouteRecovery()
        self.rotation_relocation = RotationRelocation()
        self.safe_trail = SafeTrail()
        self.trail_retreat = TrailRetreat()
        self.trail_retreat_used = False
        self.trail_retreat_hold = None
        self.straight_escape=StraightEscape()
        self.straight_escape_intent=None
        self.straight_escape_seen=None
        self.create_subscription(String,'/goal/straight_escape',self._on_straight_escape,10)
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
        self.path_follower.reset()
        self.navigation_recovery.reset()
        self.rotation_relocation.reset()
        self.safe_trail = SafeTrail()
        self.trail_retreat.reset()
        self.trail_retreat_used = False
        self.trail_retreat_hold = None
        self.straight_escape=StraightEscape()
        self.straight_escape_intent=None
        self.straight_escape_seen=None

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

    def _on_straight_escape(self,msg):
        try:
            intent=json.loads(msg.data)
            now=self.now().nanoseconds*1e-9
            if intent is not None and (not isinstance(intent,dict) or
                    not self.navigation_mode or not 0<=now-intent['issued_s']<=3. or
                    intent['issued_s']<self.navigation_started):
                return
            self.straight_escape_intent=intent
            self.straight_escape_seen=now
            if intent is None and self.straight_escape.identity is not None:
                self.straight_escape.failed=True
        except (TypeError,ValueError,KeyError):
            self.straight_escape_intent=None

    def _tick_navigation(self):
        now = self.now().nanoseconds * 1e-9
        pose, tf_age = None, math.inf
        try:
            tf = self.navigation_tf.lookup_transform('map', 'base_link', Time())
            t, q = tf.transform.translation, tf.transform.rotation
            pose = planar_pose(t.x, t.y, (q.x, q.y, q.z, q.w))
            tf_age = now - (tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9)
        except TransformException:
            pass
        hazard = hazard_action(self.tilt, self.cliff, self.seen_forward,
                               self._can_reverse()) != 'none'
        blocked = (hazard or self.estop or self.pickup or not self._ir_ready())
        age = math.inf if self.navigation_received is None else max(
            now - self.navigation_received, now - self.navigation_stamp)
        intent=self.straight_escape_intent
        if intent is not None:
            limits=self.motion_limits
            safe=(os.environ.get('ROS_DOMAIN_ID')=='227' and os.environ.get('GZ_PARTITION')=='pinky_calmap227'
                and not blocked and pose is not None and 0<=tf_age<=1. and self._odom_fresh()
                and self._motion_limits_fresh() and limits.get('bounded_motion_enabled') is True
                and limits.get('bounded_geometry')=='trusted_footprint'
                and limits.get('geometry_revision')==intent.get('geometry')
                and self.straight_escape_seen is not None and 0<=now-self.straight_escape_seen<=3.
                and 0<=now-intent.get('issued_s',-1e9)<=3.)
            velocity,reason=self.straight_escape.update(now,pose,intent,safe,
                (self.odom_x,self.odom_y,self.odom_yaw))
            command=Twist()
            command.linear.x=velocity or 0.
            self.state='forward' if velocity and velocity>0 else 'backup' if velocity else 'wait'
            self._publish(command,'route_'+str(self.navigation_mode)+':'+reason)
            return
        if self.trail_retreat.active or self.trail_retreat_hold is not None:
            v, w, reason = 0., 0., 'trail_retreat'
        else:
            v, w, reason = self.path_follower.update(
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
        if self.trail_retreat_hold is not None:
            self.state = 'wait'
            self._publish(Twist(), f'route_{self.navigation_mode}:{self.trail_retreat_hold}')
            return
        if (reason == 'arrived' and self.navigation_mode in ('explore', 'coverage')
                and not self.trail_retreat.active):
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
        localization_ok = pose is not None and 0 <= tf_age <= float(self.get_parameter('route_tf_timeout').value)
        limits = self.motion_limits
        odom_pose = (self.odom_x,self.odom_y,self.odom_yaw) if self._odom_fresh() else None
        geometry_id = limits.get('geometry_revision')
        trail_valid = bool(not blocked and localization_ok and self._odom_fresh() and
            self._motion_limits_fresh() and limits.get('rotation_scan_observed') is True)
        if not self.trail_retreat.active:
            self.safe_trail.observe(now, odom_pose, trail_valid,
                limits.get('can_rotate') is True, geometry_id)
        if self.trail_retreat.active:
            retreat_v, retreat_w, retreat_reason = self.trail_retreat.update(
                now, odom_pose, geometry_id,
                trail_valid and self._can_reverse() and limits.get('bounded_motion_enabled') is True,
                limits.get('can_rotate') is True, rotation=limits.get('rotation_estimate'))
            if retreat_reason == 'trail_retreat_complete':
                self.navigation_progress.reset()
                self.path_follower.reset()
                self.navigation_recovery.reset()
                self.navigation_route = []
                self.navigation_goal_pub.publish(String(data='replan'))
            elif not self.trail_retreat.active:
                self.trail_retreat_hold = retreat_reason
            command = Twist()
            command.linear.x, command.angular.z = retreat_v, retreat_w
            self.state = 'backup' if retreat_v else 'turn' if retreat_w else 'wait'
            self._publish(command, f'route_{self.navigation_mode}:{retreat_reason}')
            return
        suggestion = limits.get('rotation_recovery_m')
        relocation = self.rotation_relocation
        direction = relocation.direction if relocation.active else (
            1 if isinstance(suggestion, (int,float)) and suggestion > 0 else -1)
        relocation_safe = (not blocked and localization_ok and self._odom_fresh() and
            self._motion_limits_fresh() and limits.get('rotation_scan_observed') is True and
            (self._can_reverse() if direction < 0 else not (self.blocked or self._on_wall())))
        capsule = limits.get('rotation_translation_limits_m')
        remaining = (max(0.,relocation.target_distance-math.dist(
            (self.odom_x,self.odom_y),relocation.start_pose[:2])) if relocation.active else
            abs(suggestion) if isinstance(suggestion,(int,float)) else math.inf)
        available = (capsule[int(direction < 0)] if isinstance(capsule,(list,tuple)) and len(capsule)==2 else None)
        relocation_safe = bool(relocation_safe and isinstance(available,(int,float)) and
            math.isfinite(available) and available >= remaining and available > 0.)
        # Complete the initial hysteresis distance even after crossing the bare
        # rotation threshold; a 1 mm nudge must not repeatedly relatch this state.
        margin = limits.get('rotation_pivot_clearance_m')
        restored = (limits.get('can_rotate') is True and isinstance(margin,(int,float)) and margin >= .013)
        if relocation.active and limits.get('can_rotate') is True and relocation_safe:
            suggestion = relocation.direction*relocation.target_distance
        relocation_v, relocation_reason = relocation.update(now,
            (self.odom_x,self.odom_y,self.odom_yaw) if self._odom_fresh() else None,
            relocation_safe, restored, suggestion, bool(w) and limits.get('can_rotate') is False
            and not limits.get('bounded_motion_enabled', False))
        if relocation_v is not None:
            if relocation_reason in ('rotation_relocation_clear','rotation_relocation_distance'):
                self.navigation_progress.reset()
                self.navigation_recovery.waiting = False
                self.navigation_recovery.failed_goal = self.navigation_recovery.failed_exit = None
                self.navigation_recovery.progress_since = now
                self.navigation_recovery.blocked_since = None
            command = Twist()
            command.linear.x = relocation_v
            self.state = 'forward' if relocation_v > 0 else 'backup' if relocation_v < 0 else 'wait'
            self._publish(command, f'route_{self.navigation_mode}:{relocation_reason}')
            return
        self.navigation_progress.pause(now, not localization_ok)
        if self.navigation_progress.check(now, pose, bool(v or w)):
            v, w, reason = 0.0, 0.0, 'stalled_restart_required'
            if (not self.trail_retreat_used and trail_valid and self._can_reverse() and
                    limits.get('bounded_motion_enabled') is True):
                retreat_route = self.safe_trail.retreat(now, odom_pose, geometry_id)
                if retreat_route and self.trail_retreat.start(now, odom_pose, retreat_route, geometry_id,
                        rotation=limits.get('rotation_estimate')):
                    self.path_follower.reset()
                    # One bounded attempt per explicit navigation command.
                    # Replans and a return to the same throat cannot renew it.
                    self.trail_retreat_used = True
                    self.state = 'wait'
                    self._publish(Twist(), f'route_{self.navigation_mode}:trail_retreat_start')
                    return
        recoverable = (not hazard and not self.estop and not self.pickup and self._ir_ready()
                       and localization_ok)
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
