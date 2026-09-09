"""ROS adapter for post-translation, pre-rotation calibration relocation."""
import math
import os
from geometry_msgs.msg import Twist
from .control.calibration_relocation import CalibrationRelocation
from .control.rotation_trial import MAX_TRANSLATION_M
from .control.calibration import StationaryBaseline
from .sensing.wall_tracker import WallTracker
from .sensing.range_filter import CalibrationRangeFilter
from std_msgs.msg import String


class CalibrationRelocationAdapter:
    def relocation_enabled(self):
        # Release only in the isolated wheel rig until physical braking and
        # independent rear sensing have equivalent measured evidence.
        return (self.get_parameter('calibration_relocation_enabled').value and
                self.get_parameter('calibration_rotation').value and
                self.get_parameter('calibration_round_trip').value and
                self.get_parameter('use_sim_time').value and
                os.environ.get('ROS_DOMAIN_ID') == '227' and
                os.environ.get('GZ_PARTITION') == 'pinky_calmap227')

    def begin_relocation(self, now, before_translation=False):
        verified = self.round_trip is not None and self.round_trip.done
        if (not self.relocation_enabled() or (not verified and not before_translation)
                or self.relocation is not None or self.rotation_eligibility(now, allow_front_blocked=True)):
            return False
        odom = self.baseline.latest('odom')
        if odom is None:
            return False
        if self.calibration_origin is None:
            self.calibration_origin = tuple(odom[:3])
        self.relocation_before_translation = before_translation
        self.trial_geometry_revision = self.geometry_revision
        self.wander_pub.publish(String(data='stop'))
        self.requested = now
        self.zero()
        self.relocation = CalibrationRelocation()
        self.phase = 'relocating_calibration'
        self.relocation_wait = now+.4
        self.message = ('Seeking calibration space with bounded direction probe' if before_translation else
                        'Translation verified; seeking straight observed clearance for rotation calibration')
        self.publish()
        return True

    def tick_relocation(self, now):
        reason = self.rotation_eligibility(now, allow_front_blocked=True)
        state, seen = self.wander_state
        if not state.startswith('stop') or not 0 <= now-seen <= .75 or seen < self.requested:
            if now-self.requested < 2.:
                self.zero()
                return
            reason = 'Wander must remain stopped during calibration relocation'
        if reason:
            self.finish(False, reason)
            return
        if now < self.relocation_wait:
            self.zero()
            return
        full = False
        ranges = ()
        if self.rotation_scan and self.relocation_points is not None:
            stamp, ranges, increment = self.rotation_scan
            full = (0 <= now-stamp <= .2 and self.relocation_scan_deadline is not None and
                    now <= self.relocation_scan_deadline and len(ranges) >= 360 and
                    abs(len(ranges)*increment-2*math.pi) < .02 and
                    all(math.isfinite(v) and 0 < v < 8. for v in ranges) and
                    len(self.relocation_points) == len(ranges))
        geometry = self.geometry_profile['effective']
        body = geometry.get('rotation_body_radius', geometry['radius'])
        # Reserve the full permitted trial-center travel before choosing a
        # station; barely-clear endpoints can block on the first pivot step.
        rotation_radius = (geometry['turn_clear']+.05+getattr(self, 'rotation_mount_offset', 0.)
                           +MAX_TRANSLATION_M)
        if self.relocation_before_translation:
            limits_seen, limits = self.safety_limits
            if not 0 <= now-limits_seen <= .2 or any(
                    not isinstance(limits.get(key),(int,float)) or not math.isfinite(limits[key])
                    for key in ('front_stop_m','rear_stop_m','us_stop_m')):
                self.finish(False, 'Fresh translation clearance required while seeking calibration space')
                return
            rotation_radius = max(rotation_radius, limits['front_stop_m']+.028,
                                  limits['rear_stop_m']+.008, limits['us_stop_m']+.028)
        odom = self.baseline.latest('odom')
        speed, result = self.relocation.update(now, odom[:3] if odom else None,
            self.relocation_points, full_scan_observed=full, body_radius=body,
            rotation_radius=rotation_radius, fresh_guard=True,
            translation_verified=self.round_trip is not None and self.round_trip.done,
            bootstrap_allowed=self.relocation_before_translation,
            rotation_clear_current=full and self.rotation_clear(now) and
                all(value > rotation_radius+.003 for value in ranges) and
                (not self.relocation_before_translation or self.safe_motion(now) is None))
        report = self.relocation.report()
        if report['error']:
            space_blocked = result in ('no_observed_candidate','straight_capsule_blocked',
                                       'bootstrap_bilateral_clearance','target_not_clear')
            self.finish(False, 'Calibration relocation stopped: '+result,
                        'waiting_space' if space_blocked else None)
            return
        if report['done']:
            self.zero()
            if self.relocation_before_translation:
                self.baseline = StationaryBaseline(require_us_stable=bool(
                    self.get_parameter('calibration_require_us_agreement').value))
                self.wall_tracker = WallTracker()
                self.range_filters = {name: CalibrationRangeFilter() for name in ('lidar','us')}
                self.raw_ranges = {}
                self.phase = 'collecting'
                self.message = 'Suitable space reached; acquiring a new stationary calibration baseline'
                self.publish()
                return
            self.phase = 'validating_rotation'
            self.rotation_wait = now+.8
            self.message = 'Suitable observed space reached; settling before rotation calibration'
            self.publish()
            return
        command = Twist()
        command.linear.x = speed
        self.publish_trial(command)
        self.message = 'Calibration relocation: '+result
        if now-self.last_report >= .5:
            self.publish()

    def complete_rotation(self, passed, message, now):
        if (not passed or self.after_relocation != 'return_origin' or self.relocation is None
                or not self.relocation.report()['done']):
            self.finish(passed, message)
            return
        from .control.calibration_return import CalibrationReturn
        self.zero()
        self.return_motion = CalibrationReturn(self.calibration_origin)
        self.phase = 'returning_calibration'
        self.message = 'Calibration measured; checking observed path back to the recorded origin'
        self.return_wait = now+.4
        self.publish()

    def tick_return(self, now):
        reason = self.rotation_eligibility(now, allow_front_blocked=True)
        state, seen = self.wander_state
        if not state.startswith('stop') or not 0 <= now-seen <= .75:
            reason = 'Wander must remain stopped during calibration return'
        if reason:
            self.finish(False, reason)
            return
        if now < self.return_wait:
            self.zero()
            return
        ranges = self.rotation_scan[1] if self.rotation_scan else ()
        full = bool(self.rotation_scan and self.relocation_points is not None and
            0 <= now-self.rotation_scan[0] <= .2 and self.relocation_scan_deadline is not None and
            now <= self.relocation_scan_deadline and len(ranges) >= 360 and
            len(self.relocation_points) == len(ranges) and
            abs(len(ranges)*self.rotation_scan[2]-2*math.pi) < .02 and
            all(math.isfinite(v) and 0 < v < 8. for v in ranges))
        geometry = self.geometry_profile['effective']
        odom = self.baseline.latest('odom')
        speed, result = self.return_motion.update(now, odom[:3] if odom else None,
            self.relocation_points, body_radius=geometry.get('rotation_body_radius',geometry['radius']),
            full_scan_observed=full, fresh_guard=True,
            rotation_verified=bool(self.rotation_report() and self.rotation_report()['done']))
        report = self.return_motion.report()
        if report['error']:
            self.finish(False, 'Calibration measured but return stopped: '+result, 'return_blocked')
            return
        if report['done']:
            self.finish(True, 'Calibration verified and recorded origin reached')
            return
        command = Twist()
        command.linear.x = speed
        self.publish_trial(command)
        self.message = 'Returning to calibration origin: '+result
        if now-self.last_report >= .5:
            self.publish()
