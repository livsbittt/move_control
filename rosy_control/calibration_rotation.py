"""ROS adapter for bounded rotation trials; all estimates remain pure subjects."""
import math
import time
from geometry_msgs.msg import Twist
from .control.calibration import wrap
from .control.rotation_trial import RotationTrial
from .sensing.scan_rotation import scan_rotation
from .sensing.scan_motion import scan_points, match_motion
from .control.rotation_envelope import RotationEnvelope


class CalibrationRotation:
    def reset_rotation(self):
        self.saved_rotation = None
        self.rotation_trial = None
        self.rotation_reference = None
        self.rotation_alignment = None
        self.rotation_scan = None
        self.rotation_imu_yaw = None
        self.rotation_points = self.rotation_reference_points = None
        self.relocation_points = None
        self.relocation_scan_deadline = None
        self.rotation_envelope = None

    def complete_translation(self, passed, message):
        if not passed or not self.get_parameter('calibration_rotation').value:
            self.finish(passed, message)
            return
        self.zero()
        self.phase, self.message = 'validating_rotation', 'Settling before bounded left/right validation'
        self.rotation_wait = time.monotonic()+.8

    def rotation_scan_sample(self, msg, valid):
        self.rotation_scan = (time.monotonic(), tuple(msg.ranges), msg.angle_increment) if valid else None
        source = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        age = self.get_clock().now().nanoseconds*1e-9-source
        self.relocation_scan_deadline = (time.monotonic()+.2-max(0.,age)
                                         if valid and -.1 <= age <= .2 else None)
        mount = getattr(self, 'rotation_mount', None)
        self.rotation_points = (scan_points(msg.ranges, msg.angle_min, msg.angle_increment, mount)
                                if valid and mount is not None else None)
        self.relocation_points = (scan_points(msg.ranges, msg.angle_min, msg.angle_increment, mount, max_points=1440)
                                  if valid and mount is not None else None)
        if self.rotation_reference is not None and valid:
            reference, increment = self.rotation_reference
            self.rotation_alignment = (scan_rotation(reference, msg.ranges, increment)
                                       if abs(msg.angle_increment-increment) < 1e-8 else None)

    def rotation_clear(self, now):
        if not self.rotation_scan or not self.geometry_profile:
            return False
        seen, ranges, increment = self.rotation_scan
        radius = self.geometry_profile['effective']['turn_clear']
        # Scan returns are measured from the sensor, so include its offset from the body.
        radius += getattr(self, 'rotation_mount_offset', 0.)
        return (0 <= now-seen <= .5 and abs(len(ranges)*increment-2*math.pi) < .02 and
                all(math.isfinite(v) and radius+.05 < v < 8. for v in ranges))

    def rotation_eligibility(self, now, allow_front_blocked=False):
        reason = self.trial_gate_reason(now)
        if reason:
            return reason
        if self.estop is not False:
            return 'Emergency stop must be explicitly released'
        # Translation wall association, rear-return and forward target are not
        # meaningful while turning. The complete swept scan replaces them.
        for name in ('odom', 'imu', 'ir', 'tf'):
            rows = self.baseline.samples[name]
            if not rows or not rows[-1][2] or not 0 <= now-rows[-1][0] <= .25:
                return 'Rotation requires fresh valid ' + name
            if self.phase in ('relocating_calibration','returning_calibration') and name in ('odom','imu'):
                deadline = self.relocation_sensor_deadlines.get(name)
                if deadline is None or now > deadline:
                    return 'Relocation requires fresh source ' + name
        for key in ('/safety/blocked', '/safety/cliff', '/safety/tilt', '/safety/pickup'):
            if allow_front_blocked and key == '/safety/blocked':
                continue  # Directional capsule and the final gate still guard translation.
            row = self.hazards.get(key)
            if not row or not 0 <= now-row[0] <= .75 or row[1]:
                return 'Rotation safety hazard or missing state'
        return None

    def tick_rotation(self, now):
        reason = self.rotation_eligibility(now)
        state, seen = self.wander_state
        if not state.startswith('stop') or not 0 <= now-seen <= .75 or seen < self.requested:
            reason = 'Wander must remain stopped throughout rotation calibration'
        if not self.rotation_clear(now) and self.rotation_trial is None:
            if (not self.rotation_eligibility(now, allow_front_blocked=True) and
                    state.startswith('stop') and 0 <= now-seen <= .75 and seen >= self.requested
                    and self.begin_relocation(now)):
                return
        if reason or not self.rotation_clear(now):
            self.finish(False, reason or 'Rotation needs fresh fully observed clearance around the body')
            return
        if now < self.rotation_wait:
            self.zero()
            return
        odom = self.baseline.latest('odom')
        if self.rotation_imu_yaw is None or odom is None:
            self.finish(False, 'Rotation orientation evidence unavailable')
            return
        if self.rotation_trial is None:
            _, ranges, increment = self.rotation_scan
            alignment = scan_rotation(ranges, ranges, increment)
            if alignment is None:
                self.finish(False, 'Scene does not make rotation observable')
                return
            self.rotation_reference = (ranges, increment)
            self.rotation_alignment = alignment
            self.rotation_odom_origin = odom
            self.rotation_imu_origin = self.rotation_imu_yaw
            self.rotation_trial = RotationTrial(now)
            self.rotation_reference_points = self.rotation_points
            geometry = self.geometry_profile['effective']
            flat = geometry.get('rotation_footprint_xy') or []
            try:
                if len(flat) % 2:
                    raise ValueError('Odd footprint coordinate count')
                self.rotation_envelope = RotationEnvelope(
                    geometry.get('rotation_body_radius', geometry['radius']),
                    list(zip(flat[::2], flat[1::2])))
            except (ValueError, TypeError, OverflowError):
                self.finish(False, 'Invalid trusted rotation footprint')
                return
        alignment = self.rotation_alignment
        if alignment is None:
            self.finish(False, 'Rotation scan alignment ambiguous or inconsistent')
            return
        origin = self.rotation_odom_origin
        previous_index = self.rotation_trial.index
        speed = self.rotation_trial.update(now, alignment['yaw'],
            wrap(self.rotation_imu_yaw-self.rotation_imu_origin), wrap(odom[2]-origin[2]),
            math.hypot(odom[0]-origin[0], odom[1]-origin[1]), True)
        # Independent settled endpoints, not hundreds of correlated scan ticks.
        if self.rotation_trial.index != previous_index and abs(alignment['yaw']) >= math.radians(5):
            measured = match_motion(self.rotation_reference_points, self.rotation_points, alignment['yaw'])
            dx, dy = odom[0]-origin[0], odom[1]-origin[1]
            c, s = math.cos(origin[2]), math.sin(origin[2])
            accepted = measured is not None and self.rotation_envelope.add(
                (measured['dx'], measured['dy'], measured['yaw']),
                (c*dx+s*dy, -s*dx+c*dy, wrap(odom[2]-origin[2])),
                wrap(self.rotation_imu_yaw-self.rotation_imu_origin), measured['residual_m'])
            if not accepted:
                self.finish(False, 'Independent rotation-center sensor evidence disagrees or is unobservable')
                return
        if self.rotation_trial.error or self.rotation_trial.done:
            valid = self.rotation_trial.done and self.rotation_envelope.report()['valid']
            self.complete_rotation(valid, self.rotation_trial.error or (
                'Translation and bilateral rotation verified; swept envelope estimated' if valid else 'Insufficient bilateral rotation-center evidence'), now)
            return
        command = Twist()
        command.angular.z = speed
        self.publish_trial(command)
        self.message = f'Rotation leg {self.rotation_trial.index+1}/8'
        if now-self.last_report >= .5:
            self.publish()
