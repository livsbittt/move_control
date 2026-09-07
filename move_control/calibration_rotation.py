"""ROS adapter for bounded rotation trials; all estimates remain pure subjects."""
import math
import time
from geometry_msgs.msg import Twist
from .control.calibration import wrap
from .control.rotation_trial import RotationTrial
from .sensing.scan_rotation import scan_rotation


class CalibrationRotation:
    def reset_rotation(self):
        self.saved_rotation = None
        self.rotation_trial = None
        self.rotation_reference = None
        self.rotation_alignment = None
        self.rotation_scan = None
        self.rotation_imu_yaw = None

    def complete_translation(self, passed, message):
        if not passed or not self.get_parameter('calibration_rotation').value:
            self.finish(passed, message)
            return
        self.zero()
        self.phase, self.message = 'validating_rotation', 'Settling before bounded left/right validation'
        self.rotation_wait = time.monotonic()+.8

    def rotation_scan_sample(self, msg, valid):
        self.rotation_scan = (time.monotonic(), tuple(msg.ranges), msg.angle_increment) if valid else None
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

    def rotation_eligibility(self, now):
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
        for key in ('/safety/blocked', '/safety/cliff', '/safety/tilt', '/safety/pickup'):
            row = self.hazards.get(key)
            if not row or not 0 <= now-row[0] <= .75 or row[1]:
                return 'Rotation safety hazard or missing state'
        return None

    def tick_rotation(self, now):
        reason = self.rotation_eligibility(now)
        state, seen = self.wander_state
        if not state.startswith('stop') or not 0 <= now-seen <= .75 or seen < self.requested:
            reason = 'Wander must remain stopped throughout rotation calibration'
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
        alignment = self.rotation_alignment
        if alignment is None:
            self.finish(False, 'Rotation scan alignment ambiguous or inconsistent')
            return
        origin = self.rotation_odom_origin
        speed = self.rotation_trial.update(now, alignment['yaw'],
            wrap(self.rotation_imu_yaw-self.rotation_imu_origin), wrap(odom[2]-origin[2]),
            math.hypot(odom[0]-origin[0], odom[1]-origin[1]), True)
        if self.rotation_trial.error or self.rotation_trial.done:
            self.finish(self.rotation_trial.done, self.rotation_trial.error or 'Translation and bilateral rotation verified')
            return
        command = Twist()
        command.angular.z = speed
        self.publish_trial(command)
        self.message = f'Rotation leg {self.rotation_trial.index+1}/8'
        if now-self.last_report >= .5:
            self.publish()
