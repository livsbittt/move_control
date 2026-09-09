"""Atomic calibration transport; readiness requires the final gate's live acknowledgement."""
import json
import math
import time
from std_msgs.msg import String
from .control.calibration_profile import make_profile

class CalibrationAtomic:
    def publish_trial(self, command):
        pair = (command.linear.x, command.angular.z)
        previous = getattr(self, '_trial_request', None)
        if previous is None or previous[1:] != pair:
            self._trial_request = (time.monotonic(), *pair)
        self.raw_pub.publish(command)

    def on_safety_profile(self, msg):
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict):
                raise ValueError('Invalid safety profile')
            valid = (value.get('valid') is True and isinstance(value.get('revision'), str)
                     and bool(value['revision']) and math.isfinite(value['effective']['turn_clear'])
                     and value['effective']['turn_clear'] > 0.)
            self.geometry_revision = value['revision'] if valid else None
            self.geometry_profile = value if self.geometry_revision else None
            self.geometry_received = time.monotonic()
        except (ValueError, TypeError, KeyError):
            self.geometry_revision = None
            self.geometry_profile = None
        if self.phase == 'ready':
            if self.trial_geometry_revision is None and self.geometry_revision:
                # Legacy certificates predate runtime geometry identity. Bind
                # once after configuration fingerprint validation, never again.
                self.trial_geometry_revision = self.geometry_revision
            elif self.geometry_revision != self.trial_geometry_revision:
                self.runtime_ready = False
                self.runtime_healthy_since = None
                self.zero()
                self.wander_pub.publish(String(data='stop'))
                self.message = 'Saved calibration retained; safety geometry differs from verified identity'
                self.publish()
        if (self.trial_geometry_revision and self.geometry_revision != self.trial_geometry_revision and
                self.phase in ('validating_motion', 'validating_rotation', 'relocating_calibration', 'returning_calibration')):
            self.finish(False, 'Safety geometry changed; recalibration required')

    def on_applied(self, msg):
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict) or type(value.get('applied')) is not bool:
                raise ValueError('Invalid application acknowledgement')
            self.applied_profile = (time.monotonic(), value)
        except (ValueError, TypeError):
            self.applied_profile = None

    def on_gate_decision(self, msg):
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict):
                raise ValueError('Invalid gate decision')
            fields = ('requested_v', 'requested_omega', 'safe_v', 'safe_omega', 'issued_s')
            if not all(math.isfinite(value[name]) for name in fields):
                raise ValueError('Invalid gate output')
            age = self.get_clock().now().nanoseconds*1e-9-value['issued_s']
            if not -.1 <= age <= .25:
                raise ValueError('Stale gate output')
            self.gate_decision = (time.monotonic()+.25-max(0., age), value)
        except (ValueError, KeyError, TypeError):
            self.gate_decision = None

    def settings_applied(self):
        if not self.applied_profile:
            return False
        seen, value = self.applied_profile
        return (0 <= time.monotonic()-seen <= 1.5 and value.get('applied') is True and
                value.get('revision') == self.profile_revision and value.get('session') == self.profile_session)


    def geometry_fresh(self, now):
        return (self.geometry_revision is not None and self.geometry_received is not None
                and 0 <= now-self.geometry_received <= 1.
                and (self.phase != 'ready' or self.trial_geometry_revision in (None, self.geometry_revision)))

    def trial_gate_reason(self, now):
        if not self.geometry_fresh(now):
            return 'Fresh effective safety profile required'
        if not self.gate_decision or now > self.gate_decision[0]:
            return 'Fresh final safety command evidence required'
        if self.phase not in ('validating_motion', 'validating_rotation', 'relocating_calibration', 'returning_calibration'):
            return None
        decision = self.gate_decision[1]
        if (abs(decision['requested_v']-decision['safe_v']) > 1e-6 or
                abs(decision['requested_omega']-decision['safe_omega']) > 1e-6):
            return 'Safety modified the trial command; response cannot be calibrated'
        request = getattr(self, '_trial_request', None)
        if request and now-request[0] > .2 and (
                abs(decision['requested_v']-request[1]) > 1e-6 or
                abs(decision['requested_omega']-request[2]) > 1e-6):
            return 'Final safety command does not acknowledge the calibration request'
        return None

    def rotation_report(self):
        result = self.rotation_trial.report() if self.rotation_trial else self.saved_rotation
        if result is not None and self.rotation_trial and getattr(self, 'rotation_envelope', None) is not None:
            result['envelope'] = self.rotation_envelope.report()
        return result

    def profile_packet(self):
        enabled = self.phase == 'ready' and self.runtime_ready and self.geometry_fresh(time.monotonic())
        scales = self.round_trip.scales if self.phase == 'ready' and self.round_trip and self.round_trip.done else [1., 1.]
        return make_profile(self.profile_session, self.profile_sequence+1,
            self.get_clock().now().nanoseconds*1e-9, enabled, scales,
            self.trial_geometry_revision or self.geometry_revision, self.rotation_report(),
            rotation_trial=self.phase == 'validating_rotation' and self.geometry_fresh(time.monotonic()),
            translation_trial=self.phase in ('validating_motion', 'relocating_calibration', 'returning_calibration') and self.geometry_fresh(time.monotonic()))
