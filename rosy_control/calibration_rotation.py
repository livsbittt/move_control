"""ROS adapter for bounded rotation trials; all estimates remain pure subjects."""
import math
import time
import json
from pathlib import Path
from geometry_msgs.msg import Twist
from .control.calibration import wrap
from .control.rotation_trial import RotationTrial
from .sensing.scan_rotation import scan_rotation
from .sensing.scan_motion import scan_points, match_motion
from .control.rotation_envelope import RotationEnvelope, CROSS_ENDPOINT_MODEL
from .control.calibration_rotation_clearance import calibration_rotation_clearance
from .control.calibration_rotation_handoff import rotation_handoff_ready


class CalibrationRotation:
    def reset_rotation(self):
        self.saved_rotation = None
        self.rotation_trial = None
        self.rotation_reference = None
        self.rotation_alignment = None
        self.rotation_alignment_diagnostic = {}
        self.rotation_scan = None
        self.rotation_imu_yaw = None
        self.rotation_points = self.rotation_reference_points = None
        self.relocation_points = None
        self.relocation_scan_deadline = None
        self.rotation_envelope = None
        self.rotation_handoff_started = None
        self.rotation_registration_failure = None
        self.rotation_endpoint_previous = None
        self.rotation_endpoint_count = 0
        self.rotation_scan_pause = None
        self.rotation_start_observation_wait = None

    def wait_rotation_start_observation(self, now):
        diagnostic = getattr(self, 'rotation_clearance_diagnostic', {})
        reason = diagnostic.get('reason')
        _, gate = self.safety_limits
        partial_block = reason == 'safety_rotation_clearance_blocked' and diagnostic.get('missing_bins', 0) > 0
        eligible = reason in ('stale_scan', 'stale_safety_motion_limits') or partial_block
        waiting = getattr(self, 'rotation_start_observation_wait', None)
        if not eligible and waiting is None:
            return False
        if partial_block:
            radius = gate.get('rotation_radius_m')
            points = getattr(self, 'rotation_points', None)
            # An observed obstacle is not a transport gap. With partial scan
            # coverage the final gate uses its conservative base-frame circle.
            if (isinstance(radius, (int, float)) and math.isfinite(radius) and
                    points is not None and any(math.hypot(x,y) <= radius+.010 for x,y in points)):
                return False
        self.zero()
        odom, imu = self.baseline.latest('odom'), self.rotation_imu_yaw
        if odom is None or imu is None:
            return False
        if waiting is None:
            waiting = (now, tuple(odom), imu)
            self.rotation_start_observation_wait = waiting
        started, origin, original_imu = waiting
        if (now-started > 1. or math.hypot(odom[0]-origin[0],odom[1]-origin[1]) > .002 or
                abs(wrap(odom[2]-origin[2])) > .01 or abs(wrap(imu-original_imu)) > .01):
            self.finish(False, 'Rotation preflight observation wait expired or stationary pose changed: '+str(diagnostic))
            return True
        if not eligible:
            self.rotation_start_observation_wait = None
            return False
        self.message = 'Rotation held at zero; awaiting fresh authoritative clearance (maximum 1s): '+str(diagnostic)
        if now-self.last_report >= .5:
            self.publish()
        return True

    def pause_rotation_scan(self, now):
        trial = self.rotation_trial
        pause = getattr(self, 'rotation_scan_pause', None)
        diagnostic = getattr(self, 'rotation_alignment_diagnostic', {})
        dropout = (self.rotation_alignment is None and
                   diagnostic.get('reason') == 'insufficient_observed_returns' and
                   diagnostic.get('reference_observed', 0) >= diagnostic.get('minimum_observed', math.inf))
        if pause is None and (not dropout or trial is None or trial.last_speed != 0.):
            return False  # An unobserved moving interval invalidates the trial.
        self.zero()
        odom = self.baseline.latest('odom')
        imu = self.rotation_imu_yaw
        if odom is None or imu is None:
            self.finish(False, 'Rotation scan reacquisition requires independent pose evidence')
            return True
        if pause is None:
            pause = (now, tuple(odom), imu)
            self.rotation_scan_pause = pause
        started, origin, original_imu = pause
        if (now-started > 1. or now-trial.started > 60. or now-trial.leg_started > 7. or
                math.hypot(odom[0]-origin[0],odom[1]-origin[1]) > .002 or
                abs(wrap(odom[2]-origin[2])) > .01 or abs(wrap(imu-original_imu)) > .01):
            self.finish(False, 'Rotation scan reacquisition timed out or stationary pose changed')
            return True
        # No nonzero command is integrated or issued in this pause. Preserve
        # original trial/leg deadlines; only the zero-speed tick clock advances.
        trial.last_time = now
        if self.rotation_alignment is not None:
            self.rotation_scan_pause = None
            return False  # Ordinary fresh clearance and alignment gates still run.
        self.message = 'Rotation paused at zero; waiting for a complete LiDAR observation (maximum 1s)'
        if now-self.last_report >= .5:
            self.publish()
        return True

    def capture_rotation_registration_failure(self, measured, odom_delta, imu_yaw, reason=None,
                                               reference_points=None, alignment=None):
        # A single bounded endpoint artifact, never a live high-rate recorder.
        self.zero()
        reason = reason or ('registration_unobservable' if measured is None else
                            self.rotation_envelope.report().get('reason', 'envelope_rejected'))
        diagnostic = {'reason': reason, 'saved': False}
        self.rotation_registration_failure = diagnostic
        try:
            def points(values):
                if values is None:
                    return None
                if len(values) > 180:
                    raise ValueError('Unexpected registration point count')
                return [[float(x), float(y)] for x, y in values]
            path = Path(str(self.get_parameter('result_path').value)).expanduser().with_suffix('.rotation-failure.json')
            payload = {'schema_version': 1, 'recorded_unix_s': time.time(), 'reason': reason,
                       'reference_points': points(reference_points if reference_points is not None
                                                  else self.rotation_reference_points),
                       'current_points': points(self.rotation_points),
                       'alignment': alignment if alignment is not None else self.rotation_alignment, 'registration': measured,
                       'odom_delta': list(odom_delta), 'imu_yaw': imu_yaw,
                       'envelope': self.rotation_envelope.report(),
                       'rotation_trial': self.rotation_trial.report(),
                       'geometry_revision': self.geometry_revision}
            def ranges(values):
                if len(values) > 1440:
                    raise ValueError('Unexpected scan beam count')
                return [float(v) if -float('inf') < v < float('inf') else None for v in values]
            if getattr(self, 'rotation_reference', None) and getattr(self, 'rotation_scan', None):
                payload['angular_scan'] = {
                    'reference': ranges(self.rotation_reference[0]),
                    'current': ranges(self.rotation_scan[1]),
                    'increment': self.rotation_reference[1],
                    'diagnostic': getattr(self, 'rotation_alignment_diagnostic', {})}
            encoded = json.dumps(payload, allow_nan=False)
            path.parent.mkdir(parents=True, exist_ok=True)
            temporary = path.with_suffix(path.suffix+'.tmp')
            temporary.write_text(encoded, encoding='utf-8')
            temporary.replace(path)
            diagnostic.update(saved=True, path=str(path))
        except (OSError, ValueError, TypeError) as error:
            diagnostic['capture_error'] = str(error)

    def record_rotation_endpoint(self, alignment, odom):
        self.zero()
        current = {'points': self.rotation_points.copy() if self.rotation_points is not None else None,
                   'odom': tuple(odom), 'imu': self.rotation_imu_yaw, 'yaw': alignment['yaw']}
        previous = self.rotation_endpoint_previous
        if previous is None:
            self.rotation_endpoint_previous = current
            self.rotation_endpoint_count = 1
            return True  # The first endpoint anchors a pair; it is not a trial.
        yaw = wrap(current['yaw']-previous['yaw'])
        origin = previous['odom']
        dx,dy = odom[0]-origin[0],odom[1]-origin[1]
        c,s = math.cos(origin[2]),math.sin(origin[2])
        odom_delta = (c*dx+s*dy,-s*dx+c*dy,wrap(odom[2]-origin[2]))
        imu_delta = wrap(current['imu']-previous['imu'])
        measured = match_motion(previous['points'],current['points'],yaw)
        index = self.rotation_endpoint_count-1
        accepted = measured is not None and self.rotation_envelope.add(
            (measured['dx'],measured['dy'],measured['yaw']),odom_delta,imu_delta,measured['residual_m'],
            endpoint_pair=[index,index+1])
        if not accepted:
            self.capture_rotation_registration_failure(measured,odom_delta,imu_delta,
                reference_points=previous['points'],alignment={'yaw':yaw,'source':'consecutive_endpoints'})
            return False
        self.rotation_endpoint_previous = current
        self.rotation_endpoint_count += 1
        return True

    def complete_translation(self, passed, message):
        if not passed or not self.get_parameter('calibration_rotation').value:
            self.finish(passed, message)
            return
        self.zero()
        self.phase, self.message = 'validating_rotation', 'Settling before bounded left/right validation'
        self.rotation_handoff_started = time.monotonic()
        self.rotation_phase_started = self.rotation_handoff_started
        self.rotation_wait = self.rotation_handoff_started+.8
        # Revoke translation permission before interpreting the gate's
        # rotation decision. A translation trial intentionally forbids turns.
        self.publish()

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
            self.rotation_alignment_diagnostic = {}
            self.rotation_alignment = (scan_rotation(reference, msg.ranges, increment,
                                                     diagnostic=self.rotation_alignment_diagnostic)
                                       if abs(msg.angle_increment-increment) < 1e-8 else None)

    def rotation_clear(self, now):
        if not self.rotation_scan or not self.geometry_profile:
            self.rotation_clearance_diagnostic = {'clear': False, 'reason': 'missing_scan_or_geometry'}
            return False
        seen, ranges, increment = self.rotation_scan
        gate_seen, gate = self.safety_limits
        self.rotation_clearance_diagnostic = calibration_rotation_clearance(ranges, increment, now-seen,
            gate, now-gate_seen, self.geometry_revision)
        return self.rotation_clearance_diagnostic['clear']

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
        if now-getattr(self, 'rotation_phase_started', now) > 60.:
            self.finish(False, 'Rotation phase deadline exceeded')
            return
        reason = self.rotation_eligibility(now)
        state, seen = self.wander_state
        if not state.startswith('stop') or not 0 <= now-seen <= .75 or seen < self.requested:
            reason = 'Wander must remain stopped throughout rotation calibration'
        started = getattr(self, 'rotation_handoff_started', None)
        # A forward obstacle may be resolved by the existing guarded relocation
        # path after handoff. Hard hazards still abort while stationary/waiting.
        hard_reason = self.rotation_eligibility(now, allow_front_blocked=True) if reason else None
        if not state.startswith('stop') or not 0 <= now-seen <= .75 or seen < self.requested:
            hard_reason = reason
        if hard_reason:
            self.finish(False, hard_reason)
            return
        if getattr(self, 'rotation_trial', None) is not None and self.pause_rotation_scan(now):
            return
        if started is not None:
            self.zero()
            if not rotation_handoff_ready(now, started, self.applied_profile, self.safety_limits,
                    self.profile_session, self.profile_revision, self.geometry_revision):
                if now-started > 2.:
                    self.finish(False, 'Rotation profile handoff timed out without fresh gate acknowledgement')
                else:
                    self.message = 'Waiting at zero for rotation profile handoff acknowledgement'
                    if now-self.last_report >= .2:
                        self.publish()
                return
            self.rotation_handoff_started = None
        if now < self.rotation_wait:
            self.zero()
            return
        if not self.rotation_clear(now) and self.rotation_trial is None:
            detail = getattr(self, 'rotation_clearance_diagnostic', {})
            if ((detail.get('reason') in ('stale_scan','stale_safety_motion_limits') or
                    detail.get('reason') == 'safety_rotation_clearance_blocked' and detail.get('missing_bins',0) > 0) and
                    self.wait_rotation_start_observation(now)):
                return
            if (not self.rotation_eligibility(now, allow_front_blocked=True) and
                    state.startswith('stop') and 0 <= now-seen <= .75 and seen >= self.requested
                    and self.begin_relocation(now)):
                return
        if reason or not self.rotation_clear(now):
            detail = getattr(self, 'rotation_clearance_diagnostic', {})
            self.finish(False, reason or 'Rotation clearance: ' + str(detail))
            return
        if (getattr(self, 'rotation_start_observation_wait', None) is not None and
                self.wait_rotation_start_observation(now)):
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
            self.rotation_trial = RotationTrial(now, endpoint_refinement=True)
            self.rotation_trial.started = getattr(self, 'rotation_phase_started', now)
            self.rotation_reference_points = self.rotation_points
            geometry = self.geometry_profile['effective']
            flat = geometry.get('rotation_footprint_xy') or []
            try:
                if len(flat) % 2:
                    raise ValueError('Odd footprint coordinate count')
                self.rotation_envelope = RotationEnvelope(
                    geometry.get('rotation_body_radius', geometry['radius']),
                    list(zip(flat[::2], flat[1::2])), uncertainty_model=CROSS_ENDPOINT_MODEL)
            except (ValueError, TypeError, OverflowError):
                self.finish(False, 'Invalid trusted rotation footprint')
                return
        alignment = self.rotation_alignment
        if alignment is None:
            origin = self.rotation_odom_origin
            dx, dy = odom[0]-origin[0], odom[1]-origin[1]
            c, s = math.cos(origin[2]), math.sin(origin[2])
            self.capture_rotation_registration_failure(None,
                (c*dx+s*dy, -s*dx+c*dy, wrap(odom[2]-origin[2])),
                wrap(self.rotation_imu_yaw-self.rotation_imu_origin), reason='angular_alignment_unavailable')
            self.finish(False, 'Rotation scan alignment ambiguous or inconsistent')
            return
        origin = self.rotation_odom_origin
        previous_index = self.rotation_trial.index
        speed = self.rotation_trial.update(now, alignment['yaw'],
            wrap(self.rotation_imu_yaw-self.rotation_imu_origin), wrap(odom[2]-origin[2]),
            math.hypot(odom[0]-origin[0], odom[1]-origin[1]), True)
        # Independent settled endpoints, not hundreds of correlated scan ticks.
        if self.rotation_trial.index != previous_index and abs(alignment['yaw']) >= math.radians(5):
            if not self.record_rotation_endpoint(alignment,odom):
                self.finish(False, 'Independent rotation-center sensor evidence disagrees or is unobservable')
                return
        if self.rotation_trial.error or self.rotation_trial.done:
            valid = self.rotation_trial.done and self.rotation_envelope.report()['valid']
            self.complete_rotation(valid, self.rotation_trial.error or (
                'Translation and bounded bilateral rotation response verified; swept envelope estimated; angular compensation not applied' if valid else 'Insufficient bilateral rotation-center evidence'), now)
            return
        command = Twist()
        command.angular.z = speed
        self.publish_trial(command)
        self.message = f'Rotation leg {self.rotation_trial.index+1}/{len(self.rotation_trial.targets)}'
        if now-self.last_report >= .5:
            self.publish()
