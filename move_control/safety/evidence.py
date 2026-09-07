"""ROS evidence adapter. Clock and profile decisions live in pure subjects."""
import json
import time
import math
import uuid
import hashlib
from types import SimpleNamespace

from std_msgs.msg import String

from ..control.safety_profile import SafetyProfile
from ..control.calibration_profile import ProfileLease
from ..sensing.observation import Observations
from ..sensing.body import LIDAR_X


class Evidence:
    def init_evidence(self, latched):
        self.observations = Observations(max_age=self.timeout)
        self.profile = SafetyProfile.build()
        self.profile_valid = True
        self.profile_error = None
        self.calibration_lease = ProfileLease()
        self.calibration_applied_pub = self.create_publisher(String, '/calibration/applied', latched)
        self.create_subscription(String, '/calibration/profile', self.on_calibration_profile, latched)
        self._filtered_generations = {}
        self._filtered_values = {}
        self._corr_generation = -1
        self._us_hit_generation = -1
        self._ir_generation = -1
        self._ir_filtered = ()
        self.last_decision = {'action': 'stop', 'reason': 'startup'}
        self.observation_session = uuid.uuid4().hex
        self.profile_pub = self.create_publisher(String, '/safety/profile', latched)
        self.observation_pub = self.create_publisher(String, '/safety/observation', 10)
        self.decision_pub = self.create_publisher(String, '/safety/decision', 10)

    def observe(self, name, msg=None, valid=True):
        args = {}
        if msg is not None and hasattr(msg, 'header'):
            stamp = msg.header.stamp
            args = {'source': stamp.sec + stamp.nanosec*1e-9,
                    'source_now': self.now().nanoseconds*1e-9}
        return self.observations.add(name, time.monotonic(), valid=valid, **args)

    def on_calibration_profile(self, msg):
        try:
            packet = json.loads(msg.data)
        except (ValueError, TypeError):
            packet = None
        self.calibration_lease.accept(packet, self.now().nanoseconds*1e-9,
                                      time.monotonic(), self.profile.revision)
        self.publish_calibration_applied()

    def publish_calibration_applied(self):
        if (self.calibration_lease.active and (not self.profile_valid or
                self.calibration_lease.active['geometry_revision'] != self.profile.revision)):
            self.calibration_lease.active = None
            self.calibration_lease.reason = 'geometry_changed'
        self.calibration_applied_pub.publish(String(data=json.dumps(
            self.calibration_lease.report(time.monotonic()), allow_nan=False)))

    def refresh_profile(self):
        try:
            mount = getattr(self, 'lidar_mount', None)
            radius = max(self.robot_r, .083) if self.get_parameter('footprint_guard_enabled').value else self.robot_r
            effective = {
                'radius': self.robot_r, 'turn_clear': radius + .010 + (abs(LIDAR_X) if mount is None else 0.),
                'stop': self.stop_d, 'clear': self.clear_d,
                'rear_stop': self.rear_stop_d, 'rear_clear': self.rear_clear_d,
                'max_linear': float(self.get_parameter('safety_max_linear').value),
                'max_angular': float(self.get_parameter('safety_max_angular').value),
                'lidar_yaw': self.lidar_yaw, 'linear_sign': self.cmd_linear_sign,
                'half_width': self.half_w, 'us_stop': self.us_stop, 'us_clear': self.us_clear,
                'footprint_guard': bool(self.get_parameter('footprint_guard_enabled').value),
                'mount': list(mount) if mount is not None else None,
                'imu_unit': self.get_parameter('imu_angular_velocity_unit').value,
            }
            payload = json.dumps(effective, sort_keys=True, allow_nan=False)
            if not (0 < effective['max_linear'] <= 1. and 0 < effective['max_angular'] <= 3.):
                raise ValueError('invalid actuator caps')
            revision = hashlib.sha256(payload.encode()).hexdigest()[:16]
            self.profile = SimpleNamespace(revision=revision, **effective)
            self.profile_valid, self.profile_error = True, None
        except (ValueError, TypeError, OverflowError) as error:
            self.profile_valid, self.profile_error = False, str(error)
            effective = {}
        report = {'schema_version': 1, 'revision': self.profile.revision,
                  'effective': effective, 'valid': self.profile_valid, 'error': self.profile_error}
        self.profile_pub.publish(String(data=json.dumps(report, allow_nan=False)))
        self.publish_calibration_applied()

    def required_observation_failure(self):
        required = ['lidar', 'imu']
        if bool(self.get_parameter('cliff_enable').value):
            required.append('ir')
        now = time.monotonic()
        return next((name+'_unavailable' for name in required
                     if not self.observations.fresh(name, now)), None)

    def record_decision(self, v, w, reason):
        now = time.monotonic()
        self.last_decision = {'action': 'stop' if not (v or w) else ('allow' if reason == 'allow' else 'limit'),
                             'issued_s': self.now().nanoseconds*1e-9,
                             'requested_v': self.last_cmd.linear.x,
                             'requested_omega': self.last_cmd.angular.z,
                             'reason': reason, 'safe_v': v, 'safe_omega': w,
                             'profile_revision': self.profile.revision,
                             'observation_generation': self.observations.generation('lidar')}
        self.decision_pub.publish(String(data=json.dumps(self.last_decision, allow_nan=False)))
        self.observation_pub.publish(String(data=json.dumps(
            {'schema_version': 1, 'session': self.observation_session,
             'issued_s': self.now().nanoseconds*1e-9,
             'frame': 'base_link', 'range_origin': 'lidar',
             'profile_revision': self.profile.revision,
             'ranges': {name: value if math.isfinite(value) else None for name, value in
                        (('front', self.lidar_front), ('rear', self.lidar_rear),
                         ('left', self.lidar_left), ('right', self.lidar_right))},
             'streams': self.observations.report(now)}, allow_nan=False)))

    def halt_with_reason(self, reason):
        self._publish_zero()
        self.record_decision(0., 0., reason)
