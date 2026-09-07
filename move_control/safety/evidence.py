"""ROS evidence adapter. Clock and profile decisions live in pure subjects."""
import json
import time
import math
import uuid

from std_msgs.msg import String

from ..control.safety_profile import SafetyProfile
from ..control.calibration_profile import ProfileLease
from ..sensing.observation import Observations


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
            self.profile = SafetyProfile.build(
                radius=float(self.get_parameter('robot_radius').value),
                stop=float(self.get_parameter('stop_distance').value),
                clear=float(self.get_parameter('clear_distance').value),
                half_width_deg=float(self.get_parameter('front_half_width_deg').value),
                stop_floor=float(self.get_parameter('safety_stop_floor').value),
                clear_floor=float(self.get_parameter('safety_clear_floor').value),
                max_linear=float(self.get_parameter('safety_max_linear').value),
                max_angular=float(self.get_parameter('safety_max_angular').value),
                lidar_yaw_rad=self.lidar_yaw if self.get_parameter('lidar_use_tf').value else float(self.get_parameter('lidar_yaw_offset').value),
                linear_sign=float(self.get_parameter('cmd_linear_sign').value),
                imu_unit=self.get_parameter('imu_angular_velocity_unit').value)
            self.profile_valid, self.profile_error = True, None
        except ValueError as error:
            self.profile_valid, self.profile_error = False, str(error)
        report = self.profile.report()
        report.update(valid=self.profile_valid, error=self.profile_error)
        self.profile_pub.publish(String(data=json.dumps(report, allow_nan=False)))
        self.publish_calibration_applied()

    def required_observation_failure(self):
        required = ['lidar', 'imu']
        if bool(self.get_parameter('cliff_enable').value):
            required.append('ir')
        if bool(self.get_parameter('camera_as_wall').value):
            required += ['camera_cliff', 'camera_block']
        elif bool(self.get_parameter('camera_block_as_wall').value):
            required.append('camera_block')
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
