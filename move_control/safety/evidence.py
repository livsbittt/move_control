"""ROS evidence adapter. Clock and profile decisions live in pure subjects."""
import json
import time

from std_msgs.msg import String

from ..control.safety_profile import SafetyProfile
from ..sensing.observation import Observations


class Evidence:
    def init_evidence(self, latched):
        self.observations = Observations(max_age=self.timeout)
        self.profile = SafetyProfile.build()
        self.profile_valid = True
        self.profile_error = None
        self._filtered_generations = {}
        self._corr_generation = -1
        self._us_hit_generation = -1
        self._ir_generation = -1
        self._ir_filtered = ()
        self.last_decision = {'action': 'stop', 'reason': 'startup'}
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
                max_angular=float(self.get_parameter('safety_max_angular').value))
            self.profile_valid, self.profile_error = True, None
        except ValueError as error:
            self.profile_valid, self.profile_error = False, str(error)
        report = self.profile.report()
        report.update(valid=self.profile_valid, error=self.profile_error)
        self.profile_pub.publish(String(data=json.dumps(report, allow_nan=False)))

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
                             'reason': reason, 'safe_v': v, 'safe_omega': w,
                             'profile_revision': self.profile.revision,
                             'observation_generation': self.observations.generation('lidar')}
        self.decision_pub.publish(String(data=json.dumps(self.last_decision, allow_nan=False)))
        self.observation_pub.publish(String(data=json.dumps(
            {'schema_version': 1, 'streams': self.observations.report(now)}, allow_nan=False)))

    def halt_with_reason(self, reason):
        self._publish_zero()
        self.record_decision(0., 0., reason)
