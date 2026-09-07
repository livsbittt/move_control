"""Stationary startup checks followed by one bounded automatic motion validation."""
import json
import math
import os
from pathlib import Path
import tempfile
import time
import uuid

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan, Range, Imu, Image
from std_msgs.msg import Bool, String, UInt16MultiArray, Float32MultiArray
from tf2_ros import Buffer, TransformListener

from .control.calibration import (StationaryBaseline, MOTION_SPEED, MOTION_SECONDS,
                                  MOTION_LIMIT, motion_evidence, motion_result, wrap)
from .sensing.lidar import NOSE_YAW, is_robot_scan, sector_range
from .sensing.lidar_mount import nose_from_quaternion
from .sensing.range_filter import CalibrationRangeFilter
from .control.round_trip import RoundTrip
from .control.calibration_profile import make_profile
from .sensing.observation import Observations
from .calibration_rotation import CalibrationRotation


class StartupCalibrationNode(Node, CalibrationRotation):
    def __init__(self, parameter_overrides=None):
        super().__init__('startup_calibration_node', parameter_overrides=parameter_overrides or [])
        self.declare_parameter('lidar_yaw_offset', NOSE_YAW)
        self.declare_parameter('imu_angular_velocity_unit', 'rad_s')
        self.declare_parameter('calibration_auto_motion', True)
        self.declare_parameter('calibration_require_us_agreement', True)
        self.declare_parameter('calibration_us_max_range', 3.0)
        self.declare_parameter('calibration_round_trip', False)
        self.declare_parameter('calibration_distance_m', .03)
        self.declare_parameter('calibration_rotation', True)
        self.declare_parameter('result_path', str(Path.home() / '.local/state/move_control/calibration.json'))
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.status_pub = self.create_publisher(String, '/calibration/status', latched)
        self.ready_pub = self.create_publisher(Bool, '/calibration/ready', latched)
        self.scale_pub = self.create_publisher(Float32MultiArray, '/calibration/drive_scale', latched)
        self.profile_pub = self.create_publisher(String, '/calibration/profile', latched)
        self.geometry_revision = None
        self.geometry_profile = None
        self.geometry_received = None
        self.trial_geometry_revision = None
        self.applied_profile = None
        self.create_subscription(String, '/safety/profile', self.on_safety_profile, latched)
        self.create_subscription(String, '/calibration/applied', self.on_applied, latched)
        self.create_subscription(String, '/safety/decision', self.on_gate_decision, 10)
        self.gate_decision = None
        self.raw_pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        self.wander_pub = self.create_publisher(String, '/wander/cmd', 10)
        self.create_subscription(String, '/calibration/cmd', self.on_command, 10)
        self.create_subscription(Bool, '/estop/state', self.on_estop, latched)
        self.create_subscription(String, '/wander/state', self.on_wander, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, qos_profile_sensor_data)
        self.create_subscription(UInt16MultiArray, '/ir_sensor/range', self.on_ir, 10)
        self.create_subscription(Range, '/us_sensor/range', self.on_us, 10)
        self.create_subscription(Imu, '/imu_raw', self.on_imu, 10)
        self.create_subscription(Image, '/camera/front', self.on_camera, qos_profile_sensor_data)
        self.hazards = {}
        self.rear_clear = (0., False)
        self.create_subscription(Bool, '/safety/can_reverse',
            lambda msg: setattr(self, 'rear_clear', (time.monotonic(), msg.data)), 10)
        for topic in ('/safety/blocked', '/safety/cliff', '/safety/tilt', '/safety/pickup',
                      '/camera/blocked', '/camera/cliff'):
            self.create_subscription(Bool, topic,
                lambda msg, key=topic: self.hazards.__setitem__(key, (time.monotonic(), msg.data)), 10)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.scan_frame = None
        self.lidar_nose = None
        self.estop = None
        self.wander_state = ('', 0.)
        self.reset()
        self.timer = self.create_timer(.05, self.tick)

    def reset(self):
        self.zero()
        self.wander_pub.publish(String(data='stop'))
        self.profile_session = uuid.uuid4().hex
        self.profile_sequence = 0
        self.profile_revision = None
        self.applied_profile = None
        self.observations = Observations()
        self.trial_geometry_revision = None
        self.reset_rotation()
        self.baseline = StationaryBaseline(
            require_us_stable=bool(self.get_parameter('calibration_require_us_agreement').value))
        self.range_filters = {name: CalibrationRangeFilter() for name in ('lidar', 'us')}
        self.raw_ranges = {}
        self.phase, self.message = 'collecting', 'Keep robot stationary on safe level floor'
        self.started = time.monotonic()
        self.motion_start = None
        self.round_trip = None
        self.motion = None
        self.baseline_values = {}
        self.requested = None
        self.sensors = self.baseline.report(self.started)
        self.last_report = 0.
        try:
            self.persist()  # A previous boot's ready file is not current evidence.
        except (OSError, ValueError) as exc:
            self.phase, self.message = 'failed', f'Cannot invalidate previous calibration result: {exc}'
        self.publish()

    def zero(self):
        self.publish_trial(Twist())

    def publish_trial(self, command):
        pair = (command.linear.x, command.angular.z)
        previous = getattr(self, '_trial_request', None)
        if previous is None or previous[1:] != pair:
            self._trial_request = (time.monotonic(), *pair)
        self.raw_pub.publish(command)

    def stamped(self, msg, max_age=1.):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        return -.2 <= age <= max_age

    def on_safety_profile(self, msg):
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict):
                raise ValueError('Invalid safety profile')
            valid = value.get('valid') is True and math.isfinite(value['effective']['turn_clear'])
            self.geometry_revision = value['revision'] if valid else None
            self.geometry_profile = value if self.geometry_revision else None
            self.geometry_received = time.monotonic()
        except (ValueError, TypeError, KeyError):
            self.geometry_revision = None
            self.geometry_profile = None
        if (self.trial_geometry_revision and self.geometry_revision != self.trial_geometry_revision and
                self.phase in ('validating_motion', 'validating_rotation', 'ready')):
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

    def new_sample(self, name, msg, max_age=1.):
        tracker = self.observations
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        accepted = tracker.add(name, time.monotonic(), source=stamp,
                               source_now=self.get_clock().now().nanoseconds*1e-9, max_age=max_age)
        if not accepted and not tracker.rows[name].valid:
            self.add(name, (0.,), False)
        return accepted

    def evidence_fresh(self, now):
        return (self.baseline.fresh(now) and all(self.observations.fresh(name, now,
                max_age=5. if name == 'map' else 1.)
                for name in ('lidar', 'odom', 'map', 'us', 'imu', 'camera', 'map_tf')))

    def geometry_fresh(self, now):
        return (self.geometry_revision is not None and self.geometry_received is not None and
                0 <= now-self.geometry_received <= 1.)

    def add(self, name, values, valid=True):
        self.baseline.add(name, values, time.monotonic(), valid)

    def add_range(self, name, distance, valid):
        now = time.monotonic()
        self.raw_ranges[name] = (now, distance, valid and math.isfinite(distance))
        filtered = self.range_filters[name].update(distance, now, valid)
        self.add(name, (filtered,), valid)

    def on_estop(self, msg):
        self.estop = bool(msg.data)
        if self.estop and self.phase in ('validating_motion', 'validating_rotation'):
            self.finish(False, 'Emergency stop engaged')

    def on_wander(self, msg):
        self.wander_state = (msg.data, time.monotonic())

    def on_scan(self, msg):
        if not self.new_sample('lidar', msg):
            return
        valid = is_robot_scan(msg) and self.stamped(msg)
        self.scan_frame = msg.header.frame_id if valid else self.scan_frame
        try:
            transform = self.tf.lookup_transform('base_link', msg.header.frame_id, rclpy.time.Time())
            q = transform.transform.rotation
            self.lidar_nose = nose_from_quaternion(q.x, q.y, q.z, q.w)
        except Exception:
            valid = False
        distance = sector_range(msg, self.lidar_nose,
                                math.radians(12), pctl=.1) if valid else math.inf
        self.add_range('lidar', distance, valid)
        self.rotation_scan_sample(msg, valid)

    def on_odom(self, msg):
        if not self.new_sample('odom', msg):
            return
        p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist.linear
        yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        self.add('odom', (p.x, p.y, yaw, math.hypot(v.x, v.y)), self.stamped(msg) and .9 <= norm <= 1.1)

    def on_map(self, msg):
        if not self.new_sample('map', msg, 5.):
            return
        known = sum(value >= 0 for value in msg.data)
        valid = (msg.header.frame_id == 'map' and msg.info.width > 0 and msg.info.height > 0 and
                 len(msg.data) == msg.info.width * msg.info.height and msg.info.resolution > 0)
        self.add('map', (known, msg.info.resolution), valid and self.stamped(msg, 5.))

    def on_ir(self, msg):
        values = tuple(msg.data[:3])
        self.add('ir', values if len(values) == 3 else (0, 0, 0),
                 len(values) == 3 and all(0 < value < 4000 for value in values))

    def on_us(self, msg):
        if not self.new_sample('us', msg):
            return
        maximum = min(msg.max_range, float(self.get_parameter('calibration_us_max_range').value))
        self.add_range('us', msg.range, self.stamped(msg) and max(.02, msg.min_range) < msg.range < maximum)

    def on_imu(self, msg):
        if not self.new_sample('imu', msg):
            return
        a, g, q = msg.linear_acceleration, msg.angular_velocity, msg.orientation
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        roll = math.atan2(2*(q.w*q.x + q.y*q.z), 1 - 2*(q.x*q.x + q.y*q.y))
        pitch = math.asin(max(-1., min(1., 2*(q.w*q.y - q.z*q.x))))
        gravity = math.sqrt(a.x*a.x+a.y*a.y+a.z*a.z)
        unit = self.get_parameter('imu_angular_velocity_unit').value
        scale = math.pi / 180. if unit == 'deg_s' else 1.
        gx, gy, gz = g.x * scale, g.y * scale, g.z * scale
        gyro = math.sqrt(gx*gx+gy*gy+gz*gz)
        tilt = max(abs(roll), abs(pitch))
        self.rotation_imu_yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.add('imu', (gravity, gyro, tilt,
                         gx, gy, gz, a.x, a.y, a.z, roll, pitch),
                 unit in ('rad_s', 'deg_s') and self.stamped(msg) and .9 <= norm <= 1.1 and 8 <= gravity <= 11.5 and
                 gyro < .15 and tilt < math.radians(20))

    def on_camera(self, msg):
        if not self.new_sample('camera', msg):
            return
        pixels = np.frombuffer(bytes(msg.data), np.uint8)
        valid = (msg.encoding in ('bgr8', 'rgb8') and msg.width > 0 and msg.height > 0 and
                 msg.step >= msg.width * 3 and pixels.size >= msg.step * msg.height)
        mean, contrast = (float(pixels.mean()), float(pixels.std())) if pixels.size else (0, 0)
        self.add('camera', (mean, contrast),
                 valid and self.stamped(msg) and 5 <= mean <= 250 and contrast >= 2)

    def read_tf(self):
        try:
            if not self.scan_frame:
                raise ValueError('No scan frame')
            transform = self.tf.lookup_transform('base_link', self.scan_frame, rclpy.time.Time())
            q = transform.transform.rotation
            norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            self.lidar_nose = nose_from_quaternion(q.x, q.y, q.z, q.w)
            self.add('tf', (wrap(yaw + self.lidar_nose), self.lidar_nose), .9 <= norm <= 1.1)
        except Exception:
            self.add('tf', (0., 0.), False)
        try:
            transform = self.tf.lookup_transform('map', 'base_link', rclpy.time.Time())
            if not self.new_sample('map_tf', transform):
                return
            p, q = transform.transform.translation, transform.transform.rotation
            stamp = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
            age = self.get_clock().now().nanoseconds * 1e-9 - stamp
            norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            self.add('map_tf', (p.x, p.y, yaw), -.75 <= age <= 1. and .9 <= norm <= 1.1)
        except Exception:
            self.add('map_tf', (0., 0., 0.), False)

    def safe_motion(self, now):
        if not self.geometry_fresh(now):
            return 'Fresh effective safety profile required'
        if not self.gate_decision or now > self.gate_decision[0]:
            return 'Fresh final safety command evidence required'
        decision = self.gate_decision[1]
        if self.phase in ('validating_motion', 'validating_rotation') and (
                abs(decision['requested_v']-decision['safe_v']) > 1e-6 or
                abs(decision['requested_omega']-decision['safe_omega']) > 1e-6):
            return 'Safety modified the trial command; response cannot be calibrated'
        request = getattr(self, '_trial_request', None)
        if (self.phase in ('validating_motion', 'validating_rotation') and request and
                now-request[0] > .2 and
                (abs(decision['requested_v']-request[1]) > 1e-6 or
                 abs(decision['requested_omega']-request[2]) > 1e-6)):
            return 'Final safety command does not acknowledge the calibration request'
        if self.estop is not False:
            return 'Emergency stop must be explicitly released'
        if self.get_parameter('calibration_round_trip').value:
            stamp, clear = self.rear_clear
            if not clear or now - stamp > .75:
                return 'Round-trip requires fresh rear clearance for safe return'
        if not self.evidence_fresh(now):
            return 'Sensor data became stale or invalid'
        for stamp, distance, valid in self.raw_ranges.values():
            if not valid or now - stamp > 1. or distance < .20:
                return 'Raw range unsafe or stale; filtered values cannot authorize motion'
        if len(self.hazards) != 6 or any(now - t > .75 or active for t, active in self.hazards.values()):
            return 'Safety/camera hazard or missing fresh safety state'
        if min(self.baseline.latest('lidar')[0], self.baseline.latest('us')[0]) < .20:
            return 'Need at least 20 cm clear range for motion validation'
        return None

    def on_command(self, msg):
        command = msg.data.strip().lower()
        if command == 'abort':
            self.finish(False, 'Calibration aborted', 'aborted')
        elif command == 'retry':
            self.reset()
        elif command == 'validate_motion':
            now = time.monotonic()
            self.sensors = self.baseline.report(now)
            reason = self.safe_motion(now)
            if self.phase != 'waiting_motion' or not all(s['ok'] for s in self.sensors.values()) or reason:
                self.message = reason or 'A fresh stable stationary baseline is required first'
                self.publish()
                return
            self.wander_pub.publish(String(data='stop'))
            self.baseline_values = self.baseline.statistics(now)
            self.trial_geometry_revision = self.geometry_revision
            self.phase, self.message = 'validating_motion', 'Waiting for wander stop before bounded forward validation'
            self.requested = now
            self.publish()

    def snapshot(self):
        return {name: self.baseline.latest(name) for name in ('odom', 'lidar', 'us', 'map_tf')}

    def tick(self):
        self.read_tf()
        # TF collection timestamps its own samples; evaluate freshness after it.
        now = time.monotonic()
        if self.phase == 'validating_rotation':
            self.tick_rotation(now)
            return
        if self.phase == 'ready' and (not self.evidence_fresh(now) or not self.geometry_fresh(now)):
            self.sensors = self.baseline.report(now)
            self.finish(False, 'Calibration readiness revoked: sensor or map became stale/invalid')
            return
        if self.phase in ('collecting', 'waiting_motion'):
            self.sensors = self.baseline.report(now)
            valid = all(sensor['ok'] for sensor in self.sensors.values())
            self.phase = 'waiting_motion' if valid else 'collecting'
            if valid:
                self.baseline_values = self.baseline.statistics(now)
            self.message = 'Keep stationary; waiting for healthy stable sensors'
            if valid:
                if self.get_parameter('calibration_auto_motion').value:
                    reason = self.safe_motion(now)
                    self.message = reason or 'Stationary checks passed; starting automatic motion validation'
                    if reason is None:
                        self.on_command(String(data='validate_motion'))
                else:
                    self.message = 'Stationary baseline passed; manual motion validation selected'
        elif self.phase == 'validating_motion':
            reason = self.safe_motion(now)
            if reason:
                self.finish(False, reason)
                return
            state, seen = self.wander_state
            if not state.startswith('stop') or seen < self.requested or now - seen > .75:
                self.zero()
                if now - self.requested > 2.:
                    self.finish(False, 'Wander did not confirm stopped')
                return
            if self.motion_start is None:
                self.zero()
                if now - self.requested < .5:
                    return
                self.motion_start = (now, self.snapshot())
                if self.get_parameter('calibration_round_trip').value:
                    self.round_trip = RoundTrip(now, self.snapshot(),
                        float(self.get_parameter('calibration_distance_m').value))
            if self.round_trip is not None:
                speed = self.round_trip.update(now, self.snapshot())
                self.motion = self.round_trip.report()
                self.message = f"Round trip {self.round_trip.cycle + 1}/2: {self.round_trip.stage}"
                if self.round_trip.error:
                    self.finish(False, self.round_trip.error)
                    return
                if self.round_trip.done:
                    self.complete_translation(True, 'Round-trip correction verified and saved')
                    return
                cmd = Twist()
                cmd.linear.x = speed
                self.publish_trial(cmd)
                if now - self.last_report >= .5:
                    self.publish()
                return
            evidence = motion_evidence(self.motion_start[1], self.snapshot())
            self.motion = evidence
            if (evidence['forward_m'] < -.005 or abs(evidence['lateral_m']) > .02 or
                    abs(evidence['yaw_drift_rad']) > .15):
                self.finish(False, 'Unexpected motion direction or yaw drift')
                return
            if now - self.motion_start[0] >= MOTION_SECONDS or evidence['distance_m'] >= MOTION_LIMIT:
                passed, checks = motion_result(evidence,
                    require_us=bool(self.get_parameter('calibration_require_us_agreement').value))
                self.motion['checks'] = checks
                self.complete_translation(passed, 'Motion sensors agree' if passed else 'Motion validation failed; inspect sensor agreement')
                return
            cmd = Twist()
            cmd.linear.x = MOTION_SPEED
            self.publish_trial(cmd)
        if now - self.last_report >= .5:
            self.publish()

    def report(self):
        imu = self.baseline_values.get('imu', {}).get('mean', [])
        lidar = self.baseline_values.get('lidar', {}).get('mean', [])
        us = self.baseline_values.get('us', {}).get('mean', [])
        return {'phase': self.phase, 'ready': self.phase == 'ready' and self.settings_applied(), 'message': self.message,
                'auto_motion': bool(self.get_parameter('calibration_auto_motion').value),
                'us_precision_required': bool(self.get_parameter('calibration_require_us_agreement').value),
                'elapsed_s': round(time.monotonic() - self.started, 2),
                'sensors': self.sensors, 'motion': self.motion, 'baseline': self.baseline_values,
                'rotation': self.rotation_trial.report() if self.rotation_trial else None,
                'rotation_required': bool(self.get_parameter('calibration_rotation').value),
                'estimates': {'imu_gyro_bias_rad_s': imu[3:6], 'imu_gravity_mean_mps2': imu[6:9],
                              'imu_roll_pitch_baseline_rad': imu[9:11],
                              'lidar_us_range_difference_m': lidar[0] - us[0] if lidar and us else None},
                'recorded_unix_s': time.time(),
                'limits': {'motion_speed_mps': MOTION_SPEED,
                           'motion_seconds': 35. if self.get_parameter('calibration_round_trip').value else MOTION_SECONDS,
                           'motion_distance_m': MOTION_LIMIT,
                           'round_trip_target_m': float(self.get_parameter('calibration_distance_m').value)},
                'settings_applied': self.phase == 'ready' and self.settings_applied(),
                'profile_revision': self.profile_revision}

    def profile_packet(self):
        scales = self.round_trip.scales if self.phase == 'ready' and self.round_trip and self.round_trip.done else [1., 1.]
        return make_profile(self.profile_session, self.profile_sequence+1,
            self.get_clock().now().nanoseconds*1e-9,
            self.phase == 'ready',
            scales, self.trial_geometry_revision or self.geometry_revision,
            self.rotation_trial.report() if self.rotation_trial else None)

    def publish(self):
        self.last_report = time.monotonic()
        packet = self.profile_packet()
        self.profile_sequence += 1
        self.profile_revision = packet['revision']
        self.profile_pub.publish(String(data=json.dumps(packet, allow_nan=False)))
        self.scale_pub.publish(Float32MultiArray(data=packet['linear_gains']))
        self.status_pub.publish(String(data=json.dumps(self.report(), allow_nan=False)))
        self.ready_pub.publish(Bool(data=self.phase == 'ready' and self.settings_applied()))

    def finish(self, passed, message, phase=None):
        self.zero()
        self.phase = phase or ('ready' if passed else 'failed')
        self.message = message
        self.motion_start = None
        self.profile_revision = self.profile_packet()['revision']
        try:
            self.persist()
        except (OSError, ValueError) as exc:
            self.phase, self.message = 'failed', f'Cannot persist calibration result: {exc}'
        self.publish()

    def persist(self):
        temporary = None
        try:
            path = Path(str(self.get_parameter('result_path').value)).expanduser()
            path.parent.mkdir(parents=True, exist_ok=True)
            with tempfile.NamedTemporaryFile('w', dir=path.parent, delete=False) as stream:
                temporary = stream.name
                report = self.report()
                report['profile'] = self.profile_packet()
                json.dump(report, stream, allow_nan=False, indent=2)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, path)
        finally:
            if temporary is not None and os.path.exists(temporary):
                os.unlink(temporary)


def main():
    rclpy.init()
    node = StartupCalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.zero()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
