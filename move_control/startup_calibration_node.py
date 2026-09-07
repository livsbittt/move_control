"""Stationary startup checks followed by one bounded automatic motion validation."""
import json
import math
import os
from pathlib import Path
import tempfile
import time

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
from .sensing.wall_tracker import WallTracker
from .control.round_trip import RoundTrip
from .control.calibration_clearance import motion_clearance
from .control.navigation_calibration import environment_profile, map_ray
from .planning import OccupancyMap


class StartupCalibrationNode(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__('startup_calibration_node', parameter_overrides=parameter_overrides or [])
        self.declare_parameter('lidar_yaw_offset', NOSE_YAW)
        self.declare_parameter('imu_angular_velocity_unit', 'rad_s')
        self.declare_parameter('calibration_auto_motion', True)
        self.declare_parameter('calibration_require_us_agreement', True)
        self.declare_parameter('calibration_us_max_range', 3.0)
        self.declare_parameter('calibration_round_trip', False)
        self.declare_parameter('calibration_distance_m', .03)
        self.declare_parameter('robot_radius', .076)
        self.declare_parameter('result_path', str(Path.home() / '.local/state/move_control/calibration.json'))
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.status_pub = self.create_publisher(String, '/calibration/status', latched)
        self.ready_pub = self.create_publisher(Bool, '/calibration/ready', latched)
        self.scale_pub = self.create_publisher(Float32MultiArray, '/calibration/drive_scale', latched)
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
        self.safety_limits = (0., {})
        self.create_subscription(String, '/safety/motion_limits', self.on_motion_limits, 10)
        self.rear_clear = (0., False)
        self.create_subscription(Bool, '/safety/can_reverse',
            lambda msg: setattr(self, 'rear_clear', (time.monotonic(), msg.data)), 10)
        for topic in ('/safety/blocked', '/safety/cliff', '/safety/tilt', '/safety/pickup'):
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
        self.baseline = StationaryBaseline(
            require_us_stable=bool(self.get_parameter('calibration_require_us_agreement').value))
        self.range_filters = {name: CalibrationRangeFilter() for name in ('lidar', 'us')}
        self.wall_tracker = WallTracker()
        self.raw_ranges = {}
        self.us_source_valid = False
        self.phase, self.message = 'collecting', 'Keep robot stationary on safe level floor'
        self.started = time.monotonic()
        self.motion_start = None
        self.precision_pause_started = None
        self.precision_pause_total = 0.
        self.runtime_ready = True
        self.runtime_healthy_since = None
        self.selected_target = None
        self.motion_clearance = {}
        self.round_trip = None
        self.motion = None
        self.baseline_values = {}
        self.environment_samples = []
        self.environment_map = None
        self.navigation_profile = None
        self.requested = None
        self.sensors = self.baseline.report(self.started)
        self.last_report = 0.
        try:
            self.persist()  # A previous boot's ready file is not current evidence.
        except (OSError, ValueError) as exc:
            self.phase, self.message = 'failed', f'Cannot invalidate previous calibration result: {exc}'
        self.publish()

    def zero(self):
        self.raw_pub.publish(Twist())

    def stamped(self, msg, max_age=1.):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        return -.2 <= age <= max_age

    def add(self, name, values, valid=True):
        self.baseline.add(name, values, time.monotonic(), valid)

    def add_range(self, name, distance, valid):
        now = time.monotonic()
        self.raw_ranges[name] = (now, distance, valid and math.isfinite(distance))
        filtered = self.range_filters[name].update(distance, now, valid)
        self.add(name, (filtered,), valid)

    def on_estop(self, msg):
        self.estop = bool(msg.data)
        if self.estop and self.phase == 'validating_motion':
            self.finish(False, 'Emergency stop engaged')

    def on_wander(self, msg):
        self.wander_state = (msg.data, time.monotonic())

    def on_scan(self, msg):
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
        pose = self.baseline.latest('odom')
        odom_rows = self.baseline.samples['odom']
        if not odom_rows or not 0 <= time.monotonic()-odom_rows[-1][0] <= .2:
            pose = None
        if self.phase == 'ready':
            # A navigation turn may leave the calibration wall entirely.
            # Runtime sensor health uses actual scan returns, not a wall fit.
            precision = distance
        elif valid and pose is not None:
            p = transform.transform.translation
            precision = self.wall_tracker.update(msg, self.lidar_nose,
                locked=self.phase == 'validating_motion', pose=pose[:3], mount=(p.x, p.y),
                now=time.monotonic())
        else:
            precision = math.inf
        self.add_range('lidar', precision, valid and math.isfinite(precision))
        # Braking still observes the original raw cone; the fitted wall only
        # supplies measurement evidence and cannot hide a closer obstacle.
        self.raw_ranges['lidar'] = (time.monotonic(), distance, valid and math.isfinite(distance))
        if valid and self.phase in ('collecting', 'waiting_motion'):
            left = sector_range(msg, self.lidar_nose+math.pi/2, math.radians(8), pctl=.5)
            right = sector_range(msg, self.lidar_nose-math.pi/2, math.radians(8), pctl=.5)
            mapped = [None, None]
            try:
                t = self.tf.lookup_transform('map', msg.header.frame_id, rclpy.time.Time())
                q, p = t.transform.rotation, t.transform.translation
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                if self.environment_map is not None:
                    mapped = [map_ray(self.environment_map, (p.x,p.y), yaw+self.lidar_nose+side*math.pi/2)
                              for side in (1,-1)]
            except Exception:
                pass
            self.environment_samples.append((time.monotonic(), (left,right,*mapped)))
            self.environment_samples = self.environment_samples[-50:]

    def on_odom(self, msg):
        p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist.linear
        yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        self.add('odom', (p.x, p.y, yaw, math.hypot(v.x, v.y)), self.stamped(msg) and .9 <= norm <= 1.1)

    def on_map(self, msg):
        known = sum(value >= 0 for value in msg.data)
        valid = (msg.header.frame_id == 'map' and msg.info.width > 0 and msg.info.height > 0 and
                 len(msg.data) == msg.info.width * msg.info.height and msg.info.resolution > 0)
        self.add('map', (known, msg.info.resolution), valid and self.stamped(msg, 5.))
        if valid:
            self.environment_map = OccupancyMap.from_msg(msg)

    def on_ir(self, msg):
        values = tuple(msg.data[:3])
        self.add('ir', values if len(values) == 3 else (0, 0, 0),
                 len(values) == 3 and all(0 < value < 4000 for value in values))

    def on_us(self, msg):
        maximum = min(msg.max_range, float(self.get_parameter('calibration_us_max_range').value))
        self.us_source_valid = self.stamped(msg)
        self.add_range('us', msg.range, self.us_source_valid and max(.02, msg.min_range) < msg.range < maximum)

    def on_imu(self, msg):
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
        # Stationary gyro limits qualify calibration, not normal route turns.
        # Runtime tilt, timestamps, finite values and gravity remain required.
        self.add('imu', (gravity, gyro, tilt,
                         gx, gy, gz, a.x, a.y, a.z, roll, pitch),
                 unit in ('rad_s', 'deg_s') and self.stamped(msg) and .9 <= norm <= 1.1 and 8 <= gravity <= 11.5 and
                 (self.phase == 'ready' or gyro < .15) and tilt < math.radians(20))

    def on_camera(self, msg):
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
            p, q = transform.transform.translation, transform.transform.rotation
            stamp = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
            age = self.get_clock().now().nanoseconds * 1e-9 - stamp
            norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            self.add('map_tf', (p.x, p.y, yaw), -.75 <= age <= 1. and .9 <= norm <= 1.1)
        except Exception:
            self.add('map_tf', (0., 0., 0.), False)

    def on_motion_limits(self, msg):
        try:
            data = json.loads(msg.data)
            self.safety_limits = (time.monotonic(), data if isinstance(data, dict) else {})
        except (ValueError, TypeError):
            self.safety_limits = (0., {})

    def safe_motion(self, now):
        stamp, limits = self.safety_limits
        if not 0 <= now-stamp <= .75:
            self.motion_clearance = {'reason': 'Missing fresh safety motion limits', 'target_m': None}
            return self.motion_clearance['reason']
        limits = dict(limits)
        lidar_raw = self.raw_ranges.get('lidar', (0., 0., False))
        if isinstance(limits.get('front_m'), (int, float)) and lidar_raw[2]:
            limits['front_m'] = min(limits['front_m'], lidar_raw[1])
        if (limits.get('translation_mode') is True and lidar_raw[2] and
                isinstance(limits.get('front_stop_m'), (int, float)) and
                lidar_raw[1] <= limits['front_stop_m']):
            self.motion_clearance = {'reason': 'Raw nose range inside chassis stop clearance', 'target_m': None}
            return self.motion_clearance['reason']
        us_raw = self.raw_ranges.get('us', (0., 0., False))
        limits['us_m'] = us_raw[1]
        forward = 0.
        if self.motion_start is not None:
            current = self.snapshot()
            if not self.baseline.fresh(now) or any(value is None for value in current.values()):
                return self.sensor_failure(now)
            forward = motion_evidence(self.motion_start[1], current)['lidar_delta_m']
        round_trip = bool(self.get_parameter('calibration_round_trip').value)
        requested = float(self.get_parameter('calibration_distance_m').value) if round_trip else MOTION_LIMIT
        self.motion_clearance = motion_clearance(limits, requested,
            target=self.selected_target, forward=forward, round_trip=round_trip)
        if self.motion_clearance['reason']:
            return self.motion_clearance['reason']
        if self.estop is not False:
            return 'Emergency stop must be explicitly released'
        if self.get_parameter('calibration_round_trip').value:
            stamp, clear = self.rear_clear
            if not clear or now - stamp > .75:
                return 'Round-trip requires fresh rear clearance for safe return'
        if not self.baseline.fresh(now):
            return self.sensor_failure(now)
        for name in ('lidar', 'us'):
            stamp, distance, valid = self.raw_ranges.get(name, (0., 0., False))
            if not valid or not 0 <= now - stamp <= 1. or distance <= 0.:
                return 'Raw range unsafe or stale; filtered values cannot authorize motion'
        required = ('/safety/blocked', '/safety/cliff', '/safety/tilt', '/safety/pickup')
        if any(key not in self.hazards or now - self.hazards[key][0] > .75
               or self.hazards[key][1] for key in required):
            return 'Safety hazard or missing fresh safety state'
        return None

    def sensor_failure(self, now):
        failures = []
        for name, rows in self.baseline.samples.items():
            if not rows or not rows[-1][2] or not 0 <= now-rows[-1][0] <= (5. if name == 'map' else 1.):
                detail = 'no sample' if not rows else f'valid={rows[-1][2]}, age={now-rows[-1][0]:.3f}s'
                if name == 'lidar':
                    detail += ', wall=' + str(self.wall_tracker.diagnostic)
                failures.append(name + ': ' + detail)
        return 'Sensor data became stale or invalid: ' + '; '.join(failures)

    def pause_precision(self, now):
        # An ambiguous measurement is never permission to coast. Only this
        # precision channel may wait; raw braking and all other sensors remain
        # mandatory. Keep the same locked wall and bound the stationary wait.
        if self.estop is not False or self.motion_start is None or self.round_trip is None:
            return False
        if self.wall_tracker.diagnostic.get('reason') not in (
                'Tracked wall missing or ambiguous', 'Waiting for three associated scans'):
            return False
        for name, rows in self.baseline.samples.items():
            if name != 'lidar' and (not rows or not rows[-1][2] or
                    not 0 <= now-rows[-1][0] <= (5. if name == 'map' else .2 if name == 'odom' else 1.)):
                return False
        stamp, limits = self.safety_limits
        if not 0 <= now-stamp <= .75:
            return False
        for name, stop_key in (('lidar', 'front_stop_m'), ('us', 'us_stop_m')):
            stamp, distance, valid = self.raw_ranges.get(name, (0., 0., False))
            stop = limits.get(stop_key)
            if not valid or not 0 <= now-stamp <= .2 or not isinstance(stop, (float, int)) or distance <= stop:
                return False
        if any(key not in self.hazards or not 0 <= now-self.hazards[key][0] <= .75 or self.hazards[key][1]
               for key in ('/safety/blocked', '/safety/cliff', '/safety/tilt', '/safety/pickup')):
            return False
        if self.precision_pause_started is None:
            self.precision_pause_started = now
        elapsed = now-self.precision_pause_started
        if elapsed > 1. or self.precision_pause_total+elapsed > 2.:
            return False
        self.zero()
        self.round_trip.pause(now)
        if self.round_trip.error:
            return False
        self.message = 'Precision LiDAR paused at zero speed; reacquiring the same wall (maximum 1s)'
        self.publish()
        return True

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
            self.selected_target = self.motion_clearance['target_m']
            self.baseline_values = self.baseline.statistics(now)
            if self.environment_map is not None:
                self.navigation_profile = environment_profile(
                    float(self.get_parameter('robot_radius').value), self.environment_map.res,
                    [row for stamp,row in self.environment_samples if now-stamp <= 5.])
            self.phase, self.message = 'validating_motion', 'Waiting for wander stop before bounded forward validation'
            self.requested = now
            self.publish()

    def snapshot(self):
        return {name: self.baseline.latest(name) for name in ('odom', 'lidar', 'us', 'map_tf')}

    def runtime_health(self, now):
        """Current availability, never stationary motion/variance criteria."""
        result = {}
        for name, rows in self.baseline.samples.items():
            fresh = bool(rows) and 0 <= now-rows[-1][0] <= (5. if name == 'map' else 1.)
            valid = bool(rows) and rows[-1][2]
            advisory_echo = name == 'us' and not self.get_parameter('calibration_require_us_agreement').value
            eligible = fresh and (valid or (advisory_echo and self.us_source_valid))
            detail = 'Live sensor data valid; stationary limits do not apply during driving'
            if not fresh:
                detail = 'Waiting for fresh sensor data; saved calibration retained'
            elif not valid:
                detail = ('Ultrasonic source timestamp invalid; saved calibration retained'
                          if advisory_echo and not self.us_source_valid else
                          'Ultrasonic echo unavailable; LiDAR clearance remains authoritative'
                          if advisory_echo else 'Invalid sensor data; saved calibration retained')
            result[name] = dict(ok=fresh and valid, eligible=eligible,
                status='ok' if fresh and valid else 'advisory' if eligible else 'stale' if not fresh else 'invalid',
                samples=len(rows), detail=detail)
        return result

    def tick(self):
        self.read_tf()
        # TF collection timestamps its own samples; evaluate freshness after it.
        now = time.monotonic()
        if self.phase == 'ready':
            previously_ready = self.runtime_ready
            self.sensors = self.runtime_health(now)
            missing = [name for name, item in self.sensors.items() if not item['eligible']]
            if missing:
                self.zero()
                if self.runtime_ready:
                    self.wander_pub.publish(String(data='stop'))
                self.runtime_ready = False
                self.runtime_healthy_since = None
                self.message = 'Saved calibration retained; rechecking sensors: ' + ', '.join(missing)
            elif not self.runtime_ready:
                self.zero()
                if self.runtime_healthy_since is None:
                    self.runtime_healthy_since = now
                if now-self.runtime_healthy_since >= 1.:
                    self.runtime_ready = True
                    self.message = 'Saved calibration retained; sensor checks recovered'
            if previously_ready != self.runtime_ready or now-self.last_report >= .5:
                self.publish()
            return
        if self.phase in ('collecting', 'waiting_motion'):
            self.sensors = self.baseline.report(now)
            # Display current clearance even while sensor qualification waits.
            # This only calculates evidence; it does not authorize movement.
            self.safe_motion(now)
            for name in ('lidar', 'us'):
                stamp, distance, valid_range = self.raw_ranges.get(name, (0., math.inf, False))
                shown = f'{distance:.3f}m' if math.isfinite(distance) else 'no return'
                self.sensors[name]['detail'] += f'; raw={shown} valid={valid_range} age={max(0., now-stamp):.2f}s'
                if name == 'lidar':
                    self.sensors[name]['detail'] += '; wall=' + str(self.wall_tracker.diagnostic)
            valid = all(sensor['ok'] for sensor in self.sensors.values())
            self.phase = 'waiting_motion' if valid else 'collecting'
            if valid:
                self.baseline_values = self.baseline.statistics(now)
                if self.environment_map is not None:
                    self.navigation_profile = environment_profile(
                        float(self.get_parameter('robot_radius').value), self.environment_map.res,
                        [row for stamp,row in self.environment_samples if now-stamp <= 5.])
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
                if reason.startswith('Sensor data became stale or invalid') and self.pause_precision(now):
                    return
                if self.precision_pause_started is not None:
                    elapsed = now-self.precision_pause_started
                    if elapsed > 1. or self.precision_pause_total+elapsed > 2.:
                        reason = self.precision_timeout_message(elapsed)
                self.finish(False, reason)
                return
            if self.precision_pause_started is not None:
                elapsed = now-self.precision_pause_started
                if elapsed > 1. or self.precision_pause_total+elapsed > 2.:
                    self.finish(False, self.precision_timeout_message(elapsed))
                    return
                self.precision_pause_total += elapsed
                self.precision_pause_started = None
                if self.round_trip is not None:
                    self.round_trip.pause(now)
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
                        self.selected_target)
            if self.round_trip is not None:
                speed = self.round_trip.update(now, self.snapshot())
                self.motion = self.round_trip.report()
                self.message = f"Round trip {self.round_trip.cycle + 1}/2: {self.round_trip.stage}"
                if self.round_trip.error:
                    self.finish(False, self.round_trip.error)
                    return
                if self.round_trip.done:
                    self.finish(True, 'Round-trip correction verified and saved')
                    return
                cmd = Twist()
                cmd.linear.x = speed
                self.raw_pub.publish(cmd)
                if now - self.last_report >= .5:
                    self.publish()
                return
            evidence = motion_evidence(self.motion_start[1], self.snapshot())
            self.motion = evidence
            if (evidence['forward_m'] < -.005 or abs(evidence['lateral_m']) > .02 or
                    abs(evidence['yaw_drift_rad']) > .15):
                self.finish(False, 'Unexpected motion direction or yaw drift')
                return
            if now - self.motion_start[0] >= MOTION_SECONDS or evidence['distance_m'] >= self.selected_target:
                passed, checks = motion_result(evidence,
                    require_us=bool(self.get_parameter('calibration_require_us_agreement').value))
                self.motion['checks'] = checks
                self.finish(passed, 'Motion sensors agree' if passed else 'Motion validation failed; inspect sensor agreement')
                return
            cmd = Twist()
            cmd.linear.x = MOTION_SPEED
            self.raw_pub.publish(cmd)
        if now - self.last_report >= .5:
            self.publish()

    def precision_timeout_message(self, elapsed):
        return ('Precision LiDAR reacquisition time exceeded: '
                f'episode={elapsed:.2f}/1.00s, total={self.precision_pause_total+elapsed:.2f}/2.00s')

    def report(self):
        imu = self.baseline_values.get('imu', {}).get('mean', [])
        lidar = self.baseline_values.get('lidar', {}).get('mean', [])
        us = self.baseline_values.get('us', {}).get('mean', [])
        return {'phase': 'sensor_hold' if self.phase == 'ready' and not self.runtime_ready else self.phase,
                'ready': self.phase == 'ready' and self.runtime_ready,
                'calibration_verified': self.phase == 'ready', 'message': self.message,
                'auto_motion': bool(self.get_parameter('calibration_auto_motion').value),
                'us_precision_required': bool(self.get_parameter('calibration_require_us_agreement').value),
                'elapsed_s': round(time.monotonic() - self.started, 2),
                'sensors': self.sensors, 'motion': self.motion, 'baseline': self.baseline_values,
                'navigation_profile': self.navigation_profile,
                'motion_clearance': self.motion_clearance,
                'estimates': {'imu_gyro_bias_rad_s': imu[3:6], 'imu_gravity_mean_mps2': imu[6:9],
                              'imu_roll_pitch_baseline_rad': imu[9:11],
                              'lidar_us_range_difference_m': lidar[0] - us[0] if lidar and us else None},
                'recorded_unix_s': time.time(),
                'limits': {'motion_speed_mps': MOTION_SPEED,
                           'motion_seconds': 35. if self.get_parameter('calibration_round_trip').value else MOTION_SECONDS,
                           'motion_distance_m': MOTION_LIMIT,
                           'round_trip_target_m': float(self.get_parameter('calibration_distance_m').value)},
                'settings_applied': self.phase == 'ready' and self.round_trip is not None and self.round_trip.done}

    def publish(self):
        self.last_report = time.monotonic()
        scales = self.round_trip.scales if self.phase == 'ready' and self.round_trip and self.round_trip.done else [1., 1.]
        self.scale_pub.publish(Float32MultiArray(data=scales))
        self.status_pub.publish(String(data=json.dumps(self.report(), allow_nan=False)))
        self.ready_pub.publish(Bool(data=self.phase == 'ready' and self.runtime_ready))

    def finish(self, passed, message, phase=None):
        self.zero()
        self.phase = phase or ('ready' if passed else 'failed')
        self.runtime_ready = bool(passed)
        self.runtime_healthy_since = None
        self.message = message
        self.motion_start = None
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
                json.dump(self.report(), stream, allow_nan=False, indent=2)
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
