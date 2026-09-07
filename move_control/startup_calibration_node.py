"""Stationary startup calibration, then explicitly requested safe motion validation."""
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
from std_msgs.msg import Bool, String, UInt16MultiArray
from tf2_ros import Buffer, TransformListener

from .control.calibration import (StationaryBaseline, MOTION_SPEED, MOTION_SECONDS,
                                  MOTION_LIMIT, motion_evidence, motion_result, wrap)
from .sensing.lidar import NOSE_YAW, is_robot_scan, sector_range
from .sensing.lidar_mount import nose_from_quaternion


class StartupCalibrationNode(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__('startup_calibration_node', parameter_overrides=parameter_overrides or [])
        self.declare_parameter('lidar_yaw_offset', NOSE_YAW)
        self.declare_parameter('result_path', str(Path.home() / '.local/state/move_control/calibration.json'))
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.status_pub = self.create_publisher(String, '/calibration/status', latched)
        self.ready_pub = self.create_publisher(Bool, '/calibration/ready', latched)
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
        self.baseline = StationaryBaseline()
        self.phase, self.message = 'collecting', 'Keep robot stationary on safe level floor'
        self.started = time.monotonic()
        self.motion_start = None
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
        self.raw_pub.publish(Twist())

    def stamped(self, msg, max_age=1.):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        return -.2 <= age <= max_age

    def add(self, name, values, valid=True):
        self.baseline.add(name, values, time.monotonic(), valid)

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
        self.add('lidar', (distance,), valid)

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

    def on_ir(self, msg):
        values = tuple(msg.data[:3])
        self.add('ir', values if len(values) == 3 else (0, 0, 0),
                 len(values) == 3 and all(0 < value < 4000 for value in values))

    def on_us(self, msg):
        self.add('us', (msg.range,), self.stamped(msg) and max(.02, msg.min_range) < msg.range < .8)

    def on_imu(self, msg):
        a, g, q = msg.linear_acceleration, msg.angular_velocity, msg.orientation
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        roll = math.atan2(2*(q.w*q.x + q.y*q.z), 1 - 2*(q.x*q.x + q.y*q.y))
        pitch = math.asin(max(-1., min(1., 2*(q.w*q.y - q.z*q.x))))
        gravity = math.sqrt(a.x*a.x+a.y*a.y+a.z*a.z)
        gyro = math.sqrt(g.x*g.x+g.y*g.y+g.z*g.z)
        tilt = max(abs(roll), abs(pitch))
        self.add('imu', (gravity, gyro, tilt,
                         g.x, g.y, g.z, a.x, a.y, a.z, roll, pitch),
                 self.stamped(msg) and .9 <= norm <= 1.1 and 8 <= gravity <= 11.5 and
                 gyro < .15 and tilt < math.radians(20))

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

    def safe_motion(self, now):
        if self.estop is not False:
            return 'Emergency stop must be explicitly released'
        if not self.baseline.fresh(now):
            return 'Sensor data became stale or invalid'
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
            self.phase, self.message = 'validating_motion', 'Waiting for wander stop before bounded forward validation'
            self.requested = now
            self.publish()

    def snapshot(self):
        return {name: self.baseline.latest(name) for name in ('odom', 'lidar', 'us', 'map_tf')}

    def tick(self):
        now = time.monotonic()
        self.read_tf()
        if self.phase == 'ready' and not self.baseline.fresh(now):
            self.sensors = self.baseline.report(now)
            self.finish(False, 'Calibration readiness revoked: sensor or map became stale/invalid')
            return
        if self.phase in ('collecting', 'waiting_motion'):
            self.sensors = self.baseline.report(now)
            valid = all(sensor['ok'] for sensor in self.sensors.values())
            self.phase = 'waiting_motion' if valid else 'collecting'
            if valid:
                self.baseline_values = self.baseline.statistics(now)
            self.message = ('Stationary baseline passed; explicit motion validation required' if valid
                            else 'Keep stationary; waiting for healthy stable sensors')
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
            evidence = motion_evidence(self.motion_start[1], self.snapshot())
            self.motion = evidence
            if (evidence['forward_m'] < -.005 or abs(evidence['lateral_m']) > .02 or
                    abs(evidence['yaw_drift_rad']) > .15):
                self.finish(False, 'Unexpected motion direction or yaw drift')
                return
            if now - self.motion_start[0] >= MOTION_SECONDS or evidence['distance_m'] >= MOTION_LIMIT:
                passed, checks = motion_result(evidence)
                self.motion['checks'] = checks
                self.finish(passed, 'Motion sensors agree' if passed else 'Motion validation failed; inspect sensor agreement')
                return
            cmd = Twist()
            cmd.linear.x = MOTION_SPEED
            self.raw_pub.publish(cmd)
        if now - self.last_report >= .5:
            self.publish()

    def report(self):
        imu = self.baseline_values.get('imu', {}).get('mean', [])
        lidar = self.baseline_values.get('lidar', {}).get('mean', [])
        us = self.baseline_values.get('us', {}).get('mean', [])
        return {'phase': self.phase, 'ready': self.phase == 'ready', 'message': self.message,
                'elapsed_s': round(time.monotonic() - self.started, 2),
                'sensors': self.sensors, 'motion': self.motion, 'baseline': self.baseline_values,
                'estimates': {'imu_gyro_bias_rad_s': imu[3:6], 'imu_gravity_mean_mps2': imu[6:9],
                              'imu_roll_pitch_baseline_rad': imu[9:11],
                              'lidar_us_range_difference_m': lidar[0] - us[0] if lidar and us else None},
                'recorded_unix_s': time.time(),
                'limits': {'motion_speed_mps': MOTION_SPEED, 'motion_seconds': MOTION_SECONDS,
                           'motion_distance_m': MOTION_LIMIT},
                'settings_applied': False}

    def publish(self):
        self.last_report = time.monotonic()
        self.status_pub.publish(String(data=json.dumps(self.report(), allow_nan=False)))
        self.ready_pub.publish(Bool(data=self.phase == 'ready'))

    def finish(self, passed, message, phase=None):
        self.zero()
        self.phase = phase or ('ready' if passed else 'failed')
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
