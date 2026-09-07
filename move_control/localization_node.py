#!/usr/bin/env python3
"""ROS adapter for AMCL confidence, stationary updates, and global recovery."""
import json
import math
from concurrent.futures import ThreadPoolExecutor
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener
from .sensing.localization import MapAgreement, Confidence, planar_yaw


def yaw(q):
    return planar_yaw(q.x, q.y, q.z, q.w)


class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization_monitor')
        for name, value in [('scan_timeout', .5), ('pose_timeout', 2.),
                            ('minimum_agreement', .85), ('wall_tolerance', .04),
                            ('position_stddev', .035), ('yaw_stddev', .12),
                            ('stable_scans', 10), ('recovery_interval', 30.),
                            ('robot_radius', .076), ('global_minimum_agreement', .9),
                            ('global_minimum_margin', .04)]:
            self.declare_parameter(name, value)
        self.field = self.scan = self.pose = None
        self.confidence = Confidence(self.p('stable_scans'))
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.status = self.create_publisher(String, '/localization/status', 10)
        self.initial_pose = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_pose, latched)
        self.create_subscription(Bool, '/safety/pickup', self.on_pickup, 10)
        self.global_client = self.create_client(Empty, '/reinitialize_global_localization')
        self.update_client = self.create_client(Empty, '/request_nomotion_update')
        self.pending = None
        self.last_recovery = -math.inf
        self.last_update = -math.inf
        self.bad_since = None
        self.recoveries = 0
        self.search_pool = ThreadPoolExecutor(max_workers=1)
        self.search = None
        self.search_result = None
        self.global_confirmed = False
        self.picked_up = False
        self.generation = 0
        self.search_context = None
        self.recovery_episode = True
        self.previous_time = None
        self.create_timer(.1, self.tick)

    def p(self, name):
        return self.get_parameter(name).value

    def on_map(self, msg):
        self.field = None
        self.global_confirmed = False
        self.generation += 1
        self.recovery_episode = False
        self.confidence = Confidence(self.p('stable_scans'))
        try:
            rotation = yaw(msg.info.origin.orientation)
        except ValueError:
            return
        if (msg.header.frame_id != 'map' or not math.isfinite(msg.info.resolution) or msg.info.resolution <= 0 or
                msg.info.width <= 0 or msg.info.height <= 0 or
                not all(math.isfinite(v) for v in (msg.info.origin.position.x, msg.info.origin.position.y)) or
                msg.info.width * msg.info.height != len(msg.data) or
                abs(rotation) > 1e-6):
            return
        self.field = MapAgreement(np.array(msg.data).reshape(msg.info.height, msg.info.width),
                                  msg.info.resolution,
                                  (msg.info.origin.position.x, msg.info.origin.position.y),
                                  self.p('wall_tolerance'))

    def on_scan(self, msg):
        self.scan = msg

    def on_pose(self, msg):
        self.pose = msg

    def on_pickup(self, msg):
        if msg.data and not self.picked_up:
            self.generation += 1
            self.recovery_episode = False
        self.picked_up = msg.data
        if msg.data:
            self.global_confirmed = False
            self.confidence = Confidence(self.p('stable_scans'))

    def scan_arrays(self):
        scan = self.scan
        ranges = np.asarray(scan.ranges)[::4]
        angles = scan.angle_min + np.arange(len(scan.ranges))[::4] * scan.angle_increment
        return np.where((ranges < scan.range_max) & (ranges >= scan.range_min), ranges, np.nan), angles

    def finish_search(self, now):
        if self.search is None or not self.search.done():
            return
        try:
            result = self.search.result()
            self.search_result = result
            if (not result['unique'] or self.picked_up or
                    self.search_context[0] != self.generation):
                return
            current_odom = self.odom_sensor_pose(now)
            prior = self.search_context[1]
            if (math.dist(current_odom[:2], prior[:2]) > .005 or
                    abs(math.atan2(math.sin(current_odom[2]-prior[2]), math.cos(current_odom[2]-prior[2]))) > .02):
                return
            sensor = result['pose']
            ranges, angles = self.scan_arrays()
            if self.field.score(sensor, ranges, angles) < self.p('global_minimum_agreement'):
                return
            mount = self.tf.lookup_transform('base_link', self.scan.header.frame_id, rclpy.time.Time())
            base_yaw = sensor[2] - yaw(mount.transform.rotation)
            mx, my = mount.transform.translation.x, mount.transform.translation.y
            pose = PoseWithCovarianceStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.pose.position.x = sensor[0] - math.cos(base_yaw)*mx + math.sin(base_yaw)*my
            pose.pose.pose.position.y = sensor[1] - math.sin(base_yaw)*mx - math.cos(base_yaw)*my
            if not self.field.footprint_clear(pose.pose.pose.position.x, pose.pose.pose.position.y,
                                              self.p('robot_radius')):
                return
            pose.pose.pose.orientation.z = math.sin(base_yaw/2)
            pose.pose.pose.orientation.w = math.cos(base_yaw/2)
            pose.pose.covariance[0] = pose.pose.covariance[7] = .02**2
            pose.pose.covariance[35] = .04**2
            self.initial_pose.publish(pose)
            self.global_confirmed = True
            self.last_recovery = now
            self.confidence = Confidence(self.p('stable_scans'))
            self.get_logger().info('Unique map/scan candidate supplied to AMCL: ' + json.dumps(result))
        except Exception as exc:
            self.get_logger().warning('Global scan candidate rejected: ' + str(exc))
        finally:
            self.search = None

    def odom_sensor_pose(self, now):
        tf = self.tf.lookup_transform('odom', self.scan.header.frame_id, rclpy.time.Time())
        stamp = tf.header.stamp.sec + tf.header.stamp.nanosec*1e-9
        tr = tf.transform.translation
        if not 0 <= now-stamp <= self.p('scan_timeout') or not all(math.isfinite(v) for v in (tr.x, tr.y)):
            raise ValueError('No fresh finite odometry for a stationary scan snapshot')
        return tr.x, tr.y, yaw(tf.transform.rotation)

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.previous_time is not None and now < self.previous_time:
            self.generation += 1
            self.global_confirmed = False
            self.last_recovery = -math.inf
            self.bad_since = None
        self.previous_time = now
        score = 0.
        reason = 'waiting-map-scan-pose'
        good = False
        stamp = 0
        fresh_scan = False
        if self.scan is not None:
            stamp = self.scan.header.stamp.sec * 1000000000 + self.scan.header.stamp.nanosec
            fresh_scan = 0 <= now - stamp * 1e-9 <= self.p('scan_timeout')
        if fresh_scan and self.field is not None:
            self.finish_search(now)
        if self.field is not None and fresh_scan and self.pose is not None:
            pose_t = self.pose.header.stamp.sec + self.pose.header.stamp.nanosec * 1e-9
            cov = self.pose.pose.covariance
            covariance_ok = all(math.isfinite(cov[i]) and 0 <= cov[i] <= limit**2
                                for i, limit in [(0, self.p('position_stddev')),
                                                 (7, self.p('position_stddev')),
                                                 (35, self.p('yaw_stddev'))])
            try:
                # Use the sensor frame at the observation time, including the
                # real rotated/offset lidar mount; never assume scan zero is nose.
                tf = self.tf.lookup_transform('map', self.scan.header.frame_id,
                                               rclpy.time.Time.from_msg(self.scan.header.stamp))
                tr, q = tf.transform.translation, tf.transform.rotation
                ranges, angles = self.scan_arrays()
                score = self.field.score((tr.x, tr.y, yaw(q)), ranges, angles)
                good = (self.global_confirmed and not self.picked_up and covariance_ok and 0 <= now-pose_t <= self.p('pose_timeout') and
                        pose_t > self.last_recovery and score >= self.p('minimum_agreement'))
                reason = 'tracking' if good else 'uncertain-or-map-mismatch'
            except Exception:
                reason = 'waiting-scan-transform'
        ready = self.confidence.observe(stamp, good)
        if ready:
            self.recovery_episode = False
        if not ready and good:
            reason = 'confirming'
        self.status.publish(String(data=json.dumps({'ready': ready, 'stamp_ns': stamp,
            'reason': reason, 'agreement': score, 'recoveries': self.recoveries,
            'global_candidate': self.search_result})))
        if good or not fresh_scan:
            self.bad_since = None
        elif self.bad_since is None:
            self.bad_since = now
        # No sensor/map is an availability failure, not a reason to destroy a
        # valid filter. A bounded retry leaves time for global particles to settle.
        if (self.field is not None and fresh_scan and not good and not self.picked_up and self.search is None and
                now-self.bad_since >= 1. and (not self.recovery_episode or now-self.last_recovery >= self.p('recovery_interval')) and
                self.global_client.service_is_ready() and (self.pending is None or self.pending.done())):
            try:
                self.search_context = (self.generation, self.odom_sensor_pose(now))
                mount = self.tf.lookup_transform('base_link', self.scan.header.frame_id, rclpy.time.Time())
                yaw(mount.transform.rotation)
                search_radius = self.p('robot_radius') + math.hypot(mount.transform.translation.x, mount.transform.translation.y)
                if not math.isfinite(search_radius):
                    return
            except Exception:
                return
            self.pending = self.global_client.call_async(Empty.Request())
            self.last_recovery = now
            self.recoveries += 1
            self.global_confirmed = False
            self.recovery_episode = True
            self.confidence = Confidence(self.p('stable_scans'))
            ranges, angles = self.scan_arrays()
            self.search = self.search_pool.submit(self.field.global_match, ranges, angles,
                search_radius, self.p('global_minimum_agreement'), self.p('global_minimum_margin'))
            self.get_logger().warning('Localization uncertain: global AMCL recovery requested')
        # Stationary robots still have new lidar evidence. Do not repeatedly
        # reuse an old scan or invent movement to force particle updates.
        if fresh_scan and now-self.last_update >= .5 and self.update_client.service_is_ready():
            self.update_client.call_async(Empty.Request())
            self.last_update = now


def main():
    rclpy.init()
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.search_pool.shutdown(wait=False, cancel_futures=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
