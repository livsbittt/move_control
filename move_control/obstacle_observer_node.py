"""Timestamped scan tracking adapter. Publishes evidence, never velocity."""
import json
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException

from .sensing.obstacle_tracks import ObstacleTracker, scan_clusters, transform_points, observed_free, scan_plane_pose


class ObstacleObserver(Node):
    def __init__(self):
        super().__init__('obstacle_observer_node')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('tracking_frame', 'odom')
        self.declare_parameter('tracking_max_range', 2.)
        self.declare_parameter('tracking_max_extent', .15)
        self.declare_parameter('tracking_association', .15)
        self.declare_parameter('tracking_evidence_time', .6)
        self.declare_parameter('tracking_moving_speed', .06)
        self.declare_parameter('tracking_stationary_speed', .02)
        self.declare_parameter('tracking_max_tilt_rad', math.radians(5.))
        self.tracker = ObstacleTracker(
            association=float(self.get_parameter('tracking_association').value),
            evidence_time=float(self.get_parameter('tracking_evidence_time').value),
            moving_speed=float(self.get_parameter('tracking_moving_speed').value),
            stationary_speed=float(self.get_parameter('tracking_stationary_speed').value))
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.output = self.create_publisher(String, '/obstacles/tracks', 10)
        self.mapping_scan = self.create_publisher(LaserScan, '/mapping/scan', 10)
        self.pending = None
        self.processed_stamp = None
        self.extents = {}
        self.create_subscription(LaserScan, self.get_parameter('scan_topic').value,
                                 self.on_scan, qos_profile_sensor_data)
        self.create_timer(.05, self.tick)

    def on_scan(self, msg):
        # Resolve TF on a later tick so normal transport ordering is tolerated.
        self.pending = msg

    def tick(self):
        msg = self.pending
        if msg is None:
            return
        now = self.get_clock().now().nanoseconds*1e-9
        stamp_ns = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        stamp = stamp_ns*1e-9
        if self.processed_stamp is not None and stamp_ns <= self.processed_stamp:
            return
        if not 0 <= now-stamp <= .3:
            self.pending = None
            return
        frame = str(self.get_parameter('tracking_frame').value)
        try:
            transform = self.tf.lookup_transform(frame, msg.header.frame_id,
                                                  Time.from_msg(msg.header.stamp))
            t, q = transform.transform.translation, transform.transform.rotation
            pose = scan_plane_pose(t.x, t.y, (q.x, q.y, q.z, q.w),
                                   float(self.get_parameter('tracking_max_tilt_rad').value))
            if pose is None:
                return
        except TransformException:
            return
        if (not math.isfinite(msg.angle_increment) or msg.angle_increment <= 0 or
                not math.isfinite(msg.angle_min) or not 3 <= len(msg.ranges) <= 4096 or
                not all(math.isfinite(v) for v in (msg.range_min, msg.range_max)) or
                not 0 <= msg.range_min < msg.range_max or
                sum(math.isfinite(v) and msg.range_min <= v <= msg.range_max for v in msg.ranges) < 3):
            return
        groups = scan_clusters(msg.ranges, msg.angle_min, msg.angle_increment,
                               max(.05, msg.range_min), min(msg.range_max,
                               float(self.get_parameter('tracking_max_range').value)))
        # Long surfaces remain protected by the existing all-return safety gate.
        # Their changing visible centroid is not reliable object-motion evidence.
        groups = [g for g in groups if g['radius'] <= float(self.get_parameter('tracking_max_extent').value)]
        points = transform_points([g['position'] for g in groups], pose)
        tracks = self.tracker.update(points, stamp)
        cleared = []
        c, s = math.cos(pose[2]), math.sin(pose[2])
        for track in tracks:
            if track['observed'] and points:
                index = min(range(len(points)), key=lambda i: math.dist(points[i], track['position']))
                track['radius'] = groups[index]['radius']
                self.extents[track['id']] = track['radius']
            else:
                track['radius'] = self.extents.get(track['id'], float(self.get_parameter('tracking_max_extent').value))
                dx, dy = track['position'][0]-pose[0], track['position'][1]-pose[1]
                local = (c*dx+s*dy, -s*dx+c*dy)
                if observed_free(local, track['radius'], msg.ranges, msg.angle_min, msg.angle_increment):
                    cleared.append(track['id'])
        self.tracker.clear_observed_free(cleared)
        for identity in cleared:
            self.extents.pop(identity, None)
        tracks = [t for t in tracks if t['id'] not in cleared]
        if len(tracks) > 256:
            # Losing the observation lease stops the enabled gate; never drop
            # unresolved objects merely to manufacture an empty clear scene.
            self.get_logger().error('Obstacle hypothesis capacity exceeded; tracking unavailable', throttle_duration_sec=5.)
            return
        self.processed_stamp = stamp_ns
        # Preserve raw returns and source time, but do not insert tilted scans
        # into the 2-D SLAM map. The final safety gate still receives /scan.
        self.mapping_scan.publish(msg)
        self.output.publish(String(data=json.dumps(dict(frame=frame, stamp=stamp,
            tracks=tracks, status='observed', coverage='compact_scan_clusters_only'))))


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleObserver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
