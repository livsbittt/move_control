"""Subject: opt-in tracked-obstacle guard in the existing final velocity gate."""
import json

from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import TransformException

from ..control.obstacle_risk import observation_risk, camera_hold, accept_observation
from ..sensing.pose import planar_pose


class Obstacles:
    def init_obstacles(self):
        self.declare_parameter('obstacle_tracking_enabled', False)
        self.declare_parameter('obstacle_tracking_margin', .02)
        self.obstacle_observation = None
        self.camera_observation = None
        self.create_subscription(String, '/obstacles/tracks', self.on_obstacle_tracks, 10)
        self.create_subscription(String, '/camera/observation', self.on_camera_observation, 10)

    def on_camera_observation(self, msg):
        try:
            value = json.loads(msg.data)
            self.camera_observation = accept_observation(value, self.camera_observation,
                                                        self.now().nanoseconds*1e-9)
        except (TypeError, KeyError, ValueError):
            pass

    def on_obstacle_tracks(self, msg):
        try:
            value = json.loads(msg.data)
            self.obstacle_observation = accept_observation(value, self.obstacle_observation,
                                                          self.now().nanoseconds*1e-9, .3)
        except (ValueError, TypeError, KeyError):
            pass

    def obstacle_tracking_hold(self):
        if not self.get_parameter('obstacle_tracking_enabled').value:
            return None
        now = self.now().nanoseconds*1e-9
        visual_hold = camera_hold(self.camera_observation, now)
        if visual_hold:
            return visual_hold
        try:
            tf = self.lidar_tf.lookup_transform('odom', 'base_link', Time())
            stamp = tf.header.stamp.sec+tf.header.stamp.nanosec*1e-9
            t, q = tf.transform.translation, tf.transform.rotation
            pose = planar_pose(t.x, t.y, (q.x, q.y, q.z, q.w))
            if pose is None or not 0 <= now-stamp <= .3:
                return 'obstacle_pose_unavailable'
        except TransformException:
            return 'obstacle_pose_unavailable'
        result = observation_risk(self.obstacle_observation, now, pose,
            self.last_cmd.linear.x, self.robot_r,
            float(self.get_parameter('obstacle_tracking_margin').value))
        if result['action'] == 'replan' and self.last_cmd.linear.x == 0.:
            # Existing all-around swept-footprint checks still own rotation.
            return None
        if result['action'] != 'clear':
            return 'obstacle_'+result['action']+':'+result['reason']
        return None
