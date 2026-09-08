"""Subject: yield without mistaking an observed obstacle wait for a stuck motor."""
import json
from std_msgs.msg import String
from ..control.obstacle_risk import observation_risk, camera_hold, accept_observation


class ObstacleWait:
    def _init_obstacle_wait(self):
        self.declare_parameter('obstacle_tracking_enabled', False)
        self.declare_parameter('obstacle_tracking_margin', .02)
        self.navigation_obstacles = self.navigation_camera = None
        self.create_subscription(String, '/obstacles/tracks', self._on_obstacle_tracks, 10)
        self.create_subscription(String, '/camera/observation', self._on_obstacle_camera, 10)

    def _on_obstacle_tracks(self, msg):
        try:
            value = json.loads(msg.data)
            self.navigation_obstacles = accept_observation(value, self.navigation_obstacles,
                                                           self.now().nanoseconds*1e-9, .3)
        except (ValueError, TypeError, KeyError):
            pass

    def _on_obstacle_camera(self, msg):
        try:
            value = json.loads(msg.data)
            self.navigation_camera = accept_observation(value, self.navigation_camera,
                                                        self.now().nanoseconds*1e-9)
        except (ValueError, TypeError, KeyError):
            pass

    def _obstacle_wait(self, v, w):
        if not self.get_parameter('obstacle_tracking_enabled').value:
            return None
        now = self.now().nanoseconds*1e-9
        visual = camera_hold(self.navigation_camera, now)
        if visual:
            return visual
        if not self._odom_fresh():
            return 'obstacle_pose_unavailable'
        result = observation_risk(self.navigation_obstacles, now,
            (self.odom_x,self.odom_y,self.odom_yaw), v,
            float(self.get_parameter('robot_radius').value),
            float(self.get_parameter('obstacle_tracking_margin').value))
        if result['action'] == 'replan' and v == 0. and w != 0.:
            return None
        return result['action']+':'+result['reason'] if result['action'] != 'clear' else None
