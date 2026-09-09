"""Domain-228-only Gazebo localization acceptance rig (never a robot entrypoint)."""
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


def isolation():
    if os.environ.get('ROS_DOMAIN_ID') != '228' or os.environ.get('GZ_PARTITION') != 'pinky_localization228':
        raise RuntimeError('Only the isolated localization simulation is allowed')


def main():
    isolation()
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import TransformStamped, Twist, PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry, Path as RosPath
    from sensor_msgs.msg import LaserScan, Range, Imu
    from std_msgs.msg import String, UInt16MultiArray
    from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster, Buffer, TransformListener
    from rosy_control.localization_node import yaw
    from rosy_control.sensing.localization import lease_ready

    class Rig(Node):
        def __init__(self):
            super().__init__('localization_acceptance_rig')
            self.tf = TransformBroadcaster(self)
            self.static = StaticTransformBroadcaster(self)
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer, self)
            self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
            self.scan_pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
            self.us_pub = self.create_publisher(Range, '/us_sensor/range', 10)
            self.imu_pub = self.create_publisher(Imu, '/imu_raw', 10)
            self.ir_pub = self.create_publisher(UInt16MultiArray, '/ir_sensor/range', 10)
            self.raw_pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)
            self.goal_pub = self.create_publisher(String, '/goal/cmd', 10)
            self.create_subscription(LaserScan, '/scan_source', self.on_scan, qos_profile_sensor_data)
            self.create_subscription(Odometry, '/ground_truth', self.on_truth, 10)
            self.create_subscription(Twist, '/cmd_vel', self.on_output, 10)
            self.create_subscription(String, '/localization/status', self.on_status, 10)
            self.create_subscription(RosPath, '/route', self.on_route, 10)
            self.create_subscription(String, '/goal_node/state', self.on_goal_state, 10)
            self.status = {}
            self.truth = None
            self.output = Twist()
            self.x = self.y = self.heading = 0.
            self.previous = None
            self.static_frame = None
            self.phase = 'initial'
            self.phase_start = None
            self.saw_hold = False
            self.route_count = 0
            self.goal_state = ''
            self.route_empty = True
            self.events = []
            self.samples = []
            self.done = False
            self.out = Path(os.environ['LOCALIZATION_OUTPUT'])
            self.log = (self.out / 'samples.jsonl').open('w')
            self.create_timer(.05, self.tick)

        def on_truth(self, msg):
            # Ground truth has NO path into odom, TF, AMCL, or its initial pose.
            self.truth = [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw(msg.pose.pose.orientation)]

        def on_output(self, msg):
            self.output = msg

        def on_status(self, msg):
            self.status = json.loads(msg.data)

        def on_route(self, msg):
            self.route_empty = not msg.poses
            if msg.poses:
                self.route_count += 1

        def on_goal_state(self, msg):
            self.goal_state = msg.data

        def on_scan(self, msg):
            if self.static_frame != msg.header.frame_id:
                t = TransformStamped()
                t.header.stamp = msg.header.stamp
                t.header.frame_id = 'base_link'
                t.child_frame_id = msg.header.frame_id
                t.transform.translation.z = .10
                t.transform.rotation.w = 1.
                self.static.sendTransform(t)
                self.static_frame = msg.header.frame_id
            if self.phase == 'scan_outage':
                return
            self.scan_pub.publish(msg)
            echo = Range()
            echo.header = msg.header
            echo.min_range, echo.max_range = .02, 8.
            front = [r for i, r in enumerate(msg.ranges)
                     if abs(msg.angle_min + i*msg.angle_increment) < .1 and math.isfinite(r)]
            echo.range = float(min(front)) if front else 8.
            self.us_pub.publish(echo)

        def change(self, phase, now, **extra):
            if phase == 'scan_outage':
                self.recoveries_before_outage = self.status['recoveries']
            self.events.append({'phase': phase, 'sim_s': now, 'route_count': self.route_count, **extra})
            print(json.dumps(self.events[-1]), flush=True)
            self.phase, self.phase_start = phase, now
            self.saw_hold = False

        def teleport(self, pose):
            x, y, theta = pose
            request = f'name: "pinky", position: {{x: {x}, y: {y}, z: 0.01}}, orientation: {{z: {math.sin(theta/2)}, w: {math.cos(theta/2)}}}'
            result = subprocess.run(['gz', 'service', '-s', '/world/pinky_maze/set_pose',
                '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean', '--timeout', '3000',
                '--req', request], capture_output=True, text=True, timeout=5)
            if result.returncode or 'true' not in result.stdout:
                raise RuntimeError('Gazebo teleport failed: ' + result.stdout + result.stderr)

        def finish(self, passed, reason):
            self.raw_pub.publish(Twist())
            self.goal_pub.publish(String(data='stop'))
            publishers = [p.node_name for p in self.get_publishers_info_by_topic('/cmd_vel')]
            if publishers != ['safety_node']:
                passed, reason = False, 'Final motor command does not have exactly one safety publisher'
            report = {'passed': passed, 'reason': reason, 'events': self.events,
                      'samples': len(self.samples), 'route_messages': self.route_count,
                      'odom_source': 'integrated final safety command, no ground-truth pose input',
                      'ground_truth_role': 'scoring only',
                      'cmd_vel_publishers': publishers}
            (self.out / 'acceptance.json').write_text(json.dumps(report, indent=2))
            self.log.close()
            print(json.dumps(report), flush=True)
            self.done = True

        def tick(self):
            if self.done:
                return
            now = self.get_clock().now().nanoseconds * 1e-9
            if not now:
                return
            if self.phase_start is None:
                self.phase_start = now
            dt = 0. if self.previous is None else max(0., min(.2, now-self.previous))
            self.previous = now
            # The velocity plant uses body-frame commands. Integrate them as a
            # wheel-odometry stand-in; teleport must never change this state.
            v, w = self.output.linear.x, self.output.angular.z
            self.x += v * math.cos(self.heading + w*dt/2) * dt
            self.y += v * math.sin(self.heading + w*dt/2) * dt
            self.heading += w * dt
            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id, odom.child_frame_id = 'odom', 'base_link'
            odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
            odom.pose.pose.orientation.z = math.sin(self.heading/2)
            odom.pose.pose.orientation.w = math.cos(self.heading/2)
            odom.twist.twist = self.output
            self.odom_pub.publish(odom)
            tf = TransformStamped()
            tf.header, tf.child_frame_id = odom.header, 'base_link'
            tf.transform.translation.x, tf.transform.translation.y = self.x, self.y
            tf.transform.rotation = odom.pose.pose.orientation
            self.tf.sendTransform(tf)
            imu = Imu()
            imu.header = odom.header
            imu.orientation = odom.pose.pose.orientation
            imu.angular_velocity.z = w
            imu.linear_acceleration.z = 9.81
            self.imu_pub.publish(imu)
            self.ir_pub.publish(UInt16MultiArray(data=[2000, 2100, 2200]))
            ready = lease_ready(self.status, now)
            estimate = error = None
            try:
                t = self.buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                estimate = [t.transform.translation.x, t.transform.translation.y, yaw(t.transform.rotation)]
                if self.truth:
                    error = [math.dist(estimate[:2], self.truth[:2]),
                             abs(math.atan2(math.sin(estimate[2]-self.truth[2]), math.cos(estimate[2]-self.truth[2])))]
            except Exception:
                pass
            sample = {'sim_s': now, 'phase': self.phase, 'ready': ready, 'status': self.status,
                      'truth': self.truth, 'estimate': estimate, 'error': error,
                      'odom': [self.x, self.y, self.heading], 'output': [v, w], 'route_empty': self.route_empty,
                      'goal_state': self.goal_state}
            self.log.write(json.dumps(sample) + '\n')
            self.log.flush()
            self.samples.append(sample)
            elapsed = now-self.phase_start
            raw = Twist()
            if self.phase in ('initial', 'teleport_translation', 'teleport_rotation'):
                # Keep requesting motion to prove that the final safety gate,
                # rather than this test driver, owns the uncertainty stop.
                raw.linear.x = .01
                if not ready:
                    self.saw_hold = True
                    if elapsed > 1. and abs(v)+abs(w) > 1e-9:
                        return self.finish(False, 'Gate passed a command during localization loss')
                if ready and error and elapsed > 1.:
                    if error[0] > .07 or error[1] > .15:
                        return self.finish(False, 'Confident but wrong pose')
                    if not self.saw_hold:
                        return self.finish(False, 'Position change never revoked localization')
                    if self.phase == 'initial':
                        self.goal_pub.publish(String(data='2.55,0.45'))
                        self.change('drive', now, error=error)
                    elif self.phase == 'teleport_translation':
                        self.change('rotate', now, error=error)
                    else:
                        self.change('scan_outage', now, error=error)
                elif elapsed > 180.:
                    return self.finish(False, 'Localization did not converge within 180 simulation seconds')
            elif self.phase == 'drive':
                raw.linear.x = .01
                if elapsed > 8.:
                    if self.route_count == 0:
                        return self.finish(False, 'No executable route before teleport: ' + self.goal_state)
                    self.raw_pub.publish(Twist())
                    self.teleport((.45, 3.45, math.pi/2))
                    self.change('teleport_translation', now)
            elif self.phase == 'rotate':
                raw.angular.z = .12
                if elapsed > 4.:
                    self.raw_pub.publish(Twist())
                    self.teleport((.45, 3.45, -math.pi/2))
                    self.change('teleport_rotation', now)
            elif self.phase == 'scan_outage':
                raw.linear.x = .01  # Deliberate stale-sensor command tests final gate.
                if elapsed > 1. and (ready or abs(v)+abs(w) > 1e-9 or not self.route_empty):
                    return self.finish(False, 'Stale localization did not stop gate and revoke route')
                if elapsed > 3.:
                    self.change('scan_restored', now)
            elif self.phase == 'scan_restored':
                if ready and error and error[0] < .07 and error[1] < .15 and not self.route_empty:
                    if self.route_count < 2:
                        return self.finish(False, 'No recovered route was observed')
                    if self.status['recoveries'] != self.recoveries_before_outage:
                        return self.finish(False, 'Transient scan outage unnecessarily reset the global filter')
                    return self.finish(True, 'Initial localization, motion, translation/yaw teleport recovery and scan outage passed')
                if elapsed > 60.:
                    return self.finish(False, 'Scan restoration did not recover')
            self.raw_pub.publish(raw)

    rclpy.init()
    component = os.environ.get('LOCALIZATION_COMPONENT', 'rig')
    if component == 'safety':
        from rosy_control.sensing.lidar import enable_simulation_scans
        enable_simulation_scans(True)
        from rosy_control.safety.node import SafetyNode
        node = SafetyNode()
    elif component == 'monitor':
        from rosy_control.localization_node import LocalizationNode
        node = LocalizationNode()
    elif component == 'goal':
        from rosy_control.goal_node import GoalNode
        node = GoalNode()
    else:
        node = Rig()
    try:
        while rclpy.ok() and not getattr(node, 'done', False):
            rclpy.spin_once(node, timeout_sec=.1)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if hasattr(node, 'raw_pub'):
            if rclpy.ok():
                node.raw_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
