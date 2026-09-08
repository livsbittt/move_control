"""Isolated Gazebo adapter: real scan/physics, explicitly virtual auxiliary sensors.

Never installed as a robot entry point. Requires an isolated test domain.
"""
import importlib
import copy
import json
import math
import os
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
if os.environ.get('ROS_DOMAIN_ID') != '227' or os.environ.get('GZ_PARTITION') != 'pinky_calmap227':
    raise RuntimeError('This rig is restricted to its isolated simulation domain')

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan, Range, Imu, Image
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String, UInt16MultiArray
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from tools.gz.inertial import body_gravity


def normalize_odometry(msg):
    """Map the verified model-origin GT convention to this rig's TF names.

    Gazebo's default base_footprint is the Pinky model origin here, the same
    planar origin used by the adapter's odom->base_link transform. This is
    an explicit simulation convention, not a general coordinate conversion.
    """
    if (msg.header.frame_id, msg.child_frame_id) != ('pinky/odom', 'pinky/base_footprint'):
        raise ValueError('Unexpected Gazebo ground-truth odometry frames')
    normalized = copy.deepcopy(msg)
    normalized.header.frame_id = 'odom'
    normalized.child_frame_id = 'base_link'
    return normalized


class Auxiliary(Node):
    def __init__(self):
        super().__init__('gazebo_calibration_adapter')
        self.tf = TransformBroadcaster(self)
        self.static = StaticTransformBroadcaster(self)
        self.imu = self.create_publisher(Imu, '/imu_raw', 10)
        self.ir = self.create_publisher(UInt16MultiArray, '/ir_sensor/range', 10)
        self.us = self.create_publisher(Range, '/us_sensor/range', 10)
        self.camera = self.create_publisher(Image, '/camera/front', 10)
        self.camera_block = self.create_publisher(Bool, '/camera/blocked', 10)
        self.camera_cliff = self.create_publisher(Bool, '/camera/cliff', 10)
        self.estop = self.create_publisher(String, '/estop/cmd', 10)
        self.command = self.create_publisher(String, '/wander/cmd', 10)
        self.odom = self.create_publisher(Odometry, '/odom', 10)
        self.create_subscription(Odometry, '/odom_gz', self.on_odom, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(String, '/calibration/status', self.on_calibration,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String, '/goal_node/state', self.on_goal, 10)
        self.create_subscription(String, '/safety/decision', self.on_decision, 10)
        self.create_subscription(String, '/wander/state', self.on_wander, 10)
        self.started = self.get_clock().now().nanoseconds*1e-9
        self.last_print = -100.
        self.released = self.mapping = False
        self.mapping_requested = None
        self.static_frame = None
        self.status = self.goal = self.wander = ''
        self.decision = {}
        self.pose = None
        self.create_timer(.1, self.tick)

    def on_scan(self, msg):
        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = msg.header.frame_id
        transform.transform.translation.z = .10
        transform.transform.rotation.w = 1.
        if self.static_frame != msg.header.frame_id:
            self.static.sendTransform(transform)
            self.static_frame = msg.header.frame_id
        from move_control.sensing.lidar import sector_range
        distance = sector_range(msg, 0., math.radians(8), pctl=.1)
        echo = Range()
        echo.header = msg.header
        echo.min_range, echo.max_range = .02, 8.
        echo.range = float(distance)
        self.us.publish(echo)

    def on_odom(self, msg):
        try:
            msg = normalize_odometry(msg)
        except ValueError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=5.)
            return
        self.odom.publish(msg)
        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id, transform.child_frame_id = msg.header.frame_id, msg.child_frame_id
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.rotation = msg.pose.pose.orientation
        self.tf.sendTransform(transform)
        self.pose = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        imu = Imu()
        imu.header = copy.deepcopy(msg.header)
        imu.header.frame_id = 'base_link'
        imu.orientation = msg.pose.pose.orientation
        imu.angular_velocity = msg.twist.twist.angular
        attitude = msg.pose.pose.orientation
        try:
            ax, ay, az = body_gravity((attitude.x, attitude.y, attitude.z, attitude.w))
        except ValueError:
            return  # Invalid attitude must not manufacture healthy IMU evidence.
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = ax, ay, az
        self.imu.publish(imu)
        self.ir.publish(UInt16MultiArray(data=[2000, 2100, 2200]))
        if os.environ.get('RIG_RENDERED_CAMERA') == '1':
            return
        picture = Image()
        picture.header = msg.header
        picture.height, picture.width, picture.step = 8, 8, 24
        picture.encoding = 'bgr8'
        picture.data = bytes([60, 120, 180]*64)
        self.camera.publish(picture)
        self.camera_block.publish(Bool(data=False))
        self.camera_cliff.publish(Bool(data=False))

    def on_calibration(self, msg):
        value = json.loads(msg.data)
        self.status = value
        now = self.get_clock().now().nanoseconds*1e-9
        if not os.environ.get('RIG_CALIBRATION_CASE') and value.get('ready') and not self.mapping and (self.mapping_requested is None or now-self.mapping_requested >= 1.):
            self.mapping_requested = now
            self.command.publish(String(data='explore'))

    def on_goal(self, msg):
        self.goal = msg.data

    def on_wander(self, msg):
        self.wander = msg.data
        if msg.data.startswith('route_explore:'):
            self.mapping = True

    def on_decision(self, msg):
        self.decision = json.loads(msg.data)

    def tick(self):
        now = self.get_clock().now().nanoseconds*1e-9
        if (self.pose and not self.released and now-self.started > 3. and
                self.estop.get_subscription_count() > 0):
            self.estop.publish(String(data='release'))
            self.released = True
        if now-self.last_print > 5.:
            self.last_print = now
            status = self.status if isinstance(self.status, dict) else {}
            print(json.dumps({'sim_s': now, 'pose': self.pose, 'calibration': status.get('phase'),
                  'observations': self.calibration_node.observations.report(now) if hasattr(self, 'calibration_node') and not self.mapping else None,
                  'message': status.get('message'), 'sensors': status.get('sensors') if not self.mapping else None,
                  'goal': self.goal, 'wander': self.wander, 'decision': self.decision}), flush=True)


def main():
    rclpy.init()
    component = os.environ.get('RIG_COMPONENT', 'all')
    clock_node = Node('rig_clock_'+component)
    clock_provider = [clock_node]
    # Change only this test process's source acceptance. Real entry points
    # keep rejecting simulation timestamps and non-C1 scan shapes.
    def simulation_scan(msg):
        return len(msg.ranges) == 720 and msg.header.frame_id.startswith('pinky/')
    for name in ('move_control.safety.bumper', 'move_control.startup_calibration_node'):
        importlib.import_module(name).is_robot_scan = simulation_scan
    # All production deadlines use the same simulation clock in this rig.
    class SimulationTime:
        def monotonic(self):
            return clock_provider[0].get_clock().now().nanoseconds*1e-9
        def __getattr__(self, name):
            return getattr(time, name)
    for name in ('move_control.safety.node', 'move_control.safety.evidence',
                 'move_control.safety.bumper', 'move_control.startup_calibration_node',
                 'move_control.calibration_rotation', 'move_control.calibration_atomic', 'move_control.wander.node',
                 'move_control.wander.senses', 'move_control.wander.judge', 'move_control.goal_node'):
        importlib.import_module(name).time = SimulationTime()
    from move_control.safety.node import SafetyNode
    from move_control.startup_calibration_node import StartupCalibrationNode
    from move_control.wander.node import WanderNode
    from move_control.goal_node import GoalNode
    factories = {'adapter': Auxiliary, 'safety': SafetyNode, 'calibration': StartupCalibrationNode,
                 'wander': WanderNode, 'goal': GoalNode}
    selected = list(factories) if component == 'all' else [component]
    nodes = [clock_node]+[factories[name]() for name in selected]
    if component != 'all':
        # Source-age and monotonic-equivalent checks must use the exact same
        # ROS clock. Two independently serviced /clock subscriptions can lag
        # one another during accelerated simulation.
        clock_provider[0] = nodes[1]
        clock_node.destroy_node()
        nodes = nodes[1:]
    executor = SingleThreadedExecutor()
    for node in nodes:
        executor.add_node(node)
    try:
        executor.spin()
    finally:
        for node in nodes:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
