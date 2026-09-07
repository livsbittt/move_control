"""Real HTTP/ROS UI integration fixture, isolated from hardware on domain 231."""
import json
import os
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
assert os.environ.get('ROS_DOMAIN_ID') == '231'
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import BatteryState, Image
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from std_msgs.msg import String, Bool
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterValue, ParameterType
from slam_toolbox.srv import Pause, Reset
from move_control.web_node import WebNode


class Fixture(Node):
    def __init__(self):
        super().__init__('dashboard_test_fixture')
        self.paused, self.estopped, self.ready = False, False, True
        self.wander, self.battery_mode = 'stop', 'live'
        self.refuse_pause = False
        self.events = []
        self.ticks = 0
        self.tf = TransformBroadcaster(self)
        self.pubs = {}
        for topic, typ in [('/battery_state', BatteryState), ('/camera/front', Image),
                           ('/map', OccupancyGrid), ('/odom', Odometry),
                           ('/calibration/status', String), ('/calibration/ready', Bool),
                           ('/wander/state', String), ('/estop/state', Bool), ('/robot/mode', String),
                           ('/robot/health', String), ('/robot/ok', Bool)]:
            self.pubs[topic] = self.create_publisher(typ, topic, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String, '/wander/cmd', self.on_wander, 10)
        self.create_subscription(String, '/estop/cmd', self.on_estop, 10)
        self.create_subscription(String, '/calibration/cmd', self.on_calibration, 10)
        self.create_subscription(String, '/dashboard_test/scenario', self.scenario, 10)
        self.create_service(GetParameters, '/slam_toolbox/get_parameters', self.parameters)
        self.create_service(Pause, '/slam_toolbox/pause_new_measurements', self.pause)
        self.create_service(Reset, '/slam_toolbox/reset', self.reset_map)
        self.create_timer(.2, self.tick)

    def log(self, topic, value):
        self.events.append([topic, value])
        Path('/tmp/dashboard-operations-events.json').write_text(json.dumps(self.events))

    def on_wander(self, msg):
        self.log('wander', msg.data)
        self.wander = 'stop' if msg.data == 'stop' else ('route_'+msg.data+':forward' if msg.data in ('explore','coverage') else 'forward')

    def on_estop(self, msg):
        self.log('estop', msg.data)
        self.estopped = msg.data == 'stop'
        if self.estopped:
            self.wander = 'stop'

    def on_calibration(self, msg):
        self.log('calibration', msg.data)
        self.ready = msg.data != 'abort'

    def scenario(self, msg):
        self.log('scenario', msg.data)
        self.battery_mode = msg.data
        self.refuse_pause = msg.data == 'refuse_pause'

    def parameters(self, req, res):
        res.values = [ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=self.paused)]
        return res

    def pause(self, req, res):
        self.log('pause', self.paused)
        res.status = not self.refuse_pause
        if res.status:
            self.paused = not self.paused
        return res

    def reset_map(self, req, res):
        self.paused = True
        res.result = Reset.Response.RESULT_SUCCESS
        return res

    def tick(self):
        self.ticks += 1
        stamp = self.get_clock().now().to_msg()
        if self.battery_mode != 'silent':
            battery = BatteryState()
            battery.header.stamp = stamp
            battery.present = self.battery_mode != 'absent'
            battery.percentage = float('nan') if self.battery_mode == 'unknown' else .72
            battery.voltage, battery.current = 7.8, -.3
            battery.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
            self.pubs['/battery_state'].publish(battery)
        image = Image(height=8, width=8, step=24, encoding='bgr8', data=bytes([60,120,180]*64))
        image.header.stamp = stamp
        self.pubs['/camera/front'].publish(image)
        grid = OccupancyGrid()
        grid.header.frame_id, grid.header.stamp = 'map', stamp
        grid.info.width, grid.info.height, grid.info.resolution = 50, 30, .02
        grid.info.origin.orientation.w = 1.
        grid.data = [100 if x in (0,49) or y in (0,29) or (x==30 and y>8) else 0 for y in range(30) for x in range(50)]
        if self.ticks % 5 == 0:
            self.pubs['/map'].publish(grid)
        odom = Odometry()
        odom.header.frame_id, odom.header.stamp = 'odom', stamp
        odom.pose.pose.orientation.w = 1.
        odom.pose.pose.position.x = .25
        odom.pose.pose.position.y = .25
        self.pubs['/odom'].publish(odom)
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id, tf.header.stamp = 'odom', 'base_link', stamp
        tf.transform.translation.x, tf.transform.translation.y = .25, .25
        tf.transform.rotation.w = 1.
        self.tf.sendTransform(tf)
        map_tf = TransformStamped()
        map_tf.header.frame_id, map_tf.child_frame_id, map_tf.header.stamp = 'map', 'odom', stamp
        map_tf.transform.rotation.w = 1.
        self.tf.sendTransform(map_tf)
        status = {'phase':'ready' if self.ready else 'aborted', 'ready':self.ready,
                  'auto_motion':True, 'message':'UI integration fixture', 'sensors':{},
                  'settings_applied':self.ready}
        self.pubs['/calibration/status'].publish(String(data=json.dumps(status)))
        self.pubs['/calibration/ready'].publish(Bool(data=self.ready))
        self.pubs['/wander/state'].publish(String(data=self.wander))
        self.pubs['/estop/state'].publish(Bool(data=self.estopped))
        self.pubs['/robot/mode'].publish(String(data='ESTOP' if self.estopped else 'IDLE'))
        self.pubs['/robot/health'].publish(String(data='isolated UI fixture'))
        self.pubs['/robot/ok'].publish(Bool(data=True))


if __name__ == '__main__':
    rclpy.init()
    nodes = [WebNode(), Fixture()]
    executor = MultiThreadedExecutor(num_threads=4)
    for node in nodes:
        executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        for node in nodes:
            node.destroy_node()
