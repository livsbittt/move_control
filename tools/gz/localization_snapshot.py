"""Read-only ROS scan/map capture for reproducing localization failures offline."""
import os
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from localization_rig import isolation
isolation()
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan

rclpy.init()
n = Node('localization_snapshot')
data = {}
def scan(msg):
    data['ranges'] = np.array(msg.ranges)[::4]
    data['angles'] = msg.angle_min + np.arange(len(msg.ranges))[::4]*msg.angle_increment
def grid(msg):
    data['grid'] = np.array(msg.data).reshape(msg.info.height, msg.info.width)
    data['resolution'] = msg.info.resolution
    data['origin'] = [msg.info.origin.position.x, msg.info.origin.position.y]
def truth(msg):
    from rosy_control.localization_node import yaw
    data['truth'] = [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw(msg.pose.pose.orientation)]
n.create_subscription(LaserScan, '/scan_source', scan, qos_profile_sensor_data)
n.create_subscription(OccupancyGrid, '/map', grid, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
n.create_subscription(Odometry, '/ground_truth', truth, 10)
for _ in range(200):
    rclpy.spin_once(n, timeout_sec=.1)
    if len(data) == 6:
        np.savez('/tmp/localization228-snapshot.npz', **data)
        from rosy_control.sensing.localization import MapAgreement
        field = MapAgreement(data['grid'], data['resolution'], data['origin'])
        print('truth', data['truth'], 'agreement', field.score(data['truth'], data['ranges'], data['angles']), flush=True)
        print('axes', data['ranges'][::45], flush=True)
        print('global', field.global_match(data['ranges'], data['angles'], .105), flush=True)
        break
n.destroy_node()
rclpy.shutdown()
