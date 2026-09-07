#!/usr/bin/env python3
"""Dump /odom poses to JSONL + /map snapshots to .npz (offline diagnosis).

  python3 tools/gz/pose_logger.py /tmp/gztest14/poses.jsonl
"""
import json
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class PoseLogger(Node):
    def __init__(self, out):
        super().__init__('pose_logger')
        self.out = out
        self.f = open(out, 'a', buffering=1)
        self.map_n = 0
        self.create_subscription(
            Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(
            OccupancyGrid, '/map', self.on_map, qos_profile_sensor_data)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.f.write(json.dumps(
            [time.time(), round(p.x, 4), round(p.y, 4)]) + '\n')

    def on_map(self, msg):
        self.map_n += 1
        if self.map_n % 30:  # snapshot every 30th /map (30 s at 1 Hz republish)
            return
        arr = np.array(msg.data, dtype=np.int16).reshape(
            msg.info.height, msg.info.width)
        np.savez_compressed(
            self.out + '.map.npz', data=arr,
            res=msg.info.resolution,
            ox=msg.info.origin.position.x, oy=msg.info.origin.position.y)
        print(f'[{time.strftime("%H:%M:%S")}] map snapshot '
              f'#{self.map_n} known={int((arr >= 0).sum())}', flush=True)


def main():
    rclpy.init()
    n = PoseLogger(sys.argv[1])
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.f.close()
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
