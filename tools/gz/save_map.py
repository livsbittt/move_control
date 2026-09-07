#!/usr/bin/env python3
"""Save the live SLAM map to pgm+yaml without map_saver_cli.

map_saver_cli is a late-joining CLI node and this machine's DDS graph
routinely refuses late joiners after long runs (the monitor's proven QoS
works). Writes the same trinary pgm+yaml contract as map_saver: occ<0.65
-> 254 free, p>0.65 -> 0 occupied, else 205 unknown.

Usage: python3 tools/gz/save_map.py [out_prefix] [wait_s]
"""
import math
import re
import sys
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid


def main():
    prefix = sys.argv[1] if len(sys.argv) > 1 else 'map/gz_maze'
    wait_s = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
    rclpy.init()
    n = rclpy.create_node('map_dump')
    got = [None]
    n.create_subscription(
        OccupancyGrid, '/map', lambda m: got.__setitem__(0, m),
        qos_profile_sensor_data)
    ex = SingleThreadedExecutorShim()
    ex.add_node(n)
    t0 = time.monotonic()
    while time.monotonic() - t0 < wait_s and got[0] is None:
        ex.spin_once(0.1)
    rclpy.shutdown()
    if got[0] is None:
        print('NO /map received — rig down?')
        sys.exit(1)
    msg = got[0]
    info = msg.info
    arr = np.array(msg.data, np.int16).reshape(info.height, info.width)
    # map_saver trinary: v = -1 unknown; p_occ = v/100 for v >= 0.
    occ = arr >= 65
    free = (arr >= 0) & (arr < 65)
    img = np.full(arr.shape, 205, np.uint8)
    img[free] = 254
    img[occ] = 0
    with open(prefix + '.pgm', 'wb') as f:
        f.write(f'P5\n{info.width} {info.height}\n255\n'.encode())
        f.write(img.tobytes())
    ox, oy = info.origin.position.x, info.origin.position.y
    yaw = 0.0
    with open(prefix + '.yaml', 'w') as f:
        f.write(f'image: {prefix.split("/")[-1]}.pgm\n'
                f'mode: trinary\nresolution: {info.resolution:.3f}\n'
                f'origin: [{ox:.3f}, {oy:.3f}, {yaw:.3f}]\n'
                f'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    print(f'saved {info.width}x{info.height} -> {prefix}.pgm/.yaml '
          f'(occ {int(occ.sum())}, free {int(free.sum())})')


class SingleThreadedExecutorShim:
    """Minimal spin_once wrapper (avoids importing the executor twice)."""

    def __init__(self):
        from rclpy.executors import SingleThreadedExecutor
        self._ex = SingleThreadedExecutor()

    def add_node(self, n):
        self._ex.add_node(n)

    def spin_once(self, t):
        self._ex.spin_once(t)


if __name__ == '__main__':
    main()
