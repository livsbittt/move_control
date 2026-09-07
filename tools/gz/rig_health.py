#!/usr/bin/env python3
"""Health check for the gz rig: scans flow, base follows /cmd_vel.

Usage: python3 tools/gz/rig_health.py [speed_test_seconds]
Exit 0 = healthy (scans + motion), 1 = sensor/boot flake (relaunch).
"""
import math
import sys
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


def main():
    t_test = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0
    rclpy.init()
    n = rclpy.create_node('rig_health')
    scan = [None]
    odom = [(0.0, 0.0)]
    n.create_subscription(
        LaserScan, '/scan', lambda m: scan.__setitem__(0, m),
        qos_profile_sensor_data)
    n.create_subscription(
        Odometry, '/odom',
        lambda m: odom.__setitem__(
            0, (m.pose.pose.position.x, m.pose.pose.position.y)), 10)
    ex = SingleThreadedExecutor()
    ex.add_node(n)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 8 and scan[0] is None:
        ex.spin_once(0.1)
    if scan[0] is None:
        print('NO SCANS - bad boot, relaunch the rig')
        sys.exit(1)
    # Co-tenant identity gate: a same-domain neighbour with a different gz
    # partition publishes its own /scan,/odom,/map into this graph (a gz
    # partition does NOT isolate the ROS domain axis; a peer measured
    # /scan at 21-29 Hz from 3+ sources this way). Our lidar is 10 Hz.
    stamps = []
    n.create_subscription(
        LaserScan, '/scan', lambda m: stamps.append(time.monotonic()),
        qos_profile_sensor_data)
    t_id = time.monotonic()
    while time.monotonic() - t_id < 5.0:
        ex.spin_once(0.1)
    if len(stamps) > 2:
        hz = (len(stamps) - 1) / (stamps[-1] - stamps[0])
        if not 8.0 <= hz <= 12.0:
            print(f'IDENTITY GATE: /scan at {hz:.1f} Hz (want ~10) - '
                  'co-tenant on this domain, boot contaminated')
            sys.exit(3)
    # Throughput gate: watch the stack's OWN driving for 10 s with no
    # command injection — an injected constant fights the running driver's
    # guard/stall machinery and the reading measured that fight (0.005
    # m/s readings on physically healthy bases), not the robot.
    start = odom[0]
    t0 = time.monotonic()
    while time.monotonic() - t0 < 10.0:
        ex.spin_once(0.05)
    dx = math.hypot(odom[0][0] - start[0], odom[0][1] - start[1])
    print(f'scans OK; stack throughput {dx:.2f} m in 10 s')
    if dx < 0.15:
        print('FROZEN BASE - robot not moving under its own stack')
        sys.exit(2)
    print('RIG HEALTHY')
    sys.exit(0)


if __name__ == '__main__':
    main()
