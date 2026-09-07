#!/usr/bin/env python3
"""gz rig monitor: watch /goal_node/state until coverage done, or fail fast.

The explore run is long (tens of minutes); nobody babysits it. This
subscribes /map, /odom, /goal_node/state, /goal_point, renders the map +
trail to /tmp/gztest/monitor/*.png every SAVE_EVERY s, and exits so the
operator can check the result instead of watching:

  exit 0  'coverage done' held for --done-hold s (probe goals exhausted,
          map complete)
  exit 2  stalled: known-cell count unchanged for --stall s while not done
  exit 1  --timeout s elapsed without done
"""
import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))  # tools/gz -> repo root

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from tools.gz.save_map import map_pixels

SAVE_EVERY = 30.0     # s wall time between PNG saves
HEARTBEAT = 10.0      # s between stdout status lines
DONE_HOLD = 60.0      # s 'coverage done' must persist
MIN_KNOWN = 500       # cells: a done map smaller than this is a lie


class Mon(Node):
    def __init__(self, a):
        super().__init__('map_run_monitor')
        self.out = a.out
        self.done_hold = a.done_hold
        self.stall_s = a.stall
        self.timeout = a.timeout
        self.t0 = time.monotonic()
        self.state = ''
        self.state_t = time.monotonic()
        self.done_t = None
        self.goal = None
        self.pose = None
        self.trail = []
        self.path_m = 0.0
        self.map = None          # (res, ox, oy, h, w, arr)
        self.known = -1
        self.known_t = self.t0
        self.last_trail_t = self.t0
        self.last_save = 0.0
        self.last_beat = 0.0
        self.create_subscription(
            OccupancyGrid, '/map', self.on_map, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(
            PoseStamped, '/goal_point', self.on_goal, 10)
        self.create_subscription(String, '/goal_node/state', self.on_state, 10)
        self.timer = self.create_timer(1.0, self.tick)

    def on_map(self, msg):
        h, w = msg.info.height, msg.info.width
        arr = np.array(msg.data, dtype=np.int16).reshape(h, w)
        self.map = (msg.info.resolution, msg.info.origin.position.x,
                    msg.info.origin.position.y, h, w, arr)
        k = int((arr >= 0).sum())
        if k != self.known:  # slam republishes /map every update tick
            self.known = k
            self.known_t = time.monotonic()

    def on_odom(self, msg):
        p = msg.pose.pose.position
        if self.pose is not None:
            d = ((p.x - self.pose[0]) ** 2 + (p.y - self.pose[1]) ** 2) ** .5
            if d < 1.0:  # teleport = TF reset, not travel
                self.path_m += d
            now = time.monotonic()
            if d > 0.02 or now - self.last_trail_t > 2.0:
                # Displacement gates never fire at rig speeds: crawl is
                # mm/msg at 20 Hz. Time-sample every 2 s so the trail shows
                # where the robot SPENT time (stall spots included).
                self.trail.append((p.x, p.y))
                self.last_trail_t = now
        self.pose = (p.x, p.y)

    def on_goal(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)

    def on_state(self, msg):
        # A resumed exploration breaks the continuous completion hold,
        # even when both transitions arrive between monitor ticks.
        if not msg.data.startswith('coverage done'):
            self.done_t = None
        if msg.data != self.state:
            self.log(f"state -> {msg.data!r} path={self.path_m:.2f}m")
            self.state = msg.data
            self.state_t = time.monotonic()

    def log(self, s):
        print(f'[MON {time.monotonic() - self.t0:6.0f}s] {s}', flush=True)

    def render(self, path):
        if self.map is None:
            return
        res, ox, oy, h, w, arr = self.map
        img = np.repeat(map_pixels(arr)[:, :, None], 3, axis=2)
        for x, y in self.trail:
            px, py = int((x - ox) / res), h - 1 - int((y - oy) / res)
            if 0 <= px < w and 0 <= py < h:
                img[py, px] = (0, 0, 255)  # trail red (BGR)
        if self.pose:
            px, py = int((self.pose[0] - ox) / res), \
                h - 1 - int((self.pose[1] - oy) / res)
            if 0 <= px < w and 0 <= py < h:
                img[py, px] = (0, 255, 0)  # robot green
        if self.goal:
            px, py = int((self.goal[0] - ox) / res), \
                h - 1 - int((self.goal[1] - oy) / res)
            if 0 <= px < w and 0 <= py < h:
                img[py, px] = (255, 0, 0)  # goal blue... red in BGR
        import cv2
        cv2.imwrite(path, img)

    def tick(self):
        t = time.monotonic()
        if t - self.last_beat >= HEARTBEAT:
            self.last_beat = t
            self.log(f"state={self.state[:48]!r} path={self.path_m:.2f}m "
                     f"known={self.known} trail={len(self.trail)}")
        if t - self.last_save >= SAVE_EVERY:
            self.last_save = t
            self.render(os.path.join(self.out, 'map_live.png'))
        # goal_node appends ' eta=.. v=.. pose~..' to the status — prefix
        # match only (an exact match never fired).
        done = self.state.startswith('coverage done')
        if done:
            if self.done_t is None:
                self.done_t = t
            elif t - self.done_t >= self.done_hold:
                if self.known >= MIN_KNOWN:
                    self.render(os.path.join(self.out, 'map_done.png'))
                    self.log(f"DONE: coverage done held {self.done_hold}s, "
                             f"path={self.path_m:.2f}m known={self.known}")
                    raise SystemExit(0)
                self.done_t = t  # tiny map: keep watching, could be churn
        # Stalled: map stopped growing, never reached done (robot grinding
        # the same pocket or wedged with no route).
        if not done and self.known > 0 and \
                t - self.known_t > self.stall_s:
            self.render(os.path.join(self.out, 'map_stalled.png'))
            self.log(f"STALL: known={self.known} unchanged {self.stall_s}s, "
                     f"state={self.state!r}")
            raise SystemExit(2)
        if t - self.t0 > self.timeout:
            self.render(os.path.join(self.out, 'map_timeout.png'))
            self.log(f"TIMEOUT after {self.timeout}s, "
                     f"state={self.state!r} path={self.path_m:.2f}m "
                     f"known={self.known}")
            raise SystemExit(1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='/tmp/gztest/monitor')
    ap.add_argument('--done-hold', type=float, default=DONE_HOLD)
    ap.add_argument('--stall', type=float, default=900.0)
    ap.add_argument('--timeout', type=float, default=5400.0)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    rclpy.init()
    n = Mon(a)
    code = 1
    try:
        rclpy.spin(n)
    except SystemExit as e:
        code = int(e.code or 0)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
