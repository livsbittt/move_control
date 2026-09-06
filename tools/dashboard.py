#!/usr/bin/env python3
"""Web dashboard: live view + control of the planning stack in a browser.

python3 tools/dashboard.py --ros-args -p use_sim_time:=true
Open http://localhost:8080

Map, robot pose, goal, option routes and ETA render on a canvas; the
explore/coverage/stop buttons publish /goal/cmd; teleop publishes Twist
on /cmd_vel (SIM ONLY — on hardware the safety gate owns /cmd_vel, so
wire teleop through wander's command surface instead).
"""
import json
import math
import os
import threading
import http.server

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Float32
from visualization_msgs.msg import MarkerArray

STATE = {}
LOCK = threading.Lock()
PUBS = {}


def cb_map(msg):
    info = msg.info
    data = [0 if v >= 65 else (1 if 0 <= v < 65 else 2) for v in msg.data]
    with LOCK:
        STATE['map'] = [info.width, info.height, info.resolution,
                        info.origin.position.x, info.origin.position.y,
                        data]


def cb_odom(msg):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    with LOCK:
        STATE['pose'] = [round(p.x, 3), round(p.y, 3), round(yaw, 3)]


def cb_goal(msg):
    with LOCK:
        STATE['goal'] = [round(msg.pose.position.x, 3),
                         round(msg.pose.position.y, 3)]


def cb_route(msg):
    with LOCK:
        STATE['route'] = [[round(p.pose.position.x, 3),
                           round(p.pose.position.y, 3)] for p in msg.poses]


def cb_options(msg):
    pts = []
    for m in msg.markers:
        if m.points:
            pts.append([[round(p.x, 3), round(p.y, 3)] for p in m.points])
    with LOCK:
        STATE['options'] = pts


def cb_state(msg):
    with LOCK:
        STATE['state'] = msg.data


def cb_eta(msg):
    with LOCK:
        STATE['eta'] = round(msg.data, 1)


def make_handler():
    html = open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             'dashboard.html'), 'rb').read()
    """Build the HTTP request handler with access to STATE/LOCK/PUBS."""

    class Handler(http.server.BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            if self.path == '/':
                self.send_response(200)
                self.send_header('Content-Type', 'text/html')
                self.end_headers()
                self.wfile.write(html)
            elif self.path == '/state.json':
                with LOCK:
                    body = json.dumps(STATE).encode()
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                self.wfile.write(body)
            else:
                self.send_response(404)
                self.end_headers()

        def do_POST(self):
            if self.path == '/cmd':
                ln = int(self.headers.get('Content-Length', 0))
                mode = self.rfile.read(ln).decode()
                if mode in ('explore', 'coverage', 'stop'):
                    PUBS['cmd'].publish(String(data=mode))
                    self.send_response(200)
                else:
                    self.send_response(400)
                self.end_headers()
            elif self.path == '/teleop':
                ln = int(self.headers.get('Content-Length', 0))
                d = json.loads(self.rfile.read(ln))
                tw = Twist()
                tw.linear.x = float(d.get('x', 0.0))
                tw.angular.z = float(d.get('z', 0.0))
                PUBS['teleop'].publish(tw)
                self.send_response(200)
                self.end_headers()
            else:
                self.send_response(404)
                self.end_headers()

    return Handler


def main():
    rclpy.init()
    node = rclpy.create_node('dashboard')
    node.declare_parameter('port', 8080)
    node.declare_parameter('teleop_topic', '/cmd_vel')
    port = int(node.get_parameter('port').value)
    teleop = str(node.get_parameter('teleop_topic').value)
    PUBS['cmd'] = node.create_publisher(String, '/goal/cmd', 10)
    PUBS['teleop'] = node.create_publisher(Twist, teleop, 10)
    node.create_subscription(OccupancyGrid, '/map', cb_map,
                             qos_profile_sensor_data)
    node.create_subscription(Odometry, '/odom', cb_odom, 10)
    node.create_subscription(PoseStamped, '/goal_point', cb_goal, 10)
    node.create_subscription(Path, '/route', cb_route, 10)
    node.create_subscription(MarkerArray, '/goal/options', cb_options, 10)
    node.create_subscription(String, '/goal_node/state', cb_state, 10)
    node.create_subscription(Float32, '/goal/eta', cb_eta, 10)
    handler = make_handler()
    httpd = http.server.ThreadingHTTPServer(('0.0.0.0', port), handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    node.get_logger().info(f'dashboard on http://localhost:{port}')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
