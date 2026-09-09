"""Real HTTP/ROS session test on domain 229, paused ROS clock, no motor plant."""
import http.server
import json
import os
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
assert os.environ.get('ROS_DOMAIN_ID') == '229'
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from rosy_control import web_node as web
from rosy_control.wander.node import WanderNode


def main():
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true', '-p', 'auto_start:=false',
                    '-p', 'calibration_required:=false'])
    wander = WanderNode()
    wander.estop = False
    relay = Node('navigation_session_web_fixture')
    publishers = SimpleNamespace(wander_pub=relay.create_publisher(String, '/wander/cmd', 10),
                                 load_metrics=lambda: None)
    events = []
    output = Path('/tmp/navigation-session-web-events.json')
    def record(kind, value):
        events.append(dict(kind=kind, value=value, wall_s=time.monotonic(), ros_ns=wander.now().nanoseconds))
        output.write_text(json.dumps(events, indent=2))
    relay.create_subscription(String, '/navigation/session', lambda m: web.WebNode.on_navigation_session(relay, m), 10)
    relay.create_subscription(String, '/wander/state', lambda m: web.WebNode.on_wander(relay, m), 10)
    relay.create_subscription(String, '/goal/cmd', lambda m: record('goal', m.data), 10)
    relay.create_subscription(Twist, '/cmd_vel_raw', lambda m: record('raw', [m.linear.x, m.angular.z]), 10)
    web.STATE.update(wander='stop', map=[10,10,.02,0,0,1], pose=[.1,.1,0],
                     pose_available=True, map_control={'available':True,'paused':False},
                     estop=False, calibration_ready=True,
                     calibration={'ready':True,'phase':'ready','settings_applied':True,'sensors':{},
                                  'message':'WEB / ROS TEST FIXTURE — no motor plant'},
                     gstate='Paused simulation clock test; synthetic readiness', mode='STOP')
    steady = Clock(clock_type=ClockType.STEADY_TIME)
    def refresh():
        with web.LOCK:
            web.STATE['calibration_received'] = time.monotonic()
    relay.create_timer(.1, refresh, clock=steady)
    html = (Path(__file__).resolve().parents[1]/'web/dashboard.html').read_bytes()
    html = html.replace(b'__BACKEND_PORT__', b'28862')
    servers = [http.server.ThreadingHTTPServer(('127.0.0.1', 28861), web.make_page_handler(html)),
               http.server.ThreadingHTTPServer(('127.0.0.1', 28862), web.make_api_handler(publishers, html))]
    for server in servers:
        threading.Thread(target=server.serve_forever, daemon=True).start()
    executor = SingleThreadedExecutor()
    executor.add_node(wander)
    executor.add_node(relay)
    deadline = time.monotonic()+45
    try:
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.1)
    finally:
        wander.stop_motors()
        for server in servers:
            server.shutdown()
        executor.shutdown()
        wander.destroy_node()
        relay.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
