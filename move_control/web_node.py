#!/usr/bin/env python3
"""Web node: live map + control state in a browser.

    ros2 run move_control web_node
    ros2 launch move_control web.launch.py
    frontend  http://localhost:28161   (port param — the page)
    backend   http://localhost:28162   (backend_port — the API below)

Backend API (all CORS *, JSON contract unchanged since v1):
  GET  /state.json   pose/trail/map/scan/route/options + control labels
  GET  /map.png      occupancy raster (gen counter; browser refetches on change)
  GET  /camera.jpg   latest /camera/front frame, JPEG, ~4 Hz (gen counter)
  GET  /result.json  gstate + odom path total + optional check_map metrics
  POST /cmd          explore|coverage|stop        -> /goal/cmd
  POST /goal         "x,y" (map metres)           -> /goal/cmd (manual goal)
  POST /wander       start|stop                   -> /wander/cmd
  POST /estop        stop|release                 -> /estop/cmd
  POST /teleop       {"x":..,"z":..} Twist on the resolved teleop topic.
                     'auto' resolution: safety alive -> /cmd_vel_raw (every
                     Twist passes the gate); sim rig (no safety, /cmd_vel has
                     consumers) -> /cmd_vel. Re-checked every 1 s and always
                     shown in the UI.

No decision logic — a view + relay, like goal_node is thin I/O over
planning. STATE is written by ROS callbacks (spin thread) and read by the
HTTP thread under one lock. Page polls state at 333 ms.
"""
import json
import math
import os
import threading
import time
import http.server

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, Float32, String
from visualization_msgs.msg import MarkerArray

# STATE keys — the JSON contract with the page; never rename these strings.
K_TELEOP = 'teleop_topic'
K_SENSORS = 'sensors'
K_POSE = 'pose'
K_PREV = 'pose_prev'
K_TRAIL = 'trail'
K_PATH = 'path_m'
K_MAP = 'map'
K_SCAN = 'scan'
K_GOAL = 'goal_pt'
K_ROUTE = 'route'
K_OPTIONS = 'options'
K_MODE = 'mode'
K_WANDER = 'wander'
K_GSTATE = 'gstate'
K_ETA = 'eta'
K_ESTOP = 'estop'
K_OK = 'ok'
K_HEALTH = 'health'
K_VEL = 'vel'
K_CAM = 'cam'

STATE = {}
LOCK = threading.Lock()
MAP_PNG = {'bytes': None, 'gen': 0}
CAM_JPG = {'bytes': None, 'gen': 0, 't': 0.0}
TRAIL_MAX = 3000
CAM_MIN_DT = 0.25          # ~4 Hz JPEG re-encode ceiling
CAM_WIDTH = 320            # inspection scale, not documentation
CAM_QUALITY = 70


def render_png(msg):
    """OccupancyGrid -> PNG for /map.png. Colors match the canvas tokens
    (unknown #161615 / free #232322 / wall #e1e0d9) so overlays blend —
    dark unknown recedes, light walls read as structure."""
    try:
        import numpy as np
        import cv2
    except ImportError:
        if not MAP_PNG.get('warned'):
            MAP_PNG['warned'] = True
            print('web_node: numpy/opencv missing — /map.png disabled '
                  '(install python3-numpy python3-opencv)')
        return
    info = msg.info
    arr = np.array(msg.data, np.int16).reshape(info.height, info.width)
    img = np.full((info.height, info.width, 3), (22, 22, 21), np.uint8)
    img[(arr >= 0) & (arr < 65)] = (35, 35, 34)      # free
    img[arr >= 65] = (225, 224, 217)                 # wall
    ok, buf = cv2.imencode('.png', img)
    if ok:
        with LOCK:
            MAP_PNG['bytes'] = buf.tobytes()
            MAP_PNG['gen'] += 1


def render_cam(msg):
    """/camera/front -> downscaled JPEG for /camera.jpg, throttled so a slow
    encode never backpressures the executor. BGR8 like camera_detect_node
    publishes (libcamera RGB888 is BGR in memory); rgb8 gets swapped."""
    try:
        import numpy as np
        import cv2
    except ImportError:
        if not CAM_JPG.get('warned'):
            CAM_JPG['warned'] = True
            print('web_node: numpy/opencv missing — /camera.jpg disabled')
        return
    now = time.monotonic()
    if now - CAM_JPG['t'] < CAM_MIN_DT:
        return
    h, w = msg.height, msg.width
    step = max(1, msg.step)
    if h <= 0 or w <= 0 or len(msg.data) < h * step:
        return
    img = np.frombuffer(bytes(msg.data[:h * step]), np.uint8)
    img = img.reshape(h, step)[:, :w * 3].reshape(h, w, 3)
    if msg.encoding == 'rgb8':
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    if w > CAM_WIDTH:
        img = cv2.resize(img, (CAM_WIDTH, max(1, int(h * CAM_WIDTH / w))))
    ok, buf = cv2.imencode('.jpg', img,
                           [int(cv2.IMWRITE_JPEG_QUALITY), CAM_QUALITY])
    if ok:
        with LOCK:
            CAM_JPG['bytes'] = buf.tobytes()
            CAM_JPG['gen'] += 1
            CAM_JPG['t'] = now


# Sensor view: the safety-fused values wander actually consumes, plus the
# camera verdicts. The sim rig runs none of these nodes -> keys stay
# absent and the browser falls back to raw /scan buckets for F/L/R.
SENSOR_TOPICS = [
    # topic                 type     STATE key
    ('/safety/min_range',   Float32, 'F'),
    ('/safety/left_range',  Float32, 'L'),
    ('/safety/right_range', Float32, 'R'),
    ('/safety/us_range',    Float32, 'US'),
    ('/safety/rear_range',  Float32, 'rear'),
    ('/safety/rear_left',   Float32, 'rear L'),
    ('/safety/rear_right',  Float32, 'rear R'),
    ('/safety/open_range',  Float32, 'open'),
    ('/safety/corridor',    Float32, 'corridor'),
    ('/safety/blocked',     Bool,    'blocked'),
    ('/safety/cliff',       Bool,    'cliff'),
    ('/safety/tilt',        Bool,    'tilt'),
    ('/safety/pickup',      Bool,    'pickup'),
    ('/safety/rear_clear',  Bool,    'rear clear'),
    ('/safety/can_reverse', Bool,    'can reverse'),
    ('/camera/blocked',     Bool,    'cam blocked'),
    ('/camera/cliff',       Bool,    'cam cliff'),
    ('/camera/side',        Float32, 'cam side'),
    ('/camera/debug',       String,  'cam dbg'),
]


def sensor_cb(key):
    """One callback per sensor key: Bool as bool, String truncated, Float32
    rounded to mm."""
    def cb(msg):
        if isinstance(msg, Bool):
            v = bool(msg.data)
        elif isinstance(msg, String):
            v = str(msg.data)[:100]
        else:
            v = round(float(msg.data), 3)
        with LOCK:
            STATE.setdefault(K_SENSORS, {})[key] = v
    return cb


class WebNode(Node):
    def __init__(self):
        super().__init__('web_node')
        self.declare_parameter('port', 28161)
        self.declare_parameter('backend_port', 28162)
        self.declare_parameter('teleop_topic', 'auto')
        self.declare_parameter('scan_step', 4)
        # Optional map-QA metrics JSON (check_map.py output) for the result
        # panel; empty = repo map/gz_maze_metrics.json if present.
        self.declare_parameter('metrics_file', '')
        port = int(self.get_parameter('port').value)
        backend_port = int(self.get_parameter('backend_port').value)
        self.scan_step = max(1, int(self.get_parameter('scan_step').value))
        with LOCK:
            STATE[K_SENSORS] = {}
        self.teleop_target = None
        self.teleop_pub = None
        self.resolve_teleop()
        self.goal_pub = self.create_publisher(String, '/goal/cmd', 10)
        self.wander_pub = self.create_publisher(String, '/wander/cmd', 10)
        self.estop_pub = self.create_publisher(String, '/estop/cmd', 10)

        self.create_subscription(
            OccupancyGrid, '/map', self.on_map, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(
            PoseStamped, '/goal_point', self.on_goal, 10)
        self.create_subscription(Path, '/route', self.on_route, 10)
        self.create_subscription(
            MarkerArray, '/goal/options', self.on_options, 10)
        self.create_subscription(
            LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(
            Image, '/camera/front', render_cam, qos_profile_sensor_data)
        for topic, typ, key in SENSOR_TOPICS:
            self.create_subscription(
                typ, topic, sensor_cb(key), 10)
        self.create_timer(1.0, self.resolve_teleop)
        self.create_subscription(String, '/robot/mode', self.on_mode, 10)
        self.create_subscription(String, '/wander/state', self.on_wander, 10)
        self.create_subscription(
            String, '/goal_node/state', self.on_gstate, 10)
        self.create_subscription(Float32, '/goal/eta', self.on_eta, 10)
        # Same latched profile wander uses: safety publishes /estop/state
        # transient_local, so a volatile sub would never see the latch.
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Bool, '/estop/state', self.on_estop, latched)
        self.create_subscription(Bool, '/robot/ok', self.on_ok, 10)
        self.create_subscription(String, '/robot/health', self.on_health, 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_vel, 10)

        html_path = self.html_path()
        with open(html_path, 'rb') as f:
            raw = f.read()
        html = raw.replace(b'__BACKEND_PORT__', str(backend_port).encode())
        page = make_page_handler(html)
        httpd = http.server.ThreadingHTTPServer(('0.0.0.0', port), page)
        threading.Thread(target=httpd.serve_forever, daemon=True).start()
        if backend_port != port:
            api = make_api_handler(self, html)
            httpd2 = http.server.ThreadingHTTPServer(
                ('0.0.0.0', backend_port), api)
            threading.Thread(target=httpd2.serve_forever, daemon=True).start()
        else:
            api = make_api_handler(self, html)
            httpd2 = http.server.ThreadingHTTPServer(
                ('0.0.0.0', port), api)
        self.get_logger().info(
            f'web_node page on :{port} api on :{backend_port} '
            f'(teleop -> {self.teleop_target})')

    # -- ROS callbacks ----------------------------------------------------

    def on_map(self, msg):
        info = msg.info
        render_png(msg)
        with LOCK:
            STATE[K_MAP] = [info.width, info.height, info.resolution,
                            info.origin.position.x, info.origin.position.y,
                            MAP_PNG['gen']]

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with LOCK:
            STATE[K_POSE] = [round(p.x, 3), round(p.y, 3), round(yaw, 3)]
            trail = STATE.setdefault(K_TRAIL, [])
            if not trail or math.hypot(p.x - trail[-1][0],
                                       p.y - trail[-1][1]) >= 0.01:
                trail.append([round(p.x, 3), round(p.y, 3)])
                del trail[:-TRAIL_MAX]
            prev = STATE.get(K_PREV)
            if prev is not None:
                d = math.hypot(p.x - prev[0], p.y - prev[1])
                if d < 1.0:  # teleport = odom reset, not travel
                    STATE[K_PATH] = round(STATE.get(K_PATH, 0.0) + d, 2)
            STATE[K_PREV] = (p.x, p.y)

    def on_goal(self, msg):
        with LOCK:
            STATE[K_GOAL] = [round(msg.pose.position.x, 3),
                             round(msg.pose.position.y, 3)]

    def on_route(self, msg):
        with LOCK:
            STATE[K_ROUTE] = [[round(p.pose.position.x, 3),
                               round(p.pose.position.y, 3)]
                              for p in msg.poses]

    def on_options(self, msg):
        pts = []
        for m in msg.markers:
            if m.points:
                pts.append([[round(p.x, 3), round(p.y, 3)]
                            for p in m.points])
        with LOCK:
            STATE[K_OPTIONS] = pts

    def on_scan(self, msg):
        """Downsampled /scan for the nose-up polar view. Angles stay in the
        scan frame; the browser rotates by NOSE_YAW (pi + 10 deg) like the
        stack does (sensing/lidar.robot_yaw)."""
        rs = list(msg.ranges)[::self.scan_step]
        # Lidar no-return beams are inf; json.dumps would emit bare
        # Infinity, which every browser's JSON.parse rejects — the whole
        # state payload dies. Zero them: below range_min = invalid, so the
        # dial and the F/L/R buckets skip them.
        rs = [round(r, 3) if math.isfinite(r) else 0.0 for r in rs]
        with LOCK:
            STATE[K_SCAN] = {
                'amin': msg.angle_min, 'inc': msg.angle_increment * self.scan_step,
                'rmin': msg.range_min, 'rmax': msg.range_max,
                'rs': rs,
            }

    def on_mode(self, msg):
        with LOCK:
            STATE[K_MODE] = msg.data

    def on_wander(self, msg):
        with LOCK:
            STATE[K_WANDER] = msg.data

    def on_gstate(self, msg):
        with LOCK:
            STATE[K_GSTATE] = msg.data

    def on_eta(self, msg):
        with LOCK:
            STATE[K_ETA] = round(msg.data, 1)

    def on_estop(self, msg):
        with LOCK:
            STATE[K_ESTOP] = bool(msg.data)

    def on_ok(self, msg):
        with LOCK:
            STATE[K_OK] = bool(msg.data)

    def on_health(self, msg):
        with LOCK:
            STATE[K_HEALTH] = msg.data

    def on_vel(self, msg):
        with LOCK:
            STATE[K_VEL] = [round(msg.linear.x, 3), round(msg.angular.z, 3)]

    def load_metrics(self):
        """Map-QA metrics (check_map.py output) for the result panel; None
        when absent — the panel hides itself. Cached by mtime so the HTTP
        thread does not stat+parse a JSON file 12x a minute for nothing."""
        path = str(self.get_parameter('metrics_file').value)
        if not path:
            path = os.path.join(
                os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                'map', 'gz_maze_metrics.json')
        try:
            mt = os.path.getmtime(path)
        except OSError:
            with LOCK:
                self._metrics_cache = (None, None)
            return None
        with LOCK:
            cache = getattr(self, '_metrics_cache', (None, None))
        if cache[0] == mt:
            return cache[1]
        try:
            with open(path) as f:
                data = json.load(f)
        except (OSError, ValueError):
            data = None
        with LOCK:
            self._metrics_cache = (mt, data)
        return data

    def resolve_teleop(self):
        """'auto' target: safety alive -> /cmd_vel_raw (every Twist passes
        the gate); sim rig (no safety, /cmd_vel has consumers) -> /cmd_vel
        so teleop drives; otherwise /cmd_vel_raw. Re-checked every 1 s and
        always visible in the UI."""
        want = str(self.get_parameter('teleop_topic').value)
        if want == 'auto':
            if self.count_publishers('/safety/min_range') > 0:
                want = '/cmd_vel_raw'
            elif self.count_subscribers('/cmd_vel') > 0:
                want = '/cmd_vel'
            else:
                want = '/cmd_vel_raw'
        if want == self.teleop_target:
            return
        if self.teleop_pub is not None:
            self.destroy_publisher(self.teleop_pub)
        self.teleop_target = want
        self.teleop_pub = self.create_publisher(Twist, want, 10)
        with LOCK:
            STATE[K_TELEOP] = want
        self.get_logger().info(f'teleop -> {want}')

    def html_path(self):
        """Installed share copy first; source-tree fallback for running
        from the repo before colcon build."""
        try:
            p = os.path.join(get_package_share_directory('move_control'),
                             'web', 'dashboard.html')
            if os.path.isfile(p):
                return p
        except Exception:
            pass
        return os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'web', 'dashboard.html')


# POST routes that relay a fixed verb whitelist to one publisher.
POST_VERBS = {
    '/cmd': ('goal_pub', ('explore', 'coverage', 'stop')),
    '/wander': ('wander_pub', ('start', 'stop')),
    '/estop': ('estop_pub', ('stop', 'release')),
}
GOAL_BOUND = 50.0   # metres; a dashboard goal beyond this is a typo


def _handler(node, html, api):
    """HTTP handler closing over the node. api=True = backend (state/map/
    camera/result + POSTs); api=False = frontend (the page only). CORS on
    every response so the frontend page can fetch the backend."""
    class Handler(http.server.BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def send_response(self, code, *a):
            super().send_response(code, *a)
            self.send_header('Access-Control-Allow-Origin', '*')

        def _html(self):
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            self.wfile.write(html)

        def _png_jpg(self, data, typ):
            if data is None:
                self.send_response(404)
                self.end_headers()
                return
            self.send_response(200)
            self.send_header('Content-Type', typ)
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            path = self.path.split('?')[0]
            if path == '/':
                self._html()
                return
            if not api:
                self.send_response(404)
                self.end_headers()
                return
            if path == '/state.json':
                with LOCK:
                    body = json.dumps(STATE).encode()
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(body)
            elif path == '/map.png':
                with LOCK:
                    png = MAP_PNG['bytes']
                self._png_jpg(png, 'image/png')
            elif path == '/camera.jpg':
                with LOCK:
                    jpg = CAM_JPG['bytes']
                self._png_jpg(jpg, 'image/jpeg')
            elif path == '/result.json':
                with LOCK:
                    body = {'state': STATE.get(K_GSTATE),
                            'path_m': STATE.get(K_PATH, 0.0)}
                body['metrics'] = node.load_metrics()
                body = json.dumps(body).encode()
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(body)
            else:
                self.send_response(404)
                self.end_headers()

        def do_POST(self):
            if not api:
                self.send_response(404)
                self.end_headers()
                return
            ln = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(ln).decode()
            verb = POST_VERBS.get(self.path)
            if verb:
                attr, allowed = verb
                if body in allowed:
                    getattr(node, attr).publish(String(data=body))
                    self.send_response(200)
                else:
                    self.send_response(400)
            elif self.path == '/teleop':
                try:
                    d = json.loads(body)
                    tw = Twist()
                    # Clamp at desk-robot scale; the safety gate still owns
                    # the final /cmd_vel on hardware.
                    tw.linear.x = max(-0.2, min(0.2, float(d.get('x', 0.0))))
                    tw.angular.z = max(-1.0, min(1.0,
                                                 float(d.get('z', 0.0))))
                    node.teleop_pub.publish(tw)
                    self.send_response(200)
                except (ValueError, TypeError):
                    self.send_response(400)
            elif self.path == '/goal':
                xy = _parse_xy(body)
                if xy is None or max(abs(xy[0]), abs(xy[1])) > GOAL_BOUND:
                    self.send_response(400)
                else:
                    node.goal_pub.publish(
                        String(data=f'{xy[0]:.3f},{xy[1]:.3f}'))
                    self.send_response(200)
            else:
                self.send_response(404)
            self.end_headers()

    return Handler


def make_api_handler(node, html):
    return _handler(node, html, api=True)


def make_page_handler(html):
    return _handler(None, html, api=False)


def _parse_xy(text):
    """'x,y' or 'x y' -> (x, y) floats; None otherwise. Same semantics as
    planning.goals.parse_goal_cmd but local: web_node must keep running as
    a standalone script (no package-relative imports)."""
    try:
        a, b = str(text).replace(',', ' ').split()
        x, y = float(a), float(b)
    except ValueError:
        return None
    return (x, y) if (math.isfinite(x) and math.isfinite(y)) else None


def main():
    rclpy.init()
    node = WebNode()
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
