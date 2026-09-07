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
  POST /wander       start|stop|explore|coverage  -> /wander/cmd
  POST /estop        stop|release                 -> /estop/cmd
  POST /map/reset    stop commands + reset SLAM, paused; clear map caches
  POST /map/resume   resume SLAM measurements only (no motion commands)
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
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.clock import Clock, ClockType
from move_control.sensing.map_pose import record_odom, display_pose
from move_control.sensing.lidar_mount import nose_from_quaternion
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, Float32, String
from visualization_msgs.msg import MarkerArray
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterType
try:
    from slam_toolbox.srv import Reset, Pause
except ImportError:
    Reset = Pause = None  # Viewing remains available without SLAM installed.

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
K_LIMITS = 'limits'

STATE = {}
LOCK = threading.Lock()
MAP_PNG = {'bytes': None, 'gen': 0}
CAM_JPG = {'bytes': None, 'gen': 0, 't': 0.0}
TRAIL_MAX = 3000
CAM_MIN_DT = 0.25          # ~4 Hz JPEG re-encode ceiling
CAM_WIDTH = 320            # inspection scale, not documentation
CAM_QUALITY = 70


class MapControl:
    """Serialize HTTP SLAM operations while ROS futures run on the spin thread."""
    def __init__(self, node):
        self.node = node
        self.lock = threading.Lock()
        self.pending = None
        self.map_after_ns = 0
        self.reset = node.create_client(Reset, '/slam_toolbox/reset') if Reset else None
        self.pause = node.create_client(Pause, '/slam_toolbox/pause_new_measurements') if Pause else None
        self.params = node.create_client(GetParameters, '/slam_toolbox/get_parameters')
        self.update(available=False, paused=None, busy=False, error='', epoch=0)
        threading.Thread(target=self.poll, daemon=True).start()

    def update(self, **values):
        with LOCK:
            STATE.setdefault('map_control', {}).update(values)

    def snapshot(self):
        with LOCK:
            return dict(STATE['map_control'])

    def call(self, client, request):
        if self.pending is not None and not self.pending.done():
            raise RuntimeError('Previous SLAM request is still pending')
        if client is None or not client.service_is_ready():
            raise ConnectionError('SLAM service is unavailable')
        future = client.call_async(request)
        self.pending = future
        ready = threading.Event()
        future.add_done_callback(lambda _: ready.set())
        if not ready.wait(2.0):
            # Do not cancel: the server may still apply a timed-out toggle.
            # Retaining this future prevents a second mutation until it ends.
            raise TimeoutError('SLAM response timed out; outcome is unknown')
        return future.result()

    def read_paused(self):
        response = self.call(self.params, GetParameters.Request(
            names=['paused_new_measurements']))
        if len(response.values) != 1 or response.values[0].type != ParameterType.PARAMETER_BOOL:
            raise RuntimeError('SLAM paused state is unavailable')
        paused = response.values[0].bool_value
        self.update(paused=paused)
        return paused

    def poll(self):
        while rclpy.ok():
            if self.lock.acquire(blocking=False):
                try:
                    available = bool(self.reset and self.reset.service_is_ready())
                    self.update(available=available)
                    if available:
                        self.read_paused()
                        self.update(error='', busy=False)
                    else:
                        self.update(paused=None)
                except Exception as exc:
                    self.update(paused=None, error=str(exc))
                finally:
                    self.lock.release()
            time.sleep(2.0)

    def execute(self, action):
        if not self.lock.acquire(blocking=False):
            return 409, {'ok': False, 'message': 'Map control is busy',
                         'map_control': self.snapshot()}
        self.update(busy=True, error='')
        status, message = 200, ''
        try:
            if self.pending is not None and not self.pending.done():
                raise RuntimeError('Previous SLAM request is still pending')
            if action == 'reset':
                self.node.wander_pub.publish(String(data='stop'))
                self.node.estop_pub.publish(String(data='stop'))
                self.node.goal_pub.publish(String(data='stop'))
                response = self.call(self.reset, Reset.Request(pause_new_measurements=True)
                                     if Reset else None)
                if response.result != Reset.Response.RESULT_SUCCESS:
                    raise RuntimeError('SLAM rejected map reset')
                self.node.goal_pub.publish(String(data='reset'))
                self.node.calibration_pub.publish(String(data='retry'))
                self.map_after_ns = self.node.get_clock().now().nanoseconds
                with LOCK:
                    for key in (K_MAP, K_GOAL, K_ROUTE, K_OPTIONS, K_TRAIL, K_PREV,
                                K_POSE, 'trail_odom', 'path_exact_m'):
                        STATE.pop(key, None)
                    STATE[K_PATH] = 0.0
                    STATE[K_ETA] = None
                    STATE[K_GSTATE] = 'map reset; mapping paused'
                    MAP_PNG['bytes'] = None
                    MAP_PNG['gen'] += 1
                    STATE['map_control']['epoch'] += 1
                    STATE['map_control']['paused'] = True
                message = 'Map reset; mapping paused and stop commands sent'
            elif action == 'resume':
                if self.read_paused():
                    response = self.call(self.pause, Pause.Request() if Pause else None)
                    if not response.status:
                        raise RuntimeError('SLAM rejected mapping resume')
                    if self.read_paused():
                        raise RuntimeError('SLAM still reports mapping paused')
                message = 'Mapping active; robot motion remains unchanged'
            else:
                raise ValueError('Unknown map operation')
        except Exception as exc:
            status = 504 if isinstance(exc, TimeoutError) else 503 if isinstance(exc, ConnectionError) else 409
            message = str(exc)
            self.update(paused=None, error=message)
        finally:
            self.update(busy=bool(self.pending is not None and not self.pending.done()))
            self.lock.release()
        return status, {'ok': status == 200, 'message': message,
                        'map_control': self.snapshot()}


def render_png(msg, epoch=None):
    """OccupancyGrid -> PNG for /map.png. Colors match the canvas tokens
    (unknown #161615 / free #232322 / wall #e1e0d9) so overlays blend —
    dark unknown recedes, light walls read as structure."""
    try:
        from move_control.sensing.map_raster import occupancy_bgr
        import cv2
    except ImportError:
        if not MAP_PNG.get('warned'):
            MAP_PNG['warned'] = True
            print('web_node: numpy/opencv missing — /map.png disabled '
                  '(install python3-numpy python3-opencv)')
        return
    info = msg.info
    img = occupancy_bgr(msg.data, info.width, info.height)
    ok, buf = cv2.imencode('.png', img)
    if ok:
        with LOCK:
            if epoch is not None and epoch != STATE.get('map_control', {}).get('epoch', 0):
                return
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


# The distances the page draws its gauges and dial thresholds against. Same
# names as config/robot.yaml, which every launch loads first under the /**
# wildcard — so the browser shows the numbers safety is actually running on
# rather than a second copy that can drift.
LIMIT_PARAMS = [
    ('stop_distance', 0.12),
    ('clear_distance', 0.14),
    ('warn_front', 0.18),
    ('us_stop_distance', 0.020),
    ('us_clear_distance', 0.028),
    ('robot_radius', 0.076),
    ('open_max', 0.40),
]
LIMIT_KEYS = {'stop_distance': 'stop', 'clear_distance': 'clear',
              'warn_front': 'warn', 'us_stop_distance': 'us_stop',
              'us_clear_distance': 'us_clear', 'robot_radius': 'radius',
              'open_max': 'open_max'}


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
        for name, default in LIMIT_PARAMS:
            self.declare_parameter(name, default)
        port = int(self.get_parameter('port').value)
        backend_port = int(self.get_parameter('backend_port').value)
        self.scan_step = max(1, int(self.get_parameter('scan_step').value))
        with LOCK:
            STATE[K_SENSORS] = {}
            STATE[K_LIMITS] = self.read_limits()
        self.teleop_target = None
        self.teleop_pub = None
        self.resolve_teleop()
        self.goal_pub = self.create_publisher(String, '/goal/cmd', 10)
        self.wander_pub = self.create_publisher(String, '/wander/cmd', 10)
        self.estop_pub = self.create_publisher(String, '/estop/cmd', 10)
        self.calibration_pub = self.create_publisher(String, '/calibration/cmd', 10)
        with LOCK:
            STATE['calibration_ready'] = False
            STATE['calibration'] = {'phase': 'unavailable', 'ready': False,
                                    'message': 'Waiting for startup calibration', 'sensors': {}}
        self.map_control = MapControl(self)
        self.map_frame = 'map'
        self.declare_parameter('pose_timeout', 1.0)
        self.odom_received = None
        self.odom_stamp = None
        self.odom_frame = 'odom'
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.pose_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.2, self.refresh_pose, clock=self.pose_clock)

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
        self.create_subscription(String, '/safety/motion_limits', self.on_motion_limits, 10)
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
        self.create_subscription(Bool, '/calibration/ready', self.on_calibration_ready, latched)
        self.create_subscription(String, '/calibration/status', self.on_calibration_status, 10)
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
        with LOCK:
            control = STATE.get('map_control', {})
            epoch = control.get('epoch', 0)
            if epoch and control.get('paused') is not False:
                return
        stamp = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        if stamp <= self.map_control.map_after_ns and self.map_control.map_after_ns:
            return
        info = msg.info
        self.map_frame = msg.header.frame_id or 'map'
        render_png(msg, epoch)
        with LOCK:
            if epoch != STATE.get('map_control', {}).get('epoch', 0):
                return
            STATE[K_MAP] = [info.width, info.height, info.resolution,
                            info.origin.position.x, info.origin.position.y,
                            MAP_PNG['gen']]

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.odom_received = time.monotonic()
        self.odom_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.odom_frame = msg.header.frame_id or 'odom'
        with LOCK:
            record_odom(STATE, p.x, p.y, TRAIL_MAX)
        self.refresh_pose()

    def refresh_pose(self):
        """Invalidate stopped publishers even when no odometry callback arrives."""
        now = self.get_clock().now().nanoseconds * 1e-9
        age = math.inf if self.odom_received is None else max(
            time.monotonic() - self.odom_received, now - self.odom_stamp)
        if self.odom_stamp is not None and now - self.odom_stamp < -.1:
            age = now - self.odom_stamp
        pose, transform, tf_age = None, None, math.inf
        try:
            base = self.tf.lookup_transform(self.map_frame, 'base_link', Time())
            transforms = [base]
            if self.odom_frame == self.map_frame:
                transform = (0.0, 0.0, 0.0)
            else:
                # SLAM future-dates its latest map->odom by transform_timeout.
                # Match the trail transform to the authoritative base pose time.
                odom = self.tf.lookup_transform(
                    self.map_frame, self.odom_frame, Time.from_msg(base.header.stamp))
                transforms.append(odom)
                transform = self._tf_pose(odom)
            pose = self._tf_pose(base)
            ages = [now - (tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9)
                    for tf in transforms]
            tf_age = min(ages) if min(ages) < -.75 else max(ages)
        except TransformException:
            pass
        with LOCK:
            reason = 'mapping_paused' if STATE.get('map_control', {}).get('paused') else None
            display_pose(STATE, pose, transform, odom_age=age, tf_age=tf_age,
                         timeout=float(self.get_parameter('pose_timeout').value), reason=reason)
            STATE['pose_frame'] = self.map_frame

    @staticmethod
    def _tf_pose(tf):
        t, r = tf.transform.translation, tf.transform.rotation
        angle = math.atan2(2.0 * (r.w * r.z + r.x * r.y),
                           1.0 - 2.0 * (r.y * r.y + r.z * r.z))
        return (t.x, t.y, angle)

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
        """Keep scan angles raw and supply the same TF-derived nose as safety."""
        try:
            if not msg.header.frame_id:
                raise ValueError('Missing scan frame')
            tf = self.tf.lookup_transform('base_link', msg.header.frame_id,
                                          Time.from_msg(msg.header.stamp))
            q = tf.transform.rotation
            nose = nose_from_quaternion(q.x, q.y, q.z, q.w)
        except (TransformException, ValueError):
            with LOCK:
                STATE[K_SCAN] = None
                STATE['scan_reason'] = 'missing_mount_tf'
            return
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
                'nose_yaw': nose,
                'frame': msg.header.frame_id,
            }
            STATE['scan_reason'] = 'ready'

    def on_mode(self, msg):
        with LOCK:
            STATE[K_MODE] = msg.data

    def on_calibration_ready(self, msg):
        with LOCK:
            STATE['calibration_ready'] = bool(msg.data)
            STATE['calibration_received'] = time.monotonic()

    def on_calibration_status(self, msg):
        try:
            status = json.loads(msg.data)
            if not isinstance(status, dict) or not isinstance(status.get('sensors', {}), dict):
                raise ValueError('Invalid calibration status')
        except (ValueError, TypeError):
            with LOCK:
                STATE['calibration_ready'] = False
                STATE['calibration'] = {'phase': 'unavailable', 'ready': False,
                                        'message': 'Invalid calibration status', 'sensors': {}}
            return
        with LOCK:
            STATE['calibration'] = status

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

    def read_limits(self):
        """robot.yaml distances -> the JSON keys the page gauges use."""
        return {LIMIT_KEYS[name]: float(self.get_parameter(name).value)
                for name, _ in LIMIT_PARAMS}

    def on_motion_limits(self, msg):
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict):
                return
        except (ValueError, TypeError):
            return
        with LOCK:
            STATE['motion_limits'] = value
            STATE['motion_limits_received'] = time.monotonic()

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
    '/wander': ('wander_pub', ('start', 'stop', 'explore', 'coverage')),
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
                    STATE['motion_limits_fresh'] = 0 <= time.monotonic()-STATE.get('motion_limits_received', -1e9) <= .75
                    if time.monotonic() - STATE.get('calibration_received', -1e9) > 3.0:
                        STATE['calibration_ready'] = False
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
            def reject(reason):
                self.send_response(409)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({'ok': False, 'error': reason}).encode())

            with LOCK:
                ready = (time.monotonic() - STATE.get('calibration_received', -1e9) <= 3.0 and
                         STATE.get('calibration_ready') is True and
                         STATE.get('calibration', {}).get('ready') is True)
                phase = STATE.get('calibration', {}).get('phase')
                released = STATE.get(K_ESTOP) is False
                mapping_active = STATE.get('map_control', {}).get('paused') is False
            if self.path == '/calibration':
                if body not in ('retry', 'validate_motion', 'abort'):
                    self.send_response(400)
                    self.end_headers()
                    return
                if body == 'validate_motion' and (
                        phase != 'waiting_motion' or not released or not mapping_active):
                    reject('Motion validation requires waiting_motion, active mapping and released emergency stop')
                    return
                if body in ('retry', 'abort'):
                    with LOCK:
                        STATE['calibration_ready'] = False
                node.calibration_pub.publish(String(data=body))
                self.send_response(200)
                self.end_headers()
                return
            if self.path in ('/map/reset', '/map/resume'):
                status, result = node.map_control.execute(self.path.rsplit('/', 1)[1])
                self.send_response(status)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps(result).encode())
                return
            verb = POST_VERBS.get(self.path)
            if verb:
                attr, allowed = verb
                if body in allowed:
                    if self.path == '/wander' and body != 'stop' and not ready:
                        reject('Startup calibration must pass before driving')
                        return
                    if self.path == '/wander' and body != 'stop' and not released:
                        reject('Release emergency stop before selecting a driving mode')
                        return
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
                    if (tw.linear.x or tw.angular.z) and not ready:
                        reject('Startup calibration must pass before teleoperation')
                        return
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
