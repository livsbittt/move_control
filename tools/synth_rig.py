#!/usr/bin/env python3
#!/usr/bin/env python3
"""Synthetic web-test rig: maze /map + animated /odom + /scan (with inf
beams) on plain rclpy — no gz, no bridge, so the dev machine's process
reaper can't kill it. Closes the loop: integrates /cmd_vel into the pose
(teleop wins for 0.5 s after each Twist, matching the safety stale gate),
otherwise follows /route so goal_node's routes drive the robot.

    python3 tools/synth_rig.py          # domain 13, feed the :28161 web
Run goal_node beside it for explore/coverage/manual-goal routing.
"""
import json, math, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, DurabilityPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String

W, H, RES = 60, 40, 0.05           # 3.0 x 2.0 m world
OX, OY = -1.5, -1.0

rclpy.init()
n = rclpy.create_node('synth_rig')
latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
p_map = n.create_publisher(OccupancyGrid, '/map', latched)
p_odom = n.create_publisher(Odometry, '/odom', 10)
p_scan = n.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)

x, y, yaw = -1.2, -0.6, 0.0
last_cmd = (0.0, 0.0, 0.0)          # (v, w, t)
route, wi = [], 0

def q(a):
    return Quaternion(x=0.0, y=0.0, z=math.sin(a / 2), w=math.cos(a / 2))

# --- map: walls + two pockets of unknown (frontiers for the brain) --------
g = OccupancyGrid()
g.info.width, g.info.height, g.info.resolution = W, H, RES
g.info.origin.position.x, g.info.origin.position.y = OX, OY
g.info.origin.orientation.w = 1.0
g.data = [0] * (W * H)
for c in range(W):
    g.data[0 * W + c] = 100
    g.data[(H - 1) * W + c] = 100
for r in range(H):
    g.data[r * W + 0] = 100
    g.data[r * W + (W - 1)] = 100
for c in range(10, 30):             # internal wall with a door
    if c != 20:
        g.data[20 * W + c] = 100
for r in range(6, 14):              # unknown pocket A (frontier source)
    for c in range(35, 45):
        g.data[r * W + c] = -1
for r in range(24, 32):             # unknown pocket B
    for c in range(8, 18):
        g.data[r * W + c] = -1

def on_cmd(m):
    global last_cmd
    last_cmd = (m.linear.x, m.angular.z, time.monotonic())

def on_route(m):
    global route, wi
    route = [(p.pose.position.x, p.pose.position.y) for p in m.poses]
    # routes republish at 1 Hz with route[0] == plan-time pose, which the
    # robot may already have passed — resume at the nearest waypoint so the
    # follower converges instead of oscillating back to the route start.
    best_i, best_d = 0, float('inf')
    for i, (wx, wy) in enumerate(route):
        d = math.hypot(wx - x, wy - y)
        if d < best_d:
            best_d, best_i = d, i
    while best_i < len(route) - 1 and math.hypot(
            route[best_i][0] - x, route[best_i][1] - y) < 0.07:
        best_i += 1
    wi = best_i

n.create_subscription(Twist, '/cmd_vel', on_cmd, 10)
n.create_subscription(Path, '/route', on_route, 10)

scan = LaserScan()
scan.angle_min, scan.angle_max = -math.pi, math.pi
scan.angle_increment = 2 * math.pi / 360
scan.range_min, scan.range_max = 0.02, 8.0

def build_scan(px, py):
    rs = []
    for i in range(360):
        a = scan.angle_min + i * scan.angle_increment
        ra = a + math.pi / 2.0       # nose = 190 deg convention-ish
        d = 2.5
        # walls of the 3x2 m box
        for wx, wy in ((px, OY), (px, OY + 2.0), (OX, py), (OX + 3.0, py)):
            d = min(d, max(0.05, abs(wx - px) + abs(wy - py) - 0.02))
        rs.append(float('inf') if i % 37 == 0 else min(d, 2.5))
    scan.ranges = rs

def on_timer():
    global x, y, yaw, route, wi
    now = time.monotonic()
    v, w = 0.0, 0.0
    if now - last_cmd[2] < 0.5:      # teleop window wins
        v, w = last_cmd[0], last_cmd[1]
    elif route:
        while wi < len(route) - 1 and math.hypot(
                route[wi][0] - x, route[wi][1] - y) < 0.06:
            wi += 1
        tx, ty = route[wi]
        d = math.hypot(tx - x, ty - y)
        if d >= 0.05:
            bearing = math.atan2(ty - y, tx - x)
            w = max(-0.8, min(0.8, wrap_pi(3.0 * wrap_pi(bearing - yaw))))
            v = 0.06 if abs(wrap_pi(bearing - yaw)) < 0.5 else 0.0
    x += v * math.cos(yaw) * 0.05
    y += v * math.sin(yaw) * 0.05
    yaw = wrap_pi(yaw + w * 0.05)

    odom = Odometry()
    odom.header.frame_id = 'odom'
    odom.child_frame_id = 'base_link'
    odom.pose.pose.position.x = x
    odom.pose.pose.position.y = y
    odom.pose.pose.orientation = q(yaw)
    p_odom.publish(odom)

    build_scan(x, y)
    scan.header.stamp = n.get_clock().now().to_msg()
    p_scan.publish(scan)

def wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi

n.create_timer(0.05, on_timer)
map_i = [0]

def on_map_timer():
    global x, y
    g.header.stamp = n.get_clock().now().to_msg()
    p_map.publish(g)
    map_i[0] += 1

n.create_timer(1.0, on_map_timer)
rclpy.spin(n)
