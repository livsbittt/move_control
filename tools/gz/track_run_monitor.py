"""Bounded exact-track observation; records partial maps without claiming completion."""
import json
import math
import os
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import numpy as np
import rclpy
from rclpy.parameter import Parameter
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from tools.gz.prepare_track_world import clearance
from tools.gz.save_map import map_pixels


def main():
    assert os.environ.get('ROS_DOMAIN_ID') == '227'
    out = Path('/tmp/pinky-calmap227')
    identity = json.loads((out/'track_identity.json').read_text())
    rclpy.init()
    node = rclpy.create_node('track_run_monitor', parameter_overrides=[Parameter('use_sim_time', value=True)])
    state = {'calibration': {}, 'goal': '', 'wander': '', 'pose': None, 'safe': [0., 0.]}
    rows, maps = [], []
    node.create_subscription(String, '/calibration/status', lambda m: state.update(calibration=json.loads(m.data)), 10)
    node.create_subscription(String, '/goal_node/state', lambda m: state.update(goal=m.data), 10)
    node.create_subscription(String, '/wander/state', lambda m: state.update(wander=m.data), 10)
    node.create_subscription(Odometry, '/odom', lambda m: state.update(pose=[m.pose.pose.position.x, m.pose.pose.position.y]), 10)
    node.create_subscription(Twist, '/cmd_vel', lambda m: state.update(safe=[m.linear.x, m.angular.z]), 10)
    node.create_subscription(OccupancyGrid, '/map', lambda m: maps.append(m) if not maps else maps.__setitem__(0, m), 10)
    start, last, wall = None, -1., time.monotonic()
    while rclpy.ok() and time.monotonic()-wall < 600:
        rclpy.spin_once(node, timeout_sec=.1)
        now = node.get_clock().now().nanoseconds*1e-9
        if now <= 0:
            continue
        if start is None:
            start = now
        if now-last >= .5:
            last = now
            row = {'sim_s': now, **state, 'calibration': state['calibration'].get('phase'),
                   'ready': state['calibration'].get('ready'), 'message': state['calibration'].get('message')}
            rows.append(json.loads(json.dumps(row)))
        if state['calibration'].get('phase') in ('failed', 'aborted') or now-start >= float(os.environ.get('RIG_DURATION', '180')):
            break
    (out/'track_samples.json').write_text(json.dumps(rows, indent=2))
    (out/'track_last_status.json').write_text(json.dumps(state, indent=2))
    points = np.array([r['pose'] for r in rows if r['pose'] is not None])
    stats = {'elapsed_sim_s': (last-start) if start else 0, 'calibration_phase': state['calibration'].get('phase'),
             'calibration_ready': state['calibration'].get('ready'), 'message': state['calibration'].get('message'),
             'map_received': bool(maps), 'mapping_complete': False,
             'cmd_vel_publishers': [i.node_name for i in node.get_publishers_info_by_topic('/cmd_vel')]}
    if len(points):
        stats.update(path_m=float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum()),
                     min_center_to_wall_m=float(clearance(points, identity['walls']).min()))
    if maps:
        msg = maps[0]
        arr = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        info = msg.info
        np.savez(out/'track_map.npz', data=arr, origin=[info.origin.position.x, info.origin.position.y], resolution=info.resolution)
        pixels = map_pixels(arr)
        (out/'track_map.pgm').write_bytes(f'P5\n{info.width} {info.height}\n255\n'.encode()+pixels.tobytes())
        (out/'track_map.yaml').write_text(f'image: track_map.pgm\nresolution: {info.resolution}\norigin: [{info.origin.position.x}, {info.origin.position.y}, 0.]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
        stats['known_cells'] = int((arr >= 0).sum())
    (out/'track_result.json').write_text(json.dumps(stats, indent=2))
    print(json.dumps(stats), flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
