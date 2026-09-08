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
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from tools.gz.prepare_track_world import clearance
from tools.gz.save_map import map_pixels
from tools.gz.track_map_audit import measure


def main():
    assert os.environ.get('ROS_DOMAIN_ID') == '227'
    rig = Path('/tmp/pinky-calmap227')
    out = Path(os.environ.get('RIG_MONITOR_OUT', str(rig)))
    out.mkdir(parents=True, exist_ok=True)
    identity = json.loads((rig/'track_identity.json').read_text())
    if out != rig:
        (out/'track_identity.json').write_text(json.dumps(identity, indent=2))
        (out/'run_manifest.json').write_text((rig/'run_manifest.json').read_text())
    rclpy.init()
    node = rclpy.create_node('track_run_monitor', parameter_overrides=[Parameter('use_sim_time', value=True)])
    state = {'calibration': {}, 'goal': '', 'wander': '', 'pose': None, 'safe': [0., 0.]}
    rows, maps = [], []
    calibration_seen = [None]
    def on_calibration(msg):
        state['calibration'] = json.loads(msg.data)
        calibration_seen[0] = node.get_clock().now().nanoseconds*1e-9
    node.create_subscription(String, '/calibration/status', on_calibration,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(String, '/goal_node/state', lambda m: state.update(goal=m.data), 10)
    node.create_subscription(String, '/wander/state', lambda m: state.update(wander=m.data), 10)
    node.create_subscription(Odometry, '/odom', lambda m: state.update(pose=[m.pose.pose.position.x, m.pose.pose.position.y]), 10)
    node.create_subscription(Twist, '/cmd_vel', lambda m: state.update(safe=[m.linear.x, m.angular.z]), 10)
    node.create_subscription(OccupancyGrid, '/map', lambda m: maps.append(m) if not maps else maps.__setitem__(0, m), 10)
    start, last, wall = None, -1., time.monotonic()
    terminal_since = None
    observation_error = None
    while rclpy.ok() and time.monotonic()-wall < float(os.environ.get('RIG_WALL_TIMEOUT', '600')):
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
        if (os.environ.get('RIG_CALIBRATION_CASE') and now-start >= 15. and
                (calibration_seen[0] is None or now-calibration_seen[0] > 3.)):
            observation_error = 'calibration_heartbeat_missing'
            break
        terminal = (state['calibration'].get('phase') in ('failed','aborted','waiting_space','return_blocked') or
                    (os.environ.get('RIG_CALIBRATION_CASE') and state['calibration'].get('ready') is True and
                     state['calibration'].get('settings_applied') is True))
        if terminal:
            if terminal_since is None: terminal_since = now
            if now-terminal_since >= .5 and state['safe'] == [0.,0.]: break
        else:
            terminal_since = None
        if now-start >= float(os.environ.get('RIG_DURATION', '180')):
            break
    (out/'track_samples.json').write_text(json.dumps(rows, indent=2))
    (out/'track_last_status.json').write_text(json.dumps(state, indent=2))
    points = np.array([r['pose'] for r in rows if r['pose'] is not None])
    stats = {'elapsed_sim_s': (last-start) if start else 0, 'calibration_phase': state['calibration'].get('phase'),
             'calibration_ready': state['calibration'].get('ready'), 'message': state['calibration'].get('message'),
             'map_received': bool(maps), 'mapping_complete': False,
             'observation_error': observation_error,
             'cmd_vel_publishers': [i.node_name for i in node.get_publishers_info_by_topic('/cmd_vel')]}
    stats['run_id'] = json.loads((out/'run_manifest.json').read_text())['run_id']
    if len(points):
        stats.update(path_m=float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum()),
                     min_center_to_wall_m=float(clearance(points, identity['walls']).min()))
    if maps:
        msg = maps[0]
        source_s = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        rotation = msg.info.origin.orientation
        origin = msg.info.origin.position
        stats['map_geometry_and_time_valid'] = (
            msg.header.frame_id == 'map' and source_s > 0 and
            -.1 <= node.get_clock().now().nanoseconds*1e-9-source_s <= 2. and
            all(math.isfinite(v) for v in (origin.x, origin.y, rotation.x, rotation.y, rotation.z, rotation.w)) and
            max(abs(rotation.x), abs(rotation.y), abs(rotation.z)) < 1e-6 and
            abs(abs(rotation.w)-1.) < 1e-6)
        arr = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        info = msg.info
        np.savez(out/'track_map.npz', data=arr, origin=[info.origin.position.x, info.origin.position.y], resolution=info.resolution)
        pixels = map_pixels(arr)
        (out/'track_map.pgm').write_bytes(f'P5\n{info.width} {info.height}\n255\n'.encode()+pixels.tobytes())
        (out/'track_map.yaml').write_text(f'image: track_map.pgm\nresolution: {info.resolution}\norigin: [{info.origin.position.x}, {info.origin.position.y}, 0.]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
        stats['known_cells'] = int((arr >= 0).sum())
        quality = measure(arr, [info.origin.position.x, info.origin.position.y], info.resolution, identity['walls'])
        quality.update(run_id=stats['run_id'], map_source_s=source_s,
                       map_geometry_and_time_valid=stats['map_geometry_and_time_valid'])
        (out/'track_map_quality.json').write_text(json.dumps(quality, indent=2))
        stats['mapping_complete'] = quality['map_raster_complete'] and stats['map_geometry_and_time_valid']
    (out/'track_result.json').write_text(json.dumps(stats, indent=2))
    print(json.dumps(stats), flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
