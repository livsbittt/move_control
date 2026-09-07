"""Read-only ROS/web evidence capture. Never publishes or calls motion services."""
import argparse
import hashlib
import inspect
import json
import math
from pathlib import Path
import time
import urllib.request

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', required=True)
    parser.add_argument('--api', default='http://127.0.0.1:28162')
    args = parser.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = rclpy.create_node('web_map_readonly_audit')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    latest = {}
    node.create_subscription(OccupancyGrid, '/map', lambda m: latest.update(map=m),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(LaserScan, '/scan', lambda m: latest.update(scan=m),
                             qos_profile_sensor_data)
    deadline = time.monotonic() + 6
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.1)

    def fetch(path):
        return urllib.request.urlopen(args.api + path, timeout=5).read()

    try:
        state = json.loads(fetch('/state.json'))
        png = fetch('/map.png')
        state_after = json.loads(fetch('/state.json'))
        (out / 'state.json').write_text(json.dumps(state, indent=2), encoding='utf-8')
        (out / 'map.png').write_bytes(png)
        grid = latest['map']
        scan = latest['scan']
        info = grid.info
        cells = np.asarray(grid.data).reshape(info.height, info.width)
        np.save(out / 'occupancy.npy', cells)
        meta = [info.width, info.height, info.resolution,
                info.origin.position.x, info.origin.position.y]
        image = cv2.imdecode(np.frombuffer(png, np.uint8), cv2.IMREAD_COLOR)
        expected = np.full((*cells.shape, 3), (22, 22, 21), np.uint8)
        expected[(cells >= 0) & (cells < 65)] = (35, 35, 34)
        expected[cells >= 65] = (225, 224, 217)
        matched = state.get('map') == state_after.get('map') and state.get('map', [])[:5] == meta
        raster_ok = matched and np.array_equal(image, expected[::-1])
        base = buffer.lookup_transform(grid.header.frame_id, 'base_link', rclpy.time.Time())
        t = base.transform.translation
        gx = math.floor((t.x - meta[3]) / meta[2])
        gy = math.floor((t.y - meta[4]) / meta[2])
        occupancy = int(cells[gy, gx]) if 0 <= gx < info.width and 0 <= gy < info.height else None
        pose = state.get('pose')
        pose_error = math.hypot(pose[0]-t.x, pose[1]-t.y) if pose else None

        # Evaluate endpoints at the acquisition timestamp, not at latest TF.
        transform = buffer.lookup_transform(grid.header.frame_id, scan.header.frame_id,
                                            rclpy.time.Time.from_msg(scan.header.stamp)).transform
        q = transform.rotation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        points = np.array([(transform.translation.x+r*math.cos(yaw+scan.angle_min+i*scan.angle_increment),
                            transform.translation.y+r*math.sin(yaw+scan.angle_min+i*scan.angle_increment))
                           for i, r in enumerate(scan.ranges)
                           if math.isfinite(r) and scan.range_min <= r <= min(scan.range_max, 2.)])
        wy, wx = np.where(cells >= 65)
        walls = np.column_stack((meta[3]+(wx+.5)*meta[2], meta[4]+(wy+.5)*meta[2]))
        distances = np.concatenate([
            np.sqrt(((chunk[:, None]-walls[None, :])**2).sum(axis=2)).min(axis=1)
            for chunk in np.array_split(points, max(1, math.ceil(len(points)/100)))
        ]) if len(points) and len(walls) else np.array([])
        from move_control import web_node
        paths = [Path(inspect.getfile(web_node)).resolve(), Path(web_node.WebNode.html_path(None)).resolve()]
        helper = paths[0].parent / 'sensing/map_raster.py'
        if helper.exists():
            paths.append(helper)
        report = {
            'captured_unix_s': time.time(), 'map_meta': meta,
            'same_map_generation': matched, 'png_matches_ros_north_up': bool(raster_ok),
            'png_is_unflipped_ros_rows': bool(matched and np.array_equal(image, expected)),
            'robot_cell': [gx, gy], 'robot_cell_occupancy': occupancy,
            'web_pose_tf_error_m': pose_error, 'pose_reason': state.get('pose_reason'),
            'scan_endpoints': len(points),
            'scan_wall_median_m': float(np.median(distances)) if len(distances) else None,
            'scan_wall_p90_m': float(np.percentile(distances, 90)) if len(distances) else None,
            'scan_within_4cm_fraction': float(np.mean(distances <= .04)) if len(distances) else None,
            'estop': state.get('estop'), 'velocity': state.get('vel'),
            'files': {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},
        }
        report['display_pass'] = bool(raster_ok and pose_error is not None and pose_error < .01
                                      and occupancy is not None and 0 <= occupancy < 65)
        (out / 'report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
        print(json.dumps(report, indent=2))
        return 0 if report['display_pass'] else 1
    except Exception as exc:
        report = {'captured_unix_s': time.time(), 'display_pass': False,
                  'audit_error': f'{type(exc).__name__}: {exc}'}
        (out / 'report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
        print(json.dumps(report, indent=2))
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
