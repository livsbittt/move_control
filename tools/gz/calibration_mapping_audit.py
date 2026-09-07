"""Independent map evidence for the isolated calibration rig, never a driver."""
import json
import hashlib
import math
import os
from pathlib import Path
import sys
import time
import uuid

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import numpy as np
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import String
from tools.gz.check_map import measure, parse_sdf_walls
from tools.gz.save_map import map_pixels


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '227':
        raise RuntimeError('Audit requires the isolated test domain')
    out = Path(os.environ.get('RIG_AUDIT_OUT', '/tmp/pinky-calmap227'))
    manifest = json.loads((out/'run_manifest.json').read_text())
    if hashlib.sha256((out/'world.sdf').read_bytes()).hexdigest() != manifest['world_sha256']:
        raise RuntimeError('World asset differs from the run manifest')
    walls = parse_sdf_walls(str(out/'world.sdf'))
    rclpy.init()
    node = rclpy.create_node('calibration_map_snapshot_audit',
                            parameter_overrides=[Parameter('use_sim_time', value=True)])
    last = [0.]
    last_map_source = [None]
    motion = {'path_m': 0., 'min_center_wall_m': 10., 'max_linear_mps': 0.,
              'samples': 0, 'first_source_s': None, 'last_source_s': None}
    previous_pose = [None]
    last_odom_at = [None]
    odom_valid = [False]
    calibration = [None, 0.]

    def status(msg):
        calibration[:] = [json.loads(msg.data), time.monotonic()]

    node.create_subscription(String, '/calibration/status', status,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def odom(msg):
        point = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        source = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if not all(math.isfinite(v) for v in (*point, speed, source)) or source <= 0:
            odom_valid[0] = False
            return
        if motion['last_source_s'] is not None and source <= motion['last_source_s']:
            return
        odom_valid[0], last_odom_at[0] = True, time.monotonic()
        motion['samples'] += 1
        if motion['first_source_s'] is None:
            motion['first_source_s'] = source
        motion['last_source_s'] = source
        if previous_pose[0] is not None:
            motion['path_m'] += math.dist(point, previous_pose[0])
        previous_pose[0] = point
        distance = min(math.hypot(max(x0-point[0], 0., point[0]-x1),
                                 max(y0-point[1], 0., point[1]-y1)) for x0, x1, y0, y1 in walls)
        motion['min_center_wall_m'] = min(motion['min_center_wall_m'], distance)
        motion['max_linear_mps'] = max(motion['max_linear_mps'], speed)

    node.create_subscription(Odometry, '/odom', odom, 10)

    def receive(msg):
        now = time.monotonic()
        source = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        sim_now = node.get_clock().now().nanoseconds*1e-9
        origin, rotation = msg.info.origin.position, msg.info.origin.orientation
        if (msg.header.frame_id != 'map' or source <= 0 or not -.1 <= sim_now-source <= 2.
                or msg.info.width <= 0 or msg.info.height <= 0
                or len(msg.data) != msg.info.width*msg.info.height
                or not math.isfinite(msg.info.resolution) or msg.info.resolution <= 0
                or not all(math.isfinite(v) for v in (origin.x, origin.y, origin.z,
                                                      rotation.x, rotation.y, rotation.z, rotation.w))
                or max(abs(rotation.x), abs(rotation.y), abs(rotation.z)) > 1e-6
                or abs(abs(rotation.w)-1.) > 1e-6
                or (last_map_source[0] is not None and source <= last_map_source[0])):
            return
        last_map_source[0] = source
        if now-last[0] < 20.:
            return
        last[0] = now
        info = msg.info
        arr = np.array(msg.data, dtype=np.int16).reshape(info.height, info.width)
        pixels = map_pixels(arr)
        ox, oy, res = info.origin.position.x, info.origin.position.y, info.resolution
        with (out/'map.pgm').open('wb') as stream:
            stream.write(f'P5\n{info.width} {info.height}\n255\n'.encode()+pixels.tobytes())
        (out/'map.yaml').write_text(f'image: map.pgm\nresolution: {res}\norigin: [{ox}, {oy}, 0.]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
        np.savez(out/'map_grid.npz', data=arr, origin=[ox, oy], resolution=res)
        measured = measure(pixels, res, ox, oy, walls)
        metrics = {key: value for key, value in measured.items() if not isinstance(value, np.ndarray)}
        # Full-maze acceptance is stricter than the legacy 60 percent plateau gate.
        # Legacy measurements round to three decimals. Conservative half-unit
        # bounds prevent rounding a failing raster into an acceptance pass.
        metrics['map_raster_complete'] = (metrics['interior_unknown']+.0005 <= .01 and
            metrics['wall_recall'] >= .98 and metrics['corridor_purity']-.0005 >= .95 and metrics['phantom_frac']+.0005 < .02)
        metrics['sim_s'] = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        metrics['motion'] = dict(motion)
        report, seen = calibration
        # Status is advisory. rig_capture_evidence separately verifies the
        # live profile/ack/geometry/TTL and sole motor publisher.
        metrics['calibration_status_ready'] = bool(report and report.get('ready') and now-seen <= 2.)
        metrics['sampled_clearance_ok'] = bool(odom_valid[0] and motion['samples'] > 0
            and last_odom_at[0] is not None and now-last_odom_at[0] <= 1.
            and abs(metrics['sim_s']-motion['last_source_s']) <= 1.
            and motion['min_center_wall_m'] >= manifest['robot_radius_m'])
        metrics['raster_and_sampled_clearance_ok'] = (metrics['map_raster_complete']
                                                    and metrics['sampled_clearance_ok'])
        metrics['valid_until_sim_s'] = source+2.
        metrics['run_id'] = manifest['run_id']
        snapshot = out/'snapshots'/uuid.uuid4().hex
        snapshot.mkdir(parents=True)
        (snapshot/'run_manifest.json').write_text(json.dumps(manifest, indent=2))
        hashes = {}
        for name in ('map.pgm', 'map.yaml', 'map_grid.npz'):
            content = (out/name).read_bytes()
            (snapshot/name).write_bytes(content)
            hashes[name] = hashlib.sha256(content).hexdigest()
        metrics['snapshot'] = str(snapshot.relative_to(out))
        metrics['sha256'] = hashes
        payload = json.dumps(metrics, indent=2)
        (snapshot/'metrics.json').write_text(payload)
        temporary = out/'mapping_metrics.pending'
        temporary.write_text(payload)
        temporary.replace(out/'mapping_metrics.json')
        print(json.dumps(metrics), flush=True)

    node.create_subscription(OccupancyGrid, '/map', receive, qos_profile_sensor_data)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
