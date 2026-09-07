"""Capture coherent live calibration/profile acknowledgement in domain 227."""
import json
import os
from pathlib import Path
import time


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '227' or os.environ.get('GZ_PARTITION') != 'pinky_calmap227':
        raise RuntimeError('Isolated simulation required')
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from std_msgs.msg import String
    rclpy.init()
    node = rclpy.create_node('rig_evidence_capture',
                            parameter_overrides=[Parameter('use_sim_time', value=True)])
    rows = {}
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    topics = {'status': '/calibration/status', 'profile': '/calibration/profile',
              'applied': '/calibration/applied', 'geometry': '/safety/profile'}
    for key, topic in topics.items():
        node.create_subscription(String, topic,
            lambda msg, key=key: rows.__setitem__(key, (time.monotonic(), json.loads(msg.data))), qos)
    deadline = time.monotonic()+15.
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if len(rows) != len(topics) or any(time.monotonic()-seen > 1.5 for seen, _ in rows.values()):
                continue
            value = {key: row for key, (_, row) in rows.items()}
            status, profile, applied, geometry = (value[key] for key in topics)
            now = node.get_clock().now().nanoseconds*1e-9
            valid = (status.get('ready') and status.get('settings_applied') and profile.get('enabled')
                     and applied.get('applied') and geometry.get('valid')
                     and status.get('profile_revision') == profile.get('revision') == applied.get('revision')
                     and profile.get('session') == applied.get('session')
                     and profile.get('geometry_revision') == geometry.get('revision')
                     and 0 <= now-profile['issued_s'] <= profile['ttl_s'])
            if not valid:
                continue
            publishers = node.get_publishers_info_by_topic('/cmd_vel')
            if len(publishers) != 1 or publishers[0].node_name != 'safety_node':
                continue
            value['motor_publishers'] = [publisher.node_name for publisher in publishers]
            out = Path('/tmp/pinky-calmap227')
            value.update(run_id=json.loads((out/'run_manifest.json').read_text())['run_id'], sim_s=now)
            temporary = out/'live_evidence.pending'
            temporary.write_text(json.dumps(value, indent=2))
            temporary.replace(out/'live_evidence.json')
            print(json.dumps({'coherent': True, 'sim_s': now, 'revision': profile['revision']}))
            return
        raise RuntimeError('No fresh, coherent applied calibration evidence')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
