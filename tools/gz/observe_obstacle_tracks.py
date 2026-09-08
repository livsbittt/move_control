"""Record tracking output on a static Gazebo world before granting authority."""
import json
import os
from pathlib import Path
import time

import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import String


def main():
    assert os.environ.get('ROS_DOMAIN_ID') == '227'
    rclpy.init()
    node = rclpy.create_node('obstacle_track_audit', parameter_overrides=[Parameter('use_sim_time', value=True)])
    rows = []
    node.create_subscription(String, '/obstacles/tracks', lambda m: rows.append(json.loads(m.data)), 10)
    start = time.monotonic()
    while time.monotonic()-start < 60:
        rclpy.spin_once(node, timeout_sec=.2)
    result = dict(samples=len(rows), moving=sum(t['state']=='moving' for r in rows for t in r['tracks']),
                  stationary=sum(t['state']=='stationary' for r in rows for t in r['tracks']), rows=rows)
    Path('/root/pinky-dynamic-tracks.json').write_text(json.dumps(result, indent=2))
    print({k:v for k,v in result.items() if k!='rows'})
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
