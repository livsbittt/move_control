"""Persist reason changes and their tracking inputs while the simulation runs."""
import json
import os
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Recorder(Node):
    def __init__(self):
        super().__init__('obstacle_decision_recorder')
        self.tracks, self.last, self.events = None, None, []
        self.create_subscription(String, '/obstacles/tracks', lambda m: setattr(self, 'tracks', json.loads(m.data)), 10)
        self.create_subscription(String, '/safety/decision', self.on_decision, 10)

    def on_decision(self, msg):
        value = json.loads(msg.data)
        key = value['reason']
        if key != self.last:
            self.events.append(dict(decision=value, observation=self.tracks))
            self.last = key
            Path('/root/pinky-dynamic-evidence-0908/decision-events.json').write_text(json.dumps(self.events, indent=2))


def main():
    assert os.environ.get('ROS_DOMAIN_ID') == '227'
    rclpy.init()
    node = Recorder()
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
