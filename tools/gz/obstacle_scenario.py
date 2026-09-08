"""Scripted Gazebo box: real rendered/scan observations, GT only for audit."""
import json
import math
import os
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tools.gz.prepare_track_world import clearance


class Scenario(Node):
    def __init__(self):
        super().__init__('obstacle_scenario_audit')
        self.pose = None
        self.wander = ''
        self.stage, self.stage_at = 'waiting', None
        self.events, self.rows = [], []
        self.latest = {}
        self.center = self.anchor = None
        self.last_sample = self.last_move = -1.
        self.identity = json.loads(Path('/tmp/pinky-calmap227/track_identity.json').read_text())
        self.run_id = json.loads(Path('/tmp/pinky-calmap227/run_manifest.json').read_text())['run_id']
        self.create_subscription(Odometry, '/odom_gz', self.on_odom, 10)
        self.create_subscription(String, '/wander/state', lambda m: setattr(self, 'wander', m.data), 10)
        for topic in ('/obstacles/tracks', '/safety/decision', '/camera/observation'):
            self.create_subscription(String, topic, lambda m, key=topic: self.latest.update({key: json.loads(m.data)}), 10)

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.pose = (p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))

    def service(self, name, kind, request):
        result = subprocess.run(['gz', 'service', '-s', '/world/map_260905/'+name,
            '--reqtype', 'gz.msgs.'+kind, '--reptype', 'gz.msgs.Boolean', '--timeout', '2000',
            '--req', request], capture_output=True, text=True, timeout=4)
        if result.returncode or 'true' not in result.stdout:
            raise RuntimeError(name+': '+result.stdout+result.stderr)

    def spawn(self, center):
        root = ET.fromstring('<sdf version="1.9"><model name="dynamic_audit_box"><static>true</static><pose>0 0 .1 0 0 0</pose><link name="body"><collision name="body"><geometry><box><size>.08 .08 .2</size></box></geometry></collision><visual name="body"><geometry><box><size>.08 .08 .2</size></box></geometry><material><ambient>1 .05 .05 1</ambient><diffuse>1 .05 .05 1</diffuse></material></visual></link></model></sdf>')
        root.find('.//pose').text = f'{center[0]} {center[1]} .1 0 0 0'
        sdf = ET.tostring(root, encoding='unicode')
        self.service('create', 'EntityFactory', 'sdf: '+json.dumps(sdf))
        self.center = self.anchor = center

    def move(self, center):
        self.service('set_pose', 'Pose', f'name: "dynamic_audit_box", position: {{x: {center[0]}, y: {center[1]}, z: .1}}, orientation: {{w: 1}}')
        self.center = center

    def tick(self):
        now = self.get_clock().now().nanoseconds*1e-9
        if self.pose is None:
            return
        if self.stage == 'waiting' and self.wander.endswith(':forward'):
            x, y, yaw = self.pose
            for distance in (.24, .22, .20):
                candidate = (x+distance*math.cos(yaw), y+distance*math.sin(yaw))
                if float(clearance(np.asarray(candidate), self.identity['walls'])) > .07:
                    self.spawn(candidate)
                    self.stage, self.stage_at = 'stationary', now
                    self.axis = (-math.sin(yaw), math.cos(yaw))
                    self.events.append(dict(stamp=now, event='spawn_stationary', center=candidate))
                    break
        elif self.stage == 'stationary' and now-self.stage_at >= 12.:
            self.stage, self.stage_at = 'moving', now
            self.events.append(dict(stamp=now, event='start_scripted_crossing'))
        elif self.stage == 'moving' and now-self.last_move >= .5:
            displacement = min(.2, (now-self.stage_at)*.08)
            target = tuple(self.anchor[k]+self.axis[k]*displacement for k in (0,1))
            if float(clearance(np.asarray(target), self.identity['walls'])) > .06:
                self.move(target)
            self.last_move = now
            if now-self.stage_at >= 10.:
                self.service('remove', 'Entity', 'name: "dynamic_audit_box", type: MODEL')
                self.center = None
                self.stage, self.stage_at = 'removed', now
                self.events.append(dict(stamp=now, event='removed'))
        if now-self.last_sample >= .2:
            self.last_sample = now
            self.rows.append(dict(stamp=now, stage=self.stage, robot=self.pose,
                box=self.center, observations=self.latest.copy()))
        return self.stage == 'removed' and now-self.stage_at >= 10.


def main():
    assert os.environ.get('ROS_DOMAIN_ID') == '227'
    assert os.environ.get('GZ_PARTITION') == 'pinky_calmap227'
    rclpy.init()
    node = Scenario()
    start = time.monotonic()
    error = None
    try:
        while time.monotonic()-start < 1200:
            rclpy.spin_once(node, timeout_sec=.05)
            if node.tick():
                break
    except Exception as exc:
        error = str(exc)
    finally:
        out = Path('/tmp/pinky-calmap227/obstacle-scenario.json')
        out.write_text(json.dumps(dict(run_id=node.run_id, stage=node.stage, error=error, events=node.events,
            motion='scripted kinematic box', gt_usage='fixture placement and audit only', rows=node.rows), indent=2))
        print(json.dumps(dict(stage=node.stage, error=error, events=node.events)))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
