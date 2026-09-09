"""Run the production classifier on Gazebo-rendered pixels, keeping source time."""
import json
import os
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, String
from rosy_control.sensing.camera import classify_frame
from rosy_control.sensing.camera_evidence import observation_payload
from rosy_control.sensing.camera_policy import CameraPolicy


class RenderedCamera(Node):
    def __init__(self):
        super().__init__('rendered_camera_adapter')
        self.declare_parameter('region_min_area_fraction', .0005)
        # Same hysteresis the real node runs. This adapter used to keep its own
        # (hits hardcoded to 2, no warmup, no cliff hysteresis), so a green sim
        # run said nothing about camera_detect_node.
        self.declare_parameter('hits', 2)
        self.declare_parameter('warmup_frames', 12)
        # Same thresholds as the robot. Relying on classify_frame's signature
        # defaults would let config/camera.yaml drift away from the sim again,
        # which is the divergence this adapter was unified to remove.
        self.declare_parameter('void_v_ratio', .50)
        self.declare_parameter('obst_frac', .45)
        self.floor, self.last_stamp = None, None
        self.policy = CameraPolicy(hits=int(self.get_parameter('hits').value),
                                   warmup_frames=int(self.get_parameter('warmup_frames').value))
        self.image = self.create_publisher(Image, '/camera/front', 10)
        self.blocked = self.create_publisher(Bool, '/camera/blocked', 10)
        self.cliff = self.create_publisher(Bool, '/camera/cliff', 10)
        self.side = self.create_publisher(Float32, '/camera/side', 10)
        self.evidence = self.create_publisher(String, '/camera/observation', 10)
        self.create_subscription(Image, '/pinky/rendered_camera', self.on_image, qos_profile_sensor_data)
        self.out = Path('/tmp/pinky-calmap227/rendered-camera')
        self.out.mkdir(exist_ok=True)
        self.frames = 0

    def on_image(self, msg):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        now = self.get_clock().now().nanoseconds*1e-9
        if not 0 <= now-stamp <= .3 or (self.last_stamp is not None and stamp <= self.last_stamp):
            return
        if msg.encoding not in ('rgb8', 'bgr8') or msg.step < msg.width*3 or len(msg.data) != msg.step*msg.height:
            return
        bgr = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.step)[:, :msg.width*3].reshape(msg.height, msg.width, 3)
        if msg.encoding == 'rgb8':
            bgr = bgr[:, :, ::-1]
        bgr = np.ascontiguousarray(bgr)
        result = classify_frame(bgr, floor_hsv=self.floor,
            void_v_ratio=float(self.get_parameter('void_v_ratio').value),
            obst_frac=float(self.get_parameter('obst_frac').value),
            region_min_area_fraction=float(self.get_parameter('region_min_area_fraction').value))
        if result['floor_hsv'] is not None:
            self.floor = result['floor_hsv']
        cliff, blocked = self.policy.update(result)
        self.blocked.publish(Bool(data=blocked))
        self.cliff.publish(Bool(data=cliff))
        self.image.publish(msg)
        self.last_stamp = stamp
        self.frames += 1
        if self.frames % 20 == 1:
            cv2.imwrite(str(self.out/f'frame-{self.frames:06}.png'), bgr)
            (self.out/'latest.json').write_text(json.dumps(dict(stamp=stamp, frames=self.frames,
                blocked=blocked, cliff=cliff, ready=self.policy.ready,
                source='gazebo_rendered_pixels')))
        if not self.policy.ready:
            return
        self.side.publish(Float32(data=result['side']))
        self.evidence.publish(String(data=json.dumps(observation_payload(
            stamp, cliff, blocked, result['side'], result, (msg.width, msg.height),
            'gazebo_rendered_pixels'))))


def main():
    assert os.environ.get('ROS_DOMAIN_ID') == '227'
    rclpy.init()
    node = RenderedCamera()
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
