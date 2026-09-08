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
from move_control.sensing.camera import classify_frame


class RenderedCamera(Node):
    def __init__(self):
        super().__init__('rendered_camera_adapter')
        self.floor, self.last_stamp = None, None
        self.hits = 0
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
        result = classify_frame(bgr, floor_hsv=self.floor)
        self.floor = result['floor_hsv']
        self.hits = self.hits+1 if result['blocked'] else 0
        self.blocked.publish(Bool(data=self.hits >= 2))
        self.cliff.publish(Bool(data=bool(result['cliff'])))
        self.side.publish(Float32(data=result['side']))
        self.image.publish(msg)
        self.evidence.publish(String(data=json.dumps(dict(stamp=stamp, blocked=self.hits >= 2,
            cliff=bool(result['cliff']), side=result['side'], source='gazebo_rendered_pixels'))))
        self.last_stamp = stamp
        self.frames += 1
        if self.frames % 20 == 1:
            cv2.imwrite(str(self.out/f'frame-{self.frames:06}.png'), bgr)
            (self.out/'latest.json').write_text(json.dumps(dict(stamp=stamp, frames=self.frames,
                blocked=self.hits >= 2, cliff=bool(result['cliff']), source='gazebo_rendered_pixels')))


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
