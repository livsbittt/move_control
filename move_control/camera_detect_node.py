#!/usr/bin/env python3
"""Front-camera look-ahead for desk wander.

  /camera/cliff    drop visible ahead — still on the floor, so turn (do not reverse)
  /camera/blocked  obstacle filling the view ahead
  /camera/side     -1 left, +1 right, 0 center/unknown
  /camera/front    real OV5647 RGB image for the LCD (not Gazebo)
"""
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, String

from .camera import classify_frame


class CameraDetectNode(Node):
    def __init__(self):
        super().__init__('camera_detect_node')
        self.declare_parameter('width', 320)
        self.declare_parameter('height', 240)
        self.declare_parameter('fps', 8.0)
        self.declare_parameter('void_frac', 0.12)
        self.declare_parameter('obst_frac', 0.35)
        self.declare_parameter('hits', 2)
        self.declare_parameter('warmup_frames', 12)
        self.declare_parameter('rotate_deg', 180)

        self.cliff_pub = self.create_publisher(Bool, '/camera/cliff', 10)
        self.block_pub = self.create_publisher(Bool, '/camera/blocked', 10)
        self.side_pub = self.create_publisher(Float32, '/camera/side', 10)
        self.dbg_pub = self.create_publisher(String, '/camera/debug', 10)
        self.img_pub = self.create_publisher(Image, '/camera/front', 10)

        self._cam = None
        self._floor_hsv = None
        self._cliff_hits = 0
        self._block_hits = 0
        self._warmup = 0
        self._cliff = False
        self._blocked = False
        self._start_cam()
        period = 1.0 / max(1.0, float(self.get_parameter('fps').value))
        self.create_timer(period, self.tick)
        self.get_logger().info(
            f'camera_detect ready {int(self.get_parameter("width").value)}x'
            f'{int(self.get_parameter("height").value)} @ '
            f'{float(self.get_parameter("fps").value):.0f}Hz '
            f'rotate={int(self.get_parameter("rotate_deg").value)}'
        )

    def _start_cam(self):
        try:
            from picamera2 import Picamera2
        except ImportError:
            self.get_logger().error('picamera2 missing — camera detection off')
            return
        try:
            cam = Picamera2()
            w = int(self.get_parameter('width').value)
            h = int(self.get_parameter('height').value)
            cfg = cam.create_preview_configuration(
                main={'size': (w, h), 'format': 'RGB888'},
                controls={'FrameRate': float(self.get_parameter('fps').value)},
            )
            cam.configure(cfg)
            cam.start()
            time.sleep(0.3)
            self._cam = cam
        except Exception as exc:
            self.get_logger().error(f'camera start failed: {exc}')
            self._cam = None

    def _stop_cam(self):
        cam = self._cam
        self._cam = None
        if cam is None:
            return
        try:
            cam.stop()
        except Exception:
            pass
        try:
            cam.close()
        except Exception:
            pass

    def tick(self):
        if self._cam is None:
            self.cliff_pub.publish(Bool(data=False))
            self.block_pub.publish(Bool(data=False))
            return
        try:
            rgb = self._cam.capture_array('main')
        except Exception as exc:
            self.get_logger().warn(f'capture failed: {exc}', throttle_duration_sec=2.0)
            return
        if rgb is None or rgb.ndim != 3:
            return
        rot = int(self.get_parameter('rotate_deg').value) % 360
        if rot == 180:
            rgb = np.rot90(rgb, 2)
        elif rot == 90:
            rgb = np.rot90(rgb, 1)
        elif rot == 270:
            rgb = np.rot90(rgb, 3)
        self._publish_front(rgb)
        res = classify_frame(
            rgb,
            floor_hsv=self._floor_hsv,
            void_frac=float(self.get_parameter('void_frac').value),
            obst_frac=float(self.get_parameter('obst_frac').value),
        )
        if res['floor_hsv'] is not None:
            self._floor_hsv = res['floor_hsv']

        self._warmup += 1
        if self._warmup < int(self.get_parameter('warmup_frames').value):
            self.cliff_pub.publish(Bool(data=False))
            self.block_pub.publish(Bool(data=False))
            return
        need = int(self.get_parameter('hits').value)
        self._cliff_hits = self._cliff_hits + 1 if res['cliff'] else 0
        self._block_hits = self._block_hits + 1 if res['blocked'] else 0
        if self._cliff_hits >= need:
            self._cliff = True
        elif self._cliff_hits == 0:
            self._cliff = False
        if self._block_hits >= need:
            self._blocked = True
        elif self._block_hits == 0:
            self._blocked = False

        self.cliff_pub.publish(Bool(data=self._cliff))
        self.block_pub.publish(Bool(data=self._blocked))
        self.side_pub.publish(Float32(data=float(res['side'])))
        cols = res['cols']
        mcols = res.get('mid_cols', cols)
        self.dbg_pub.publish(
            String(
                data=(
                    f'cliff={int(self._cliff)} block={int(self._blocked)} '
                    f'side={res["side"]:.0f} mid_obst={res["mid_obst"]:.2f} '
                    f'L o={mcols[0]["obst"]:.2f} C o={mcols[1]["obst"]:.2f} '
                    f'R o={mcols[2]["obst"]:.2f} '
                    f'near L={cols[0]["obst"]:.2f} C={cols[1]["obst"]:.2f} '
                    f'R={cols[2]["obst"]:.2f}'
                )
            )
        )

    def _publish_front(self, rgb):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        msg.height = int(rgb.shape[0])
        msg.width = int(rgb.shape[1])
        msg.encoding = 'rgb8'
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = np.ascontiguousarray(rgb).tobytes()
        self.img_pub.publish(msg)

    def destroy_node(self):
        self._stop_cam()
        super().destroy_node()


def main():
    rclpy.init()
    node = CameraDetectNode()
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
