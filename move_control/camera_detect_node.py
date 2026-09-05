#!/usr/bin/env python3
"""Front-camera look-ahead for desk wander.

Classifies the near field as floor / void (drop) / obstacle using an
adaptive HSV floor sample from the lower image.

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


def classify_frame(
    rgb,
    floor_hsv=None,
    void_v_ratio=0.50,
    hue_shift=35.0,
    floor_h_tol=28.0,
    floor_v_tol=55.0,
    near_y0=0.68,
    near_y1=0.95,
    mid_y0=0.22,
    mid_y1=0.62,
    void_frac=0.12,
    obst_frac=0.35,
):
    """Return dict of flags and column scores. rgb is HxWx3 uint8 RGB."""
    import cv2

    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV).astype(np.float32)
    h_ch, s_ch, v_ch = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
    h, w = v_ch.shape

    y_lo0, y_lo1 = int(0.70 * h), int(0.92 * h)
    x_lo0, x_lo1 = int(0.15 * w), int(0.85 * w)
    roi = hsv[y_lo0:y_lo1, x_lo0:x_lo1]
    if roi.size == 0:
        return _empty_result()
    vv = roi[:, :, 2]
    p20, p80 = np.percentile(vv, [20, 80])
    sample = roi[(vv >= p20) & (vv <= p80)]
    if sample.size == 0:
        sample = roi.reshape(-1, 3)
    inst = np.median(sample, axis=0)
    if floor_hsv is None:
        fh, fs, fv = (float(inst[0]), float(inst[1]), float(inst[2]))
        new_floor = (fh, fs, fv)
    else:
        a = 0.12
        fh = (1 - a) * floor_hsv[0] + a * float(inst[0])
        fs = (1 - a) * floor_hsv[1] + a * float(inst[1])
        fv = (1 - a) * floor_hsv[2] + a * float(inst[2])
        new_floor = (fh, fs, fv)

    d_h = np.minimum(np.abs(h_ch - fh), 180.0 - np.abs(h_ch - fh))
    floor = (d_h < floor_h_tol) & (np.abs(v_ch - fv) < floor_v_tol) & (v_ch > fv * 0.55)
    void = (
        (v_ch < fv * void_v_ratio)
        | ((d_h > hue_shift) & (v_ch < fv * 0.90) & (s_ch > 25.0))
    ) & (~floor)
    obst = (~floor) & (~void)

    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    void = cv2.morphologyEx(void.astype(np.uint8), cv2.MORPH_OPEN, k).astype(bool)
    obst = cv2.morphologyEx(obst.astype(np.uint8), cv2.MORPH_OPEN, k).astype(bool)

    ny0, ny1 = int(near_y0 * h), int(near_y1 * h)
    cols = []
    for x0, x1 in ((0, w // 3), (w // 3, 2 * w // 3), (2 * w // 3, w)):
        sl = (slice(ny0, ny1), slice(x0, x1))
        cols.append(
            {
                'void': float(void[sl].mean()) if void[sl].size else 0.0,
                'obst': float(obst[sl].mean()) if obst[sl].size else 0.0,
                'floor': float(floor[sl].mean()) if floor[sl].size else 0.0,
            }
        )
    my0, my1 = int(mid_y0 * h), int(mid_y1 * h)
    mx0, mx1 = int(0.20 * w), int(0.80 * w)
    mid_obst = float(obst[my0:my1, mx0:mx1].mean()) if obst[my0:my1, mx0:mx1].size else 0.0

    on_floor = any(c['floor'] >= 0.20 for c in cols)
    cliff = on_floor and any(c['void'] >= void_frac for c in cols)
    blocked = on_floor and mid_obst >= obst_frac
    scores = [c['void'] + 0.5 * c['obst'] for c in cols]
    imax = int(np.argmax(scores))
    if max(scores) - min(scores) < 0.08:
        side = 0.0
    else:
        side = float((-1, 0, 1)[imax])
    return {
        'cliff': cliff,
        'blocked': blocked,
        'side': side,
        'cols': cols,
        'mid_obst': mid_obst,
        'floor_hsv': new_floor,
    }


def _empty_result():
    return {
        'cliff': False,
        'blocked': False,
        'side': 0.0,
        'cols': [{'void': 0.0, 'obst': 0.0, 'floor': 0.0}] * 3,
        'mid_obst': 0.0,
        'floor_hsv': None,
    }


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
            f'{float(self.get_parameter("fps").value):.0f}Hz'
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
        self.dbg_pub.publish(
            String(
                data=(
                    f'cliff={int(self._cliff)} block={int(self._blocked)} '
                    f'side={res["side"]:.0f} mid_obst={res["mid_obst"]:.2f} '
                    f'L v={cols[0]["void"]:.2f}o={cols[0]["obst"]:.2f} '
                    f'C v={cols[1]["void"]:.2f}o={cols[1]["obst"]:.2f} '
                    f'R v={cols[2]["void"]:.2f}o={cols[2]["obst"]:.2f}'
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
