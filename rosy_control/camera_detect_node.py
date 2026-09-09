#!/usr/bin/env python3
"""Front-camera look-ahead for desk wander.

  /camera/cliff       always False. A single camera cannot tell a dark wall from
                      a dark hole at the same ground line, so no drop verdict is
                      drawn here; the floor IR owns that decision. The darkness
                      is still published as evidence on /camera/observation.
  /camera/blocked     obstacle filling the view ahead
  /camera/side        -1 left, +1 right, 0 center/unknown
  /camera/front       OV5647 BGR8 (libcamera RGB888 is BGR in memory)
  /camera/observation compact per-frame evidence; schema in sensing/camera_evidence
  /camera/controls    the exposure/gain/white-balance the sensor was frozen at
  /camera/debug       one-line human-readable column scores
"""
import time
import json

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, String

from .sensing.camera import classify_frame
from .sensing.camera_controls import (
    lock_action, lock_controls, lock_summary, static_controls)
from .sensing.camera_evidence import observation_payload
from .sensing.camera_ground import ground_plane
from .sensing.camera_policy import CameraPolicy


def _rotate(img, deg):
    k = (int(deg) // 90) % 4
    if k:
        img = np.rot90(img, k)
    return np.ascontiguousarray(img)


class CameraDetectNode(Node):
    def __init__(self):
        super().__init__('camera_detect_node')
        self.declare_parameter('width', 320)
        self.declare_parameter('height', 240)
        self.declare_parameter('fps', 8.0)
        # A pixel is dark below this fraction of the floor's own brightness.
        self.declare_parameter('void_v_ratio', 0.50)
        self.declare_parameter('obst_frac', 0.45)
        self.declare_parameter('region_min_area_fraction', .0005)
        self.declare_parameter('hits', 2)
        self.declare_parameter('warmup_frames', 12)
        self.declare_parameter('rotate_deg', 180)
        # The ISP's AWB and AGC are adaptive loops on the same signal the floor
        # reference tracks, so they must stop moving before the reference means
        # anything. Settle with them on, then freeze what they chose.
        self.declare_parameter('camera_lock_enabled', True)
        self.declare_parameter('camera_settle_seconds', 2.5)
        # Ground-plane calibration. No invented defaults: these zeros are refused
        # by ground_plane(), so an uncalibrated robot keeps reporting regions as
        # unranged instead of publishing distances nobody measured. See
        # docs/camera-ground-calibration.md for the 20-minute procedure.
        self.declare_parameter('camera_height_m', 0.0)
        self.declare_parameter('camera_pitch_rad', 0.0)
        self.declare_parameter('camera_focal_px', 0.0)
        self.declare_parameter('camera_principal_x', 0.0)
        self.declare_parameter('camera_principal_y', 0.0)
        self.declare_parameter('camera_max_range_m', 0.0)

        self.cliff_pub = self.create_publisher(Bool, '/camera/cliff', 10)
        self.block_pub = self.create_publisher(Bool, '/camera/blocked', 10)
        self.side_pub = self.create_publisher(Float32, '/camera/side', 10)
        self.dbg_pub = self.create_publisher(String, '/camera/debug', 10)
        self.img_pub = self.create_publisher(Image, '/camera/front', 10)
        self.observation_pub = self.create_publisher(String, '/camera/observation', 10)
        self.controls_pub = self.create_publisher(String, '/camera/controls', 10)

        self._settle_deadline = None
        self._locked = None
        self._lock_attempts = 0
        self._cam = None
        self._floor_hsv = None
        self._ground = ground_plane(
            height_m=float(self.get_parameter('camera_height_m').value),
            pitch_rad=float(self.get_parameter('camera_pitch_rad').value),
            focal_px=float(self.get_parameter('camera_focal_px').value),
            principal_x=float(self.get_parameter('camera_principal_x').value),
            principal_y=float(self.get_parameter('camera_principal_y').value),
            max_range_m=float(self.get_parameter('camera_max_range_m').value))
        # Hysteresis and the warmup gate belong to the policy, not to this node:
        # the Gazebo adapter needs the same ones, and when each kept its own they
        # diverged silently.
        self._policy = CameraPolicy(
            hits=int(self.get_parameter('hits').value),
            warmup_frames=int(self.get_parameter('warmup_frames').value))
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
            # configure() discards controls set before it, so these go after.
            static = static_controls(float(self.get_parameter('fps').value))
            if static:
                cam.set_controls(static)
            cam.start()
            time.sleep(0.3)
            self._cam = cam
            self._settle_deadline = time.monotonic() + max(
                0.0, float(self.get_parameter('camera_settle_seconds').value))
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

    def _maybe_lock(self):
        """Freeze AE/AWB once they have settled.

        The floor reference cannot mean anything while the ISP is still
        re-deciding gains underneath it: measured, a 1.3x gain step on one
        unchanged frame flips blocked, and a white-balance shift flips the whole
        near band. Bounded retries so one bad metadata read at the settle instant
        does not disable the lock for the session; after that, stay in auto and
        say so rather than freezing on a value we do not trust.
        """
        if self._locked is not None:
            return
        action = lock_action(bool(self.get_parameter('camera_lock_enabled').value),
                             time.monotonic(), self._settle_deadline, self._lock_attempts)
        if action == 'wait':
            return
        if action == 'disabled':
            self._announce_lock('auto (lock disabled by parameter)', froze=False)
            return
        if action == 'exhausted':
            self._announce_lock('auto (lock unavailable after '
                                f'{self._lock_attempts} attempts)', froze=False)
            return
        settle = max(0.0, float(self.get_parameter('camera_settle_seconds').value))
        try:
            controls = lock_controls(self._cam.capture_metadata())
        except Exception as exc:
            self.get_logger().warn(f'camera metadata unreadable: {exc}')
            controls = None
        if controls:
            try:
                self._cam.set_controls(controls)
            except Exception as exc:
                self.get_logger().warn(f'camera lock rejected: {exc}')
                controls = None
        if controls is None:
            self._lock_attempts += 1
            self._settle_deadline = time.monotonic() + max(0.5, settle)
            return
        self._announce_lock(lock_summary(controls))

    def _announce_lock(self, summary, froze=True):
        self._locked = summary
        if froze:
            # The reference must bootstrap on frozen gains, not on the auto
            # frames that preceded them, or it starts life describing a scene
            # that has since changed brightness and colour underneath it. When
            # nothing was frozen there is nothing to re-bootstrap against, so
            # the reset would only cost another warmup of forced blocked=True.
            self._floor_hsv = None
            self._policy.reset()
        self.controls_pub.publish(String(data=summary))
        self.get_logger().info(f'camera controls {summary}')

    def tick(self):
        if self._cam is None:
            self.cliff_pub.publish(Bool(data=self._policy.cliff))
            self.block_pub.publish(Bool(data=True))
            return
        self._maybe_lock()
        capture_stamp = self.get_clock().now().nanoseconds*1e-9
        try:
            bgr = self._cam.capture_array('main')
        except Exception as exc:
            self.get_logger().warn(f'capture failed: {exc}', throttle_duration_sec=2.0)
            self.block_pub.publish(Bool(data=True))
            return
        if bgr is None or bgr.ndim != 3:
            self.block_pub.publish(Bool(data=True))
            return
        bgr = _rotate(bgr, self.get_parameter('rotate_deg').value)
        self._publish_front(bgr)
        res = classify_frame(
            bgr,
            floor_hsv=self._floor_hsv,
            void_v_ratio=float(self.get_parameter('void_v_ratio').value),
            obst_frac=float(self.get_parameter('obst_frac').value),
            region_min_area_fraction=float(self.get_parameter('region_min_area_fraction').value),
            ground=self._ground,
        )
        if res['floor_hsv'] is not None:
            self._floor_hsv = res['floor_hsv']
        cliff, blocked = self._policy.update(res)
        self.cliff_pub.publish(Bool(data=cliff))
        self.block_pub.publish(Bool(data=blocked))
        if not self._policy.ready:
            return

        self.side_pub.publish(Float32(data=float(res['side'])))
        self.observation_pub.publish(String(data=json.dumps(observation_payload(
            capture_stamp, cliff, blocked, res['side'], res,
            (bgr.shape[1], bgr.shape[0]), 'onboard_camera_pixels'))))
        cols = res['cols']
        mcols = res.get('mid_cols', cols)
        self.dbg_pub.publish(
            String(
                data=(
                    f'cliff={int(cliff)} block={int(blocked)} '
                    f'side={res["side"]:.0f} mid_obst={res["mid_obst"]:.2f} '
                    f'L o={mcols[0]["obst"]:.2f} C o={mcols[1]["obst"]:.2f} '
                    f'R o={mcols[2]["obst"]:.2f} '
                    f'near L={cols[0]["obst"]:.2f} C={cols[1]["obst"]:.2f} '
                    f'R={cols[2]["obst"]:.2f}'
                )
            )
        )

    def _publish_front(self, bgr):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        msg.height = int(bgr.shape[0])
        msg.width = int(bgr.shape[1])
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = np.ascontiguousarray(bgr).tobytes()
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
