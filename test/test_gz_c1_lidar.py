"""GPU lidar in the isolated rig uses the C1 rear-zero mount."""
import math
import unittest
import xml.etree.ElementTree as ET

from rosy_control.sensing.lidar_mount import nose_from_quaternion
from tools.gz.c1_lidar import (
    C1_SCAN_YAW,
    EVIDENCE_SCOPE,
    align_gpu_lidar,
    scan_tf_quaternion,
)


class C1LidarAlignmentTest(unittest.TestCase):
    def test_aligned_gpu_lidar_has_scan_zero_at_the_rear(self):
        root = ET.fromstring(
            "<model><link><sensor name='lidar' type='gpu_lidar'>"
            "<pose>0 0 0.10 0 0 0</pose>"
            "<lidar><scan><horizontal>"
            "<samples>180</samples><min_angle>-3.14</min_angle>"
            "<max_angle>3.14</max_angle></horizontal></scan>"
            "<range><min>0.02</min><max>3.0</max></range></lidar>"
            "</sensor></link></model>")
        sensor = align_gpu_lidar(root.find('.//sensor'))
        pose = [float(v) for v in sensor.find('pose').text.split()]
        self.assertAlmostEqual(pose[5], C1_SCAN_YAW)
        self.assertEqual(sensor.find('.//samples').text, '720')
        self.assertAlmostEqual(float(sensor.find('.//range/min').text), 0.05)
        self.assertAlmostEqual(float(sensor.find('.//range/max').text), 40.0)
        x, y, z, w = scan_tf_quaternion()
        self.assertAlmostEqual(nose_from_quaternion(x, y, z, w), math.pi)
        self.assertEqual(EVIDENCE_SCOPE['lidar'], 'gpu_lidar')
        self.assertEqual(EVIDENCE_SCOPE['ir'], 'synthetic')
