"""Calibration may update measured fields only; existing configuration survives."""
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock

import rclpy
import yaml
from rclpy.parameter import Parameter
from rosy_control.calib_node import CalibNode


class CalibrationPreserveTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(domain_id=226)

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_compute_preserves_unmeasured_settings_on_disk_and_in_runtime(self):
        with tempfile.TemporaryDirectory() as directory:
            node = CalibNode()
            try:
                sign = Path(directory)/'auto.yaml'
                cliff = Path(directory)/'cliff.yaml'
                existing = {'stop_distance': .115, 'clear_distance': .13,
                            'robot_radius': .076, 'lidar_yaw_offset': 3.14,
                            'camera_block_as_wall': False, 'custom_setting': 42}
                sign.write_text(yaml.safe_dump({'safety_node': {'ros__parameters': existing}}))
                cliff.write_text(yaml.safe_dump({'safety_node': {'ros__parameters': {'cliff_hits': 5}}}))
                node.set_parameters([Parameter('sign_path', value=str(sign)),
                                     Parameter('save_path', value=str(cliff))])
                node.sets = {'floor': [(2000, 2100, 2200)]*20,
                             'cliff': [(200, 210, 220)]*20}
                node.linear_sign, node.lidar_yaw, node.imu_buf = -1., None, []
                node._apply_safety = Mock()
                node.compute()
                stored = yaml.safe_load(sign.read_text())['safety_node']['ros__parameters']
                for key, value in existing.items():
                    self.assertEqual(stored[key], value, key)
                self.assertEqual(stored['cmd_linear_sign'], -1.)
                applied = dict(node._apply_safety.call_args.args[0])
                self.assertFalse(set(existing) & set(applied))
                self.assertEqual(yaml.safe_load(cliff.read_text())['safety_node']['ros__parameters']['cliff_hits'], 5)
            finally:
                node.destroy_node()

    def test_malformed_existing_file_is_preserved(self):
        with tempfile.TemporaryDirectory() as directory:
            node = CalibNode()
            try:
                path = Path(directory)/'auto.yaml'
                path.write_text('broken: [')
                self.assertFalse(node._write(str(path), 'safety_node:\n  ros__parameters:\n    cmd_linear_sign: -1.0\n'))
                self.assertEqual(path.read_text(), 'broken: [')
            finally:
                node.destroy_node()
