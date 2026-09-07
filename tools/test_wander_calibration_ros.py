import time
import json
import unittest
from unittest.mock import Mock

import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, String
from move_control.wander.node import WanderNode


class CalibrationGateTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(domain_id=222)

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = WanderNode()
        self.node.pub = Mock()
        self.node.set_parameters([Parameter('calibration_required', value=True)])

    def tearDown(self):
        self.node.destroy_node()

    def test_start_and_map_modes_cannot_bypass_initial_calibration(self):
        for command in ('start', 'explore', 'coverage'):
            self.node.on_cmd(String(data=command))
            self.assertFalse(self.node.enabled)
        self.node.on_enable(Bool(data=True))
        self.assertFalse(self.node.enabled)

    def test_look_counts_one_snapshot_once_and_uses_its_ranges(self):
        packet = {'schema_version': 1, 'session': 'gate-a', 'frame': 'base_link',
                  'range_origin': 'lidar', 'issued_s': self.node.get_clock().now().nanoseconds*1e-9,
                  'streams': {'lidar': {'valid': True, 'generation': 1, 'age_s': 0.}},
                  'ranges': {'front': .3, 'rear': .4, 'left': .5, 'right': .6}}
        self.node.on_observation(String(data=json.dumps(packet)))
        before = self.node._look_n
        self.node.front_range = 9.
        for _ in range(10):
            self.node._look_accum()
        self.assertEqual(self.node._look_n, before+1)
        self.assertEqual(self.node._samp_F[-1], .3)

    def test_lost_ready_heartbeat_stops_active_autonomy(self):
        self.node.on_calibration(Bool(data=True))
        self.node.on_cmd(String(data='start'))
        self.assertTrue(self.node.enabled)
        self.node.calibration_received = time.monotonic() - 4
        self.node.tick()
        self.assertFalse(self.node.enabled)
        self.assertEqual(self.node.stop_reason, 'calibration_required')

    def test_ready_does_not_start_a_mode_until_a_command_arrives(self):
        self.node.on_calibration(Bool(data=True))
        self.node.tick()
        self.assertFalse(self.node.enabled)
        self.assertIsNone(self.node.navigation_mode)
        self.node.on_cmd(String(data='coverage'))
        self.assertTrue(self.node.enabled)
        self.assertEqual(self.node.navigation_mode, 'coverage')
        self.node.on_cmd(String(data='stop'))
        self.assertFalse(self.node.enabled)
