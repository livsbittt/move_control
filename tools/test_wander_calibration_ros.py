import time
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
