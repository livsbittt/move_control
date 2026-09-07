import unittest
import time
from types import SimpleNamespace
from unittest.mock import Mock
from sensor_msgs.msg import BatteryState, Image
from rclpy.time import Time
from move_control import web_node as web
from move_control.sensing.battery import battery_snapshot


class BatteryAdapterTest(unittest.TestCase):
    def setUp(self):
        web.STATE.clear()
        self.node = SimpleNamespace(battery_stamp_ns=None,
            get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=100)))

    def sample(self, seconds):
        msg = BatteryState()
        msg.header.stamp = Time(seconds=seconds).to_msg()
        msg.present, msg.voltage, msg.percentage = True, 7.8, .72
        return msg

    def test_duplicate_source_does_not_extend_battery_lifetime(self):
        web.WebNode.on_battery(self.node, self.sample(99))
        received = web.STATE['battery_received']
        web.WebNode.on_battery(self.node, self.sample(99))
        self.assertEqual(web.STATE['battery_received'], received)
        self.assertEqual(web.STATE['battery_sample']['percent'], 72.)
        self.assertEqual(battery_snapshot(web.STATE['battery_sample'], received, received+6)['reason'], 'stale')

    def test_old_or_future_source_never_looks_live(self):
        web.WebNode.on_battery(self.node, self.sample(90))
        web.WebNode.on_battery(self.node, self.sample(102))
        self.assertNotIn('battery_sample', web.STATE)

    def test_camera_jpeg_generation_reaches_state_poll_contract(self):
        web.CAM_JPG.update(bytes=None, gen=0, t=0.)
        image = Image(height=8, width=8, step=24, encoding='bgr8', data=bytes([60,120,180]*64))
        web.render_cam(image)
        self.assertEqual(web.STATE['cam'], 1)
        self.assertTrue(web.CAM_JPG['bytes'].startswith(b'\xff\xd8'))

    def test_unconfigured_metrics_never_show_old_repository_map_as_live_quality(self):
        self.node.get_parameter = Mock(return_value=SimpleNamespace(value=''))
        self.assertIsNone(web.WebNode.load_metrics(self.node))
