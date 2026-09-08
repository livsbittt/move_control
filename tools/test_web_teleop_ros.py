import unittest
from types import SimpleNamespace
from unittest.mock import Mock
from move_control import web_node as web


class TeleopGateTest(unittest.TestCase):
    def test_discovery_and_configuration_never_select_final_motor_topic(self):
        for configured in ('auto', '/cmd_vel', '/cmd_vel_raw'):
            node = SimpleNamespace(
                get_parameter=Mock(return_value=SimpleNamespace(value=configured)),
                count_publishers=Mock(return_value=0),
                count_subscribers=Mock(return_value=1),
                teleop_target=None, teleop_pub=None,
                create_publisher=Mock(), destroy_publisher=Mock(),
                get_logger=Mock())
            web.WebNode.resolve_teleop(node)
            self.assertEqual(node.teleop_target, '/cmd_vel_raw')
            self.assertEqual(node.create_publisher.call_args.args[1], '/cmd_vel_raw')
            web.WebNode.resolve_teleop(node)
            self.assertEqual(node.create_publisher.call_count, 1)
