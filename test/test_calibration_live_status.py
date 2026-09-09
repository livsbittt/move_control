from types import SimpleNamespace
import unittest
from test.test_sensing_only import node_method


class LiveStatusTest(unittest.TestCase):
    def test_failed_trial_reports_recovered_sensor_without_claiming_ready(self):
        current = {'lidar': {'ok': True, 'status': 'ok'}}
        calls = []
        node = SimpleNamespace(phase='failed', sensing_only=False, existing_settings=False,
            sensors={'lidar': {'ok': False, 'status': 'invalid'}}, runtime_ready=False,
            read_tf=lambda: None, runtime_health=lambda now: current,
            last_report=0., publish=lambda: calls.append('publish'))
        node_method('tick', time=SimpleNamespace(monotonic=lambda: 10.))(node)
        self.assertEqual(node.sensors, current)
        self.assertEqual(node.phase, 'failed')
        self.assertFalse(node.runtime_ready)
        self.assertEqual(calls, ['publish'])
