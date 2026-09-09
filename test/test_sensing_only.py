"""IMU exclusion must remain diagnostic even when every other sensor passes."""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest

from rosy_control.control.calibration import SENSORS
from rosy_control.control.sensing_only import partial_sensing_report


class PartialSensingTests(unittest.TestCase):
    def healthy(self):
        return {name: {'ok': True} for name in SENSORS}

    def test_healthy_partial_baseline_never_grants_motion(self):
        sensors = self.healthy()
        result = partial_sensing_report(sensors)
        self.assertTrue(result['partial_baseline_ready'])
        for name in ('ready', 'calibration_verified', 'motion_allowed', 'rotation_verified', 'settings_applied'):
            self.assertFalse(result[name])
        self.assertFalse(result['sensors']['imu']['eligible'])
        self.assertTrue(sensors['imu']['ok'])  # Reporting cannot weaken the normal baseline.

    def test_missing_imu_is_the_only_exclusion(self):
        sensors = self.healthy()
        del sensors['imu']
        self.assertTrue(partial_sensing_report(sensors)['partial_baseline_ready'])
        for name in SENSORS:
            if name != 'imu':
                missing = dict(sensors)
                del missing[name]
                self.assertFalse(partial_sensing_report(missing)['partial_baseline_ready'], name)

    def test_stale_or_unstable_required_sensor_blocks_partial_baseline(self):
        for name in SENSORS:
            if name != 'imu':
                sensors = self.healthy()
                sensors[name] = {'ok': False, 'eligible': False, 'status': 'stale'}
                self.assertFalse(partial_sensing_report(sensors)['partial_baseline_ready'], name)

    def test_existing_advisory_sensor_policy_is_preserved(self):
        sensors = self.healthy()
        sensors['camera'] = {'ok': False, 'eligible': True}
        self.assertTrue(partial_sensing_report(sensors)['partial_baseline_ready'])


def node_method(name, **bindings):
    """Execute the actual thin adapter method without requiring a ROS installation."""
    path = Path(__file__).parents[1] / 'rosy_control/startup_calibration_node.py'
    tree = ast.parse(path.read_text(encoding='utf-8'))
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == name)
    namespace = dict(bindings)
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), namespace)
    return namespace[name]


class SensingAdapterTests(unittest.TestCase):
    def test_entry_and_retry_use_distinct_reset_modes(self):
        calls = []
        node = SimpleNamespace(reset=lambda **kw: calls.append(kw))
        command = node_method('on_command')
        command(node, SimpleNamespace(data='sensing_only'))
        command(node, SimpleNamespace(data='retry'))
        self.assertEqual(calls, [{'sensing_only': True},
                                 {'invalidate_certificate': True}])

    def test_motion_command_is_rejected_before_motion_safety_or_baseline_work(self):
        calls = []
        node = SimpleNamespace(sensing_only=True, zero=lambda: calls.append('zero'),
                               publish=lambda: calls.append('publish'))
        node_method('on_command')(node, SimpleNamespace(data='validate_motion'))
        self.assertEqual(calls, ['zero', 'publish'])
        self.assertIn('Motion prohibited', node.message)

    def test_tick_stops_and_never_dispatches_motion_even_with_motion_phase(self):
        calls = []
        sensors = {name: {'ok': True} for name in SENSORS}
        node = SimpleNamespace(sensing_only=True, phase='validating_rotation',
            read_tf=lambda: None, zero=lambda: calls.append('zero'), last_report=0.,
            stationary_report=lambda now: sensors,
            baseline=SimpleNamespace(statistics=lambda now: {'imu': {}, 'lidar': {}}),
            wander_pub=SimpleNamespace(publish=lambda msg: calls.append(msg.data)),
            publish=lambda: calls.append('publish'))
        node_method('tick', time=SimpleNamespace(monotonic=lambda: 10.),
                    partial_sensing_report=partial_sensing_report, String=SimpleNamespace)(node)
        self.assertEqual(calls, ['zero', 'stop', 'publish'])
        self.assertFalse(node.runtime_ready)
        self.assertNotIn('imu', node.baseline_values)

    def test_success_callback_cannot_write_certificate_in_sensing_mode(self):
        calls = []
        node = SimpleNamespace(sensing_only=True, zero=lambda: calls.append('zero'),
            round_trip=SimpleNamespace(done=True), persist=lambda: calls.append('persist'),
            publish=lambda: calls.append('publish'))
        node_method('finish')(node, True, 'Unexpected success callback')
        self.assertEqual(node.phase, 'failed')
        self.assertFalse(node.runtime_ready)
        self.assertEqual(calls, ['zero', 'persist', 'publish'])


if __name__ == '__main__':
    unittest.main()
