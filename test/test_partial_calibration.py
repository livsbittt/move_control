"""The node decides the partial-calibration route; the screen only renders the verdict."""
import ast
import unittest
from pathlib import Path
from types import SimpleNamespace

from rosy_control.control.configured_operation import configured_status
from rosy_control.control.sensor_tiers import partial_plan


def node_method(name, **bindings):
    path = Path(__file__).parents[1] / 'rosy_control/startup_calibration_node.py'
    tree = ast.parse(path.read_text(encoding='utf-8'))
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == name)
    namespace = dict(bindings)
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), namespace)
    return namespace[name]


class PartialCalibrationCommandTests(unittest.TestCase):
    def node(self, sensors):
        calls = []
        return SimpleNamespace(
            calls=calls, message='',
            stationary_report=lambda now: sensors,
            get_parameter=lambda name: SimpleNamespace(value=False),
            reset=lambda **kwargs: calls.append(kwargs),
            publish=lambda: None,
            after_relocation='stay', calibration_scope='full')

    def run_command(self, node, command):
        node_method('on_command', partial_plan=partial_plan,
                    time=SimpleNamespace(monotonic=lambda: 0.))(node, SimpleNamespace(data=command))

    def healthy(self):
        return {name: {'eligible': True} for name in
                ('lidar', 'odom', 'ir', 'imu', 'us', 'camera', 'tf', 'map', 'map_tf')}

    def test_imu_failure_routes_to_limited_sensor_operation(self):
        sensors = self.healthy()
        sensors['imu']['eligible'] = False
        node = self.node(sensors)
        self.run_command(node, 'partial_calibration')
        self.assertEqual(node.calls, [{'existing_settings': True, 'limited_sensors': True,
                                       'excluded_sensors': ('imu',)}])

    def test_camera_failure_continues_a_real_calibration(self):
        sensors = self.healthy()
        sensors['camera']['eligible'] = False
        node = self.node(sensors)
        self.run_command(node, 'partial_calibration')
        self.assertEqual(node.calls, [{'excluded_sensors': ('camera',)}])

    def test_required_sensor_failure_resets_nothing_and_names_the_sensor(self):
        sensors = self.healthy()
        sensors['lidar']['eligible'] = False
        node = self.node(sensors)
        self.run_command(node, 'partial_calibration')
        self.assertEqual(node.calls, [])
        self.assertIn('lidar', node.message)

    def test_no_failure_resets_nothing(self):
        node = self.node(self.healthy())
        self.run_command(node, 'partial_calibration')
        self.assertEqual(node.calls, [])
        self.assertIn('full calibration', node.message)

    def test_retry_carries_position_and_scope(self):
        node = self.node(self.healthy())
        self.run_command(node, 'retry:return_origin:skip_motion')
        self.assertEqual(node.after_relocation, 'return_origin')
        self.assertEqual(node.calibration_scope, 'skip_motion')

    def test_legacy_two_part_retry_still_means_full_scope(self):
        node = self.node(self.healthy())
        self.run_command(node, 'retry:stay')
        self.assertEqual(node.after_relocation, 'stay')
        self.assertEqual(node.calibration_scope, 'full')

    def test_skipped_motion_scope_routes_to_existing_settings(self):
        node = self.node(self.healthy())
        self.run_command(node, 'retry:stay:skip_motion')
        self.assertEqual(node.calls, [{'invalidate_certificate': True, 'existing_settings': True}])

    def test_full_scope_runs_a_real_calibration(self):
        node = self.node(self.healthy())
        self.run_command(node, 'retry:stay:full')
        self.assertEqual(node.calls, [{'invalidate_certificate': True}])

    def test_partial_calibration_never_claims_a_verified_calibration(self):
        status = configured_status(True, [], limited_sensors=True, excluded_sensors=('imu',))
        self.assertFalse(status['calibration_verified'])
        self.assertFalse(status['rotation_verified'])
        self.assertTrue(status['calibration_skipped'])


if __name__ == '__main__':
    unittest.main()
