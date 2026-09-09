import unittest
import ast
import os
from pathlib import Path
from types import SimpleNamespace

from rosy_control.control.startup_profile import FEATURES, startup_profile


class StartupProfileTests(unittest.TestCase):
    def test_full_retains_all_nodes(self):
        profile = startup_profile('full')
        self.assertTrue(all(profile['nodes'].values()))
        self.assertFalse(profile['calibration_sensing_only'])

    def test_sensing_loads_only_stationary_core(self):
        profile = startup_profile('sensing')
        self.assertEqual({name for name, enabled in profile['nodes'].items() if enabled},
                         {'safety', 'camera', 'web', 'calibration'})
        self.assertTrue(profile['calibration_sensing_only'])

    def test_explicit_camera_and_external_imu(self):
        profile = startup_profile('sensing', {'camera': 'true', 'imu': 'false'})
        self.assertTrue(profile['nodes']['camera'])
        self.assertFalse(profile['nodes']['imu'])

    def test_full_accepts_optional_disable(self):
        profile = startup_profile('full', {'lcd': 'false', 'wander': 'false'})
        self.assertFalse(profile['nodes']['lcd'])
        self.assertTrue(profile['nodes']['safety'])

    def test_invalid_inputs_fail_before_launch(self):
        for name, overrides, sensing in [('unknown', {}, 'auto'), ('full', {'imu': 'maybe'}, 'auto'),
                ('full', {'safety': 'false'}, 'auto'), ('sensing', {'calibration': 'false'}, 'auto'),
                ('sensing', {}, 'false')]:
            with self.assertRaises(ValueError):
                startup_profile(name, overrides, sensing)


class StartupLaunchTests(unittest.TestCase):
    def actions(self, profile, overrides=None):
        looked_up = []
        def share(package):
            looked_up.append(package)
            if package != 'rosy_control':
                raise AssertionError('Disabled optional package must not be resolved')
            return '/share/' + package
        context = {'profile': profile, 'calibration_sensing_only': 'auto',
                   **{'start_' + name: 'auto' for name in FEATURES}, **(overrides or {})}
        source = Path(__file__).parents[1] / 'launch/robot.launch.py'
        tree = ast.parse(source.read_text(encoding='utf-8-sig'))
        functions = [n for n in tree.body if isinstance(n, ast.FunctionDef)]
        namespace = dict(os=os, FEATURES=FEATURES, startup_profile=startup_profile,
            get_package_share_directory=share,
            LaunchConfiguration=lambda name: SimpleNamespace(perform=lambda ctx: ctx[name]),
            Node=lambda **kw: dict(kind='node', **kw),
            TimerAction=lambda **kw: dict(kind='timer', **kw),
            LogInfo=lambda **kw: dict(kind='log', **kw))
        exec(compile(ast.Module(body=functions, type_ignores=[]), str(source), 'exec'), namespace)
        return namespace['_processing_actions'](context), looked_up

    def test_sensing_constructs_only_selected_nodes_without_lcd_package(self):
        actions, looked_up = self.actions('sensing')
        nodes = [item for item in actions if item['kind'] == 'node']
        nodes += [node for item in actions if item['kind'] == 'timer' for node in item['actions']]
        self.assertEqual({node['executable'] for node in nodes},
                         {'camera_detect_node', 'safety_node', 'web_node', 'startup_calibration_node'})
        self.assertEqual(set(looked_up), {'rosy_control'})
        calibration = next(node for node in nodes if node['executable'] == 'startup_calibration_node')
        self.assertIs(calibration['parameters'][-1]['calibration_sensing_only'], True)
        self.assertTrue(all(node['respawn'] for node in nodes))
        self.assertEqual([item['period'] for item in actions if item['kind'] == 'timer'], [1.5, 3.5, 4.0])
        self.assertIn('disabled:', actions[0]['msg'])

    def test_full_can_skip_missing_lcd_and_preserves_wander_delay(self):
        actions, _ = self.actions('full', {'start_lcd': 'false'})
        self.assertEqual([item['period'] for item in actions if item['kind'] == 'timer'],
                         [1.5, 3.0, 3.5, 4.0])
