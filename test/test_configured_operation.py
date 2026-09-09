import unittest
import ast
from pathlib import Path
from types import SimpleNamespace

from rosy_control.control.configured_operation import configured_waiting_reasons, configured_status
from rosy_control.control.calibration_profile import make_profile


def node_method(name, **bindings):
    path = Path(__file__).parents[1] / 'rosy_control/startup_calibration_node.py'
    tree = ast.parse(path.read_text(encoding='utf-8'))
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == name)
    namespace = dict(bindings)
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), namespace)
    return namespace[name]


class ConfiguredOperationTests(unittest.TestCase):
    def test_invalid_front_obstacle_status_is_not_a_directional_permission(self):
        hazards = {name: (10., False) for name in ('blocked', 'cliff', 'tilt', 'pickup')}
        for invalid in (None, 0, 'false'):
            hazards['blocked'] = (10., invalid)
            self.assertIn('safety_blocked', self.reasons(hazards=hazards))

    def health(self):
        return {name: {'eligible': True} for name in ('lidar', 'odom', 'ir', 'imu', 'us', 'tf')}

    def reasons(self, sensors=None, **kwargs):
        args = dict(sensors=self.health() if sensors is None else sensors,
                    localization_required=False, geometry_fresh=True, estop=False,
                    hazards={name: (10., False) for name in ('blocked', 'cliff', 'tilt', 'pickup')}, now=10.)
        args.update(kwargs)
        return configured_waiting_reasons(**args)

    def test_live_manual_sensors_do_not_require_map(self):
        self.assertEqual(self.reasons(), [])

    def test_every_required_sensor_must_be_fresh_and_valid(self):
        for name in self.health():
            health = self.health()
            health[name]['eligible'] = False
            self.assertIn(name, self.reasons(health))

    def test_localization_profile_requires_map_and_map_tf(self):
        self.assertEqual(self.reasons(localization_required=True), ['map', 'map_tf'])

    def test_limited_mode_excludes_only_imu_from_live_requirements(self):
        sensors = self.health()
        sensors['imu']['eligible'] = False
        self.assertEqual(self.reasons(sensors, exclude_imu=True), [])
        for name in ('lidar', 'odom', 'ir', 'us', 'tf'):
            missing = {key: dict(value) for key, value in sensors.items()}
            missing[name]['eligible'] = False
            self.assertIn(name, self.reasons(missing, exclude_imu=True))

    def test_limited_mode_completes_stage_without_claiming_calibration(self):
        result = configured_status(False, ['estop'], limited_sensors=True)
        self.assertTrue(result['calibration_complete'])
        self.assertEqual(result['completion_source'], 'operator_override')
        self.assertFalse(result['calibration_verified'])
        self.assertFalse(result['ready'])
        self.assertEqual(result['excluded_sensors'], ['imu'])
        self.assertEqual(result['speed_limits'], {'linear_mps': .005, 'angular_rad_s': .05})

    def test_estop_unknown_and_active_geometry_and_hazards_block(self):
        for estop in (None, True):
            self.assertIn('estop', self.reasons(estop=estop))
        self.assertIn('safety_geometry', self.reasons(geometry_fresh=False))
        for name in ('blocked', 'cliff', 'tilt', 'pickup'):
            self.assertIn('safety_' + name, self.reasons(hazards={}))
            self.assertIn('safety_' + name, self.reasons(hazards={name: (9., False)}))
            if name != 'blocked':
                self.assertIn('safety_' + name, self.reasons(hazards={name: (10., True)}))

    def test_fresh_front_obstacle_keeps_both_modes_available_for_directional_escape(self):
        hazards = {name: (10., False) for name in ('blocked', 'cliff', 'tilt', 'pickup')}
        hazards['blocked'] = (10., True)
        for limited in (False, True):
            self.assertEqual(self.reasons(hazards=hazards, exclude_imu=limited), [])
            hazards['blocked'] = (9., True)
            self.assertIn('safety_blocked', self.reasons(hazards=hazards, exclude_imu=limited))
            hazards['blocked'] = (10., True)

    def test_operating_permission_never_becomes_calibration_evidence(self):
        for ready in (False, True):
            report = configured_status(ready, [])
            self.assertEqual(report['operating_ready'], ready)
            self.assertEqual(report['ready'], ready)
            self.assertFalse(report['calibration_verified'])
            self.assertFalse(report['rotation_verified'])
            self.assertTrue(report['calibration_skipped'])


class ConfiguredAdapterTests(unittest.TestCase):
    def test_explicit_operating_mode_preserves_saved_certificate(self):
        calls = []
        node = SimpleNamespace(reset=lambda **kw: calls.append(kw))
        node_method('on_command')(node, SimpleNamespace(data='use_existing_settings'))
        self.assertEqual(calls, [{'existing_settings': True}])

    def test_limited_selection_and_retry_keep_modes_explicit(self):
        calls = []
        node = SimpleNamespace(reset=lambda **kw: calls.append(kw))
        command = node_method('on_command')
        command(node, SimpleNamespace(data='use_limited_sensors'))
        command(node, SimpleNamespace(data='retry:stay'))
        self.assertEqual(calls, [{'existing_settings': True, 'limited_sensors': True},
                                {'invalidate_certificate': True}])

    def test_configured_success_callback_cannot_create_certificate(self):
        calls = []
        node = SimpleNamespace(sensing_only=False, existing_settings=True,
            zero=lambda: calls.append('zero'), round_trip=SimpleNamespace(done=True),
            persist=lambda: calls.append('persist'), publish=lambda: calls.append('publish'))
        node_method('finish')(node, True, 'Unexpected callback')
        self.assertFalse(node.runtime_ready)
        self.assertEqual(node.phase, 'failed')
        self.assertEqual(calls, ['zero', 'persist', 'publish'])

    def tick_node(self, missing_imu=False, estop=False, limited_sensors=False, front_blocked=False):
        calls = []
        sensors = {name: {'eligible': True} for name in ('lidar', 'odom', 'ir', 'imu', 'us', 'tf')}
        sensors['imu']['eligible'] = not missing_imu
        node = SimpleNamespace(existing_settings=True, limited_sensors=limited_sensors,
            phase='limited_sensors' if limited_sensors else 'existing_settings', runtime_ready=True,
            runtime_healthy_since=None, read_tf=lambda: None, runtime_health=lambda now: sensors,
            get_parameter=lambda name: SimpleNamespace(value=False), geometry_fresh=lambda now: True,
            estop=estop, hazards={'/safety/' + key: (10., False) for key in ('blocked', 'cliff', 'tilt', 'pickup')},
            zero=lambda: calls.append('zero'), wander_pub=SimpleNamespace(publish=lambda msg: calls.append(msg.data)),
            last_report=0., publish=lambda: calls.append('publish'))
        node.hazards['/safety/blocked'] = (10., front_blocked)
        node_method('tick', time=SimpleNamespace(monotonic=lambda: 10.),
                    configured_waiting_reasons=configured_waiting_reasons, String=SimpleNamespace)(node)
        return node, calls

    def test_missing_imu_immediately_revokes_runtime_permission(self):
        node, calls = self.tick_node(missing_imu=True)
        self.assertFalse(node.runtime_ready)
        self.assertIn('imu', node.configured_waiting)
        self.assertEqual(calls, ['zero', 'stop', 'publish'])

    def test_estop_blocks_without_release_command(self):
        node, calls = self.tick_node(estop=True)
        self.assertFalse(node.runtime_ready)
        self.assertIn('estop', node.configured_waiting)
        self.assertEqual(calls, ['zero', 'stop', 'publish'])

    def test_healthy_mode_does_not_send_motion_or_repeated_stop(self):
        node, calls = self.tick_node()
        self.assertTrue(node.runtime_ready)
        self.assertEqual(calls, ['publish'])

    def test_front_obstacle_does_not_stop_escape_in_either_operating_mode(self):
        for limited in (False, True):
            node, calls = self.tick_node(limited_sensors=limited, front_blocked=True)
            self.assertTrue(node.runtime_ready)
            self.assertEqual(calls, ['publish'])

    def test_actual_safety_directional_gate_still_blocks_forward_and_checks_rear(self):
        source = Path(__file__).parents[1] / 'rosy_control/safety/node.py'
        tree = ast.parse(source.read_text(encoding='utf-8'))
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
        tick = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'tick')
        start = next(i for i, stmt in enumerate(tick.body) if isinstance(stmt, ast.Assign)
                     and any(isinstance(target, ast.Name) and target.id == 'halt_fwd' for target in stmt.targets))
        # Execute the real forward/reverse gate statements without ROS hardware.
        code = compile(ast.Module(body=tick.body[start:start+3], type_ignores=[]), str(source), 'exec')
        for velocity, rear_blocked, expected in ((.005, False, 0.), (-.005, False, -.005), (-.005, True, 0.)):
            cmd = SimpleNamespace(linear=SimpleNamespace(x=velocity))
            scope = dict(cmd=cmd, self=SimpleNamespace(cliff=False, rear_blocked=rear_blocked), obstacle=True, tilt=False)
            exec(code, scope)
            self.assertEqual(cmd.linear.x, expected)

    def test_explicit_limited_mode_accepts_missing_imu_without_motion_command(self):
        node, calls = self.tick_node(missing_imu=True, limited_sensors=True)
        self.assertTrue(node.runtime_ready)
        self.assertTrue(node.sensors['imu']['excluded'])
        self.assertFalse(node.sensors['imu']['required_for_operation'])
        self.assertEqual(calls, ['publish'])

    def test_existing_parameters_lease_has_identity_gains_and_no_rotation_certificate(self):
        path = Path(__file__).parents[1] / 'rosy_control/calibration_atomic.py'
        cls = next(n for n in ast.parse(path.read_text(encoding='utf-8')).body if isinstance(n, ast.ClassDef))
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'profile_packet')
        namespace = {'make_profile': make_profile, 'time': SimpleNamespace(monotonic=lambda: 10.)}
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), namespace)
        node = SimpleNamespace(phase='existing_settings', runtime_ready=True, geometry_fresh=lambda now: True,
            round_trip=None, profile_session='session', profile_sequence=0, trial_geometry_revision=None,
            geometry_revision='geometry', rotation_report=lambda: None,
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_000_000_000)))
        packet = namespace['profile_packet'](node)
        self.assertTrue(packet['enabled'])
        self.assertEqual(packet['linear_gains'], [1., 1.])
        self.assertIsNone(packet['rotation'])
        self.assertFalse(packet['physical_commissioned'])
        self.assertFalse(packet['rotation_trial'])
        self.assertFalse(packet['translation_trial'])
        node.phase = 'limited_sensors'
        node.limited_sensors = True
        packet = namespace['profile_packet'](node)
        self.assertTrue(packet['enabled'])
        self.assertEqual(packet['sensor_exclusions'], ['imu'])
        self.assertEqual(packet['max_linear_mps'], .005)
        self.assertEqual(packet['max_angular_rad_s'], .05)
        self.assertIsNone(packet['rotation'])
