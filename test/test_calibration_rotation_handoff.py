import ast
from pathlib import Path
from types import SimpleNamespace
import unittest

from rosy_control.control.calibration_rotation_handoff import rotation_handoff_ready


def method(name):
    path = Path(__file__).parents[1] / 'rosy_control/calibration_rotation.py'
    cls = next(n for n in ast.parse(path.read_text(encoding='utf-8')).body if isinstance(n, ast.ClassDef))
    function = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == name)
    namespace = {'time': SimpleNamespace(monotonic=lambda: 10.), 'rotation_handoff_ready': rotation_handoff_ready}
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(path), 'exec'), namespace)
    return namespace[name]


class RotationHandoffTests(unittest.TestCase):
    def ack(self):
        return (10.1, {'session': 'new', 'revision': 'rotation', 'reason': 'revoked', 'applied': False})

    def limits(self):
        return (10.1, {'rotation_trial': True, 'translation_trial': False, 'geometry_revision': 'body'})

    def ready(self, ack=None, limits=None, now=10.2):
        return rotation_handoff_ready(now, 10., self.ack() if ack is None else ack,
            self.limits() if limits is None else limits, 'new', 'rotation', 'body')

    def test_disabled_rotation_trial_identity_can_acknowledge_handoff(self):
        self.assertTrue(self.ready())

    def test_old_translation_identity_or_prior_session_cannot_acknowledge(self):
        for key, value in [('revision', 'translation'), ('session', 'old'), ('reason', 'expired')]:
            stamp, ack = self.ack()
            ack[key] = value
            self.assertFalse(self.ready(ack=(stamp, ack)))
        self.assertFalse(self.ready(ack=(9.99, self.ack()[1])))
        self.assertFalse(self.ready(now=10.4))

    def test_old_motion_limits_cannot_acknowledge_rotation(self):
        stamp, limits = self.limits()
        limits.update(rotation_trial=False, translation_trial=True)
        self.assertFalse(self.ready(limits=(stamp, limits)))
        self.assertFalse(self.ready(limits=(9.99, self.limits()[1])))

    def test_transition_immediately_publishes_new_trial_before_any_clearance_test(self):
        calls = []
        node = SimpleNamespace(get_parameter=lambda name: SimpleNamespace(value=True),
            zero=lambda: calls.append('zero'), publish=lambda: calls.append(node.phase))
        method('complete_translation')(node, True, 'Translation passed')
        self.assertEqual(calls, ['zero', 'validating_rotation'])
        self.assertEqual(node.rotation_handoff_started, 10.)

    def node(self, now):
        calls = []
        node = SimpleNamespace(rotation_eligibility=lambda now: None, wander_state=('stop', now), requested=0.,
            rotation_handoff_started=10., applied_profile=None, safety_limits=self.limits(),
            profile_session='new', profile_revision='rotation', geometry_revision='body',
            rotation_wait=10.8, last_report=10., zero=lambda: calls.append('zero'),
            publish=lambda: calls.append('publish'), finish=lambda *args: calls.append(args))
        return node, calls

    def test_pending_handoff_holds_zero_without_failing_old_translation_clearance(self):
        node, calls = self.node(10.1)
        method('tick_rotation')(node, 10.1)
        self.assertEqual(calls, ['zero'])

    def test_missing_ack_times_out_at_zero_without_rotation(self):
        node, calls = self.node(12.1)
        method('tick_rotation')(node, 12.1)
        self.assertEqual(calls[0], 'zero')
        self.assertFalse(calls[1][0])
        self.assertIn('handoff', calls[1][1])

    def test_front_obstacle_preserves_guarded_relocation_after_handoff(self):
        node, calls = self.node(11.)
        node.rotation_handoff_started = None
        node.rotation_trial = None
        node.rotation_eligibility = lambda now, allow_front_blocked=False: (
            None if allow_front_blocked else 'Rotation safety hazard or missing state')
        node.rotation_clear = lambda now: False
        node.begin_relocation = lambda now: calls.append('relocate') or True
        method('tick_rotation')(node, 11.)
        self.assertEqual(calls, ['relocate'])

    def test_hard_hazard_aborts_during_handoff(self):
        node, calls = self.node(10.1)
        node.rotation_eligibility = lambda now, allow_front_blocked=False: 'Emergency stop must be explicitly released'
        method('tick_rotation')(node, 10.1)
        self.assertEqual(calls, [(False, 'Emergency stop must be explicitly released')])
