"""Exercise the real stuck check without importing ROS."""
import ast
from pathlib import Path
from types import SimpleNamespace
import math
import unittest
from rosy_control.control.recover import is_stuck_motion


class FinalSpeedStuckTest(unittest.TestCase):
    def check(self, elapsed, moved, final_speed=.005, age=.05):
        path = Path(__file__).parents[1] / 'rosy_control/wander/motion.py'
        cls = next(n for n in ast.parse(path.read_text(encoding='utf-8')).body if isinstance(n, ast.ClassDef))
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '_is_stuck')
        scope = dict(math=math, is_stuck_motion=is_stuck_motion)
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), scope)
        class Stamp:
            def __sub__(self, other): return SimpleNamespace(nanoseconds=elapsed*1e9)
        node = SimpleNamespace(seen_forward=True, motion_t=Stamp(), have_odom=True,
            now=Stamp, odom_x=moved, odom_y=0., motion_x=0., motion_y=0.,
            _fwd_speed=lambda:.014, _session_now=lambda:10., session_final_received=10.-age,
            session_final_v=final_speed,
            get_parameter=lambda name:SimpleNamespace(value={'stuck_m':.008,'stuck_sec':1.2}[name]))
        return scope['_is_stuck'](node)

    def test_limited_speed_cannot_be_called_stuck_before_expected_travel(self):
        self.assertFalse(self.check(1.2, .006))

    def test_real_stall_at_limited_speed_still_detected(self):
        self.assertTrue(self.check(2., 0.))

    def test_stale_final_output_cannot_suppress_recovery(self):
        self.assertTrue(self.check(1.2, 0., final_speed=0., age=1.))

    def test_fresh_gate_stop_is_not_motor_stall(self):
        self.assertFalse(self.check(2., 0., final_speed=0.))
