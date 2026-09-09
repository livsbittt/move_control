"""Exercise the actual observation gate without needing ROS on the test host."""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest
from rosy_control.control.calibration_profile import ProfileLease, make_profile


class LimitedSafetyTest(unittest.TestCase):
    def gate(self, lease, fresh, now=10.):
        path = Path(__file__).parents[1] / 'rosy_control/safety/evidence.py'
        tree = ast.parse(path.read_text(encoding='utf-8'))
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'required_observation_failure')
        scope = {'time': SimpleNamespace(monotonic=lambda: now)}
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), scope)
        node = SimpleNamespace(calibration_lease=lease,
            get_parameter=lambda name: SimpleNamespace(value=True),
            observations=SimpleNamespace(fresh=lambda name, stamp: name in fresh))
        return scope[method.name](node)

    def test_only_imu_excluded_and_other_hazards_stay_required(self):
        lease = ProfileLease()
        self.assertEqual(self.gate(lease, {'lidar','ir'}), 'imu_unavailable')
        lease.accept(make_profile('manual',1,100.,True,[1.,1.],'g',limited_sensors=True),100.,10.,'g')
        self.assertIsNone(self.gate(lease, {'lidar','ir'}))
        self.assertEqual(self.gate(lease, {'ir'}), 'lidar_unavailable')
        self.assertEqual(self.gate(lease, {'lidar'}), 'ir_unavailable')
        self.assertEqual(self.gate(lease, {'lidar','ir','imu'},12.), 'limited_sensor_authorization_unavailable')
