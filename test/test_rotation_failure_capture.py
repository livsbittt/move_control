import ast
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace as NS
import unittest


def adapter_method(filename, name):
    path = Path(__file__).parents[1]/'rosy_control'/filename
    cls = next(n for n in ast.parse(path.read_text(encoding='utf-8')).body if isinstance(n, ast.ClassDef))
    function = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == name)
    scope={'Path':Path,'json':json,'time':NS(time=lambda:100.)}
    exec(compile(ast.Module(body=[function], type_ignores=[]),str(path),'exec'),scope)
    return scope[name]


class RegistrationFailureCaptureTest(unittest.TestCase):
    def test_angular_failure_saves_bounded_raw_scan_with_json_safe_dropouts(self):
        with TemporaryDirectory() as folder:
            calls=[];node=self.node(Path(folder)/'result.json',calls)
            node.rotation_reference=([.3,float('inf')],.01)
            node.rotation_scan=(10.,[float('nan'),.31],.01)
            node.rotation_alignment_diagnostic={'reason':'insufficient_observed_returns'}
            adapter_method('calibration_rotation.py','capture_rotation_registration_failure')(
                node,None,(0.,0.,.17),.17,reason='angular_alignment_unavailable')
            data=json.loads(Path(node.rotation_registration_failure['path']).read_text())
            self.assertEqual(data['angular_scan']['reference'],[.3,None])
            self.assertEqual(data['angular_scan']['current'],[None,.31])
            self.assertEqual(data['reason'],'angular_alignment_unavailable')
            self.assertEqual(calls,['zero'])

    def node(self, path, calls):
        return NS(zero=lambda:calls.append('zero'),get_parameter=lambda name:NS(value=str(path)),
            rotation_reference_points=[[.1,.2],[.3,.4]],rotation_points=[[.11,.21],[.31,.41]],
            rotation_alignment={'yaw':.17,'residual_m':.002},geometry_revision='g',
            rotation_envelope=NS(report=lambda:{'valid':False,'reason':'insufficient_bilateral_evidence'}),
            rotation_trial=NS(report=lambda:{'done':False,'legs':[{'ratio':1.1}]}))

    def test_capture_stops_first_and_saves_replay_inputs_but_report_only_has_summary(self):
        with TemporaryDirectory() as folder:
            calls=[];node=self.node(Path(folder)/'result.json',calls)
            adapter_method('calibration_rotation.py','capture_rotation_registration_failure')(node,None,(.001,0.,.17),.17)
            self.assertEqual(calls,['zero'])
            diagnostic=node.rotation_registration_failure
            self.assertTrue(diagnostic['saved'])
            data=json.loads(Path(diagnostic['path']).read_text())
            self.assertEqual(data['reference_points'],node.rotation_reference_points)
            self.assertEqual(data['current_points'],node.rotation_points)
            self.assertEqual(data['reason'],'registration_unobservable')
            report=adapter_method('calibration_atomic.py','rotation_report')(node)
            self.assertEqual(report['registration_failure'],diagnostic)
            self.assertNotIn('reference_points',json.dumps(report))

    def test_invalid_or_oversized_capture_remains_stopped_and_reports_error(self):
        with TemporaryDirectory() as folder:
            calls=[];node=self.node(Path(folder)/'result.json',calls)
            node.rotation_reference_points=[[0.,0.]]*181
            adapter_method('calibration_rotation.py','capture_rotation_registration_failure')(node,None,(0.,0.,.17),.17)
            self.assertEqual(calls,['zero'])
            self.assertFalse(node.rotation_registration_failure['saved'])
            self.assertIn('capture_error',node.rotation_registration_failure)
            self.assertFalse((Path(folder)/'result.rotation-failure.json').exists())
