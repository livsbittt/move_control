import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from rosy_control.control.calibration_runtime import precision_scan_required


def method(name):
    path = Path(__file__).parents[1]/'rosy_control/startup_calibration_node.py'
    cls = next(n for n in ast.parse(path.read_text(encoding='utf-8')).body if isinstance(n, ast.ClassDef))
    function = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == name)
    scope = {'precision_scan_required': precision_scan_required, 'math': math, 'time': NS(monotonic=lambda: 10.),
             'rclpy': NS(time=NS(Time=lambda: None)), 'is_robot_scan': lambda m: True,
             'nose_from_quaternion': lambda *args: 0., 'sector_range': lambda *args, **kwargs: .4}
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(path), 'exec'), scope)
    return scope[name]


class PrecisionOdomPauseTest(unittest.TestCase):
    def node(self):
        calls=[]
        health={name: {'eligible': name!='lidar'} for name in ('lidar','odom','imu','ir','tf','map_tf')}
        node=NS(estop=False,motion_start=(0.,{}),round_trip=NS(error=None,pause=lambda now:calls.append('pause')),
            runtime_health=lambda now:health,map_tf_diagnostic={},
            wall_tracker=NS(locked=True,diagnostic={'status':'invalid','reason':'Waiting for fresh odometry'}),
            baseline=NS(samples={'odom':[(9.7,(0.,0.,0.),True)]}),
            safety_limits=(10.,{'front_stop_m':.1,'us_stop_m':.1}),
            raw_ranges={'lidar':(10.,.4,True),'us':(10.,.4,True)},
            hazards={key:(10.,False) for key in ('/safety/blocked','/safety/cliff','/safety/tilt','/safety/pickup')},
            precision_pause_started=None,precision_pause_map=False,
            zero=lambda:calls.append('zero'),publish=lambda:calls.append('publish'))
        return node,calls

    def test_transient_odom_gap_holds_zero_with_bounded_origin_preserving_pause(self):
        node,calls=self.node()
        self.assertTrue(method('pause_precision')(node,10.))
        self.assertEqual(calls,['zero','pause','publish'])
        self.assertIn('fresh odometry',node.message)
        self.assertEqual(node.motion_start,(0.,{}))
        node.safety_limits=(11.01,node.safety_limits[1])
        node.raw_ranges={key:(11.01,.4,True) for key in ('lidar','us')}
        node.hazards={key:(11.01,False) for key in node.hazards}
        self.assertFalse(method('pause_precision')(node,11.01))

    def test_invalid_odom_hazard_or_stale_raw_scan_cannot_reacquire(self):
        for change in ('invalid_odom','hazard','raw'):
            node,calls=self.node()
            if change=='invalid_odom':node.baseline.samples['odom'][-1]=(9.7,(),False)
            elif change=='hazard':node.hazards['/safety/cliff']=(10.,True)
            else:node.raw_ranges['lidar']=(9.7,.4,True)
            self.assertFalse(method('pause_precision')(node,10.))
            self.assertEqual(calls,[])

    def test_scan_skipped_for_odom_age_replaces_old_success_diagnostic(self):
        node,calls=self.node()
        node.phase='validating_motion';node.stamped=lambda *args:True
        node.baseline.latest=lambda name:(0.,0.,0.)
        transform=NS(transform=NS(rotation=NS(x=0.,y=0.,z=0.,w=1.),translation=NS(x=0.,y=0.)))
        node.tf=NS(lookup_transform=lambda *args:transform)
        node.rotation_scan_sample=lambda *args:None
        node.add_range=lambda *args:calls.append(args)
        node.wall_tracker.diagnostic={'status':'ok','locked':True}
        method('on_scan')(node,NS(header=NS(frame_id='laser')))
        self.assertEqual(node.wall_tracker.diagnostic['reason'],'Waiting for fresh odometry')
        self.assertEqual(calls,[('lidar',math.inf,False)])
        self.assertTrue(node.raw_ranges['lidar'][2])
