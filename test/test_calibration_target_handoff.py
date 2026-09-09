"""Run the real preflight adapter across footprint/standard limit changes."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
from rosy_control.control.calibration_clearance import motion_clearance, preflight_clearance_wait


class TargetHandoffTest(unittest.TestCase):
    def setUp(self):
        path=Path(__file__).parents[1]/'rosy_control/startup_calibration_node.py'
        cls=next(n for n in ast.parse(path.read_text(encoding='utf-8')).body if isinstance(n,ast.ClassDef))
        method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='safe_motion')
        scope=dict(math=math,motion_clearance=motion_clearance,
                   motion_evidence=lambda start,current:{'lidar_delta_m':0.})
        exec(compile(ast.Module(body=[method],type_ignores=[]),str(path),'exec'),scope)
        self.safe=scope['safe_motion']
        self.node=SimpleNamespace(phase='validating_motion',motion_start=None,selected_target=.03,
            trial_gate_reason=lambda now:None,
            safety_limits=(10.,dict(front_m=.153,rear_m=.138,front_stop_m=.12,rear_stop_m=.09,
                                    us_stop_m=.02,translation_mode=False)),
            raw_ranges={'lidar':(10.,.153,True),'us':(10.,.8,True)},us_source_valid=True,
            get_parameter=lambda name:SimpleNamespace(value={'calibration_require_us_agreement':False,
                'calibration_round_trip':True,'calibration_distance_m':.03}[name]),
            estop=False,rear_clear=(10.,True),precision_sensors_fresh=lambda now:True,
            snapshot=lambda:{'odom':(0.,0.,0.),'lidar':.153,'us':.8,'map_tf':(0.,0.,0.)},
            hazards={name:(10.,False) for name in ('/safety/blocked','/safety/cliff','/safety/tilt','/safety/pickup')})

    def test_pre_origin_reselects_33mm_clearance_to_22mm_target(self):
        self.assertIsNone(self.safe(self.node,10.))
        self.assertAlmostEqual(self.node.selected_target,.022)
        self.assertAlmostEqual(self.node.motion_clearance['required_travel_m'],.03)

    def test_later_room_cannot_expand_selected_target(self):
        self.node.selected_target=.022
        self.node.safety_limits[1].update(translation_mode=True,forward_travel_m=.114,reverse_travel_m=.05)
        self.assertIsNone(self.safe(self.node,10.))
        self.assertEqual(self.node.selected_target,.022)

    def test_origin_capture_freezes_target_and_insufficient_room_stops(self):
        self.node.motion_start=(9.5,{})
        self.assertEqual(self.safe(self.node,10.),'Insufficient remaining forward travel clearance')
        self.assertEqual(self.node.selected_target,.03)

    def test_too_little_preflight_room_cannot_shrink_below_evidence_minimum(self):
        self.node.safety_limits[1]['front_m']=.14
        self.assertIsNotNone(self.safe(self.node,10.))
        self.assertEqual(self.node.selected_target,.03)

    def test_clearance_grace_is_bounded_and_only_before_origin(self):
        reason='Insufficient clearance for minimum 2 cm motion evidence'
        self.assertTrue(preflight_clearance_wait('validating_motion',None,reason,11.99,10.))
        for phase,origin,why,now in [('validating_motion',None,reason,12.),
                ('validating_motion',(10.,{}),reason,10.1),('waiting_motion',None,reason,10.1),
                ('validating_motion',None,'Safety hazard or missing fresh safety state',10.1)]:
            self.assertFalse(preflight_clearance_wait(phase,origin,why,now,10.))

    def test_hard_hazard_overrides_transient_clearance_failure(self):
        self.node.safety_limits[1]['front_m']=.139
        self.node.hazards['/safety/cliff']=(10.,True)
        self.assertEqual(self.safe(self.node,10.),'Safety hazard or missing fresh safety state')
        self.node.hazards['/safety/cliff']=(10.,False)
        self.node.precision_sensors_fresh=lambda now:False
        self.node.sensor_failure=lambda now:'Sensor data became stale or invalid'
        self.assertEqual(self.safe(self.node,10.),'Sensor data became stale or invalid')

    def test_valid_footprint_recovers_without_expanding_target(self):
        self.node.safety_limits[1]['front_m']=.139
        reason=self.safe(self.node,10.)
        self.assertTrue(preflight_clearance_wait(self.node.phase,None,reason,10.,9.5))
        self.assertEqual(self.node.selected_target,.03)
        self.node.safety_limits[1].update(translation_mode=True,forward_travel_m=.042,reverse_travel_m=.05)
        self.assertIsNone(self.safe(self.node,10.))
        self.assertEqual(self.node.selected_target,.03)

    def test_actual_tick_clearance_wait_publishes_zero_and_expires(self):
        path=Path(__file__).parents[1]/'rosy_control/startup_calibration_node.py'
        tree=ast.parse(path.read_text(encoding='utf-8'))
        tick=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='tick')
        branch=next(n for n in ast.walk(tick) if isinstance(n,ast.If)
                    and ast.unparse(n.test)=="self.phase == 'validating_motion'")
        fn=ast.parse('def run(self, now): pass').body[0]
        fn.body=branch.body
        scope={'preflight_clearance_wait':preflight_clearance_wait}
        exec(compile(ast.fix_missing_locations(ast.Module(body=[fn],type_ignores=[])),str(path),'exec'),scope)
        calls=[]
        node=self.node
        node.safety_limits[1]['front_m']=.139
        node.safe_motion=lambda now:self.safe(node,10.)
        node.requested=9.5
        node.precision_pause_started=None
        node.zero=lambda:calls.append('zero')
        node.publish=lambda:calls.append('publish')
        node.finish=lambda ok,reason:calls.append(('finish',ok,reason))
        scope['run'](node,10.)
        self.assertEqual(calls,['zero','publish'])
        self.assertIsNone(node.motion_start)
        scope['run'](node,11.5)
        self.assertEqual(calls[-1][0:2],('finish',False))
