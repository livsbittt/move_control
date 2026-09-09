import ast
import copy
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
import numpy as np
from rosy_control.control.rotation_envelope import RotationEnvelope,CROSS_ENDPOINT_MODEL,validate_envelope
from rosy_control.control.rotation_trial import RotationTrial


def displacement(center,yaw):
 c,s=math.cos(yaw),math.sin(yaw);x,y=center
 return ((1-c)*x+s*y,-s*x+(1-c)*y,yaw)


class CrossEndpointEnvelopeTests(unittest.TestCase):
 def envelope(self,cross):
  result=RotationEnvelope(.083,uncertainty_model=CROSS_ENDPOINT_MODEL) if cross else RotationEnvelope(.083)
  for i,sign in enumerate((-1,1,-1,1)):
   yaw=sign*math.radians(20 if cross else 10)
   delta=displacement((.001,-.002),yaw)
   self.assertTrue(result.add(delta,delta,yaw,.0009,endpoint_pair=[i,i+1] if cross else None))
  return result.report()

 def test_cross_excitation_reduces_conditioning_error_without_changing_body(self):
  legacy,new=self.envelope(False),self.envelope(True)
  self.assertTrue(validate_envelope(legacy));self.assertTrue(validate_envelope(new))
  self.assertEqual(new['body_radius_m'],legacy['body_radius_m'])
  self.assertAlmostEqual(new['center_uncertainty_m']/legacy['center_uncertainty_m'],math.sin(math.radians(5))/math.sin(math.radians(10)))
  self.assertLess(new['required_radius_m'],legacy['required_radius_m'])

 def test_reused_or_all_pairs_and_model_relabeling_are_rejected(self):
  report=self.envelope(True)
  for pair in ([0,2],[0,1],[3,2]):
   bad=copy.deepcopy(report);bad['trials'][1]['endpoint_pair']=pair
   self.assertFalse(validate_envelope(bad))
  bad=copy.deepcopy(report);bad['uncertainty_model']='empirical_residual_over_excitation_v1'
  self.assertFalse(validate_envelope(bad))
  envelope=RotationEnvelope(.083,uncertainty_model=CROSS_ENDPOINT_MODEL)
  delta=displacement((0.,0.),math.radians(-10))
  self.assertFalse(envelope.add(delta,delta,delta[2],.001,endpoint_pair=[0,1]))

 def test_opt_in_ten_legs_returns_home_with_physical_guards_unchanged(self):
  trial=RotationTrial(0.,endpoint_refinement=True);yaw=speed=0.
  for i in range(1,1400):
   yaw+=speed*.05/1.1
   speed=trial.update(i*.05,yaw,yaw,yaw,0.,True)
   self.assertLessEqual(abs(speed),.06)
   self.assertLess(abs(yaw),math.radians(15))
   if trial.done or trial.error:break
  self.assertIsNone(trial.error);self.assertTrue(trial.done)
  report=trial.report();self.assertEqual(len(report['legs']),10)
  self.assertEqual(report['trial_sequence'],'cross_endpoint_v2')
  self.assertLess(abs(yaw),math.radians(2))
  self.assertEqual(len(RotationTrial(0.).targets),8)
  self.assertNotIn('trial_sequence',RotationTrial(0.).report())

 def test_adapter_pairs_only_consecutive_nonzero_endpoints_in_their_own_frames(self):
  path=Path(__file__).parents[1]/'rosy_control/calibration_rotation.py'
  cls=next(n for n in ast.parse(path.read_text(encoding='utf-8')).body if isinstance(n,ast.ClassDef))
  fn=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='record_rotation_endpoint')
  center=(.001,-.002);hints=[]
  def match(ref,cur,hint):
   hints.append(hint);x,y,yaw=displacement(center,hint)
   return dict(dx=x,dy=y,yaw=yaw,residual_m=.0009)
  scope={'math':math,'wrap':lambda a:math.atan2(math.sin(a),math.cos(a)),'match_motion':match}
  exec(compile(ast.Module(body=[fn],type_ignores=[]),str(path),'exec'),scope)
  node=NS(zero=lambda:None,rotation_endpoint_previous=None,rotation_endpoint_count=0,
          rotation_envelope=RotationEnvelope(.083,uncertainty_model=CROSS_ENDPOINT_MODEL))
  for index,deg in enumerate((10,-10,10,-10,10)):
   yaw=math.radians(deg);node.rotation_imu_yaw=yaw;node.rotation_points=np.array([[yaw,0.]])
   self.assertTrue(scope['record_rotation_endpoint'](node,{'yaw':yaw},displacement(center,yaw)))
   self.assertEqual(len(node.rotation_envelope.trials),index)
  self.assertEqual([round(math.degrees(yaw)) for yaw in hints],[-20,20,-20,20])
  self.assertTrue(validate_envelope(node.rotation_envelope.report()))