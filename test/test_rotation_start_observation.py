import math
from types import SimpleNamespace as NS
from test.test_rotation_failure_capture import adapter_method


def node():
 calls=[]
 state=NS(rotation_clearance_diagnostic={'reason':'safety_rotation_clearance_blocked','missing_bins':51},
   safety_limits=(10.,{'rotation_radius_m':.104}),rotation_points=[(.15,0.),(0.,.16)],
   baseline=NS(latest=lambda name:(0.,0.,.1)),rotation_imu_yaw=.1,
   rotation_start_observation_wait=None,last_report=10.,zero=lambda:calls.append('zero'),
   publish=lambda:calls.append('publish'),finish=lambda *args:calls.append(args))
 return state,calls


def wait(state,now):
 fn=adapter_method('calibration_rotation.py','wait_rotation_start_observation')
 fn.__globals__.update(math=math,wrap=lambda a:math.atan2(math.sin(a),math.cos(a)))
 return fn(state,now)


def test_partial_observation_wait_never_turns_blocked_gate_into_clearance():
 state,calls=node();assert wait(state,10.)
 assert calls==['zero']
 assert state.safety_limits[1].get('can_rotate') is None
 state.rotation_clearance_diagnostic={'reason':'current_safety_gate_clearance','missing_bins':0}
 assert not wait(state,10.5)
 assert state.rotation_start_observation_wait is None


def test_definite_measured_obstacle_is_not_classified_as_observation_gap():
 state,calls=node();state.rotation_points=[(.11,0.)]
 assert not wait(state,10.)
 assert calls==[]
 state,calls=node();state.rotation_clearance_diagnostic['missing_bins']=0
 assert not wait(state,10.)


def test_wait_expiry_and_pose_change_stop_even_when_gate_becomes_clear():
 for kind in ('timeout','translation','yaw'):
  state,calls=node();assert wait(state,10.)
  state.rotation_clearance_diagnostic={'reason':'current_safety_gate_clearance','missing_bins':0}
  if kind=='translation':state.baseline.latest=lambda name:(.003,0.,.1)
  if kind=='yaw':state.rotation_imu_yaw=.12
  assert wait(state,11.01 if kind=='timeout' else 10.5)
  assert calls[-1][0] is False


def test_rotation_phase_deadline_includes_preflight_wait():
 calls=[];state=NS(rotation_phase_started=10.,finish=lambda *args:calls.append(args))
 adapter_method('calibration_rotation.py','tick_rotation')(state,70.01)
 assert calls==[(False,'Rotation phase deadline exceeded')]