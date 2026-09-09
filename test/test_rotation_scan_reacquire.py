import math
from types import SimpleNamespace as NS
from test.test_rotation_failure_capture import adapter_method


def node(speed=0.):
    calls=[]
    trial=NS(last_speed=speed,last_time=10.,started=8.,leg_started=10.)
    state=NS(rotation_trial=trial,rotation_scan_pause=None,rotation_alignment=None,
        rotation_alignment_diagnostic={'reason':'insufficient_observed_returns','reference_observed':679,'current_observed':4,'minimum_observed':648.},
        baseline=NS(latest=lambda name:(0.,0.,.18)),rotation_imu_yaw=.18,last_report=10.,
        zero=lambda:calls.append('zero'),finish=lambda *args:calls.append(args),publish=lambda:calls.append('publish'))
    return state,calls


def pause(state,now):
    fn=adapter_method('calibration_rotation.py','pause_rotation_scan')
    fn.__globals__.update(math=math,wrap=lambda a:math.atan2(math.sin(a),math.cos(a)))
    return fn(state,now)


def test_stationary_dropout_holds_zero_then_fresh_alignment_returns_to_normal_gates():
    state,calls=node()
    assert pause(state,10.1)
    assert calls==['zero']
    assert state.rotation_trial.last_time==10.1
    assert state.rotation_trial.started==8.
    state.rotation_alignment={'yaw':.18}
    assert not pause(state,10.7)
    assert state.rotation_scan_pause is None
    assert calls==['zero','zero']


def test_unobserved_motion_or_ambiguous_scan_is_not_recoverable_pause():
    state,calls=node(.06)
    assert not pause(state,10.1)
    assert not calls
    state,calls=node();state.rotation_alignment_diagnostic['reason']='ambiguous_alignment'
    assert not pause(state,10.1)
    assert not calls


def test_stationary_wait_has_one_second_limit_and_pose_stability_guard():
    for reason in ('timeout','movement','yaw'):
        state,calls=node();assert pause(state,10.1)
        if reason=='movement':state.baseline.latest=lambda name:(.003,0.,.18)
        if reason=='yaw':state.rotation_imu_yaw=.20
        assert pause(state,11.2 if reason=='timeout' else 10.2)
        assert calls[-1][0] is False
        assert 'stationary pose changed' in calls[-1][1]