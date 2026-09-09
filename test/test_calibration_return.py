import math
import pytest
from rosy_control.control.calibration_return import CalibrationReturn


def scan(radius=.4):
    return [(radius*math.cos(i*math.tau/72),radius*math.sin(i*math.tau/72)) for i in range(72)]


def tick(r,t,pose=(.018,0,0),**kwargs):
    args=dict(body_radius=.1144,full_scan_observed=True,fresh_guard=True,rotation_verified=True)
    args.update(kwargs)
    return r.update(t,pose,scan(),**args)


def test_returns_straight_and_confirms_original_pose_before_done():
    r=CalibrationReturn((0,0,0))
    assert tick(r,0)[0]==-.006
    for i in range(1,6):tick(r,i*.5,(.018-i*.003,0,0))
    assert not r.done
    assert tick(r,3,(.003,0,0))==(0.,'complete')
    assert r.report()['done'] and r.error is None


def test_forward_return_and_original_pose_offset_are_supported():
    r=CalibrationReturn((.001,0,0))
    assert tick(r,0,(-.02,0,0))[0]==.006


@pytest.mark.parametrize('kwargs',[
    {'fresh_guard':False},{'full_scan_observed':False},{'rotation_verified':False},
    {'body_radius':float('nan')},
])
def test_missing_live_authority_stops(kwargs):
    r=CalibrationReturn((0,0,0))
    assert tick(r,0,**kwargs)[0]==0.
    assert r.error


def test_new_obstacle_aborts_even_with_previously_clear_return_path():
    r=CalibrationReturn((0,0,0));tick(r,0)
    assert r.update(.1,(.018,0,0),scan(.12),body_radius=.1144,full_scan_observed=True,
                    fresh_guard=True,rotation_verified=True)==(0.,'straight_capsule_blocked')


@pytest.mark.parametrize('pose',[(.221,0,0),(.02,.011,0),(.02,0,.051)])
def test_start_distance_lateral_or_heading_limits(pose):
    r=CalibrationReturn((0,0,0))
    assert tick(r,0,pose)[0]==0. and r.error


def test_discontinuous_clock_or_pose_stops():
    for t,p in [(.501,(.018,0,0)),(-.1,(.018,0,0)),(.1,(-.02,0,0))]:
        r=CalibrationReturn((0,0,0));tick(r,0)
        assert tick(r,t,p)[0]==0. and r.error


def test_timeout_cannot_be_renewed_by_stationary_updates():
    r=CalibrationReturn((0,0,0));tick(r,0)
    for i in range(1,120):assert tick(r,i*.5)[0]<0
    assert tick(r,60)==(0.,'time_budget')


def test_unordered_or_partial_scan_cannot_grant_capsule():
    r=CalibrationReturn((0,0,0))
    points=scan();points[10],points[40]=points[40],points[10]
    assert r.update(0,(.018,0,0),points,body_radius=.1144,full_scan_observed=True,
                    fresh_guard=True,rotation_verified=True)==(0.,'scan_coverage')


def test_total_path_budget_and_geometry_change_stop():
    r=CalibrationReturn((0,0,0));tick(r,0,(.12,0,0))
    for i in range(1,25):
        result=tick(r,i*.5,(.13 if i%2 else .12,0,0))
    assert result==(0.,'distance_budget')
    r=CalibrationReturn((0,0,0));tick(r,0)
    assert tick(r,.1,body_radius=.115)==(0.,'geometry_changed')


def test_origin_confirmation_resets_if_pose_leaves_tolerance():
    r=CalibrationReturn((0,0,0))
    assert tick(r,0,(.003,0,0))[1]=='confirming_origin'
    assert tick(r,.25,(.006,0,0))[1]=='returning'
    assert tick(r,.5,(.003,0,0))[1]=='confirming_origin'
    assert tick(r,.75,(.003,0,0))[1]=='confirming_origin'
    assert tick(r,1,(.003,0,0))[1]=='complete'
