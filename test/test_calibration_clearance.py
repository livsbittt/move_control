import math
from move_control.control.calibration_clearance import motion_clearance
from move_control.control.lidar_guard import directional_lidar_limits


def limits(front=.183):
    return dict(front_m=front,rear_m=.157,front_stop_m=.12,rear_stop_m=.091,
                us_stop_m=.02,us_m=.3)


def test_optional_missing_echo_is_not_fake_distance_or_permission_past_lidar():
    d = limits()
    d.update(us_optional=True, us_m=None)
    result = motion_clearance(d, .03)
    assert result['reason'] is None and result['available_us_m'] is None
    d['front_m'] = .13
    assert motion_clearance(d, .03)['reason']
    d.update(front_m=.183, us_optional=False)
    assert motion_clearance(d, .03)['reason']


def test_verified_body_travel_replaces_unrelated_diagonal_range():
    d = limits(.09)
    d.update(translation_mode=True, forward_travel_m=.04, reverse_travel_m=.02,
             front_stop_m=.07, rear_stop_m=.07)
    result = motion_clearance(d, .03)
    assert result['reason'] is None
    assert result['target_m'] == .03
    d['forward_travel_m'] = .015
    assert motion_clearance(d, .03)['reason']
    d['forward_travel_m'] = None
    assert motion_clearance(d, .03)['reason']


def test_actual_gate_limits_reserve_only_planned_travel():
    r=motion_clearance(limits(),.03)
    assert r['reason'] is None and r['target_m']==.03
    assert abs(r['required_front_m']-.158)<1e-9


def test_adapts_within_evidence_bounds_and_never_uses_negative_target():
    assert motion_clearance(limits(.153),.03)['target_m']==.025
    assert motion_clearance(limits(.147),.03)['reason']
    r=motion_clearance(limits(.1),.03)
    assert r['target_m']==0 and r['reason']


def test_wide_raw_jamb_cannot_be_overridden_by_clear_precision_reference():
    r=motion_clearance(limits(.13),.03)
    assert r['reason']  # The stable narrow reference may still read 0.65m.
    assert motion_clearance(limits(.145),.03,target=.03)['reason']
    assert motion_clearance(limits(.145),.03,target=.03,forward=.02)['reason'] is None


def test_rear_and_invalid_inputs_fail_closed():
    d=limits(); d['rear_m']=.095
    assert motion_clearance(d,.03)['reason']
    for value in (None,math.nan,math.inf,0.):
        d=limits(); d['front_m']=value
        assert motion_clearance(d,.03)['reason']
    assert motion_clearance(limits(),.03,target=math.nan)['reason']
    d=limits(); d['us_m']=None
    assert motion_clearance(d,.03)['reason']


def test_rear_geometry_covers_full_cone_and_keeps_hysteresis():
    front,rear=directional_lidar_limits(.12,.14,.076,(-.017,0.))
    assert front==(.12,.14)
    assert .09<rear[0]<.1
    assert abs((rear[1]-rear[0])-.02)<1e-9
    # All rays in rear cone must clear radius plus18mm and configured extra.
    for i in range(101):
        a=math.pi-math.pi/4+i*math.pi/200
        assert math.hypot(-.017+rear[0]*math.cos(a),rear[0]*math.sin(a))>.094
    assert directional_lidar_limits(.12,.14,.076)[1]==(.12,.14)


def test_generated_low_request_is_normalized_to_geometry_not_used_as_range():
    front,rear=directional_lidar_limits(.018,.028,.076,(-.017,0.))
    assert abs(front[0]-.111)<1e-9
    assert .08<rear[0]<.083
    d=limits(.144); d['front_stop_m']=front[0]; d['rear_stop_m']=rear[0]
    r=motion_clearance(d,.03)
    assert r['target_m']==.025 and r['reason'] is None
