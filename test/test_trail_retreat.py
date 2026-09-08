import math
import pytest
from move_control.control.trail_retreat import TrailRetreat, _tracking_target


def started(route=None):
    r = TrailRetreat()
    assert r.start(0, (0,0,0), route or [(0,0),(-.1,0)], 'g1')
    return r


def test_straight_back_and_endpoint_stop_without_rotation():
    r = started([(0,0),(-.01,0)])
    assert r.update(.1,(0,0,0),'g1',True,False) == (-.006,0.,'trail_retreat')
    assert r.update(.2,(-.01,0,0),'g1',True,False) == (0.,0.,'trail_retreat_endpoint_hold')
    assert r.active
    assert r.update(.3,(-.01,0,0),'g1',True,True) == (0.,0.,'trail_retreat_complete')
    assert not r.active


def test_curve_correction_turns_toward_reverse_heading():
    r=started([(0,0),(-.1,-.01)])
    v,w,reason=r.update(.1,(0,0,0),'g1',True,False)
    assert -.006<v<0 and w==.04 and reason=='trail_retreat'


def test_dense_straight_trail_does_not_turn_toward_a_nearly_reached_sample():
    r=started([(0,0),(-.01,0),(-.02,0),(-.03,0),(-.1,0)])
    # Two mm of lateral error is trackable; aiming at the sample only three
    # mm behind turns this into a spurious 34-degree heading failure.
    v,w,reason=r.update(.1,(-.007,.002,0),'g1',True,False)
    assert v < 0 and 0 < w <= .04 and reason == 'trail_retreat'


def test_reverse_lookahead_does_not_skip_a_sharp_corner():
    r=started([(0,0),(-.01,0),(-.01,.1)])
    assert r.update(.1,(-.007,0,0),'g1',True,False)[2] == 'trail_retreat_heading'


def test_dense_trail_closed_loop_reaches_refuge_with_lateral_error():
    r=started([(0,0)]+[(-i*.01,0) for i in range(1,11)])
    x,y,yaw=0.,.002,0.
    for tick in range(1,401):
        v,w,reason=r.update(tick*.1,(x,y,yaw),'g1',True,True)
        if reason == 'trail_retreat_complete':
            break
        assert reason == 'trail_retreat'
        x+=v*math.cos(yaw)*.1
        y+=v*math.sin(yaw)*.1
        yaw+=w*.1
    assert reason == 'trail_retreat_complete'
    assert math.hypot(x+.1,y) <= .005


def test_lookahead_does_not_use_a_later_crossing_as_clearance():
    r=started([(0,0),(-.01,0),(-.03,0),(-.03,.03),(0,.03)])
    assert r.update(.1,(0,.03,0),'g1',True,False) == (0.,0.,'trail_retreat_off_route')


def test_duplicate_xy_sample_does_not_hide_a_following_corner():
    route=[(0,0),(0,0),(-.01,0),(-.01,.02)]
    assert _tracking_target(route,1,(0,.004,0)) == (-.01,0)


def test_recorded_offset_pivot_retreat_returns_to_refuge_without_false_reverse():
    rotation={'center_m':[-.0398094981068605,-.010226923630367434],
              'center_uncertainty_m':.003105398233770068}
    pose=(.30381500290790603,-.2465664264190329,-.622196413104942)
    route=[(.30448805631278675,-.24474336574229683,-.5750591410519561),
           (.3052192596658002,-.24238041179558392,-.5150591487443499),
           (.30576395547741236,-.24017937855748317,-.4600591567218455)]
    r=TrailRetreat()
    assert r.start(0,pose,route,'g1',rotation=rotation)
    cx,cy=rotation['center_m']
    x,y,yaw=pose
    px=x+math.cos(yaw)*cx-math.sin(yaw)*cy
    py=y+math.sin(yaw)*cx+math.cos(yaw)*cy
    for tick in range(1,151):
        v,w,reason=r.update(tick*.1,(x,y,yaw),'g1',True,True,rotation=rotation)
        assert v == 0.
        if reason == 'trail_retreat_complete':
            break
        assert reason == 'trail_retreat_turn' and 0 < w <= .04
        yaw+=w*.1
        x=px-math.cos(yaw)*cx+math.sin(yaw)*cy
        y=py-math.sin(yaw)*cx-math.cos(yaw)*cy
    assert reason == 'trail_retreat_complete'
    assert abs(yaw-route[-1][2]) <= .005
    assert math.dist((x,y),route[-1][:2]) <= .003


def pivot_pose(yaw):
    return (-.04*math.cos(yaw)+.01*math.sin(yaw),
            -.04*math.sin(yaw)-.01*math.cos(yaw),yaw)


def pivot_started(direction=1):
    rotation={'center_m':[.04,.01],'center_uncertainty_m':.001}
    initial=pivot_pose(direction*3.13)
    route=[pivot_pose(math.atan2(math.sin(initial[2]+direction*a),
                                math.cos(initial[2]+direction*a))) for a in (.05,.1,.15)]
    r=TrailRetreat()
    assert r.start(0,initial,route,'g1',rotation=rotation)
    return r,initial,route,rotation


@pytest.mark.parametrize('direction',[-1,1])
def test_pivot_retreat_wraps_yaw_and_requires_current_refuge_clearance(direction):
    r,pose,route,rotation=pivot_started(direction)
    v,w,reason=r.update(.1,pose,'g1',True,True,rotation=rotation)
    assert v == 0 and w*direction > 0 and reason == 'trail_retreat_turn'
    assert r.update(.5,route[-1],'g1',True,False,rotation=rotation)[2]=='trail_retreat_endpoint_hold'
    assert r.update(.6,route[-1],'g1',True,True,rotation=rotation)[2]=='trail_retreat_complete'


@pytest.mark.parametrize('change,reason', [('lost','estimate_changed'),('changed','estimate_changed'),
                                         ('drift','pivot_drift'),('unsafe','unsafe')])
def test_pivot_retreat_aborts_on_lost_model_or_current_safety(change,reason):
    r,pose,route,rotation=pivot_started()
    if change=='lost': rotation=None
    if change=='changed': rotation={'center_m':[.041,.01],'center_uncertainty_m':.001}
    if change=='drift': pose=(pose[0]+.003,pose[1],pose[2])
    assert r.update(.1,pose,'g1',change!='unsafe',True,rotation=rotation)==(0.,0.,'trail_retreat_'+reason)


def test_short_xy_distance_does_not_finish_unreached_pivot_yaw():
    r,pose,route,rotation=pivot_started()
    near=pivot_pose(route[-1][2]-.04)
    assert math.dist(near[:2],route[-1][:2]) < .003
    assert r.update(.5,near,'g1',True,True,rotation=rotation)[2]=='trail_retreat_turn'


def test_pivot_yaw_arrival_with_unreachable_xy_error_stops_explicitly():
    rotation={'center_m':[.04,.01],'center_uncertainty_m':.01}
    pose=pivot_pose(0.)
    target=pivot_pose(.2)
    shifted=(target[0]+.004,target[1],target[2])
    r=TrailRetreat()
    assert r.start(0,pose,[pivot_pose(.1),shifted],'g1',rotation=rotation)
    assert r.turn_target is not None
    assert r.update(.5,target,'g1',True,True,rotation=rotation)==(0.,0.,'trail_retreat_position_mismatch')


@pytest.mark.parametrize('kind',['translation','yaw_reversal'])
def test_pivot_shortcut_does_not_merge_mixed_motion_or_yaw_reversals(kind):
    _,pose,route,rotation=pivot_started()
    if kind=='translation':
        route[-1]=(route[-1][0]+.01,route[-1][1],route[-1][2])
    else:
        route.insert(1,pose)
    r=TrailRetreat()
    assert r.start(0,pose,route,'g1',rotation=rotation)
    assert r.turn_target is None


@pytest.mark.parametrize('now,pose,identity,safe,reason',[
    (.1,(0,0,0),'g1',False,'unsafe'),
    (.1,None,'g1',True,'stale_pose'),
    (.1,(0,0,0),'g2',True,'geometry_changed'),
    (120,(0,0,0),'g1',True,'timeout'),
    (.1,(0,.021,0),'g1',True,'off_route'),
    (.1,(0,0,.31),'g1',True,'heading'),
])
def test_unsafe_execution_aborts_zero(now,pose,identity,safe,reason):
    r=started()
    if reason == 'heading':
        r=TrailRetreat()
        assert r.start(0,pose,[(0,0),(-.1,0)],'g1')
    assert r.update(now,pose,identity,safe,False)==(0.,0.,'trail_retreat_'+reason)
    assert not r.active


@pytest.mark.parametrize('route', [[(0,0)],[(.03,0),(-.1,0)],[(0,0),(-.501,0)],
                                  [(0,0),(float('nan'),0)]])
def test_invalid_route_rejected(route):
    assert not TrailRetreat().start(0,(0,0,0),route,'g1')


def test_waypoints_advance_in_sequence_and_pi_wrap_is_normalized():
    r=TrailRetreat()
    assert r.start(0,(0,0,math.pi),[(0,0),(.01,0),(.1,0)],'g1')
    assert r.update(.1,(.01,0,-math.pi),'g1',True,False)==(-.006,0.,'trail_retreat')


def test_safe_trail_three_coordinate_poses_are_supported():
    r=TrailRetreat()
    assert r.start(0,(0,0,0),[(0,0,0),(-.1,0,.01)],'g1')
    assert r.update(.1,(0,0,0),'g1',True,False)[0]==-.006


def test_endpoint_hold_has_deadline_and_replan_cannot_extend_it():
    r=started([(0,0),(-.01,0)])
    for i in range(1,240):
        assert r.update(i*.5,(-.01,0,0),'g1',True,False)[2]=='trail_retreat_endpoint_hold'
    assert not r.start(119,(0,0,0),[(0,0),(-.1,0)],'g1')
    assert r.update(120,(-.01,0,0),'g1',True,False)[2]=='trail_retreat_timeout'


@pytest.mark.parametrize('now,pose,reason',[
    (.1,(-.06,0,0),'pose_jump'),
    (.1,(0,0,.14),'yaw_jump'),
    (.501,(0,0,0),'time_gap'),
    (-.01,(0,0,0),'time_gap'),
])
def test_fresh_but_discontinuous_odometry_aborts(now,pose,reason):
    r=started()
    assert r.update(now,pose,'g1',True,False)==(0.,0.,'trail_retreat_'+reason)
    assert not r.active
