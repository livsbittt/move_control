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
