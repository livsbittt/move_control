from move_control.control.route_recovery import RouteRecovery
import math


def tick(r,t,reason='front_blocked',goal=(1.,0.),pose=(0.,0.,0.),safe=True,stamp=None):
    return r.update(t,pose,reason,goal,t if stamp is None else stamp,safe)


def test_zero_command_hold_replans_then_requires_a_different_fresh_route():
    r=RouteRecovery()
    assert tick(r,0)=='following'
    assert tick(r,5)=='replan'
    assert tick(r,6)=='waiting'
    assert tick(r,7,goal=(2.,0.),stamp=4)=='waiting'
    assert tick(r,8,goal=(2.,0.))=='alternative'
    assert r.attempts==1


def test_no_route_does_not_erase_failed_target_and_retries_are_bounded():
    r=RouteRecovery()
    tick(r,0)
    assert tick(r,5)=='replan'
    assert tick(r,10,goal=None)=='replan'
    assert tick(r,11)=='waiting'
    assert tick(r,15)=='replan'
    assert tick(r,20)=='exhausted'
    assert tick(r,50,goal=(2.,0.))=='exhausted'


def test_normal_alignment_is_allowed_but_endless_rotation_is_not_progress():
    r=RouteRecovery()
    assert tick(r,0,'align')=='following'
    assert tick(r,30,'align',pose=(0.,0.,3.))=='following'
    assert tick(r,45,'align',pose=(0.,0.,6.))=='replan'


def test_safety_hold_cannot_request_recovery_and_progress_then_freeze_is_detected():
    r=RouteRecovery()
    assert tick(r,0,safe=False)=='safety_hold'
    assert tick(r,60,safe=False)=='safety_hold'
    tick(r,61,'forward')
    tick(r,70,'forward',pose=(.03,0.,0.))
    assert tick(r,115,'forward',pose=(.03,0.,0.))=='replan'


def test_new_endpoint_on_same_blocked_exit_is_not_an_alternative_path():
    r=RouteRecovery()
    r.update(0,(0,0,0),'front_blocked',(1,0),0,True,(.06,0))
    assert r.update(5,(0,0,0),'front_blocked',(1,0),5,True,(.06,0))=='replan'
    assert r.update(6,(0,0,0),'forward',(2,0),6,True,(.06,0))=='waiting'
    assert r.update(7,(0,0,0),'forward',(1,0),7,True,(0,.06))=='alternative'


def test_slow_alignment_improvement_extends_progress_without_resetting_attempts():
    r = RouteRecovery()
    r.attempts = 2
    assert tick(r, 0, 'align', pose=(0., 0., -2.)) == 'following'
    for t, yaw in ((20, -1.8), (40, -1.6), (60, -1.4)):
        assert tick(r, t, 'align', pose=(0., 0., yaw)) == 'following'
    assert r.attempts == 2
    assert tick(r, 105, 'align', pose=(0., 0., -1.4)) == 'replan'


def test_alignment_oscillation_cannot_repeatedly_credit_same_heading():
    r = RouteRecovery()
    tick(r, 0, 'turn_away', pose=(0., 0., -1.))
    tick(r, 5, 'turn_away', pose=(0., 0., -.8))
    for t, yaw in ((15, -1.), (25, -.8), (35, -1.), (45, -.8)):
        assert tick(r, t, 'turn_away', pose=(0., 0., yaw)) == 'following'
    assert tick(r, 50, 'turn_away', pose=(0., 0., -.8)) == 'replan'


def test_alignment_target_drift_is_not_angular_progress():
    r = RouteRecovery()
    tick(r, 0, 'align', goal=(0., 1.))
    for t, target in ((10, (.5, .5)), (20, (1., .1)), (30, (.5, .5))):
        assert tick(r, t, 'align', goal=target) == 'following'
    assert tick(r, 45, 'align', goal=(1., 0.)) == 'replan'


def test_alignment_progress_wraps_pi_boundary():
    r = RouteRecovery()
    tick(r, 0, 'align', goal=(-1., 0.), pose=(0., 0., math.pi-.2))
    assert tick(r, 40, 'align', goal=(-1., 0.),
                pose=(0., 0., -math.pi+.1)) == 'following'
    assert tick(r, 60, 'align', goal=(-1., 0.),
                pose=(0., 0., -math.pi+.1)) == 'following'
    assert tick(r, 85, 'align', goal=(-1., 0.),
                pose=(0., 0., -math.pi+.1)) == 'replan'
