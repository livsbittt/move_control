from move_control.control.route_recovery import RouteRecovery


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


def test_waiting_for_planner_does_not_consume_physical_recovery_attempts():
    r=RouteRecovery()
    tick(r,0)
    assert tick(r,5)=='replan'
    assert tick(r,10,goal=None)=='waiting'
    assert tick(r,11)=='waiting'
    assert tick(r,15)=='waiting'
    assert tick(r,45,goal=None)=='waiting'
    assert tick(r,50,goal=(2.,0.))=='alternative'
    assert r.attempts == 1


def test_three_failed_executed_alternatives_exhaust_the_push_budget():
    r = RouteRecovery()
    tick(r, 0)
    assert tick(r, 5) == 'replan'
    for n in range(1, 4):
        assert tick(r, n*10, goal=(n+1., 0.)) == 'alternative'
        tick(r, n*10+1, goal=(n+1., 0.))
        assert tick(r, n*10+6, goal=(n+1., 0.)) == ('exhausted' if n == 3 else 'replan')


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
