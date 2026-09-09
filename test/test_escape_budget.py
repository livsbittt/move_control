from rosy_control.control.escape_budget import EscapeBudget


def move(b,now,start,end):
    for i in range(1,201):
        b.observe(now+i*.1,(start+(end-start)*i/200,0.))
    return now+20.


def test_only_success_and_continuous_separated_progress_allow_next_region():
    b=EscapeBudget(); b.observe(1.,(0.,0.)); assert b.begin('one',1.)
    assert not b.eligible(1.)
    b.complete()
    t=move(b,1.,0.,.2); assert not b.eligible(t)
    t=move(b,t,.2,.3); assert b.begin('two',t)
    assert not b.begin('new-id-same-place',t)


def test_replays_returning_to_old_region_and_total_count_are_bounded():
    b=EscapeBudget(); b.observe(1.,(0.,0.)); t=1.
    for i in range(4):
        assert b.begin(str(i),t); b.complete()
        t=move(b,t,i*.3,(i+1)*.3)
    assert not b.begin('fifth',t)
    c=EscapeBudget(); c.observe(1.,(0.,0.)); c.begin('one',1.); c.complete()
    t=move(c,1.,0.,.3); t=move(c,t,.3,0.)
    assert not c.eligible(t)


def test_teleport_or_observation_gap_cannot_renew_budget():
    for now,pose in ((1.1,(.3,0.)),(2.,(.001,0.))):
        b=EscapeBudget(); b.observe(1.,(0.,0.)); b.begin('one',1.); b.complete()
        b.observe(now,pose)
        assert b.failed and not b.eligible(now)
