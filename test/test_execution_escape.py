"""Measured escape proposals, not a fixed displacement."""
from rosy_control.control.execution_escape import ExecutionEscape


def proposal(t,direction=-1,target=.023,margin=.015):
    return dict(geometry_revision='g',issued_s=t,scan_age_s=.05,
        rotation_restored=False,candidates=[dict(direction=direction,target_m=target,
        available_m=.10,predicted_rotation_clearance_m=margin)])


def update(e,t,pose=(0.,0.,0.),**changes):
    args=dict(safe=True,geometry='g',remaining_s=30.,waiting=True,proposal=proposal(t))
    args.update(changes)
    return e.update(t,pose,**args)


def test_measured_target_changes_live_and_clearance_restoration_stops_early():
    e=ExecutionEscape();assert update(e,0)[0] is None
    assert update(e,5)==(-.005,'execution_escape_reverse')
    assert update(e,7,(-.01,0,0),proposal=proposal(7,target=.009))[0]==-.005
    p=proposal(8,target=.005);p['rotation_restored']=True
    assert update(e,8,(-.015,0,0),proposal=p)==(0.,'execution_escape_complete')
    assert update(e,20)[0] is None


def test_forward_candidate_uses_measured_distance():
    e=ExecutionEscape();p=proposal(0,1,target=.036)
    update(e,0,proposal=p);p['issued_s']=5
    assert update(e,5,proposal=p)==(.005,'execution_escape_forward')


def test_short_restoring_move_wins_over_larger_clearance_gain():
    e=ExecutionEscape();p=proposal(0,1,target=.07,margin=.03)
    p['candidates'][0]['objective']='improve_clearance'
    other=proposal(0,-1,target=.001,margin=.014)['candidates'][0]
    other['objective']='restore_rotation'
    p['candidates'].append(other)
    assert e.select(0,p,'g',30)['target_m']==.001
    p['candidates'][0]['objective']='restore_rotation'
    assert e.select(0,p,'g',30)['target_m']==.001


def test_hazards_stale_proposal_geometry_or_candidate_loss_stop_consumed_attempt():
    for failure in ('hazard','stale','geometry','missing','scan'):
        e=ExecutionEscape();update(e,0);update(e,5)
        p=proposal(6);args=dict(proposal=p)
        if failure=='hazard':args['safe']=False
        if failure=='stale':p['issued_s']=5
        if failure=='geometry':args['geometry']='other'
        if failure=='missing':p['candidates']=[]
        if failure=='scan':p['scan_age_s']=.3
        assert update(e,6,**args)==(0.,'execution_escape_stopped')
        assert update(e,20)[0] is None


def test_direction_never_flips_midmove_and_wrong_direction_motion_stops():
    for pose,p in (((.004,0,0),proposal(6)),((0,0,0),proposal(6,1))):
        e=ExecutionEscape();update(e,0);update(e,5)
        assert update(e,6,pose,proposal=p)==(0.,'execution_escape_stopped')


def test_target_requires_observed_room_and_time_and_cannot_expand_past_total_cap():
    for target,room,remaining in ((.081,.1,30),(.03,.02,30),(.05,.1,10)):
        e=ExecutionEscape();p=proposal(0,target=target);p['candidates'][0]['available_m']=room
        update(e,0,proposal=p,remaining_s=remaining);p['issued_s']=5
        assert update(e,5,proposal=p,remaining_s=remaining)[0] is None
    e=ExecutionEscape();update(e,0);update(e,5)
    assert update(e,6,(-.02,0,0),proposal=proposal(6,target=.07))==(0.,'execution_escape_stopped')


def test_time_heading_lateral_and_session_end_stop():
    for t,pose,changes in ((25,(0,0,0),{}),(6,(0,0,.04),{}),
                           (6,(0,.004,0),{}),(6,(0,0,0),dict(waiting=False))):
        e=ExecutionEscape();update(e,0);update(e,5)
        assert update(e,t,pose,**changes)==(0.,'execution_escape_stopped')


def test_bounded_two_mm_candidate_starts_without_attempt_or_stability_latch():
    e=ExecutionEscape();e.used=True
    p=proposal(0,target=.002)
    assert update(e,0,proposal=p,time_bounded=True)==(-.005,'execution_escape_reverse')
    assert e.snapshot()['reason']=='motion'


def test_bounded_transient_null_resumes_same_episode_without_resetting_clock():
    e=ExecutionEscape()
    assert update(e,0,time_bounded=True)[0]==-.005
    assert update(e,.1,proposal=None,time_bounded=True)[0]==0.
    assert e.snapshot()['reason']=='waiting_for_candidate'
    assert update(e,.2,time_bounded=True)[0]==-.005
    assert e.started==0.
    assert update(e,8,time_bounded=True)[0]==0.
    assert e.snapshot()['reason']=='no_progress'
    assert update(e,9,time_bounded=True)[0] is None


def test_actual_thirteen_mm_escape_transient_null_then_two_mm_candidate_resumes():
    e=ExecutionEscape()
    assert update(e,10.045,proposal=proposal(10.045,target=.013),time_bounded=True)[0]==-.005
    assert update(e,10.646,(-.0025,0,0),proposal=proposal(10.646,target=.011),time_bounded=True)[0]==-.005
    assert update(e,10.747,(-.0028578,0,0),proposal=None,time_bounded=True)[0]==0.
    assert update(e,10.847,(-.0028578,0,0),proposal=proposal(10.847,target=.002),time_bounded=True)[0]==-.005
    assert e.started==10.045


def test_bounded_new_actual_space_allows_new_episode_after_no_progress():
    e=ExecutionEscape();update(e,0,time_bounded=True)
    update(e,8,time_bounded=True)
    p=proposal(9);p['current_rotation_clearance_m']=.01
    assert update(e,9,(.011,0,0),proposal=p,time_bounded=True)[0]==-.005


def test_physical_dropout_fixture_resumes_with_original_episode_clock():
    import json
    from pathlib import Path
    rows=json.loads((Path(__file__).parent/'fixtures/execution_escape_dropout_20260910.json').read_text())['samples']
    e=ExecutionEscape();velocities=[]
    for row in rows:
        p=row['proposal']
        now=max(row['now'],p['issued_s'] if p else row['now'])+.001
        velocity,_=e.update(now,tuple(row['pose']),safe=True,proposal=p,
            geometry=row['geometry'],remaining_s=30.,waiting=True,time_bounded=True)
        velocities.append(velocity)
    assert velocities==[-.005,-.005,0.,-.005,-.005]
    assert e.started==max(rows[0]['now'],rows[0]['proposal']['issued_s'])+.001


def test_one_restored_frame_then_loss_resumes_original_episode():
    e=ExecutionEscape();update(e,0,time_bounded=True)
    p=proposal(.2);p['rotation_restored']=True
    assert update(e,.2,proposal=p,time_bounded=True)[0]==0.
    assert update(e,.3,time_bounded=True)[0]==-.005
    assert e.started==0.


def test_escape_distance_before_restored_pulse_does_not_count_as_new_navigation_progress():
    e=ExecutionEscape();update(e,0,time_bounded=True)
    p=proposal(2);p['rotation_restored']=True
    update(e,2,(-.01,0,0),proposal=p,time_bounded=True)
    assert update(e,2.1,(-.01,0,0),time_bounded=True)[0]==-.005
    assert e.started==0.


def test_worse_clearance_does_not_rearm_no_progress_hold():
    e=ExecutionEscape();p=proposal(0);p['current_rotation_clearance_m']=.01
    update(e,0,proposal=p,time_bounded=True)
    p['issued_s']=8;update(e,8,proposal=p,time_bounded=True)
    p['issued_s']=9;p['current_rotation_clearance_m']=.001
    assert update(e,9,proposal=p,time_bounded=True)[0] is None
    assert e.started==0.


def test_actual_heading_progress_after_restoration_opens_new_pose_episode():
    e=ExecutionEscape();update(e,0,time_bounded=True)
    p=proposal(.2);p['rotation_restored']=True
    update(e,.2,proposal=p,time_bounded=True)
    assert update(e,1,(0,0,.1),time_bounded=True)[0]==-.005
    assert e.started==1.
