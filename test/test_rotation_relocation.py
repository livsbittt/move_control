import pytest
from rosy_control.control.rotation_relocation import RotationRelocation


def tick(controller, now, pose=(0.,0.,0.), **kwargs):
    args = dict(safe=True, can_rotate=False, suggestion=.02, requested_turn=True)
    args.update(kwargs)
    return controller.update(now, pose, **args)


def test_moves_straight_once_per_progress_region():
    r = RotationRelocation()
    assert tick(r, 0) == (.006, 'rotation_relocation')
    assert tick(r, 1, suggestion=.02)[0] == .006
    assert tick(r, 2, pose=(.02,0,0)) == (0., 'rotation_relocation_distance')
    assert tick(r, 3)[0] is None
    assert tick(r, 4, pose=(.101,0,0))[0] == .006


@pytest.mark.parametrize('changes,reason', [
    ({'safe':False}, 'unsafe'), ({'pose':None}, 'stale_pose'),
    ({'can_rotate':True}, 'clear'), ({'pose':(0,0,.051)}, 'heading'),
    ({'suggestion':None}, 'stale_suggestion')])
def test_abort_stops_once_and_keeps_attempt_consumed(changes, reason):
    r = RotationRelocation()
    tick(r, 0)
    assert tick(r, 1, **changes) == (0., 'rotation_relocation_' + reason)
    assert tick(r, 2)[0] is None


def test_timeout_reverse_and_explicit_reset():
    r = RotationRelocation()
    assert tick(r, 0, suggestion=-.03)[0] == -.006
    assert tick(r, 8, suggestion=-.03) == (0., 'rotation_relocation_timeout')
    assert tick(r, 9)[0] is None
    r.reset()
    assert tick(r, 10)[0] == .006


@pytest.mark.parametrize('changes', [
    {'suggestion':.031}, {'suggestion':float('nan')}, {'suggestion':0},
    {'safe':False}, {'can_rotate':True}, {'requested_turn':False},
    {'pose':None}, {'pose':(0,0,float('nan'))}])
def test_invalid_or_unneeded_request_cannot_start(changes):
    assert tick(RotationRelocation(), 0, **changes)[0] is None


def test_wrapped_heading_and_actual_displacement_bound():
    r = RotationRelocation()
    assert tick(r, 0, pose=(0,0,3.13))[0] == .006
    assert tick(r, 1, pose=(0,0,-3.13))[0] == .006
    assert tick(r, 2, pose=(0,.021,-3.13))[0] == 0


def test_repeated_small_movements_do_not_renew_attempt_and_clock_reset_stops():
    r = RotationRelocation()
    tick(r, 10)
    assert tick(r, 9) == (0., 'rotation_relocation_timeout')
    for now, x in enumerate((.03, .06, .09, .02, .08), 11):
        assert tick(r, now, pose=(x,0,0))[0] is None


def test_changed_safety_direction_stops_without_flipping_or_renewing_budget():
    r = RotationRelocation()
    assert tick(r, 0, suggestion=-.02)[0] == -.006
    assert tick(r, 1, suggestion=.01) == (0., 'rotation_relocation_direction_changed')
    assert not r.active and r.used
    assert tick(r, 2, suggestion=.01)[0] is None
