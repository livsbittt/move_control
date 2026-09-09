import math

from rosy_control.control.escape_space import escape_space_plan


def plan(points, **kwargs):
    return escape_space_plan(points, rotation_radius=.1, center=(0., 0.),
        pivot_radius=.1, fully_observed=False, can_rotate=False,
        forward_room=.08, reverse_room=.08, **kwargs)


def test_distance_is_measured_from_geometry_and_direction_is_not_fixed():
    near = plan([(.109, 0), (.3, .2), (-.3, .2)])
    farther = plan([(.102, 0), (.3, .2), (-.3, .2)])
    assert near['candidates'][0]['direction'] == -1
    assert farther['candidates'][0]['target_m'] > near['candidates'][0]['target_m']
    assert plan([(-.109, 0), (.3, .2), (-.3, .2)])['candidates'][0]['direction'] == 1


def test_no_predicted_gain_is_not_invented_permission_to_escape():
    points = [(i*.01, side*.105) for i in range(-20, 21) for side in (-1, 1)]
    report = plan(points)
    assert not report['candidates']


def test_available_travel_and_observation_validity_bound_every_candidate():
    for points in ([], [(float('nan'), 0)], [(0, .2)]):
        assert not plan(points)['candidates']
    report = escape_space_plan([(.102, 0), (.3, .2), (-.3, .2)],
        rotation_radius=.1, center=(0.,0.), pivot_radius=.1,
        fully_observed=False, can_rotate=False, forward_room=.02, reverse_room=.01)
    assert all(c['target_m'] + .005 <= c['available_m'] + 1e-9 for c in report['candidates'])


def test_restoration_requires_actual_rotation_permission_and_measured_margin():
    points = [(0., .13), (.3, .2), (-.3, .2)]
    args = dict(rotation_radius=.1, center=(0.,0.), pivot_radius=.1,
        fully_observed=True, forward_room=.08, reverse_room=.08)
    assert escape_space_plan(points, can_rotate=True, **args)['rotation_restored']
    assert not escape_space_plan(points, can_rotate=False, **args)['rotation_restored']
    report = escape_space_plan([(0., .111), (.3, .2), (-.3, .2)], can_rotate=True, **args)
    assert report['rotation_restored'] and not report['candidates']
