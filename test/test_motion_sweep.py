import math

import pytest

from move_control.control.motion_sweep import bounded_sweep_clearance, bounded_translation_limits


def test_translation_prefilter_uses_body_capsule_not_sector_radius():
    # A return beside the path is inside a 14.9 cm sector threshold, but
    # outside the body, uncertainty, stopping and unchanged 10 mm margins.
    points=[(.04,.14),(.4,0.),(-.4,0.)]
    limits=bounded_translation_limits(points,(-.04,-.01),.003,.114,.1)
    assert limits == (.03,.03)
    assert bounded_sweep_clearance(points,(-.04,-.01),.003,.119,.014,0.,.8)>0


def test_translation_prefilter_stops_at_obstacle_and_rejects_stale_observation():
    assert bounded_translation_limits([(.12,0.),(.4,1.),(-.4,1.)],(0.,0.),.003,.114,.1)==(0.,0.)
    assert bounded_translation_limits([(.4,0.),(.4,1.),(-.4,1.)],(0.,0.),.003,.114,.201) is None


def sweep(points, **changes):
    args = dict(center=(0., 0.), uncertainty=0., body_radius=.1,
                v=0., w=0., horizon=.8)
    args.update(changes)
    return bounded_sweep_clearance(points, **args)


def scene(point):
    return [point, (2., 2.), (-2., 2.)]


def test_stationary_clearance_and_initial_collision():
    assert sweep(scene((.2, 0.))) == pytest.approx(.09)
    assert sweep(scene((.11, 0.))) == pytest.approx(0.)
    assert sweep(scene((.105, 0.)), v=.014) < 0.
    assert sweep(scene((.2, 0.)), uncertainty=.02) == pytest.approx(.05)


@pytest.mark.parametrize('direction', [-1., 1.])
def test_straight_sweep_includes_end_and_preserves_direction(direction):
    point = (direction*.12, 0.)
    assert sweep(scene(point), v=direction*.014) < 0.
    assert sweep(scene(point), v=-direction*.014) > 0.


def test_dilation_covers_obstacle_between_sampled_poses():
    # At t=.025 the obstacle is tangent to the actual swept circle. It is
    # outside every undilated sample at t=0,.05,...; checking endpoints fails.
    assert sweep(scene((.014*.025, .11)), v=.014) <= 0.


@pytest.mark.parametrize('direction', [-1., 1.])
def test_offset_pivot_turn_has_correct_translation_sign(direction):
    # Positive yaw about a pivot at +x moves base_link toward -y.
    assert sweep(scene((0., -direction*.115)), center=(.1, 0.),
                 w=direction*.1) < 0.
    assert sweep(scene((0., -direction*.115)), center=(.1, 0.),
                 w=-direction*.1) > 0.


@pytest.mark.parametrize('v,w', [(.014, .1), (-.014, .1), (.014, -.1), (-.014, -.1)])
def test_mixed_sweep_is_conservative_against_dense_reference(v, w):
    center = (.025, -.01)
    points = [(math.cos(i)*.16, math.sin(i)*.16) for i in range(16)]
    sampled = sweep(points, center=center, uncertainty=.003, v=v, w=w)
    exact_upper_bound = math.inf
    for i in range(1601):
        t = .8*i/1600
        a = w*t
        x = (1-math.cos(a))*center[0]+math.sin(a)*center[1]+v*math.sin(a)/w
        y = -math.sin(a)*center[0]+(1-math.cos(a))*center[1]+v*(1-math.cos(a))/w
        exact_upper_bound = min(exact_upper_bound,
                                *(math.hypot(px-x, py-y)-.116 for px, py in points))
    assert sampled <= exact_upper_bound+1e-12
    assert exact_upper_bound-sampled < .001


def test_near_zero_angular_velocity_is_continuous():
    points = scene((.2, .02))
    straight = sweep(points, center=(.04, -.01), v=.014)
    for w in (1e-14, -1e-14):
        assert sweep(points, center=(.04, -.01), v=.014, w=w) == pytest.approx(straight, abs=1e-12)


@pytest.mark.parametrize('changes', [
    {'v': .01401}, {'w': -.10001}, {'horizon': .749}, {'horizon': 1.501},
    {'uncertainty': -.001}, {'uncertainty': .03001}, {'body_radius': 0.},
    {'margin': .009}, {'margin': float('nan')}, {'center': (0.,)},
    {'center': (float('inf'), 0.)}, {'v': True}, {'horizon': '1'},
    {'v': float('nan')}, {'body_radius': 10**1000},
])
def test_invalid_parameters_return_no_evidence(changes):
    assert sweep(scene((.2, 0.)), **changes) is None


@pytest.mark.parametrize('points', [None, [], [(1., 0.)]*2, [(1., 0., 0.)]*3,
                                   [(1., 0.), (2., 0.), (float('nan'), 1.)],
                                   [('1', 0.)]*3, [(True, 0.)]*3])
def test_invalid_point_cloud_returns_no_evidence(points):
    assert sweep(points) is None


def test_horizon_boundaries_and_longer_horizon():
    points = scene((.13, 0.))
    assert sweep(points, v=.014, horizon=.75) > 0.
    assert sweep(points, v=.014, horizon=1.5) < 0.
