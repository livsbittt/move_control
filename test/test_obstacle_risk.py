from move_control.control.obstacle_risk import collision_risk, observation_risk
from move_control.control.obstacle_risk import camera_hold


def test_future_or_malformed_packet_does_not_poison_current_observation():
    from move_control.control.obstacle_risk import accept_observation
    old = {'stamp': 9.9}
    for stamp in (1000., float('nan'), True, '10', -1.):
        assert accept_observation({'stamp': stamp}, old, 10.) is old
    current = {'stamp': 10.}
    assert accept_observation(current, old, 10.) is current


def track(x, y, vx=0., vy=0., state='moving'):
    return dict(position=(x, y), velocity=(vx, vy), state=state,
                age=0., observed=True, radius=.03)


def test_crossing_object_requires_yield_before_contact():
    result = collision_risk([track(.15, .3, 0., -.3)], (0., 0., 0.), .1, .08, .02)
    assert result['action'] == 'wait'


def test_diverging_object_does_not_force_wait():
    assert collision_risk([track(.5, .3, 0., .3)], (0., 0., 0.), .1, .08, .02)['action'] == 'clear'


def test_static_object_requests_replan_and_unknown_requires_wait():
    for state, action in [('stationary', 'replan'), ('unknown', 'wait')]:
        assert collision_risk([track(.15, 0., state=state)], (0., 0., 0.), .1, .08, .02)['action'] == action


def test_body_and_uncertainty_cannot_shrink_with_slow_speed():
    assert collision_risk([track(.1, 0.)], (0., 0., 0.), 0., .08, .02)['action'] == 'wait'


def test_reverse_uses_reverse_swept_path():
    assert collision_risk([track(-.15, 0.)], (0., 0., 0.), -.1, .08, .02)['action'] == 'wait'


def test_missing_stale_future_and_wrong_frame_evidence_holds():
    for evidence in [None, {'stamp': 0., 'frame': 'odom', 'tracks': []},
                     {'stamp': 2., 'frame': 'odom', 'tracks': []},
                     {'stamp': 1., 'frame': 'map', 'tracks': []}]:
        assert observation_risk(evidence, 1., (0.,0.,0.), .1, .08, .02)['action'] == 'wait'
    assert observation_risk({'stamp': 1., 'frame': 'odom', 'tracks': []}, 1.,
                            (0.,0.,0.), .1, .08, .02)['action'] == 'clear'


def test_camera_near_obstacle_and_missing_observation_require_hold():
    assert camera_hold(None, 1.) == 'camera_observation_unavailable'
    assert camera_hold({'stamp': 0., 'blocked': False}, 1.) == 'camera_observation_unavailable'
    assert camera_hold({'stamp': 1., 'blocked': True}, 1.) == 'camera_obstacle_unranged'
    assert camera_hold({'stamp': 1., 'blocked': False}, 1.) is None
