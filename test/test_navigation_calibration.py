from rosy_control.control.navigation_calibration import environment_profile, map_ray
from rosy_control.planning import OccupancyMap


def samples(noise=0., left=.14, right=.13):
    return [(left+(noise if i%2 else 0),right,.10,.10) for i in range(40)]


def test_environment_noise_increases_margin_without_changing_robot_size():
    quiet=environment_profile(.076,.02,samples())
    noisy=environment_profile(.076,.02,samples(.004))
    assert quiet['body_radius_m']==noisy['body_radius_m']==.076
    assert abs(quiet['minimum_clearance_m']-.096)<1e-9
    assert noisy['minimum_clearance_m']>quiet['minimum_clearance_m']
    assert quiet['map_lidar_mismatch']
    assert quiet['passage_fits']


def test_narrow_scene_does_not_reduce_hard_floor_and_high_noise_is_rejected():
    narrow=environment_profile(.076,.02,samples(left=.08,right=.08))
    assert not narrow['passage_fits'] and narrow['minimum_clearance_m']>=.096
    assert environment_profile(.076,.02,samples(.10)) is None
    assert environment_profile(.076,.02,[]) is None


def test_map_ray_does_not_invent_wall_at_unknown_boundary():
    m=OccupancyMap(20,20,.02,fill=0)
    m.set_cell(10,5,100)
    assert .08 < map_ray(m,m.grid_to_world(5,5),0) < .11
    m.set_cell(8,5,-1)
    assert map_ray(m,m.grid_to_world(5,5),0) is None
