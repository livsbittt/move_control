import math
import pytest

from move_control.sensing.obstacle_tracks import ObstacleTracker, transform_points
from move_control.sensing.obstacle_tracks import scan_clusters
from move_control.sensing.obstacle_tracks import observed_free


def test_stationary_world_point_survives_robot_translation_and_rotation():
    tracker = ObstacleTracker()
    for i in range(12):
        yaw, x = i*.04, i*.01
        dx, dy = 1.-x, .2
        local = [(math.cos(yaw)*dx+math.sin(yaw)*dy,
                  -math.sin(yaw)*dx+math.cos(yaw)*dy)]
        tracks = tracker.update(transform_points(local, (x, 0., yaw)), i*.1)
    assert len(tracks) == 1
    assert tracks[0]['state'] == 'stationary'
    assert math.hypot(*tracks[0]['velocity']) < 1e-6


def test_moving_track_requires_time_evidence_and_keeps_identity():
    tracker = ObstacleTracker()
    first = tracker.update([(1., 0.)], 0.)[0]
    assert first['state'] == 'unknown'
    for i in range(1, 12):
        track = tracker.update([(1., i*.02)], i*.1)[0]
    assert track['id'] == first['id']
    assert track['state'] == 'moving'
    assert track['velocity'][1] == pytest.approx(.2)


def test_duplicate_and_regressed_time_do_not_refresh_tracks():
    tracker = ObstacleTracker()
    tracker.update([(1., 0.)], 1.)
    tracker.update([(2., 0.)], 1.)
    tracker.update([(2., 0.)], .9)
    tracks = tracker.snapshot(1.2)
    assert len(tracks) == 1
    assert tracks[0]['position'] == (1., 0.)
    assert tracks[0]['age'] == pytest.approx(.2)


def test_lost_track_is_unknown_before_expiry_not_free_space():
    tracker = ObstacleTracker()
    for i in range(10):
        tracker.update([(1., 0.)], i*.1)
    tracks = tracker.update([], 1.)
    assert len(tracks) == 1
    assert tracks[0]['state'] == 'unknown'
    assert tracks[0]['observed'] is False
    assert tracker.snapshot(3.)[0]['state'] == 'unknown'
    tracker.clear_observed_free([tracks[0]['id']])
    assert tracker.snapshot(3.) == []


def test_ambiguous_association_does_not_invent_object_velocity():
    tracker = ObstacleTracker()
    tracker.update([(1., -.03), (1., .03)], 0.)
    tracks = tracker.update([(1., 0.)], .1)
    assert all(t['state'] == 'unknown' for t in tracks)


def test_invalid_observation_does_not_poison_previous_state():
    tracker = ObstacleTracker()
    tracker.update([(1., 0.)], 0.)
    with pytest.raises(ValueError):
        tracker.update([(float('nan'), 0.)], .1)
    assert tracker.snapshot(.1)[0]['position'] == (1., 0.)


def test_scan_clusters_excludes_no_return_and_keeps_extent():
    groups = scan_clusters([float('inf'), .5, .5, .5, float('nan'), 1., 1., 1.], -.04, .01, .05, 8.)
    assert len(groups) == 2
    assert all(g['radius'] > 0 for g in groups)
    assert groups[0]['position'][0] == pytest.approx(.5, abs=.001)


def test_retirement_requires_finite_rays_beyond_entire_previous_obstacle():
    assert observed_free((.5, 0.), .03, [1.]*21, -.1, .01)
    assert not observed_free((.5, 0.), .03, [float('inf')]*21, -.1, .01)
    assert not observed_free((.5, 0.), .03, [.4]*21, -.1, .01)
    assert not observed_free((.5, 0.), .03, [1.]*3, -.01, .01)


def test_return_to_start_does_not_make_moving_object_stationary():
    tracker = ObstacleTracker()
    for i, y in enumerate((0., .04, .08, .12, .08, .04, 0.)):
        tracks = tracker.update([(1., y)], i*.1)
    assert tracks[0]['state'] != 'stationary'


def test_distinct_nearby_objects_keep_ids_when_nearest_match_is_unambiguous():
    tracker = ObstacleTracker()
    first = tracker.update([(1.,0.), (1.,.12)], 0.)
    for i in range(1, 10):
        tracks = tracker.update([(1.+i*.002,0.), (1.,.12)], i*.1)
    assert {t['id'] for t in tracks} == {t['id'] for t in first}


def test_reappearance_after_long_occlusion_restarts_evidence_without_ghost():
    tracker = ObstacleTracker()
    original = tracker.update([(1., 0.)], 0.)[0]['id']
    tracker.update([], 2.)
    tracks = tracker.update([(1.01, 0.)], 3.)
    assert len(tracks) == 1
    assert tracks[0]['id'] == original
    assert tracks[0]['state'] == 'unknown'
    for stamp in (3.2, 3.4, 3.6, 3.8):
        tracks = tracker.update([(1.01, 0.)], stamp)
    assert tracks[0]['state'] == 'stationary'


def test_repeated_ambiguous_merge_keeps_bounded_unknown_hypotheses():
    tracker = ObstacleTracker()
    tracker.update([(1.,-.03), (1.,.03)], 0.)
    for i in range(1, 301):
        tracks = tracker.update([(1.,0.)], i*.1)
    assert len(tracks) <= 3
    assert all(t['state'] == 'unknown' for t in tracks)


def test_repeated_ambiguous_split_keeps_one_unknown_group_per_return():
    tracker = ObstacleTracker()
    tracker.update([(1.,-.03), (1.,.03)], 0.)
    tracker.update([(1.,0.)], .1)
    for i in range(2, 152):
        tracks = tracker.update([(1.,-.01), (1.,.01)], i*.1)
    assert len(tracks) <= 4
    assert all(t['state'] == 'unknown' for t in tracks)


def test_tilted_scan_cannot_be_projected_into_a_flat_obstacle_map():
    from move_control.sensing.obstacle_tracks import scan_plane_pose
    import math
    angle = math.radians(20.)
    assert scan_plane_pose(0.,0.,(0.,math.sin(angle/2),0.,math.cos(angle/2))) is None
    assert scan_plane_pose(0.,0.,(0.,0.,math.sin(angle/2),math.cos(angle/2))) is not None
