import numpy as np
import pytest
from move_control.sensing.camera import classify_frame
from move_control.control.obstacle_risk import camera_hold
from move_control.sensing.camera_evidence import legacy_flags


def test_legacy_flags_hold_on_invalid_then_recover_on_valid_clear():
    assert legacy_flags(dict(quality={'valid': True}, cliff=True, blocked=True)) == (True, True)
    assert legacy_flags(classify_frame(None), previous_cliff=True) == (True, True)
    assert legacy_flags(classify_frame(None)) == (False, True)
    assert legacy_flags(classify_frame(scene()), previous_cliff=True) == (False, False)


def scene():
    return np.full((240,320,3), 100, dtype=np.uint8)


@pytest.mark.parametrize('width', [2, 15])
def test_small_near_object_is_not_lost_to_morphology_or_column_average(width):
    frame = scene()
    frame[170:225,155:155+width] = (20,20,220)
    result = classify_frame(frame, floor_hsv=(0.,0.,100.))
    assert result['blocked']
    assert any(r['near_path'] for r in result['regions'])
    assert all(r['distance_m'] is None and r['motion'] == 'unknown' for r in result['regions'])


def test_distant_object_is_reported_without_claiming_near_contact():
    frame = scene()
    frame[50:110,140:170] = (20,20,220)
    result = classify_frame(frame, floor_hsv=(0.,0.,100.))
    assert not result['blocked']
    assert result['regions'][0]['bbox_xyxy'] == [140,50,170,110]


def test_isolated_pixel_noise_does_not_create_an_obstacle():
    frame = scene()
    frame[170:220:5,110:210:5] = 220
    result = classify_frame(frame, floor_hsv=(0.,0.,100.))
    assert not result['blocked']
    assert result['regions'] == []


@pytest.mark.parametrize('frame', [None, np.zeros((240,320,3),dtype=np.uint8), np.full((240,320,3),255,dtype=np.uint8)])
def test_unusable_image_cannot_authorize_clear_motion(frame):
    result = classify_frame(frame)
    assert result['quality']['valid'] is False
    assert camera_hold(dict(stamp=1., blocked=False, quality=result['quality']), 1.) == 'camera_observation_unavailable'


def test_dark_near_region_is_ambiguous_and_cannot_be_treated_as_clear():
    frame = scene()
    frame[170:225,150:165] = 5
    result = classify_frame(frame, floor_hsv=(0.,0.,100.))
    assert result['blocked']
    assert result['regions'][0]['kind'] == 'dark_region'
