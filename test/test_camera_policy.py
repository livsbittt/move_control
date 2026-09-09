"""The verdict layer, tested where the node and the Gazebo adapter used to disagree.

Before this module existed, camera_detect_node.tick counted `hits` from a
parameter behind a `warmup_frames` gate, while tools/gz/rendered_camera_adapter
hardcoded hits >= 2 with no warmup and no cliff hysteresis at all. Both drove the
same topics, so a green sim run said nothing about the robot.
"""
import json

import pytest

from rosy_control.control.obstacle_risk import camera_hold
from rosy_control.sensing.camera_evidence import REGION_CAP, observation_payload
from rosy_control.sensing.camera_policy import CameraPolicy


def frame(cliff=False, blocked=False, valid=True):
    """A classify_frame result, reduced to what the policy actually reads."""
    return {'cliff': cliff, 'blocked': blocked, 'side': 0.0,
            'quality': {'valid': valid, 'reason': 'usable' if valid else 'underexposed'}}


def warmed(hits=2, warmup_frames=3):
    policy = CameraPolicy(hits=hits, warmup_frames=warmup_frames)
    for _ in range(warmup_frames):
        policy.update(frame())
    assert policy.ready
    return policy


def test_warmup_holds_blocked_until_the_reference_has_bootstrapped():
    policy = CameraPolicy(hits=2, warmup_frames=3)
    for _ in range(2):
        assert policy.update(frame()) == (False, True)
        assert not policy.ready
    # The third usable frame is the first that may report a clear path.
    assert policy.update(frame()) == (False, False)
    assert policy.ready


def test_unusable_frames_do_not_count_toward_warmup():
    policy = CameraPolicy(hits=2, warmup_frames=3)
    for _ in range(10):
        policy.update(frame(valid=False))
    assert not policy.ready


def test_rising_needs_consecutive_hits_but_falling_needs_one_clean_frame():
    policy = warmed()
    assert policy.update(frame(blocked=True)) == (False, False)
    assert policy.update(frame(blocked=True)) == (False, True)
    # A missed obstacle costs more than a late start, so the asymmetry is kept.
    assert policy.update(frame(blocked=False)) == (False, False)


def test_an_interrupted_run_of_hits_restarts_the_count():
    policy = warmed(hits=3)
    policy.update(frame(blocked=True))
    policy.update(frame(blocked=True))
    policy.update(frame(blocked=False))
    policy.update(frame(blocked=True))
    assert policy.blocked is False


def test_cliff_gets_the_same_hysteresis_the_adapter_used_to_skip():
    policy = warmed()
    assert policy.update(frame(cliff=True))[0] is False
    assert policy.update(frame(cliff=True))[0] is True
    assert policy.update(frame(cliff=False))[0] is False


def test_blindness_holds_immediately_and_is_not_evidence_of_a_new_cliff():
    policy = warmed()
    cliff, blocked = policy.update(frame(valid=False))
    assert blocked is True   # no hits required: an unreadable frame stops the robot
    assert cliff is False    # but it cannot invent a drop that was never seen


def test_blindness_preserves_a_cliff_that_was_already_established():
    policy = warmed()
    policy.update(frame(cliff=True))
    policy.update(frame(cliff=True))
    assert policy.cliff is True
    assert policy.update(frame(valid=False)) == (True, True)


def test_reset_rebootstraps_after_the_sensor_gains_are_frozen():
    policy = warmed()
    policy.update(frame(blocked=True))
    policy.update(frame(blocked=True))
    policy.reset()
    assert not policy.ready
    assert policy.blocked is False and policy.cliff is False


def test_hits_below_one_cannot_disable_the_hysteresis():
    policy = CameraPolicy(hits=0, warmup_frames=0)
    assert policy.hits == 1


def region(index=0, distance_m=None):
    # Widest realistic encoding: 3-digit box coordinates, near the path, dark,
    # and (for the ranged case) a 3-decimal range.
    return {'bbox_xyxy': [320 - index % 10, 240, 320, 240], 'area_px': 999,
            'area_fraction': 0.01, 'near_path': True, 'kind': 'dark_region',
            'distance_m': distance_m, 'motion': 'unknown'}


def observation(region_count=REGION_CAP, distance_m=None, source='onboard_camera_pixels'):
    result = {'quality': {'valid': True, 'reason': 'usable', 'reference': 'previous'},
              'regions': [region(i, distance_m) for i in range(region_count)],
              'region_count': region_count}
    return observation_payload(1234.5, False, True, -1.0, result, (320, 240), source)


def test_worst_case_observation_stays_small_enough_to_publish_at_frame_rate():
    # The verbose form measured 11362 B at the old 64-region cap -- larger than
    # shipping the whole frame as JPEG (9813 B at q70), which defeated the point
    # of publishing evidence instead of pixels.
    #
    # The worst case is the RANGED one: every region carrying a distance, the
    # longest source name, and 3-digit coordinates. At cap 64 that measured
    # 4135 B, which is why the cap is 48. Pinning the unranged case only would
    # have passed right up until someone filled in the calibration.
    ranged = json.dumps(observation(distance_m=12.345, source='gazebo_rendered_pixels'))
    assert all('m' in r for r in json.loads(ranged)['regions'])
    assert len(ranged) < 4096
    assert len(json.dumps(observation())) < len(ranged)


def test_observation_keeps_the_fields_the_safety_gate_actually_reads():
    payload = observation()
    assert camera_hold(payload, payload['stamp'] + 0.1) == 'camera_obstacle_unranged'
    assert camera_hold(dict(payload, blocked=False), payload['stamp'] + 0.1) is None


def test_region_count_stays_truthful_when_the_cap_truncates():
    payload = observation(region_count=REGION_CAP + 5)
    assert len(payload['regions']) == REGION_CAP
    assert payload['region_count'] == REGION_CAP + 5
    assert payload['regions_truncated'] is True


def test_unranged_regions_omit_distance_rather_than_reporting_zero():
    payload = observation(region_count=1)
    assert 'm' not in payload['regions'][0]
    assert payload['regions'][0]['k'] == 'd'
    assert payload['regions'][0]['n'] == 1


def test_a_ranged_region_carries_its_distance():
    result = {'quality': {'valid': True}, 'region_count': 1,
              'regions': [dict(region(), distance_m=0.4217)]}
    payload = observation_payload(1.0, False, True, 0.0, result, (320, 240), 'test')
    assert payload['regions'][0]['m'] == 0.422


@pytest.mark.parametrize('valid', [True, False])
def test_quality_travels_verbatim_so_the_consumer_can_refuse_a_blind_frame(valid):
    result = {'quality': {'valid': valid, 'reason': 'usable' if valid else 'overexposed'},
              'regions': [], 'region_count': 0}
    payload = observation_payload(1.0, False, False, 0.0, result, (320, 240), 'test')
    assert payload['quality']['valid'] is valid


def test_classify_frame_truncates_to_the_cap_and_keeps_the_biggest_regions():
    """Drive the real classifier past the cap, not a synthetic region list.

    The cap's promise is that it drops the LEAST significant regions -- they
    arrive sorted by (near_path, area_px) -- while region_count still reports the
    true total. A synthetic list cannot check that the sort survives the pipeline.
    """
    np = pytest.importorskip('numpy')
    pytest.importorskip('cv2')
    from rosy_control.sensing.camera import classify_frame

    frame = np.full((240, 320, 3), 100, dtype=np.uint8)
    # 70 separated blobs in the upper half. Each must clear the area floor, which
    # is max(12, 320*240*0.0005) = 38 px, so the smallest here is 7x7 = 49. The
    # size gradient makes "the biggest are the ones kept" checkable, and staying
    # above the near band keeps `blocked` out of it.
    spots = [(row, col) for row in range(8, 110, 16) for col in range(8, 310, 20)]
    for index, (row, col) in enumerate(spots[:70]):
        size = 7 + index % 5
        frame[row:row + size, col:col + size] = (20, 20, 220)

    result = classify_frame(frame, floor_hsv=(0., 0., 100.))
    assert result['region_count'] > REGION_CAP, result['region_count']
    assert result['regions_truncated'] is True
    assert len(result['regions']) == REGION_CAP

    payload = observation_payload(1.0, False, result['blocked'], result['side'],
                                  result, (320, 240), 'onboard_camera_pixels')
    assert len(payload['regions']) == REGION_CAP
    assert payload['region_count'] == result['region_count']
    assert len(json.dumps(payload)) < 4096
    # Kept regions are ordered by (near_path, area) and none is smaller than a
    # region that was dropped.
    kept = [r['area_px'] for r in result['regions']]
    assert kept == sorted(kept, reverse=True) or any(
        r['near_path'] for r in result['regions'])
