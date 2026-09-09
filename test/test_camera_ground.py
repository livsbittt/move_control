"""Ground-plane geometry, checked against values worked out by hand.

The calibration below is the simulated rig's, so the numbers are traceable:
tools/gz/obstacle_camera.py mounts the sensor at pose '.055 0 .08 0 .25 0' --
0.08 m up, pitched 0.25 rad down -- and renders 320x240 with a 1.2 rad
horizontal FOV.
"""
import math

import pytest

from rosy_control.sensing.camera_ground import focal_from_hfov, ground_plane

HEIGHT, PITCH = 0.08, 0.25
WIDTH, HEIGHT_PX, HFOV = 320, 240, 1.2
FOCAL = (WIDTH / 2) / math.tan(HFOV / 2)          # 233.87 px
CX, CY = WIDTH / 2, HEIGHT_PX / 2
MAX_RANGE = 3.0


def plane(**overrides):
    calibration = dict(height_m=HEIGHT, pitch_rad=PITCH, focal_px=FOCAL,
                       principal_x=CX, principal_y=CY, max_range_m=MAX_RANGE)
    calibration.update(overrides)
    return ground_plane(**calibration)


def hand_computed(row):
    """Z = h / tan(pitch + atan((v - cy) / f)) -- the derivation, spelled out."""
    return HEIGHT / math.tan(PITCH + math.atan((row - CY) / FOCAL))


def test_focal_from_hfov_matches_the_rendered_camera():
    assert focal_from_hfov(WIDTH, HFOV) == pytest.approx(233.87, abs=0.01)


def test_the_principal_row_is_the_pure_pitch_ray():
    # At v == cy the atan term vanishes, so Z is h / tan(pitch) exactly.
    assert plane().distance(CY) == pytest.approx(HEIGHT / math.tan(PITCH))
    assert plane().distance(CY) == pytest.approx(0.3133, abs=1e-4)


def test_rows_lower_in_the_image_are_nearer():
    p = plane()
    distances = [p.distance(row) for row in (140, 180, 220, 239)]
    assert all(d is not None for d in distances)
    assert distances == sorted(distances, reverse=True)
    assert distances[-1] == pytest.approx(hand_computed(239), rel=1e-9)


def test_the_bottom_row_matches_the_hand_computed_range():
    assert plane().distance(239) == pytest.approx(0.09109, abs=1e-5)


def test_the_horizon_row_is_where_the_ray_stops_meeting_the_floor():
    p = plane()
    horizon = p.horizon_row
    assert horizon == pytest.approx(CY - FOCAL * math.tan(PITCH), rel=1e-12)
    assert horizon == pytest.approx(60.28, abs=0.01)
    assert p.distance(horizon) is None
    assert p.distance(horizon - 1) is None
    assert p.distance(0) is None
    # Rows just below the horizon are still refused, but by the range clip
    # rather than the horizon: at horizon + 5 the geometry says 3.97 m, past the
    # 3 m a 240-row segmentation can support. Well down the frame it ranges.
    assert p.distance(horizon + 5) is None
    assert p.distance(200) == pytest.approx(0.12221, abs=1e-5)


def test_beyond_the_trustworthy_range_is_unknown_not_free():
    # Just below the horizon the inverse tangent blows up: one row of
    # segmentation jitter there is metres of error, so it is refused outright.
    near_horizon = plane().horizon_row + 0.5
    assert hand_computed(near_horizon) > MAX_RANGE
    assert plane().distance(near_horizon) is None
    # Widening the clip is the only thing that admits it -- nothing is silently free.
    assert plane(max_range_m=50.0).distance(near_horizon) is not None


def hand_computed_lateral(column, row):
    """X = h(u - cx) / (f*sin(pitch) + (v - cy)*cos(pitch)).

    The naive Z*(u - cx)/f is the zero-pitch approximation and under-reads by
    4.2% at row 130 rising to 15.7% at row 239 on this rig, so the sign-and-
    symmetry checks below are not on their own enough to pin the formula.
    """
    return HEIGHT * (column - CX) / (FOCAL * math.sin(PITCH) +
                                     (row - CY) * math.cos(PITCH))


@pytest.mark.parametrize('row,expected', [
    (130, 0.07106), (160, 0.04968), (200, 0.03546), (239, 0.02772)])
def test_lateral_matches_the_pitched_geometry_not_the_flat_approximation(row, expected):
    p = plane()
    assert p.lateral(CX + 60, row) == pytest.approx(expected, abs=1e-5)
    assert p.lateral(CX + 60, row) == pytest.approx(hand_computed_lateral(CX + 60, row))
    # The approximation this replaced would have been noticeably smaller.
    naive = p.distance(row) * 60 / FOCAL
    assert naive < p.lateral(CX + 60, row)


def test_lateral_offset_is_signed_about_the_optical_axis():
    p = plane()
    assert p.lateral(CX, 200) == pytest.approx(0.0)
    assert p.lateral(CX + 40, 200) > 0
    assert p.lateral(CX - 40, 200) == pytest.approx(-p.lateral(CX + 40, 200))


@pytest.mark.parametrize('row', [0, 30, 'row', None, float('nan')])
def test_lateral_is_unknown_wherever_the_distance_is(row):
    assert plane().lateral(CX + 40, row) is None


def test_a_region_is_ranged_at_its_bottom_edge_not_its_centroid():
    p = plane()
    tall = [100, 80, 140, 230]          # a tall object: centroid row 155, base 230
    assert p.region_distance(tall) == pytest.approx(p.distance(230))
    assert p.region_distance(tall) != pytest.approx(p.distance(155))
    # The centroid would overestimate the range -- an error toward collision.
    assert p.distance(155) > p.distance(230)


@pytest.mark.parametrize('bbox', [None, [], [1, 2], 'bbox', {'y': 3}])
def test_a_malformed_bbox_is_unranged_rather_than_guessed(bbox):
    assert plane().region_distance(bbox) is None


@pytest.mark.parametrize('row', [None, float('nan'), float('inf'), 'bottom'])
def test_an_unusable_row_is_unranged(row):
    assert plane().distance(row) is None


@pytest.mark.parametrize('overrides', [
    {'height_m': 0.0}, {'height_m': -0.08}, {'focal_px': 0.0}, {'focal_px': -231.0},
    {'max_range_m': 0.0}, {'pitch_rad': 0.0}, {'pitch_rad': -0.25},
    {'pitch_rad': math.pi / 2}, {'height_m': float('nan')},
    {'pitch_rad': float('inf')}, {'focal_px': None}, {'principal_y': 'centre'},
])
def test_an_uncalibrated_camera_produces_no_plane_at_all(overrides):
    assert plane(**overrides) is None


def test_absent_calibration_is_not_an_error_it_is_simply_unranged():
    assert ground_plane() is None


def test_classify_frame_reports_unranged_until_a_calibration_is_supplied():
    """End to end: the pipeline stays honest without a measured calibration."""
    np = pytest.importorskip('numpy')
    pytest.importorskip('cv2')
    from rosy_control.sensing.camera import classify_frame

    frame = np.full((240, 320, 3), 100, dtype=np.uint8)
    frame[170:225, 155:170] = (20, 20, 220)

    uncalibrated = classify_frame(frame, floor_hsv=(0., 0., 100.))
    assert uncalibrated['regions']
    assert all(r['distance_m'] is None for r in uncalibrated['regions'])

    calibrated = classify_frame(frame, floor_hsv=(0., 0., 100.), ground=plane())
    ranged = [r for r in calibrated['regions'] if r['distance_m'] is not None]
    assert ranged, 'a calibrated plane must range at least the near region'
    for region in ranged:
        # Measured at the bottom edge, and never past the trustworthy range.
        assert region['distance_m'] == pytest.approx(plane().distance(region['bbox_xyxy'][3]))
        assert 0 < region['distance_m'] <= MAX_RANGE
