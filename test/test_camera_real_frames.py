"""Real OV5647 frames, because uniform synthetic fills hid two real defects.

Every other camera test builds its scene with np.full, which has no texture, no
lens shading, no rolling-shutter skew and no ISP colour cast. Both bugs pinned
here passed that suite green:

  1. A dark red wall above a bright wood floor reported cliff=True while the
     robot sat safely on the desk (near-column void 0.45, one region covering
     69.6% of the frame).
  2. With the floor reference frozen, a 1.30x global gain step on one unchanged
     frame flipped blocked False -> True; at 1.50x the near centre read
     obst=0.94, i.e. "obstacle filling the view" from a pure exposure change.

Defect 1 is fixed by withdrawing the verdict: camera.py no longer draws a cliff
from appearance at all, because a dark wall and a dark hole at the same ground
line are the same image and both appearance tests were measured wrong. The
darkness is still published, and floor IR owns the drop decision.

Defect 2 is NOT a classifier fix and is not claimed as one: the tolerance band
below is what the classifier actually withstands, and camera_controls.py exists
so the sensor never presents it with a swing that large.
"""
import pathlib

import numpy as np
import pytest

from rosy_control.sensing.camera import classify_frame

ARTIFACTS = pathlib.Path(__file__).resolve().parent.parent / 'artifacts' / 'pinky-real-20260907'
THRESHOLDS = dict(void_v_ratio=0.50, obst_frac=0.45)   # config/camera.yaml


def load(name):
    path = ARTIFACTS / f'{name}.jpg'
    if not path.exists():
        pytest.skip(f'recorded frame {path} is not in this checkout')
    cv2 = pytest.importorskip('cv2')
    frame = cv2.imread(str(path))
    if frame is None:
        pytest.skip(f'{path} could not be decoded')
    return frame


def settle(frame, passes=21):
    """Run the reference EMA to convergence, as the node does after warmup."""
    reference = None
    result = None
    for _ in range(passes):
        result = classify_frame(frame, floor_hsv=reference, **THRESHOLDS)
        if result['floor_hsv'] is not None:
            reference = result['floor_hsv']
    return result, reference


def test_dark_wall_over_a_bright_floor_is_not_a_drop():
    """Regression for the measured false cliff.

    A dark red wall above a bright wood floor used to report cliff=True. The
    darkness is real and stays published; what is refused is the verdict drawn
    from it, because the same image is produced by a hole and by a wall.
    """
    result, reference = settle(load('camera-before-wander'))
    assert result['cliff'] is False
    assert result['blocked'] is True                       # the wall is close
    assert reference[2] > 150.0                            # bright wood floor: V ~203
    assert max(c['void'] for c in result['cols']) > 0.20    # darkness still reported
    assert any(r['kind'] == 'dark_region' for r in result['regions'])


def test_the_operational_maze_frame_reads_clear():
    result, _ = settle(load('camera-current'))
    assert result['cliff'] is False
    assert result['blocked'] is False
    # Carpet in the near band is recognised as floor, not as texture noise.
    assert min(c['floor'] for c in result['cols']) > 0.95


def test_the_maze_frame_merges_the_whole_upper_scene_into_one_region():
    """Documents the limitation the region layer has, so it cannot regress silently.

    Wall, people, desks and monitors arrive as a single component. This is why
    regions carry no class and no distance: there is nothing here that could name
    an object, and pretending otherwise downstream would be a fabrication.
    """
    result, _ = settle(load('camera-current'))
    largest = max(result['regions'], key=lambda r: r['area_px'])
    assert largest['area_fraction'] > 0.25
    assert all(r['distance_m'] is None and r['motion'] == 'unknown'
               for r in result['regions'])


@pytest.mark.parametrize('gain', [0.85, 0.925, 1.0, 1.075, 1.15])
def test_a_small_exposure_step_does_not_invent_an_obstacle(gain):
    """The band the classifier tolerates with a frozen reference.

    Measured outside this band, pre-lock: 1.30x -> blocked True (near obst 0.19),
    1.50x -> near obst 0.94, 0.70x -> blocked True, 0.55x -> near obst 0.89. The
    ISP's AGC can swing further than that on its own, which is why
    camera_controls.lock_controls freezes ExposureTime and AnalogueGain rather
    than leaving this to the classifier.
    """
    frame = load('camera-current')
    _, reference = settle(frame)
    stepped = np.clip(frame.astype(np.float32) * gain, 0, 255).astype(np.uint8)
    result = classify_frame(stepped, floor_hsv=reference,
                            allow_floor_update=False, **THRESHOLDS)
    assert result['quality']['valid'] is True
    assert result['blocked'] is False, f'gain {gain}x invented an obstacle'
    assert result['cliff'] is False


def test_the_reference_survives_a_settled_run_without_drifting_onto_the_wall():
    """The bottom ROI is shallow, which Ulrich & Nourbakhsh call out as the weak
    spot of this family of methods. Pin that the wall does not become the floor."""
    frame = load('camera-current')
    _, first = settle(frame, passes=2)
    _, later = settle(frame, passes=60)
    assert abs(first[2] - later[2]) < 20.0
