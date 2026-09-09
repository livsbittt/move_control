"""Freezing the sensor is the precondition for every colour-reference method.

Measured before the lock existed: with the floor reference frozen, a 1.3x gain
step on one unchanged real frame flipped blocked False -> True, and a per-channel
white-balance shift drove near-floor 1.00 -> 0.00. So a lock that silently fails
open is worse than no lock -- it hides the drift instead of reporting it.
"""
import pytest

from rosy_control.sensing.camera_controls import (
    COLOUR_GAIN_MAX, LOCK_MAX_ATTEMPTS, LOCKED_KEYS, lock_action, lock_controls,
    lock_summary, static_controls)


def settled(**overrides):
    """Metadata shaped like a real capture_metadata() after AE/AWB converged."""
    return dict({'ExposureTime': 19999, 'AnalogueGain': 2.5,
                 'ColourGains': (1.8, 1.6), 'Lux': 120.0}, **overrides)


def test_lock_emits_only_the_three_implicit_disable_controls():
    controls = lock_controls(settled())
    assert set(controls) == set(LOCKED_KEYS)
    assert controls == {'ExposureTime': 19999, 'AnalogueGain': 2.5,
                        'ColourGains': (1.8, 1.6)}


def test_lock_never_emits_the_explicit_enable_flags():
    # Setting ExposureTime+AnalogueGain disables AE and ColourGains disables AWB
    # implicitly. Sending AeEnable/AwbEnable as well has reported sequencing bugs.
    controls = lock_controls(settled())
    assert 'AeEnable' not in controls and 'AwbEnable' not in controls


@pytest.mark.parametrize('missing', LOCKED_KEYS)
def test_missing_metadata_stays_in_auto(missing):
    metadata = settled()
    del metadata[missing]
    assert lock_controls(metadata) is None


@pytest.mark.parametrize('metadata', [
    settled(ExposureTime=0),
    settled(ExposureTime=-1),
    settled(AnalogueGain=0.0),
    settled(AnalogueGain=-2.0),
    settled(AnalogueGain=float('nan')),
    settled(AnalogueGain=float('inf')),
    settled(ColourGains=(0.0, 1.6)),
    settled(ColourGains=(1.8, 0.0)),
    settled(ColourGains=(COLOUR_GAIN_MAX + 0.1, 1.6)),
    settled(ColourGains=(1.8, float('nan'))),
    settled(ColourGains=(1.8,)),
    settled(ColourGains=1.8),
    settled(ExposureTime='19999us'),
    settled(AnalogueGain=True),
    # bools survive float(), so an unguarded True would become a unity gain.
    settled(ExposureTime=True),
    settled(ColourGains=(True, 1.6)),
    settled(ColourGains=(1.8, True)),
    None,
    # int(float('inf')) raises OverflowError, which is neither TypeError nor
    # ValueError: without an explicit finiteness check first, an unreadable
    # exposure escaped as an exception instead of as "stay in auto".
    settled(ExposureTime=float('inf')),
    settled(ExposureTime=float('-inf')),
    settled(ExposureTime=float('nan')),
    # An hour of exposure is a bad metadata read, not a dim room.
    settled(ExposureTime=10 ** 30),
])
def test_unusable_metadata_cannot_freeze_the_sensor(metadata):
    assert lock_controls(metadata) is None


def test_colour_gain_ceiling_is_inclusive():
    controls = lock_controls(settled(ColourGains=(COLOUR_GAIN_MAX, COLOUR_GAIN_MAX)))
    assert controls['ColourGains'] == (COLOUR_GAIN_MAX, COLOUR_GAIN_MAX)


def test_static_controls_pin_the_frame_period_to_the_requested_rate():
    controls = static_controls(8.0)
    assert controls['FrameDurationLimits'] == (125000, 125000)
    # Denoise stays Fast (HighQuality is documented to cost framerate) and
    # sharpening stays off so it cannot invent floor/obstacle boundary edges.
    assert controls['NoiseReductionMode'] == 1
    assert controls['Sharpness'] == 0.0


@pytest.mark.parametrize('fps', [0, -8.0, float('nan'), float('inf'), None, 'fast'])
def test_static_controls_reject_an_unusable_rate(fps):
    assert static_controls(fps) is None


def test_lock_waits_until_the_isp_has_settled():
    assert lock_action(True, now=1.0, deadline=3.5, attempts=0) == 'wait'
    assert lock_action(True, now=3.5, deadline=3.5, attempts=0) == 'attempt'
    assert lock_action(True, now=9.0, deadline=3.5, attempts=0) == 'attempt'


def test_lock_disabled_by_parameter_short_circuits_every_other_condition():
    assert lock_action(False, now=9.0, deadline=3.5, attempts=0) == 'disabled'
    assert lock_action(False, now=0.0, deadline=None, attempts=99) == 'disabled'


def test_lock_waits_when_there_is_no_deadline_to_settle_against():
    assert lock_action(True, now=9.0, deadline=None, attempts=0) == 'wait'


def test_repeated_failures_give_up_rather_than_freeze_on_a_bad_read():
    for attempts in range(LOCK_MAX_ATTEMPTS):
        assert lock_action(True, now=9.0, deadline=3.5, attempts=attempts) == 'attempt'
    assert lock_action(True, now=9.0, deadline=3.5,
                       attempts=LOCK_MAX_ATTEMPTS) == 'exhausted'


@pytest.mark.parametrize('now,deadline', [
    (float('nan'), 3.5), (3.5, float('nan')), ('soon', 3.5), (3.5, 'soon')])
def test_an_unusable_clock_waits_instead_of_locking(now, deadline):
    assert lock_action(True, now=now, deadline=deadline, attempts=0) == 'wait'


def test_summary_records_what_the_camera_was_frozen_at():
    assert lock_summary(None) == 'auto'
    assert lock_summary(lock_controls(settled())) == (
        'exposure=19999us gain=2.500 colour_gains=1.800,1.600')
