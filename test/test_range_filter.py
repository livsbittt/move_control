from rosy_control.sensing.range_filter import CalibrationRangeFilter
from rosy_control.control.calibration import StationaryBaseline


def test_isolated_spike_is_rejected_but_sustained_change_is_preserved():
    f = CalibrationRangeFilter()
    for i in range(5):
        f.update(.65, i*.05)
    assert f.update(1.1, .25) == .65
    for i in range(3):
        result = f.update(.2, .3+i*.05)
    assert result == .2


def test_stale_or_invalid_window_cannot_reuse_previous_good_reading():
    f = CalibrationRangeFilter()
    f.update(.65, 0.)
    assert f.update(.1, 1.) == .1
    f.update(float('nan'), 1.1, False)
    assert f.update(.2, 1.2) == .2


def test_baseline_tolerates_one_spike_but_rejects_persistent_range_change():
    values = [(.65,)]*49 + [(1.1,)]
    assert StationaryBaseline._quality('lidar', values)[0] == 'ok'
    assert StationaryBaseline._quality('lidar', [(.65,)]*25+[(1.1,)]*25)[0] == 'unstable'
