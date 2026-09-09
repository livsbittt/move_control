from rosy_control.sensing.battery import battery_values, battery_snapshot


def test_zero_charge_is_valid_and_nan_never_becomes_an_estimated_percentage():
    assert battery_values(0., 7., float('nan'), True, 2)['percent'] == 0
    result = battery_values(float('nan'), 7.8, -.3, True, 1)
    assert result == {'available': True, 'reason':'live', 'percent':None, 'voltage':7.8, 'current':-.3, 'status':'charging'}


def test_missing_absent_invalid_and_expired_are_distinct():
    assert battery_snapshot({}, None, 1)['reason'] == 'missing'
    assert battery_values(.7, 7.8, 0, False, 0)['reason'] == 'not_present'
    assert not battery_values(2., float('inf'), 0, True, 0)['available']
    assert battery_snapshot({'available':True}, 1., 7.) == {'available':False, 'reason':'stale'}
