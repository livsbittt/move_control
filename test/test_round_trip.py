from move_control.control.round_trip import RoundTrip


def snapshot(x):
    return {'odom': (x, 0., 0., 0.), 'map_tf': (x, 0., 0.), 'lidar': (.65-x,), 'us': (.65-x,)}


def test_measures_corrects_repeats_and_returns_home():
    trip = RoundTrip(0., snapshot(0.))
    x = 0.
    speed = 0.
    for i in range(1, 701):
        x += speed * .05 * .92
        speed = trip.update(i*.05, snapshot(x))
        if trip.done or trip.error:
            break
    assert trip.error is None, trip.error
    assert trip.done
    assert len(trip.legs) == 4
    assert abs(x) < .006
    assert abs(trip.scales[0] - 1/.92) < .02
    assert abs(trip.scales[1] - 1/.92) < .02
    assert speed == 0.


def test_stall_never_publishes_ready_and_stops():
    trip = RoundTrip(0., snapshot(0.))
    for i in range(1, 150):
        speed = trip.update(i*.05, snapshot(0.))
    assert trip.error and not trip.done and speed == 0.


def test_pause_in_controller_aborts_instead_of_reusing_command():
    trip = RoundTrip(0., snapshot(0.))
    assert trip.update(1., snapshot(0.)) == 0.
    assert trip.error


def test_explicit_zero_pause_does_not_integrate_motion_while_waiting():
    trip = RoundTrip(0., snapshot(0.))
    trip.update(.05, snapshot(0.))
    trip.pause(.10)
    issued = trip.commanded
    for now in (.15, .20, .25):
        assert trip.pause(now) == 0.
    assert trip.commanded == issued
    assert trip.last_speed == 0.
    assert trip.update(.30, snapshot(.0004)) > 0.
    assert trip.error is None


def test_failed_gain_preserves_measurements_without_relaxing_bounds():
    trip = RoundTrip(0., snapshot(0.))
    trip.stage = 'settle_out'
    trip.settle_until = 0.
    trip.commanded = .045
    assert trip.update(.1, snapshot(.03)) == 0.
    assert trip.error == 'Required speed correction exceeds calibrated bounds'
    evidence = trip.report()['last_leg_evidence']
    assert abs(evidence['ratio'] - 1.5) < 1e-9
    assert evidence['commanded_m'] == .045
    assert not trip.done and trip.scales == [1., 1.]
