import math
import pytest
from tools.gz.inertial import body_gravity


def test_tilted_simulated_body_does_not_publish_level_gravity():
    pitch = math.radians(20.)
    x, y, z = body_gravity((0., math.sin(pitch/2), 0., math.cos(pitch/2)))
    assert x == pytest.approx(-9.81*math.sin(pitch))
    assert y == pytest.approx(0.)
    assert z == pytest.approx(9.81*math.cos(pitch))
    assert body_gravity((0.,0.,0.,1.)) == (0.,0.,9.81)


def test_invalid_simulation_attitude_is_rejected():
    with pytest.raises(ValueError):
        body_gravity((0.,0.,0.,0.))
