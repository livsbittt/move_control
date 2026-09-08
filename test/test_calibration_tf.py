import math
import pytest
from move_control.control.calibration_tf import transform_health, map_motion_continuous


@pytest.mark.parametrize('age,reason', [(0.1,'ok'),(1.01,'stale'),(-.76,'future')])
def test_transform_source_age_is_not_renewed_by_lookup(age, reason):
    assert transform_health(10.,10.-age,(0.,0.,0.),(0.,0.,0.,1.))['reason'] == reason


def test_invalid_geometry_cannot_be_retried_as_a_transport_gap():
    assert transform_health(10.,10.,(math.nan,0.,0.),(0.,0.,0.,1.))['reason'] == 'invalid_geometry'
    assert transform_health(10.,10.,(0.,0.,0.),(0.,0.,0.,0.))['reason'] == 'invalid_geometry'


def test_recovered_map_pose_must_agree_with_wheel_motion():
    evidence=dict(forward_m=.01,lateral_m=0.,yaw_drift_rad=0.,map_forward_m=.01,map_lateral_m=0.,map_yaw_drift_rad=0.)
    assert map_motion_continuous(evidence)
    for key in ('map_forward_m','map_lateral_m','map_yaw_drift_rad'):
        assert not map_motion_continuous(dict(evidence,**{key:.2}))
