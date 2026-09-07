import math
from move_control.sensing.pose import planar_pose


def test_map_pose_requires_finite_valid_quaternion_and_position():
    assert planar_pose(1., 2., (0., 0., 0., 1.)) == (1., 2., 0.)
    for q in ((0., 0., 0., 0.), (0., 0., 0., .1), (0., math.nan, 0., 1.)):
        assert planar_pose(1., 2., q) is None
    assert planar_pose(math.inf, 2., (0., 0., 0., 1.)) is None
