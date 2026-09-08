import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from tools.gz.prepare_track_world import boxes, clearance


def test_exact_track_includes_both_diagonal_collision_walls():
    walls = boxes(ET.parse(Path(__file__).resolve().parents[1]/'map/map_260905.world'))
    assert len(walls) == 16
    assert sum(abs(pose[5]) > .1 for pose, _ in walls) == 2


def test_diagonal_distance_uses_oriented_box_not_bounding_rectangle():
    walls = [([0, 0, 0, 0, 0, math.pi/4], [1., .01, .155])]
    points = np.array([[.3, .3], [.3, -.3]])
    distances = clearance(points, walls)
    assert distances[0] == 0.
    assert abs(distances[1]-(math.sqrt(.18)-.005)) < 1e-9


def test_wheel_hinges_follow_the_cylinder_axles_in_child_frames():
    root = ET.parse(Path(__file__).resolve().parents[1]/'tools/gz/pinky_maze.sdf')
    for joint in root.findall('.//joint'):
        # The joint inherits its rolled child wheel frame, whose cylinder
        # axle is local Z. Local Y would command a nearly vertical hinge.
        assert joint.find('axis/xyz').get('expressed_in') in (None, '')
        assert [float(v) for v in joint.findtext('axis/xyz').split()] == [0., 0., 1.]
