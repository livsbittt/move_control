"""Exercise the adapter's pure frame normalization without importing ROS."""
import ast
import copy
from pathlib import Path
from types import SimpleNamespace
import pytest


def normalizer():
    source = Path(__file__).parents[1] / 'tools/gz/calibration_mapping_rig.py'
    functions = [node for node in ast.parse(source.read_text()).body
                 if isinstance(node, ast.FunctionDef) and node.name == 'normalize_odometry']
    assert functions, 'Adapter must normalize frame identity before publishing odometry'
    namespace = {'copy': copy}
    exec(compile(ast.Module(body=functions, type_ignores=[]), str(source), 'exec'), namespace)
    return namespace['normalize_odometry']


def test_confirmed_gazebo_model_frames_are_copied_to_canonical_identity():
    message = SimpleNamespace(header=SimpleNamespace(frame_id='pinky/odom', stamp=123),
                              child_frame_id='pinky/base_footprint', pose={'x': .3})
    result = normalizer()(message)
    assert result.header.frame_id == 'odom' and result.child_frame_id == 'base_link'
    assert result.header.stamp == 123 and result.pose == message.pose
    assert message.header.frame_id == 'pinky/odom'
    assert result.pose is not message.pose


def test_unknown_source_frame_cannot_be_relabelled_as_odom():
    for parent, child in [('map','pinky/base_footprint'), ('pinky/odom','other/base')]:
        with pytest.raises(ValueError):
            normalizer()(SimpleNamespace(header=SimpleNamespace(frame_id=parent), child_frame_id=child))
