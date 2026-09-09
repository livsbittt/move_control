"""Exercise the combined escape/obstacle path without a ROS node or motors."""
from types import SimpleNamespace
from unittest.mock import Mock
from pathlib import Path
import ast
import math
import os

import pytest

from rosy_control.control.straight_escape import StraightEscape
from rosy_control.sensing.pose import planar_pose


def tick_navigation(node):
    # Execute the production callback with message constructors replaced only;
    # no ROS import, discovery or publisher is needed for this integration seam.
    source = Path(__file__).parents[1] / 'rosy_control/wander/navigator.py'
    tree = ast.parse(source.read_text(encoding='utf-8'))
    owner = next(item for item in tree.body if isinstance(item, ast.ClassDef)
                 and item.name == 'Navigator')
    method = next(item for item in owner.body if isinstance(item, ast.FunctionDef)
                  and item.name == '_tick_navigation')
    namespace = dict(math=math, os=os, Time=lambda: None, TransformException=RuntimeError,
        planar_pose=planar_pose, hazard_action=lambda *args: 'none',
        Twist=lambda: SimpleNamespace(linear=SimpleNamespace(x=0.),
                                      angular=SimpleNamespace(z=0.)))
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'), namespace)
    namespace['_tick_navigation'](node)


@pytest.mark.parametrize('direction', [-1., 1.])
def test_obstacle_aborts_escape_and_clearing_it_cannot_restart(direction, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '227')
    monkeypatch.setenv('GZ_PARTITION', 'pinky_calmap227')
    tf = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0)),
        transform=SimpleNamespace(translation=SimpleNamespace(x=0., y=0.),
            rotation=SimpleNamespace(x=0., y=0., z=0., w=1.)))
    node = SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=1000000000),
        navigation_tf=Mock(), tilt=False, cliff=False, seen_forward=True,
        _can_reverse=lambda: True, estop=False, pickup=False, _ir_ready=lambda: True,
        navigation_received=1., navigation_stamp=1.,
        straight_escape_intent=dict(id='escape', target=[direction*.05, 0.],
                                    yaw=0., geometry='trusted', issued_s=1.),
        straight_escape=StraightEscape(), straight_escape_seen=1.,
        motion_limits=dict(bounded_motion_enabled=True, bounded_geometry='trusted_footprint',
                           geometry_revision='trusted'),
        _odom_fresh=lambda: True, _motion_limits_fresh=lambda: True,
        odom_x=0., odom_y=0., odom_yaw=0., navigation_mode='explore',
        _publish=Mock(), _obstacle_wait=Mock(return_value=None))
    node.navigation_tf.lookup_transform.return_value = tf
    tick_navigation(node)
    assert node._publish.call_args.args[0].linear.x * direction > 0.
    node._obstacle_wait.return_value = 'wait:predicted_obstacle'
    tick_navigation(node)
    command, reason = node._publish.call_args.args
    assert command.linear.x == command.angular.z == 0.
    assert 'predicted_obstacle' in reason
    assert node.straight_escape.failed
    node._obstacle_wait.return_value = None
    tick_navigation(node)
    assert node._publish.call_args.args[0].linear.x == 0.
