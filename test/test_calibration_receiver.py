"""Validate calibration relay readiness without a ROS dependency."""
import ast
from pathlib import Path
from types import SimpleNamespace


def available(received, publisher):
    path=Path(__file__).parents[1]/'rosy_control/web_node.py'
    tree=ast.parse(path.read_text(encoding='utf-8'))
    method=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='calibration_receiver_available')
    scope={}
    exec(compile(ast.Module(body=[method],type_ignores=[]),str(path),'exec'),scope)
    return scope['calibration_receiver_available'](received,100.,publisher)


def test_missing_stale_or_future_heartbeat_is_not_an_available_receiver():
    publisher=SimpleNamespace(get_subscription_count=lambda:1)
    for received in (None,96.,101.):
        assert not available(received,publisher)
    assert available(99.,publisher)


def test_discovery_must_confirm_subscriber_when_supported():
    for count in (0,False,None):
        assert not available(99.,SimpleNamespace(get_subscription_count=lambda:count))
    assert available(99.,SimpleNamespace(get_subscription_count=lambda:1))
    assert available(99.,SimpleNamespace())
