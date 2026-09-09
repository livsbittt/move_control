"""Exercise safety callbacks without starting ROS nodes or publishing to a robot."""
from types import SimpleNamespace
import json
import pytest
pytest.importorskip('geometry_msgs')
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rosy_control.safety.gate import Gate
from rosy_control.safety.node import SafetyNode


def fake_gate():
    command = Twist()
    command.linear.x = .01
    return SimpleNamespace(estop=False, last_cmd=command, last_cmd_time=object(),
        localization_status={'ready': True, 'stamp_ns': 1000000000},
        get_parameter=lambda _: SimpleNamespace(value=True),
        now=lambda: SimpleNamespace(nanoseconds=1200000000), _publish_zero=lambda: None)


def test_queued_loss_and_recovery_cannot_replay_pre_loss_command():
    gate = fake_gate()
    SafetyNode.on_localization(gate, String(data=json.dumps({'ready': False, 'stamp_ns': 1100000000})))
    SafetyNode.on_localization(gate, String(data=json.dumps({'ready': True, 'stamp_ns': 1200000000})))
    assert gate.last_cmd.linear.x == 0.
    assert gate.last_cmd_time is None


def test_command_received_while_lost_is_not_cached_for_recovery():
    gate = fake_gate()
    gate.localization_status = None
    msg = Twist()
    msg.linear.x = .01
    Gate.on_cmd(gate, msg)
    assert gate.last_cmd.linear.x == 0.
    assert gate.last_cmd_time is None
