"""Run the real ROS adapter method with deterministic clocks and messages."""
import ast
import math
from types import SimpleNamespace as NS
from unittest.mock import Mock

from test.test_arrival_delivery import method
from rosy_control.control.execution_escape import ExecutionEscape
from rosy_control.control.recover import hazard_action
from rosy_control.control.route_recovery import RouteRecovery
from test.test_execution_escape import proposal


def adapter():
    clock = NS(value=100.)
    scope=dict(math=math,time=NS(monotonic=lambda:clock.value),hazard_action=hazard_action)
    exec(compile(ast.Module(body=[method('wander/navigator.py','_execution_escape_step')],type_ignores=[]),'<adapter>','exec'),scope)
    node=NS(navigation_session=NS(active=True,options=dict(duration_s=30,stall_s=20),started=0.,progress_at=0.),
        motion_limits=dict(translation_mode=True,reverse_travel_m=.1,geometry_revision='g',execution_escape=proposal(0)),
        navigation_recovery=NS(waiting=True,failed_exit=(.1,0.)),
        execution_escape=ExecutionEscape(),odom_x=0.,odom_y=0.,odom_yaw=0.,
        tilt=False,cliff=False,_odom_fresh=lambda:True,_motion_limits_fresh=lambda:True,
        blocked=False,_on_wall=lambda:False,
        _can_reverse=lambda:True,_obstacle_wait=Mock(return_value=None),
        _session_now=lambda:clock.value-100.)
    def step(t, stale=None):
        clock.value=100.+t
        node.motion_limits_received=node.odom_received=clock.value
        node.odom_stamp_ns=int(t*1e9)
        node.motion_limits['execution_escape']['issued_s']=t
        if stale=='limits':node.motion_limits_received-=.3
        if stale=='odom':node.odom_received-=.3
        if stale=='source':node.odom_stamp_ns-=300_000_000
        return scope['_execution_escape_step'](node,t,(0.,0.,0.),True)
    return node,step


def test_real_adapter_fresh_rear_motion_works_without_full_rotation_scan():
    node,step=adapter()
    assert step(0)[0] == -.005
    assert step(5)==(-.005,'execution_escape_reverse')
    node._obstacle_wait.assert_called_with(-.005,0.)
    assert node.navigation_session.started==0.
    assert node.navigation_recovery.waiting


def test_real_adapter_requires_both_receipt_and_source_freshness():
    for stale in ('limits','odom','source'):
        node,step=adapter()
        step(0)
        assert step(5,stale)[0] == 0.
        assert not node.execution_escape.used


def test_real_adapter_never_retreats_without_active_bounded_session_or_with_cliff():
    for failure in ('inactive','deadline','stall_deadline','cliff','obstacle'):
        node,step=adapter()
        step(0)
        if failure=='inactive':node.navigation_session.active=False
        if failure=='deadline':node.navigation_session.options['duration_s']=10
        if failure=='stall_deadline':node.navigation_session.options['stall_s']=10
        if failure=='cliff':node.cliff=True
        if failure=='obstacle':node._obstacle_wait.return_value='yield:obstacle'
        result=step(5)
        assert result[0] in (None,0.)
        assert node.navigation_session.started==0.


def test_known_gate_rotation_block_requests_alternative_without_motor_stall_wait():
    recovery=RouteRecovery()
    args=((0.,0.,0.),'gate_rotation_blocked',(1.,0.),0.,True,(.06,0.))
    assert recovery.update(0.,*args)=='following'
    assert recovery.update(5.,*args)=='replan'
    assert recovery.failed_exit==(.06,0.)
    assert recovery.failed_reason=='gate_rotation_blocked'


def test_rotation_denial_behind_uses_measured_prediction_not_goal_direction():
    node,step=adapter()
    node.navigation_recovery.failed_exit=(-.1,0.)
    node.navigation_recovery.failed_reason='gate_rotation_blocked'
    node.motion_limits.update(forward_travel_m=.074,reverse_travel_m=.270)
    step(0)
    assert step(5)==(-.005,'execution_escape_reverse')


def test_forward_prediction_requires_forward_clearance_and_front_guards():
    for blocked in (False,True):
        node,step=adapter()
        node.navigation_recovery.failed_exit=(-.1,0.)
        node.motion_limits.update(forward_travel_m=.10,reverse_travel_m=0.)
        node.motion_limits['execution_escape']=proposal(0,direction=1,target=.036)
        node.blocked=blocked
        step(0)
        assert step(5)[0] == (None if blocked else .005)


def test_every_ended_escape_replans_with_zero_and_keeps_attempt_consumed():
    tick=method('wander/navigator.py','_tick_navigation')
    branch=next(n for n in tick.body if isinstance(n,ast.If)
                and ast.unparse(n.test)=='escape_v is not None')
    fn=ast.parse('def finish(self, escape_v, escape_reason):\n pass\n return v,w,reason').body[0]
    fn.body[0]=branch
    scope=dict(String=lambda **k:NS(**k))
    exec(compile(ast.fix_missing_locations(ast.Module(body=[fn],type_ignores=[])),'<finish>','exec'),scope)
    for reason in ('execution_escape_complete','execution_escape_stopped'):
        node=NS(navigation_goal_pub=Mock(),execution_escape=ExecutionEscape())
        node.execution_escape.used=True
        assert scope['finish'](node,0.,reason)==(0.,0.,reason)
        node.navigation_goal_pub.publish.assert_called_once()
        assert node.navigation_goal_pub.publish.call_args.args[0].data=='replan'
        assert node.execution_escape.used
