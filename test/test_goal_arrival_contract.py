import ast
import json
import math
from types import SimpleNamespace as NS
from unittest.mock import Mock
from rosy_control.control.path_follow import GOAL_TOLERANCE_M,PathFollower,follow_path
from rosy_control.planning.goals import GoalBrain
from test.test_arrival_delivery import method


def test_planner_follower_and_receiver_share_inclusive_arrival_boundary():
    brain=GoalBrain()
    assert brain.reach_tol==GOAL_TOLERANCE_M==.025
    scope={'math':math,'json':json}
    exec(compile(ast.Module(body=[method('goal_node.py','on_arrival')],type_ignores=[]),'<receiver>','exec'),scope)
    for distance,arrived in ((.025,True),(.025001,False),(.04,False)):
        v,w,reason=follow_path([(distance,0.),(0.,0.)],(distance,0.,math.pi),route_age=0.,tf_age=0.)
        assert (reason=='arrived')==arrived
        receiver=NS(mode='explore',map_obj=object(),issued_routes=[(1,(0.,0.))],last_executable_goal=(0.,0.),
                    brain=NS(reach_tol=brain.reach_tol,complete_goal=Mock()),pose=lambda:((distance,0.),'tf'),
                    get_logger=Mock(),_clear_route=Mock(),plan=Mock())
        scope['on_arrival'](receiver,NS(data=json.dumps({'route_stamp_ns':1,'target':[0.,0.]})))
        assert receiver.plan.called==arrived
    brain._manual=(0.,0.)
    _,_,status=brain._manual_plan(None,(.025,0.))
    assert status=='manual goal reached'


def test_real_seventy_four_mm_gap_is_not_arrival_or_follower_deadzone():
    pose=(.285,.633);goal=(.27,.56)
    yaw=math.atan2(goal[1]-pose[1],goal[0]-pose[0])
    assert math.dist(pose,goal)>.074
    v,w,reason=follow_path([pose,goal],(*pose,yaw),route_age=0.,tf_age=0.)
    assert reason=='forward' and v>0.
    assert abs(w)<1e-10


def test_tiny_intermediate_vertex_does_not_stop_short_of_real_goal():
    follower=PathFollower()
    v,w,reason=follower.update([(0.,0.),(.003,0.),(.003,.1)],(0.,0.,math.pi/2),route_age=0.,tf_age=0.)
    assert reason=='forward' and v>0.
    assert follower.cursor==2