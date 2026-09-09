"""Exercise the actual arrival sender/receiver without ROS transport."""
import ast
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock

from rosy_control.planning import GoalBrain, OccupancyMap


def method(path, name):
    tree = ast.parse((Path(__file__).parents[1] / 'rosy_control' / path).read_text(encoding='utf-8'))
    return next(n for cls in tree.body if isinstance(cls, ast.ClassDef)
                for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == name)


def sender():
    tick = method('wander/navigator.py', '_tick_navigation')
    branch = next(n for n in tick.body if isinstance(n, ast.If)
                  and "reason == 'arrived'" in ast.unparse(n.test))
    fn = ast.parse('def send(self, now, pose, reason="arrived"):\n pass').body[0]
    fn.body = [branch]
    scope = dict(math=math, json=json, String=lambda **k: NS(**k), Twist=lambda: NS())
    exec(compile(ast.fix_missing_locations(ast.Module(body=[fn], type_ignores=[])), '<sender>', 'exec'), scope)
    return scope['send']


def test_rejected_arrival_retries_current_route_then_advances_real_planner():
    m = OccupancyMap(30, 30, .02, fill=0)
    brain = GoalBrain(clear_m=.04)
    brain.mode = 'coverage'
    target, _, _ = brain.plan(m, (.3, .3))
    receiver = NS(mode='coverage', map_obj=m, brain=brain, get_logger=Mock(),
                  issued_routes=[(100, tuple(target))], last_executable_goal=target,
                  pose=lambda: ((target[0]+.05, target[1]), 'tf'))
    received = []
    def clear(_):
        receiver.issued_routes.clear()
    def plan():
        received.append(brain.plan(m, target)[0])
    receiver._clear_route, receiver.plan = clear, plan
    scope = dict(math=math, json=json)
    exec(compile(ast.Module(body=[method('goal_node.py', 'on_arrival')], type_ignores=[]), '<receiver>', 'exec'), scope)
    publications = []
    def publish(msg):
        publications.append(json.loads(msg.data))
        scope['on_arrival'](receiver, msg)
    node = NS(navigation_mode='coverage', trail_retreat=NS(active=False),
              navigation_route=[target], navigation_stamp_ns=100,
              navigation_arrived_target=None, navigation_arrival_sent_at=None,
              navigation_arrival_pub=NS(publish=publish),
              navigation_progress=Mock(), navigation_recovery=Mock(), _publish=Mock())
    send = sender()
    send(node, 10., (*target, 0.))
    assert not received  # Other TF listener has not caught up yet.
    for i in range(1, 25):
        send(node, 10.+i*.02, (*target, 0.))
    assert len(publications) == 1  # Never flood at the 50 Hz control rate.
    receiver.pose = lambda: (target, 'tf')
    receiver.issued_routes = [(200, tuple(target))]
    node.navigation_stamp_ns = 200  # Planner periodically refreshed the route.
    send(node, 10.5, (*target, 0.))
    assert len(publications) == 2
    assert publications[-1]['route_stamp_ns'] == 200
    assert received and math.dist(received[0], target) > brain.reach_tol
    assert not brain._failed_goals
    # Delayed duplicate still fails the receiver's issued-route validation.
    send(node, 11., (*target, 0.))
    assert len(received) == 1


def test_resized_map_endpoint_uses_exact_published_route_identity():
    from test.test_preferred_clearance_progress import committed_escape_fixture
    m, brain, start, original = committed_escape_fixture()
    # Actual run shifted the map lattice by 2.739 mm during a committed
    # escape. The planner permits <5 mm endpoint drift, but arrival identity
    # must refer to the endpoint it actually published after replanning.
    m.ox -= .00273923699646522
    target, route, _ = brain.plan(m, start)
    endpoint = route['points'][-1]
    assert target == original and 0 < math.dist(endpoint, target) < .005
    node = NS(last_executable_goal=None, issued_routes=[],
              get_clock=lambda: NS(now=lambda: NS(to_msg=lambda: NS(sec=1, nanosec=2))),
              goal_pub=Mock(), route_pub=Mock())
    def pose_stamped():
        return NS(header=NS(), pose=NS(position=NS(), orientation=NS()))
    scope = dict(math=math, PoseStamped=pose_stamped, Path=lambda: NS(poses=[]))
    exec(compile(ast.Module(body=[method('goal_node.py', '_pub_goal')], type_ignores=[]), '<publisher>', 'exec'), scope)
    scope['_pub_goal'](node, *target, route)
    published = node.route_pub.publish.call_args.args[0].poses[-1].pose.position
    assert node.issued_routes[-1] == (1_000_000_002, (published.x, published.y))
    node.mode, node.map_obj, node.brain = 'explore', m, brain
    node.pose = lambda: (endpoint, 'tf')
    node._clear_route, node.plan, node.get_logger = Mock(), Mock(), Mock()
    scope.update(json=json)
    exec(compile(ast.Module(body=[method('goal_node.py', 'on_arrival')], type_ignores=[]), '<receiver>', 'exec'), scope)
    scope['on_arrival'](node, NS(data=json.dumps(dict(route_stamp_ns=1_000_000_002, target=endpoint))))
    node.plan.assert_called_once()
    assert brain._committed_escape is None
