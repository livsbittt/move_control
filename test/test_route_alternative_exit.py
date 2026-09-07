import math
from move_control.planning import OccupancyMap, GoalBrain
from move_control.planning.astar import best_route


def test_alternative_avoids_failed_entry_without_modifying_map():
    m = OccupancyMap(35, 25, .02, fill=0)
    start, goal, failed = (.21, .21), (.55, .21), (.27, .21)
    original = list(m.data)
    route = best_route(m, start, goal, clear_m=.08, avoid_points=[failed])
    assert route
    assert all(math.dist(p, failed) > .05 for p in route['points'][1:])
    assert m.data == original
    exit_point = next(p for p in route['points'] if math.dist(p, start) >= .06)
    assert math.dist(exit_point, failed) > .05


def test_brain_applies_failed_exit_to_real_route_and_expires_it():
    m = OccupancyMap(35, 25, .02, fill=0)
    b = GoalBrain(clear_m=.08, blacklist_plans=3)
    b.set_manual(.55, .21)
    b.avoid_route_exit((.27, .21))
    _, route, status = b.plan(m, (.21, .21))
    assert route and status.startswith('alternative exit:')
    assert all(math.dist(p, (.27, .21)) > .05 for p in route['points'][1:])
    b.plan(m, (.21, .21))
    b.plan(m, (.21, .21))
    assert not b._failed_exits


def test_blocked_primary_tries_a_different_ranked_goal():
    from unittest.mock import Mock
    m = OccupancyMap(12, 9, .02, fill=100)
    for x in range(1, 11):
        m.data[5*m.w+x] = 0
    for y in range(1, 6):
        m.data[y*m.w+2] = 0
    start, first, other = (.05,.11), (.19,.11), (.05,.03)
    b = GoalBrain(clear_m=0.)
    initial = best_route(m,start,first,clear_m=0.)
    b._plan_candidate = Mock(return_value=(first,initial,'first'))
    b.last_options = [{'x':other[0],'y':other[1],'route':best_route(m,start,other,clear_m=0.)}]
    b.avoid_route_exit((.11,.11))
    goal, route, _ = b.plan(m,start)
    assert route and math.dist(goal,other)<1e-8
