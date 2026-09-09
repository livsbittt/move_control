"""Comfort clearance must not turn a safe mission into a backward escape."""
from rosy_control.planning import FREE, OCC, OccupancyMap, GoalBrain


def committed_escape_fixture():
    m=OccupancyMap(50,30,.02,fill=FREE)
    for col in range(m.w):m.set_cell(col,0,OCC)
    for col in range(45,50):
        for row in range(m.h):m.set_cell(col,row,-1)
    brain=GoalBrain(clear_m=.16,retry_clear_m=.12,start_escape_clear_m=.12)
    brain.execution_feedback=True
    start=m.grid_to_world(10,7)
    target,route,status=brain.plan(m,start)
    assert status.startswith('escape:')
    return m,brain,start,target


def test_escape_target_commits_until_matching_execution_completion():
    m,brain,start,target=committed_escape_fixture()
    pose=m.grid_to_world(10,8)
    for _ in range(3):
        goal,route,status=brain.plan(m,pose)
        assert goal==target
        assert status.startswith('escape:')
        assert route['clearance_m']==.12
        assert all(m.inflate(.12/m.res).is_free(*cell) for cell in route['cells'])
    brain.execution_feedback=False  # A missed feedback heartbeat is not arrival.
    assert brain.plan(m,pose)[0]==target
    brain.execution_feedback=True
    brain.complete_goal(m,pose,(.8,.4))
    assert brain.plan(m,pose)[0]==target
    brain.complete_goal(m,target,target)
    assert not brain.plan(m,target)[2].startswith('escape:')


def test_new_obstacle_releases_invalid_escape_without_claiming_arrival():
    m,brain,start,target=committed_escape_fixture()
    m.set_cell(*m.world_to_grid(*target),OCC)
    goal,route,status=brain.plan(m,start)
    assert goal is None and route is None
    assert brain._committed_escape is None
    assert not brain._completed_goals


def test_offline_planning_does_not_require_execution_arrival_feedback():
    m,brain,start,_=committed_escape_fixture()
    brain.reset()
    brain.execution_feedback=False
    assert brain.plan(m,start)[2].startswith('escape:')
    assert brain._committed_escape is None


def test_explicit_new_manual_or_reset_cancels_escape_commitment():
    for action in ('manual','reset','restart','avoid_goal','avoid_exit'):
        m,brain,start,target=committed_escape_fixture()
        if action=='manual':brain.set_manual(*m.grid_to_world(40,8))
        elif action=='reset':brain.reset()
        elif action=='restart':brain.restart_recovery()
        elif action=='avoid_goal':brain.avoid_goal(target)
        else:brain.avoid_route_exit(target)
        assert brain._committed_escape is None


def test_minimum_safe_manual_target_wins_over_nearby_comfort_escape():
    m = OccupancyMap(50, 30, .02, fill=FREE)
    for col in range(m.w):
        m.set_cell(col, 0, OCC)
    pose = m.grid_to_world(10, 7)
    target = m.grid_to_world(40, 7)
    brain = GoalBrain(clear_m=.16, retry_clear_m=.12, start_escape_clear_m=.12)
    brain.set_manual(*target)
    goal, route, status = brain.plan(m, pose)
    assert goal == target
    assert route['clearance_m'] == .12
    assert all(m.inflate(.12/m.res).is_free(*cell) for cell in route['cells'])
    assert 'manual goal' in status
    assert brain.clear_m == .16


def test_active_frontier_survives_pivot_motion_across_comfort_boundary():
    m = OccupancyMap(50, 30, .02, fill=FREE)
    for col in range(m.w):
        m.set_cell(col, 0, OCC)
    for col in range(45, 50):
        for row in range(m.h):
            m.set_cell(col, row, -1)
    brain = GoalBrain(clear_m=.16, retry_clear_m=.12, start_escape_clear_m=.12)
    brain.execution_feedback = True
    target, route, _ = brain.plan(m, m.grid_to_world(10, 10))
    assert target is not None
    goal, route, status = brain.plan(m, m.grid_to_world(10, 7))
    assert goal == target
    assert status.startswith('narrow passage: explore')
    assert route['clearance_m'] == .12
    assert all(m.inflate(.12/m.res).is_free(*cell) for cell in route['cells'])


def test_obstructed_escape_target_selects_another_safe_goal_same_tick():
    m=OccupancyMap(40,30,.05,fill=FREE)
    brain=GoalBrain(clear_m=.10,retry_clear_m=.10,start_escape_clear_m=.10)
    brain.execution_feedback=True
    start=m.grid_to_world(5,10)
    target=m.grid_to_world(25,10)
    brain._committed_escape=(target,.10)
    m.set_cell(*m.world_to_grid(*target),OCC)
    goal,route,status=brain.plan(m,start)
    assert goal is not None and route is not None
    assert goal != target
    assert all(m.inflate(.10/m.res).is_free(*cell) for cell in route['cells'])
    assert not brain._completed_goals


def test_obstacle_across_direct_route_is_routed_around_without_abandoning_goal():
    m=OccupancyMap(40,30,.05,fill=FREE)
    brain=GoalBrain(clear_m=.10,retry_clear_m=.10)
    start=m.grid_to_world(5,10);target=m.grid_to_world(25,10)
    brain._committed_escape=(target,.10)
    for row in range(6,15):m.set_cell(15,row,OCC)
    goal,route,status=brain.plan(m,start)
    assert goal==target and route is not None
    assert all(m.inflate(.10/m.res).is_free(*cell) for cell in route['cells'])
    assert any(row<4 or row>16 for col,row in route['cells'])
