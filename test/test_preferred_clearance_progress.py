"""Comfort clearance must not turn a safe mission into a backward escape."""
from move_control.planning import FREE, OCC, OccupancyMap, GoalBrain


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
