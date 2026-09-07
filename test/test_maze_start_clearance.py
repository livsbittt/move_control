"""A base-centred robot must fit its configured maze planning margin."""
from pathlib import Path
import yaml
from move_control.planning import OccupancyMap, GoalBrain


def corridor():
    m = OccupancyMap(50, 15, .02, fill=0)
    for c in range(m.w):
        m.set_cell(c, 0, 100)
        m.set_cell(c, 14, 100)
        if c > 42:
            for r in range(1, 14):
                m.set_cell(c, r, -1)
    return m


def test_robot_profile_can_plan_from_six_cells_off_maze_wall():
    root = Path(__file__).resolve().parents[1]
    cfg = yaml.safe_load((root / 'config/goal.yaml').read_text(encoding='utf-8'))['goal_node']['ros__parameters']
    robot = yaml.safe_load((root / 'config/robot.yaml').read_text(encoding='utf-8'))
    radius = robot['/**']['ros__parameters']['robot_radius']
    assert cfg['start_escape_clear_m'] >= radius + .02
    assert cfg['retry_clear_m'] >= cfg['clear_m']
    m = corridor()
    pose = m.grid_to_world(12, 6)
    brain = GoalBrain(clear_m=cfg['clear_m'], retry_clear_m=cfg['retry_clear_m'],
                      start_escape_clear_m=cfg['start_escape_clear_m'])
    goal, route, status = brain.plan(m, pose)
    assert goal and route and status.startswith('escape:')
    assert route['length'] <= .08
    grid = m.inflate(round(cfg['start_escape_clear_m'] / m.res))
    assert all(grid.is_free(*cell) for cell in route['cells'])
    goal2, route2, status2 = brain.plan(m, goal)
    assert route2 and status2.startswith('explore')


def test_start_inside_clearance_reports_reason_and_never_snaps_robot():
    m = corridor()
    brain = GoalBrain(clear_m=.12, retry_clear_m=.12)
    goal, route, status = brain.plan(m, m.grid_to_world(12, 6))
    assert goal is None and route is None
    assert status == 'planning blocked: robot inside obstacle clearance'


def test_genuinely_too_close_start_stays_blocked_at_robot_margin():
    m = corridor()
    goal, route, status = GoalBrain(clear_m=.12, start_escape_clear_m=.10).plan(m, m.grid_to_world(12, 3))
    assert goal is None and route is None
    assert 'inside obstacle clearance' in status


def test_escape_cannot_cross_unknown_or_travel_far_to_find_clearance():
    from move_control.planning.escape import start_escape
    m = corridor()
    for c in range(m.w):
        m.set_cell(c, 7, -1)
    assert start_escape(m, m.grid_to_world(12,6), .12, .10, .08) is None
    m = corridor()
    assert start_escape(m, m.grid_to_world(12,6), .12, .10, .01) is None


def test_long_narrow_corridor_replans_at_hard_margin_without_lowering_setting():
    m = corridor()
    for c in range(m.w):
        m.set_cell(c, 13, 100)
    brain = GoalBrain(clear_m=.12, retry_clear_m=.12, start_escape_clear_m=.10)
    goal, route, status = brain.plan(m, m.grid_to_world(12,6))
    assert route and status.startswith('narrow passage: explore')
    assert brain.clear_m == .12 and brain.retry_clear_m == .12
    assert all(m.inflate(5).is_free(*cell) for cell in route['cells'])
