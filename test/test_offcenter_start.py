import gzip
import json
import math
from pathlib import Path

from rosy_control.planning import OccupancyMap, GoalBrain
from rosy_control.planning.astar import best_route
from rosy_control.planning.start_connection import start_connections, swept_clear, connection_route_clear


def captured_map():
    with gzip.open(Path(__file__).parent/'fixtures/map_offcenter_start_20260910.json.gz', 'rt') as stream:
        d = json.load(stream)
    m = OccupancyMap(d['width'], d['height'], d['resolution'], d['origin'])
    m.data = d['data']
    return m, tuple(d['pose'])


def test_real_offcenter_pose_connects_without_using_unsafe_cell_center():
    m, pose = captured_map()
    assert not m.inflate(.096/m.res).is_free(*m.world_to_grid(*pose))
    route = best_route(m, pose, (.305, .70), clear_m=.096)
    assert route is not None
    assert route['points'][0] == pose
    assert m.grid_to_world(*m.world_to_grid(*pose)) not in route['points']
    assert route['clearance_m'] == .096
    assert swept_clear(m, *route['points'][:2], .096)


def test_brain_can_issue_safe_start_connector_before_normal_mission():
    m, pose = captured_map()
    brain = GoalBrain(clear_m=.12, retry_clear_m=.12, start_escape_clear_m=.096)
    brain.execution_feedback = True
    target, route, status = brain.plan(m, pose)
    assert route is not None, status
    assert route['points'][0] == pose
    assert math.dist(target, pose) > .025  # Arrival cannot swallow this move.
    assert route['clearance_m'] == .096
    assert target != (.234, 1.145)  # Old forward target is disconnected.
    assert swept_clear(m, *route['points'][:2], .096)


def test_truly_colliding_start_is_not_snapped_to_free_space():
    m, pose = captured_map()
    assert best_route(m, (pose[0]-.015, pose[1]), (.234, 1.145), clear_m=.096) is None


def test_clear_endpoints_do_not_authorize_segment_through_obstacle():
    m = OccupancyMap(50,50,.02,fill=0)
    m.set_cell(25,25,100)
    start, end = (.38,.51), (.64,.51)
    assert swept_clear(m,start,start,.096)
    assert swept_clear(m,end,end,.096)
    assert not swept_clear(m,start,end,.096)


def test_unknown_swept_body_is_blocked_even_when_centerline_is_known():
    m = OccupancyMap(50,50,.02,fill=0)
    m.set_cell(25,29,-1)  # 8 cm beside known centreline.
    assert not swept_clear(m,(.38,.51),(.64,.51),.096)


def test_pursuit_shortcut_across_otherwise_safe_corner_is_rejected():
    m = OccupancyMap(50,50,.02,fill=0)
    m.set_cell(20,20,100)
    points = [(.31,.31),(.31,.51),(.51,.51)]
    assert all(swept_clear(m,a,b,.06) for a,b in zip(points,points[1:]))
    assert not connection_route_clear(m,points,.06)


def test_no_connection_across_failed_exit_or_unknown_start():
    m, pose = captured_map()
    assert not start_connections(m,pose,.096,avoid_points=[pose])
    m.set_cell(*m.world_to_grid(*pose),-1)
    assert not start_connections(m,pose,.096)


def test_nonfinite_and_outside_map_start_rejected():
    m, pose = captured_map()
    assert not start_connections(m,(math.nan,pose[1]),.096)
    assert not start_connections(m,(-1.,pose[1]),.096)


def test_failed_start_reports_metric_clearance_without_claiming_live_obstacle():
    m, pose = captured_map()
    brain = GoalBrain(clear_m=.12,start_escape_clear_m=.096)
    _, route, status = brain.plan(m,(pose[0]-.015,pose[1]))
    assert route is None
    assert 'map start clearance' in status
    assert 'required=96.0mm' in status
