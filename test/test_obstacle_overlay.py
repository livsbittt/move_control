from move_control.planning.gridmap import OccupancyMap
from move_control.planning.obstacle_overlay import obstacle_overlay
from move_control.planning.astar import best_route


def test_local_obstacle_overlay_preserves_saved_map_and_unknown_space():
    original = OccupancyMap(20, 20, .05, fill=0)
    original.set_cell(0, 0, -1)
    updated = obstacle_overlay(original, [{'position': (.5, .5), 'radius': .08}], (0., 0., 0.))
    assert updated.cell(10, 10) == 100
    assert original.cell(10, 10) == 0
    assert updated.cell(0, 0) == -1
    assert obstacle_overlay(original, [], (0., 0., 0.)).cell(10, 10) == 0


def test_off_map_obstacle_is_not_clamped_to_map_border():
    original = OccupancyMap(10, 10, .05, fill=0)
    updated = obstacle_overlay(original, [{'position': (-5., 0.), 'radius': .1}], (0.,0.,0.))
    assert updated.data == original.data


def test_overlay_applies_odom_to_map_transform():
    original = OccupancyMap(30, 30, .05, fill=0)
    updated = obstacle_overlay(original, [{'position': (.2, .2), 'radius': .02}], (.5,.5,0.))
    assert updated.cell(14, 14) == 100


def test_route_detours_new_obstacle_without_changing_original_target():
    original = OccupancyMap(40, 30, .05, fill=0)
    updated = obstacle_overlay(original, [{'position': (1., .75), 'radius': .1}], (0.,0.,0.))
    route = best_route(updated, (.25,.75), (1.75,.75), clear_m=.15)
    assert route is not None
    assert any(abs(y-.75) > .2 for x,y in route['points'])
    assert all(original.data[i] == 0 for i in range(len(original.data)))


def test_local_padding_matches_risk_margin_without_inflating_existing_walls():
    original = OccupancyMap(100, 100, .01, fill=0)
    original.set_cell(10, 10, 100)
    updated = obstacle_overlay(original, [{'position': (.5,.5), 'radius': .03}],
                               (0.,0.,0.), padding=.05)
    assert updated.cell(57,50) == 100
    assert updated.cell(11,10) == 0
    assert original.cell(57,50) == 0
