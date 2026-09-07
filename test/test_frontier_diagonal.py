"""Raycast boundaries are staircases; diagonal adjacency must retain them."""
from move_control.planning.gridmap import OccupancyMap
from move_control.planning.frontier import frontier_points


def test_diagonal_boundary_is_one_frontier_instead_of_discarded_singletons():
    m = OccupancyMap(12, 12, .02)
    for row in range(11):
        for col in range(11-row):
            m.set_cell(col, row, 0)
    frontiers = frontier_points(m, min_size=6)
    assert len(frontiers) == 1
    assert frontiers[0]['size'] == 11
