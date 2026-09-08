import numpy as np

from tools.gz.track_map_audit import measure


WALLS = [([0, -1, 0, 0, 0, 0], [2, .02, .2]),
         ([0, 1, 0, 0, 0, 0], [2, .02, .2]),
         ([-1, 0, 0, 0, 0, 0], [.02, 2, .2]),
         ([1, 0, 0, 0, 0, 0], [.02, 2, .2])]


def raster():
    arr = np.zeros((202, 202), dtype=np.int16)
    arr[:2] = arr[-2:] = 100
    arr[:, :2] = arr[:, -2:] = 100
    return arr


def test_empty_and_cropped_maps_cannot_pass_full_world_audit():
    empty = measure(np.full((202, 202), -1), [-1.01, -1.01], .01, WALLS)
    assert empty['interior_unknown_fraction'] == 1
    assert not empty['map_raster_complete']
    cropped = measure(np.zeros((50, 50)), [-.25, -.25], .01, WALLS)
    assert cropped['interior_unknown_fraction'] > .9
    assert not cropped['map_raster_complete']


def test_complete_raster_passes_and_phantom_obstacle_fails():
    arr = raster()
    good = measure(arr, [-1.01, -1.01], .01, WALLS)
    assert good['map_raster_complete']
    arr[70:130, 70:130] = 100
    bad = measure(arr, [-1.01, -1.01], .01, WALLS)
    assert not bad['map_raster_complete']
    assert bad['phantom_fraction'] > .02


def test_exact_world_has_a_sealed_triangle_outside_spawn_component():
    from pathlib import Path
    import xml.etree.ElementTree as ET
    from tools.gz.prepare_track_world import boxes
    from tools.gz.track_map_audit import topology
    walls = boxes(ET.parse(Path(__file__).resolve().parents[1]/'map/map_260905.world'))
    result = topology(walls, [-.2, .27])
    assert result['free_components'] == 2
    assert .08 < result['sealed_free_area_m2'] < .11
    assert .02 < result['sealed_free_fraction'] < .04
