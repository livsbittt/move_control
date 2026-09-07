"""Saved maps must retain world north when loaded as top-down images."""
import unittest

import numpy as np

from tools.gz.save_map import map_pixels
from tools.gz.check_map import measure


class MapExportTest(unittest.TestCase):
    def test_north_row_is_first_in_pgm(self):
        # ROS rows start at the south edge; image rows start at the north.
        cells = np.array([[0, -1], [100, 0]], dtype=np.int16)
        np.testing.assert_array_equal(
            map_pixels(cells), [[0, 254], [254, 205]])

    def test_small_free_crop_does_not_mean_full_maze_explored(self):
        result = measure(np.full((10, 10), 254, np.uint8),
                         0.05, 0.3, 0.3, [])
        self.assertGreater(result['interior_unknown'], 0.95)
