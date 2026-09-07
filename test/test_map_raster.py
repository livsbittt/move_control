"""ROS map rows and browser pixel rows must represent the same world cell."""
import unittest

import numpy as np

from move_control.sensing.map_raster import occupancy_bgr


class MapRasterTest(unittest.TestCase):
    def test_north_south_and_east_west_are_not_mirrored(self):
        # ROS row zero is south; PNG row zero is north.
        image = occupancy_bgr([0, -1, 100, 0, -1, 100], 2, 3)
        np.testing.assert_array_equal(image[0, 1], [225, 224, 217])
        np.testing.assert_array_equal(image[0, 0], [22, 22, 21])
        np.testing.assert_array_equal(image[2, 0], [35, 35, 34])

    def test_free_world_cell_remains_free_in_its_image_pixel(self):
        cells = [100] * 20
        cells[1 * 5 + 3] = 0
        image = occupancy_bgr(cells, 5, 4)
        np.testing.assert_array_equal(image[4 - 1 - 1, 3], [35, 35, 34])
        self.assertEqual(np.count_nonzero(np.all(image == [35, 35, 34], axis=2)), 1)

    def test_rejects_mismatched_dimensions(self):
        with self.assertRaises(ValueError):
            occupancy_bgr([0], 2, 3)


if __name__ == '__main__':
    unittest.main()
