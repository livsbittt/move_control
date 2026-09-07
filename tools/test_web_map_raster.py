"""Check rendered raster orientation against ROS world coordinates."""
import unittest

import cv2
import numpy as np
from nav_msgs.msg import OccupancyGrid
from move_control import web_node as web


class MapRasterTest(unittest.TestCase):
    def test_north_wall_is_above_south_free_cell(self):
        grid = OccupancyGrid()
        grid.info.width = 2
        grid.info.height = 3
        grid.data = [0, 0, -1, -1, 100, 100]
        web.render_png(grid)
        self.assertEqual(web.STATE[web.K_MAP],
                         [2, 3, grid.info.resolution, 0.0, 0.0, web.MAP_PNG['gen']])
        raster = cv2.imdecode(np.frombuffer(web.MAP_PNG['bytes'], np.uint8),
                              cv2.IMREAD_COLOR)
        self.assertTrue(np.all(raster[0] > 200), 'North wall must be at image top')
        self.assertTrue(np.all(raster[-1] < 50), 'South free cells must be at bottom')


if __name__ == '__main__':
    unittest.main()
