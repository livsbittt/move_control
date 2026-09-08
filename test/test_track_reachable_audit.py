import unittest
import numpy as np

from tools.gz.track_map_audit import measure, measure_point_reachable


class ReachableAuditTests(unittest.TestCase):
    def setUp(self):
        # Two sealed rooms; spawn is in the left room, right stays unobserved.
        self.walls = [([x,y,0,0,0,0], [w,h,.2]) for x,y,w,h in (
            (0,1,.1,2), (2,1,.1,2), (1,0,2,.1), (1,2,2,.1), (1,1,.1,2))]
        self.arr = np.full((210,210), -1, dtype=int)
        self.arr[:, :105] = 0
        self.origin = [-.05,-.05]

    def test_sealed_room_excluded_only_from_observational_metric(self):
        full = measure(self.arr, self.origin, .01, self.walls)
        result = measure_point_reachable(self.arr, self.origin, .01, self.walls, [.5,1])
        self.assertGreater(full['interior_unknown_fraction'], .45)
        self.assertFalse(full['map_raster_complete'])
        self.assertEqual(result['unknown_fraction'], 0.)
        self.assertFalse(result['robot_footprint_accessibility'])
        self.assertNotIn('map_raster_complete', result)

    def test_cropped_raster_keeps_fixed_world_denominator(self):
        full = measure_point_reachable(self.arr, self.origin, .01, self.walls, [.5,1])
        crop = measure_point_reachable(self.arr[:, :55], self.origin, .01, self.walls, [.5,1])
        self.assertEqual(crop['sample_count'], full['sample_count'])
        self.assertGreater(crop['unknown_fraction'], .4)
