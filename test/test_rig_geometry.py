"""Collision geometry, rather than a nominal chassis size, bounds rotation."""
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

from tools.gz.prepare_track_world import collision_circumradius


class RigGeometryTests(unittest.TestCase):
    def test_wheels_define_full_robot_envelope(self):
        model = ET.parse(Path(__file__).parents[1] / 'tools/gz/pinky_maze.sdf').find(
            ".//model[@name='pinky']")
        result = collision_circumradius(model)
        self.assertGreaterEqual(result['radius_m'], math.hypot(.068, .092))
        self.assertLess(result['radius_m'], .116)
        self.assertEqual(len(result['collisions']), 4)
        self.assertEqual(result['frame'], 'model_origin')

    def test_projected_footprint_retains_offsets_and_bounds_radius(self):
        model = ET.fromstring('''<model><link name="base">
          <collision><pose>-0.04 0.082 0.028 -1.5707963267948966 0 0</pose>
          <geometry><cylinder><radius>0.028</radius><length>0.02</length></cylinder></geometry>
          </collision></link></model>''')
        result = collision_circumradius(model)
        vertices = result['footprint_xy']
        self.assertEqual(len(vertices), 8)
        self.assertAlmostEqual(max(math.hypot(x, y) for x, y in vertices), result['radius_m'])
        self.assertAlmostEqual(min(x for x, y in vertices), -.068)
        self.assertAlmostEqual(max(y for x, y in vertices), .092)

    def test_composes_link_and_collision_rotation_translation(self):
        model = ET.fromstring('''<model><pose>999 999 0 0 0 1</pose>
          <link name="base"><pose>1 0 0 0 0 1.5707963267948966</pose>
          <collision name="body"><pose>1 0 0 0 1.5707963267948966 0</pose>
          <geometry><box><size>2 4 6</size></box></geometry>
          </collision></link></model>''')
        self.assertAlmostEqual(collision_circumradius(model)['radius_m'], 5.)

    def test_missing_unsupported_or_invalid_geometry_fails(self):
        for geometry in ('', '<geometry/>', '<geometry><mesh/></geometry>',
                         '<geometry><box><size>1 -2 3</size></box></geometry>',
                         '<geometry><cylinder><radius>nan</radius><length>1</length></cylinder></geometry>'):
            with self.subTest(geometry=geometry), self.assertRaises(ValueError):
                collision_circumradius(ET.fromstring(
                    '<model><link><collision>' + geometry + '</collision></link></model>'))
        with self.assertRaises(ValueError):
            collision_circumradius(ET.fromstring('<model><link/></model>'))

    def test_unresolved_pose_frames_and_nested_models_fail(self):
        for xml in ('<model><model/></model>',
                    '<model><link><pose relative_to="other">0 0 0 0 0 0</pose></link></model>'):
            with self.assertRaises(ValueError):
                collision_circumradius(ET.fromstring(xml))
