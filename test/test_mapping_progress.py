import math
import unittest
from unittest.mock import patch

from rosy_control.planning import GoalBrain, OccupancyMap, ZigzagPlanner, FREE
from rosy_control.planning.zigzag import cover_ring


class MappingProgressTest(unittest.TestCase):
    def test_portrait_room_generates_long_axis_lanes(self):
        m = OccupancyMap(8, 18, .05, fill=FREE)
        points = ZigzagPlanner(m, start=(.1, .1)).waypoints()
        self.assertGreater(len(points), 5)
        self.assertGreater(max(y for x, y in points), .7)
        self.assertTrue(all(0 <= x < .4 and 0 <= y < .9 for x, y in points))

    def test_growth_origin_shift_preserves_world_locations_already_swept(self):
        brain = GoalBrain()
        old = OccupancyMap(20, 20, .05, fill=FREE)
        brain._track_map_lattice(old)
        brain.covered = {(2, 2)}
        new = OccupancyMap(24, 24, .05, origin=(-.1, -.1), fill=FREE)
        brain._track_map_lattice(new)
        self.assertEqual(brain.covered, {(4, 4)})
        self.assertEqual(old.grid_to_world(2, 2), new.grid_to_world(4, 4))

    def test_unreachable_waypoint_is_not_counted_as_visited_or_done(self):
        brain = GoalBrain(blacklist_plans=100)
        brain.mode = 'coverage'
        m = OccupancyMap(12, 12, .05, fill=FREE)
        pose = (.225, .225)
        with patch('rosy_control.planning.goals.best_route', return_value=None):
            statuses = [brain.plan(m, pose)[2] for _ in range(50)]
        self.assertTrue(brain._coverage_deferred)
        self.assertNotIn('coverage done', statuses)
        for cell in brain.covered:
            x, y = m.grid_to_world(*cell)
            self.assertLessEqual(math.hypot(x - pose[0], y - pose[1]), .06)

    def test_sweep_radius_is_not_rounded_up_to_a_larger_disk(self):
        m = OccupancyMap(10, 10, .05, fill=FREE)
        covered = set()
        cover_ring(covered, m, .225, .225, radius_m=.06)
        self.assertNotIn((6, 4), covered)  # 10 cm away was falsely swept.

    def test_manual_goal_does_not_retry_raw_map_by_default(self):
        brain = GoalBrain(clear_m=.14)
        brain.set_manual(.8, .8)
        with patch('rosy_control.planning.goals.best_route', return_value=None) as route:
            brain.plan(OccupancyMap(20, 20, .05, fill=FREE), (.4, .4))
        self.assertEqual([call.kwargs['clear_m'] for call in route.call_args_list], [.14])

    def test_map_reset_discards_coverage_and_deferred_memory(self):
        brain = GoalBrain(clear_m=.14)
        brain.covered.add((3, 3))
        brain._coverage_deferred[(4, 4)] = 100
        brain.set_manual(1., 1.)
        brain.reset()
        self.assertEqual(brain.covered, set())
        self.assertEqual(brain._coverage_deferred, {})
        self.assertIsNone(brain._manual)
        self.assertEqual(brain.clear_m, .14)
