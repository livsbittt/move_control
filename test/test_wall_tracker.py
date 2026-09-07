import math
import unittest
from types import SimpleNamespace

from move_control.sensing.wall_tracker import WallTracker, _fit


def corner(travel=0., outer=False, dropout=False, slope=1.):
    scan = SimpleNamespace(angle_min=-math.pi, angle_increment=math.pi/360,
                           range_min=.05, range_max=40., ranges=[])
    for i in range(720):
        angle = math.atan2(math.sin(scan.angle_min+i*scan.angle_increment-math.pi),
                           math.cos(scan.angle_min+i*scan.angle_increment-math.pi))
        a = slope if angle < 0 else -.6
        b = (.35 if outer and angle >= 0 else .3)-travel
        divisor = math.cos(angle)-a*math.sin(angle)
        value = b/divisor if divisor > 0 else math.inf
        if dropout and angle < 0:
            value = math.inf
        scan.ranges.append(value)
    return scan


def collect(tracker, scan):
    for _ in range(3):
        value = tracker.update(scan, math.pi)
    return value


def world_wall(pose, mount=(-.017, 0.)):
    scan = SimpleNamespace(angle_min=-math.pi, angle_increment=math.pi/360,
                           range_min=.05, range_max=40., ranges=[])
    x, y, yaw = pose
    sx = x+math.cos(yaw)*mount[0]-math.sin(yaw)*mount[1]
    sy = y+math.sin(yaw)*mount[0]+math.cos(yaw)*mount[1]
    for i in range(720):
        angle = scan.angle_min+i*scan.angle_increment-math.pi+yaw
        denominator = math.cos(angle)-math.sin(angle)
        scan.ranges.append((.3+sy-sx)/denominator if denominator > 0 else math.inf)
    return scan


class WallTrackerTest(unittest.TestCase):
    def test_captured_frame5_single_boundary_residual_is_excluded(self):
        # Compact replay of the measured frame5 residual pattern (mm):
        # 36 supporting returns and one 4.46mm endpoint, not a wall cluster.
        errors = [-1.738,-.121,1.424,2.897,2.324,.684,-.047,1.126,.243,-.716,
                  .235,.120,-.069,.664,.327,-.083,-.566,-.125,.245,-.457,
                  -.232,-.080,0.,-.992,-1.057,-.194,-.403,-.684,-1.038,
                  .533,.034,.459,-.185,1.090,2.289,1.424,4.460]
        points = []
        for i, error in enumerate(errors):
            angle = math.radians(-12+i*.5)
            y = .947*math.tan(angle)
            points.append((angle, y, .947+.01217*y+error/1000))
        fit = _fit(points)
        self.assertIsNotNone(fit)
        self.assertEqual(fit['excluded_rays'], 1)
        self.assertEqual(len(fit['_raw_points']), 37)
        self.assertEqual(len(fit['_points']), 36)
        self.assertLessEqual(fit['residual_m'], .003)

    def test_cluster_or_large_outlier_cannot_be_trimmed_into_wall(self):
        points = [(math.radians(-10+i*.5), (i-20)*.008, .4) for i in range(41)]
        for indices, offset in [([19,20], .008), ([20], .012)]:
            changed = [(a,y,x+offset if i in indices else x) for i,(a,y,x) in enumerate(points)]
            self.assertIsNone(_fit(changed))

    def test_inner_and_outer_corner_round_trip_tracks_same_wall(self):
        for outer in (False, True):
            tracker = WallTracker()
            origin = collect(tracker, corner(outer=outer))
            self.assertTrue(math.isfinite(origin))
            slope = tracker.wall['slope']
            for travel in (.01, .02, .03, .02, .01, 0.):
                distance = tracker.update(corner(travel, outer), math.pi, locked=True)
                self.assertAlmostEqual(origin-distance, travel, delta=.001)
                self.assertAlmostEqual(tracker.wall['slope'], slope, delta=.01)

    def test_locked_dropout_cannot_select_other_corner_wall(self):
        tracker = WallTracker()
        collect(tracker, corner())
        self.assertGreater(tracker.wall['slope'], 0.)
        self.assertTrue(math.isinf(tracker.update(corner(dropout=True), math.pi, locked=True)))
        self.assertTrue(tracker.locked)

    def test_locked_loss_requires_three_consecutive_same_wall_scans(self):
        tracker = WallTracker()
        origin = collect(tracker, corner())
        anchor = dict(tracker.anchor)
        self.assertTrue(math.isinf(tracker.update(corner(dropout=True), math.pi, locked=True)))
        self.assertEqual(tracker.stable, 0)
        self.assertTrue(math.isinf(tracker.update(corner(), math.pi, locked=True)))
        self.assertTrue(math.isinf(tracker.update(corner(), math.pi, locked=True)))
        # Another dropout restarts the streak, never accumulates sparse hits.
        self.assertTrue(math.isinf(tracker.update(corner(dropout=True), math.pi, locked=True)))
        for _ in range(2):
            self.assertTrue(math.isinf(tracker.update(corner(), math.pi, locked=True)))
        self.assertAlmostEqual(tracker.update(corner(), math.pi, locked=True), origin)
        self.assertEqual(tracker.anchor, anchor)
        self.assertTrue(tracker.locked)

    def test_same_support_different_wall_is_rejected(self):
        tracker = WallTracker()
        collect(tracker, corner())
        self.assertTrue(math.isinf(tracker.update(corner(travel=-.08), math.pi, locked=True)))

    def test_grazing_and_no_valid_support_are_rejected(self):
        tracker = WallTracker()
        scan = corner(slope=4.)
        for i in range(361):
            scan.ranges[i] = math.inf
        self.assertTrue(math.isinf(collect(tracker, scan)))

    def test_isolated_outlier_is_not_in_selected_support(self):
        tracker = WallTracker()
        scan = corner()
        scan.ranges[690] += .05
        value = collect(tracker, scan)
        self.assertTrue(math.isfinite(value))
        self.assertLessEqual(tracker.diagnostic['residual_m'], .003)

    def test_reset_requires_new_collection_and_releases_identity(self):
        tracker = WallTracker()
        collect(tracker, corner())
        tracker.update(corner(), math.pi, locked=True)
        tracker.reset()
        self.assertFalse(tracker.locked)
        self.assertTrue(math.isinf(tracker.update(corner(), math.pi)))

    def test_pose_compensates_yaw_lateral_and_mount_without_forward_odom(self):
        tracker = WallTracker()
        mount = (-.017, 0.)
        for _ in range(3):
            origin = tracker.update(world_wall((0., 0., 0.)), math.pi,
                                    pose=(0., 0., 0.), mount=mount)
        for x in (.01, .02, .03):
            actual = (x, .005, .03)
            # Deliberately wrong forward wheel scale must not bias LiDAR delta.
            value = tracker.update(world_wall(actual), math.pi, locked=True,
                                   pose=(x*1.2, .005, .03), mount=mount)
            self.assertAlmostEqual(origin-value, x, delta=.001)
        self.assertFalse(tracker.diagnostic['compensation']['forward_odom_used'])

    def test_pure_turn_with_lidar_lever_arm_does_not_report_translation(self):
        tracker = WallTracker()
        for _ in range(3):
            origin = tracker.update(world_wall((0., 0., 0.)), math.pi,
                                    pose=(0., 0., 0.), mount=(-.017, 0.))
        value = tracker.update(world_wall((0., 0., .03)), math.pi, locked=True,
                               pose=(0., 0., .03), mount=(-.017, 0.))
        self.assertAlmostEqual(origin-value, 0., delta=.001)

    def test_missing_pose_or_large_motion_cannot_reuse_compensation(self):
        tracker = WallTracker()
        tracker.update(world_wall((0., 0., 0.)), math.pi, pose=(0., 0., 0.), mount=(-.017, 0.))
        self.assertTrue(math.isinf(tracker.update(world_wall((0., 0., 0.)), math.pi)))
        self.assertTrue(math.isinf(tracker.update(world_wall((0., .02, 0.)), math.pi,
                          pose=(0., .02, 0.), mount=(-.017, 0.))))

    def test_coplanar_fragments_after_dropout_remain_same_locked_wall(self):
        tracker = WallTracker()
        origin = collect(tracker, world_wall((0., 0., 0.)))
        scan = world_wall((.01, 0., 0.))
        scan.ranges[0] = math.inf
        value = tracker.update(scan, math.pi, locked=True)
        self.assertAlmostEqual(origin-value, .01, delta=.001)
        self.assertEqual(tracker.diagnostic['fragments'], 2)
        self.assertNotIn('_points', tracker.diagnostic)

    def test_nearby_parallel_fragments_cannot_be_merged_as_one_wall(self):
        tracker = WallTracker()
        collect(tracker, world_wall((0., 0., 0.)))
        scan = world_wall((0., 0., 0.))
        scan.ranges[0] = math.inf
        # Both fragments match the 12mm association window, but one is a
        # distinct wall displaced 8mm along x, not a dropout in one plane.
        other = world_wall((-.008, 0., 0.))
        for i in range(1, 71):
            scan.ranges[i] = other.ranges[i]
        self.assertTrue(math.isinf(tracker.update(scan, math.pi, locked=True)))
        self.assertEqual(tracker.diagnostic['matches'], 2)
