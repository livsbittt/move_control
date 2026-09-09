import math
import unittest

from rosy_control.control.path_follow import ProgressGuard, follow_path, PathFollower


class PathFollowTest(unittest.TestCase):
    def test_actual_escape_with_refreshed_prefix_and_exact_offset_pivot_arrives(self):
        follower=PathFollower()
        route=[[-.221,-.177],[-.22,-.178],[-.24,-.178],[-.26,-.178],[-.28,-.178]]
        pose=[-.221,-.177,.596]
        pivot=(-.0398,-.0102)
        signs=[]
        for i in range(3000):
            if i%100==0:route=[pose[:2]]+route[1:]
            v,w,reason=follower.update(route,pose,route_age=0.,tf_age=0.)
            if reason=='arrived':break
            if abs(w)>.01:signs.append(1 if w>0 else -1)
            angle=w*.02
            c,s=math.cos(angle),math.sin(angle)
            dx=(1-c)*pivot[0]+s*pivot[1]
            dy=-s*pivot[0]+(1-c)*pivot[1]
            if abs(w)>1e-12:
                dx+=v/w*s;dy+=v/w*(1-c)
            else:dx+=v*.02
            pose[0]+=math.cos(pose[2])*dx-math.sin(pose[2])*dy
            pose[1]+=math.sin(pose[2])*dx+math.cos(pose[2])*dy
            pose[2]+=angle
        self.assertEqual(reason,'arrived')
        self.assertLess(i*.02,40.)
        self.assertLess(sum(a!=b for a,b in zip(signs,signs[1:])),4)

    def test_persistent_follower_skips_reached_initial_stub(self):
        follower=PathFollower()
        route=[(-.221,-.177),(-.22,-.178),(-.24,-.178),(-.26,-.178),(-.28,-.178)]
        v,w,_=follower.update(route,(-.221,-.177,.596),route_age=0.,tf_age=0.)
        self.assertEqual(v,0.)
        self.assertGreater(w,0.)  # Turn towards west, not the 1.4mm southeast stub.

    def test_skipping_stub_still_targets_next_unreached_real_corner(self):
        follower=PathFollower()
        route=[(0.,.001),(0.,0.),(.02,0.),(.02,.1)]
        v,w,_=follower.update(route,(0.,0.,0.),route_age=0.,tf_age=0.)
        self.assertGreater(v,0.)
        self.assertEqual(w,0.)
        self.assertEqual(follower.cursor,2)

    def test_persistent_corner_survives_prefix_refresh_and_resets_on_hazard(self):
        follower=PathFollower()
        route=[(0.,0.),(.1,0.),(.1,.15)]
        follower.update(route,(.096,0.,0.),route_age=0.,tf_age=0.)
        refreshed=[(.09,-.003),(.1,0.),(.1,.15)]
        _,w,_=follower.update(refreshed,(.09,-.003,.5),route_age=0.,tf_age=0.)
        self.assertGreater(w,0.)
        follower.update(refreshed,(.09,-.003,.5),route_age=0.,tf_age=0.,blocked=True)
        _,w,_=follower.update(refreshed,(.09,-.003,.5),route_age=0.,tf_age=0.)
        self.assertLess(w,0.)

    def test_offset_pivot_closed_loop_with_periodic_route_prefix_refresh(self):
        follower=PathFollower()
        route=[(0.,0.),(.1,0.),(.1,.16)]
        pose=[0.,0.,0.]
        signs=[]
        maximum_error=0.
        for i in range(6500):
            if i%25==0:
                route=[tuple(pose[:2])]+route[1:]
            v,w,reason=follower.update(route,pose,route_age=0.,tf_age=0.)
            if reason=='arrived':break
            if abs(w)>.01:signs.append(1 if w>0 else -1)
            # Base-origin displacement includes the measured off-center pivot.
            dx=v+w*(-.0102)
            dy=-w*(-.0398)
            pose[0]+=(math.cos(pose[2])*dx-math.sin(pose[2])*dy)*.02
            pose[1]+=(math.sin(pose[2])*dx+math.cos(pose[2])*dy)*.02
            pose[2]+=w*.02
            maximum_error=max(maximum_error,min(abs(pose[1]),abs(pose[0]-.1)))
        self.assertEqual(reason,'arrived')
        self.assertLess(sum(a!=b for a,b in zip(signs,signs[1:])),8)
        self.assertLess(maximum_error,.06)

    def test_persistent_progress_does_not_survive_new_detour_or_stale_input(self):
        follower=PathFollower()
        route=[(0.,0.),(.1,0.),(.1,.15)]
        follower.update(route,(.096,0.,0.),route_age=0.,tf_age=0.)
        changed=[(.09,0.),(.08,0.),(.08,.05),(.1,.15)]
        expected=follow_path(changed,(.09,0.,0.),route_age=0.,tf_age=0.)
        self.assertEqual(follower.update(changed,(.09,0.,0.),route_age=0.,tf_age=0.),expected)
        follower.update(route,(.096,0.,0.),route_age=0.,tf_age=0.)
        follower.update(route,(.096,0.,0.),route_age=6.,tf_age=0.)
        self.assertIsNone(follower.cursor)

    def test_closed_loop_corner_keeps_tracking_error_within_five_mm(self):
        route = [(0., 0.), (.10, 0.), (.10, .16)]
        pose = [0., 0., 0.]
        largest_error = 0.
        for _ in range(3000):
            v, w, reason = follow_path(route, pose, route_age=0., tf_age=0.)
            if reason == 'arrived':
                break
            pose[0] += v*math.cos(pose[2])*.02
            pose[1] += v*math.sin(pose[2])*.02
            pose[2] += w*.02
            largest_error = max(largest_error, min(abs(pose[1]), abs(pose[0]-.10)))
        self.assertEqual(reason, 'arrived')
        # A 4cm corner shortcut consumes the planner's 5mm rotation margin.
        self.assertLessEqual(largest_error, .005)

    def test_localization_hold_freezes_instead_of_replenishing_push_budget(self):
        guard = ProgressGuard(timeout=8)
        guard.check(0., (0., 0., 0.), True)
        guard.pause(3., True)
        guard.pause(100., False)
        self.assertFalse(guard.check(100., (0., 0., 0.), True))
        self.assertTrue(guard.check(105., (0., 0., 0.), True))

    def test_small_repeated_oscillation_is_not_endless_progress(self):
        guard = ProgressGuard(timeout=8)
        for i in range(20):
            guard.check(i, (.006*(i % 2), 0, .06*(i % 2)), True)
        self.assertTrue(guard.stalled)

    def command(self, route=None, pose=(0, 0, 0), **kwargs):
        args = dict(route_age=0.1, tf_age=0.1)
        args.update(kwargs)
        return follow_path([(0, 0), (.3, 0)] if route is None else route, pose, **args)

    def test_valid_route_forward_respects_hardware_speed(self):
        v, w, reason = self.command()
        self.assertEqual(reason, 'forward')
        self.assertGreater(v, 0)
        self.assertLessEqual(v, .014)
        self.assertEqual(w, 0)

    def test_heading_alignment_never_advances(self):
        v, w, reason = self.command(pose=(0, 0, math.pi / 2))
        self.assertEqual((v, reason), (0, 'align'))
        self.assertLess(w, 0)
        self.assertGreaterEqual(w, -.10)

    def test_missing_empty_stale_and_future_inputs_stop(self):
        cases = [dict(route=[]), dict(pose=None), dict(route_age=6),
                 dict(tf_age=2), dict(route_age=-1), dict(tf_age=-1),
                 dict(route=[(math.nan, 0)]), dict(pose=(math.inf, 0, 0))]
        for values in cases:
            with self.subTest(values=values):
                self.assertEqual(self.command(**values)[:2], (0, 0))

    def test_hazard_always_stops_even_turn(self):
        self.assertEqual(self.command(pose=(0, 0, 2), blocked=True), (0, 0, 'hazard'))

    def test_arrival_stops(self):
        self.assertEqual(self.command(pose=(.29, 0, 0)), (0, 0, 'arrived'))

    def test_localization_jump_cannot_drive_across_to_distant_route(self):
        self.assertEqual(self.command(pose=(.1, .081, 0)), (0, 0, 'off_route'))
        self.assertGreater(self.command(pose=(.15, 0, 0))[0], 0)

    def test_brief_empty_route_cannot_replenish_stall_budget(self):
        guard = ProgressGuard(timeout=8)
        self.assertFalse(guard.check(0, (0, 0, 0), True))
        self.assertFalse(guard.check(7, (0, 0, 0), False))
        self.assertTrue(guard.check(8, (0, 0, 0), True))

    def test_corner_target_does_not_cut_diagonal(self):
        route = [(0, 0), (.1, 0), (.1, .1)]
        v, w, _ = self.command(route=route, pose=(.04, 0, 0), lookahead=.20)
        self.assertGreater(v, 0)
        self.assertEqual(w, 0)

    def test_stall_latches_across_new_route_and_requires_reset(self):
        guard = ProgressGuard(timeout=8)
        self.assertFalse(guard.check(0, (0, 0, 0), True))
        self.assertTrue(guard.check(8, (0, 0, 0), True))
        self.assertTrue(guard.check(9, (.3, 0, 0), False))
        guard.reset()
        self.assertFalse(guard.check(10, (.3, 0, 0), True))

    def test_slow_real_motion_and_turns_are_progress(self):
        guard = ProgressGuard(timeout=8)
        for i in range(30):
            self.assertFalse(guard.check(i, (i * .001, 0, 0), True))
        guard.reset()
        for i in range(30):
            self.assertFalse(guard.check(i, (0, 0, i * .02), True))
