import unittest

from move_control.control.safe_trail import SafeTrail


class SafeTrailTest(unittest.TestCase):
    def test_stable_refuge_records_exact_pose_not_prior_unsafe_breadcrumb(self):
        trail=SafeTrail()
        trail.observe(0.,(0.,0.,0.),True,False,'body')
        for i in range(1,8):
            trail.observe(i*.1,(.009,0.,0.),True,True,'body')
        self.assertFalse(trail.samples[0]['refuge'])
        self.assertTrue(trail.samples[-1]['refuge'])
        self.assertEqual(trail.samples[-1]['pose'],(.009,0.,0.))
        trail.observe(.8,(.025,0.,0.),True,False,'body')
        self.assertEqual(trail.retreat(.8,(.025,0.,0.),'body')[-1],(.009,0.,0.))

    def test_heading_jump_discards_history_and_heading_mismatch_rejects_route(self):
        trail=SafeTrail()
        for i in range(7):
            trail.observe(i*.1,(0.,0.,0.),True,True,'body')
        trail.observe(.7,(.01,0.,0.),True,False,'body')
        self.assertIsNone(trail.retreat(.7,(.01,0.,.06),'body'))
        trail.observe(.8,(.02,0.,.5),True,False,'body')
        self.assertIsNone(trail.retreat(.8,(.02,0.,.5),'body'))

    def test_narrow_entry_retreats_to_stable_turn_refuge(self):
        trail=SafeTrail()
        for i in range(7):
            trail.observe(i*.1,(0.,0.,0.),True,True,'body')
        for i in range(1,6):
            trail.observe(.6+i*.1,(i*.01,0.,0.),True,False,'body')
        route=trail.retreat(1.1,(.05,0.,0.),'body')
        self.assertIsNotNone(route)
        self.assertEqual(route[-1],(0.,0.,0.))
        self.assertGreater(route[0][0],route[-1][0])
        self.assertIsNone(trail.retreat(1.1,(.05,0.,0.),'body',.02))

    def test_transient_clearance_is_not_a_refuge(self):
        trail=SafeTrail()
        for i in range(5):
            trail.observe(i*.1,(0.,0.,0.),True,True,'body')
        trail.observe(.5,(.01,0.,0.),True,False,'body')
        self.assertIsNone(trail.retreat(.5,(.01,0.,0.),'body'))

    def test_discontinuity_discards_previous_refuge(self):
        for now,pose,valid,geometry in [(1.,(.01,0.,0.),True,'other'),
                                       (.3,(.01,0.,0.),True,'body'),
                                       (1.2,(.01,0.,0.),True,'body'),
                                       (.7,(.2,0.,0.),True,'body'),
                                       (.7,(float('nan'),0.,0.),True,'body'),
                                       (.7,(.01,0.,0.),False,'body')]:
            trail=SafeTrail()
            for i in range(7):
                trail.observe(i*.1,(0.,0.,0.),True,True,'body')
            trail.observe(now,pose,valid,False,geometry)
            self.assertIsNone(trail.retreat(now,pose,geometry))

    def test_freshness_pose_and_geometry_gate_retrieval(self):
        trail=SafeTrail()
        for i in range(7):
            trail.observe(i*.1,(0.,0.,0.),True,True,'body')
        trail.observe(.7,(.01,0.,0.),True,False,'body')
        for now,pose,geometry in [(1.3,(.01,0.,0.),'body'),(.6,(.01,0.,0.),'body'),
                                  (.7,(.04,0.,0.),'body'),(.7,(.01,0.,0.),'other')]:
            self.assertIsNone(trail.retreat(now,pose,geometry))

    def test_storage_caps_drop_old_refuges(self):
        trail=SafeTrail()
        for i in range(7):
            trail.observe(i*.1,(0.,0.,0.),True,True,'body')
        for i in range(1,401):
            trail.observe(.6+i*.1,(i*.01,0.,0.),True,False,'body')
        self.assertLessEqual(len(trail.samples),256)
        self.assertLessEqual(trail.samples[-1]['pose'][0]-trail.samples[0]['pose'][0],2.+1e-9)
        self.assertIsNone(trail.retreat(40.6,(4.,0.,0.),'body'))
