import math
import unittest

from move_control.control.calibration_relocation import CalibrationRelocation, _capsule_clear


def room(rear=-.14,front=.5,side=.4):
    points=[]
    for i in range(360):
        angle=-math.pi+i*math.tau/360
        x,y=math.cos(angle),math.sin(angle)
        distance=min(front/x if x>1e-12 else rear/x if x<-1e-12 else math.inf,
                     side/abs(y) if abs(y)>1e-12 else math.inf)
        points.append((distance*x,distance*y))
    return points


class CalibrationRelocationTest(unittest.TestCase):
    def test_bootstrap_requires_explicit_permission_and_bilateral_pilot_capsules(self):
        trial=CalibrationRelocation()
        self.assertEqual(self.update(trial,translation_verified=False)[0],0.)
        trial=CalibrationRelocation()
        v,_=self.update(trial,translation_verified=False,bootstrap_allowed=True)
        self.assertEqual(v,.003)
        self.assertFalse(trial.report()['bootstrap_verified'])
        trial=CalibrationRelocation()
        v,reason=self.update(trial,translation_verified=False,bootstrap_allowed=True,
                             points=room(rear=-.113))
        self.assertEqual(v,0.)
        self.assertEqual(reason,'bootstrap_bilateral_clearance')

    def test_bootstrap_confirms_direction_before_continuing_and_keeps_lower_speed(self):
        trial=CalibrationRelocation()
        self.update(trial,translation_verified=False,bootstrap_allowed=True)
        v,_=self.update(trial,.4,(.001,0.,0.),translation_verified=False,bootstrap_allowed=True)
        self.assertFalse(trial.report()['bootstrap_verified'])
        v,_=self.update(trial,.8,(.0021,0.,0.),translation_verified=False,bootstrap_allowed=True)
        self.assertTrue(trial.report()['bootstrap_verified'])
        self.assertGreater(v,0.)
        self.assertLessEqual(v,.003)
        self.assertEqual(trial.report()['max_duration_s'],75.)

    def test_bootstrap_wrong_direction_and_stationary_pilot_fail(self):
        trial=CalibrationRelocation()
        self.update(trial,translation_verified=False,bootstrap_allowed=True)
        self.assertEqual(self.update(trial,.1,(-.0021,0.,0.),translation_verified=False,bootstrap_allowed=True)[0],0.)
        self.assertEqual(trial.report()['error'],'reversed_motion')
        trial=CalibrationRelocation()
        commanded=0.
        v,_=self.update(trial,translation_verified=False,bootstrap_allowed=True)
        for i in range(1,31):
            commanded+=abs(v)*.1
            v,_=self.update(trial,i*.1,translation_verified=False,bootstrap_allowed=True)
        self.assertLessEqual(commanded,.0041)
        self.assertEqual(trial.report()['error'],'bootstrap_no_progress')

    def update(self,trial,t=0.,pose=(0.,0.,0.),**overrides):
        args=dict(points=room(),full_scan_observed=True,body_radius=.1,
                  rotation_radius=.18,fresh_guard=True,translation_verified=True,
                  rotation_clear_current=False)
        args.update(overrides)
        return trial.update(t,pose,**args)

    def test_selects_once_and_completes_only_on_fresh_stable_clearance(self):
        trial=CalibrationRelocation()
        self.assertEqual(self.update(trial)[0],.006)
        self.assertEqual(trial.report()['direction'],1)
        for i in range(1,9):
            self.update(trial,i*.1,(i*.005,0.,0.),points=room(-.14-i*.005,.5-i*.005))
        self.assertFalse(trial.report()['done'])
        for i in range(9,15):
            v,_=self.update(trial,i*.1,(.045,0.,0.),points=room(-.185,.455),rotation_clear_current=True)
            self.assertEqual(v,0.)
        self.assertTrue(trial.report()['done'])

    def test_freshness_geometry_and_reverse_motion_fail_permanently(self):
        for override in (dict(full_scan_observed=False),dict(fresh_guard=False),
                         dict(translation_verified=False),dict(points=[])):
            trial=CalibrationRelocation()
            self.assertEqual(self.update(trial,**override)[0],0.)
            self.assertIsNotNone(trial.report()['error'])
            self.assertEqual(self.update(trial,.1)[0],0.)
        for t,pose in ((.6,(0.,0.,0.)),(-.1,(0.,0.,0.)),(.1,(-.003,0.,0.)),
                       (.1,(.1,0.,0.)),(.1,(0.,.011,0.)),(.1,(0.,0.,.051))):
            trial=CalibrationRelocation();self.update(trial)
            self.assertEqual(self.update(trial,t,pose)[0],0.)
            self.assertIsNotNone(trial.report()['error'])

    def test_no_exit_and_new_obstacle_stop(self):
        trial=CalibrationRelocation()
        self.assertEqual(self.update(trial,points=room(-.14,.14,.14))[0],0.)
        self.assertEqual(trial.report()['error'],'no_observed_candidate')
        trial=CalibrationRelocation();self.update(trial)
        self.assertEqual(self.update(trial,.1,points=room(-.14,.105))[0],0.)
        self.assertEqual(trial.report()['error'],'straight_capsule_blocked')

    def test_timeout_and_no_current_clearance_at_target(self):
        trial=CalibrationRelocation();self.update(trial)
        for i in range(1,451):
            self.update(trial,i*.1)
        self.assertEqual(trial.report()['error'],'time_budget')
        trial=CalibrationRelocation();self.update(trial)
        target=trial.report()['target_estimate_m']
        for i in range(1,int(target/.005)+2):
            result=self.update(trial,i*.1,(min(i*.005,target),0.,0.))
        self.assertEqual(result[0],0.)
        self.assertEqual(trial.report()['error'],'target_not_clear')

    def test_occlusion_edge_blocks_capsule_even_when_endpoints_are_distant(self):
        polygon=[(-.4,-.4),(.115,-.4),(.115,.4),(-.4,.4)]
        self.assertTrue(all(math.hypot(x,y)>.4 for x,y in polygon))
        self.assertFalse(_capsule_clear(polygon,.01,.11))
        self.assertTrue(_capsule_clear(polygon,-.01,.11))

    def test_partial_order_or_clearance_transient_never_completes(self):
        trial=CalibrationRelocation()
        self.assertEqual(self.update(trial,points=room()[::30])[0],0.)
        trial=CalibrationRelocation()
        self.update(trial,rotation_clear_current=True)
        self.update(trial,.4,rotation_clear_current=True)
        self.assertFalse(trial.done)
        self.update(trial,.5,rotation_clear_current=False)
        self.update(trial,.6,rotation_clear_current=True)
        self.update(trial,1.,rotation_clear_current=True)
        self.assertFalse(trial.done)
