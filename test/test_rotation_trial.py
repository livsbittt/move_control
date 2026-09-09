import math
import unittest
import numpy as np
from rosy_control.sensing.scan_rotation import scan_rotation
from rosy_control.control.rotation_trial import RotationTrial


class RotationTrialTest(unittest.TestCase):
    def test_gain_above_one_repeats_within_trial_command_limit(self):
        trial=RotationTrial(0.)
        yaw=speed=0.
        for i in range(1,1400):
            yaw+=speed*.05/1.1
            speed=trial.update(i*.05,yaw,yaw,yaw,0.,True)
            self.assertLessEqual(abs(speed),.06)
            if trial.done or trial.error:
                break
        self.assertIsNone(trial.error)
        self.assertTrue(trial.done)
        self.assertEqual(len(trial.legs),8)
        for leg in trial.legs:
            self.assertAlmostEqual(leg['ratio'],1.1,places=8)
        self.assertLess(abs(yaw),.025)

    def test_scan_yaw_sign_and_unobservable_circle(self):
        angles = np.arange(720)*math.pi/360
        reference = .7 + .15*np.sin(3*angles) + .1*np.cos(7*angles)
        result = scan_rotation(reference, np.roll(reference, -20), math.pi/360)
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result['yaw'], math.radians(10))
        self.assertIsNone(scan_rotation(np.ones(720), np.ones(720), math.pi/360))

    def test_repeat_verifies_both_directions_and_returns_home(self):
        trial = RotationTrial(0.)
        yaw = speed = 0.
        for i in range(1, 1400):
            now = i*.05
            yaw += speed*.05*.92
            speed = trial.update(now, yaw, yaw, yaw, 0., True)
            if trial.done or trial.error:
                break
        self.assertIsNone(trial.error)
        self.assertTrue(trial.done)
        self.assertEqual(len(trial.legs), 8)
        self.assertLess(abs(yaw), .025)
        self.assertTrue(all(.75 <= value <= 1.25 for value in trial.scales))

    def test_lost_clearance_disagreement_and_stall_stop(self):
        for kwargs in ((.05, 0., 0., 0., 0., False),
                       (.05, .1, -.1, .1, 0., True),
                       (.05, 0., 0., 0., .03, True)):
            trial = RotationTrial(0.)
            self.assertEqual(trial.update(*kwargs), 0.)
            self.assertIsNotNone(trial.error)
