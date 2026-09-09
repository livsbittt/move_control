import math
import unittest
import numpy as np
from rosy_control.sensing.scan_rotation import scan_rotation
from rosy_control.control.rotation_trial import RotationTrial, steady_response


class RotationTrialTest(unittest.TestCase):
    def test_quantized_scan_and_startup_delay_validate_steady_response_without_compensation(self):
        trial = RotationTrial(0.)
        yaw = speed = 0.
        moving_since = None
        for i in range(1, 1400):
            now = i*.05
            if speed:
                if moving_since is None:
                    moving_since = now
                if now-moving_since >= .4:
                    yaw += speed*.05/1.12
            else:
                moving_since = None
            # Real scan alignment has 0.5-degree bins and arrives at 10 Hz.
            if i % 2 == 0:
                lidar = round(yaw/math.radians(.5))*math.radians(.5)
            elif i == 1:
                lidar = 0.
            speed = trial.update(now, lidar, yaw, yaw, 0., True)
            if trial.done or trial.error:
                break
        self.assertIsNone(trial.error)
        self.assertTrue(trial.done)
        report = trial.report()
        self.assertTrue(report['response_verified'])
        self.assertFalse(report['compensation_verified'])
        self.assertEqual(report['angular_gains'], [1., 1.])
        from rosy_control.control.calibration_profile import ProfileLease, make_profile
        lease = ProfileLease()
        packet = make_profile('latency-test', 1, 100., True, (1., 1.), 'geometry', report)
        self.assertTrue(lease.accept(packet, 100., 10., 'geometry'))
        self.assertEqual(lease.angular_gains(10.), (1., 1.))
        self.assertFalse(lease.active['rotation']['compensation_verified'])
        self.assertEqual(len(report['legs']), 8)
        self.assertTrue(any(leg['integrated_ratio'] > 1.25 for leg in report['legs']))
        for leg in report['legs']:
            self.assertAlmostEqual(leg['ratio'], 1.12, delta=.08)
            self.assertGreater(leg['onset_latency_s'], .2)
            self.assertGreaterEqual(leg['fit_span_rad'], math.radians(5))

    def test_steady_fit_rejects_insufficient_span_and_sensor_rate_disagreement(self):
        rows = [(i*.1, i*.005, i*.005, i*.005) for i in range(10)]
        self.assertIsNone(steady_response(rows, 0., 1, 0.))
        rows = [(i*.1, i*.005, i*.008, i*.005) for i in range(30)]
        self.assertIsNone(steady_response(rows, 0., 1, 0.))

    def test_rejected_gain_retains_evidence_without_accepting_failed_leg(self):
        trial = RotationTrial(0.)
        yaw = speed = 0.
        for i in range(1, 200):
            yaw += speed*.05/1.4
            speed = trial.update(i*.05, yaw, yaw, yaw, 0., True)
            if trial.error:
                break
        self.assertEqual(trial.error, 'Rotation correction outside trial bounds')
        self.assertEqual(speed, 0.)
        self.assertEqual(trial.legs, [])
        evidence = trial.report()['failed_leg']
        self.assertAlmostEqual(evidence['ratio'], 1.4)
        self.assertAlmostEqual(evidence['commanded_rad']/evidence['measured_rad'], 1.4)
        self.assertEqual(evidence['index'], 0)
        self.assertEqual(evidence['imu_yaw_rad'], yaw)
        trial.update(20., 0., 0., 0., 0., True)
        self.assertEqual(trial.report()['failed_leg'], evidence)

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
