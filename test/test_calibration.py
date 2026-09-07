import unittest

from move_control.control.calibration import StationaryBaseline, SENSORS, motion_evidence, motion_result


VALUES = {'lidar': (.65,), 'us': (.65,), 'odom': (0., 0., 0., 0.),
          'ir': (2200., 2210., 2190.), 'imu': (9.81, .01, 0., .001, .002, .003, 0., 0., 9.81, 0., 0.),
          'camera': (110., 20.), 'tf': (.174,), 'map': (200., .03),
          'map_tf': (0., 0., 0.)}


class CalibrationTest(unittest.TestCase):
    def baseline(self):
        baseline = StationaryBaseline()
        for i in range(21):
            for name in SENSORS:
                baseline.add(name, VALUES[name], i * .2)
        return baseline

    def test_all_sensor_baselines_need_samples_duration_and_freshness(self):
        baseline = self.baseline()
        self.assertTrue(all(item['ok'] for item in baseline.report(4.).values()))
        self.assertFalse(baseline.fresh(6.))
        self.assertFalse(any(item['ok'] for item in StationaryBaseline().report(4.).values()))

    def test_missing_map_blocks_even_with_robot_sensor_data(self):
        baseline = self.baseline()
        baseline.samples['map'] = []
        self.assertFalse(baseline.fresh(4.))
        self.assertEqual(baseline.report(4.)['map']['status'], 'stale')

    def test_saturated_ir_and_inverted_nose_tf_do_not_pass(self):
        for key, bad in (('ir', (4095, 4095, 4095)), ('tf', (3.14,))):
            baseline = self.baseline()
            for i in range(21):
                baseline.add(key, bad, 5 + i * .2)
            self.assertFalse(baseline.report(9.)[key]['ok'])

    def test_stationary_odom_movement_rejects_baseline(self):
        baseline = self.baseline()
        baseline.add('odom', (.02, 0., 0., .01), 4.1)
        self.assertFalse(baseline.report(4.1)['odom']['ok'])

    def test_positive_straight_motion_requires_both_range_sensors_to_agree(self):
        start = {key: VALUES[key] for key in ('odom', 'lidar', 'us', 'map_tf')}
        current = {'odom': (.03, 0., .01, 0.), 'lidar': (.62,), 'us': (.624,), 'map_tf': (.03, 0., .01)}
        evidence = motion_evidence(start, current)
        self.assertTrue(motion_result(evidence)[0])
        current['us'] = (.70,)
        self.assertFalse(motion_result(motion_evidence(start, current))[0])

    def test_reverse_stall_and_excess_yaw_never_become_ready(self):
        start = {key: VALUES[key] for key in ('odom', 'lidar', 'us', 'map_tf')}
        for odom in ((-.02, 0., 0., 0.), (0., 0., 0., 0.), (.03, 0., .4, 0.)):
            current = {'odom': odom, 'lidar': (.62,), 'us': (.62,), 'map_tf': (.03, 0., 0.)}
            self.assertFalse(motion_result(motion_evidence(start, current))[0])

    def test_stationary_map_pose_cannot_pass_even_if_odom_and_ranges_move(self):
        start = {key: VALUES[key] for key in ('odom', 'lidar', 'us', 'map_tf')}
        current = {'odom': (.03, 0., 0., 0.), 'lidar': (.62,), 'us': (.62,), 'map_tf': (0., 0., 0.)}
        passed, checks = motion_result(motion_evidence(start, current))
        self.assertFalse(passed)
        self.assertFalse(checks['map_pose_agrees'])
