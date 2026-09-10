"""Exclusion tiers decide what a partial calibration may drop; nothing else may."""
import unittest

from rosy_control.control.calibration import SENSORS
from rosy_control.control.sensor_tiers import EXCLUDABLE, REQUIRED_FOR_MOTION, partial_plan


class SensorTierTests(unittest.TestCase):
    def healthy(self):
        return {name: {'eligible': True} for name in SENSORS}

    def failing(self, *names):
        sensors = self.healthy()
        for name in names:
            sensors[name]['eligible'] = False
        return sensors

    def test_tiers_do_not_overlap_and_stay_inside_the_sensor_list(self):
        self.assertEqual(set(EXCLUDABLE) & set(REQUIRED_FOR_MOTION), set())
        for name in EXCLUDABLE + REQUIRED_FOR_MOTION:
            self.assertIn(name, SENSORS)

    def test_no_failure_leaves_nothing_to_exclude(self):
        plan = partial_plan(self.healthy())
        self.assertFalse(plan['available'])
        self.assertEqual(plan['reason'], 'no_failed_sensor')

    def test_camera_failure_keeps_rotation_available(self):
        plan = partial_plan(self.failing('camera'))
        self.assertTrue(plan['available'])
        self.assertEqual(plan['excluded_sensors'], ('camera',))
        self.assertTrue(plan['rotation_available'])

    def test_imu_failure_withdraws_rotation(self):
        plan = partial_plan(self.failing('imu'))
        self.assertTrue(plan['available'])
        self.assertEqual(plan['excluded_sensors'], ('imu',))
        self.assertFalse(plan['rotation_available'])

    def test_every_motion_sensor_blocks_partial_calibration(self):
        for name in REQUIRED_FOR_MOTION:
            plan = partial_plan(self.failing(name))
            self.assertFalse(plan['available'], name)
            self.assertEqual(plan['reason'], 'required_sensor_failed')
            self.assertIn(name, plan['blocking_sensors'])

    def test_map_blocks_only_when_localization_is_required(self):
        self.assertTrue(partial_plan(self.failing('map', 'imu'))['available'])
        blocked = partial_plan(self.failing('map', 'imu'), localization_required=True)
        self.assertFalse(blocked['available'])
        self.assertIn('map', blocked['blocking_sensors'])

    def test_ok_is_accepted_when_eligible_is_absent(self):
        self.assertEqual(partial_plan({name: {'ok': True} for name in SENSORS})['reason'], 'no_failed_sensor')


if __name__ == '__main__':
    unittest.main()
