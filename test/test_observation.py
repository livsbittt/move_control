import unittest

from move_control.sensing.observation import Observations


class ObservationTest(unittest.TestCase):
    def test_replayed_packets_do_not_refresh_a_measurement(self):
        samples = Observations()
        self.assertTrue(samples.add('scan', 10., source=100., source_now=100.))
        self.assertFalse(samples.add('scan', 10.8, source=100., source_now=100.8))
        self.assertEqual(samples.generation('scan'), 1)
        self.assertFalse(samples.fresh('scan', 11.1))
        self.assertTrue(samples.add('scan', 11.2, source=101.2, source_now=101.2))
        self.assertTrue(samples.fresh('scan', 11.3))

    def test_source_age_and_receive_age_both_count(self):
        samples = Observations()
        self.assertFalse(samples.add('scan', 10., source=90., source_now=100.))
        self.assertFalse(samples.add('scan', 10., source=101., source_now=100.))
        self.assertTrue(samples.add('scan', 10., source=99.5, source_now=100.))
        self.assertTrue(samples.fresh('scan', 10.2))
        self.assertFalse(samples.fresh('scan', 10.6))

    def test_invalid_and_independent_camera_streams(self):
        samples = Observations()
        samples.add('camera_cliff', 10.)
        samples.add('camera_block', 12.)
        self.assertFalse(samples.fresh('camera_cliff', 12.))
        self.assertTrue(samples.fresh('camera_block', 12.))
        samples.add('ir', 12., valid=False)
        self.assertFalse(samples.fresh('ir', 12.))
        self.assertEqual(samples.report(12.)['ir']['status'], 'invalid')
        self.assertEqual(samples.report(12.)['camera_block']['clock'], 'receive_only')

    def test_nonfinite_and_backward_receive_clock_fail_closed(self):
        samples = Observations()
        self.assertFalse(samples.add('imu', float('nan')))
        samples.add('imu', 10.)
        self.assertFalse(samples.fresh('imu', 9.))
