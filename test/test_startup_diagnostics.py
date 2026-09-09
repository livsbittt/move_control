import unittest

from rosy_control.control.startup_diagnostics import StartupDiagnostics


class StartupDiagnosticsTest(unittest.TestCase):
    def test_logs_missing_then_recovered_sensor_without_sample_spam(self):
        logger = StartupDiagnostics()
        report = {'phase': 'collecting', 'ready': False,
                  'sensors': {'imu': {'status': 'stale', 'samples': 0,
                                      'detail': 'No fresh sensor sample'}}}
        self.assertEqual(logger.update(report)['details']['imu'], 'No fresh sensor sample')
        report['sensors']['imu']['samples'] = 1
        self.assertIsNone(logger.update(report))
        report['sensors']['imu']['status'] = 'ok'
        self.assertEqual(logger.update(report)['sensors']['imu'], 'ok')

    def test_mode_and_readiness_changes_are_logged(self):
        logger = StartupDiagnostics()
        report = {'phase': 'ready', 'ready': True}
        logger.update(report)
        report.update(mode='sensing_only', ready=False)
        event = logger.update(report)
        self.assertFalse(event['ready'])
        self.assertEqual(event['mode'], 'sensing_only')
