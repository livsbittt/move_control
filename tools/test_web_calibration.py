"""HTTP calibration interlocks using fake publishers, no robot graph."""
import http.client
import http.server
import json
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from rosy_control import web_node as web


class CalibrationHttpTest(unittest.TestCase):
    def test_limited_sensors_relay_does_not_unlock_motion_or_release_estop(self):
        web.STATE.update(calibration_ready=True, calibration={'ready': True},
                         calibration_received=time.monotonic(), estop=False)
        self.assertEqual(self.post('/calibration', 'use_limited_sensors'), 200)
        self.assertEqual(self.node.calibration_pub.publish.call_args.args[0].data, 'use_limited_sensors')
        self.assertFalse(web.STATE['calibration_ready'])
        self.assertEqual(self.post('/wander', 'start'), 409)
        self.assertEqual(self.post('/teleop', '{"x":0.01}'), 409)
        self.node.estop_pub.publish.assert_not_called()

    def test_existing_settings_requires_runtime_ready_and_preserves_estop(self):
        web.STATE.update(calibration_ready=True, calibration={'ready': True},
                         calibration_received=time.monotonic(), estop=False)
        self.assertEqual(self.post('/calibration', 'use_existing_settings'), 200)
        self.assertEqual(self.node.calibration_pub.publish.call_args.args[0].data, 'use_existing_settings')
        self.assertFalse(web.STATE['calibration_ready'])
        self.assertEqual(self.post('/wander', 'start'), 409)
        web.STATE.update(calibration_ready=True, calibration_received=time.monotonic(),
                         calibration={'ready': True, 'mode': 'existing_settings',
                                      'calibration_verified': False, 'operating_ready': True})
        self.assertEqual(self.post('/wander', 'start'), 200)
        web.STATE['estop'] = True
        self.assertEqual(self.post('/wander', 'start'), 409)
        self.assertEqual(self.post('/teleop', '{"x":0.01}'), 409)
        self.node.estop_pub.publish.assert_not_called()
        self.assertEqual(self.post('/teleop', '{}'), 200)
        web.STATE['calibration_ready'] = False
        self.assertEqual(self.post('/estop', 'release'), 200)
        self.node.estop_pub.publish.assert_called_once()

    def test_sensing_only_invalidates_ready_and_never_releases_estop_or_motion(self):
        web.STATE.update(calibration_ready=True, calibration={'ready': True},
                         calibration_received=time.monotonic(), estop=False)
        self.assertEqual(self.post('/calibration', 'sensing_only'), 200)
        self.assertEqual(self.node.calibration_pub.publish.call_args.args[0].data, 'sensing_only')
        self.assertFalse(web.STATE['calibration_ready'])
        for mode in ('start', 'explore', 'coverage'):
            self.assertEqual(self.post('/wander', mode), 409)
        self.assertEqual(self.post('/teleop', '{"x":0.01}'), 409)
        self.assertEqual(self.post('/calibration', 'validate_motion'), 409)
        self.node.estop_pub.publish.assert_not_called()
        self.node.teleop_pub.publish.assert_not_called()
        self.assertEqual(self.post('/calibration', 'retry'), 200)
        self.assertFalse(web.STATE['calibration_ready'])

    def test_calibration_after_position_option_is_relayed_without_motion_unlock(self):
        for option in ('stay','return_origin'):
            self.assertEqual(self.post('/calibration','retry:'+option), 200)
            self.assertEqual(self.node.calibration_pub.publish.call_args.args[0].data, 'retry:'+option)
        self.assertEqual(self.post('/calibration','retry:anywhere'), 400)
        self.node.estop_pub.publish.assert_not_called()

    def test_navigation_session_requires_fresh_idle_and_valid_budgets(self):
        payload = json.dumps(dict(strategy='nearest', duration_s=60, stall_s=30))
        self.assertEqual(self.post('/navigation/start', payload), 409)
        web.STATE.update(calibration_ready=True, calibration={'ready': True},
                         calibration_received=time.monotonic(), estop=False,
                         wander='stop', navigation_session={'active':False},
                         navigation_session_received=time.monotonic())
        self.assertEqual(self.post('/navigation/start', payload), 202)
        command = self.node.wander_pub.publish.call_args.args[0].data
        self.assertEqual(json.loads(command.partition(':')[2])['strategy'], 'nearest')
        self.assertEqual(self.post('/navigation/start', payload.replace('60', 'true')), 400)
        web.STATE['navigation_session']['active'] = True
        self.assertEqual(self.post('/navigation/start', payload), 409)
        web.STATE['navigation_session']['active'] = False
        web.STATE['navigation_session_received'] -= 2
        self.assertEqual(self.post('/navigation/start', payload), 409)

    def test_display_limits_come_from_valid_effective_profile(self):
        packet = {'valid': True, 'revision': 'test',
                  'effective': {'stop': .15, 'clear': .17, 'radius': .09}}
        web.WebNode.on_safety_profile(self.node, SimpleNamespace(data=json.dumps(packet)))
        self.assertEqual(web.STATE[web.K_LIMITS]['stop'], .15)
        web.WebNode.on_safety_profile(self.node, SimpleNamespace(data='[]'))
        self.assertFalse(web.STATE['safety_profile']['valid'])

    def setUp(self):
        web.STATE.clear()
        web.STATE['planner_received'] = time.monotonic()
        web.STATE['calibration_received'] = time.monotonic()
        self.node = SimpleNamespace(wander_pub=Mock(), teleop_pub=Mock(), calibration_pub=Mock(), estop_pub=Mock())
        self.node.calibration_pub.get_subscription_count.return_value = 1
        self.server = http.server.ThreadingHTTPServer(('127.0.0.1', 0), web.make_api_handler(self.node, b''))
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def test_cold_calibration_receiver_returns_503_without_silent_command_loss(self):
        web.STATE.pop('calibration_received')
        for command in ('retry:stay', 'sensing_only', 'use_existing_settings', 'use_limited_sensors', 'validate_motion'):
            self.assertEqual(self.post('/calibration', command), 503)
        self.node.calibration_pub.publish.assert_not_called()
        self.assertEqual(self.post('/calibration', 'abort'), 200)

    def test_receiver_discovery_and_heartbeat_required_but_baseline_not_required(self):
        web.STATE.update(calibration_ready=False, calibration={'phase':'failed','ready':False})
        self.node.calibration_pub.get_subscription_count.return_value = 0
        self.assertEqual(self.post('/calibration', 'retry'), 503)
        self.node.calibration_pub.get_subscription_count.return_value = 1
        web.STATE['calibration_received'] = time.monotonic()-4.
        self.assertEqual(self.post('/calibration', 'retry'), 503)
        web.STATE['calibration_received'] = time.monotonic()
        self.assertEqual(self.post('/calibration', 'retry'), 200)

    def test_missing_or_stale_planner_rejects_map_starts_but_not_reactive_driving(self):
        web.STATE.update(calibration_ready=True, calibration={'ready': True},
                         calibration_received=time.monotonic(), estop=False, wander='stop',
                         navigation_session={'active': False}, navigation_session_received=time.monotonic())
        for stamp in (None, time.monotonic()-6.):
            if stamp is None:
                web.STATE.pop('planner_received', None)
            else:
                web.STATE['planner_received'] = stamp
            self.assertEqual(self.post('/wander', 'explore'), 409)
            self.assertEqual(self.post('/wander', 'coverage'), 409)
            self.assertEqual(self.post('/navigation/start', json.dumps(dict(strategy='gain',duration_s=60,stall_s=30))), 409)
            self.assertEqual(self.post('/goal', '0.2,0.1'), 409)
        self.assertEqual(self.post('/wander', 'start'), 200)
        web.WebNode.on_gstate(self.node, SimpleNamespace(data='stopped'))
        self.assertEqual(self.post('/wander', 'explore'), 200)

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()

    def post(self, path, body):
        client = http.client.HTTPConnection(*self.server.server_address)
        client.request('POST', path, body=body)
        response = client.getresponse()
        response.read()
        client.close()
        return response.status

    def test_start_and_nonzero_teleop_fail_before_ready_but_stops_pass(self):
        for mode in ('start', 'explore', 'coverage'):
            self.assertEqual(self.post('/wander', mode), 409)
        self.assertEqual(self.post('/teleop', '{"x":0.01}'), 409)
        self.assertEqual(self.post('/teleop', '{"z":0.1}'), 409)
        self.node.wander_pub.publish.assert_not_called()
        self.node.teleop_pub.publish.assert_not_called()
        self.assertEqual(self.post('/wander', 'stop'), 200)
        self.assertEqual(self.post('/teleop', '{}'), 200)
        self.assertEqual(self.post('/calibration', 'abort'), 200)

    def test_ready_requires_status_and_latched_ready(self):
        web.STATE['calibration'] = {'ready': True}
        self.assertEqual(self.post('/wander', 'start'), 409)
        web.STATE['calibration_ready'] = True
        web.STATE['calibration_received'] = time.monotonic()
        self.assertEqual(self.post('/wander', 'start'), 409)
        web.STATE['estop'] = False
        self.assertEqual(self.post('/wander', 'start'), 200)
        self.assertEqual(self.post('/teleop', '{"x":0.01}'), 200)
        web.STATE['calibration_received'] = time.monotonic() - 4
        self.assertEqual(self.post('/teleop', '{"x":0.01}'), 409)
        web.STATE['calibration_received'] = time.monotonic()
        self.assertEqual(self.post('/calibration', 'retry'), 200)
        self.assertEqual(self.post('/wander', 'start'), 409)

    def test_mode_selection_is_explicit_and_relays_exactly_the_clicked_mode(self):
        web.STATE.update(calibration_ready=True, calibration={'ready': True},
                         calibration_received=time.monotonic(), estop=False)
        self.node.wander_pub.publish.assert_not_called()
        for mode in ('start', 'explore', 'coverage'):
            self.assertEqual(self.post('/wander', mode), 200)
            self.assertEqual(self.node.wander_pub.publish.call_args.args[0].data, mode)

    def test_motion_requires_correct_phase_release_and_active_mapping(self):
        web.STATE['calibration'] = {'phase': 'waiting_motion'}
        self.assertEqual(self.post('/calibration', 'validate_motion'), 409)
        web.STATE['estop'] = False
        web.STATE['map_control'] = {'paused': True}
        self.assertEqual(self.post('/calibration', 'validate_motion'), 409)
        web.STATE['map_control']['paused'] = False
        self.assertEqual(self.post('/calibration', 'validate_motion'), 200)
        self.node.estop_pub.publish.assert_not_called()
        web.STATE['calibration']['phase'] = 'collecting'
        self.assertEqual(self.post('/calibration', 'validate_motion'), 409)

    def test_status_callback_and_malformed_data_fail_closed(self):
        status = {'phase': 'waiting_motion', 'ready': False,
                  'sensors': {'lidar': {'ok': True, 'samples': 8}}}
        web.WebNode.on_calibration_status(self.node, SimpleNamespace(data=json.dumps(status)))
        self.assertEqual(web.STATE['calibration'], status)
        web.WebNode.on_calibration_ready(self.node, SimpleNamespace(data=True))
        web.WebNode.on_calibration_status(self.node, SimpleNamespace(data='[]'))
        self.assertFalse(web.STATE['calibration_ready'])
