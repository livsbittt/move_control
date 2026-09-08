"""HTTP calibration interlocks using fake publishers, no robot graph."""
import http.client
import http.server
import json
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from move_control import web_node as web


class CalibrationHttpTest(unittest.TestCase):
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
        self.node = SimpleNamespace(wander_pub=Mock(), teleop_pub=Mock(), calibration_pub=Mock(), estop_pub=Mock())
        self.server = http.server.ThreadingHTTPServer(('127.0.0.1', 0), web.make_api_handler(self.node, b''))
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

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
