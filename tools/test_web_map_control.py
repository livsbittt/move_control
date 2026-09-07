"""ROS-backed types, isolated fake services: no robot or motion nodes."""
import http.client
import http.server
import json
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from rclpy.task import Future
from rcl_interfaces.msg import ParameterValue, ParameterType
from rcl_interfaces.srv import GetParameters
from slam_toolbox.srv import Reset, Pause
from nav_msgs.msg import OccupancyGrid
from move_control import web_node as web


class Service:
    def __init__(self, fn):
        self.fn = fn
        self.ready = True
        self.calls = []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.calls.append(request)
        future = Future()
        future.set_result(self.fn(request))
        return future


class MapControlTest(unittest.TestCase):
    def setUp(self):
        web.STATE.clear()
        web.MAP_PNG.update(bytes=b'old', gen=10)
        self.paused = True
        self.node = SimpleNamespace(
            wander_pub=Mock(), estop_pub=Mock(), goal_pub=Mock(), calibration_pub=Mock(),
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=100)))
        self.control = web.MapControl.__new__(web.MapControl)
        self.control.node = self.node
        self.control.lock = threading.Lock()
        self.control.pending = None
        self.control.map_after_ns = 0
        self.control.reset = Service(lambda req: Reset.Response(result=0))
        self.control.pause = Service(self.toggle)
        self.control.params = Service(lambda req: GetParameters.Response(values=[
            ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=self.paused)]))
        self.control.update(available=True, paused=True, busy=False, error='', epoch=0)
        self.node.map_control = self.control

    def toggle(self, request):
        self.paused = not self.paused
        return Pause.Response(status=True)

    def test_reset_clears_cache_and_sends_only_stops(self):
        web.STATE.update(map=[1], goal_pt=[1], route=[1], options=[1], trail=[1], path_m=3)
        status, result = self.control.execute('reset')
        self.assertEqual(status, 200)
        self.assertTrue(result['ok'])
        self.assertTrue(self.control.reset.calls[0].pause_new_measurements)
        for pub in (self.node.wander_pub, self.node.estop_pub):
            self.assertEqual(pub.publish.call_args.args[0].data, 'stop')
        self.assertEqual([c.args[0].data for c in self.node.goal_pub.publish.call_args_list],
                         ['stop', 'reset'])
        for key in ('map', 'goal_pt', 'route', 'options', 'trail'):
            self.assertNotIn(key, web.STATE)
        self.assertIsNone(web.MAP_PNG['bytes'])
        self.assertEqual(web.MAP_PNG['gen'], 11)
        self.assertEqual(web.STATE['map_control']['epoch'], 1)

    def test_duplicate_resume_does_not_toggle_back_or_start_motors(self):
        self.assertEqual(self.control.execute('resume')[0], 200)
        self.assertEqual(self.control.execute('resume')[0], 200)
        self.assertEqual(len(self.control.pause.calls), 1)
        self.assertFalse(self.paused)
        for pub in (self.node.wander_pub, self.node.estop_pub, self.node.goal_pub):
            pub.publish.assert_not_called()

    def test_missing_reset_service_retains_map_and_fails(self):
        self.control.reset.ready = False
        status, result = self.control.execute('reset')
        self.assertEqual(status, 503)
        self.assertFalse(result['ok'])
        self.assertEqual(web.MAP_PNG['bytes'], b'old')

    def test_unknown_paused_state_never_toggles(self):
        self.control.params = Service(lambda req: GetParameters.Response())
        self.assertEqual(self.control.execute('resume')[0], 409)
        self.assertEqual(self.control.pause.calls, [])

    def test_pending_timeout_cannot_be_retried_as_second_toggle(self):
        future = Future()
        self.control.pause.call_async = Mock(return_value=future)
        real_event = threading.Event
        with patch.object(web.threading, 'Event') as event_factory:
            def immediate_event():
                event = real_event()
                event.wait = Mock(side_effect=lambda timeout: event.is_set())
                return event
            event_factory.side_effect = immediate_event
            status, _ = self.control.execute('resume')
        self.assertEqual(status, 504)
        self.assertEqual(self.control.execute('resume')[0], 409)
        self.control.pause.call_async.assert_called_once()
        # Once the late response arrives, a retry queries the new state and
        # does not invert it a second time.
        self.paused = False
        future.set_result(Pause.Response(status=True))
        self.assertEqual(self.control.execute('resume')[0], 200)
        self.control.pause.call_async.assert_called_once()

    def test_queued_old_map_cannot_restore_reset_image(self):
        self.control.execute('reset')
        msg = OccupancyGrid()
        msg.info.width = msg.info.height = 1
        msg.data = [0]
        web.WebNode.on_map(self.node, msg)
        self.assertIsNone(web.MAP_PNG['bytes'])
        self.control.update(paused=False)
        web.WebNode.on_map(self.node, msg)
        self.assertIsNone(web.MAP_PNG['bytes'])

    def test_http_missing_service_returns_error_json(self):
        self.control.reset.ready = False
        server = http.server.ThreadingHTTPServer(('127.0.0.1', 0),
                                                 web.make_api_handler(self.node, b''))
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            client = http.client.HTTPConnection(*server.server_address)
            client.request('POST', '/map/reset', body='')
            response = client.getresponse()
            self.assertEqual(response.status, 503)
            self.assertFalse(json.loads(response.read())['ok'])
            client.close()
        finally:
            server.shutdown()
            server.server_close()
            thread.join()

    def test_manual_goal_activates_follower_only_when_ready_and_released(self):
        server = http.server.ThreadingHTTPServer(('127.0.0.1', 0),
                                                 web.make_api_handler(self.node, b''))
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            for ready, released, expected in ((False, True, 409),
                                               (True, False, 409),
                                               (True, True, 200)):
                web.STATE.update(calibration_ready=ready,
                                 calibration_received=time.monotonic(),
                                 calibration={'ready':ready})
                web.STATE[web.K_ESTOP] = not released
                client = http.client.HTTPConnection(*server.server_address)
                client.request('POST', '/goal', body='0.2,0.1')
                response = client.getresponse()
                self.assertEqual(response.status, expected)
                response.read()
                client.close()
            self.node.wander_pub.publish.assert_called_once()
            self.assertEqual(self.node.wander_pub.publish.call_args.args[0].data,
                             'manual:0.200,0.100')
            self.node.goal_pub.publish.assert_not_called()
        finally:
            server.shutdown()
            server.server_close()
            thread.join()
