"""ROS-environment regression tests for the mapping run monitor."""
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from tools.gz.map_run_monitor import Mon


class CompletionHoldTest(unittest.TestCase):
    def test_reexploration_restarts_continuous_completion_hold(self):
        monitor = SimpleNamespace(
            state='', state_t=0.0, done_t=None, done_hold=60.0,
            path_m=0.0, known=1000, known_t=0.0, stall_s=900.0,
            timeout=3600.0, t0=0.0, last_beat=0.0, last_save=0.0,
            trail=[], out='/unused', log=Mock(), render=Mock())
        with patch('tools.gz.map_run_monitor.time.monotonic') as clock:
            clock.return_value = 10.0
            Mon.on_state(monitor, SimpleNamespace(data='coverage done'))
            Mon.tick(monitor)
            clock.return_value = 30.0
            Mon.on_state(monitor, SimpleNamespace(data='explore frontier'))
            # State callbacks may leave and re-enter done between ticks.
            clock.return_value = 100.0
            Mon.on_state(monitor, SimpleNamespace(data='coverage done'))
            Mon.tick(monitor)
            clock.return_value = 159.0
            Mon.tick(monitor)
            clock.return_value = 160.0
            with self.assertRaises(SystemExit) as result:
                Mon.tick(monitor)
            self.assertEqual(result.exception.code, 0)
