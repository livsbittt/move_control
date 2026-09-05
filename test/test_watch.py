#!/usr/bin/env python3
import unittest

from move_control.watch import FOREIGN, REQUIRED, inspect


def healthy():
    nodes = list(REQUIRED) + ['lcd_node', 'web_node']
    pubs = {
        '/cmd_vel': ['safety_node'],
        '/cmd_vel_raw': ['wander_node'],
        '/scan': ['sllidar_node'],
    }
    return nodes, pubs


class WatchTest(unittest.TestCase):
    def test_ok(self):
        r = inspect(*healthy())
        self.assertTrue(r.ok, r.line())
        self.assertEqual(r.line(), 'ok')

    def test_missing_safety(self):
        nodes, pubs = healthy()
        nodes.remove('safety_node')
        pubs['/cmd_vel'] = []
        r = inspect(nodes, pubs)
        kinds = {i.kind for i in r.issues}
        self.assertFalse(r.ok)
        self.assertIn('missing', kinds)
        self.assertIn('missing_pub', kinds)
        self.assertTrue(any('safety_node' in i.detail for i in r.issues))

    def test_duplicate_wander(self):
        nodes, pubs = healthy()
        nodes.append('wander_node')
        r = inspect(nodes, pubs)
        self.assertFalse(r.ok)
        self.assertTrue(any(i.kind == 'duplicate' and i.node == 'wander_node' for i in r.issues))

    def test_foreign_cmd_vel(self):
        nodes, pubs = healthy()
        pubs['/cmd_vel'] = ['safety_node', 'pinky_control']
        r = inspect(nodes, pubs)
        self.assertFalse(r.ok)
        hit = [i for i in r.issues if i.kind == 'foreign' and i.topic == '/cmd_vel']
        self.assertEqual(len(hit), 1)
        self.assertIn('pinky_control', hit[0].node)

    def test_foreign_scan_and_raw(self):
        nodes, pubs = healthy()
        pubs['/scan'] = ['sllidar_node', 'parameter_bridge']
        pubs['/cmd_vel_raw'] = ['wander_node', 'pinky_move']
        r = inspect(nodes, pubs)
        topics = {i.topic for i in r.issues if i.kind == 'foreign'}
        self.assertEqual(topics, {'/scan', '/cmd_vel_raw'})

    def test_cmd_vel_only_remote(self):
        nodes, pubs = healthy()
        nodes.remove('safety_node')
        pubs['/cmd_vel'] = ['pinky_control']
        r = inspect(nodes, pubs)
        self.assertFalse(r.ok)
        self.assertTrue(any(i.kind == 'missing_pub' for i in r.issues))
        self.assertTrue(any(i.kind == 'foreign' and i.topic == '/cmd_vel' for i in r.issues))

    def test_ignore_cli_and_watch(self):
        nodes, pubs = healthy()
        nodes += ['watch_node', '_ros2cli_daemon_13_abc']
        r = inspect(nodes, pubs)
        self.assertTrue(r.ok, r.line())

    def test_foreign_set(self):
        self.assertIn('pinky_control', FOREIGN)
        self.assertIn('parameter_bridge', FOREIGN)

    def test_safety_zero_on_raw_is_ok(self):
        nodes, pubs = healthy()
        pubs['/cmd_vel_raw'] = ['wander_node', 'safety_node']
        r = inspect(nodes, pubs)
        self.assertTrue(r.ok, r.line())

    def test_web_teleop_on_raw_is_ok(self):
        nodes, pubs = healthy()
        pubs['/cmd_vel_raw'] = ['wander_node', 'web_node']
        r = inspect(nodes, pubs)
        self.assertTrue(r.ok, r.line())


if __name__ == '__main__':
    unittest.main()
