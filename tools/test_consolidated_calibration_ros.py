"""Isolated ROS adapter checks for merged calibration authority and rotation."""
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import String
from rosy_control.startup_calibration_node import StartupCalibrationNode
from rosy_control.control.calibration_certificate import make_certificate
from test.test_calibration_certificate import complete_motion


class ConsolidatedCalibrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(domain_id=221)

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.node=StartupCalibrationNode(parameter_overrides=[Parameter('result_path',value=str(Path(self.tmp.name)/'calibration.json'))])
        self.node.raw_pub=Mock()
        self.node.ready_pub=Mock()
        self.clock=patch('time.monotonic',return_value=100.)
        self.now=self.clock.start()
        self.node.geometry_revision='geometry-a'
        self.node.geometry_received=100.
        self.node.geometry_profile={'effective':{'turn_clear':.10}}

    def tearDown(self):
        self.clock.stop()
        self.node.destroy_node()
        self.tmp.cleanup()

    def test_legacy_restore_has_no_rotation_and_waits_for_live_ack(self):
        n=self.node
        n.write_json(n.certificate_path(),make_certificate(n.certificate_configuration(),complete_motion()))
        self.assertTrue(n.restore_certificate())
        self.assertIsNone(n.rotation_report())
        self.assertEqual(n.phase,'ready')
        n.runtime_ready=True
        n.publish()
        self.assertTrue(n.profile_packet()['enabled'])
        self.assertFalse(n.report()['ready'])
        n.on_applied(String(data=json.dumps({'applied':True,'revision':n.profile_revision,'session':n.profile_session})))
        self.assertTrue(n.report()['ready'])
        n.runtime_ready=False
        self.assertFalse(n.profile_packet()['enabled'])
        self.assertFalse(n.report()['ready'])
        self.assertEqual(n.phase,'ready')

    def test_rotation_does_not_use_translation_wall_or_target(self):
        n=self.node
        n.phase='validating_rotation'; n.estop=False
        n.safe_motion=Mock(side_effect=AssertionError('straight-wall guard used for rotation'))
        n.gate_decision=(100.25,dict(requested_v=0.,safe_v=0.,requested_omega=0.,safe_omega=0.))
        n._trial_request=(100.,0.,0.)
        for name in ('odom','imu','ir','tf'):
            n.baseline.add(name,(0.,0.,0.,0.),100.,True)
        for topic in ('/safety/blocked','/safety/cliff','/safety/tilt','/safety/pickup'):
            n.hazards[topic]=(100.,False)
        n.rotation_scan=(100.,(.5,)*720,2*math.pi/720)
        self.assertIsNone(n.rotation_eligibility(100.))
        self.assertTrue(n.rotation_clear(100.))
        self.assertIsNotNone(n.rotation_eligibility(100.3))
        n.rotation_scan=(100.,(.5,)*719+(float('inf'),),2*math.pi/720)
        self.assertFalse(n.rotation_clear(100.))

    def test_ready_geometry_drift_revokes_without_rebinding_old_evidence(self):
        n=self.node;n.phase='ready';n.runtime_ready=True
        n.trial_geometry_revision='geometry-a'
        n.on_safety_profile(String(data=json.dumps({'valid':True,'revision':'geometry-b',
            'effective':{'turn_clear':.11}})))
        self.assertFalse(n.runtime_ready)
        self.assertFalse(n.profile_packet()['enabled'])
        self.assertEqual(n.profile_packet()['geometry_revision'],'geometry-a')
        self.assertEqual(n.phase,'ready')
        self.assertEqual(n.raw_pub.publish.call_args.args[0].angular.z,0.)

    def test_modified_or_unacknowledged_final_output_rejects_trial(self):
        n=self.node;n.phase='validating_rotation'
        n._trial_request=(99.,0.,.06)
        n.gate_decision=(100.25,dict(requested_v=0.,safe_v=0.,requested_omega=.06,safe_omega=.04))
        self.assertIn('modified',n.trial_gate_reason(100.))
        n.gate_decision=(100.25,dict(requested_v=0.,safe_v=0.,requested_omega=0.,safe_omega=0.))
        self.assertIn('acknowledge',n.trial_gate_reason(100.))
        n.gate_decision=(100.25,dict(requested_v=0.,safe_v=0.,requested_omega=.06,safe_omega=.06))
        self.assertIsNone(n.trial_gate_reason(100.))

if __name__=='__main__':unittest.main()
