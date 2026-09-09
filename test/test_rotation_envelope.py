import math
import copy
from pathlib import Path
import xml.etree.ElementTree as ET
import unittest

import numpy as np

from rosy_control.control.rotation_envelope import RotationEnvelope, validate_envelope, pivot_clearance, suggest_rotation_translation, straight_translation_limits
from rosy_control.sensing.scan_motion import match_motion, scan_points


def delta(center, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return ((1-c)*center[0]+s*center[1], -s*center[0]+(1-c)*center[1], yaw)


def polygon_scan(polygon, pose, mount=(.012,-.008,0.)):
    """Raycast actual angular beams, not identical transformed point samples."""
    yaw=pose[2]
    c,s=math.cos(yaw),math.sin(yaw)
    origin=np.array(pose[:2])+np.array([[c,-s],[s,c]])@np.array(mount[:2])
    angles=np.linspace(-math.pi,math.pi,720,endpoint=False)
    rays=np.column_stack([np.cos(angles+yaw+mount[2]),np.sin(angles+yaw+mount[2])])
    ranges=np.full(720,np.inf)
    for start,end in zip(polygon,polygon[1:]+polygon[:1]):
        start=np.array(start)
        edge=np.array(end)-start
        denom=rays[:,0]*edge[1]-rays[:,1]*edge[0]
        offset=start-origin
        with np.errstate(divide='ignore',invalid='ignore'):
            distance=(offset[0]*edge[1]-offset[1]*edge[0])/denom
            fraction=(offset[0]*rays[:,1]-offset[1]*rays[:,0])/denom
        keep=(distance>0)&(fraction>=0)&(fraction<=1)
        ranges[keep]=np.minimum(ranges[keep],distance[keep])
    return scan_points(ranges,-math.pi,2*math.pi/720,mount)


class RotationEnvelopeTest(unittest.TestCase):
    def test_straight_translation_limits_bound_capsule_and_initial_contact(self):
        self.assertEqual(straight_translation_limits([[.1,0]],.1),(0.,0.))
        forward,reverse=straight_translation_limits([[-.12,0]],.1)
        self.assertEqual(forward,.03)
        self.assertAlmostEqual(reverse,.0099)
        forward,reverse=straight_translation_limits([[.125,.02]],.1)
        self.assertAlmostEqual(forward,.125-math.sqrt(.11**2-.02**2)-.0001)
        self.assertEqual(reverse,.03)
        self.assertEqual(straight_translation_limits([[1,1]],.1),(.03,.03))
        for points,radius in [([], .1),([[float('nan'),0]],.1),([[0,0]],-.1)]:
            self.assertIsNone(straight_translation_limits(points,radius))

    def test_translation_suggestion_restores_pivot_without_body_path_collision(self):
        self.assertEqual(suggest_rotation_translation([[-.18,0]],[-.04,0],.1314,.1144),.005)
        self.assertEqual(suggest_rotation_translation([[.14,0]],[0,0],.1314,.1144),-.005)
        self.assertIsNone(suggest_rotation_translation([[-.18,0],[.126,0]],[-.04,0],.1314,.1144))
        self.assertIsNone(suggest_rotation_translation([[0,.14]],[0,0],.1314,.1144))
        self.assertIsNone(suggest_rotation_translation([[1,0]],[0,0],.1314,.1144))
        self.assertIsNone(suggest_rotation_translation([[float('nan'),0]],[0,0],.1314,.1144))
        self.assertIsNone(suggest_rotation_translation([[-.18,0]],[-.04,0],-.1,.1144))

    def test_angular_raycast_registration_recovers_offset_pivot(self):
        polygon=[[-.4,-.14],[.6,-.14],[.6,.4],[.2,.4],[.2,.14],[-.4,.14]]
        ref=polygon_scan(polygon,(0,0,0))
        for yaw in (-.175,.175):
            expected=delta((-.04,0),yaw)
            current=polygon_scan(polygon,expected)
            result=match_motion(ref,current,yaw)
            self.assertIsNotNone(result)
            self.assertLess(result['residual_m'],.0008)
            np.testing.assert_allclose([result['dx'],result['dy'],result['yaw']],expected,atol=.0008)

    def test_wall_registration_retains_range_noise_in_residual(self):
        polygon=[[-.35,-.2],[.45,-.2],[.45,.45],[-.1,.45],[-.1,.2],[-.35,.2]]
        expected=delta((-.04,0),.175)
        rng=np.random.default_rng(10)
        ref=polygon_scan(polygon,(0,0,0))
        cur=polygon_scan(polygon,expected)
        ref+=rng.normal(0,.0002,ref.shape)
        cur+=rng.normal(0,.0002,cur.shape)
        result=match_motion(ref,cur,.175)
        self.assertIsNotNone(result)
        self.assertGreater(result['residual_m'],.00005)
        self.assertLess(result['residual_m'],.001)
        np.testing.assert_allclose([result['dx'],result['dy'],result['yaw']],expected,atol=.001)
    def test_requires_bilateral_repeated_evidence(self):
        est = RotationEnvelope(0.08)
        for yaw in (0.12, 0.12, -0.12):
            d = delta((0.02, -0.01), yaw)
            self.assertTrue(est.add(d, d, yaw, 0.0001))
            self.assertFalse(est.report()['valid'])
        d = delta((0.02, -0.01), -0.12)
        est.add(d, d, -0.12, 0.0001)
        report = est.report()
        self.assertTrue(report['valid'])
        np.testing.assert_allclose(report['center_m'], (0.02, -0.01), atol=1e-8)
        self.assertGreaterEqual(report['required_radius_m'], 0.08+2*math.hypot(.02,.01))

    def test_rejects_disagreement_and_unexcited_motion(self):
        est = RotationEnvelope(.08)
        for scan, odom, imu, residual in [
            ((0,0,.01),(0,0,.01),.01,.001),
            ((0,0,.12),(0,0,-.12),.12,.001),
            ((0,0,.12),(0,0,.12),-.12,.001),
            ((.1,0,.12),(0,0,.12),.12,.001),
            ((0,0,.12),(0,0,.12),.12,.03),
            ((0,0,float('nan')),(0,0,.12),.12,.001),
        ]:
            self.assertFalse(est.add(scan, odom, imu, residual))
        self.assertEqual(est.report()['required_radius_m'], .08)

    def test_zero_pivot_does_not_add_large_uncertainty(self):
        est = RotationEnvelope(.08)
        for yaw in (.2,.2,-.2,-.2):
            est.add((0,0,yaw),(0,0,yaw),yaw,.0001)
        self.assertLess(est.report()['required_radius_m'], .086)
        self.assertGreater(est.report()['required_radius_m'], .085)

    def test_inconsistent_pivots_are_not_certified(self):
        est = RotationEnvelope(.08)
        for center,yaw in [((.08,0),.12),((.08,0),.12),((-.08,0),-.12),((-.08,0),-.12)]:
            d=delta(center,yaw)
            est.add(d,d,yaw,.0001)
        self.assertFalse(est.report()['valid'])

    def test_scan_mount_and_invalid_ranges(self):
        pts=scan_points([1,float('nan'),float('inf'),0,.01],0,.1,(.02,.03,math.pi))
        np.testing.assert_allclose(pts,[[-.98,.03]],atol=1e-8)

    def test_matches_rigid_motion_and_rejects_wall(self):
        rng=np.random.default_rng(31)
        ref=rng.uniform(-.6,.6,(160,2))
        yaw=.12
        rot=np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]])
        current=(ref-np.array([.004,-.003]))@rot
        result=match_motion(ref,current,yaw)
        self.assertIsNotNone(result)
        np.testing.assert_allclose([result['dx'],result['dy'],result['yaw']],[.004,-.003,yaw],atol=1e-5)
        wall=np.column_stack([np.linspace(-1,1,100),np.ones(100)])
        self.assertIsNone(match_motion(wall,wall))
        self.assertIsNone(match_motion(ref, current+10,yaw))

    def test_circular_geometry_is_ambiguous(self):
        angles=np.linspace(-math.pi,math.pi,180,endpoint=False)
        circle=np.column_stack([np.cos(angles),np.sin(angles)])
        self.assertIsNone(match_motion(circle,circle,.12))

    def test_certificate_validator_rejects_invalid_geometry_and_counts(self):
        est = RotationEnvelope(.08)
        for yaw in (.2,.2,-.2,-.2):
            est.add((0,0,yaw),(0,0,yaw),yaw,.0001)
        report=est.report()
        self.assertTrue(validate_envelope(report, .08))
        for change in ({'required_radius_m': .079}, {'center_m':[float('nan'),0]},
                       {'center_uncertainty_m':-.001}, {'valid':1},
                       {'sample_count':float('inf')},
                       {'directional_counts':{'positive':2.5,'negative':1.5}},
                       {'directional_counts':{'positive':2,'negative':float('nan')}}):
            self.assertFalse(validate_envelope(dict(report, **change)))
        self.assertFalse(validate_envelope(report,.081))

    def test_rejection_permanently_invalidates_trial_session(self):
        est=RotationEnvelope(.08)
        for yaw in (.2,.2,-.2,-.2):
            self.assertTrue(est.add((0,0,yaw),(0,0,yaw),yaw,.0001))
        self.assertTrue(est.report()['valid'])
        self.assertFalse(est.add((0,0,.2),(0,0,-.2),.2,.0001))
        self.assertFalse(est.report()['valid'])
        self.assertFalse(est.add((0,0,.2),(0,0,.2),.2,.0001))
        self.assertFalse(est.report()['valid'])

    def test_recomputes_persisted_trial_evidence(self):
        est=RotationEnvelope(.08)
        for yaw in (.2,.2,-.2,-.2):
            est.add((0,0,yaw),(0,0,yaw),yaw,.0001)
        report=est.report()
        self.assertEqual(len(report['trials']),4)
        for change in ({'center_m':[.01,0], 'required_radius_m':report['required_radius_m']+.02},
                       {'required_radius_m':report['required_radius_m']+.1},
                       {'trials':[]}, {'body_radius_m':10**1000}):
            self.assertFalse(validate_envelope(dict(report,**change)))
        altered=copy.deepcopy(report)
        altered['trials'][0]['imu_yaw']=-.2
        self.assertFalse(validate_envelope(altered))

    def test_malformed_point_input_returns_no_evidence(self):
        for bad in ('bad', [[1,2],[3]], None, [[10**1000,0]]):
            self.assertIsNone(match_motion(bad,bad))

    def test_trusted_footprint_bounds_rotation_about_measured_pivot(self):
        footprint=[[-.04,-.05],[-.04,.05],[.08,.05],[.08,-.05]]
        radius=math.hypot(.08,.05)
        est=RotationEnvelope(radius,footprint)
        for yaw in (.2,.2,-.2,-.2):
            d=delta((-.04,0),yaw)
            est.add(d,d,yaw,.0001)
        report=est.report()
        self.assertTrue(validate_envelope(report,radius))
        self.assertAlmostEqual(report['pivot_radius_m'], .13+2*report['center_uncertainty_m'])
        self.assertAlmostEqual(report['required_radius_m'], .04+report['pivot_radius_m'])
        self.assertLess(report['pivot_radius_m'],.14)
        altered=copy.deepcopy(report)
        altered['pivot_radius_m']-=.01
        self.assertFalse(validate_envelope(altered,radius))
        with self.assertRaises(ValueError):
            RotationEnvelope(radius,[[0,0],[.01,0],[0,.01]])

    def test_unordered_duplicate_footprint_and_pivot_clearance(self):
        footprint=[[.08,.05],[-.04,-.05],[.08,-.05],[-.04,.05]]*8
        est=RotationEnvelope(math.hypot(.08,.05),footprint)
        for yaw in (.2,.2,-.2,-.2):
            d=delta((-.04,0),yaw)
            est.add(d,d,yaw,.0001)
        report=est.report()
        self.assertTrue(validate_envelope(report))
        self.assertAlmostEqual(pivot_clearance([[.16,0],[0,.4]],report['center_m'],report['pivot_radius_m']),
                               .2-report['pivot_radius_m'])
        self.assertLess(pivot_clearance([[0,0]],report['center_m'],report['pivot_radius_m']),0.)
        for points,center,radius in [([], [0,0],.1),([[float('nan'),0]],[0,0],.1),
                                     ([[0,0]],[float('nan'),0],.1),([[0,0]],[0,0],-.1)]:
            self.assertIsNone(pivot_clearance(points,center,radius))

    def test_real_gazebo_collision_projection_is_accepted(self):
        from tools.gz.prepare_track_world import collision_circumradius
        model=ET.parse(Path(__file__).parents[1]/'tools/gz/pinky_maze.sdf').find(".//model[@name='pinky']")
        geometry=collision_circumradius(model)
        self.assertEqual(len(geometry['footprint_xy']),32)
        est=RotationEnvelope(geometry['radius_m'],geometry['footprint_xy'])
        for yaw in (.2,.2,-.2,-.2):
            d=delta((-.04,0),yaw)
            self.assertTrue(est.add(d,d,yaw,.0001))
        self.assertTrue(validate_envelope(est.report(),geometry['radius_m']))
