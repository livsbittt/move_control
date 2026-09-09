import gzip
import json
import math
from pathlib import Path
import unittest
import numpy as np
from rosy_control.sensing.scan_motion import match_motion


class RegistrationEndpointTests(unittest.TestCase):
    def frames(self):
        with gzip.open(Path(__file__).parent/'fixtures/registration_stationary_endpoints_20260909.json.gz','rt') as f:
            return [np.array(p) for p in json.load(f)]

    def test_all_real_stationary_frames_align_with_themselves(self):
        for points in self.frames():
            result=match_motion(points,points,0.)
            self.assertIsNotNone(result)
            self.assertLess(math.hypot(result['dx'],result['dy']),1e-10)
            self.assertLess(abs(result['yaw']),1e-10)

    def test_known_bilateral_rotation_and_missing_points(self):
        for points in self.frames()[:4]:
            for yaw in (-math.radians(10),math.radians(10)):
                c,s=math.cos(yaw),math.sin(yaw)
                rotation=np.array([[c,-s],[s,c]])
                translation=np.array([0.,0.])
                current=(points-translation)@rotation
                current=np.delete(current,[4,19,47],axis=0)
                result=match_motion(points,current,yaw)
                self.assertIsNotNone(result)
                self.assertAlmostEqual(result['yaw'],yaw,places=8)
                self.assertAlmostEqual(result['dx'],translation[0],places=8)
                self.assertAlmostEqual(result['dy'],translation[1],places=8)

    def test_single_wall_and_uniform_circle_remain_unobservable(self):
        wall=np.column_stack((np.linspace(-.5,.5,180),np.ones(180)*.3))
        self.assertIsNone(match_motion(wall,wall,0.))
        angle=np.linspace(0.,2*math.pi,180,endpoint=False)
        circle=np.column_stack((np.cos(angle),np.sin(angle)))*.5
        self.assertIsNone(match_motion(circle,circle,0.))

    def test_real_cloud_translation_seed_preserves_final_segment_validation(self):
        for points in self.frames()[:4]:
            for yaw in (-math.radians(10),math.radians(10)):
                c,s=math.cos(yaw),math.sin(yaw)
                rotation=np.array([[c,-s],[s,c]])
                translation=np.array([.001,-.002])
                current=(points-translation)@rotation
                current=np.delete(current,[4,19,47],axis=0)
                result=match_motion(points,current,yaw)
                self.assertIsNotNone(result)
                self.assertAlmostEqual(result['yaw'],yaw,delta=.001)
                self.assertAlmostEqual(result['dx'],translation[0],delta=.0002)
                self.assertAlmostEqual(result['dy'],translation[1],delta=.0002)

    def test_noisy_real_wall_edges_gain_independent_local_line_support(self):
        points=self.frames()[1]
        current=points+np.random.default_rng(530).normal(0.,.00005,points.shape)
        result=match_motion(points,current,0.)
        self.assertIsNotNone(result)
        self.assertLess(math.hypot(result['dx'],result['dy']),.0001)

    def test_actual_failed_endpoint_pair_has_observable_supported_registration(self):
        with gzip.open(Path(__file__).parent/'fixtures/registration_endpoint_v14_20260909.json.gz','rt') as stream:
            frame=json.load(stream)
        result=match_motion(frame['reference_points'],frame['current_points'],frame['alignment']['yaw'])
        self.assertIsNotNone(result)
        self.assertLess(result['residual_m'],.001)
        self.assertAlmostEqual(result['yaw'],frame['odom_delta'][2],delta=.002)
        self.assertLess(math.hypot(result['dx']-frame['odom_delta'][0],result['dy']-frame['odom_delta'][1]),.001)

    def test_parallel_walls_still_cannot_observe_along_wall_translation(self):
        x=np.linspace(-.4,.4,90)
        points=np.r_[np.column_stack((x,np.full(90,.2))),np.column_stack((x[::-1],np.full(90,-.2)))]
        self.assertIsNone(match_motion(points,points,0.))
