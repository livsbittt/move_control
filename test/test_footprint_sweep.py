import json
import math
from pathlib import Path

import numpy as np
import pytest

from rosy_control.control.footprint_sweep import footprint_sweep_clearance, footprint_translation_limits


BOX=[[-.07,-.05],[.07,-.05],[.07,.05],[-.07,.05]]
RADIUS=math.hypot(.07,.05)


def sweep(points, **changes):
    args=dict(footprint=BOX,center=(0.,0.),uncertainty=0.,body_radius=RADIUS,
              v=0.,w=0.,horizon=.8,scan_age=0.)
    args.update(changes)
    return footprint_sweep_clearance(points,**args)


def scene(point):
    return [point,(-1.,1.),(1.,1.)]


def test_recorded_corner_command_has_clear_trusted_footprint():
    data=json.loads((Path(__file__).parent/'fixtures/mapping_corner_scan.json').read_text())
    envelope=data['limits']['rotation_estimate']
    args=dict(footprint=envelope['footprint_xy'],center=envelope['center_m'],
              uncertainty=envelope['center_uncertainty_m'],body_radius=envelope['body_radius_m'],scan_age=.1)
    assert footprint_sweep_clearance(data['scan']['points'],v=.01367767470908227,w=.1,horizon=.8,**args)>.019
    assert footprint_translation_limits(data['scan']['points'],**args)[0]>.02


def test_current_collision_and_unseen_invalid_geometry_fail_closed():
    assert sweep(scene((0.,0.)))<0
    assert sweep(scene((.07,0.)))<0
    assert sweep(scene((.2,0.)),footprint=[[0,0],[.01,0],[.02,0]]) is None
    assert sweep(scene((.2,0.)),body_radius=.01) is None
    assert sweep(scene((.2,0.)),scan_age=.201) is None


@pytest.mark.parametrize('sign',[-1,1])
def test_translation_stops_before_box_contact_and_keeps_rear_direction(sign):
    points=scene((sign*.09,0.))
    limits=footprint_translation_limits(points,BOX,(0.,0.),0.,RADIUS,0.)
    assert 0 < limits[int(sign<0)] < .01
    assert limits[int(sign>0)] == .03
    assert sweep(points,v=sign*.014)<0
    assert sweep(points,v=-sign*.014)>0


def test_rotation_sweep_detects_a_corner_missed_by_center_translation():
    assert sweep(scene((.07,.065)),w=0.)>0
    assert sweep(scene((.07,.065)),w=.1)<0


def test_duplicate_unordered_vertices_keep_the_same_bound_and_uncertainty_only_expands_it():
    points=scene((.2,.1))
    normal=sweep(points)
    assert sweep(points,footprint=list(reversed(BOX))*4)==pytest.approx(normal)
    assert sweep(points,uncertainty=.003)<normal
    assert sweep(points,scan_age=.2)<normal


@pytest.mark.parametrize('v,w',[(.014,.1),(-.014,.1),(.014,-.1),(-.014,-.1)])
def test_continuous_bound_is_conservative_against_dense_independent_polygon_distance(v,w):
    import cv2
    center=(.04,-.01)
    points=[(.13*math.cos(a),.13*math.sin(a)) for a in np.linspace(0,2*math.pi,33)[:-1]]
    actual=sweep(points,center=center,uncertainty=.002,v=v,w=w,scan_age=.1)
    pivot_radius=max(math.dist(p,center) for p in BOX)
    margin=.01+.004+(.014+.1*(pivot_radius+.004))*.25
    reference=math.inf
    for t in np.linspace(0,.8,321):
        a=w*t;c=math.cos(a);s=math.sin(a)
        dx=(1-c)*center[0]+s*center[1]+v/w*s
        dy=-s*center[0]+(1-c)*center[1]+v/w*(1-c)
        polygon=np.array([(c*x-s*y+dx,s*x+c*y+dy) for x,y in BOX],dtype=np.float32)
        reference=min(reference,*[-cv2.pointPolygonTest(polygon,p,True)-margin for p in points])
    assert actual <= reference+2e-8
    assert reference-actual < .001


@pytest.mark.parametrize('change',[{'v':.0141},{'w':.101},{'horizon':.74},
                                  {'uncertainty':-.001},{'center':(float('nan'),0.)}])
def test_invalid_command_or_model_is_unavailable(change):
    assert sweep(scene((.3,0.)),**change) is None
