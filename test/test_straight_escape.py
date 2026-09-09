import json
from pathlib import Path
import numpy as np
import pytest
from rosy_control.planning.gridmap import OccupancyMap
from rosy_control.planning.straight_escape import straight_escape
from rosy_control.control.straight_escape import StraightEscape
from rosy_control.control.footprint_sweep import footprint_translation_limits


def test_actual_corner_has_a_short_fixed_heading_exit():
    folder=Path(__file__).parents[1]/'docs/validation/mapping-finish-2026-09-08/v4-polygon-planner'
    saved=np.load(folder/'track_map.npz')
    arr=saved['data']; m=OccupancyMap(arr.shape[1],arr.shape[0],float(saved['resolution']),saved['origin'])
    m.data=arr.ravel().tolist()
    identity=json.loads((folder/'track_identity.json').read_text())
    pose=(.41379112,-.38244803,-.32155716)
    target=straight_escape(m,pose,identity['robot_geometry']['footprint_xy'],.01621,.134405)
    assert target is not None
    assert .01<np.linalg.norm(np.asarray(target)-pose[:2])<=.12
    assert straight_escape(m,pose,identity['robot_geometry']['footprint_xy'],.01621,.134405,target)==target


def scene():
    m=OccupancyMap(100,100,.01,(-.5,-.5),fill=0)
    return m,[[-.04,-.02],[.04,-.02],[.04,.02],[-.04,.02]]


@pytest.mark.parametrize('value',[-1,100])
def test_whole_unknown_and_occupied_cells_block_midway(value):
    m,shape=scene()
    target=(.1,0.)
    m.set_cell(*m.world_to_grid(.06,.025),value)
    assert straight_escape(m,(0.,0.,0.),shape,.01,.06,target) is None


def test_map_boundary_and_invalid_pose_fail_closed():
    m,shape=scene()
    assert straight_escape(m,(.46,0.,0.),shape,.01,.06) is None
    assert straight_escape(m,(float('nan'),0.,0.),shape,.01,.06) is None


def test_opposite_heading_changes_asymmetric_body_clearance():
    m,_=scene(); shape=[[-.02,-.02],[.08,-.02],[.08,.02],[-.02,.02]]
    m.set_cell(*m.world_to_grid(-.075,0.),100)
    assert straight_escape(m,(0.,0.,0.),shape,.01,.04,(.02,0.)) is not None
    assert straight_escape(m,(0.,0.,np.pi),shape,.01,.04,(-.02,0.)) is None


def test_executor_keeps_heading_zero_turn_and_never_renews_deadline():
    e=StraightEscape(); intent=dict(id='one',target=[.1,0.],yaw=0.)
    assert e.update(0.,(0.,0.,0.),intent,True)==(.006,'straight_escape')
    for index in range(1,291):
        assert e.update(index*.1,(index*.00017,0.,0.),intent,True)[0]>.0
    assert e.update(30.,(.05,0.,0.),intent,True)==(0.,'straight_escape_stopped')
    assert e.update(31.,(.05,0.,0.),{**intent,'id':'two'},True)[0]==0.


@pytest.mark.parametrize('pose,safe',[((.01,0.,.021),True),((.01,.004,0.),True),((.01,0.,0.),False)])
def test_executor_latches_evidence_or_heading_loss(pose,safe):
    e=StraightEscape(); intent=dict(id='one',target=[.1,0.],yaw=0.)
    e.update(0.,(0.,0.,0.),intent,True)
    assert e.update(1.,pose,intent,safe)[0]==0.
    assert e.update(2.,(.01,0.,0.),intent,True)[0]==0.


def test_executor_requires_position_completion():
    e=StraightEscape(); intent=dict(id='one',target=[.1,0.],yaw=0.)
    e.update(0.,(0.,0.,0.),intent,True)
    for index in range(1,196):
        e.update(index*.1,(index*.0005,0.,0.),intent,True)
    assert e.update(19.6,(.098,0.,0.),intent,True)==(0.,'complete')
    assert e.update(19.7,(.094,0.,0.),intent,True,odom=(.098,0.,0.))==(0.,'complete')


def test_reverse_escape_keeps_yaw_and_direction():
    e=StraightEscape(); intent=dict(id='reverse',target=[-.1,0.],yaw=0.)
    assert e.update(0.,(0.,0.,0.),intent,True)==(-.006,'straight_escape')


def test_completed_executor_accepts_next_region_only_after_continuous_travel():
    e=StraightEscape(); intent=dict(id='one',target=[.02,0.],yaw=0.)
    e.update(1.,(0.,0.,0.),intent,True)
    for i in range(1,41):e.update(1+i*.1,(i*.0005,0.,0.),intent,True)
    assert e.completed
    for i in range(1,301):e.update(5+i*.1,(.02+i*.001,0.,0.),None,True)
    assert e.update(35.1,(.32,0.,0.),dict(id='two',target=[.35,0.],yaw=0.),True)[0]>.0


def test_sensor_travel_selects_reverse_and_rejects_missing_full_segment():
    m,shape=scene()
    target=straight_escape(m,(0.,0.,0.),shape,.01,.06,travel=(.001,.12))
    assert target is not None and target[0]<0
    assert straight_escape(m,(0.,0.,0.),shape,.01,.06,travel=(.001,.001)) is None


def test_actual_lidar_blocked_forward_escape_selects_observed_reverse():
    folder=Path(__file__).parents[1]/'docs/validation/mapping-finish-2026-09-08/v5-forward-escape'
    saved=np.load(folder/'track_map.npz'); arr=saved['data']
    m=OccupancyMap(arr.shape[1],arr.shape[0],float(saved['resolution']),saved['origin']); m.data=arr.ravel().tolist()
    scan=json.loads((folder/'corner_scan.json').read_text()); r=scan['limits']['rotation_estimate']
    travel=footprint_translation_limits(scan['scan']['points'],r['footprint_xy'],r['center_m'],
        r['center_uncertainty_m'],r['body_radius_m'],.1,.12)
    pose=(.42007386,-.38425686,-.31412414)
    target=straight_escape(m,pose,r['footprint_xy'],.01+2*r['center_uncertainty_m'],.134405,travel=travel)
    assert travel[0]<.002
    assert target is not None and target[0]<pose[0]
    aged=footprint_translation_limits(scan['scan']['points'],r['footprint_xy'],r['center_m'],
        r['center_uncertainty_m'],r['body_radius_m'],.2,.12)
    assert aged==(0.,0.)
