import json
import math
import xml.etree.ElementTree as ET
import numpy as np
import pytest
from tools.gz.calibration_spaces import CASES, prepare_case
from tools.gz.prepare_track_world import boxes, clearance


@pytest.mark.parametrize('name', CASES)
def test_case_preserves_robot_and_has_collision_free_spawn(name, tmp_path):
    result=prepare_case(name,tmp_path)
    root=ET.parse(tmp_path/'track.sdf')
    assert json.loads(json.dumps(boxes(root)))==result['walls']
    assert json.loads((tmp_path/'track_identity.json').read_text())==result
    assert .114 < result['robot_radius_m'] < .116
    robot=root.find(".//model[@name='pinky']")
    assert len(robot.findall('joint'))==2
    assert robot.find("plugin[@name='gz::sim::systems::DiffDrive']") is not None
    points=np.array(result['robot_geometry']['footprint_xy'])
    yaw=result['spawn_yaw_rad'];c,s=math.cos(yaw),math.sin(yaw)
    points=points@np.array([[c,s],[-s,c]])+result['spawn']
    assert np.min(clearance(points,result['walls']))>0
    assert float(robot.findtext('.//sensor/lidar/scan/horizontal/samples'))==720
    # Every scan bearing hits a finite enclosing wall within the 8m range.
    angles=np.arange(720)*2*math.pi/720
    directions=np.column_stack((np.cos(angles),np.sin(angles)))
    nearest=np.full(720,np.inf)
    for pose,size in result['walls']:
        c,s=math.cos(pose[5]),math.sin(pose[5])
        rotation=np.array([[c,-s],[s,c]])
        origin=(np.array(result['spawn'])-pose[:2])@rotation
        rays=directions@rotation
        with np.errstate(divide='ignore',invalid='ignore'):
            a=(-np.array(size[:2])/2-origin)/rays
            b=(np.array(size[:2])/2-origin)/rays
        enter=np.minimum(a,b).max(axis=1)
        leave=np.maximum(a,b).min(axis=1)
        hit=(leave>=enter)&(enter>=0)
        nearest[hit]=np.minimum(nearest[hit],enter[hit])
    assert np.isfinite(nearest).all() and nearest.max()<8.


def test_corridor_has_known_short_exit_with_true_circumradius(tmp_path):
    case=prepare_case('corridor_exit',tmp_path)
    target=case['known_exit_pose'][:2]
    path=np.linspace(case['spawn'],target,101)
    assert np.min(clearance(path,case['walls']))>case['robot_radius_m']+.01
    assert math.dist(case['spawn'],target)<=.2
    assert clearance(path[-1:],case['walls'])[0]>.20


def test_trapped_is_physically_fit_but_not_circumcircle_clear(tmp_path):
    case=prepare_case('trapped',tmp_path)
    assert case['spawn_clearance_m']<case['robot_radius_m']
    assert case['expected_calibration']=='hold'


def test_output_is_deterministic_and_unknown_case_rejected(tmp_path):
    a=prepare_case('open',tmp_path)
    original=(tmp_path/'track.sdf').read_bytes()
    assert prepare_case('open',tmp_path)==a
    assert (tmp_path/'track.sdf').read_bytes()==original
    with pytest.raises(ValueError):prepare_case('unknown',tmp_path)


@pytest.mark.parametrize('name', ['front_wall','rear_wall'])
def test_wall_cases_leave_translation_room_before_rotation_relocation(name,tmp_path):
    case=prepare_case(name,tmp_path)
    assert case['spawn_clearance_m']==pytest.approx(.16)
    assert case['expected_calibration']=='ready_after_relocation'


def test_cli_case_and_output_arguments(tmp_path):
    import subprocess
    import sys
    subprocess.run([sys.executable,'-m','tools.gz.calibration_spaces','open',str(tmp_path)],check=True)
    assert json.loads((tmp_path/'track_identity.json').read_text())['case']=='open'
