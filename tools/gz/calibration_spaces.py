"""Deterministic enclosed calibration fixtures using the unchanged wheel robot."""
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from tools.gz.prepare_track_world import collision_circumradius, clearance

CASES=('open','front_wall','rear_wall','corridor_exit','trapped')


def _wall(x,y,width,depth,yaw=0.):
    return [[x,y,.15,0.,0.,yaw],[width,depth,.30]]


def prepare_case(name,out):
    """Write runner-compatible SDF/identity; readiness is a test expectation only."""
    if name not in CASES:
        raise ValueError('Unknown calibration space: '+str(name))
    source=Path(__file__).with_name('pinky_maze.sdf')
    tree=ET.parse(source)
    world=tree.find('world')
    world.set('name','calibration_'+name)
    robot=world.find("model[@name='pinky']")
    geometry=collision_circumradius(robot)
    for model in list(world.findall('model')):
        if model.get('name') not in ('pinky','ground'):
            world.remove(model)
    # Keep the existing contact/friction definitions, extending only floor coverage.
    ground=world.find("model[@name='ground']")
    for shape in ground.findall('.//collision')+ground.findall('.//visual'):
        shape.find('pose').text='0 0 -0.05 0 0 0'
        shape.find('geometry/box/size').text='4 4 0.1'
    walls=[_wall(-1.025,0,.05,2.1),_wall(1.025,0,.05,2.1),
           _wall(0,-1.025,2,.05),_wall(0,1.025,2,.05),
           _wall(.67,.48,.19,.13,.27),_wall(-.64,.69,.11,.25,-.19)]
    expected='ready'
    note='Wide asymmetric room; direct calibration expected.'
    exit_pose=None
    if name=='front_wall':
        walls.append(_wall(.185,0,.05,.50))
        note='Front face 16cm away; verify short translation before reverse relocation for rotation.'
        expected='ready_after_relocation'
    elif name=='rear_wall':
        walls.append(_wall(-.185,0,.05,.50))
        note='Rear face 16cm away; verify short translation before forward relocation for rotation.'
        expected='ready_after_relocation'
    elif name=='corridor_exit':
        walls.extend([_wall(-.16,.175,.40,.05),_wall(-.16,-.175,.40,.05)])
        exit_pose=[.19,0.,0.]
        note='30cm corridor ends at x=4cm; 19cm forward reaches the wide room.'
        expected='ready_after_relocation'
    elif name=='trapped':
        walls.extend([_wall(.105,0,.05,.254),_wall(-.105,0,.05,.254),
                      _wall(0,.127,.16,.05),_wall(0,-.127,.16,.05)])
        expected='hold'
        note='16x20.4cm pocket fits the stationary footprint but lacks safe calibration travel/rotation.'
    track=ET.SubElement(world,'model',name='track_260905')
    ET.SubElement(track,'static').text='true'
    link=ET.SubElement(track,'link',name='walls')
    for index,(pose,size) in enumerate(walls):
        for tag in ('collision','visual'):
            shape=ET.SubElement(link,tag,name=f'{tag}_{index}')
            ET.SubElement(shape,'pose').text=' '.join(map(str,pose))
            box=ET.SubElement(ET.SubElement(shape,'geometry'),'box')
            ET.SubElement(box,'size').text=' '.join(map(str,size))
    robot.find('pose').text='0 0 .01 0 0 0'
    sensor=robot.find('.//sensor')
    sensor.find('.//samples').text='720'
    sensor.find('.//min_angle').text=str(-math.pi)
    sensor.find('.//max_angle').text=str(math.pi-2*math.pi/720)
    sensor.find('.//range/max').text='8.0'
    payload=ET.tostring(tree.getroot(),encoding='utf-8')
    identity={'source':'generated:calibration_spaces/'+name,'sha256':hashlib.sha256(payload).hexdigest(),
              'robot_source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
              'case':name,'walls':walls,'spawn':[0.,0.],'spawn_yaw_rad':0.,
              'spawn_clearance_m':float(clearance(np.array([[0.,0.]]),walls)[0]),
              'robot_radius_m':geometry['radius_m'],'robot_geometry':geometry,'scale':1.,
              'wall_collision_count':len(walls),'known_exit_pose':exit_pose,
              'expected_calibration':expected,'expectation_note':note,
              'physical_robot_verified':False}
    output=Path(out);output.mkdir(parents=True,exist_ok=True)
    (output/'track.sdf').write_bytes(payload)
    (output/'track_identity.json').write_text(json.dumps(identity,indent=2),encoding='utf-8')
    return identity


def main():
    import argparse
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('case',choices=CASES)
    parser.add_argument('out')
    args=parser.parse_args()
    print(json.dumps(prepare_case(args.case,args.out)))


if __name__=='__main__':
    main()
