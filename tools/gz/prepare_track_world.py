"""Keep the specified track intact and add the existing simulated robot."""
import copy
import hashlib
import json
import math
from itertools import product
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


def _pose_transform(element):
    pose = element.find('pose')
    if pose is not None and pose.attrib:
        raise ValueError('Only default-frame xyz/rpy poses are supported')
    values = [float(v) for v in (element.findtext('pose') or '0 0 0 0 0 0').split()]
    if len(values) != 6 or not all(math.isfinite(v) for v in values):
        raise ValueError('Pose must contain six finite xyz/rpy values')
    x, y, z, roll, pitch, yaw = values
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                             math.sin(pitch), math.cos(yaw), math.sin(yaw))
    transform = np.eye(4)
    transform[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                          [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                          [-sp, cp*sr, cp*cr]]
    transform[:3, 3] = [x, y, z]
    return transform


def collision_circumradius(model):
    """Bound every robot collision about model origin, independent of world pose.

    Cylinders use their enclosing box, so arbitrary roll/pitch remains conservative.
    Unknown geometry or frame semantics must fail instead of shrinking the envelope.
    Sensors and visuals cannot establish a physical body dimension.
    """
    if model is None or model.findall('model') or model.findall('include'):
        raise ValueError('Expected a flat explicit robot model')
    collisions = []
    footprint_xy = []
    for link in model.findall('link'):
        link_transform = _pose_transform(link)
        for col in link.findall('collision'):
            geometry = col.find('geometry')
            if geometry is None or len(geometry) != 1:
                raise ValueError('Each collision needs one supported geometry')
            shape = geometry[0]
            try:
                if shape.tag == 'box':
                    size = [float(v) for v in shape.findtext('size', '').split()]
                    method = 'box_vertices'
                elif shape.tag == 'cylinder':
                    radius = float(shape.findtext('radius', 'nan'))
                    size = [2*radius, 2*radius, float(shape.findtext('length', 'nan'))]
                    method = 'cylinder_box_envelope'
                else:
                    raise ValueError(f'Unsupported collision geometry: {shape.tag}')
            except (TypeError, ValueError) as exc:
                raise ValueError('Invalid collision geometry') from exc
            if len(size) != 3 or not all(math.isfinite(v) and v > 0 for v in size):
                raise ValueError('Collision dimensions must be finite and positive')
            vertices = np.array([(*xyz, 1.) for xyz in product(*[(-v/2, v/2) for v in size])])
            points = (link_transform @ _pose_transform(col) @ vertices.T).T
            footprint_xy.extend(points[:, :2].tolist())
            radius = float(np.hypot(points[:, 0], points[:, 1]).max())
            collisions.append({'link': link.get('name'), 'collision': col.get('name'),
                               'method': method, 'radius_m': radius})
    if not collisions:
        raise ValueError('Robot has no collision geometry')
    return {'frame': 'model_origin', 'method': 'transformed_collision_box_envelopes',
            'radius_m': max(c['radius_m'] for c in collisions), 'collisions': collisions,
            'footprint_xy': footprint_xy}


def boxes(root):
    result = []
    for model in root.findall('.//world/model'):
        if model.get('name') != 'track_260905':
            continue
        for col in model.findall('.//collision'):
            size = col.findtext('geometry/box/size')
            if size:
                result.append(([float(v) for v in col.findtext('pose').split()],
                               [float(v) for v in size.split()]))
    return result


def clearance(points, walls):
    distance = np.full(points.shape[:-1], np.inf)
    for pose, size in walls:
        dx, dy = points[..., 0]-pose[0], points[..., 1]-pose[1]
        c, s = math.cos(pose[5]), math.sin(pose[5])
        x, y = c*dx+s*dy, -s*dx+c*dy
        distance = np.minimum(distance, np.hypot(np.maximum(abs(x)-size[0]/2, 0),
                                                 np.maximum(abs(y)-size[1]/2, 0)))
    return distance


def main():
    source = Path('map/map_260905.world')
    root = ET.parse(source)
    walls = boxes(root)
    if len(walls) != 16:
        raise ValueError(f'Expected 16 track collision walls, got {len(walls)}')
    x, y = np.meshgrid(np.arange(-1.2, 1.21, .01), np.arange(-.48, .49, .01))
    points = np.stack((x, y), axis=-1)
    d = clearance(points, walls)
    spawn = points.reshape(-1, 2)[d.argmax()]
    world = root.find('world')
    robot = copy.deepcopy(ET.parse('tools/gz/pinky_maze.sdf').find(".//model[@name='pinky']"))
    body_geometry = collision_circumradius(robot)
    robot.find('pose').text = f'{spawn[0]} {spawn[1]} .01 0 0 0'
    sensor = robot.find('.//sensor')
    sensor.find('.//samples').text = '720'
    sensor.find('.//min_angle').text = str(-math.pi)
    sensor.find('.//max_angle').text = str(math.pi-2*math.pi/720)
    sensor.find('.//range/max').text = '8.0'
    world.append(robot)
    physics = ET.SubElement(world, 'physics', name='track_physics', type='ode')
    ET.SubElement(physics, 'max_step_size').text = '.005'
    ET.SubElement(physics, 'real_time_factor').text = '1'
    out = Path('/tmp/pinky-calmap227')
    out.mkdir(exist_ok=True)
    root.write(out/'track.sdf')
    (out/'track_identity.json').write_text(json.dumps({
        'source': str(source), 'sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
        'walls': walls, 'spawn': spawn.tolist(), 'spawn_clearance_m': float(d.max()),
        'robot_radius_m': body_geometry['radius_m'], 'robot_geometry': body_geometry,
        'scale': 1., 'wall_collision_count': len(walls)}, indent=2))
    print(f'Original track: 16 walls, scale 1; spawn={spawn.tolist()}, clearance={d.max():.3f}m')


if __name__ == '__main__':
    main()
