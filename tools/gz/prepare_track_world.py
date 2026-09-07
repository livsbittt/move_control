"""Keep the specified track intact and add the existing simulated robot."""
import copy
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


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
        'scale': 1., 'wall_collision_count': len(walls)}, indent=2))
    print(f'Original track: 16 walls, scale 1; spawn={spawn.tolist()}, clearance={d.max():.3f}m')


if __name__ == '__main__':
    main()
