"""Audit the full exact-track interior, including space outside the SLAM raster."""
import json
import hashlib
import math
from pathlib import Path
import sys

import numpy as np

from tools.gz.prepare_track_world import clearance


def wall_edges(wall, step=.01):
    pose, size = wall
    x, y = size[0]/2, size[1]/2
    a = np.linspace(-x, x, max(2, math.ceil(2*x/step)+1))
    b = np.linspace(-y, y, max(2, math.ceil(2*y/step)+1))
    points = np.concatenate((np.column_stack((a, a*0+y)), np.column_stack((a, a*0-y)),
                             np.column_stack((b*0+x, b)), np.column_stack((b*0-x, b))))
    c, s = math.cos(pose[5]), math.sin(pose[5])
    return points @ np.array([[c, s], [-s, c]]) + pose[:2]


def measure(arr, origin, resolution, walls):
    arr = np.asarray(arr)
    if arr.ndim != 2 or not arr.size or not walls or not np.isfinite(resolution) or resolution <= 0:
        raise ValueError('Nonempty raster, positive resolution and collision walls required')
    if not np.isfinite(origin).all() or not np.isfinite(arr).all() or np.any((arr < -1) | (arr > 100)):
        raise ValueError('Invalid map coordinates or occupancy values')
    edges = [wall_edges(wall) for wall in walls]
    lo, hi = np.concatenate(edges).min(axis=0), np.concatenate(edges).max(axis=0)
    # Fixed world sampling prevents a cropped raster from shrinking the denominator.
    x, y = np.meshgrid(np.arange(lo[0]+.005, hi[0], .01), np.arange(lo[1]+.005, hi[1], .01))
    points = np.stack((x, y), axis=-1)
    interior = clearance(points, walls) > .03
    cells = np.floor((points-np.array(origin))/resolution).astype(int)
    valid = (cells[..., 0] >= 0) & (cells[..., 0] < arr.shape[1]) & (cells[..., 1] >= 0) & (cells[..., 1] < arr.shape[0])
    values = np.full(x.shape, -1.)
    values[valid] = arr[cells[..., 1][valid], cells[..., 0][valid]]
    unknown = float(np.mean(values[interior] < 0))
    known = interior & (values >= 0)
    purity = float(np.mean(values[known] < 65)) if known.any() else 0.
    iy, ix = np.where(arr >= 65)
    occupied = np.column_stack((ix+.5, iy+.5))*resolution+origin
    # 5 cm tolerance accommodates the 2 cm raster without crediting entire wall objects.
    tolerance = .05
    recalls = []
    for edge in edges:
        hits = []
        for chunk in np.array_split(edge, max(1, math.ceil(len(edge)/100))):
            hits.extend((np.linalg.norm(chunk[:, None]-occupied[None], axis=2).min(axis=1) <= tolerance).tolist()
                        if len(occupied) else [False]*len(chunk))
        recalls.append(float(np.mean(hits)))
    inside = (occupied >= lo).all(axis=1) & (occupied <= hi).all(axis=1)
    phantom = float(np.mean(clearance(occupied[inside], walls) > tolerance)) if inside.any() else 0.
    minimum_recall = min(recalls)
    return {'interior_unknown_fraction': unknown, 'known_corridor_purity': purity,
            'wall_surface_recall': recalls, 'minimum_wall_surface_recall': minimum_recall,
            'phantom_fraction': phantom, 'bounds_m': [lo.tolist(), hi.tolist()],
            'wall_tolerance_m': tolerance, 'interior_wall_exclusion_m': .03,
            'map_raster_complete': bool(unknown <= .01 and purity >= .95 and minimum_recall >= .98 and phantom < .02)}


def topology(walls, spawn, step=.002):
    """Point connectivity at lidar height; this is not robot-radius reachability."""
    import cv2
    edge = np.concatenate([wall_edges(wall) for wall in walls])
    lo, hi = edge.min(axis=0), edge.max(axis=0)
    # Exclude the outer 1 cm wall strip, including sub-cell exterior slivers.
    lo, hi = lo+.01, hi-.01
    x, y = np.meshgrid(np.arange(lo[0]+step/2, hi[0], step),
                       np.arange(lo[1]+step/2, hi[1], step))
    free = clearance(np.stack((x, y), axis=-1), walls) > 0
    count, labels = cv2.connectedComponents(free.astype(np.uint8), connectivity=4)
    col, row = np.floor((np.array(spawn)-lo)/step).astype(int)
    if not (0 <= row < labels.shape[0] and 0 <= col < labels.shape[1]) or labels[row, col] == 0:
        raise ValueError('Spawn must lie in free space')
    sealed = free & (labels != labels[row, col])
    return {'sampling_m': step, 'free_components': count-1,
            'sealed_free_area_m2': float(sealed.sum()*step*step),
            'sealed_free_fraction': float(sealed.sum()/free.sum()),
            'sealed_centroid_m': [float(x[sealed].mean()), float(y[sealed].mean())] if sealed.any() else None}


def main():
    folder = Path(sys.argv[1])
    saved = np.load(folder/'track_map.npz')
    identity = json.loads((folder/'track_identity.json').read_text())
    result = measure(saved['data'], saved['origin'], float(saved['resolution']), identity['walls'])
    result['topology'] = topology(identity['walls'], identity['spawn'])
    result['map_npz_sha256'] = hashlib.sha256((folder/'track_map.npz').read_bytes()).hexdigest()
    result['source_world_sha256'] = identity['sha256']
    (folder/'track_map_audit.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result))
    return 0 if result['map_raster_complete'] else 1


if __name__ == '__main__':
    sys.exit(main())
