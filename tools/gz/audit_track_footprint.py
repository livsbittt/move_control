"""Independent sampled collision audit against the original world wall boxes."""
import json
import math
from pathlib import Path
import sys

import cv2
import numpy as np


def separation(a, b):
    """Positive separating-axis gap is a lower bound on polygon distance."""
    edges = np.concatenate((np.roll(a, -1, axis=0)-a, np.roll(b, -1, axis=0)-b))
    axes = np.column_stack((-edges[:, 1], edges[:, 0]))
    axes /= np.linalg.norm(axes, axis=1)[:, None]
    pa, pb = a @ axes.T, b @ axes.T
    return float(np.maximum(pa.min(axis=0)-pb.max(axis=0),
                            pb.min(axis=0)-pa.max(axis=0)).max())


def audit(identity, samples):
    poses = np.asarray(samples, dtype=float)
    vertices = np.asarray(identity['robot_geometry']['footprint_xy'], dtype=float)
    if (poses.ndim != 2 or poses.shape[1] != 4 or not len(poses) or
            not np.isfinite(poses).all() or np.any(np.diff(poses[:, 0]) <= 0) or
            vertices.ndim != 2 or vertices.shape[1] != 2 or len(vertices) < 3 or
            not np.isfinite(vertices).all() or not identity['walls']):
        raise ValueError('Finite ordered poses and nonempty collision geometry required')
    hull = cv2.convexHull(np.asarray(identity['robot_geometry']['footprint_xy'],
                                   dtype=np.float32)).reshape(-1, 2).astype(float)
    if len(hull) < 3 or cv2.contourArea(hull.astype(np.float32)) <= 0:
        raise ValueError('Nondegenerate footprint required')
    walls = []
    for pose, size in identity['walls']:
        if (len(pose) != 6 or len(size) != 3 or not np.isfinite(pose+size).all() or
                min(size) <= 0 or abs(pose[3])+abs(pose[4]) > 1e-9):
            raise ValueError('Finite planar wall boxes required')
        c, s = math.cos(pose[5]), math.sin(pose[5])
        box = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]])*np.array(size[:2])/2
        walls.append(box @ np.array([[c, s], [-s, c]])+pose[:2])
    minimum, worst, max_step, max_dt, previous = math.inf, None, 0., 0., None
    for stamp, x, y, yaw in samples:
        c, s = math.cos(yaw), math.sin(yaw)
        body = hull @ np.array([[c, s], [-s, c]])+[x, y]
        gap = min(separation(body, wall) for wall in walls)
        if gap < minimum:
            minimum, worst = gap, [stamp, x, y, yaw]
        if previous is not None:
            max_step = max(max_step, float(np.linalg.norm(body-previous[1], axis=1).max()))
            max_dt = max(max_dt, stamp-previous[0])
        previous = stamp, body
    return dict(sample_count=len(samples), minimum_sampled_separating_gap_m=minimum,
                worst_pose=worst, maximum_vertex_sample_step_m=max_step,
                maximum_sample_interval_s=max_dt,
                sampled_collision_free=bool(samples and minimum > 0),
                continuous_collision_proof=False,
                limitation='Ground-truth sampled projected collision hull; no contact sensor or intersample dynamic proof.')


def main():
    folder = Path(sys.argv[1])
    result = audit(json.loads((folder/'track_identity.json').read_text()),
                   json.loads((folder/'track_odometry.json').read_text()))
    (folder/'track_footprint_audit.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
