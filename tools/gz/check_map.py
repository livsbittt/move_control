#!/usr/bin/env python3
"""Verify the saved maze map against the SDF ground truth.

map_saver output is only as right as the SLAM that painted it; this makes
"the map is correct" a measurement instead of a feeling. Rasterizes the SDF
wall boxes at the map's own resolution/origin, then measures:

  wall recall      SDF wall cells with an occupied map pixel within tol
  corridor purity  known corridor cells that are free (not painted wall)
  phantom walls    map occupied pixels farther than tol from any SDF wall
  interior unknown fraction of walkable interior left unknown

Writes map/gz_maze_overlay.png (map + SDF wall outlines in red) and prints
the four numbers. Exit 1 if any gate fails.
"""
import argparse
import os
import re
import sys

import cv2
import numpy as np

MAZE = (0.0, 3.9)   # SDF walls span [0, 3.9] x [0, 3.9]
INTERIOR = (0.3, 3.6)  # walkable interior, away from the outer shell


def parse_sdf_walls(path):
    """Wall rectangles (x0, x1, y0, y1) from the maze collision boxes.

    All boxes carry yaw 0 in the generated SDF, so pose +- half size is
    the axis-aligned rectangle.
    """
    sdf = open(path).read()
    pat = (r'<collision name="w\d+">\s*<pose>([^<]+)</pose>\s*'
           r'<geometry><box><size>([^<]+)</size>')
    walls = []
    for m in re.finditer(pat, sdf):
        p = [float(v) for v in m.group(1).split()]
        s = [float(v) for v in m.group(2).split()]
        walls.append((p[0] - s[0] / 2, p[0] + s[0] / 2,
                      p[1] - s[1] / 2, p[1] + s[1] / 2))
    return walls


def load_map(yaml_path):
    """(pgm uint8 h x w, res, ox, oy) from map_saver's yaml + pgm pair."""
    txt = open(yaml_path).read()
    image = re.search(r'image:\s*(\S+)', txt).group(1)
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(os.path.abspath(yaml_path)),
                             image)
    res = float(re.search(r'resolution:\s*([\d.]+)', txt).group(1))
    ox, oy = [float(v) for v in
              re.search(r'origin:\s*\[([^\]]+)\]', txt).group(1).split(',')[:2]]
    img = load_pgm(image)
    if img is None:
        raise SystemExit(f'cannot load pgm {image}')
    return img, res, ox, oy


def load_pgm(path):
    data = open(path, 'rb').read()
    if not data.startswith(b'P5'):
        return None
    m = re.match(rb'P5\s+(\d+)\s+(\d+)\s+(\d+)\s', data)
    if not m:
        return None
    w, h, maxv = int(m.group(1)), int(m.group(2)), int(m.group(3))
    start = m.end()
    return np.frombuffer(data[start:start + w * h],
                         np.uint8).reshape(h, w).copy()


def raster_walls(walls, raster_fn, h, w, clip=None):
    """bool mask from world rectangles via world->pixel mapper."""
    mask = np.zeros((h, w), bool)
    for x0, x1, y0, y1 in walls:
        (px0, py0) = raster_fn(x0, y1)
        (px1, py1) = raster_fn(x1, y0)
        px0, px1 = sorted((int(px0), int(px1) + 1))
        py0, py1 = sorted((int(py0), int(py1) + 1))
        if clip:
            px0, py0 = clip(px0, py0)
            px1, py1 = clip(px1, py1)
        mask[py0:py1, px0:px1] = True
    return mask


def measure(img, res, ox, oy, walls, tol_m=0.10):
    """Metrics of one map raster vs SDF wall rects: dict of numbers+masks.

    Gates mirror the PRD: walls present, corridors free, few phantom walls,
    interior covered. Dashboard /result.json reuses this on the saved map.
    """
    h, w = img.shape
    # Unobserved world outside SLAM's current bounds is still unknown.
    # Pad on the existing lattice so the denominator covers the full maze.
    left = max(0, int(np.ceil((ox - MAZE[0]) / res)))
    bottom = max(0, int(np.ceil((oy - MAZE[0]) / res)))
    right = max(0, int(np.ceil((MAZE[1] - ox - w * res) / res)))
    top = max(0, int(np.ceil((MAZE[1] - oy - h * res) / res)))
    img = np.pad(img, ((top, bottom), (left, right)),
                 constant_values=205)
    ox, oy = ox - left * res, oy - bottom * res
    h, w = img.shape

    def w2p(x, y):
        return (x - ox) / res, h - 1 - (y - oy) / res

    def clip(px, py):
        return (max(0, min(w, px)), max(0, min(h, py)))

    wall_px = raster_walls(walls, w2p, h, w, clip=clip)
    known = img != 205
    occ = img < 100
    free = img > 230

    def box(x0, y0, x1, y1):
        px0, py0 = w2p(x0, y1)
        px1, py1 = w2p(x1, y0)
        px0, px1 = sorted((int(px0), int(px1) + 1))
        py0, py1 = sorted((int(py0), int(py1) + 1))
        px0, py0 = clip(px0, py0)
        px1, py1 = clip(px1, py1)
        m = np.zeros((h, w), bool)
        m[py0:py1, px0:px1] = True
        return m

    maze_m = box(MAZE[0], MAZE[0], MAZE[1], MAZE[1])
    walkable_m = box(INTERIOR[0], INTERIOR[0],
                     INTERIOR[1], INTERIOR[1]) & ~wall_px
    corridor_m = maze_m & ~wall_px

    # Per-WALL detection: a 0.3 m thick wall is only ever seen from its
    # facing side, so per-cell recall over the full box undercounts 5:1.
    # A wall counts as found when an occupied pixel sits within tol of it.
    tol = max(1, int(round(tol_m / res)))
    occ_dil = cv2.dilate(occ.astype(np.uint8),
                         np.ones((2 * tol + 1,) * 2, np.uint8)) > 0
    hit = 0
    for x0, x1, y0, y1 in walls:
        m = box(x0, y0, x1, y1)
        if m.any() and (occ_dil & m).any():
            hit += 1
    recall = hit / max(1, len(walls))

    # Explored-adjacent walls: a wall the map never faced cannot be in the
    # map, so correctness is measured among walls bordering known space.
    known_dil = cv2.dilate(known.astype(np.uint8),
                           np.ones((7, 7), np.uint8)) > 0
    adj = 0
    hit_adj = 0
    for x0, x1, y0, y1 in walls:
        m = box(x0, y0, x1, y1)
        if not m.any() or not (known_dil & m).any():
            continue
        adj += 1
        if (occ_dil & m).any():
            hit_adj += 1
    wall_detect_adj = hit_adj / max(1, adj)
    n_wall_adj = adj

    kcorr = known & corridor_m
    purity = (free & corridor_m)[kcorr].mean() if kcorr.any() else 0.0

    dist_px = cv2.distanceTransform((~wall_px).astype(np.uint8),
                                    cv2.DIST_L2, 3)
    ph = int((occ & (dist_px > tol)).sum())
    ph_frac = ph / max(1, int(occ.sum()))

    walk = int(walkable_m.sum())
    unk_frac = (walk - int((known & walkable_m).sum())) / max(1, walk)
    n_walkable_px = walk

    return {
        'w': w, 'h': h, 'res': res, 'ox': ox, 'oy': oy,
        'walls': len(walls), 'occ_px': int(occ.sum()),
        'known_px': int((known & maze_m).sum()),
        'wall_recall': round(float(recall), 3),
        'n_wall': len(walls), 'n_wall_adj': n_wall_adj,
        'wall_detect_adj': round(float(wall_detect_adj), 3),
        'corridor_purity': round(float(purity), 3),
        'phantom_frac': round(float(ph_frac), 3),
        'interior_unknown': round(float(unk_frac), 3),
        'walkable_px': n_walkable_px,
        'img': img, 'wall_px': wall_px, 'maze_m': maze_m,
        # Amended PRD gates: purity >=0.9, phantom <0.05, per-wall
        # detection among explored-adjacent walls >=0.85, interior
        # unknown <=0.40 (the 60 percent plateau coverage).
        'gates': {
            'wall_detect_adj>=0.85': bool(wall_detect_adj >= 0.85),
            'corridor_purity>=0.9': bool(purity >= 0.9),
            'phantom<0.05': bool(ph_frac < 0.05),
            'interior_unknown<=0.40': bool(unk_frac <= 0.40),
        },
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', default='map/gz_maze.yaml')
    ap.add_argument('--sdf', default='tools/gz/pinky_maze.sdf')
    ap.add_argument('--tol', type=float, default=0.10,
                    help='wall match tolerance, m')
    ap.add_argument('--out', default='map/gz_maze_overlay.png')
    a = ap.parse_args()

    img, res, ox, oy = load_map(a.map)
    walls = parse_sdf_walls(a.sdf)
    r = measure(img, res, ox, oy, walls, tol_m=a.tol)
    img = r['img']
    h, w = r['h'], r['w']
    wall_px, occ, known = r['wall_px'], r['img'] < 100, r['img'] != 205
    occ_dil = cv2.dilate(occ.astype(np.uint8),
                         np.ones((2 * int(round(a.tol / res)) + 1,) * 2,
                                 np.uint8)) > 0
    maze_m = r['maze_m']
    recall = r['wall_recall']
    purity = r['corridor_purity']
    ph_frac = r['phantom_frac']
    unk_frac = r['interior_unknown']
    print(f"map {w}x{h} res={res} origin=({r['ox']:.3f},{r['oy']:.3f}) "
          f"walls={len(walls)}")
    print(f"wall_recall(per-wall) {recall:.3f} "
          f"(explored-adjacent {r['wall_detect_adj']:.3f} of "
          f"{r['n_wall_adj']})")
    print(f"corridor_purity {purity:.3f}")
    print(f"phantom_frac {ph_frac:.3f} occ_px={r['occ_px']}")
    print(f"interior_unknown {unk_frac:.3f}")
    gates = r['gates']
    print('gates:', gates)
    ok = all(gates.values())
    if not ok:
        print('FAIL')
    # The web node's /result.json serves this file verbatim; keep the
    # name stable (map/gz_maze_metrics.json next to the saved map).
    import json
    metrics_path = os.path.join(os.path.dirname(os.path.abspath(a.map)),
                                'gz_maze_metrics.json')
    with open(metrics_path, 'w') as f:
        json.dump({
            'wall_recall': recall, 'n_wall': r['n_wall'],
            'n_wall_adj': r['n_wall_adj'],
            'wall_detect_adj': r['wall_detect_adj'],
            'corridor_purity': purity, 'phantom_frac': ph_frac,
            'interior_unknown': unk_frac,
            'n_walkable': r.get('walkable_px', 0),
            'gates': gates, 'ok': bool(ok),
        }, f, indent=2)
    print('metrics ->', metrics_path)
    # Evidence: map | SDF truth | overlay (SDF wall outline on the map).
    truth = np.full((h, w, 3), 255, np.uint8)
    truth[wall_px] = (0, 0, 0)
    truth[~maze_m] = (205, 205, 205)
    over = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    edge = cv2.dilate(wall_px.astype(np.uint8), np.ones((3, 3), np.uint8)) \
        != wall_px
    over[edge & maze_m] = (0, 0, 255)
    over[wall_px] = np.minimum(over[wall_px], (0, 0, 255))
    panel = np.concatenate([cv2.cvtColor(img, cv2.COLOR_GRAY2BGR), truth,
                            over], axis=1)
    cv2.imwrite(a.out, panel)
    print('overlay ->', a.out)
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
