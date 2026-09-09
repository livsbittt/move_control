"""Subject: image-space foreground regions, without guessed class or range."""
import numpy as np


def foreground_regions(mask, dark, near_y0, near_y1, min_area_fraction=.0005):
    import cv2
    h, w = mask.shape
    minimum = max(12, int(h*w*min_area_fraction))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8), 8)
    near = labels[int(near_y0*h):int(near_y1*h), w//3:2*w//3]
    near_counts = np.bincount(near.ravel(), minlength=count)
    regions = []
    for identity in range(1, count):
        x, y, width, height, area = map(int, stats[identity])
        # Preserve two-pixel poles/wires; reject isolated pixels without
        # eroding every foreground object with a fixed five-pixel kernel.
        if area < minimum or min(width, height) < 2:
            continue
        component = labels[y:y+height, x:x+width] == identity
        darkness = np.count_nonzero(dark[y:y+height, x:x+width] & component)/area
        regions.append(dict(bbox_xyxy=[x,y,x+width,y+height], area_px=area,
            area_fraction=area/(h*w), near_path=bool(near_counts[identity] >= minimum),
            kind='dark_region' if darkness > .5 else 'foreground_region',
            distance_m=None, motion='unknown'))
    regions.sort(key=lambda r: (r['near_path'], r['area_px']), reverse=True)
    return regions
