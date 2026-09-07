"""Subject: one associated planar wall for straight calibration, never safety."""
import math
from statistics import median

from .lidar import robot_yaw


def _fit(points):
    if len(points) < 6 or points[-1][0]-points[0][0] < math.radians(5):
        return None
    # Wide-baseline pair slopes resist isolated noise; every original return
    # must subsequently satisfy the physical 3mm wall-normal residual.
    quarter = max(1, len(points)//4)
    slopes = [(q[2]-p[2])/(q[1]-p[1]) for p in points[:quarter]
              for q in points[-quarter:] if abs(q[1]-p[1]) > 1e-6]
    if not slopes:
        return None
    a = median(slopes)
    b = median(x-a*y for _, y, x in points)
    residual = max(abs(x-a*y-b)/math.hypot(1., a) for _, y, x in points)
    if abs(a) > 2.5 or not .05 < b < 8. or residual > .003:
        return None
    return {'slope': a, 'intercept': b, 'lo': points[0][0],
            'hi': points[-1][0], 'rays': len(points), 'residual_m': residual,
            '_points': tuple(points)}


def _segments(scan, nose, compensation=None):
    rays = []
    for i, r in enumerate(scan.ranges):
        angle = robot_yaw(scan.angle_min+i*scan.angle_increment, nose)
        if abs(angle) <= math.radians(35)+1e-9:
            valid = math.isfinite(r) and max(.05, scan.range_min) < r < min(8., scan.range_max)
            rays.append((angle, r if valid else None))
    rays.sort()
    # Merge the duplicated +/-pi endpoint only when ranges agree; an
    # inconsistent endpoint becomes a break rather than joining two walls.
    unique = []
    for a, r in rays:
        if unique and abs(a-unique[-1][0]) < 1e-5:
            old_a, old_r = unique[-1]
            unique[-1] = (old_a, r if old_r is None else old_r if r is None else
                          (old_r+r)/2 if abs(old_r-r) <= .003 else None)
        else:
            unique.append((a, r))
    runs, run = [], []
    for a, r in unique:
        point = (a, r*math.sin(a), r*math.cos(a)) if r is not None else None
        if point is None or (run and (a-run[-1][0] > math.radians(2)+1e-9 or
                math.hypot(point[1]-run[-1][1], point[2]-run[-1][2]) > .04)):
            if run:
                runs.append(run)
            run = []
        if point is not None:
            run.append(point)
    if run:
        runs.append(run)
    if compensation is not None:
        yaw, lateral, mount, initial_mount = compensation
        c, s = math.cos(yaw), math.sin(yaw)
        # Rotate about base_link, retaining the LiDAR lever arm. Only lateral
        # odometry is added: forward odometry must not enter the range result.
        runs = [[(angle, s*(x+mount[0])+c*(y+mount[1])-initial_mount[1]+lateral,
                  c*(x+mount[0])-s*(y+mount[1])-initial_mount[0])
                 for angle, y, x in run] for run in runs]
    segments = []
    for run in runs:
        start = 0
        while start+6 <= len(run):
            end = start+6
            while end <= len(run) and run[end-1][0]-run[start][0] < math.radians(5):
                end += 1
            fit = _fit(run[start:end]) if end <= len(run) else None
            if fit is None:
                start += 1
                continue
            while end < len(run):
                grown = _fit(run[start:end+1])
                if grown is None:
                    break
                fit = grown
                end += 1
            segments.append(fit)
            start = end
    return segments


class WallTracker:
    """Select during stationary collection; lock identity for a <=4cm trial.

    This is geometric association, not global localization. Parallel nearby
    walls can be ambiguous, so multiple matches fail closed. Optional odometry
    compensates yaw/lateral movement, never forward displacement. Callers must
    retain freshness guards; angular/lateral odometry error remains uncertainty.
    """
    def __init__(self):
        self.reset()

    def reset(self):
        self.wall = self.anchor = None
        self.stable = 0
        self.locked = False
        self.pose_anchor = self.mount_anchor = None
        self.diagnostic = {'status': 'collecting', 'reason': 'No associated wall'}

    def update(self, scan, nose, locked=False, pose=None, mount=None):
        self.locked = self.locked or bool(locked)
        compensation = None
        if pose is not None or mount is not None or self.pose_anchor is not None:
            if (pose is None or mount is None or len(pose) != 3 or len(mount) != 2
                    or not all(math.isfinite(v) for v in (*pose, *mount))):
                self.diagnostic = {'status': 'invalid', 'reason': 'Finite pose and mount required'}
                return math.inf
            anchor = self.pose_anchor if self.pose_anchor is not None else tuple(pose)
            initial_mount = self.mount_anchor if self.mount_anchor is not None else tuple(mount)
            yaw = math.atan2(math.sin(pose[2]-anchor[2]), math.cos(pose[2]-anchor[2]))
            lateral = -(pose[0]-anchor[0])*math.sin(anchor[2])+(pose[1]-anchor[1])*math.cos(anchor[2])
            if abs(yaw) > .1 or abs(lateral) > .015 or any(abs(a-b) > 1e-6 for a,b in zip(mount, initial_mount)):
                self.diagnostic = {'status': 'invalid', 'reason': 'Pose or mount outside calibration envelope'}
                return math.inf
            compensation = (yaw, lateral, tuple(mount), initial_mount)
        candidates = _segments(scan, nose, compensation)
        if candidates and pose is not None and self.pose_anchor is None:
            self.pose_anchor, self.mount_anchor = tuple(pose), tuple(mount)
        matches = []
        if self.wall is not None:
            for wall in candidates:
                overlap = min(wall['hi'], self.wall['hi'])-max(wall['lo'], self.wall['lo'])
                width = min(wall['hi']-wall['lo'], self.wall['hi']-self.wall['lo'])
                if (overlap >= .5*width and abs(wall['slope']-self.anchor['slope']) <= .12
                        and abs(wall['intercept']-self.wall['intercept']) <= .012
                        and abs(wall['intercept']-self.anchor['intercept']) <= .06):
                    matches.append(wall)
        if len(matches) > 1:
            points = sorted(point for wall in matches for point in wall['_points'])
            merged = _fit(points)
            # A tilted bridge can fit two nearby parallel surfaces despite
            # neither fragment supporting the other. Require mutual support,
            # as well as the combined fit, before treating them as one wall.
            equivalent = merged is not None and all(
                abs(x-wall['slope']*y-wall['intercept'])/math.hypot(1., wall['slope']) <= .003
                for wall in matches for _, y, x in points)
            if equivalent and (abs(merged['slope']-self.anchor['slope']) <= .12
                    and abs(merged['intercept']-self.wall['intercept']) <= .012
                    and abs(merged['intercept']-self.anchor['intercept']) <= .06):
                merged['fragments'] = len(matches)
                matches = [merged]
        if len(matches) == 1:
            self.wall = matches[0]
            self.stable += 1
        elif self.locked:
            self.diagnostic = {'status': 'invalid', 'reason': 'Tracked wall missing or ambiguous',
                               'candidates': len(candidates), 'matches': len(matches)}
            return math.inf
        elif candidates:
            self.wall = max(candidates, key=lambda w: (w['hi']-w['lo'], w['rays']))
            self.anchor = dict(self.wall)
            self.stable = 1
        else:
            self.wall = self.anchor = None
            self.stable = 0
        if self.wall is None or self.stable < 3:
            self.diagnostic = {'status': 'collecting', 'reason': 'Waiting for three associated scans'}
            return math.inf
        public_wall = {key: value for key, value in self.wall.items() if not key.startswith('_')}
        self.diagnostic = {'status': 'ok', 'locked': self.locked, **public_wall,
                           'compensation': None if compensation is None else {
                               'yaw_rad': compensation[0], 'lateral_m': compensation[1],
                               'mount_m': list(compensation[2]), 'forward_odom_used': False}}
        return self.wall['intercept']
