"""Subject: one associated planar wall for straight calibration, never safety."""
import math
from statistics import median

from .lidar import robot_yaw


def _fit(points, allow_exclusion=True):
    if len(points) < 6 or points[-1][0]-points[0][0] < math.radians(5):
        return None
    # Wide-baseline pair slopes resist isolated noise. Accepted support must
    # satisfy 3mm; sparse isolated/boundary returns may be excluded once only.
    quarter = max(1, len(points)//4)
    slopes = [(q[2]-p[2])/(q[1]-p[1]) for p in points[:quarter]
              for q in points[-quarter:] if abs(q[1]-p[1]) > 1e-6]
    if not slopes:
        return None
    a = median(slopes)
    b = median(x-a*y for _, y, x in points)
    errors = [abs(x-a*y-b)/math.hypot(1., a) for _, y, x in points]
    residual = max(errors)
    if abs(a) > 2.5 or not .05 < b < 8.:
        return None
    if residual > .003:
        excluded = [i for i, error in enumerate(errors) if error > .003]
        if (not allow_exclusion or len(excluded) > .1*len(points) or residual > .010
                or any(j == i+1 for i, j in zip(excluded, excluded[1:]))):
            return None
        fit = _fit([p for i, p in enumerate(points) if i not in excluded], False)
        if fit is None:
            return None
        raw_residual = max(abs(x-fit['slope']*y-fit['intercept'])/math.hypot(1., fit['slope'])
                           for _, y, x in points)
        if raw_residual > .010:
            return None
        fit.update(excluded_rays=len(excluded), raw_residual_m=raw_residual,
                   _raw_points=tuple(points))
        return fit
    return {'slope': a, 'intercept': b, 'lo': points[0][0],
            'hi': points[-1][0], 'rays': len(points), 'residual_m': residual,
            'excluded_rays': 0, 'raw_residual_m': residual, 'outlier_cap_m': .010,
            '_points': tuple(points), '_raw_points': tuple(points)}


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
        self.observed_pose = self.observed_time = None
        self.diagnostic = {'status': 'collecting', 'reason': 'No associated wall'}

    def update(self, scan, nose, locked=False, pose=None, mount=None, now=None):
        self.locked = self.locked or bool(locked)
        compensation = None
        prediction = None
        if now is not None:
            if (not math.isfinite(now) or pose is None or mount is None
                    or len(pose) != 3 or not all(math.isfinite(v) for v in pose)):
                self.stable = 0
                self.diagnostic = {'status': 'invalid', 'reason': 'Prediction requires finite time and pose'}
                return math.inf
            if self.wall is not None:
                if self.observed_time is None or self.observed_pose is None or self.pose_anchor is None:
                    self.stable = 0
                    self.diagnostic = {'status': 'invalid', 'reason': 'Prediction history unavailable'}
                    return math.inf
                dt = now-self.observed_time
                step = ((pose[0]-self.observed_pose[0])*math.cos(self.pose_anchor[2])
                        +(pose[1]-self.observed_pose[1])*math.sin(self.pose_anchor[2]))
                if not 0 < dt <= 1.2 or abs(step) > .014*min(dt,.2)+.003:
                    self.stable = 0
                    self.diagnostic = {'status': 'invalid', 'reason': 'Prediction pose step or interval implausible'}
                    return math.inf
                prediction = {'predicted_b': self.wall['intercept']-step,
                              'forward_step_m': step, 'dt_s': dt}
        elif self.observed_time is not None:
            self.stable = 0
            self.diagnostic = {'status': 'invalid', 'reason': 'Prediction time missing'}
            return math.inf
        if pose is not None or mount is not None or self.pose_anchor is not None:
            if (pose is None or mount is None or len(pose) != 3 or len(mount) != 2
                    or not all(math.isfinite(v) for v in (*pose, *mount))):
                self.diagnostic = {'status': 'invalid', 'reason': 'Finite pose and mount required'}
                if self.locked:
                    self.stable = 0
                return math.inf
            anchor = self.pose_anchor if self.pose_anchor is not None else tuple(pose)
            initial_mount = self.mount_anchor if self.mount_anchor is not None else tuple(mount)
            yaw = math.atan2(math.sin(pose[2]-anchor[2]), math.cos(pose[2]-anchor[2]))
            lateral = -(pose[0]-anchor[0])*math.sin(anchor[2])+(pose[1]-anchor[1])*math.cos(anchor[2])
            if abs(yaw) > .1 or abs(lateral) > .015 or any(abs(a-b) > 1e-6 for a,b in zip(mount, initial_mount)):
                self.diagnostic = {'status': 'invalid', 'reason': 'Pose or mount outside calibration envelope'}
                if self.locked:
                    self.stable = 0
                return math.inf
            compensation = (yaw, lateral, tuple(mount), initial_mount)
        candidates = _segments(scan, nose, compensation)
        if candidates and pose is not None and self.pose_anchor is None:
            self.pose_anchor, self.mount_anchor = tuple(pose), tuple(mount)
        matches = []
        reference_b = prediction['predicted_b'] if prediction is not None else (
            self.wall['intercept'] if self.wall is not None else None)
        if self.wall is not None:
            for wall in candidates:
                overlap = min(wall['hi'], self.wall['hi'])-max(wall['lo'], self.wall['lo'])
                width = min(wall['hi']-wall['lo'], self.wall['hi']-self.wall['lo'])
                if (overlap >= .5*width and abs(wall['slope']-self.anchor['slope']) <= .12
                        and abs(wall['intercept']-reference_b) <= .012
                        and abs(wall['intercept']-self.anchor['intercept']) <= .06):
                    matches.append(wall)
        if len(matches) > 1:
            points = sorted(point for wall in matches for point in wall['_raw_points'])
            merged = _fit(points)
            # A short fragment's slope is noisy outside its observed support.
            # Check continuity at adjacent boundaries, not by extrapolating
            # every fragment across the far end of the entire wall. The joint
            # fit still requires 3mm support; an 8mm parallel step cannot hide
            # behind a tilted joint fit because its boundary is discontinuous.
            ordered = sorted(matches, key=lambda wall: wall['lo'])
            equivalent = merged is not None
            for left, right in zip(ordered, ordered[1:]):
                boundary_y = (left['_points'][-1][1]+right['_points'][0][1])/2
                separation = abs((left['slope']-right['slope'])*boundary_y+
                                 left['intercept']-right['intercept'])
                normal = min(math.hypot(1., left['slope']), math.hypot(1., right['slope']))
                equivalent = equivalent and left['hi'] < right['lo'] and separation/normal <= .003
            if equivalent and (abs(merged['slope']-self.anchor['slope']) <= .12
                    and abs(merged['intercept']-reference_b) <= .012
                    and abs(merged['intercept']-self.anchor['intercept']) <= .06):
                merged['fragments'] = len(matches)
                matches = [merged]
        if len(matches) > 1:
            # A short edge segment can meet the broad reacquisition window
            # without representing the established wall. Retain a unique
            # live plane only if it covers >=2/3 of the previous support and
            # stays within the 3mm predicted normal-distance precision. Equal
            # competing surfaces, an 8mm replacement, and overlapping support
            # remain ambiguous; no historical distance substitutes for a scan.
            old_width = self.wall['hi']-self.wall['lo']
            dominant = [wall for wall in matches
                        if min(wall['hi'], self.wall['hi'])-max(wall['lo'], self.wall['lo']) >= 2*old_width/3
                        and abs(wall['intercept']-reference_b)/math.hypot(1., wall['slope']) <= .003]
            if len(dominant) == 1:
                chosen = dominant[0]
                peripheral = [wall for wall in matches if wall is not chosen]
                if all(wall['hi'] < chosen['lo'] or wall['lo'] > chosen['hi'] for wall in peripheral):
                    chosen = dict(chosen, association='dominant_support', peripheral_fragments=len(peripheral))
                    matches = [chosen]
        if len(matches) == 1:
            self.wall = matches[0]
            self.stable += 1
        elif self.locked:
            self.stable = 0
            self.diagnostic = {'status': 'invalid', 'reason': 'Tracked wall missing or ambiguous',
                               'candidates': len(candidates), 'matches': len(matches)}
            return math.inf
        elif candidates:
            self.wall = max(candidates, key=lambda w: (w['hi']-w['lo'], w['rays']))
            self.anchor = dict(self.wall)
            self.stable = 1
            prediction = None
        else:
            self.wall = self.anchor = None
            self.stable = 0
        if self.wall is not None and now is not None:
            self.observed_pose, self.observed_time = tuple(pose), now
        elif self.wall is None:
            self.observed_pose = self.observed_time = None
        if self.wall is None or self.stable < 3:
            self.diagnostic = {'status': 'collecting', 'reason': 'Waiting for three associated scans'}
            return math.inf
        public_wall = {key: value for key, value in self.wall.items() if not key.startswith('_')}
        self.diagnostic = {'status': 'ok', 'locked': self.locked, **public_wall,
                           'prediction': None if prediction is None else {
                               **prediction, 'innovation_m': self.wall['intercept']-prediction['predicted_b']},
                           'compensation': None if compensation is None else {
                               'yaw_rad': compensation[0], 'lateral_m': compensation[1],
                               'mount_m': list(compensation[2]), 'forward_odom_used': False}}
        return self.wall['intercept']
