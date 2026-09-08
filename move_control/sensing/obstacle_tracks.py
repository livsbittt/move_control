"""Subject: conservative temporal association in a continuous odometry frame."""
import math
from .pose import planar_pose


def scan_plane_pose(x, y, quaternion, max_tilt=math.radians(5.)):
    """A pitched laser plane cannot provide distances on a horizontal map."""
    pose = planar_pose(x, y, quaternion)
    if pose is None or not math.isfinite(max_tilt) or not 0 <= max_tilt < math.pi/2:
        return None
    norm2 = sum(v*v for v in quaternion)
    if 1-2*(quaternion[0]**2+quaternion[1]**2)/norm2 < math.cos(max_tilt):
        return None
    return pose


def observed_free(position, radius, ranges, angle_min, increment):
    """Require finite clear returns across the whole previous angular extent."""
    distance = math.hypot(*position)
    if increment <= 0 or distance <= radius or radius < 0:
        return False
    center = math.atan2(position[1], position[0])
    half = math.asin(min(1., radius/distance))+increment
    selected = []
    offsets = []
    for i, value in enumerate(ranges):
        offset = (angle_min+i*increment-center+math.pi) % (2*math.pi)-math.pi
        if abs(offset) <= half:
            offsets.append(offset)
            selected.append(value)
    return (len(selected) >= 3 and min(offsets) <= -half+increment and
            max(offsets) >= half-increment and
            all(math.isfinite(v) and v > distance+radius+.03 for v in selected))


def scan_clusters(ranges, angle_min, angle_increment, range_min, range_max,
                  gap=.08, min_points=3):
    """Adjacent scan returns; coordinates remain in the sensor's TF frame."""
    groups, current = [], []
    for index, distance in enumerate(ranges):
        valid = math.isfinite(distance) and range_min <= distance <= range_max
        angle = angle_min+index*angle_increment
        point = (distance*math.cos(angle), distance*math.sin(angle)) if valid else None
        if current and (point is None or math.dist(current[-1], point) > gap):
            groups.append(current)
            current = []
        if point is not None:
            current.append(point)
    if current:
        groups.append(current)
    result = []
    for group in groups:
        if len(group) < min_points:
            continue
        center = tuple(sum(p[k] for p in group)/len(group) for k in (0, 1))
        result.append(dict(position=center, radius=max(math.dist(center, p) for p in group)))
    return result


def transform_points(points, pose):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x+c*px-s*py, y+s*px+c*py) for px, py in points]


class ObstacleTracker:
    """Centroid tracks are evidence, never a declaration that lost space is free.

    Callers own timestamped extrinsics and ego-motion compensation. Ambiguous
    associations restart evidence instead of converting an ID swap into speed.
    """
    def __init__(self, association=.15, evidence_time=.6, moving_speed=.06,
                 stationary_speed=.02, stale_after=.3, retain_for=1.5):
        values = (association, evidence_time, moving_speed, stationary_speed,
                  stale_after, retain_for)
        if not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError('Positive finite tracking limits required')
        if stationary_speed >= moving_speed or stale_after >= retain_for:
            raise ValueError('Tracking hysteresis must be ordered')
        self.association, self.evidence_time = association, evidence_time
        self.moving_speed, self.stationary_speed = moving_speed, stationary_speed
        self.stale_after, self.retain_for = stale_after, retain_for
        self.tracks, self.next_id, self.last_time = {}, 1, None

    def update(self, points, stamp):
        points = [tuple(p) for p in points]
        if not math.isfinite(stamp) or any(len(p) != 2 or
                not all(math.isfinite(v) for v in p) for p in points):
            raise ValueError('Finite timestamp and planar points required')
        if self.last_time is not None and stamp <= self.last_time:
            return self.snapshot(self.last_time)
        self.last_time = stamp
        candidates = {}
        for j, p in enumerate(points):
            distances = sorted((math.dist(p, t['position']), i) for i, t in self.tracks.items()
                               if math.dist(p, t['position']) <= self.association)
            if len(distances) > 1 and distances[1][0]-distances[0][0] > .03:
                distances = distances[:1]
            candidates[j] = [i for _, i in distances]
        counts = {i: sum(i in ids for ids in candidates.values()) for i in self.tracks}
        for t in self.tracks.values():
            t['observed'] = False
        assigned_groups = set()
        for j, p in enumerate(points):
            ids = candidates[j]
            if len(ids) == 1 and counts[ids[0]] == 1:
                t = self.tracks[ids[0]]
                if stamp-t['stamp'] > self.stale_after:
                    t['history'] = []
                t['history'].append((stamp, p))
                t['history'] = [(ts, xy) for ts, xy in t['history']
                                if stamp-ts <= max(1.2, self.evidence_time*2)]
                t.update(position=p, stamp=stamp, observed=True)
            else:
                # Keep ambiguous hypotheses until free-ray proof; no guessed match.
                for i in ids:
                    self.tracks[i]['history'] = []
                groups = [i for i in ids if self.tracks[i].get('ambiguous') and i not in assigned_groups]
                if groups:
                    # Repeated merged returns refresh one unknown group rather
                    # than allocating a permanent ghost for every scan.
                    identity = min(groups, key=lambda i: math.dist(p, self.tracks[i]['position']))
                    assigned_groups.add(identity)
                    self.tracks[identity].update(position=p, stamp=stamp, observed=True,
                                                history=[(stamp, p)])
                    continue
                t = dict(id=self.next_id, position=p, stamp=stamp,
                         observed=True, history=[(stamp, p)], ambiguous=bool(ids))
                self.tracks[self.next_id] = t
                self.next_id += 1
        return self.snapshot(stamp)

    def snapshot(self, now):
        result = []
        for t in self.tracks.values():
            age = now-t['stamp']
            if age < 0:
                continue
            state, velocity = 'unknown', (0., 0.)
            history = t['history']
            if t['observed'] and age <= self.stale_after and len(history) >= 3:
                dt = history[-1][0]-history[0][0]
                if dt >= self.evidence_time:
                    velocity = tuple((history[-1][1][k]-history[0][1][k])/dt for k in (0, 1))
                    speed = math.hypot(*velocity)
                    travelled = sum(math.dist(a[1], b[1]) for a, b in zip(history, history[1:]))/dt
                    if speed >= self.moving_speed:
                        state = 'moving'
                    elif speed <= self.stationary_speed and travelled <= self.stationary_speed:
                        state = 'stationary'
            result.append(dict(id=t['id'], position=t['position'], velocity=velocity,
                               state=state, age=age, observed=t['observed'] and age <= self.stale_after))
        return result

    def clear_observed_free(self, ids):
        """Only the sensor adapter may retire hypotheses after free-ray proof."""
        for identity in ids:
            self.tracks.pop(identity, None)
