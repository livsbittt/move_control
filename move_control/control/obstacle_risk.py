"""Subject: short-horizon relative collision evidence; never motor authority."""
import math

TRACK_UNCERTAINTY = .03


def accept_observation(value, previous, now, max_age=.5):
    """Reject unusable source time before updating the monotonic watermark."""
    try:
        stamp = value['stamp']
        if (type(stamp) not in (int, float) or not math.isfinite(stamp) or
                not 0 <= now-stamp <= max_age or
                (previous is not None and stamp <= previous['stamp'])):
            return previous
        return value
    except (KeyError, TypeError, ValueError):
        return previous


def camera_hold(evidence, now, max_age=.5):
    try:
        if not 0 <= now-evidence['stamp'] <= max_age or type(evidence['blocked']) is not bool:
            raise ValueError('Invalid image evidence')
        return 'camera_obstacle_unranged' if evidence['blocked'] else None
    except (TypeError, KeyError, ValueError):
        return 'camera_observation_unavailable'


def observation_risk(evidence, now, pose, speed, radius, margin, max_age=.3):
    try:
        age = now-evidence['stamp']
        if not 0 <= age <= max_age or evidence['frame'] != 'odom':
            raise ValueError('Stale or incompatible observation')
        tracks = [dict(t, age=t['age']+age) for t in evidence['tracks']]
    except (TypeError, KeyError, ValueError):
        return dict(action='wait', reason='obstacle_observation_unavailable', ids=[])
    return collision_risk(tracks, pose, speed, radius, margin)


def collision_risk(tracks, pose, speed, radius, margin, horizon=2., uncertainty=TRACK_UNCERTAINTY):
    values = (*pose, speed, radius, margin, horizon, uncertainty)
    if not all(math.isfinite(v) for v in values) or radius <= 0 or min(margin, uncertainty) < 0 or horizon <= 0:
        return dict(action='wait', reason='invalid_geometry', ids=[])
    vx, vy = speed*math.cos(pose[2]), speed*math.sin(pose[2])
    action, ids = 'clear', []
    for t in tracks:
        try:
            px, py = t['position'][0]-pose[0], t['position'][1]-pose[1]
            ox, oy = t['velocity']
            extent = float(t.get('radius', 0.))
            age = float(t['age'])
            if not all(math.isfinite(v) for v in (px, py, ox, oy, extent, age)) or min(extent, age) < 0:
                raise ValueError('Invalid track')
            dx, dy = ox-vx, oy-vy
            norm = dx*dx+dy*dy
            closest = max(0., min(horizon, -(px*dx+py*dy)/norm)) if norm > 1e-12 else 0.
            envelope = radius+margin+extent+uncertainty+age*math.hypot(ox, oy)
            if math.hypot(px+dx*closest, py+dy*closest) > envelope:
                continue
            ids.append(t.get('id'))
            state = t['state'] if t.get('observed') else 'unknown'
            candidate = 'replan' if state == 'stationary' else 'wait'
            if action != 'wait':
                action = candidate
        except (KeyError, TypeError, ValueError, IndexError):
            return dict(action='wait', reason='invalid_track', ids=[])
    return dict(action=action, reason='predicted_obstacle' if ids else 'clear', ids=ids)
