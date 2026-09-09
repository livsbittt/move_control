"""Fail-closed map-route following for the slow desk robot."""
import math

from .pursuit import pursuit_index, pursuit_speed

GOAL_TOLERANCE_M = .025

def follow_path(route, pose, *, route_age, tf_age, blocked=False,
                max_age=5.0, max_tf_age=1.0, speed=.014, turn=.10,
                lookahead=.06, tolerance=GOAL_TOLERANCE_M, max_deviation=.08, _aim_index=None, _aligning=False):
    """Return semantic (linear, angular, reason); no odometry fallback."""
    if blocked:
        return 0.0, 0.0, 'hazard'
    if not route:
        return 0.0, 0.0, 'no_route'
    if not math.isfinite(route_age) or not 0 <= route_age <= max_age:
        return 0.0, 0.0, 'stale_route'
    if pose is None or not math.isfinite(tf_age) or not 0 <= tf_age <= max_tf_age:
        return 0.0, 0.0, 'no_map_tf'
    if not all(math.isfinite(v) for p in route for v in p):
        return 0.0, 0.0, 'invalid_route'
    if not all(math.isfinite(v) for v in pose):
        return 0.0, 0.0, 'no_map_tf'
    x, y, yaw = pose
    deviation = math.hypot(route[0][0] - x, route[0][1] - y)
    for a, b in zip(route, route[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        length2 = dx * dx + dy * dy
        fraction = 0.0 if length2 == 0 else max(0.0, min(
            1.0, ((x - a[0]) * dx + (y - a[1]) * dy) / length2))
        deviation = min(deviation, math.hypot(
            x - a[0] - fraction * dx, y - a[1] - fraction * dy))
    if deviation > max_deviation:
        return 0.0, 0.0, 'off_route'
    if math.hypot(route[-1][0] - x, route[-1][1] - y) <= tolerance:
        return 0.0, 0.0, 'arrived'
    aim = route[pursuit_index(route, x, y, lookahead) if _aim_index is None else _aim_index]
    distance = math.hypot(aim[0] - x, aim[1] - y)
    error = math.atan2(aim[1] - y, aim[0] - x) - yaw
    error = math.atan2(math.sin(error), math.cos(error))
    linear = pursuit_speed(max(0.0, speed), distance, error)
    # Keep the existing 0.3-rad stop boundary; resume further inside it so
    # translation toward a nearby corner cannot toggle the mode every tick.
    if _aligning and abs(error) > .2:
        linear = 0.
    angular = max(-abs(turn), min(abs(turn), 1.4 * error))
    return linear, angular, 'forward' if linear > 0 else 'align'


class PathFollower:
    """Retain corner progression while the base moves around an offset pivot.

    Route timestamps carry no progress authority. Only a matching remaining
    geometric suffix permits a periodically refreshed start prefix to retain it.
    """
    def __init__(self):
        self.reset()

    def reset(self):
        self.route=()
        self.cursor=None
        self.aligning=False

    def update(self,route,pose,**kwargs):
        initial=follow_path(route,pose,**kwargs)
        if initial[2] not in ('align','forward'):
            self.reset()
            return initial
        retained=None
        if self.cursor is not None:
            suffix=self.route[self.cursor:]
            for index in range(len(route)):
                remaining=route[index:]
                # Only the first point may refresh arbitrarily to current pose.
                # A new intermediate detour is a genuinely different route.
                prefix=route[1:index]
                old_prefix=self.route[:self.cursor]
                prefix_matches=(not prefix or len(prefix)<=len(old_prefix) and
                    all(math.dist(a,b)<=.005 for a,b in zip(prefix,old_prefix[-len(prefix):])))
                if prefix_matches and len(remaining)==len(suffix) and all(math.dist(a,b)<=.005 for a,b in zip(remaining,suffix)):
                    retained=index
                    break
        selected=pursuit_index(route,pose[0],pose[1],kwargs.get('lookahead',.06))
        if retained is not None:
            selected=max(selected,retained)
        else:
            # A new target geometry has no alignment history to inherit.
            self.aligning=False
        # A generated current-pose prefix can leave a tiny sharp-corner stub.
        # Reaching that vertex already satisfies the same 5mm corner tolerance.
        reached=False
        while selected<len(route)-1 and math.dist(route[selected],pose[:2])<=.005:
            selected+=1
            reached=True
        if reached:
            # A single next grid vertex can lie inside the offset pivot's
            # orbit. Rebuild the outgoing lookahead, still stopping at its
            # next real corner, rather than chasing that unreachable bearing.
            outgoing=selected-1
            selected=max(selected,outgoing+pursuit_index(route[outgoing:],pose[0],pose[1],kwargs.get('lookahead',.06)))
        self.route=tuple(tuple(p) for p in route)
        self.cursor=selected
        result=follow_path(route,pose,_aim_index=selected,_aligning=self.aligning,**kwargs)
        self.aligning=result[2]=='align'
        return result


class ProgressGuard:
    """Latch a stop after commanded motion produces no measured progress."""
    def __init__(self, timeout=8.0):
        self.timeout = timeout
        self.reset()

    def reset(self):
        self.anchor = None
        self.since = None
        self.stalled = False
        self.paused_since = None

    def pause(self, now, paused):
        if paused and self.paused_since is None:
            self.paused_since = now
        elif not paused and self.paused_since is not None:
            if self.since is not None:
                self.since += max(0., now-self.paused_since)
            self.paused_since = None

    def check(self, now, pose, moving):
        if self.stalled:
            return True
        if not moving or pose is None:
            # Empty replans / obstacle holds must not replenish the push budget.
            # Only measured progress or an explicit new command resets it.
            return False
        if self.anchor is None:
            self.anchor, self.since = pose, now
            return False
        distance = math.hypot(pose[0] - self.anchor[0], pose[1] - self.anchor[1])
        angle = pose[2] - self.anchor[2]
        angle = abs(math.atan2(math.sin(angle), math.cos(angle)))
        # Judge net displacement over a whole window. Resetting on every
        # tiny excursion lets a repeated back-and-forth motion live forever.
        if now - self.since >= self.timeout:
            if distance >= .005 or angle >= .05:
                self.anchor, self.since = pose, now
            else:
                self.stalled = True
        return self.stalled
