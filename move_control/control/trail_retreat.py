"""Bounded reverse tracking of an already verified odometry trail."""
import math


def _finite(values, count):
    try:
        return len(values)==count and all(math.isfinite(v) for v in values)
    except (TypeError, ValueError):
        return False


def _wrap(angle):
    return math.atan2(math.sin(angle),math.cos(angle))


def _rotation_key(rotation):
    try:
        center=rotation['center_m']
        uncertainty=rotation['center_uncertainty_m']
        if _finite(center,2) and math.isfinite(uncertainty) and 0 <= uncertainty <= .03:
            return (tuple(center),uncertainty)
    except (KeyError,TypeError,ValueError):
        pass
    return None


def _pivot(pose, center):
    x,y,yaw=pose
    cx,cy=center
    return (x+math.cos(yaw)*cx-math.sin(yaw)*cy,
            y+math.sin(yaw)*cx+math.cos(yaw)*cy)


def _segment_distance(point, a, b):
    dx,dy=b[0]-a[0],b[1]-a[1]
    length2=dx*dx+dy*dy
    t=max(0.,min(1.,((point[0]-a[0])*dx+(point[1]-a[1])*dy)/length2)) if length2 else 0.
    return math.hypot(point[0]-a[0]-t*dx,point[1]-a[1]-t*dy)


def _tracking_target(route, index, pose, lookahead=.03):
    """Look along the current straight leg without advancing route provenance."""
    target=route[index]
    remaining=lookahead-math.dist(pose[:2],target)
    direction=(target[0]-route[index-1][0],target[1]-route[index-1][1])
    for following in route[index+1:]:
        if remaining <= 0:
            break
        delta=(following[0]-target[0],following[1]-target[1])
        length=math.hypot(*delta)
        if length <= 1e-12:
            continue
        if math.hypot(*direction) <= 1e-12:
            direction=delta
        bend=math.atan2(direction[0]*delta[1]-direction[1]*delta[0],
                        direction[0]*delta[0]+direction[1]*delta[1])
        # Dense straight breadcrumbs are samples, not separate steering goals.
        # Keep real bends as targets; current-segment clearance still applies.
        if abs(bend) > .05:
            break
        if length >= remaining:
            return (target[0]+delta[0]*remaining/length,
                    target[1]+delta[1]*remaining/length)
        remaining-=length
        target=following
    return target


class TrailRetreat:
    """Caller owns freshness, rear hazard checks, route provenance and budget.

    Returned commands still require the sole safety gate. ``safe`` must cover
    current sensors and the intended command on every tick. With an unchanged
    measured pivot, a short monotonic pure-turn trail can return by yaw instead
    of treating its base-link orbit as a reverse translation path.
    """

    def __init__(self):
        self.reset()

    def reset(self):
        self.active=False
        self.route=[]
        self.index=1
        self.geometry_id=None
        self.started=None
        self.last_now=self.last_pose=None
        self.turn_target=self.rotation_key=self.pivot_anchor=None
        self.pivot_tolerance=None

    def start(self, now, pose, route, geometry_id, *, rotation=None):
        if self.active:
            return False  # A periodic replan cannot renew the deadline.
        try:
            valid=(math.isfinite(now) and _finite(pose,3) and bool(geometry_id)
                   and len(route)>=2 and all(_finite(p,2) or _finite(p,3) for p in route)
                   and math.dist(pose[:2],route[0][:2])<=.02
                   and 0 < sum(math.dist(a[:2],b[:2]) for a,b in zip(route,route[1:]))<=.5)
        except (TypeError,ValueError):
            valid=False
        if not valid:
            return False
        key=_rotation_key(rotation)
        if rotation is not None and key is None:
            return False
        self.turn_target=self.rotation_key=self.pivot_anchor=None
        if key is not None and all(_finite(p,3) for p in route):
            poses=[tuple(pose)]+[tuple(p) for p in route]
            changes=[_wrap(b[2]-a[2]) for a,b in zip(poses,poses[1:])]
            total=sum(abs(a) for a in changes)
            net=_wrap(poses[-1][2]-poses[0][2])
            anchor=_pivot(poses[0],key[0])
            tolerance=.001+2*key[1]*total
            # Do not merge translation, reversals or a larger turn into a
            # shortcut. Every recorded pose must share this measured pivot.
            if (.005 < total <= .3 and abs(total-abs(net)) <= 1e-6 and
                    all(math.dist(_pivot(p,key[0]),anchor)<=tolerance for p in poses)):
                self.turn_target=poses[-1]
                self.rotation_key=key
                self.pivot_anchor=anchor
                self.pivot_tolerance=tolerance
        self.route=[tuple(p[:2]) for p in route]
        self.started=now
        self.last_now,self.last_pose=now,tuple(pose)
        self.geometry_id=geometry_id
        self.index=1
        self.active=True
        return True

    def _stop(self, reason):
        self.active=False
        return 0.,0.,'trail_retreat_'+reason

    def update(self, now, pose, geometry_id, safe, turn_clear, *, rotation=None):
        if not self.active:
            return None,None,'inactive'
        if not safe:
            return self._stop('unsafe')
        if not _finite(pose,3):
            return self._stop('stale_pose')
        if geometry_id!=self.geometry_id:
            return self._stop('geometry_changed')
        if not math.isfinite(now) or now-self.started>=120.:
            return self._stop('timeout')
        dt=now-self.last_now
        if dt<0 or dt>.5:
            return self._stop('time_gap')
        if math.dist(pose[:2],self.last_pose[:2])>.05+.03*dt:
            return self._stop('pose_jump')
        yaw_delta=pose[2]-self.last_pose[2]
        if abs(math.atan2(math.sin(yaw_delta),math.cos(yaw_delta)))>.1+.3*dt:
            return self._stop('yaw_jump')
        self.last_now,self.last_pose=now,tuple(pose)
        if self.turn_target is not None:
            if _rotation_key(rotation)!=self.rotation_key:
                return self._stop('estimate_changed')
            if math.dist(_pivot(pose,self.rotation_key[0]),self.pivot_anchor)>self.pivot_tolerance:
                return self._stop('pivot_drift')
            error=_wrap(self.turn_target[2]-pose[2])
            if abs(error) > .3:
                return self._stop('heading')
            if abs(error) <= .005:
                if math.dist(pose[:2],self.turn_target[:2]) > .003:
                    return self._stop('position_mismatch')
                if turn_clear:
                    return self._stop('complete')
                return 0.,0.,'trail_retreat_endpoint_hold'
            return 0.,max(-.04,min(.04,1.4*error)),'trail_retreat_turn'
        # Only the current segment grants tracking clearance; a crossing or
        # nearby later segment cannot excuse leaving the verified sequence.
        if _segment_distance(pose[:2],self.route[self.index-1],self.route[self.index])>.02:
            return self._stop('off_route')
        while self.index<len(self.route)-1 and math.dist(pose[:2],self.route[self.index])<=.003:
            self.index+=1
        target=self.route[self.index]
        if self.index==len(self.route)-1 and math.dist(pose[:2],target)<=.005:
            if turn_clear:
                return self._stop('complete')
            return 0.,0.,'trail_retreat_endpoint_hold'
        target=_tracking_target(self.route,self.index,pose)
        desired=math.atan2(target[1]-pose[1],target[0]-pose[0])+math.pi
        error=math.atan2(math.sin(desired-pose[2]),math.cos(desired-pose[2]))
        if abs(error)>.3:
            return self._stop('heading')
        if abs(error)<1e-12:
            error=0.
        return -.006*math.cos(error),max(-.04,min(.04,1.4*error)),'trail_retreat'
