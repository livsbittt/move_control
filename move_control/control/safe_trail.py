"""Subject: measured odometry breadcrumbs back to a stable turn-safe refuge.

Vacuum-style retracing names a place actually visited with turning clearance,
instead of assuming a timed blind backup will create room. This memory grants
no motion authority: current rear/side sensing and safety gates remain required.
Use continuous odometry, never a SLAM pose that can jump during loop closure.
"""
import math


def _pose(value):
    x,y,yaw=value
    if not all(math.isfinite(v) for v in (x,y,yaw)):
        raise ValueError('invalid odometry')
    return (float(x),float(y),float(yaw))


def _distance(a,b):
    return math.hypot(a[0]-b[0],a[1]-b[1])


def _yaw_distance(a,b):
    return abs(math.atan2(math.sin(a[2]-b[2]),math.cos(a[2]-b[2])))


class SafeTrail:
    def __init__(self):
        self.samples=[]
        self.last_time=None
        self.last_pose=None
        self.geometry_id=None
        self.clear_since=None

    def _reset(self):
        self.samples=[]
        self.last_time=self.last_pose=self.clear_since=None
        self.geometry_id=None

    def observe(self, now, pose, valid, turn_clear, geometry_id):
        """Record actual odometry; discontinuities discard all old refuges."""
        try:
            pose=_pose(pose)
            if (not math.isfinite(now) or valid is not True or
                    type(turn_clear) is not bool or not isinstance(geometry_id,str) or not geometry_id):
                raise ValueError('invalid observation')
        except (TypeError,ValueError,OverflowError):
            self._reset()
            return False
        if self.last_time is not None:
            dt=now-self.last_time
            if (geometry_id != self.geometry_id or dt < 0 or dt > .5 or
                    _distance(pose,self.last_pose) > .05+.03*dt or
                    _yaw_distance(pose,self.last_pose) > .1+.3*dt):
                self._reset()
        self.geometry_id=geometry_id
        self.last_time,self.last_pose=now,pose
        if turn_clear:
            if self.clear_since is None:
                self.clear_since=now
        else:
            self.clear_since=None
        refuge=self.clear_since is not None and now-self.clear_since >= .5-1e-12
        previous=self.samples[-1]['pose'] if self.samples else None
        yaw_delta=_yaw_distance(pose,previous) if previous else 0.
        # A clearance observation certifies only its exact current pose; never
        # upgrade a nearby older breadcrumb that may have been turn-unsafe.
        first_refuge=refuge and self.samples and not self.samples[-1]['refuge']
        if previous is None or _distance(pose,previous) >= .01-1e-12 or yaw_delta >= .05 or first_refuge:
            self.samples.append({'pose':pose,'refuge':refuge})
        # Both an arc-length and a count cap bound storage even when rotating.
        while len(self.samples)>256:
            self.samples.pop(0)
        length=sum(_distance(a['pose'],b['pose']) for a,b in zip(self.samples,self.samples[1:]))
        while length>2.+1e-12 and len(self.samples)>1:
            length-=_distance(self.samples[0]['pose'],self.samples[1]['pose'])
            self.samples.pop(0)
        return True

    def retreat(self, now, pose, geometry_id, max_distance=.5):
        """Return newest-to-refuge odometry poses, or no fresh bounded route."""
        try:
            pose=_pose(pose)
            if (not math.isfinite(now) or not math.isfinite(max_distance) or max_distance <= 0 or
                    self.last_time is None or not 0 <= now-self.last_time <= .5 or
                    geometry_id != self.geometry_id or _distance(pose,self.last_pose) > .02 or
                    _yaw_distance(pose,self.last_pose) > .05 or
                    not self.samples):
                return None
        except (TypeError,ValueError,OverflowError):
            return None
        route=[]
        previous=pose
        length=0.
        for sample in reversed(self.samples):
            point=sample['pose']
            length+=_distance(previous,point)
            if length>max_distance:
                return None
            route.append(point)
            if sample['refuge'] and _distance(pose,point)>.005:
                return route
            previous=point
        return None
