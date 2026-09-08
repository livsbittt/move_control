"""Fixed-heading translation back into the ordinary planner's free space."""
import math
import numpy as np
from ..control.footprint_sweep import _hull, _clearance


def straight_escape(m, pose, footprint, margin, hard_clearance, target=None):
    """Advisory forward-only escape; full occupied/unknown cells block it."""
    try:
        x,y,yaw=pose
        if not all(math.isfinite(v) for v in (*pose,margin,hard_clearance)) or margin<.01:
            return None
        shape=np.asarray(footprint,dtype=float)
        if shape.ndim!=2 or shape.shape[1]!=2 or not np.isfinite(shape).all():return None
        c,s=math.cos(yaw),math.sin(yaw)
        hull=_hull(shape)@np.array([[c,s],[-s,c]])+[x,y]
        values=np.asarray(m.data).reshape(m.h,m.w)
        iy,ix=np.where((values<0)|(values>=65))
        obstacles=np.column_stack((ix+.5,iy+.5))*m.res+[m.ox,m.oy]
        # The circumscribed cell disk includes corners and entire cell edges.
        padded=margin+m.res/math.sqrt(2)
        grid=m.inflate(hard_clearance/m.res)
        candidates=[target] if target is not None else [
            (x+c*d,y+s*d) for d in np.arange(.02,.1201,.01)]
        for end in candidates:
            dx,dy=end[0]-x,end[1]-y
            distance=c*dx+s*dy
            if not .001<=distance<=.121 or abs(-s*dx+c*dy)>.003:continue
            if not grid.is_free(*grid.world_to_grid(*end)):continue
            if not all(grid.is_free(*grid.world_to_grid(end[0]+a,end[1]+b))
                       for a in (-.004,.004) for b in (-.004,.004)):continue
            swept=_hull(np.vstack((hull,hull+[dx,dy])))
            lo,hi=swept.min(axis=0)-padded,swept.max(axis=0)+padded
            if lo[0]<m.ox or lo[1]<m.oy or hi[0]>m.ox+m.w*m.res or hi[1]>m.oy+m.h*m.res:continue
            if len(obstacles) and _clearance(obstacles,swept)<=padded:continue
            return tuple(map(float,end))
    except (TypeError,ValueError,OverflowError):
        pass
    return None
