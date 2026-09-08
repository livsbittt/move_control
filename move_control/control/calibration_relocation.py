"""Subject: one straight, observed-space calibration relocation attempt.

Targets are hypotheses, never rotation permission. Only the next 1 cm capsule
inside the current ordered scan polygon grants translation. Assumes calibrated
base-frame rays surround the base origin; full scan metadata, sensor freshness,
translation commissioning and independent hazard gates remain caller evidence.
Angular gaps add a conservative local chord margin; occluded polygon edges stay
obstacles instead of treating the empty space behind their endpoints as free.
"""
import math


def _point_segment(p,a,b):
    dx,dy=b[0]-a[0],b[1]-a[1]
    length2=dx*dx+dy*dy
    t=max(0.,min(1.,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/length2)) if length2 else 0.
    return math.hypot(p[0]-a[0]-t*dx,p[1]-a[1]-t*dy)


def _inside(point,polygon):
    x,y=point;inside=False
    for a,b in zip(polygon,polygon[1:]+polygon[:1]):
        if (a[1]>y)!=(b[1]>y) and x<(b[0]-a[0])*(y-a[1])/(b[1]-a[1])+a[0]:
            inside=not inside
    return inside


def _capsule_clear(polygon,distance,radius):
    a,b=(0.,0.),(distance,0.)
    if not _inside(a,polygon) or not _inside(b,polygon):return False
    for c,d in zip(polygon,polygon[1:]+polygon[:1]):
        # An edge crossing the horizontal center segment has zero clearance.
        if (c[1]<=0<=d[1] or d[1]<=0<=c[1]) and c[1]!=d[1]:
            crossing=c[0]+(d[0]-c[0])*(-c[1])/(d[1]-c[1])
            if min(0.,distance)<=crossing<=max(0.,distance):return False
        separation=min(_point_segment(a,c,d),_point_segment(b,c,d),
                       _point_segment(c,a,b),_point_segment(d,a,b))
        if separation<=radius:return False
    return True


class CalibrationRelocation:
    def __init__(self):
        self.started=self.start_pose=self.last_time=self.last_pose=None
        self.direction=0
        self.travel=0.
        self.max_travel=0.
        self.path_distance=0.
        self.target=None
        self.clear_since=None
        self.done=False
        self.error=None
        self.geometry=None
        self.bootstrap=False
        self.bootstrap_verified=False
        self.pilot_started=None
        self.pilot_commanded=0.
        self.last_speed=0.

    def _fail(self,reason):
        self.error=reason
        self.last_speed=0.
        return 0.,reason

    def update(self,now,pose,points,*,full_scan_observed,body_radius,rotation_radius,
               fresh_guard,translation_verified,rotation_clear_current,bootstrap_allowed=False):
        if self.error:return 0.,self.error
        if self.done:return 0.,'complete'
        try:
            pose=tuple(pose)
            polygon=[tuple(p) for p in points]
            if (len(pose)!=3 or not all(math.isfinite(v) for v in (now,*pose,body_radius,rotation_radius)) or
                    not 0<body_radius<=rotation_radius or not 24<=len(polygon)<=1440 or
                    not all(len(p)==2 and all(math.isfinite(v) for v in p) for p in polygon) or
                    full_scan_observed is not True or fresh_guard is not True or
                    type(translation_verified) is not bool or type(bootstrap_allowed) is not bool or
                    not (translation_verified or bootstrap_allowed) or
                    type(rotation_clear_current) is not bool):
                return self._fail('observation_unavailable')
            angles=[math.atan2(y,x) for x,y in polygon]
            gaps=[(b-a)%math.tau for a,b in zip(angles,angles[1:]+angles[:1])]
            # Accept either ordered scan winding, but never unordered points.
            reverse=[(-gap)%math.tau for gap in gaps]
            gap=min(max(gaps),max(reverse))
            if gap>math.radians(15):return self._fail('scan_coverage')
        except (TypeError,ValueError,OverflowError):
            return self._fail('observation_unavailable')
        if self.started is None:
            self.started=self.last_time=now
            self.start_pose=self.last_pose=pose
            self.geometry=(body_radius,rotation_radius)
            self.bootstrap=not translation_verified
        dt=now-self.last_time
        if dt<0 or dt>.5:return self._fail('clock_discontinuity')
        if self.bootstrap and not bootstrap_allowed:return self._fail('bootstrap_permission_lost')
        if not self.bootstrap and not translation_verified:return self._fail('translation_verification_lost')
        if now-self.started>=(75. if self.bootstrap else 45.):return self._fail('time_budget')
        if self.bootstrap and not self.bootstrap_verified:
            self.pilot_commanded+=abs(self.last_speed)*dt
        self.last_speed=0.
        if self.geometry!=(body_radius,rotation_radius):return self._fail('geometry_changed')
        if math.dist(pose[:2],self.last_pose[:2])>.02+.02*dt:return self._fail('odometry_jump')
        self.path_distance+=math.dist(pose[:2],self.last_pose[:2])
        if self.path_distance>.20:return self._fail('distance_budget')
        c,s=math.cos(self.start_pose[2]),math.sin(self.start_pose[2])
        dx,dy=pose[0]-self.start_pose[0],pose[1]-self.start_pose[1]
        forward,lateral=c*dx+s*dy,-s*dx+c*dy
        yaw=math.atan2(math.sin(pose[2]-self.start_pose[2]),math.cos(pose[2]-self.start_pose[2]))
        if abs(lateral)>.01 or abs(yaw)>.05:return self._fail('pose_drift')
        travel=forward*self.direction
        if self.direction and travel<self.max_travel-.002:return self._fail('reversed_motion')
        self.travel=travel
        self.max_travel=max(self.max_travel,travel)
        self.last_time,self.last_pose=now,pose
        if abs(forward)>.20:return self._fail('distance_budget')
        if self.bootstrap and self.direction and not self.bootstrap_verified:
            if travel>=.002:
                self.bootstrap_verified=True
            elif self.pilot_started is not None and now-self.pilot_started>=3.:
                return self._fail('bootstrap_no_progress')
        if rotation_clear_current:
            if self.clear_since is None:self.clear_since=now
            if now-self.clear_since>=.5-1e-12:
                self.done=True
                return 0.,'complete'
            return 0.,'confirming_rotation_clearance'
        self.clear_since=None
        margin=(body_radius+.02)*math.sin(gap/2)
        if self.bootstrap and self.pilot_started is None:
            # Before motor polarity is evidenced, either pilot direction must
            # fit the current observed free-space polygon with full stand-off.
            if not all(_capsule_clear(polygon,d*.004,body_radius+.01+margin) for d in (-1,1)):
                return self._fail('bootstrap_bilateral_clearance')
        if not self.direction:
            for step in range(1,21):
                options=[]
                for direction in (-1,1):
                    offset=direction*step*.01
                    clearance=min(math.hypot(x-offset,y) for x,y in polygon)
                    if (clearance>rotation_radius+.003 and
                            _capsule_clear(polygon,direction*.01,body_radius+.01+margin)):
                        options.append((clearance,direction))
                if options:
                    _,self.direction=max(options)
                    self.target=step*.01
                    break
            if not self.direction:return self._fail('no_observed_candidate')
        remaining=min(.20-self.travel,self.target-self.travel)
        if remaining<=.0001:return self._fail('target_not_clear')
        step=min(.01,remaining)
        if self.bootstrap and not self.bootstrap_verified:
            if self.pilot_started is None:self.pilot_started=now
            remaining=min(remaining,max(0.,.004-self.pilot_commanded))
            if remaining<=1e-6:return 0.,'bootstrap_waiting_for_progress'
            step=min(step,remaining)
        if not _capsule_clear(polygon,self.direction*step,body_radius+.01+margin):
            return self._fail('straight_capsule_blocked')
        self.last_speed=self.direction*min(.003 if self.bootstrap else .006,remaining/.5)
        return self.last_speed,'relocating'

    def report(self):
        return dict(start_pose=self.start_pose,direction=self.direction,travel_m=self.travel,
                    path_distance_m=self.path_distance,
                    target_estimate_m=self.target,done=self.done,error=self.error,
                    bootstrap=self.bootstrap,bootstrap_verified=self.bootstrap_verified,
                    pilot_commanded_m=self.pilot_commanded,
                    max_distance_m=.20,max_duration_s=75. if self.bootstrap else 45.,
                    max_speed_mps=.003 if self.bootstrap else .006)
