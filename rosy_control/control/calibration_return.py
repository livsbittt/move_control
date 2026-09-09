"""Optional straight return to the recorded pre-calibration odometry origin."""
import math
from .calibration_relocation import _capsule_clear


class CalibrationReturn:
    def __init__(self, originpose):
        self.origin=tuple(originpose)
        if len(self.origin)!=3 or not all(math.isfinite(v) for v in self.origin):
            raise ValueError('Finite original odometry pose required')
        self.started=self.last_time=self.last_pose=self.start_pose=None
        self.radius=None
        self.path_distance=self.travel=0.
        self.clear_since=None
        self.done=False
        self.error=None

    def _fail(self, reason):
        self.error=reason
        return 0.,reason

    def update(self, now, pose, points, *, body_radius, full_scan_observed,
               fresh_guard, rotation_verified):
        """Return linear speed only; caller must set angular velocity to zero.

        Current ordered scan and fresh independent hazards grant each capsule.
        A previously traveled path never grants authority through a new obstacle.
        """
        if self.error:return 0.,self.error
        if self.done:return 0.,'complete'
        try:
            pose=tuple(pose);polygon=[tuple(p) for p in points]
            if (len(pose)!=3 or not all(math.isfinite(v) for v in (now,*pose,body_radius))
                    or not 0<body_radius<=.3 or not 24<=len(polygon)<=1440
                    or not all(len(p)==2 and all(math.isfinite(v) for v in p) for p in polygon)
                    or full_scan_observed is not True or fresh_guard is not True
                    or rotation_verified is not True):
                return self._fail('observation_unavailable')
            angles=[math.atan2(y,x) for x,y in polygon]
            gaps=[(b-a)%math.tau for a,b in zip(angles,angles[1:]+angles[:1])]
            gap=min(max(gaps),max((-g)%math.tau for g in gaps))
            if gap>math.radians(15):return self._fail('scan_coverage')
        except (TypeError,ValueError,OverflowError):
            return self._fail('observation_unavailable')
        if self.started is None:
            if math.dist(pose[:2],self.origin[:2])>.22:return self._fail('start_distance')
            self.started=self.last_time=now
            self.start_pose=self.last_pose=pose
            self.radius=body_radius
        dt=now-self.last_time
        if dt<0 or dt>.5:return self._fail('clock_discontinuity')
        if now-self.started>=60.:return self._fail('time_budget')
        if self.radius!=body_radius:return self._fail('geometry_changed')
        distance=math.dist(pose[:2],self.last_pose[:2])
        if distance>.02+.02*dt:return self._fail('odometry_jump')
        yaw_delta=pose[2]-self.last_pose[2]
        if abs(math.atan2(math.sin(yaw_delta),math.cos(yaw_delta)))>.05:
            return self._fail('odometry_jump')
        self.path_distance+=distance
        if self.path_distance>.23:return self._fail('distance_budget')
        self.travel=math.dist(pose[:2],self.start_pose[:2])
        self.last_time,self.last_pose=now,pose
        c,s=math.cos(self.origin[2]),math.sin(self.origin[2])
        dx,dy=pose[0]-self.origin[0],pose[1]-self.origin[1]
        forward,lateral=c*dx+s*dy,-s*dx+c*dy
        yaw=math.atan2(math.sin(pose[2]-self.origin[2]),math.cos(pose[2]-self.origin[2]))
        if abs(lateral)>.01 or abs(yaw)>.05:return self._fail('pose_drift')
        if math.hypot(dx,dy)<=.005:
            if self.clear_since is None:self.clear_since=now
            if now-self.clear_since>=.5-1e-12:
                self.done=True
                return 0.,'complete'
            return 0.,'confirming_origin'
        self.clear_since=None
        # A straight controller cannot remove a lateral-only origin error.
        if abs(forward)<=.001:return self._fail('origin_not_on_straight_track')
        remaining=abs(forward)
        direction=-1 if forward>0 else 1
        beam_margin=(body_radius+.02)*math.sin(gap/2)
        if not _capsule_clear(polygon,direction*min(.01,remaining),body_radius+.01+beam_margin):
            return self._fail('straight_capsule_blocked')
        return direction*min(.006,remaining/.5),'returning'

    def report(self):
        return dict(origin_pose=self.origin,start_pose=self.start_pose,done=self.done,error=self.error,
                    travel_m=self.travel,path_distance_m=self.path_distance,max_start_distance_m=.22,
                    max_path_distance_m=.23,max_duration_s=60.,max_speed_mps=.006)
