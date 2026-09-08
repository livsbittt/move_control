"""A fixed-budget, fixed-heading escape executor; no ROS or motor authority."""
import math


class StraightEscape:
    def __init__(self):
        self.identity=None
        self.started=None
        self.origin=None
        self.failed=False
        self.previous=None
        self.distance=0.
        self.direction=None

    def update(self, now, pose, intent, safe, odom=None):
        if intent is None:
            return None,'inactive'
        try:
            identity=intent['id']
            target=intent['target']
            yaw=intent['yaw']
            odom=pose if odom is None else odom
            if (not safe or not isinstance(identity,str) or not identity or
                    len(pose)!=3 or len(odom)!=3 or len(target)!=2 or
                    not all(math.isfinite(v) for v in (now,*pose,*odom,*target,yaw))):
                raise ValueError()
            if self.identity is None:
                self.identity=identity
                self.started=now
                self.origin=tuple(pose)
                self.direction=1 if math.cos(yaw)*(target[0]-pose[0])+math.sin(yaw)*(target[1]-pose[1])>0 else -1
            if identity!=self.identity or self.failed or not 0<=now-self.started<30.:
                raise ValueError()
            if self.previous is not None:
                last,old_pose,old_odom=self.previous
                dt=now-last
                step=math.dist(odom[:2],old_odom[:2])
                if dt<0 or dt>.5 or step>.005+.02*dt or abs(math.dist(pose[:2],old_pose[:2])-step)>.005:
                    raise ValueError()
                self.distance+=step
                if self.distance>.12:raise ValueError()
            self.previous=(now,tuple(pose),tuple(odom))
            error=math.atan2(math.sin(pose[2]-yaw),math.cos(pose[2]-yaw))
            dx,dy=target[0]-pose[0],target[1]-pose[1]
            along=math.cos(yaw)*dx+math.sin(yaw)*dy
            lateral=-math.sin(yaw)*dx+math.cos(yaw)*dy
            if abs(error)>.02 or abs(lateral)>.003:
                raise ValueError()
            if math.hypot(dx,dy)<=.003:
                return 0.,'complete'
            if not 0<along*self.direction<=.121:raise ValueError()
            return math.copysign(min(.006,abs(along)),along),'straight_escape'
        except (KeyError,TypeError,ValueError,OverflowError):
            self.failed=True
            return 0.,'straight_escape_stopped'
