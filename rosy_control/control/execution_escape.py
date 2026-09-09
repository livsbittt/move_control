"""Execute one live, measured escape proposal within the existing mission."""
import math


def _number(value):
    return type(value) in (int,float) and math.isfinite(value)


class ExecutionEscape:
    SPEED = .005
    MAX_DISTANCE = .08
    MAX_SECONDS = 20.

    def __init__(self):
        self.used = self.active = False
        self.wait_since = self.origin = self.started = self.geometry = None
        self.direction = self.pending_direction = 0
        self.episode_open = False
        self.saw_restored = False
        self.restored_pose = None
        self.hold_pose = self.hold_clearance = None
        self.hold_reason = None
        self.progress_at = self.best_progress = None
        self.decision = dict(reason='waiting_for_candidate',active=False)

    def snapshot(self):
        return dict(self.decision)

    @staticmethod
    def proposal_valid(now, proposal, geometry):
        return bool(isinstance(proposal,dict) and geometry and
            proposal.get('geometry_revision') == geometry and
            _number(now) and _number(proposal.get('issued_s')) and
            0 <= now-proposal['issued_s'] <= .25 and
            _number(proposal.get('scan_age_s')) and 0 <= proposal['scan_age_s'] <= .2 and
            type(proposal.get('rotation_restored')) is bool)

    def select(self, now, proposal, geometry, remaining_s):
        if not self.proposal_valid(now,proposal,geometry) or not _number(remaining_s):
            return None
        candidates=proposal.get('candidates')
        if not isinstance(candidates,list) or len(candidates)>2:
            return None
        valid=[]
        for c in candidates:
            if not isinstance(c,dict):
                continue
            direction=c.get('direction')
            if type(direction) is not int or direction not in (-1,1):
                continue
            if (self.active or self.episode_open) and direction!=self.direction:
                continue
            if not all(_number(c.get(k)) for k in (
                    'target_m','available_m','predicted_rotation_clearance_m')):
                continue
            if not 0 < c['target_m'] <= min(self.MAX_DISTANCE,c['available_m']):
                continue
            if c['target_m']/self.SPEED+2 > remaining_s:
                continue
            valid.append(c)
        restoring=[c for c in valid if c.get('objective')=='restore_rotation']
        if restoring:
            return min(restoring,key=lambda c:(c['target_m'],-c['predicted_rotation_clearance_m']))
        return max(valid,key=lambda c:(c['predicted_rotation_clearance_m'],-c['target_m'])) if valid else None

    def update(self, now, pose, *, safe, proposal, geometry, remaining_s, waiting, time_bounded=False):
        if time_bounded:
            return self._bounded_update(now,pose,safe,proposal,geometry,remaining_s,waiting)
        return self._legacy_update(now,pose,safe=safe,proposal=proposal,geometry=geometry,
                                   remaining_s=remaining_s,waiting=waiting)

    def _bounded_update(self,now,pose,safe,proposal,geometry,remaining_s,waiting):
        was_active=self.active
        finite=pose is not None and len(pose)==3 and all(_number(v) for v in (*pose,now))
        clearance=proposal.get('current_rotation_clearance_m') if isinstance(proposal,dict) else None
        issued=proposal.get('issued_s') if isinstance(proposal,dict) else None
        age=now-issued if _number(now) and _number(issued) else None
        scan_age=proposal.get('scan_age_s') if isinstance(proposal,dict) else None
        diagnostic_remaining=(max(0.,min(remaining_s,self.MAX_SECONDS-(now-self.started)))
            if _number(now) and _number(remaining_s) and self.started is not None else
            max(0.,min(remaining_s,self.MAX_SECONDS)) if _number(remaining_s) else None)
        def stop(reason,hold=False,complete=False):
            self.active=False
            if hold and finite and self.hold_pose is None:
                self.hold_pose=tuple(pose)
                self.hold_clearance=clearance if _number(clearance) else None
                self.hold_reason=reason
            self.decision=dict(reason=reason,active=False,direction=self.direction,
                episode_elapsed_s=max(0.,now-self.started) if _number(now) and self.started is not None else 0.,
                remaining_s=diagnostic_remaining if _number(diagnostic_remaining) else None,
                proposal_age_s=age,scan_age_s=scan_age if _number(scan_age) else None)
            return ((0.,'execution_escape_complete' if complete else 'execution_escape_stopped')
                    if was_active else (None,''))
        if not _number(remaining_s) or remaining_s<=0:
            return stop('budget',hold=True)
        if not finite or not safe:
            return stop('safety_hold')
        if not waiting:
            return stop('waiting_for_route')
        if not self.proposal_valid(now,proposal,geometry):
            return stop('waiting_for_candidate')
        if self.episode_open and self.saw_restored and self.restored_pose is not None:
            yaw=math.atan2(math.sin(pose[2]-self.restored_pose[2]),math.cos(pose[2]-self.restored_pose[2]))
            if math.dist(pose[:2],self.restored_pose[:2])>=.005 or abs(yaw)>=.05:
                # A real navigation turn/translation after restored clearance
                # creates a new measured pose, unlike a toggling sensor frame.
                self.episode_open=self.active=False
                self.origin=self.started=None
                self.hold_pose=self.hold_clearance=self.hold_reason=None
                self.saw_restored=False
                self.restored_pose=None
        if self.hold_pose is not None:
            moved=math.dist(pose[:2],self.hold_pose[:2])>=.01
            changed=(_number(clearance) and self.hold_clearance is not None and
                     clearance-self.hold_clearance>=.005)
            if not moved and not changed:
                return stop(self.hold_reason or 'no_progress')
            self.hold_pose=self.hold_clearance=None
            self.hold_reason=None
            self.episode_open=False
            self.started=self.origin=None
        if proposal['rotation_restored']:
            if self.episode_open and not self.saw_restored:
                self.restored_pose=tuple(pose)
            self.saw_restored=self.episode_open
            return stop('rotation_restored',complete=True)
        if self.episode_open:
            dx,dy=pose[0]-self.origin[0],pose[1]-self.origin[1]
            forward=dx*math.cos(self.origin[2])+dy*math.sin(self.origin[2])
            lateral=-dx*math.sin(self.origin[2])+dy*math.cos(self.origin[2])
            yaw=math.atan2(math.sin(pose[2]-self.origin[2]),math.cos(pose[2]-self.origin[2]))
            progress=forward*self.direction
            if geometry!=self.geometry or abs(yaw)>.035 or abs(lateral)>.003 or progress<-.003:
                return stop('pose_or_geometry_changed',hold=True)
            if now<self.started or now-self.started>=self.MAX_SECONDS or progress>=self.MAX_DISTANCE:
                return stop('budget',hold=True)
            if progress-self.best_progress>=.001:
                self.best_progress=progress
                self.progress_at=now
            if now-self.progress_at>=8:
                return stop('no_progress',hold=True)
        else:
            progress=0.
        candidate=self.select(now,proposal,geometry,remaining_s)
        if candidate is None:
            return stop('waiting_for_candidate')
        if self.episode_open and (progress+candidate['target_m']>self.MAX_DISTANCE or
                now-self.started+candidate['target_m']/self.SPEED>self.MAX_SECONDS):
            return stop('budget',hold=True)
        if not self.episode_open:
            self.episode_open=True
            self.origin=tuple(pose)
            self.started=self.progress_at=now
            self.best_progress=0.
            self.geometry=geometry
            self.direction=candidate['direction']
        self.active=True
        self.decision=dict(reason='motion',active=True,direction=self.direction,
            target_remaining_m=candidate['target_m'],available_m=candidate['available_m'],
            episode_elapsed_s=now-self.started,
            remaining_s=max(0.,min(remaining_s,self.MAX_SECONDS-(now-self.started))),
            proposal_age_s=age,scan_age_s=scan_age if _number(scan_age) else None)
        return self.SPEED*self.direction,'execution_escape_forward' if self.direction>0 else 'execution_escape_reverse'

    def _legacy_update(self, now, pose, *, safe, proposal, geometry, remaining_s, waiting):
        finite = pose is not None and len(pose)==3 and all(_number(v) for v in (*pose,now))
        allowed = bool(safe and finite and waiting and self.proposal_valid(now,proposal,geometry))
        candidate=self.select(now,proposal,geometry,remaining_s)
        if self.active:
            forward = lateral = yaw = 0.
            if finite:
                dx,dy=pose[0]-self.origin[0],pose[1]-self.origin[1]
                forward=dx*math.cos(self.origin[2])+dy*math.sin(self.origin[2])
                lateral=-dx*math.sin(self.origin[2])+dy*math.cos(self.origin[2])
                yaw=math.atan2(math.sin(pose[2]-self.origin[2]),math.cos(pose[2]-self.origin[2]))
            progress=forward*self.direction
            if (not allowed or geometry!=self.geometry or not _number(remaining_s) or remaining_s<=0 or
                    now<self.started or now-self.started>=self.MAX_SECONDS or
                    abs(lateral)>.003 or abs(yaw)>.035 or progress<-.003 or progress>self.MAX_DISTANCE):
                self.active=False
                return 0.,'execution_escape_stopped'
            if proposal['rotation_restored']:
                self.active=False
                return 0.,'execution_escape_complete'
            if (candidate is None or progress+candidate['target_m']>self.MAX_DISTANCE or
                    now-self.started+candidate['target_m']/self.SPEED>self.MAX_SECONDS):
                self.active=False
                return 0.,'execution_escape_stopped'
            return self.SPEED*self.direction,'execution_escape_forward' if self.direction>0 else 'execution_escape_reverse'
        if self.used:
            return None,''
        if not allowed or candidate is None or proposal['rotation_restored']:
            self.wait_since=None
            return None,''
        direction=candidate['direction']
        if self.wait_since is None or now<self.wait_since or direction!=self.pending_direction:
            self.wait_since=now
            self.pending_direction=direction
        if now-self.wait_since<5:
            return None,''
        self.used=self.active=True
        self.origin=tuple(pose)
        self.started=now
        self.geometry=geometry
        self.direction=direction
        return self.SPEED*direction,'execution_escape_forward' if direction>0 else 'execution_escape_reverse'
