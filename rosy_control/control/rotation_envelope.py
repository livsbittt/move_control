"""Subject: sensor-evidenced pivot envelope; never infer unseen chassis geometry."""
import math
import copy


RANGE_RESIDUAL_FLOOR_M = .0005
UNCERTAINTY_MODEL = 'empirical_residual_over_excitation_v1'


def pivot_clearance(points, center, pivot_radius):
    """Signed clearance using already validated lease primitives and base points.

    Unknown or malformed input is no clearance evidence. This only describes
    supplied obstacle returns; the caller retains scan coverage/freshness gates.
    """
    try:
        cx,cy = center
        if not all(math.isfinite(v) for v in (cx,cy,pivot_radius)) or pivot_radius <= 0:
            return None
        nearest = math.inf
        for x,y in points:
            if not math.isfinite(x) or not math.isfinite(y):
                return None
            nearest = min(nearest,math.hypot(x-cx,y-cy))
        return nearest-pivot_radius if math.isfinite(nearest) else None
    except (TypeError,ValueError,OverflowError):
        return None


def suggest_rotation_translation(points, center, pivot_radius, body_radius, max_distance=.03):
    """Suggest a short straight relocation; the caller owns actuation authority.

    Full scan freshness/coverage and independent hazards remain caller gates.
    The exact point-to-segment distance bounds the entire translating body
    circle, including its initial position, with the unchanged 10 mm stand-off.
    """
    try:
        cx,cy = center
        obstacles = [(float(x),float(y)) for x,y in points]
        if (not obstacles or not all(math.isfinite(v) for v in (cx,cy,pivot_radius,body_radius,max_distance)) or
                not all(math.isfinite(v) for p in obstacles for v in p) or
                pivot_radius <= 0 or body_radius <= 0 or not 0 < max_distance <= .03):
            return None
        current = pivot_clearance(obstacles,center,pivot_radius)
        if current is None or current > .010:
            return None
        for step in range(1, min(6,int((max_distance+1e-12)/.005))+1):
            candidates=[]
            for direction in (-1.,1.):
                distance=direction*step*.005
                low,high=min(0.,distance),max(0.,distance)
                # Capsule sweep: clamp each obstacle onto the entire x segment.
                path_clearance=min(math.hypot(x-min(high,max(low,x)),y) for x,y in obstacles)
                if path_clearance <= body_radius+.010:
                    continue
                margin=pivot_clearance(obstacles,(cx+distance,cy),pivot_radius)
                if margin is not None and margin > .013:
                    candidates.append((margin,distance))
            if candidates:
                return max(candidates)[1]
        return None
    except (TypeError,ValueError,OverflowError):
        return None


def straight_translation_limits(points, body_radius, max_distance=.03):
    """Forward/reverse clear travel for a circle plus 10 mm stand-off.

    Exact ray-circle entry distance bounds every intermediate position. Caller
    must supply a fresh full scan and recheck these limits while moving.
    """
    try:
        obstacles=[(float(x),float(y)) for x,y in points]
        if (not obstacles or not math.isfinite(body_radius) or body_radius <= 0 or
                not math.isfinite(max_distance) or not 0 < max_distance <= .03 or
                not all(math.isfinite(v) for p in obstacles for v in p)):
            return None
        radius=body_radius+.010
        limits=[float(max_distance),float(max_distance)]
        for x,y in obstacles:
            if math.hypot(x,y) <= radius:
                return (0.,0.)
            if abs(y) <= radius:
                entry=abs(x)-math.sqrt(max(0.,radius*radius-y*y))
                slot=int(x<0)
                limits[slot]=min(limits[slot],max(0.,entry-.0001))
        return tuple(limits)
    except (TypeError,ValueError,OverflowError):
        return None


def validate_envelope(report, radius_floor=0.):
    """Recompute a sensor-consistent estimate; this is not sensor authentication."""
    try:
        x, y = report['center_m']
        uncertainty = report['center_uncertainty_m']
        body, required = report['body_radius_m'], report['required_radius_m']
        pivot_radius = report['pivot_radius_m']
        counts = report['directional_counts']
        summary_valid = (report['valid'] is True and
                all(type(v) is int for v in (report['sample_count'], counts['positive'], counts['negative'])) and
                all(math.isfinite(v) for v in (x,y,uncertainty,body,required,pivot_radius,radius_floor)) and
                uncertainty >= 0 and body > 0 and body >= radius_floor and
                required >= body and pivot_radius > 0 and
                report['sample_count'] >= 4 and counts['positive'] >= 2 and counts['negative'] >= 2 and
                report['sample_count'] == counts['positive']+counts['negative'])
        trials = report['trials']
        if not summary_valid or not isinstance(trials, list) or not 4 <= len(trials) <= 24:
            return False
        recomputed = RotationEnvelope(body, report['footprint_xy'])
        for trial in trials:
            if not recomputed.add(trial['scan_delta'], trial['odom_delta'], trial['imu_yaw'], trial['residual_m']):
                return False
        expected = recomputed.report()
        if not expected['valid'] or report['uncertainty_model'] != UNCERTAINTY_MODEL:
            return False
        if report['sample_count'] != expected['sample_count'] or counts != expected['directional_counts']:
            return False
        return all(math.isclose(actual, target, rel_tol=1e-9, abs_tol=1e-9) for actual,target in
                   zip((x,y,uncertainty,required,pivot_radius), (*expected['center_m'], expected['center_uncertainty_m'], expected['required_radius_m'],expected['pivot_radius_m'])))
    except (KeyError, TypeError, ValueError, OverflowError):
        return False


class RotationEnvelope:
    """Collect independent settled trials (not adjacent correlated scan ticks).

    Each delta is expressed in its own trial's initial base frame. IMU yaw is
    a delta, not absolute heading. Radius is a trusted chassis circumradius.
    """
    def __init__(self, radius, footprint=()):
        if not math.isfinite(radius) or radius <= 0:
            raise ValueError('positive finite body radius required')
        self.radius = float(radius)
        self.footprint = [[float(x),float(y)] for x,y in footprint]
        self.footprint_padding = 0.
        if self.footprint:
            if not 3 <= len(self.footprint) <= 32 or not all(math.isfinite(v) for p in self.footprint for v in p):
                raise ValueError('finite trusted footprint with 3..32 vertices required')
            extent = max(math.hypot(x,y) for x,y in self.footprint)
            # URDF boxes project duplicate unordered vertices; only their hull
            # matters for maximum pivot distance, never input winding/order.
            ax,ay = self.footprint[0]
            bx,by = max(self.footprint,key=lambda p:math.hypot(p[0]-ax,p[1]-ay))
            area2 = max(abs((bx-ax)*(y-ay)-(by-ay)*(x-ax)) for x,y in self.footprint)
            if abs(extent-self.radius) > .001 or area2 <= 1e-8:
                raise ValueError('trusted footprint must agree with body circumradius')
            self.footprint_padding = max(0.,self.radius-extent)
        self.samples = []
        self.trials = []
        self.rejected = False
        self.reason = 'insufficient_bilateral_evidence'

    def add(self, scan_delta, odom_delta, imu_yaw, residual_m):
        if self.rejected:
            return False
        try:
            x,y,yaw = scan_delta
            ox,oy,oyaw = odom_delta
            if not all(math.isfinite(v) for v in (x,y,yaw,ox,oy,oyaw,imu_yaw,residual_m)):
                raise ValueError()
        except (TypeError, ValueError, OverflowError):
            return self._reject('nonfinite_evidence')
        if not math.radians(5) <= abs(yaw) <= math.radians(15) or not 0 <= residual_m <= .008:
            return self._reject('insufficient_excitation_or_poor_scan')
        if abs(yaw-oyaw) > .035 or abs(yaw-imu_yaw) > .035 or math.hypot(x-ox,y-oy) > .012:
            return self._reject('sensor_disagreement')
        a, b = 1-math.cos(yaw), math.sin(yaw)
        denom = a*a+b*b
        cx, cy = (a*x-b*y)/denom, (b*x+a*y)/denom
        # Empirical error assumption, not a statistical confidence guarantee:
        # ICP residual can understate range bias and correspondence ambiguity.
        # Preserve a 0.5 mm floor even on perfectly matching simulated scans.
        uncertainty = max(RANGE_RESIDUAL_FLOOR_M, residual_m) / (2*abs(math.sin(yaw/2)))
        if math.hypot(cx,cy) > .15 or uncertainty > .03:
            return self._reject('unbounded_pivot')
        self.samples.append((cx,cy,uncertainty,1 if yaw > 0 else -1))
        self.samples = self.samples[-24:]
        self.trials.append(dict(scan_delta=[x,y,yaw], odom_delta=[ox,oy,oyaw],
                                imu_yaw=imu_yaw, residual_m=residual_m))
        self.trials = self.trials[-24:]
        self.reason = 'insufficient_bilateral_evidence'
        return True

    def _reject(self, reason):
        self.rejected = True
        self.reason = reason
        return False

    def report(self):
        samples = self.samples
        counts = {'positive': sum(s[3]>0 for s in samples), 'negative': sum(s[3]<0 for s in samples)}
        cx = sum(s[0] for s in samples)/len(samples) if samples else 0.
        cy = sum(s[1] for s in samples)/len(samples) if samples else 0.
        spread = max((math.hypot(s[0]-cx,s[1]-cy) for s in samples), default=0.)
        uncertainty = max((math.hypot(s[0]-cx,s[1]-cy)+s[2] for s in samples), default=0.)
        valid = not self.rejected and min(counts.values()) >= 2 and spread <= .015
        reason = ('sensor_consistent_estimate' if valid else
                  self.reason if self.rejected else
                  'inconsistent_pivot' if spread > .015 else self.reason)
        pivot_radius = (max(math.hypot(x-cx,y-cy) for x,y in self.footprint)+self.footprint_padding
                        if self.footprint else self.radius+math.hypot(cx,cy)) + 2*uncertainty
        return dict(valid=valid, center_m=[cx,cy], center_uncertainty_m=uncertainty,
                    body_radius_m=self.radius, required_radius_m=max(self.radius,math.hypot(cx,cy)+pivot_radius),
                    footprint_xy=copy.deepcopy(self.footprint), pivot_radius_m=pivot_radius,
                    sample_count=len(samples), directional_counts=counts, reason=reason,
                    uncertainty_model=UNCERTAINTY_MODEL, trials=copy.deepcopy(self.trials))
