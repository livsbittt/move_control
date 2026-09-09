"""Subject: bounded independent lidar registration in the robot base frame."""
import math

import numpy as np

from .lidar import wrap_pi


def scan_points(ranges, angle_min, increment, mount=(0., 0., 0.), max_points=180):
    """Apply the caller's calibrated lidar-to-base transform, including nose yaw."""
    values = np.asarray(ranges, dtype=float)
    angles = angle_min + np.arange(len(values)) * increment
    keep = np.isfinite(values) & (values >= .05) & (values <= 8.)
    values, angles = values[keep], angles[keep]
    if len(values) > max_points:
        indices = np.linspace(0, len(values)-1, max_points).astype(int)
        values, angles = values[indices], angles[indices]
    angles = np.array([wrap_pi(float(a) + mount[2]) for a in angles])
    return np.column_stack((values*np.cos(angles)+mount[0],
                            values*np.sin(angles)+mount[1]))


def _rotation(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])


def _fit(ref, cur, hint):
    rotation, translation = _rotation(hint), np.zeros(2)
    for _ in range(18):
        aligned = cur @ rotation.T + translation
        distances = np.sum((aligned[:, None, :] - ref[None, :, :])**2, axis=2)
        closest = np.argmin(distances, axis=1)
        errors = np.sqrt(distances[np.arange(len(cur)), closest])
        keep = errors <= min(.04, max(.004, float(np.quantile(errors, .8))))
        if keep.sum() < max(20, int(.65*len(cur))):
            return None
        src, dst = cur[keep], ref[closest[keep]]
        u, _, vt = np.linalg.svd((src-src.mean(0)).T @ (dst-dst.mean(0)))
        correction = np.eye(2)
        correction[1, 1] = np.linalg.det(vt.T @ u.T)
        next_rotation = vt.T @ correction @ u.T
        next_translation = dst.mean(0) - next_rotation @ src.mean(0)
        converged = (np.linalg.norm(next_rotation-rotation) < 1e-7 and
                     np.linalg.norm(next_translation-translation) < 1e-7)
        rotation, translation = next_rotation, next_translation
        if converged:
            break
    aligned = cur @ rotation.T + translation
    errors = np.sqrt(np.min(np.sum((aligned[:, None, :] - ref[None, :, :])**2, axis=2), axis=1))
    return rotation, translation, float(np.sqrt(np.mean(np.sort(errors)[:int(.8*len(errors))]**2)))


def _segments(ref):
    edges = np.diff(ref,axis=0)
    lengths = np.linalg.norm(edges,axis=1)
    directions = edges/np.maximum(lengths[:,None],1e-12)
    aligned = np.sum(directions[1:]*directions[:-1],axis=1) > .995
    supported = np.r_[False,aligned] | np.r_[aligned,False]
    # Quantized millimetre returns make individual short-edge angles noisy.
    # A contiguous four-return line fit can independently support those edges
    # while retaining the existing gap limit and rejecting non-collinear corners.
    for start in range(len(ref)-3):
        if np.any(lengths[start:start+3] >= .08):
            continue
        window=ref[start:start+4]
        centered=window-window.mean(axis=0)
        _,_,axes=np.linalg.svd(centered,full_matrices=False)
        extent=np.ptp(centered@axes[0])
        if extent >= .01 and np.max(np.abs(centered@axes[1])) <= .001:
            supported[start:start+3]=True
    keep = (lengths > .0001) & (lengths < .08) & supported
    if keep.sum() < max(20,.4*len(ref)):
        return None  # Unordered point clouds retain point-to-point registration.
    return ref[:-1][keep],edges[keep],lengths[keep]**2


def _fit_segments(segments,cur,hint,initial=None):
    starts,edges,length2 = segments
    normals = np.column_stack((-edges[:,1],edges[:,0]))/np.sqrt(length2[:,None])
    rotation,translation = _rotation(hint),np.zeros(2)
    if initial is not None:
        rotation,translation=initial[0].copy(),initial[1].copy()
    def correspondence(aligned):
        offsets=aligned[:,None,:]-starts[None,:,:]
        fractions=np.sum(offsets*edges[None,:,:],axis=2)/length2
        projections=starts[None,:,:]+np.clip(fractions,0.,1.)[:,:,None]*edges[None,:,:]
        distance2=np.sum((aligned[:,None,:]-projections)**2,axis=2)
        closest=np.argmin(distance2,axis=1)
        errors=np.sqrt(distance2[np.arange(len(cur)),closest])
        # Dot/divide roundoff can place an exact endpoint at 1+2e-16.
        # This admits at most 8e-14 m beyond an <=8 cm supported segment,
        # not extrapolation across an unobserved physical gap.
        interior=(fractions[np.arange(len(cur)),closest]>=-1e-12)&(fractions[np.arange(len(cur)),closest]<=1.+1e-12)
        keep=interior&(errors<=min(.04,max(.002,float(np.quantile(errors,.8)))))
        return closest,errors,keep
    for _ in range(14):
        aligned=cur@rotation.T+translation
        closest,errors,keep=correspondence(aligned)
        if keep.sum()<max(20,.6*len(cur)):
            return None
        p=aligned[keep]
        n=normals[closest[keep]]
        residual=np.sum(n*(p-starts[closest[keep]]),axis=1)
        jacobian=np.column_stack((n,np.sum(n*np.column_stack((-p[:,1],p[:,0])),axis=1)))
        # Parallel walls and circular scans lack a fully observable rigid pose.
        scale=max(.05,float(np.sqrt(np.mean(np.sum(p*p,axis=1)))))
        singular=np.linalg.svd(jacobian/np.array([1.,1.,scale]),compute_uv=False)
        if singular[-1]<.025*singular[0]:
            return None
        update=np.linalg.lstsq(jacobian,-residual,rcond=None)[0]
        if np.linalg.norm(update[:2])>.06 or abs(update[2])>.2:
            return None
        incremental=_rotation(float(update[2]))
        rotation=incremental@rotation
        translation=incremental@translation+update[:2]
        if np.linalg.norm(update)<1e-7:
            break
    _,errors,keep=correspondence(cur@rotation.T+translation)
    if keep.sum()<max(20,.6*len(cur)):
        return None
    return rotation,translation,float(np.sqrt(np.mean(errors[keep]**2)))


def match_motion(reference_points, current_points, yaw_hint=0.):
    """Return current base pose in reference base coordinates; None is no evidence.

    Multiple yaw seeds reject competing alignments. This cannot resolve all
    repeating scenes; odometry and IMU agreement remain mandatory downstream.
    """
    try:
        ref, cur = np.asarray(reference_points, dtype=float), np.asarray(current_points, dtype=float)
        if not math.isfinite(yaw_hint):
            return None
    except (TypeError, ValueError, OverflowError):
        return None
    for pts in (ref, cur):
        if pts.ndim != 2 or pts.shape[1] != 2 or len(pts) < 24 or not np.isfinite(pts).all():
            return None
        eigenvalues = np.linalg.eigvalsh(np.cov(pts.T))
        if eigenvalues[0] < .0001 or eigenvalues[0] < .02*eigenvalues[1]:
            return None  # A single wall cannot constrain translation along it.
    ref = ref[np.linspace(0,len(ref)-1,min(len(ref),180)).astype(int)]
    cur = cur[np.linspace(0,len(cur)-1,min(len(cur),180)).astype(int)]
    segments=_segments(ref)
    # Beam endpoints move along walls as scan angle changes. Their sampling
    # distance is not robot translation; register against supported wall segments.
    initial=_fit(ref,cur,yaw_hint) if segments is not None else None
    def fit(hint):
        if segments is None:
            return _fit(ref,cur,hint)
        # Point correspondences initialize only; acceptance still requires
        # supported segment coverage, geometric rank and final residual guards.
        # Retain distinct yaw seeds; only their translation initializer is shared.
        seed=(_rotation(hint),initial[1]) if initial is not None else None
        return _fit_segments(segments,cur,hint,seed)
    candidates = [r for offset in (0., -.08, .08) if
                  (r := fit(yaw_hint+offset)) is not None]
    if not candidates:
        return None
    candidates.sort(key=lambda r: r[2])
    rotation, translation, residual = candidates[0]
    yaw = math.atan2(rotation[1,0], rotation[0,0])
    if residual > .008 or np.linalg.norm(translation) > .06 or abs(wrap_pi(yaw-yaw_hint)) > .04:
        return None
    for other_rotation, other_translation, other_residual in candidates[1:]:
        other_yaw = math.atan2(other_rotation[1,0], other_rotation[0,0])
        if other_residual <= residual + .001 and (abs(wrap_pi(yaw-other_yaw)) > .025 or np.linalg.norm(translation-other_translation) > .008):
            return None
    return dict(dx=float(translation[0]), dy=float(translation[1]), yaw=yaw, residual_m=residual)
