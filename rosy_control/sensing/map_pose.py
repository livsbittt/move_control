"""Map display geometry, retaining raw odometry for distance accounting."""
import math


def record_odom(state, x, y, trail_max=3000):
    """Keep raw odometry independent of localization corrections."""
    prev = state.get('pose_prev')
    total = state.get('path_exact_m', 0.0)
    if prev is not None:
        distance = math.hypot(x - prev[0], y - prev[1])
        if distance < 1.0:
            total += distance
    state['pose_prev'] = (x, y)
    state['path_exact_m'] = total
    state['path_m'] = round(total, 2)
    raw = state.setdefault('trail_odom', [])
    if not raw or math.hypot(x - raw[-1][0], y - raw[-1][1]) >= 0.01:
        raw.append((x, y))
        del raw[:-trail_max]


def display_pose(state, pose, transform, *, odom_age=0.0, tf_age=0.0,
                 timeout=1.0, reason=None):
    """Display authoritative map<-base pose only while both inputs are fresh."""
    state['pose_frame'] = 'map'
    if reason is None:
        if not math.isfinite(odom_age) or not -.1 <= odom_age <= timeout:
            reason = 'stale_odom'
        elif pose is None or transform is None:
            reason = 'missing_map_tf'
        # slam_toolbox stamps map->odom ahead by transform_timeout (0.5 s).
        elif not math.isfinite(tf_age) or not -.75 <= tf_age <= timeout:
            reason = 'stale_map_tf'
        elif not all(math.isfinite(v) for v in (*pose, *transform)):
            reason = 'invalid_map_tf'
    state['pose_available'] = reason is None
    state['pose_reason'] = reason or 'ready'
    if reason:
        state['pose'] = None
        state['trail'] = []
        return
    tx, ty, angle = transform
    c, s = math.cos(angle), math.sin(angle)

    def project(px, py):
        return [round(tx + c * px - s * py, 3),
                round(ty + s * px + c * py, 3)]

    state['pose'] = [round(v, 3) for v in pose]
    # Reproject the odometry trail together when SLAM corrects map<-odom.
    # This is an odometry trail, not SLAM's optimized historical trajectory.
    state['trail'] = [project(px, py) for px, py in state.get('trail_odom', [])]


def update_pose(state, x, y, yaw, transform, trail_max=3000):
    """Compatibility geometry helper; ROS display uses authoritative base TF."""
    record_odom(state, x, y, trail_max)
    pose = None
    if transform is not None:
        tx, ty, angle = transform
        c, s = math.cos(angle), math.sin(angle)
        pose = (tx + c*x - s*y, ty + s*x + c*y,
                math.atan2(math.sin(yaw + angle), math.cos(yaw + angle)))
    display_pose(state, pose, transform)
