"""Map display geometry, retaining raw odometry for distance accounting."""
import math


def update_pose(state, x, y, yaw, transform, trail_max=3000):
    """Apply map<-odom (tx, ty, yaw), or hide map overlays without TF."""
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
    state['pose_frame'] = 'map'
    state['pose_available'] = transform is not None
    if transform is None:
        state['pose'] = None
        state['trail'] = []
        return
    tx, ty, angle = transform
    c, s = math.cos(angle), math.sin(angle)

    def project(px, py):
        return [round(tx + c * px - s * py, 3),
                round(ty + s * px + c * py, 3)]

    heading = math.atan2(math.sin(yaw + angle), math.cos(yaw + angle))
    state['pose'] = project(x, y) + [round(heading, 3)]
    # Reproject the odometry trail together when SLAM corrects map<-odom.
    # This is an odometry trail, not SLAM's optimized historical trajectory.
    state['trail'] = [project(px, py) for px, py in raw]
