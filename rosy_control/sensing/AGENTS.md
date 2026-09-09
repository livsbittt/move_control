<!-- Parent: ../AGENTS.md -->
<!-- Generated: 2026-09-06 | Updated: 2026-09-06 -->

# sensing/ (raw data → robot-frame geometry)

## Purpose
Pure logic (no ROS) turning raw sensor data into robot-frame geometry one subject per module: scan geometry, filtering, footprint, camera classification. Consumed by safety, wander, calib, and the web/LCD stack.

## Key Files
| File | Description |
|------|-------------|
| `lidar.py` | C1 scan geometry — the one heading everyone shares. `NOSE_YAW = π + 10°` (scan 0° is the rear); every heading through `robot_yaw()`/`wrap_pi()`. Sector ranges use a 10th percentile (`scan_pctl`); `is_robot_scan()` rejects remote gazebo scans (beam count + range_max + wall-clock stamp) unless an isolated rig called `enable_simulation_scans()`; frontier/straight-route finders feed `/safety/frontier_*`, `/safety/route_*` |
| `filt.py` | Median + 1st-order low-pass filters for jumpy sensors |
| `body.py` | Robot circumradius from URDF (calib param wins if sane); `ignore_m` drops chassis hits; `turn_clear_m` for spin clearance |
| `camera.py` | HSV floor/void/obstacle classification (`classify_frame`) behind `camera_detect_node` |

## For AI Agents

### Working In This Directory
- The nose-yaw trap: any new use of scan angles must go through `robot_yaw()`/`wrap_pi()` — scan angle 0 is the **rear**, nose ≈ 190°.
- Keep modules ROS-free; callbacks and publishers belong to the nodes.
- No-echo conventions: lidar beyond 8 m is the trust horizon, US beyond `US_NOSE_MAX_M` (0.80) is noise — defined once in `control/modes.py`, don't restate them here.

### Testing Requirements
- `python3 -m pytest test/test_lidar*.py test/test_body.py test/test_filt.py -q` (whichever apply).
- Geometry fixes need a boundary test (e.g., nose-yaw wrap at ±π).

### Common Patterns
- One subject per module, `"""Subject: …"""` docstring, hardware-limit comments in English.

## Dependencies

### Internal
- `control/modes.py` owns the shared distance cutoffs; safety/wander/calib consume this geometry.

### External
- `math`, `numpy` only.

<!-- MANUAL: -->
