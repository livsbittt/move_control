# Verified Pinky footprint for slow straight motion

Audited the real robot pinky_description URDF, binary collision STLs and visual COLLADA meshes on 2026-09-08. Applied the live fixed transforms: camera pitch0, screen pitch-25deg, LiDAR x=-0.017m/y=0/yaw180deg. Include full caster swivel sweep, not just its parked pose.

Full modeled union: front+0.0420501m (screen), rear-0.0762932m (caster sweep), sides+/-0.07655m (conservative wheel collision spheres). Actual visual wheels reach+/-0.05505m; use larger collision envelope. Maximum modeled turning radius0.0825704m (base collision mesh). The old FRONT_X0.0295m was an IR sensor location and was not a chassis edge.

Round outward to front0.043/rear0.077/half-width0.077m. For straight corrected commands<=0.014m/s only, compare every observed scan point transformed into base_link against this box expanded by0.010m. Require fresh scan<=0.2s, valid mount TF, sector data and observed endpoints in both swept strips; absent observations never mean free space. Preserve immediate raw stop and10mm clearance hysteresis. Ordinary scan failure and faster/turning commands retain conservative bumper checks. Rotation floor is0.083m with existing10mm extra margin.

For an axis-aligned wall, nominal sensor-origin stop is0.070m front and rear. This number is not applied to diagonal ranges: an obstacle near a wheel can stop motion earlier. Calibration reserves its full chosen2-4cm stroke plus8mm independently of the10mm body stand-off. Camera never authorizes distance safety.

The profile is explicitly enabled for this verified physical configuration. It is not a sensor-derived estimate of unknown chassis size; altered attachments or sensor/screen mounts require profile re-audit. Unit/ROS tests include missing swept-strip observations, side jambs, corrected-speed exclusion, physical-clearance telemetry and retained calibration guards.
